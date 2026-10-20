"""Asynchronous motor topic transport; no ROS services or actions are used."""

import copy
import threading
import time
from types import SimpleNamespace
import uuid

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from dm_motor_interfaces.msg import (
    MotorRequest, MotorReply, MotorSetpoint, MotorTask, MotorTaskFeedback,
    MotorTaskResult, MotorTaskEvent,
)


class ControlLease:
    Request = MotorRequest
    Response = MotorReply


MotorOperation = RegisterAccess = GetOperation = GetCapabilities = ControlLease


class SetTarget:
    Request = MotorSetpoint
    Response = MotorReply


class RunTask:
    Goal = MotorTask
    Feedback = MotorTaskFeedback
    Result = MotorTaskResult


KINDS = {
    "control_lease": MotorRequest.LEASE, "operation": MotorRequest.OPERATION,
    "registers": MotorRequest.REGISTERS, "get_capabilities": MotorRequest.CAPABILITIES,
    "get_operation": MotorRequest.OPERATION_RESULT, "control_rate": MotorRequest.CONTROL_RATE,
    "enable": MotorRequest.LEGACY_ENABLE, "reset": MotorRequest.LEGACY_RESET,
    "set_target": MotorRequest.TARGET,
    "target": MotorRequest.TARGET,
}


class TopicEndpoint:
    def __init__(self, transport, kind):
        self.transport = transport
        self.kind = kind

    def is_ready(self):
        publisher = self.transport.targets if self.kind == MotorRequest.TARGET else self.transport.commands
        return publisher.get_subscription_count() > 0 and self.transport.node.count_publishers(
            self.transport.results.topic_name) > 0

    def wait_for_ready(self, timeout_sec=3.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.is_ready():
                return True
            time.sleep(0.01)
        return False

    def call_async(self, request):
        return self.transport.send(self.kind, request)[1]

    # Kept for source migration of existing clients; these methods only inspect topic discovery.
    service_is_ready = is_ready
    wait_for_service = wait_for_ready


class TopicTaskHandle:
    def __init__(self, transport, task, task_id, feedback_callback):
        self.transport = transport
        self.task = task
        self.task_id = task_id
        self.accepted = False
        self.feedback_callback = feedback_callback
        self.result = Future()
        duration = task.times[-1] + task.settling_time if task.kind == MotorTask.TRAJECTORY and task.times else task.duration
        self.deadline = time.monotonic() + duration + 3.0

    def get_result_async(self):
        if not self.accepted:
            raise RuntimeError("Cannot wait for a task that was not accepted.")
        return self.result

    def cancel_goal_async(self):
        return self.transport.send(MotorRequest.TASK_CANCEL, MotorRequest(
            token=self.task.token, task_id=self.task_id))[1]


class TopicTaskClient:
    def __init__(self, node, _task_type=RunTask, name="run_task"):
        prefix = name.rpartition("/")[0]
        self.transport = motor_topics(node, prefix)

    def is_ready(self):
        return self.transport.endpoint("operation").is_ready()

    def wait_for_ready(self, timeout_sec=3.0):
        return self.transport.endpoint("operation").wait_for_ready(timeout_sec)

    server_is_ready = is_ready
    wait_for_server = wait_for_ready

    def send_goal_async(self, goal, feedback_callback=None):
        request = MotorRequest(token=goal.token, task=goal)
        task_id = self.transport.next_id()
        handle = TopicTaskHandle(self.transport, copy.deepcopy(goal), task_id, feedback_callback)
        with self.transport.lock:
            self.transport.tasks[task_id] = handle
        accepted = Future()
        _, future = self.transport.send(MotorRequest.TASK_START, request, request_id=task_id)

        def completed(response):
            try:
                handle.accepted = response.result().success
                if not handle.accepted:
                    with self.transport.lock:
                        self.transport.tasks.pop(task_id, None)
                accepted.set_result(handle)
            except Exception as error:
                with self.transport.lock:
                    self.transport.tasks.pop(task_id, None)
                accepted.set_exception(error)

        future.add_done_callback(completed)
        return accepted


class MotorTopics:
    def __init__(self, node, prefix=""):
        self.node = node
        self.client_id = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.sequence = 0
        self.pending = {}
        self.tasks = {}
        self.target_listener = None
        path = lambda leaf: f"{prefix}/{leaf}" if prefix else leaf
        qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.commands = node.create_publisher(MotorRequest, path("control"), qos)
        self.targets = node.create_publisher(
            MotorSetpoint, path("target"), QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.results = node.create_subscription(MotorReply, path("control_result"), self.receive, qos)
        self.task_events = node.create_subscription(
            MotorTaskEvent, path("task_events"), self.receive_task, qos)
        self.timer = node.create_timer(0.05, self.expire)

    def endpoint(self, name):
        return TopicEndpoint(self, KINDS[name])

    def next_id(self):
        with self.lock:
            self.sequence += 1
            return self.sequence

    def send(self, kind, request, request_id=None):
        request_id = self.next_id() if request_id is None else request_id
        future = Future()
        if kind == MotorRequest.TARGET:
            message = copy.deepcopy(request)
            message.client_id = self.client_id
            message.request_id = request_id
            message.hold_until_lease_expiry = bool(message.token) or message.hold_until_lease_expiry
            publisher = self.targets
        else:
            message = MotorRequest()
            for field in request.get_fields_and_field_types():
                if field in message.get_fields_and_field_types():
                    setattr(message, field, copy.deepcopy(getattr(request, field)))
            message.kind = kind
            message.client_id = self.client_id
            message.request_id = request_id
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.valid_for = 0.5
            publisher = self.commands
        with self.lock:
            self.pending[request_id] = (future, time.monotonic() + 0.75, kind)
        try:
            publisher.publish(message)
        except Exception as error:
            with self.lock:
                self.pending.pop(request_id, None)
            future.set_exception(error)
        return request_id, future

    def publish_target(self, message):
        if not self.targets.get_subscription_count():
            raise RuntimeError("Motor target topic subscriber unavailable.")
        message.hold_until_lease_expiry = bool(message.token) or message.hold_until_lease_expiry
        message.client_id = self.client_id
        message.request_id = self.next_id()
        self.targets.publish(message)

    def receive(self, message):
        if message.client_id != self.client_id:
            return
        with self.lock:
            pending = self.pending.get(message.request_id)
            if pending and pending[2] != message.kind:
                self.node.get_logger().error("Motor reply kind does not match its pending request.")
                return
            pending = self.pending.pop(message.request_id, None)
        if message.kind == MotorRequest.TARGET and self.target_listener:
            self.target_listener(message)
        if pending and not pending[0].done():
            pending[0].set_result(message)

    def receive_task(self, message):
        if message.client_id != self.client_id:
            return
        if message.state not in (MotorTaskEvent.RUNNING, MotorTaskEvent.SUCCEEDED,
                                 MotorTaskEvent.FAILED, MotorTaskEvent.CANCELED):
            self.node.get_logger().error(f"Invalid motor task event state: {message.state}")
            return
        with self.lock:
            handle = self.tasks.get(message.task_id)
            if message.state != MotorTaskEvent.RUNNING:
                self.tasks.pop(message.task_id, None)
        if handle is None:
            return
        if message.state == MotorTaskEvent.RUNNING:
            if handle.feedback_callback:
                handle.feedback_callback(SimpleNamespace(feedback=message.feedback))
        elif not handle.result.done():
            status = {MotorTaskEvent.SUCCEEDED: 4, MotorTaskEvent.FAILED: 6,
                      MotorTaskEvent.CANCELED: 5}[message.state]
            handle.result.set_result(SimpleNamespace(result=message.result, status=status))

    def expire(self):
        now = time.monotonic()
        expired = []
        expired_tasks = []
        with self.lock:
            for request_id, (future, deadline, _) in list(self.pending.items()):
                if now >= deadline:
                    expired.append(future)
                    self.pending.pop(request_id)
            for task_id, handle in list(self.tasks.items()):
                if now >= handle.deadline:
                    self.tasks.pop(task_id)
                    expired_tasks.append(handle.result)
        for future in expired:
            if not future.done():
                future.set_exception(TimeoutError("Motor topic result timeout; outcome unknown."))
        for future in expired_tasks:
            if not future.done():
                future.set_exception(TimeoutError("Motor task event timeout; outcome unknown, STOP required."))


def motor_topics(node, prefix=""):
    clients = getattr(node, "_motor_topic_transports", None)
    if clients is None:
        clients = {}
        node._motor_topic_transports = clients
    if prefix not in clients:
        clients[prefix] = MotorTopics(node, prefix)
    return clients[prefix]


def create_motor_client(node, _message_model, name):
    prefix, _, endpoint = name.rpartition("/")
    return motor_topics(node, prefix).endpoint(endpoint)
