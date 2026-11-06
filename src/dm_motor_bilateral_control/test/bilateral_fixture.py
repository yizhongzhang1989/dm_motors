"""Scripted fake axes for controller wiring and lifecycle regressions.

No motor driver, physical plant or CAN transport is started.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from dm_motor_interfaces.msg import (
    BilateralState, MotorFeedback, MotorReply,
    MotorRequest, MotorSetpoint, MotorState,
)
from std_srvs.srv import Trigger


def configure_test_environment(directory):
    domain = int(os.environ.get('DM_MOTOR_TEST_DOMAIN',
                                os.environ.get('ROS_DOMAIN_ID', str(150 + os.getpid() % 40))))
    assert domain not in (0, 143) and 1 <= domain <= 232, 'Use an isolated test ROS domain'
    default_profile = Path(__file__).resolve().parents[2] / 'can_bus/config/fastdds_loopback_udp.xml'
    profile = Path(os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', str(default_profile))).resolve()
    document = ET.parse(profile).getroot()
    ns = {'dds': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    transports = document.findall('.//dds:transport_descriptor', ns)
    assert transports and all(
        item.findtext('dds:type', namespaces=ns) == 'UDPv4' and
        [address.text for address in item.findall('dds:interfaceWhiteList/dds:address', ns)] == ['127.0.0.1']
        for item in transports), 'Test DDS transports must use loopback UDP only'
    participants = document.findall('.//dds:participant', ns)
    assert participants and any(item.get('is_default_profile') == 'true' for item in participants)
    assert all(item.findtext('dds:rtps/dds:useBuiltinTransports', namespaces=ns) == 'false'
               for item in participants), 'Disable DDS built-in transports for this test'
    for key in ('FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_DISCOVERY_SERVER',
                'RMW_FASTRTPS_USE_QOS_FROM_XML', 'FASTDDS_ENVIRONMENT_FILE',
                'FASTDDS_BUILTIN_TRANSPORTS', 'SKIP_DEFAULT_XML'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID=str(domain),
                      ROS_LOCALHOST_ONLY=os.environ.get('ROS_LOCALHOST_ONLY', '1'),
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOG_DIR=str(directory / 'ros-logs'))


class FakeAxes:
    def __init__(self, node, directory, binary, overrides=None, feedback_transform=None):
        self.node = node
        self.directory = directory
        directory.mkdir()
        self.namespace = f'/bilateral_fixture_{os.getpid()}_{directory.name}'
        self.axes = [self.namespace + '/a', self.namespace + '/b']
        self.enabled = [False, False]
        self.leased = [False, False]
        self.source = ['', '']
        self.count = 0
        self.feedback = True
        self.fault = False
        self.states = []
        self.targets = []
        self.full_targets = []
        self.feedback_transform = feedback_transform
        self.requests = []
        self.next_feedback = self.next_state = 0.
        self.reliable = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
        self.state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.best_effort = QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publishers = []
        self.subscriptions = []
        self.state_pub, self.feedback_pub, self.reply_pub = [], [], []
        for i, axis in enumerate(self.axes):
            for kind, suffix, group, qos in (
                (MotorState, '/state', self.state_pub, self.state_qos),
                (MotorFeedback, '/feedback', self.feedback_pub, self.reliable),
                (MotorReply, '/control_result', self.reply_pub, self.reliable),
            ):
                publisher = node.create_publisher(kind, axis + suffix, qos)
                group.append(publisher)
                self.publishers.append(publisher)
            self.subscriptions.append(node.create_subscription(
                MotorRequest, axis + '/control', lambda m, i=i: self.request(i, m), self.reliable))
            self.subscriptions.append(node.create_subscription(
                MotorSetpoint, axis + '/setpoint', lambda m, i=i: self.target(i, m), self.best_effort))
        self.subscriptions.append(node.create_subscription(
            BilateralState, self.namespace + '/controller/state',
            lambda m: self.states.append((time.monotonic(), m)), self.reliable))
        self.start_client = node.create_client(Trigger, self.namespace + '/controller/start')
        self.stop_client = node.create_client(Trigger, self.namespace + '/controller/stop')
        self.parameters = node.create_client(SetParameters, self.namespace + '/controller/set_parameters')
        fit = dict(model='coulomb_viscous', parameters=dict(inertia=.003, bias=0., viscous=0., coulomb=.1),
                   units=dict(inertia='kg m2', bias='Nm', viscous='Nm s/rad', coulomb='Nm'),
                   domain=dict(velocity_min=-1., velocity_max=1.))
        calibration = {'version': 1, 'motors': {axis: {'simulation': {'calibrations': {'mechanical': {
            'schema': 3, 'qualified': True, 'source': 'declared fake-message fixture',
            'identity': dict(motor_id=i+1, mode=1, direction=1., software_offset=0., pmax=12.5, vmax=50., tmax=10.),
            'fit': fit}}}} for i, axis in enumerate(self.axes)}}
        shared = directory / 'parameters.json'
        shared.write_text(json.dumps(calibration))
        config = Path(__file__).resolve().parents[3] / 'config' / 'dm_motors.yaml'
        command = [binary, '--ros-args', '-r', '__node:=controller', '-r', '__ns:=' + self.namespace]
        parameters = dict(config_file=str(config), system_parameters_file=str(shared),
                         motor_a=self.axes[0], motor_b=self.axes[1], required_velocity_bandwidth='0.0',
                         control_mode='shared_reference', rate='500.0', stiffness='50.0',
                         coupling_damping='0.4975', local_damping='0.005', max_torque='0.0',
                         friction_compensation_enabled='true', friction_compensation_fraction='0.6')
        parameters.update(overrides or {})
        for key, value in parameters.items():
            command += ['-p', key + ':=' + str(value)]
        (directory / 'command.json').write_text(json.dumps(command, indent=2))
        self.log = (directory / 'controller.log').open('w')
        self.child = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)

    def request(self, axis, request):
        self.requests.append((request.kind, request.operation))
        reply = MotorReply()
        reply.header.stamp = self.node.get_clock().now().to_msg()
        reply.client_id, reply.request_id, reply.kind = request.client_id, request.request_id, request.kind
        reply.success = True
        if request.kind == MotorRequest.LEASE:
            self.leased[axis] = request.operation != MotorRequest.RELEASE
            reply.token = f'fixture-token-{axis}' if self.leased[axis] else ''
            reply.remaining = 2. if self.leased[axis] else 0.
        elif request.kind == MotorRequest.CAPABILITIES:
            reply.ready, reply.profile = True, 'fixture'
            cfg = reply.configuration
            cfg.control_rate, cfg.state_rate = 500., 50.
            cfg.command_timeout, cfg.feedback_timeout, cfg.watchdog_timeout = .1, .08, .12
        elif request.kind == MotorRequest.OPERATION and request.operation in (MotorRequest.ENABLE, MotorRequest.DISABLE):
            self.enabled[axis] = request.operation == MotorRequest.ENABLE
            if not self.enabled[axis]:
                self.source[axis] = ''
        else:
            raise AssertionError(f'Unexpected fake-driver request: {request}')
        self.reply_pub[axis].publish(reply)

    def target(self, axis, message):
        assert not message.hold_until_lease_expiry, 'Bilateral targets are non-held streams'
        if self.enabled[axis]:
            self.source[axis] = message.client_id
            self.targets.append((time.monotonic(), axis, message.target.kp, message.target.kd))
            self.full_targets.append((time.monotonic(), axis, message.target))

    def publish(self):
        now = time.monotonic()
        if now >= self.next_feedback:
            self.next_feedback = now + .002
            if self.feedback:
                self.count += 1
                for i, publisher in enumerate(self.feedback_pub):
                    message = MotorFeedback()
                    message.motor_id, message.feedback_count = i+1, self.count
                    message.receive_time, message.absolute_position_valid = now-.001, True
                    message.mos_temperature = message.rotor_temperature = 25
                    if self.feedback_transform is not None:
                        message = self.feedback_transform(i, message)
                        if message is None:
                            continue
                    publisher.publish(message)
        if now >= self.next_state:
            self.next_state = now + .020  # Legal 50Hz state, independent of 500Hz compact.
            for i, publisher in enumerate(self.state_pub):
                message = MotorState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.motor_id, message.master_id, message.mode = i+1, 0x11+i, 1
                message.ready = message.simulated = message.feedback_fresh = message.absolute_position_valid = True
                message.phase = MotorState.ENABLED if self.enabled[i] else MotorState.DISABLED
                message.status = int(self.enabled[i])
                message.feedback_count = self.count
                # Combined state age routinely exceeds 20ms but remains <80ms.
                message.feedback_age, message.feedback_receive_time = .015, now-.015
                message.pmax = message.position_limit = 12.5
                message.vmax = message.velocity_limit = 50.
                message.tmax = message.torque_limit = 10.
                message.kp_limit, message.kd_limit = 500., 1.
                message.direction, message.position_min, message.position_max = 1., -12.5, 12.5
                message.control_rate = message.actual_control_rate = 500.
                message.mos_temperature = message.rotor_temperature = 25
                message.active_source = self.source[i]
                message.target_active = bool(self.source[i]) and self.enabled[i]
                # Production active_session is empty for a non-held stream,
                # despite the controller owning/renewing a nonempty local token.
                message.lease_remaining = 0.
                message.activity_available = True
                message.host_fault = self.fault
                publisher.publish(message)

    def pump(self, shutdown=False):
        if not shutdown:
            assert self.child.poll() is None, (self.directory / 'controller.log').read_text()
        self.publish()
        rclpy.spin_once(self.node, timeout_sec=.0005)

    def wait(self, predicate, timeout=4.):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, self.describe()
            self.pump()

    def hold(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.pump()

    def describe(self):
        return (f'published compact pairs={self.count}, controller states={len(self.states)}, '
                f'matches={[p.get_subscription_count() for p in self.state_pub]}, '
                + (str(self.states[-1][1]) if self.states else (self.directory / 'controller.log').read_text()))

    @property
    def state(self):
        return self.states[-1][1]

    def call(self, client, request):
        future = client.call_async(request)
        self.wait(future.done)
        return future.result()

    def start(self):
        response = self.call(self.start_client, Trigger.Request())
        assert response.success, response.message
        self.wait(lambda: self.state.phase == BilateralState.ACTIVE)

    def close(self):
        if self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 4
            while self.child.poll() is None and time.monotonic() < deadline:
                self.pump(shutdown=True)
            if self.child.poll() is None:
                self.child.kill()
            self.child.wait(timeout=3)
        self.log.close()
        for item in self.subscriptions:
            self.node.destroy_subscription(item)
        for item in self.publishers:
            self.node.destroy_publisher(item)
        for item in (self.start_client, self.stop_client, self.parameters):
            self.node.destroy_client(item)
