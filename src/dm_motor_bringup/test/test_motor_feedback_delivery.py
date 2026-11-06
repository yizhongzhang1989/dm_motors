"""Isolated event-feedback integration checks; never opens a physical CAN port.

Each fixture starts a new topic-only broker, physical virtual motor, and driver.
Callback age includes broker/driver/DDS/Python scheduling; it is not an internal
driver latency measurement. All logs and acquisition timestamps are preserved.
"""

import json
import os
from pathlib import Path
import secrets
import signal
import statistics
import struct
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from can_bus.msg import Frame, Status
from dm_motor_control.topic_client import motor_topics
from dm_motor_interfaces.msg import MotorFeedback, MotorRequest, MotorSetpoint, MotorState, MotorTarget, SimulationState
from dm_motor_interfaces.srv import SimulationControl


RESULT_ROOT = Path(os.environ.get(
    "DM_MOTOR_FEEDBACK_TEST_ARTIFACTS", str(Path(tempfile.gettempdir()) / "dm-motor-feedback-tests"))).expanduser()


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return {"count": 0}
    percentile = lambda q: ordered[round((len(ordered) - 1) * q)]
    return {"count": len(ordered), "min": ordered[0], "mean": statistics.mean(ordered),
            "p50": percentile(0.5), "p95": percentile(0.95), "p99": percentile(0.99), "max": ordered[-1]}


def ordinary_mit(row):
    return (row["can_id"] == 1 and len(row["data"]) == 8
            and row["data"][:7] != [255] * 7)


def stop_frame(row):
    return row["can_id"] == 1 and row["data"] == [255] * 7 + [253]


class VirtualFeedbackFixture:
    """One lifecycle with assertions before any virtual frame injection."""

    def __init__(self, case, directory, event_driven, control_rate=500.0):
        self.case, self.directory = case, directory
        directory.mkdir(parents=True, exist_ok=False)
        self.event_driven, self.control_rate = event_driven, control_rate
        self.namespace = f"/feedback_delivery_{os.getpid()}_{secrets.token_hex(5)}"
        self.bus_namespace = self.namespace + "/can/main"
        self.state = self.physics = self.bus = None
        self.states, self.feedback, self.wire_rx, self.wire_tx = [], [], [], []
        self.processes, self.logs = {}, {}
        self.sequence, self.next_target = 0, 0.0
        self.streaming = False
        self.closed = False
        self.node = Node("probe", namespace=self.namespace)
        self.topics = motor_topics(self.node, "motor")
        self.publisher = self.node.create_publisher(
            MotorSetpoint, "motor/setpoint", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.injector = self.node.create_publisher(Frame, self.bus_namespace + "/wire_rx", 64)
        self.simulation = self.node.create_client(SimulationControl, "motor/simulation/control")
        self.subscriptions = [
            self.node.create_subscription(MotorState, "motor/state", self.on_state, 1),
            self.node.create_subscription(SimulationState, "motor/simulation/state", self.on_physics, 1),
            self.node.create_subscription(Status, self.bus_namespace + "/status", self.on_bus,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.node.create_subscription(MotorFeedback, "motor/feedback", self.on_feedback, 4096),
            self.node.create_subscription(Frame, self.bus_namespace + "/rx",
                lambda message: self.on_wire(self.wire_rx, message), 4096),
            self.node.create_subscription(Frame, self.bus_namespace + "/tx_observed",
                lambda message: self.on_wire(self.wire_tx, message), 4096),
        ]
        # A package-owned baseline avoids inheriting a user's live hardware profile.
        self.config = str(Path(get_package_share_directory("common")) / "config/runtime.yaml")
        self.manifest = {
            "namespace": self.namespace, "bus_namespace": self.bus_namespace,
            "ROS_DOMAIN_ID": os.environ["ROS_DOMAIN_ID"], "ROS_LOCALHOST_ONLY": os.environ["ROS_LOCALHOST_ONLY"],
            "event_driven_feedback": event_driven, "control_rate": control_rate,
            "latency_definition": "Python callback monotonic time minus preserved broker RX acquisition time; includes DDS and Python scheduling",
            "processes": {},
        }
        self.write_manifest()

    def write_manifest(self):
        (self.directory / "manifest.json").write_text(json.dumps(self.manifest, indent=2) + "\n")

    def start_process(self, label, package, executable, namespace, parameters):
        parameters = dict(parameters, config_file=self.config)
        params = self.directory / (label + ".parameters.yaml")
        params.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
        command = [str(Path(get_package_prefix(package)) / "lib" / package / executable),
                   "--ros-args", "-r", f"__ns:={namespace}", "--params-file", str(params)]
        log = (self.directory / (label + ".log")).open("w")
        self.logs[label] = log
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        self.processes[label] = process
        self.manifest["processes"][label] = {"pid": process.pid, "command": command, "parameters": parameters}
        self.write_manifest()

    def start(self):
        self.start_process("broker", "can_bus", "can_bus_node", self.bus_namespace,
                           {"backend": "virtual", "port": "", "can_fd": False})
        self.start_process("simulator", "dm_motor_control", "sim_motor_node", self.namespace + "/motor",
                           {"can_bus_namespace": self.bus_namespace, "motor_name": "motor", "motor_id": 1,
                            "master_id": 0, "mode": 1, "ambient_temperature": 23.0, "persistence_file": ""})
        parameters = {"can_bus_namespace": self.bus_namespace, "require_virtual_bus": True,
                      "motor_name": "motor", "motor_id": 1, "master_id": 0, "mode": 1,
                      "control_rate": self.control_rate, "state_rate": min(200.0, self.control_rate),
                      "startup_velocity_bandwidth": 0.0, "host_protection_enabled": True,
                      "mos_temperature_stop": 50.0, "rotor_temperature_stop": 50.0,
                      "protection_samples": 1}
        if self.event_driven is not None:
            parameters["event_driven_feedback"] = self.event_driven
        self.start_process("driver", "dm_motor_control", "motor_node", self.namespace + "/motor", parameters)
        self.wait(lambda: self.bus is not None and self.physics is not None and self.state is not None
                  and self.bus.ready and self.physics.virtual_bus_confirmed
                  and self.state.ready and self.state.feedback_fresh, timeout=25)
        self.assert_virtual()
        self.case.assertEqual(self.state.phase, MotorState.DISABLED)
        self.case.assertEqual(self.state.status, 0)
        expected = "enabled" if self.event_driven else "disabled"
        self.case.assertIn("event feedback " + expected, (self.directory / "driver.log").read_text())
        self.case.assertEqual(self.state.control_rate, self.control_rate)

    def assert_virtual(self):
        self.case.assertIsNotNone(self.bus)
        self.case.assertEqual(self.bus.backend, "virtual")
        self.case.assertEqual(self.bus.port, "")
        self.case.assertTrue(self.bus.ready)
        self.case.assertTrue(self.physics.virtual_bus_confirmed)
        self.case.assertEqual(self.physics.can_bus_namespace, self.bus_namespace)

    def on_state(self, message):
        self.state = message
        self.states.append({"callback_time": time.monotonic(), "phase": message.phase, "status": message.status,
                            "host_fault": message.host_fault, "feedback_fresh": message.feedback_fresh,
                            "feedback_count": message.feedback_count, "feedback_receive_time": message.feedback_receive_time,
                            "control_cycle_count": message.control_cycle_count, "error": message.error,
                            "mos_temperature": message.mos_temperature, "target_active": message.target_active})

    def on_physics(self, message):
        self.physics = message

    def on_bus(self, message):
        self.bus = message

    def on_feedback(self, message):
        now = time.monotonic()
        self.feedback.append({"callback_time": now, "receive_time": message.receive_time,
                              "age_ms": (now - message.receive_time) * 1000,
                              "feedback_count": message.feedback_count, "mos_temperature": message.mos_temperature,
                              "target_active": message.target_active})

    def on_wire(self, destination, message):
        destination.append({"callback_time": time.monotonic(), "received_steady": message.received_steady,
                            "can_id": message.can_id, "data": list(message.data), "fd": message.fd,
                            "channel": message.channel})

    def pump(self):
        now = time.monotonic()
        if self.streaming and now >= self.next_target:
            self.sequence += 1
            message = MotorSetpoint(client_id="feedback_delivery", sequence=self.sequence, valid_for=0.15,
                                    target=MotorTarget(mode=1, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0))
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(message)
            self.next_target = now + 0.01

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for label, process in self.processes.items():
                self.case.assertIsNone(process.poll(), f"{label} exited; see {self.directory}")
            self.pump()
            rclpy.spin_once(self.node, timeout_sec=0.001)
            if predicate():
                return
        self.case.fail(f"Timeout in {self.directory}; latest state={self.state}")

    def settle(self, duration):
        deadline = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= deadline, timeout=duration + 1)

    def call(self, endpoint, request):
        client = self.topics.endpoint(endpoint)
        self.wait(client.is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        result = future.result()
        self.case.assertTrue(result.success, result.message)
        return result

    def operation(self, operation):
        return self.call("operation", MotorRequest(operation=operation))

    def enable_stream(self):
        self.wait(lambda: self.state.activity_available)
        self.operation(MotorRequest.ENABLE)
        self.wait(lambda: self.state.phase == MotorState.ENABLED)
        self.streaming = True
        self.wait(lambda: self.state.phase == MotorState.ENABLED and self.state.target_active)

    def pause_feedback(self, paused):
        self.assert_virtual()
        self.wait(lambda: self.simulation.service_is_ready())
        future = self.simulation.call_async(SimulationControl.Request(
            operation=SimulationControl.Request.PAUSE_FEEDBACK, data=paused))
        self.wait(future.done)
        self.case.assertTrue(future.result().success, future.result().message)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.streaming = False
        for label, process in reversed(list(self.processes.items())):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            self.manifest["processes"][label]["returncode"] = process.returncode
        for log in self.logs.values():
            log.close()
        for name in ("states", "feedback", "wire_rx", "wire_tx"):
            (self.directory / (name + ".jsonl")).write_text(
                "".join(json.dumps(row) + "\n" for row in getattr(self, name)))
        self.manifest["residual_processes"] = [p.pid for p in self.processes.values() if p.poll() is None]
        self.write_manifest()
        self.node.destroy_node()


class MotorFeedbackDelivery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Never inherit the real motor's ROS domain or network discovery scope.
        cls.original_environment = {key: os.environ.get(key) for key in
                                    ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_LOG_DIR")}
        os.environ["ROS_DOMAIN_ID"] = os.environ.get("DM_MOTOR_TEST_DOMAIN", str(170 + secrets.randbelow(31)))
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        cls.directory = RESULT_ROOT / f"feedback-delivery-{time.strftime('%Y%m%d-%H%M%S')}-{os.getpid()}-{secrets.token_hex(3)}"
        cls.directory.mkdir(parents=True, exist_ok=False)
        os.environ["ROS_LOG_DIR"] = str(cls.directory / "ros-log")
        print(f"Feedback delivery artifacts: {cls.directory}", flush=True)
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        for key, value in cls.original_environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def fixture(self, name, event_driven, control_rate=500.0):
        fixture = VirtualFeedbackFixture(self, self.directory / name, event_driven, control_rate)
        self.addCleanup(fixture.close)
        fixture.start()
        return fixture

    def test_event_feedback_delivery_and_safety(self):
        reports = {}
        for label, event in (("off", None), ("on", True)):
            fixture = self.fixture(label, event)
            self.wait_disabled_register_read(fixture)
            fixture.enable_stream()
            fixture.settle(0.25)
            started = time.monotonic()
            fixture.settle(2.0)
            finished = time.monotonic()
            self.assertEqual(fixture.state.phase, MotorState.ENABLED, fixture.state.error)
            self.assertTrue(fixture.state.feedback_fresh)
            self.assertFalse(fixture.state.host_fault, fixture.state.error)
            feedback = [row for row in fixture.feedback if started <= row["receive_time"] <= finished]
            commands = [row for row in fixture.wire_tx if ordinary_mit(row)
                        and started <= row["received_steady"] * 1e-9 <= finished]
            self.assertGreater(len(feedback), 700)
            self.assertGreater(len(commands), 700)
            rate = len(commands) / (finished - started)
            self.assertGreater(rate, 400.0)
            self.assertLess(rate, 600.0, "RX notification must not duplicate ordinary 500 Hz MIT traffic")
            states = [row for row in fixture.states if started <= row["callback_time"] <= finished]
            self.assertGreater(len(states), 2)
            cycle_delta = states[-1]["control_cycle_count"] - states[0]["control_cycle_count"]
            cycle_observation = states[-1]["callback_time"] - states[0]["callback_time"]
            cycle_rate = cycle_delta / cycle_observation
            self.assertGreater(cycle_rate, 400.0)
            self.assertLess(cycle_rate, 600.0, "RX notification must not increment the periodic control-cycle counter")
            self.assertTrue(all(row["age_ms"] >= 0 for row in feedback))
            acquisitions = {row["received_steady"] for row in fixture.wire_rx}
            matched = sum(round(row["receive_time"] * 1e9) in acquisitions
                          or round(row["receive_time"] * 1e9) - 1 in acquisitions
                          or round(row["receive_time"] * 1e9) + 1 in acquisitions for row in feedback)
            self.assertGreater(matched / len(feedback), 0.99,
                               "MotorFeedback must preserve broker-observed RX acquisition timestamps")
            report = {"ordinary_mit_hz": rate, "duration_s": finished - started,
                      "control_cycle_hz": cycle_rate, "control_cycle_delta": cycle_delta,
                      "control_cycle_observation_s": cycle_observation,
                      "feedback_callback_age_ms": distribution([row["age_ms"] for row in feedback]),
                      "rx_timestamp_match_fraction": matched / len(feedback)}
            self.confirm_stop(fixture)
            report["stop_new_feedback_confirmed"] = True
            fixture.enable_stream()
            fixture.settle(0.2)
            fixture.streaming = False
            expired_at = time.monotonic()
            fixture.wait(lambda: fixture.state.host_fault and fixture.state.status == 0
                         and fixture.state.feedback_fresh and not fixture.state.target_active)
            self.assertIn("publisher timeout", fixture.state.error)
            self.assertTrue(any(stop_frame(row) and row["received_steady"] * 1e-9 >= expired_at
                                for row in fixture.wire_tx))
            report["publisher_timeout_error"] = fixture.state.error
            fixture.operation(MotorRequest.RESET_HOST_FAULT)
            fixture.wait(lambda: fixture.state.phase == MotorState.DISABLED and not fixture.state.host_fault)
            fixture.enable_stream()
            fixture.settle(0.15)
            fixture.pause_feedback(True)
            paused_at = time.monotonic()
            fixture.wait(lambda: fixture.state.host_fault and not fixture.state.feedback_fresh)
            self.assertIn("feedback", fixture.state.error.lower())
            report["feedback_timeout_error"] = fixture.state.error
            fixture.pause_feedback(False)
            fixture.wait(lambda: fixture.state.feedback_fresh and fixture.state.status == 0)
            fixture.settle(0.2)
            self.assertTrue(fixture.state.host_fault)
            self.assertEqual(fixture.state.phase, MotorState.FAULT)
            self.assertFalse(fixture.state.target_active)
            self.assertTrue(any(stop_frame(row) and row["received_steady"] * 1e-9 >= paused_at
                                for row in fixture.wire_tx))
            report["feedback_recovery_did_not_enable"] = True
            reports[label] = report
            (self.directory / "delivery-summary.json").write_text(json.dumps(reports, indent=2) + "\n")
            fixture.close()
        # Timing is recorded, not asserted to improve under arbitrary CI host load.
        self.assertLess(reports["on"]["ordinary_mit_hz"] / reports["off"]["ordinary_mit_hz"], 1.15)

    def wait_disabled_register_read(self, fixture):
        fixture.wait(lambda: fixture.state.maintenance_available)
        reply = fixture.call("registers", MotorRequest(addresses=[32]))
        self.assertEqual(len(reply.operation_ids), 1)
        result = None
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            result = fixture.call("get_operation", MotorRequest(operation_id=reply.operation_ids[0]))
            if result.state != result.PENDING:
                break
            fixture.settle(0.01)
        self.assertEqual(result.state, result.CONFIRMED, result.message)
        value = struct.unpack("<f", struct.pack("<I", result.raw_value))[0]
        self.assertGreater(value, 0.0)
        self.assertEqual(fixture.state.phase, MotorState.DISABLED)
        fixture.manifest["disabled_RID32_read"] = {"raw_value": result.raw_value, "float_value": value}
        fixture.write_manifest()

    def confirm_stop(self, fixture):
        fixture.streaming = False
        before = fixture.state.feedback_count
        requested = time.monotonic()
        fixture.operation(MotorRequest.DISABLE)
        fixture.wait(lambda: fixture.state.phase == MotorState.DISABLED and fixture.state.status == 0
                     and fixture.state.feedback_fresh and fixture.state.feedback_count > before
                     and fixture.state.feedback_receive_time > requested)
        fixture.wait(lambda: any(stop_frame(row) and row["received_steady"] * 1e-9 >= requested
                                for row in fixture.wire_tx))
        sent = min(row["received_steady"] for row in fixture.wire_tx
                   if stop_frame(row) and row["received_steady"] * 1e-9 >= requested)
        fixture.wait(lambda: any(row["can_id"] == 0 and len(row["data"]) == 8
                     and row["data"][0] == 1 and row["received_steady"] >= sent
                     and row["data"][1:3] != [0, 0x33] for row in fixture.wire_rx))

    def test_transient_temperature_fault_latches_despite_following_normal_frame(self):
        fixture = self.fixture("transient-temperature", True, control_rate=20.0)
        fixture.enable_stream()
        fixture.settle(0.15)
        fixture.pause_feedback(True)
        before_tx = len(fixture.wire_tx)
        fixture.wait(lambda: any(ordinary_mit(row) for row in fixture.wire_tx[before_tx:]))
        previous_tx = next(row for row in reversed(fixture.wire_tx) if ordinary_mit(row))
        fixture.assert_virtual()
        self.assertGreater(fixture.injector.get_subscription_count(), 0)
        injected_at = time.monotonic()
        # At zero p/v/t the valid packed feedback is 0x8000 / 0x800 / 0x800.
        # The motor remains status=1 in both samples; only temperature changes.
        for temperature in (51, 23):
            message = Frame(can_id=0, channel=0, length=8,
                            data=[0x11, 0x80, 0x00, 0x80, 0x08, 0x00, temperature, 23])
            message.header.stamp = fixture.node.get_clock().now().to_msg()
            fixture.injector.publish(message)
        fixture.wait(lambda: fixture.state.host_fault)
        self.assertIn("MOS temperature 51", fixture.state.error)
        fixture.wait(lambda: any(stop_frame(row) and row["received_steady"] * 1e-9 > injected_at
                                for row in fixture.wire_tx))
        fixture.wait(lambda: len([row for row in fixture.wire_rx if row["received_steady"] * 1e-9 >= injected_at
                                 and row["data"][:1] == [0x11]]) >= 2)
        pair = [row for row in fixture.wire_rx if row["received_steady"] * 1e-9 >= injected_at
                and row["data"][:1] == [0x11]][:2]
        self.assertEqual([row["data"][6] for row in pair], [51, 23])
        gap = (pair[1]["received_steady"] - pair[0]["received_steady"]) * 1e-9
        since_tx = (pair[1]["received_steady"] - previous_tx["received_steady"]) * 1e-9
        self.assertLess(gap, 0.0125)
        self.assertLess(since_tx, 0.05, "Injected pair must fit after one MIT and before its next control period")
        def feedback_for(frame):
            return next((row for row in fixture.feedback
                         if abs(round(row["receive_time"] * 1e9) - frame["received_steady"]) <= 1), None)

        fixture.wait(lambda: all(feedback_for(frame) is not None for frame in pair))
        feedback_pair = [feedback_for(frame) for frame in pair]
        self.assertEqual([row["mos_temperature"] for row in feedback_pair], [51, 23])
        self.assertTrue(all(not row["target_active"] for row in feedback_pair),
                        "Both the violating sample and following normal sample must report inactive output")
        fixture.wait(lambda: any(stop_frame(row) and row["received_steady"] >= pair[0]["received_steady"]
                                for row in fixture.wire_tx))
        first_stop = min(row["received_steady"] for row in fixture.wire_tx
                         if stop_frame(row) and row["received_steady"] >= pair[0]["received_steady"])
        hot_to_stop_ms = (first_stop - pair[0]["received_steady"]) * 1e-6
        fixture.pause_feedback(False)
        fixture.wait(lambda: fixture.state.feedback_fresh and fixture.state.status == 0)
        fixture.settle(0.25)
        self.assertEqual(fixture.state.phase, MotorState.FAULT)
        self.assertTrue(fixture.state.host_fault)
        self.assertFalse(fixture.state.target_active)
        self.assertIn("MOS temperature 51", fixture.state.error)
        result = {"control_period_s": 0.05, "injected_temperatures": [51, 23],
                  "injection_pair_gap_s": gap, "normal_frame_after_previous_MIT_s": since_tx,
                  "hot_RX_to_first_STOP_ms": hot_to_stop_ms,
                  "injected_feedback_target_active": [row["target_active"] for row in feedback_pair],
                  "host_fault": fixture.state.error, "normal_feedback_did_not_clear_fault": True}
        (self.directory / "transient-temperature-summary.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    unittest.main(verbosity=2)
