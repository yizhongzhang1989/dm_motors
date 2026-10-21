"""Optional virtual-device interactions for the shared hardware/simulation dashboard."""

import secrets
import threading
import time
from urllib.parse import urlsplit

from common import ConfigManager
from dm_motor_control.dashboard import (
    ApiError, Dashboard, DashboardServer, DashboardHandler, finite_number, plain_message,
    main as run_dashboard,
)
from dm_motor_interfaces.msg import ExternalTorque, SimulationState
from dm_motor_interfaces.srv import SimulationControl


class SimulationDashboard(Dashboard):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        config = ConfigManager(self.get_parameter("config_file").value)
        self.simulation_enabled = self.declare_parameter(
            "simulation", config.get("packages.dm_motor_control.nodes.dashboard.simulation", bool)).value
        self.shaft = {}
        for name, low, high in (("stiffness", 0, 100), ("damping", 0, 20)):
            key = "shaft_" + name
            value = self.declare_parameter(key, config.get(
                "packages.dm_motor_control.nodes.dashboard." + key, float)).value
            finite_number(value, key)
            if value > high or value < low or (name != "damping" and value == 0):
                raise ValueError(f"{key} is outside its supported range")
            self.shaft[name] = value
        self.simulation = None
        self.simulation_received = 0.0
        self.force_sequence = 0
        self.force_order = {}
        self.external_source = "dashboard_" + secrets.token_hex(12)
        self.force_publisher = self.simulation_client = None
        if self.simulation_enabled:
            self.force_publisher = self.create_publisher(ExternalTorque, "simulation/external_torque", 1)
            self.simulation_subscription = self.create_subscription(
                SimulationState, "simulation/state", self.receive_simulation, 1)
            self.simulation_client = self.create_client(SimulationControl, "simulation/control")

    def snapshot(self, session):
        return {**super().snapshot(session), "simulation": self.simulation_snapshot()}

    def receive_simulation(self, message):
        with self.lock:
            if message.motor_id != self.motor_id:
                self.get_logger().error("Simulation motor identity does not match the dashboard")
                return
            self.simulation = {key: plain_message(getattr(message, key))
                               for key in message.get_fields_and_field_types() if key != "header"}
            self.simulation_received = time.monotonic()

    def simulation_snapshot(self):
        with self.lock:
            fresh = (self.simulation is not None and time.monotonic() - self.simulation_received < 0.5)
            return {"enabled": self.simulation_enabled, "connected": fresh, "state": self.simulation,
                    "shaft": self.shaft}

    def require_virtual(self):
        snapshot = self.simulation_snapshot()
        if (not self.simulation_enabled or not snapshot["connected"] or not snapshot["state"]["virtual_bus_confirmed"]
                or not self.connected() or not self.state["simulated"]):
            raise ApiError(409, "Simulation controls require fresh virtual-device and simulated motor feedback")

    def external_torque(self, body, source=None, sequence=None):
        if set(body) != {"torque", "duration"}:
            raise ApiError(400, "Expected torque (Nm) and duration (seconds)")
        torque = finite_number(body["torque"], "torque")
        duration = finite_number(body["duration"], "duration")
        self.require_virtual()
        if not 0 < duration <= 5:
            raise ApiError(400, "Duration must be in (0,5]")
        return self.publish_force(ExternalTorque(torque=torque, valid_for=duration), source, sequence)

    def shaft_drag(self, body, source=None, sequence=None):
        if set(body) != {"position", "simulation_time"}:
            raise ApiError(400, "Expected native hand position and the observed simulation_time")
        position = finite_number(body["position"], "position")
        observed = finite_number(body["simulation_time"], "simulation_time")
        self.require_virtual()
        snapshot = self.simulation_snapshot()
        if not -0.05 <= snapshot["state"]["simulation_time"] - observed <= 0.7:
            raise ApiError(409, "Shaft request uses expired simulation feedback")
        return self.publish_force(ExternalTorque(
            spring=True, position=position, stiffness=self.shaft["stiffness"],
            damping=self.shaft["damping"], valid_for=0.3), source, sequence)

    def publish_force(self, message, source, sequence):
        self.require_virtual()
        if source is not None or sequence is not None:
            if (not source or len(source) > 128 or sequence is None or not sequence.isdecimal()
                    or len(sequence) > 20 or not 0 < int(sequence) < 2**64):
                raise ApiError(400, "External ordering requires a source and positive uint64 sequence")
            sequence = int(sequence)
        if not self.force_publisher.get_subscription_count():
            raise ApiError(503, "Simulation external torque subscriber unavailable")
        with self.lock:
            if self.closing:
                raise ApiError(503, "Dashboard is shutting down; no new external force is accepted")
            now = time.monotonic()
            self.force_order = {key: value for key, value in self.force_order.items() if now - value[1] < 5}
            if source is not None:
                if source in self.force_order and sequence <= self.force_order[source][0]:
                    raise ApiError(409, "Stale external torque request; a newer force/clear was accepted")
                if source not in self.force_order and len(self.force_order) >= 64:
                    raise ApiError(429, "External torque source capacity reached")
                self.force_order[source] = (sequence, now)
            self.force_sequence += 1
            message.source = self.external_source
            message.sequence = self.force_sequence
            message.header.stamp = self.get_clock().now().to_msg()
            self.force_publisher.publish(message)
        return {"message": "External torque published; confirm actual force in simulation feedback",
                "sequence": message.sequence}

    def simulation_control(self, body):
        if set(body) != {"operation", "value"}:
            raise ApiError(400, "Expected simulation operation and value")
        names = {"fault": SimulationControl.Request.INJECT_FAULT,
                 "pause_feedback": SimulationControl.Request.PAUSE_FEEDBACK,
                 "power_cycle": SimulationControl.Request.POWER_CYCLE,
                 "voltage": SimulationControl.Request.SET_VOLTAGE,
                 "clear_force": SimulationControl.Request.CLEAR_EXTERNAL_TORQUE}
        if not isinstance(body["operation"], str) or body["operation"] not in names:
            raise ApiError(400, "Unknown simulation operation")
        request = SimulationControl.Request(operation=names[body["operation"]])
        value = body["value"]
        if body["operation"] == "fault":
            if type(value) is not int or value not in (0, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14):
                raise ApiError(400, "fault value must be 0, 3..5 or 8..14")
            request.fault_status = value
        elif body["operation"] == "pause_feedback":
            if type(value) is not bool:
                raise ApiError(400, "pause_feedback value must be boolean")
            request.data = value
        else:
            request.value = finite_number(value, "value")
        return self.call_simulation(request)

    def shaft_anchor(self, body):
        if set(body) != {"active", "bearing", "revision", "simulation_time"}:
            raise ApiError(400, "Expected anchor active, native bearing, revision and simulation_time")
        if type(body["active"]) is not bool or type(body["revision"]) is not int or not 0 <= body["revision"] < 2**53:
            raise ApiError(400, "Anchor active must be boolean and revision must be a nonnegative safe integer")
        bearing = finite_number(body["bearing"], "bearing")
        observed = finite_number(body["simulation_time"], "simulation_time")
        self.require_virtual()
        if not -0.05 <= self.simulation_snapshot()["state"]["simulation_time"] - observed <= 0.7:
            raise ApiError(409, "Anchor request uses expired simulation feedback")
        request = SimulationControl.Request(
            operation=SimulationControl.Request.SET_ANCHOR if body["active"] else SimulationControl.Request.CLEAR_ANCHOR,
            value=bearing, check_anchor_revision=True, anchor_revision=body["revision"])
        return self.call_simulation(request)

    def call_simulation(self, request):
        self.require_virtual()
        if self.closing:
            raise ApiError(503, "Dashboard is shutting down; no new simulation operation is accepted")
        if not self.simulation_client.service_is_ready():
            raise ApiError(503, "Simulation service unavailable")
        future = self.simulation_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(1.5):
            self.simulation_client.remove_pending_request(future)
            raise ApiError(504, "Simulation operation timed out; result unknown")
        if future.exception():
            raise ApiError(503, str(future.exception()))
        result = future.result()
        if not result.success:
            raise ApiError(409, result.message)
        return {"message": result.message}

    def begin_shutdown(self):
        pending = super().begin_shutdown()
        snapshot = self.simulation_snapshot()
        if (snapshot["connected"] and snapshot["state"]["external_remaining"] > 0
                and snapshot["state"]["external_source"] == self.external_source):
            self.force_sequence += 1
            force = ExternalTorque(source=self.external_source, sequence=self.force_sequence,
                                   torque=0.0, valid_for=0.1)
            force.header.stamp = self.get_clock().now().to_msg()
            self.force_publisher.publish(force)
            return True
        return pending


class SimulationServer(DashboardServer):
    def __init__(self, node, host, port):
        super().__init__(node, host, port)
        self.RequestHandlerClass = SimulationHandler


class SimulationHandler(DashboardHandler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/simulation":
            self.json_response(200, self.server.node.simulation_snapshot())
        else:
            super().do_GET()

    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            if self.server.node.simulation_enabled and path not in ("/api/stop", "/api/release"):
                self.server.node.require_virtual()
            if path in ("/api/external-torque", "/api/shaft-drag", "/api/shaft-anchor", "/api/simulation-control"):
                self.check_origin()
                body = self.read_body()
                if path in ("/api/external-torque", "/api/shaft-drag"):
                    action = self.server.node.shaft_drag if path == "/api/shaft-drag" else self.server.node.external_torque
                    result = action(
                        body, self.headers.get("X-External-Source"), self.headers.get("X-External-Sequence"))
                elif path == "/api/shaft-anchor":
                    result = self.server.node.shaft_anchor(body)
                else:
                    result = self.server.node.simulation_control(body)
                self.json_response(200, result)
            else:
                super().do_POST()
        except ApiError as error:
            self.server.node.get_logger().warning(f"Simulation HTTP request rejected ({error.status}): {error}")
            self.json_response(error.status, {"error": str(error)})


def main():
    run_dashboard(SimulationDashboard, SimulationServer)
