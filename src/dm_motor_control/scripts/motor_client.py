#!/usr/bin/env python3
"""Typed command-line client for a launched driver, with independent lease renewal."""

import argparse
import json
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from dm_motor_interfaces.msg import MotorState, MotorTarget
from dm_motor_control.topic_client import (
    ControlLease, GetCapabilities, GetOperation, MotorOperation, RegisterAccess,
    RunTask, TopicTaskClient, motor_topics,
)


class MotorClient(Node):
    def __init__(self, namespace):
        super().__init__("motor_cli", namespace=namespace, start_parameter_services=False)
        self.state = None
        self.subscription = self.create_subscription(MotorState, "state", self.receive, 1)
        self.topics = motor_topics(self)
        self.command_clients = {name: self.topics.endpoint(name) for name in (
            "get_capabilities", "control_lease", "operation", "registers", "get_operation")}
        self.tasks = TopicTaskClient(self)
        self.token = ""
        self.renew_at = 0.0
        self.renew_future = None

    def receive(self, msg):
        self.state = msg

    def renew(self):
        if self.renew_future is not None and self.renew_future.done():
            response = self.renew_future.result()
            self.renew_future = None
            if not response.success:
                raise RuntimeError(f"Control lease lost: {response.message}")
        if self.token and self.renew_future is None and time.monotonic() >= self.renew_at:
            self.renew_future = self.command_clients["control_lease"].call_async(ControlLease.Request(
                operation=ControlLease.Request.RENEW, token=self.token, duration=1.0))
            self.renew_at = time.monotonic() + 0.2

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.renew()
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return
        raise TimeoutError("Driver response/confirmation timeout; result unknown")

    def call(self, name, request):
        client = self.command_clients[name]
        if not client.wait_for_ready(timeout_sec=3.0):
            raise RuntimeError(f"Topic endpoint unavailable: {name}")
        future = client.call_async(request)
        self.wait(future.done)
        response = future.result()
        if hasattr(response, "success") and not response.success:
            raise RuntimeError(response.message)
        return response

    def capabilities(self):
        attempts = 0
        while True:
            try:
                response = self.call("get_capabilities", GetCapabilities.Request())
            except TimeoutError:
                attempts += 1
                if attempts >= 3:
                    raise
                self.get_logger().warning(
                    f"Read-only capability handshake timed out; retrying discovery ({attempts + 1}/3). "
                    "No enable or write operation has been issued.")
                continue
            if not response.ready:
                raise RuntimeError(f"Driver not ready: {response.message}")
            return response

    def acquire(self):
        self.token = self.call("control_lease", ControlLease.Request(
            operation=ControlLease.Request.ACQUIRE, owner="motor_cli", duration=1.0)).token
        self.renew_at = time.monotonic() + 0.2

    def release(self):
        if not self.token:
            return
        token = self.token
        self.token = ""
        self.renew_future = None
        self.call("control_lease", ControlLease.Request(
            operation=ControlLease.Request.RELEASE, token=token))

    def operation_result(self, operation_id):
        result = None

        def complete():
            nonlocal result
            result = self.call("get_operation", GetOperation.Request(operation_id=operation_id))
            return result.state != GetOperation.Response.PENDING

        self.wait(complete)
        if result.state != GetOperation.Response.CONFIRMED:
            raise RuntimeError(f"Operation {operation_id}: state={result.state}, {result.message}")
        return result

    def operate(self, operation, **kwargs):
        response = self.call("operation", MotorOperation.Request(
            operation=operation, token=self.token, **kwargs))
        if response.operation_id:
            return self.operation_result(response.operation_id)
        return response

    def descriptors(self, names, caps):
        by_name = {item.name: item for item in caps.registers}
        by_address = {item.address: item for item in caps.registers}
        result = []
        for name in names:
            if name in by_name:
                result.append(by_name[name])
            else:
                try:
                    address = int(name, 0)
                except ValueError as error:
                    raise ValueError(f"Unknown register: {name}") from error
                if address not in by_address:
                    raise ValueError(f"Unknown register: {name}")
                result.append(by_address[address])
        return result

    def read(self, descriptors):
        result = []
        for begin in range(0, len(descriptors), 16):
            batch = descriptors[begin:begin + 16]
            response = self.call("registers", RegisterAccess.Request(
                addresses=[item.address for item in batch]))
            for descriptor, operation_id in zip(batch, response.operation_ids):
                operation = self.operation_result(operation_id)
                value = operation.raw_value if descriptor.integer else struct.unpack(
                    "<f", struct.pack("<I", operation.raw_value))[0]
                if not math.isfinite(value):
                    raise RuntimeError(f"{descriptor.name}: device returned nonfinite parameter")
                result.append({
                    "name": descriptor.name, "address": descriptor.address,
                    "integer": descriptor.integer, "value": value,
                    "raw_value": operation.raw_value, "unit": descriptor.unit,
                    "writable": descriptor.writable,
                    "raw_write_allowed": descriptor.raw_write_allowed,
                })
        return result

    def run(self, args):
        self.wait(lambda: self.state is not None and self.state.feedback_fresh and self.state.activity_available)
        self.operate(MotorOperation.Request.ENABLE)
        self.wait(lambda: self.state.phase == MotorState.ENABLED)
        kinds = {
            "move": RunTask.Goal.MOVE, "velocity": RunTask.Goal.VELOCITY,
            "torque": RunTask.Goal.TORQUE, "hold": RunTask.Goal.HOLD,
            "sine": RunTask.Goal.SINE, "ramp": RunTask.Goal.RAMP,
            "trajectory": RunTask.Goal.TRAJECTORY,
        }
        target = MotorTarget(mode=self.state.mode)
        if args.kind == "torque":
            target.position = self.state.position
            target.torque = args.torque
        elif args.kind == "velocity":
            target.velocity = args.velocity
            if target.mode == MotorTarget.MIT:
                target.position = self.state.position
                target.kd = args.kd
        else:
            if args.kind in ("move", "ramp") and args.position is None:
                raise ValueError("move/ramp requires an explicit --position")
            target.position = self.state.position if args.position is None else args.position
            if target.mode == MotorTarget.MIT:
                target.kp = args.kp
                target.kd = args.kd
                target.torque = args.torque
            else:
                target.velocity_limit = args.max_velocity
                if target.mode == MotorTarget.POSITION_FORCE:
                    target.current_ratio = args.current_ratio
        goal = RunTask.Goal(
            token=self.token, kind=kinds[args.kind], target=target,
            duration=args.duration, max_velocity=args.max_velocity,
            max_acceleration=args.max_acceleration, tolerance=args.tolerance,
            settling_time=args.settling_time,
            ramp_time=args.ramp_time if args.kind in ("velocity", "torque") else 0.0,
            amplitude=args.amplitude if args.kind == "sine" else 0.0,
            frequency=args.frequency if args.kind == "sine" else 0.0,
            times=args.times, positions=args.positions)
        if not self.tasks.wait_for_ready(timeout_sec=3.0):
            raise RuntimeError("Task topic endpoints unavailable")
        future = self.tasks.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Task rejected; inspect driver diagnostics for the validation reason")
        result_future = handle.get_result_async()
        self.wait(result_future.done, args.duration + 5.0)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(result.message)
        return {"success": result.success, "message": result.message, "final_position": result.final_position}


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--namespace", default="/motor_1")
    result.add_argument("--allow-hardware-motion", action="store_true")
    result.add_argument("--allow-hardware-maintenance", action="store_true")
    commands = result.add_subparsers(dest="command", required=True)
    commands.add_parser("info")
    commands.add_parser("state")
    read = commands.add_parser("read")
    read.add_argument("registers", nargs="+", help="Names or addresses such as PMAX or 0x15")
    commands.add_parser("export", help="Print a read-only typed parameter snapshot as JSON")
    write = commands.add_parser("write")
    write.add_argument("register")
    write.add_argument("value")
    change = commands.add_parser("mode")
    change.add_argument("mode", type=int, choices=range(1, 5))
    commands.add_parser("stop")
    commands.add_parser("reset")
    commands.add_parser("clear")
    commands.add_parser("save")
    zero = commands.add_parser("zero")
    zero.add_argument("--device", action="store_true")
    zero.add_argument("--value", type=float, default=0.0)
    run = commands.add_parser("run")
    run.add_argument("kind", choices=("move", "velocity", "torque", "hold", "sine", "ramp", "trajectory"))
    run.add_argument("--position", type=float)
    run.add_argument("--velocity", type=float, default=0.0)
    run.add_argument("--torque", type=float, default=0.0)
    run.add_argument("--kp", type=float, default=20.0)
    run.add_argument("--kd", type=float, default=1.0)
    run.add_argument("--current-ratio", type=float, default=0.1)
    run.add_argument("--duration", type=float, default=2.0)
    run.add_argument("--max-velocity", type=float, default=1.0)
    run.add_argument("--max-acceleration", type=float, default=2.0)
    run.add_argument("--tolerance", type=float, default=0.02)
    run.add_argument("--settling-time", type=float, default=0.2)
    run.add_argument("--ramp-time", type=float, default=0.2)
    run.add_argument("--amplitude", type=float, default=0.1)
    run.add_argument("--frequency", type=float, default=0.5)
    run.add_argument("--times", type=float, nargs="*", default=[])
    run.add_argument("--positions", type=float, nargs="*", default=[])
    return result


def main():
    args = parser().parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    client = MotorClient(args.namespace)
    exit_code = 0
    try:
        if args.command == "stop":
            client.operate(MotorOperation.Request.DISABLE)
            client.wait(lambda: client.state is not None and client.state.feedback_fresh and client.state.status == 0)
            output = {"success": True, "message": "Disabled feedback confirmed"}
        else:
            caps = client.capabilities()
            client.wait(lambda: client.state is not None and client.state.feedback_fresh)
            if args.command == "info":
                output = {
                    "name": caps.name, "profile": caps.profile, "backend": caps.backend,
                    "modes": list(caps.modes), "evidence": caps.evidence,
                    "configuration": {name: getattr(caps.configuration, name)
                                      for name in caps.configuration.get_fields_and_field_types()},
                    "simulated": caps.simulated, "registers": [
                        {"name": item.name, "address": item.address, "integer": item.integer,
                         "writable": item.writable, "raw_write_allowed": item.raw_write_allowed,
                         "unit": item.unit} for item in caps.registers]}
            elif args.command == "state":
                state = client.state
                output = {name: getattr(state, name) for name in (
                    "name", "ready", "mode", "phase", "status", "position", "native_position",
                    "absolute_position", "absolute_position_valid", "position_wraps", "velocity",
                    "torque", "mos_temperature", "rotor_temperature", "feedback_age",
                    "feedback_rate", "control_owner", "task_active", "error")}
                output.update({name: getattr(state, name) for name in (
                    "accepted_target_sequence", "applied_target_sequence", "target_accept_latency_ms")})
                output.update({name: getattr(state, name) for name in (
                    "status_text", "host_fault", "device_fault", "host_protection_enabled")})
            elif args.command in ("read", "export"):
                descriptors = caps.registers if args.command == "export" else client.descriptors(args.registers, caps)
                output = {"profile": caps.profile, "simulated": caps.simulated, "registers": client.read(descriptors)}
            else:
                if not caps.simulated and args.command == "run" and not args.allow_hardware_motion:
                    raise ValueError("Hardware motion requires --allow-hardware-motion and site safety approval")
                if not caps.simulated and args.command != "run" and not args.allow_hardware_maintenance:
                    raise ValueError("Hardware modification requires --allow-hardware-maintenance")
                client.acquire()
                if args.command == "run":
                    output = client.run(args)
                elif args.command == "write":
                    descriptor = client.descriptors([args.register], caps)[0]
                    if not descriptor.writable:
                        raise ValueError("Register is read-only")
                    if not descriptor.raw_write_allowed:
                        raise ValueError("Register requires a specialized maintenance path; raw write prohibited")
                    if descriptor.integer:
                        value = int(args.value, 0)
                        if not 0 <= value <= 0xFFFFFFFF:
                            raise ValueError("uint32 value outside range")
                        raw = value
                    else:
                        value = float(args.value)
                        if not math.isfinite(value):
                            raise ValueError("Register value must be finite")
                        raw = struct.unpack("<I", struct.pack("<f", value))[0]
                    queued = client.call("registers", RegisterAccess.Request(
                        write=True, token=client.token, addresses=[descriptor.address], raw_values=[raw]))
                    operation = client.operation_result(queued.operation_ids[0])
                    output = {"success": True, "message": operation.message, "raw_value": operation.raw_value}
                else:
                    operation = {
                        "mode": MotorOperation.Request.SET_MODE,
                        "reset": MotorOperation.Request.RESET_HOST_FAULT,
                        "clear": MotorOperation.Request.CLEAR_MOTOR_FAULT,
                        "save": MotorOperation.Request.SAVE_PARAMETERS,
                        "zero": MotorOperation.Request.SET_DEVICE_ZERO if getattr(args, "device", False)
                        else MotorOperation.Request.SET_SOFTWARE_ZERO,
                    }[args.command]
                    response = client.operate(
                        operation, mode=getattr(args, "mode", 0), value=getattr(args, "value", 0.0))
                    output = {"success": True, "message": response.message}
        print(json.dumps(output, indent=2, allow_nan=False))
    except (RuntimeError, TimeoutError, ValueError, OverflowError, struct.error) as error:
        client.get_logger().error(str(error))
        exit_code = 1
    except KeyboardInterrupt:
        client.get_logger().warning("Interrupted; requesting STOP and releasing control")
        exit_code = 130
    finally:
        if client.token:
            try:
                client.release()
            except (RuntimeError, TimeoutError) as error:
                client.get_logger().error(f"Release unconfirmed: {error}; lease/watchdog remains required")
                exit_code = exit_code or 1
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
