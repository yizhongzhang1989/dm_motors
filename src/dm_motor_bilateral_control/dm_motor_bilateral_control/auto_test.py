"""Return both physical shafts to coordinate zero, perturb bilateral control, and report."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_srvs.srv import Trigger
from dm_motor_interfaces.msg import BilateralState, MotorState, MotorRequest, MotorTask, MotorTarget
from dm_motor_interfaces.srv import BilateralControl, BilateralDisturbance, BilateralAngleOffset
from dm_motor_control.topic_client import motor_topics, TopicTaskClient
from dm_motor_bilateral_control.motion_analysis import analyze_angle_motion
from dm_motor_bilateral_control.stationarity import StationarityWindow
from dm_motor_bilateral_control.velocity_bandwidth import bandwidth_evidence, required_bandwidth


EDITED = ('motion_limits_enabled', 'max_travel', 'max_feedback_torque',
          'warning_temperature', 'max_temperature')
CONFIGURATION = EDITED + ('control_mode', 'required_velocity_bandwidth', 'direction', 'stiffness', 'coupling_damping', 'local_damping',
    'max_torque', 'friction_compensation_enabled', 'friction_compensation_fraction',
    'friction_torque_limit', 'rate', 'operator_timeout', 'target_validity')
CONTROL_MODES = ('native', 'shared_reference', 'differential_torque')


def read_configuration(values):
    if len(values) != len(CONFIGURATION):
        raise RuntimeError('Incomplete controller configuration response')
    types = {name: Parameter.Type.DOUBLE.value for name in CONFIGURATION}
    types.update(control_mode=Parameter.Type.STRING.value, direction=Parameter.Type.INTEGER.value,
                 motion_limits_enabled=Parameter.Type.BOOL.value,
                 friction_compensation_enabled=Parameter.Type.BOOL.value)
    result = {}
    for name, value in zip(CONFIGURATION, values):
        if value.type != types[name]:
            raise RuntimeError('Unexpected controller parameter type: ' + name)
        decoded = Parameter.from_parameter_msg(ParameterMessage(name=name, value=value)).value
        if value.type == Parameter.Type.DOUBLE.value and not math.isfinite(decoded):
            raise RuntimeError('Nonfinite controller parameter: ' + name)
        result[name] = decoded
    return result


def native_gain_request(configuration):
    mode = configuration['control_mode']
    if mode not in CONTROL_MODES:
        raise RuntimeError('Controller control_mode must be native, shared_reference or differential_torque')
    multiplier = 1 if mode == 'native' else 2
    gains = [configuration[name] for name in ('stiffness', 'coupling_damping', 'local_damping')]
    if any(type(value) not in (int, float) or not math.isfinite(value) or value < 0 for value in gains):
        raise RuntimeError('Controller gains must be finite and nonnegative')
    if mode == 'differential_torque' and gains[0] > 500:
        raise RuntimeError('Differential host stiffness exceeds the supported 500 Nm/rad limit')
    kp = 0 if mode == 'differential_torque' else multiplier * gains[0]
    kd = multiplier * gains[1] + gains[2]
    if kp > 500 or kd > 1:
        raise RuntimeError('Mapped controller MIT gains exceed protocol limits')
    return {'kp': kp, 'kd': kd}


def namespace(value):
    return '/' + value.strip('/')


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def fail(report, reason):
    report['result'] = 'failed'
    report.setdefault('error', reason)


def merge_guard_result(report, status, returncode=None):
    if status is not None:
        report['guard'] = status
        if status.get('failed'):
            fail(report, 'Guard: ' + str(status.get('reason', 'unknown failure')))
    elif report.get('result') == 'passed':
        fail(report, 'Independent guard final record unavailable')
    if returncode is not None and returncode != 0:
        fail(report, 'Independent guard exited with code ' + str(returncode))


class AutomaticTest(Node):
    def __init__(self, args):
        super().__init__('bilateral_auto_test_' + uuid.uuid4().hex[:8])
        self.args = args
        self.axes = [args.motor_a, args.motor_b]
        self.states, self.received = {}, {}
        self.stationarity = {axis: StationarityWindow() for axis in self.axes}
        self.follow, self.follow_received = None, 0.0
        self.observers = [self.create_subscription(MotorState, axis + '/state',
            lambda msg, a=axis: self.receive(a, msg), 1) for axis in self.axes]
        self.observers.append(self.create_subscription(BilateralState, args.controller + '/state', self.receive_follow, 1))
        self.topics = {a: motor_topics(self, a) for a in self.axes}
        self.tasks = {a: TopicTaskClient(self, name=a + '/run_task') for a in self.axes}
        self.stop_publishers = {a: self.create_publisher(MotorRequest, a + '/control', 64) for a in self.axes}
        self.operator = self.create_client(BilateralControl, args.controller + '/operator_control')
        self.disturbance = self.create_client(BilateralDisturbance, args.controller + '/disturbance')
        self.angle_service = self.create_client(BilateralAngleOffset, args.controller + '/angle_offset')
        self.stop_service = self.create_client(Trigger, args.controller + '/stop')
        self.get_params = self.create_client(GetParameters, args.controller + '/get_parameters')
        self.set_params = self.create_client(SetParametersAtomically, args.controller + '/set_parameters_atomically')
        self.session = 'autotest_' + uuid.uuid4().hex
        self.tokens, self.pending_renewals, self.renewed = {}, {}, {}
        self.heartbeat_futures, self.last_heartbeat = [], 0.0
        self.guard = None
        self.guard_status = None
        self.guard_buffer = b''
        self.guard_seen = None
        self.guard_watchdog_started = None
        self.guard_ping = 0.0
        self.armed = self.coupling = self.interrupted = False
        self.stop_sequence = 0
        self.old_settings = None
        self.settings_changed = False
        self.report = dict(result='failed', stop_confirmed=False, phases=[],
            test_kind=args.test_kind,
            created_utc=datetime.now(timezone.utc).isoformat(), environment='hardware' if args.hardware else 'simulation',
            coordinate_policy='Move to existing joint coordinate 0; never reset device/software zero or Flash',
            motor_a=args.motor_a, motor_b=args.motor_b, controller=args.controller)

    def receive(self, axis, msg):
        self.states[axis], self.received[axis] = msg, time.monotonic()
        self.stationarity[axis].add(msg.feedback_receive_time, msg.position, msg.velocity, msg.feedback_count)

    def stationary(self):
        return all(window.summary(time.monotonic())['quiet'] for window in self.stationarity.values())

    def receive_follow(self, msg):
        self.follow, self.follow_received = msg, time.monotonic()

    def check_velocity_bandwidth(self):
        required = self.report['required_velocity_bandwidth']
        evidence = bandwidth_evidence(required, self.follow)
        self.report['velocity_bandwidth'] = evidence
        if not evidence['matches_requirement']:
            raise RuntimeError(evidence['error'])

    def fresh(self):
        now = time.monotonic()
        return all(a in self.states and self.states[a].feedback_fresh and
            math.isfinite(self.states[a].feedback_age) and
            0 <= self.states[a].feedback_age + now - self.received[a] < .05 for a in self.axes)

    def disabled(self, after=None, *, stopped_at=None):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds if stopped_at is not None else 0
        return self.fresh() and all(self.states[a].phase == MotorState.DISABLED and
            self.states[a].status == 0 and not self.states[a].target_active and
            (after is None or self.states[a].feedback_count > after.get(a, 0)) and
            (stopped_at is None or (
                self.received[a] > stopped_at['monotonic'] and
                self.states[a].feedback_receive_time > stopped_at['monotonic'] and
                0 <= now - self.states[a].feedback_receive_time < .05 and
                stamp_ns(self.states[a]) > max(stopped_at['ros_ns'], stopped_at['state_stamps'].get(a, 0)) and
                0 <= ros_now - stamp_ns(self.states[a]) < 50_000_000)) for a in self.axes)

    def controller_stopped(self, after=None, *, allow_fault=False):
        return (self.follow is not None and self.follow.stop_confirmed and
            self.follow.phase in ((BilateralState.IDLE, BilateralState.FAULT) if allow_fault else (BilateralState.IDLE,)) and
            0 <= time.monotonic() - self.follow_received < .05 and
            0 <= self.get_clock().now().nanoseconds - stamp_ns(self.follow) < 50_000_000 and
            (after is None or (self.follow_received > after['monotonic'] and stamp_ns(self.follow) > after['ros_ns'])))

    def send_guard(self, op, **values):
        if self.guard is not None and self.guard.poll() is None and not self.guard.stdin.closed:
            if op == 'arm' and self.guard_watchdog_started is None:
                # Once ARM may reach the child, missing status must remain fatal
                # even if its acknowledgement never reaches this process.
                self.guard_watchdog_started = time.monotonic()
            os.write(self.guard.stdin.fileno(), (json.dumps(dict(op=op, **values)) + '\n').encode())

    def _spin_once(self, *, timeout_sec=.002, stopping=False):
        # Default scheduling is unchanged. Specialized collectors can own an
        # executor while the same STOP path still services their callbacks.
        rclpy.spin_once(self, timeout_sec=timeout_sec)

    def pump(self, stopping=False):
        self._spin_once(timeout_sec=.002, stopping=stopping)
        now = time.monotonic()
        if not stopping and self.coupling and self.follow is not None and self.follow.phase == BilateralState.ACTIVE:
            self.check_velocity_bandwidth()
        if self.guard is not None:
            try:
                block = os.read(self.guard.stdout.fileno(), 65536)
                self.guard_buffer += block
            except BlockingIOError:
                pass
            while b'\n' in self.guard_buffer:
                line, self.guard_buffer = self.guard_buffer.split(b'\n', 1)
                item = json.loads(line)
                if item.get('kind') == 'status':
                    self.guard_status, self.guard_seen = item, now
                    if self.guard_watchdog_started is None and (item.get('ready') or item.get('armed')):
                        # DDS discovery can pause pre-ready status publication.
                        # start_guard's absolute readiness deadline covers that
                        # phase; after readiness this watchdog never disarms.
                        self.guard_watchdog_started = now
            if now - self.guard_ping >= .05:
                self.send_guard('ping')
                self.guard_ping = now
            if not stopping:
                if self.guard.poll() is not None:
                    raise RuntimeError('Independent guard exited')
                if self.guard_watchdog_started is not None and now - max(
                        self.guard_watchdog_started,
                        self.guard_seen if self.guard_seen is not None else self.guard_watchdog_started) > .25:
                    raise RuntimeError('Independent guard heartbeat lost')
                if self.guard_status and self.guard_status.get('failed'):
                    raise RuntimeError('Guard: ' + str(self.guard_status.get('reason')))
        if stopping:
            return
        if self.interrupted:
            raise RuntimeError('Operator interrupted automatic test')
        for axis, token in list(self.tokens.items()):
            pending = self.pending_renewals.get(axis)
            if pending is not None:
                if not pending.done():
                    continue
                if not pending.result().success:
                    raise RuntimeError('Homing lease renewal rejected: ' + axis)
                del self.pending_renewals[axis]
            if now - self.renewed.get(axis, 0) >= .1:
                self.pending_renewals[axis] = self.topics[axis].endpoint('control_lease').call_async(
                    MotorRequest(operation=MotorRequest.RENEW, token=token, duration=.5))
                self.renewed[axis] = now
        if self.coupling:
            if now - self.last_heartbeat >= .05:
                req = BilateralControl.Request(operation=BilateralControl.Request.HEARTBEAT, session=self.session, valid_for=.2)
                req.header.stamp = self.get_clock().now().to_msg()
                self.heartbeat_futures.append((now, self.operator.call_async(req)))
                self.last_heartbeat = now
            pending = []
            for sent, future in self.heartbeat_futures:
                if future.done():
                    result = future.result()
                    if result is None or not result.success:
                        raise RuntimeError('Bilateral operator heartbeat rejected')
                elif now - sent > .25:
                    raise RuntimeError('Bilateral operator heartbeat timed out')
                else:
                    pending.append((sent, future))
            self.heartbeat_futures = pending

    def wait(self, predicate, timeout, *, stopping=False):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.pump(stopping)
            if time.monotonic() > deadline:
                raise TimeoutError('Timed out waiting for ' + self.report.get('phase', 'readiness'))

    def future(self, future, timeout=1.5, *, stopping=False):
        self.wait(future.done, timeout, stopping=stopping)
        result = future.result()
        if result is None:
            raise RuntimeError('Empty ROS response')
        return result

    def phase(self, name):
        now = time.monotonic()
        if self.report['phases']:
            self.report['phases'][-1]['finished_monotonic'] = now
        self.report['phases'].append(dict(name=name, started_monotonic=now))
        self.report['phase'] = name
        if name != 'finished':
            self.send_guard('phase', name=name)
        write_json(self.args.output / 'progress.json', self.report)

    def disable(self):
        for axis in self.axes:
            self.stop_sequence += 1
            request = MotorRequest(client_id=self.session, request_id=self.stop_sequence,
                kind=MotorRequest.OPERATION, operation=MotorRequest.DISABLE)
            request.header.stamp = self.get_clock().now().to_msg()
            self.stop_publishers[axis].publish(request)

    def stop_both(self, reason, failure=False):
        self.coupling = False
        self.heartbeat_futures.clear()
        self.tokens.clear()
        self.pending_renewals.clear()
        before = {a: self.states[a].feedback_count if a in self.states else 0 for a in self.axes}
        stopped_at = dict(monotonic=time.monotonic(), ros_ns=self.get_clock().now().nanoseconds,
            state_stamps={a: stamp_ns(self.states[a]) for a in self.axes if a in self.states})
        self.report['stop_confirmed'] = False
        evidence = self.report['stop_confirmation'] = dict(request=stopped_at, feedback_counts_before=before,
            controller_response=None)
        self.disable()
        try:
            self.send_guard('stop', reason=reason, failure=failure)
        except (OSError, ValueError):
            pass  # The independent direct DISABLE path must survive broken IPC.
        deadline, last_send = time.monotonic() + 4, 0.0
        service_pending = None
        response_after = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_send >= .04:
                self.disable()
                if response_after is None and service_pending is None and self.stop_service.service_is_ready():
                    try:
                        service_pending = self.stop_service.call_async(Trigger.Request())
                    except Exception as error:
                        evidence['service_error'] = str(error)
                last_send = now
            try:
                self.pump(stopping=True)
            except Exception:
                # Motor DISABLE does not depend on the guard pipe, service or former task lease.
                try:
                    self._spin_once(timeout_sec=.002, stopping=True)
                except Exception:
                    pass
            if service_pending is not None and service_pending.done():
                try:
                    reply = service_pending.result()
                    if reply is not None and reply.success:
                        # Poll the future: no callback signature or MessageInfo assumptions.
                        # Wait for a publication after the response, not a queued old IDLE.
                        response_after = dict(monotonic=time.monotonic(), ros_ns=self.get_clock().now().nanoseconds)
                        evidence['controller_response'] = response_after
                    else:
                        evidence['service_error'] = reply.message if reply is not None else 'Empty STOP response'
                except Exception as error:
                    evidence['service_error'] = str(error)
                service_pending = None
            if response_after is not None and self.controller_stopped(response_after, allow_fault=True) and self.follow.phase == BilateralState.FAULT:
                # Preserve explicit STOP retries after a fault stop completes. Do not
                # restart an acknowledged STOP while awaiting its ordinary IDLE.
                response_after = None
                evidence['controller_response'] = None
            evidence['motors_confirmed'] = self.disabled(before, stopped_at=stopped_at)
            evidence['controller_confirmed'] = response_after is not None and self.controller_stopped(response_after)
            evidence['guard_confirmed'] = bool(self.guard_status and self.guard_status.get('stop_confirmed'))
            if evidence['motors_confirmed'] and evidence['controller_confirmed'] and evidence['guard_confirmed']:
                evidence['confirmed_monotonic'] = time.monotonic()
                self.report['stop_confirmed'] = True
                return
        self.report['stop_confirmed'] = False

    def capabilities(self, axis):
        # Graph discovery does not prove that the new reply subscription has
        # matched at the driver yet. Establish the round trip with this read-only
        # request while stopped; never retry enables, motion or offset requests.
        record = self.report.setdefault('capability_queries', {}).setdefault(axis, {'attempts': 0, 'timeouts': 0})
        for attempt in range(3):
            if not self.disabled() or not self.follow.stop_confirmed:
                raise RuntimeError('Read-only preflight requires both axes and controller stopped')
            record['attempts'] += 1
            try:
                reply = self.future(self.topics[axis].endpoint('get_capabilities').call_async(MotorRequest()))
                if not self.disabled() or not self.follow.stop_confirmed:
                    raise RuntimeError('Motor or controller activity changed during read-only preflight')
                return reply
            except TimeoutError:
                record['timeouts'] += 1
                if attempt == 2:
                    raise TimeoutError('No capability reply after three read-only discovery probes: ' + axis)

    def preflight(self):
        def checked_mode():
            if (self.follow is None or not 0 <= time.monotonic() - self.follow_received < .1 or
                    not self.disabled() or not self.follow.stop_confirmed or self.follow.phase != BilateralState.IDLE):
                raise RuntimeError('Fresh stopped controller state required during configuration preflight')
            mode = getattr(self.follow, 'control_mode', None)
            if mode not in CONTROL_MODES:
                raise RuntimeError('Controller state control_mode must be native, shared_reference or differential_torque')
            return mode

        self.phase('preflight')
        self.wait(lambda: self.fresh() and self.follow is not None and
            time.monotonic() - self.follow_received < .1 and self.get_params.service_is_ready() and
            self.set_params.service_is_ready() and self.operator.service_is_ready() and
            (self.angle_service if self.args.test_kind == 'angle_offset' else self.disturbance).service_is_ready()
            and all(t.is_ready() for t in self.tasks.values()), 8)
        if not self.disabled() or not self.follow.stop_confirmed or self.follow.phase != BilateralState.IDLE:
            raise RuntimeError('Both motors and bilateral controller must already be stopped')
        if self.follow.motor_a != self.args.motor_a or self.follow.motor_b != self.args.motor_b:
            raise RuntimeError('Controller motor namespaces differ from requested axes')
        self.report['control_mode'] = checked_mode()
        self.wait(lambda: self.disabled() and self.stationary(), 1.5)
        self.report['initial_stationarity'] = {
            a: self.stationarity[a].summary(time.monotonic()) for a in self.axes}
        for axis, state in self.states.items():
            if state.simulated == self.args.hardware or not state.ready or state.mode != 1 or state.error or state.host_fault or state.device_fault:
                raise RuntimeError('Environment, MIT readiness or fault preflight failed: ' + axis)
            if max(state.mos_temperature, state.rotor_temperature) >= 45:
                raise RuntimeError('Both motors must be stationary and below 45 C')
            if state.kp_limit < 40 or state.kd_limit < 1:
                raise RuntimeError('Homing requires MIT Kp40/Kd1 within driver capabilities')
            for position in (state.position, 0.0):
                if abs(position - state.software_offset) > state.pmax:
                    raise RuntimeError('Homing path exceeds native MIT position range')
        response = self.future(self.get_params.call_async(GetParameters.Request(names=list(CONFIGURATION))))
        config = read_configuration(response.values)
        self.report['configuration'] = config
        if checked_mode() != config['control_mode']:
            raise RuntimeError('Controller state and parameter control_mode differ')
        native_gains = native_gain_request(config)
        self.report['native_gains'] = native_gains
        required = config['required_velocity_bandwidth']
        required_bandwidth(required)
        required_bandwidth(getattr(self.follow, 'required_velocity_bandwidth', None))
        if getattr(self.follow, 'required_velocity_bandwidth', None) != required:
            raise RuntimeError('Controller state and parameter required_velocity_bandwidth differ')
        self.report['required_velocity_bandwidth'] = required
        self.old_settings = {key: config[key] for key in EDITED}
        if config['friction_compensation_enabled'] and config['friction_compensation_fraction'] > 0:
            try:
                self.wait(lambda: self.follow.compensation_ready_a and self.follow.compensation_ready_b, 1.5)
            except TimeoutError:
                raise RuntimeError('Matched friction models unavailable; refused before homing')
        if self.args.test_kind == 'pulse' and config['max_torque'] > 0 and abs(self.args.torque) + .01 >= config['max_torque']:
            raise RuntimeError('Perturbation does not fit configured application torque budget')
        for i, axis in enumerate(self.axes):
            reply = self.capabilities(axis)
            if not reply.success:
                raise RuntimeError('Driver capabilities query failed: ' + axis)
            c = reply.configuration
            if c.control_rate < config['rate'] or c.watchdog_timeout <= 0 or c.watchdog_timeout > .2:
                raise RuntimeError('Driver rate/watchdog incompatible with bilateral test')
            if not c.position_min <= 0 <= c.position_max or not c.position_min <= self.states[axis].position <= c.position_max:
                raise RuntimeError('Zero position/path outside configured joint bounds')
            if self.args.test_kind == 'angle_offset':
                extent = abs(self.args.offset_radians) + .08
                for position in (-extent, extent):
                    if not c.position_min <= position <= c.position_max or abs(position - self.states[axis].software_offset) > self.states[axis].pmax:
                        raise RuntimeError('Angle-offset test corridor exceeds joint/native limits: ' + axis)
            if c.host_protection_enabled and c.estimated_torque_limit > 0 and (
                    config['max_torque'] == 0 or config['max_torque'] > c.estimated_torque_limit):
                raise RuntimeError('Bilateral budget incompatible with driver estimated torque limit')
        if checked_mode() != config['control_mode']:
            raise RuntimeError('Controller control_mode changed during configuration preflight')
        required_bandwidth(getattr(self.follow, 'required_velocity_bandwidth', None))
        if getattr(self.follow, 'required_velocity_bandwidth', None) != required:
            raise RuntimeError('Controller required_velocity_bandwidth changed during configuration preflight')
        self.report['driver_gain_limits'] = {}
        for axis in self.axes:
            state = self.states[axis]
            limits = {'kp': state.kp_limit, 'kd': state.kd_limit}
            if any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0
                   for value in limits.values()):
                raise RuntimeError('Fresh finite driver gain limits required: ' + axis)
            self.report['driver_gain_limits'][axis] = limits
            # A positive application budget must not hide an unsupported request.
            if any(native_gains[key] > limits[key] for key in native_gains):
                raise RuntimeError('Mapped controller MIT gains exceed driver capabilities: ' + axis)
        origins = {a: self.states[a].position for a in self.axes}
        self.report['position_resolution'] = {a: 2 * self.states[a].pmax / 65535 for a in self.axes}
        distance = max(abs(p) for p in origins.values())
        # Driver MOVE uses a quintic reference; duration includes its settling interval.
        # Match the driver's conservative 5.774 coefficient and leave room for
        # encoder quantization or tiny motion between preflight and task start.
        travel_time = 1.02 * max(1.0, 1.875 * distance / .25, math.sqrt(5.774 * distance / .5)) + .02
        duration = travel_time + .5
        if duration > 30:
            raise RuntimeError('Return-to-zero path exceeds the 30 second automatic-test allowance')
        self.report['homing'] = dict(origins=origins, target=0.0, tolerance=.005,
            reference_max_velocity=.25, reference_max_acceleration=.5, duration=duration,
            kp=40.0, kd=1.0, torque_policy='No total torque clamp in MOVE; independent measured 0.7 Nm stop')
        return duration

    def start_guard(self, duration):
        extra = 5 + self.args.transition_time if self.args.test_kind == 'angle_offset' else 0
        travel = abs(self.args.offset_radians) + .08 if self.args.test_kind == 'angle_offset' else .08
        config = dict(motor_a=self.args.motor_a, motor_b=self.args.motor_b, controller=self.args.controller,
            required_velocity_bandwidth=self.report['required_velocity_bandwidth'],
            test_kind=self.args.test_kind, offset_radians=self.args.offset_radians,
            transition_time=self.args.transition_time, baseline_seconds=5., observe_seconds=self.args.observe_seconds,
            output_directory=str(self.args.output), environment=self.report['environment'],
            initial_positions=self.report['homing']['origins'], total_timeout=duration + self.args.observe_seconds + extra + 8,
            active_timeout=self.args.observe_seconds + extra + 1.5, max_speed=.8, max_torque=.7,
            max_temperature=45, feedback_timeout=.05, max_test_travel=travel, home_margin=.08, parent_timeout=.25)
        write_json(self.args.output / 'guard.json', config)
        self.guard_log = (self.args.output / 'guard.log').open('w')
        self.guard = subprocess.Popen([sys.executable, '-m', 'dm_motor_bilateral_control.auto_test_guard',
            '--config', str(self.args.output / 'guard.json')], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.guard_log, start_new_session=True)
        os.set_blocking(self.guard.stdout.fileno(), False)
        os.set_blocking(self.guard.stdin.fileno(), False)
        self.wait(lambda: self.guard_status and self.guard_status.get('ready'), 5)
        self.send_guard('arm')
        self.armed = True  # Parent STOP remains available if the arm acknowledgement is lost.
        self.wait(lambda: self.guard_status and self.guard_status.get('armed'), 1)

    def home(self, duration):
        self.phase('homing')
        for axis in self.axes:
            response = self.future(self.topics[axis].endpoint('control_lease').call_async(
                MotorRequest(operation=MotorRequest.ACQUIRE, owner=self.session, duration=.5)))
            if not response.success or not response.token:
                raise RuntimeError('Homing lease denied: ' + axis)
            self.tokens[axis], self.renewed[axis] = response.token, time.monotonic()
        for axis in self.axes:
            response = self.future(self.topics[axis].endpoint('operation').call_async(
                MotorRequest(operation=MotorRequest.ENABLE, token=self.tokens[axis])))
            if not response.success:
                raise RuntimeError('Homing enable rejected: ' + response.message)
        self.wait(lambda: self.fresh() and all(self.states[a].phase == MotorState.ENABLED and
            self.states[a].status == 1 for a in self.axes), 2)
        handles = []
        for axis in self.axes:
            goal = MotorTask(token=self.tokens[axis], kind=MotorTask.MOVE,
                target=MotorTarget(mode=1, position=0., velocity=0., torque=0., kp=40., kd=1.),
                duration=duration, max_velocity=.25, max_acceleration=.5, tolerance=.005, settling_time=.5)
            handle = self.future(self.tasks[axis].send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Homing task rejected: ' + axis)
            handles.append((axis, handle.get_result_async()))
        for axis, future in handles:
            result = self.future(future, duration + 3).result
            if not result.success:
                raise RuntimeError('Homing failed: ' + axis + ': ' + result.message)
        self.phase('handoff')
        self.disable()
        self.wait(lambda: self.disabled(), 2)
        for axis, token in list(self.tokens.items()):
            pending = self.pending_renewals.pop(axis, None)
            if pending is not None:
                self.future(pending)
            self.tokens.pop(axis)  # Do not renew a lease while releasing it.
            response = self.future(self.topics[axis].endpoint('control_lease').call_async(
                MotorRequest(operation=MotorRequest.RELEASE, token=token)))
            if not response.success:
                raise RuntimeError('Homing lease release rejected')
        settled, deadline = None, time.monotonic() + 2
        while settled is None or time.monotonic() - settled < .25:
            self.pump()
            if not self.disabled() or not self.stationary() or any(abs(self.states[a].position) > .005 for a in self.axes):
                settled = None
            else:
                settled = settled or time.monotonic()
            if time.monotonic() > deadline:
                raise RuntimeError('Zero not confirmed after disabling homing outputs')
        self.report['homing']['final_positions'] = {a: self.states[a].position for a in self.axes}
        self.report['homing']['stationarity'] = {
            a: self.stationarity[a].summary(time.monotonic()) for a in self.axes}
        travel = abs(self.args.offset_radians) + .08 if self.args.test_kind == 'angle_offset' else .08
        settings = dict(motion_limits_enabled=True, max_travel=travel, max_feedback_torque=.7,
            warning_temperature=40., max_temperature=45.)
        # A lost response can follow a successful server-side update.
        self.settings_changed = True
        response = self.future(self.set_params.call_async(SetParametersAtomically.Request(
            parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in settings.items()])))
        if not response.result.successful:
            raise RuntimeError('Test motion limits rejected: ' + response.result.reason)

    def run_coupling(self):
        self.phase('coupling')
        request = BilateralControl.Request(operation=BilateralControl.Request.START, session=self.session, valid_for=.2)
        request.header.stamp = self.get_clock().now().to_msg()
        reply = self.future(self.operator.call_async(request))
        if not reply.success:
            raise RuntimeError('Bilateral START rejected: ' + reply.message)
        self.coupling = True
        self.wait(lambda: self.follow.phase == BilateralState.ACTIVE and self.follow.operator_guarded and
            self.follow.quality == BilateralState.NORMAL and self.follow.recovery_scale == 1, 3)
        self.check_velocity_bandwidth()
        if self.args.test_kind == 'angle_offset':
            self.run_angle_offset()
            self.finish_observation()
            return
        warmup = time.monotonic() + .3
        self.wait(lambda: time.monotonic() >= warmup, .6)
        self.phase('disturbance')
        request = BilateralDisturbance.Request(session=self.session, axis=0 if self.args.axis == 'a' else 1,
            torque=self.args.torque, duration=self.args.pulse_duration, valid_for=.2)
        request.header.stamp = self.get_clock().now().to_msg()
        reply = self.future(self.disturbance.call_async(request))
        if not reply.success:
            raise RuntimeError('Disturbance rejected: ' + reply.message)
        self.report['disturbance'] = dict(axis=self.args.axis, torque=self.args.torque,
            duration=self.args.pulse_duration, id=reply.disturbance_id, observed_active=False)
        self.wait(lambda: self.follow.disturbance_id == reply.disturbance_id and self.follow.disturbance_active, .5)
        self.report['disturbance']['observed_active'] = True
        self.wait(lambda: self.follow.disturbance_id == reply.disturbance_id and not self.follow.disturbance_active and
            self.follow.disturbance_torque_a == 0 and self.follow.disturbance_torque_b == 0, self.args.pulse_duration + .5)
        self.phase('observation')
        observation_start = time.monotonic()
        try:
            while time.monotonic() - observation_start < self.args.observe_seconds:
                self.pump()
                if self.follow.phase != BilateralState.ACTIVE or self.follow.quality != BilateralState.NORMAL:
                    raise RuntimeError('Bilateral control degraded/stopped during observation: ' + self.follow.message)
                if self.follow.disturbance_active or self.follow.disturbance_torque_a != 0 or self.follow.disturbance_torque_b != 0:
                    raise RuntimeError('Unexpected repeated disturbance during observation')
        finally:
            self.report['observation'] = dict(duration=time.monotonic() - observation_start)
        self.finish_observation()

    def require_normal(self):
        if self.follow is None or time.monotonic() - self.follow_received > .05 or not self.fresh():
            raise RuntimeError('Fresh bilateral feedback required throughout observation')
        if self.follow.phase != BilateralState.ACTIVE or self.follow.quality != BilateralState.NORMAL or not self.follow.operator_guarded:
            raise RuntimeError('Bilateral control degraded/stopped during observation: ' + self.follow.message)
        self.check_velocity_bandwidth()
        if self.follow.disturbance_active or self.follow.disturbance_torque_a != 0 or self.follow.disturbance_torque_b != 0:
            raise RuntimeError('Unexpected torque pulse during angle-offset test')

    def run_angle_offset(self):
        self.report['angle_offset'] = dict(offset_radians=self.args.offset_radians,
            offset_degrees=self.args.angle_offset_deg, transition_time=self.args.transition_time,
            hold_duration=self.args.observe_seconds + 1.,
            definition='virtual_error = raw_relative_position_error + applied_offset',
            origins={self.axes[0]: self.follow.origin_a, self.axes[1]: self.follow.origin_b},
            observed_active=False, observed_target=False)
        self.phase('baseline')
        self.wait(lambda: self.guard_status and self.guard_status.get('phase') == 'baseline', .5)
        start = time.monotonic()
        try:
            while time.monotonic() - start < 5:
                self.pump()
                self.require_normal()
                if self.follow.angle_offset_active or self.follow.angle_offset_applied != 0:
                    raise RuntimeError('Unexpected angle offset during initial five-second observation')
        finally:
            self.report['baseline'] = dict(duration=time.monotonic() - start)
        # The independent guard validates the entire initial five-second recording
        # before acknowledging this phase; a vibrating baseline never gets a larger input.
        self.phase('offset_transition')
        self.wait(lambda: self.guard_status and self.guard_status.get('phase') == 'offset_transition', .5)
        request = BilateralAngleOffset.Request(session=self.session, offset=self.args.offset_radians,
            transition_time=self.args.transition_time, hold_duration=self.args.observe_seconds + 1., valid_for=.2)
        request.header.stamp = self.get_clock().now().to_msg()
        reply = self.future(self.angle_service.call_async(request))
        if not reply.success:
            raise RuntimeError('Angle offset rejected: ' + reply.message)
        self.report['angle_offset']['id'] = reply.offset_id
        self.wait(lambda: self.follow.angle_offset_active and self.follow.angle_offset_id == reply.offset_id, .5)
        self.report['angle_offset']['observed_active'] = True
        self.wait(lambda: self.follow.angle_offset_id == reply.offset_id and self.follow.angle_offset_active and
            not self.follow.angle_offset_ramping and math.isclose(self.follow.angle_offset_applied, self.args.offset_radians, abs_tol=1e-10)
            and self.guard_status.get('angle_offset', {}).get('id') == reply.offset_id
            and self.guard_status.get('angle_offset', {}).get('ready'), self.args.transition_time + .7)
        self.report['angle_offset']['observed_target'] = True
        self.report['angle_offset']['applied_radians'] = self.follow.angle_offset_applied
        self.phase('observation')
        self.wait(lambda: self.guard_status and self.guard_status.get('phase') == 'observation', .5)
        start = time.monotonic()
        try:
            while time.monotonic() - start < self.args.observe_seconds:
                self.pump()
                self.require_normal()
                if (not self.follow.angle_offset_active or self.follow.angle_offset_id != reply.offset_id or
                    self.follow.angle_offset_ramping or
                    not math.isclose(self.follow.angle_offset_applied, self.args.offset_radians, abs_tol=1e-10)):
                    raise RuntimeError('Angle offset changed or disappeared during final observation')
        finally:
            self.report['observation'] = dict(duration=time.monotonic() - start)

    def finish_observation(self):
        # Announce a normal completion through the guard before direct STOP:
        # ROS and the pipe are independent transports and can arrive out of order.
        # The guard itself stops immediately on this phase; a missing ack falls
        # through to the unconditional parent DISABLE in execute.finally.
        self.coupling = False
        self.phase('stopping')
        self.wait(lambda: self.guard_status and self.guard_status.get('phase') == 'stopping',
                  .25, stopping=True)

    def execute(self):
        try:
            duration = self.preflight()
            self.start_guard(duration)
            self.home(duration)
            self.run_coupling()
            self.report['result'] = 'passed'
        except BaseException as error:
            self.report['error'] = str(error)
            self.report['result'] = 'failed'
        finally:
            if self.armed:
                # Direct stop must precede fallible filesystem or IPC work.
                self.disable()
                try:
                    if self.report.get('phase') != 'stopping':
                        self.phase('stopping')
                except Exception as error:
                    fail(self.report, 'Stopping phase recording failed: ' + str(error))
                self.stop_both(self.report.get('error', 'Completed five-second observation'), self.report['result'] != 'passed')
            else:
                self.report['stop_confirmed'] = self.disabled()
            if self.settings_changed and self.report['stop_confirmed']:
                try:
                    after = self.report.get('stop_confirmation', {}).get('controller_response')
                    self.wait(lambda: (not self.armed or after is not None) and
                        self.controller_stopped(after, allow_fault=not self.armed),
                        3, stopping=True)
                    result = self.future(self.set_params.call_async(SetParametersAtomically.Request(
                        parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in self.old_settings.items()])), stopping=True)
                    self.report['configuration_restored'] = result.result.successful
                    if not result.result.successful:
                        raise RuntimeError(result.result.reason)
                except Exception as error:
                    self.report['configuration_restored'] = False
                    self.report['restore_error'] = str(error)
                    self.report['result'] = 'failed'
            if self.guard is not None:
                if self.report['stop_confirmed'] or not self.armed:
                    try:
                        self.send_guard('finish')
                        self.guard.wait(timeout=2)
                    except Exception:
                        self.guard.stdin.close()
                else:
                    self.guard.stdin.close()  # Independent guard remains alive until fresh STOP can be confirmed.
                self.report['guard_retained_pid'] = self.guard.pid if self.guard.poll() is None else None
                self.guard_log.close()
                status = self.guard_status
                monitor = self.args.output / 'monitor.json'
                if monitor.exists():
                    try:
                        status = json.loads(monitor.read_text())
                    except (OSError, ValueError) as error:
                        fail(self.report, 'Unreadable independent guard record: ' + str(error))
                elif self.report['result'] == 'passed':
                    fail(self.report, 'Independent guard final record unavailable')
                merge_guard_result(self.report, status, self.guard.poll())
                if self.guard.poll() is None:
                    fail(self.report, 'Independent guard retained until fresh STOP confirmation')
            if not self.report['stop_confirmed']:
                self.report['result'] = 'failed'
                self.report.setdefault('error', 'Fresh dual DISABLED or post-STOP controller confirmation unavailable')
            self.phase('finished')
            self.report['phases'][-1]['finished_monotonic'] = time.monotonic()
        return self.report


def summarize_baseline(rows, report):
    summary = report.setdefault('baseline', {})
    summary.setdefault('duration', 0.)
    if not math.isfinite(summary.get('duration', float('nan'))) or summary['duration'] < 5:
        fail(report, 'Incomplete initial five-second observation')
    for axis in ('a', 'b'):
        selected = [r for r in rows if r['axis'] == axis and r['phase'] == 'baseline']
        summary['samples_' + axis] = len(selected)
        if not selected:
            fail(report, 'Missing initial observation samples: ' + axis)
            continue
        position = [float(r['position']) for r in selected]
        velocity = [float(r['velocity']) for r in selected]
        summary['position_span_' + axis] = max(position) - min(position)
        summary['velocity_rms_' + axis] = math.sqrt(statistics.mean(v*v for v in velocity))
        summary['quiet_' + axis] = summary['velocity_rms_' + axis] <= .04 and summary['position_span_' + axis] <= .002
        if len(selected) < 100 or float(selected[-1]['receive_time']) - float(selected[0]['receive_time']) < summary.get('duration', 5) - .15:
            fail(report, 'Incomplete initial observation coverage: ' + axis)
        if not summary['quiet_' + axis]:
            fail(report, 'Motion or oscillation detected before angle injection: ' + axis)


def recorded_velocity_resolutions(directory):
    """Use recorded device limits, without changing acquisition or querying a motor."""
    resolutions = {}
    path = directory / 'state.jsonl'
    if path.exists():
        with path.open() as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                    axis = {'motor_a': 'a', 'motor_b': 'b'}.get(item.get('kind'))
                    vmax = float(item.get('data', {}).get('vmax', 0.))
                    if axis and axis not in resolutions and math.isfinite(vmax) and vmax > 0:
                        resolutions[axis] = 2 * vmax / 4095
                    if len(resolutions) == 2:
                        break
                except (ValueError, TypeError):
                    continue
    return resolutions


def summarize(directory, report):
    path = directory / 'feedback.csv'
    if not path.exists():
        if report['result'] == 'passed':
            fail(report, 'Missing raw observation recording')
        return
    try:
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        if any(not all(math.isfinite(float(r[k])) for k in ('receive_time', 'position', 'velocity', 'torque'))
               for r in rows):
            raise ValueError('Non-finite raw feedback')
        if any(r['axis'] not in ('a', 'b') or 'phase' not in r for r in rows):
            raise ValueError('Invalid raw feedback axis or phase')
    except (OSError, ValueError, KeyError, TypeError) as error:
        fail(report, 'Invalid raw observation recording: ' + str(error))
        return
    angle_test = report.get('test_kind') == 'angle_offset'
    if angle_test:
        summarize_baseline(rows, report)
        response = report.setdefault('response', {})
        for axis in ('a', 'b'):
            selected = [r for r in rows if r['axis'] == axis and r['phase'] in ('offset_transition', 'observation')]
            response['samples_' + axis] = len(selected)
            if selected:
                position = [float(r['position']) for r in selected]
                response['position_span_' + axis] = max(position) - min(position)
                response['peak_velocity_' + axis] = max(abs(float(r['velocity'])) for r in selected)
                response['peak_torque_' + axis] = max(abs(float(r['torque'])) for r in selected)
                response['duration_' + axis] = float(selected[-1]['receive_time']) - float(selected[0]['receive_time'])
        try:
            report['motion_analysis'] = analyze_angle_motion(rows, report, recorded_velocity_resolutions(directory))
            for reason in report['motion_analysis']['failures']:
                fail(report, reason)
            write_json(directory / 'motion_analysis.json', report['motion_analysis'])
        except Exception as error:
            # Analysis is offline, but unavailable evidence must never preserve a pass.
            fail(report, 'Angle motion analysis unavailable: ' + str(error))
    # A protective stop during the transition is an important result too.
    # Preserve its trace even when the final observation window never started.
    plot_svg(directory, rows)
    if angle_test:
        plot_svg(directory, rows, filename='response.svg', phases=('offset_transition', 'observation'))
    observed = [r for r in rows if r['phase'] == 'observation']
    if not observed:
        if report['result'] == 'passed':
            fail(report, 'Missing full-rate observation samples')
        return
    summary = report.setdefault('observation', {})
    summary.setdefault('duration', 0.)
    if not math.isfinite(summary.get('duration', float('nan'))) or summary['duration'] < 5:
        fail(report, 'Incomplete five-second observation')
    motion = False
    for axis in ('a', 'b'):
        selected = [r for r in observed if r['axis'] == axis]
        if not selected:
            fail(report, 'Missing full-rate observation samples')
            continue
        last = float(selected[-1]['receive_time'])
        tail = [r for r in selected if float(r['receive_time']) >= last - 1]
        velocity = [float(r['velocity']) for r in tail]
        position = [float(r['position']) for r in tail]
        summary['samples_' + axis] = len(selected)
        summary['tail_velocity_rms_' + axis] = math.sqrt(statistics.mean(v*v for v in velocity))
        summary['tail_position_span_' + axis] = max(position)-min(position)
        summary['final_position_' + axis] = statistics.mean(position)
        response = [r for r in rows if r['axis'] == axis and r['phase'] in ('disturbance', 'offset_transition', 'observation')]
        excursion = [float(r['position']) for r in response]
        span = max(excursion)-min(excursion) if excursion else 0
        summary['peak_displacement_' + axis] = span
        summary['peak_velocity_' + axis] = max(abs(float(r['velocity'])) for r in response)
        summary['peak_torque_' + axis] = max(abs(float(r['torque'])) for r in response)
        namespace_key = report.get('motor_' + axis)
        resolution = report.get('position_resolution', {}).get(namespace_key, 25 / 65535)
        summary['motion_threshold_' + axis] = 2 * resolution
        # Subtracting two decoded positions can round a two-code span a few
        # ulps below two separately computed resolution steps.
        motion |= span >= 2 * resolution or math.isclose(span, 2 * resolution, rel_tol=1e-9, abs_tol=1e-12)
        if len(tail) < 100 or last-float(selected[0]['receive_time']) < summary.get('duration', 5)-.15:
            fail(report, 'Incomplete observation coverage')
        if summary['tail_velocity_rms_' + axis] > .04 or summary['tail_position_span_' + axis] > .001:
            fail(report, 'Motion did not settle after disturbance')
    if angle_test and all('final_position_' + axis in summary for axis in ('a', 'b')):
        try:
            angle = report['angle_offset']
            origins = angle['origins']
            raw = (summary['final_position_a'] - origins[report['motor_a']]) - report['configuration']['direction'] * (
                summary['final_position_b'] - origins[report['motor_b']])
            virtual = raw + angle['applied_radians']
            summary.update(raw_relative_error=raw, virtual_relative_error=virtual, angle_error_tolerance=.01)
            if not angle['observed_active'] or not angle['observed_target'] or not math.isclose(
                    angle['applied_radians'], angle['offset_radians'], abs_tol=1e-10):
                fail(report, 'Requested angle offset was not confirmed')
            if not math.isfinite(virtual) or abs(virtual) > .01:
                fail(report, 'Motors stopped without closing the virtual angle gap')
        except (KeyError, TypeError, ValueError) as error:
            fail(report, 'Incomplete angle-offset evidence: ' + str(error))
    if not motion and report['result'] == 'passed' and angle_test:
        fail(report, 'No measurable response to the imposed angle gap')
    elif not motion and report['result'] == 'passed':
        report['result'], report['error'] = 'inconclusive', 'Perturbation produced no motion distinguishable from encoder noise'


def plot_svg(directory, rows, filename='motion.svg', phases=None):
    phases = phases or ('coupling', 'baseline', 'disturbance', 'offset_transition', 'observation')
    rows = [r for r in rows if r['phase'] in phases]
    if not rows:
        return
    t0 = min(float(r['receive_time']) for r in rows)
    t1 = max(float(r['receive_time']) for r in rows)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="510" viewBox="0 0 1000 510">',
        '<rect width="1000" height="510" fill="white"/><style>text{font:14px sans-serif}</style>',
        '<text x="60" y="22">Bilateral automatic test: raw feedback after return to zero</text>']
    for panel, field in enumerate(('position', 'velocity')):
        top = 50 + panel*225
        values = [float(r[field]) for r in rows]
        low, high = min(values), max(values)
        margin = max((high-low)*.1, .001 if field == 'position' else .01)
        low, high = low-margin, high+margin
        svg += [f'<rect x="90" y="{top}" width="860" height="170" fill="none" stroke="#bbb"/>',
                f'<text x="8" y="{top+30}">{field}</text>', f'<text x="8" y="{top+52}">{high:.4f}</text>',
                f'<text x="8" y="{top+169}">{low:.4f}</text>']
        for phase, color in [('baseline', '#e7f2e7'), ('offset_transition', '#fff0c2'), ('observation', '#e8f0fa')]:
            samples = [r for r in rows if r['phase'] == phase]
            if not samples:
                continue
            begin = min(float(r['receive_time']) for r in samples)
            end = max(float(r['receive_time']) for r in samples)
            x = 90 + 860 * (begin - t0) / max(t1 - t0, .001)
            width = 860 * (end - begin) / max(t1 - t0, .001)
            svg.append(f'<rect x="{x:.2f}" y="{top}" width="{width:.2f}" height="170" fill="{color}"/>')
            if panel == 0:
                svg.append(f'<text x="{x+3:.2f}" y="{top-8}">{phase}</text>')
        for axis, color in [('a','#1976d2'),('b','#ee6b18')]:
            samples=[r for r in rows if r['axis']==axis]
            points=' '.join(f'{90+860*(float(r["receive_time"])-t0)/max(t1-t0,.001):.2f},{top+170*(high-float(r[field]))/(high-low):.2f}' for r in samples)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1"/>')
    svg += [f'<text x="90" y="495">0 s</text><text x="850" y="495">{t1-t0:.2f} s</text>',
        '<text x="640" y="22" fill="#1976d2">motor A</text><text x="760" y="22" fill="#ee6b18">motor B</text></svg>']
    (directory / filename).write_text('\n'.join(svg))


def write_report(directory, report):
    write_json(directory / 'report.json', report)
    angle_test = report.get('test_kind') == 'angle_offset'
    flow = ('物理回零 → 启动双边力控 → 无偏置观察 5 秒 → 平滑加入虚拟角差 → 保持观察 5 秒 → 双轴停机。'
            if angle_test else '物理回零 → 启动双边力控 → 单轴短时扰动 → 撤扰后观察 → 双轴停机。')
    lines = ['# 双边力控自动检测', '',
        f'结果：**{report["result"]}**。双轴新鲜失能确认：{report["stop_confirmed"]}。', '',
        '流程：' + flow, '',
        '归零不修改编码器零点、软件偏置或 Flash。MOVE 没有总力矩夹紧，独立守卫监测实测力矩和速度。', '']
    if report.get('control_mode') and report.get('native_gains'):
        gains = report['native_gains']
        lines += [f'控制模式：`{report["control_mode"]}`；配置映射的原生 MIT 增益：'
                  f'Kp={gains["kp"]:g}，Kd={gains["kd"]:g}。', '']
    if 'required_velocity_bandwidth' in report:
        required = report['required_velocity_bandwidth']
        evidence = report.get('velocity_bandwidth', {})
        if required == 0:
            lines += ['速度带宽：未设置独立读回核验要求。', '']
        else:
            values = [f'{evidence[key]:g} Hz' if evidence.get(key) is not None else '未取得' for key in ('motor_a', 'motor_b')]
            status = '已核验' if evidence.get('matches_requirement') else '未核验'
            lines += [f'要求速度带宽：{required:g} Hz；ACTIVE 时独立读回 A={values[0]}、B={values[1]}，{status}。', '']
    homing = report.get('homing', {})
    if homing:
        lines += ['| 电机 | 归零前位置 (rad) | 归零确认位置 (rad) |', '|---|---:|---:|']
        for axis in (report['motor_a'], report['motor_b']):
            start = homing.get('origins', {}).get(axis)
            end = homing.get('final_positions', {}).get(axis)
            lines.append(f'| {axis} | {start:.6f} | ' + (f'{end:.6f}' if end is not None else '未完成') + ' |')
        lines += ['', f'归零位置容差：{homing["tolerance"]} rad。', '']
    pulse, observed = report.get('disturbance'), report.get('observation', {})
    if pulse:
        lines += [f'扰动：轴 {pulse["axis"].upper()}，{pulse["torque"]:+.3f} Nm，'
                  f'{pulse["duration"]:.3f} s；事件 ID {pulse["id"]}。', '']
    if angle_test:
        baseline, angle = report.get('baseline', {}), report.get('angle_offset', {})
        if angle:
            lines += [f'注入角差：{angle["offset_degrees"]:+.1f}°，五次曲线过渡 {angle["transition_time"]:.2f} s，'
                      f'事件 ID {angle.get("id", "未接受")}。', '',
                '虚拟角差 = 真实相对角差 + 注入偏置；正偏置由两轴相对转动抵消。'
                '真实位置和零点不改写，不要求两轴均分位移或回到相同物理角度。', '']
        if 'velocity_rms_a' in baseline and 'velocity_rms_b' in baseline:
            lines += [f'初始无偏置观察：{baseline["duration"]:.3f} s。', '',
                '| 初始观察指标 | A | B |', '|---|---:|---:|',
                f'| 全段速度 RMS (rad/s) | {baseline["velocity_rms_a"]:.6f} | {baseline["velocity_rms_b"]:.6f} |',
                f'| 全段位置范围 (rad) | {baseline["position_span_a"]:.6f} | {baseline["position_span_b"]:.6f} |', '']
    if 'samples_a' in observed and 'samples_b' in observed:
        label = '偏置达到目标后的保持观察' if angle_test else '撤扰后观察'
        lines += [f'{label}：{observed["duration"]:.3f} s。', '',
            '| 指标 | A | B |', '|---|---:|---:|',
            f'| 反馈样本数 | {observed["samples_a"]} | {observed["samples_b"]} |',
            f'| 扰动及观察段位置范围 (rad) | {observed["peak_displacement_a"]:.6f} | {observed["peak_displacement_b"]:.6f} |',
            f'| 扰动及观察段最大速度 (rad/s) | {observed["peak_velocity_a"]:.6f} | {observed["peak_velocity_b"]:.6f} |',
            f'| 扰动及观察段最大实测力矩 (Nm) | {observed["peak_torque_a"]:.6f} | {observed["peak_torque_b"]:.6f} |',
            f'| 最后 1 秒速度 RMS (rad/s) | {observed["tail_velocity_rms_a"]:.6f} | {observed["tail_velocity_rms_b"]:.6f} |',
            f'| 最后 1 秒位置范围 (rad) | {observed["tail_position_span_a"]:.6f} | {observed["tail_position_span_b"]:.6f} |', '']
    if angle_test and 'virtual_relative_error' in observed:
        lines += [f'最终真实相对角差：{math.degrees(observed["raw_relative_error"]):+.3f}°；'
                  f'最终虚拟角差：{math.degrees(observed["virtual_relative_error"]):+.3f}°。', '']
    if angle_test and not observed.get('samples_a'):
        lines += ['偏置后的完整观察尚未完成；不能判定 30° 到位后的停稳或持续振动。', '']
        response = report.get('response', {})
        if response.get('samples_a') and response.get('samples_b'):
            lines += ['| 提前停止前的响应 | A | B |', '|---|---:|---:|',
                f'| 已记录时长 (s) | {response["duration_a"]:.3f} | {response["duration_b"]:.3f} |',
                f'| 位置范围 (rad) | {response["position_span_a"]:.6f} | {response["position_span_b"]:.6f} |',
                f'| 峰值速度 (rad/s) | {response["peak_velocity_a"]:.6f} | {response["peak_velocity_b"]:.6f} |',
                f'| 峰值力矩 (Nm) | {response["peak_torque_a"]:.6f} | {response["peak_torque_b"]:.6f} |', '']
    analysis = report.get('motion_analysis') if angle_test else None
    if analysis:
        relative_policy = analysis.get('acceptance_policy') == 'relative_motion'
        lines += ['全程角差验收使用原始接收时间与位置、速度；下列限值表示检测分辨率，'
                  '不等于数学上不存在任何振动。原始图和旧有停机、采样、尾段判据保持独立。', '',
            ('差模按相对角差的方向、可分辨位移、闭合、过冲及停稳验收；共同位置自由，'
             '不要求每轴承担目标角差的固定份额。每轴反复逆转和持续纹波检查仍然有效。'
             if relative_policy else '此模式保留每轴最小有效位移要求。'), '',
            '| 全过渡及保持段指标 | A | B |', '|---|---:|---:|']
        axes = [analysis['axes'].get(a, {}) for a in ('a', 'b')]
        for label, key, scale, unit in [
                ('沿期望方向净位移 (°)', 'directed_displacement', 180 / math.pi, '.3f'),
                (('旧模式轴份额参考值，差模不作为门限 (°)' if relative_policy else '该轴最小有效位移 (°)'),
                 'required_directed_displacement', 180 / math.pi, '.3f'),
                ('完整逆向行程数 (>2 位置码)', 'complete_counter_excursions', 1, '.0f'),
                ('最大记录间隙 (ms)', 'max_recording_gap', 1000, '.3f')]:
            values = [format(a[key] * scale, unit) if key in a else '不可用' for a in axes]
            lines.append(f'| {label} | {values[0]} | {values[1]} |')
        bands = [a.get('persistent_bands', []) for a in axes]
        labels = [f'{b[0]["frequency_hz"]:.1f} Hz，{len(b)} 对相邻窗口' if b else '未检出超过分辨率的持续纹波' for b in bands]
        lines += [f'| 持续窄带纹波 | {labels[0]} | {labels[1]} |', '',
            '反复逆转：至少 3 次超过两编码格、且随后已反向确认的逆向行程。窄带纹波：'
            '连续两个 0.4 秒窗口，在 8–100 Hz（并受实际采样率限制）内，速度同频拟合'
            '解释至少 50% 残差能量，且位置、速度幅值均超过两码及基线噪声在该拟合下的有界增益。'
            '每个窗口还须有至少 3 个超过两位置码的完整残差往返周期。'
            '二次慢趋势仅用于该诊断；不平滑原始记录或原始图。', '']
        closure = analysis.get('closure', {})
        if closure:
            if relative_policy:
                lines += [f'相对净位移：{closure["directed_relative_displacement"]:.6f} rad；'
                    f'相对分辨阈值：{closure["required_resolvable_relative_displacement"]:.6f} rad；'
                    f'终端虚拟角差：{closure["terminal_virtual_error"]:.6f} rad；'
                    f'目标外最大过冲：{closure["overshoot_radians"]:.6f} rad；'
                    f'完整相对逆向行程：{closure["complete_relative_counter_excursions"]}。', '']
            def elapsed(value):
                return f'{value:.3f} s' if value is not None else '未证实'
            lines += ['| 闭合比例 | 首次达到 | 此后始终留在对应终值带，至少确认 1 秒 |', '|---|---:|---:|']
            for percentage in (90, 95):
                lines.append(f'| {percentage}% | {elapsed(closure.get(f"first_{percentage}_seconds"))} | '
                             f'{elapsed(closure.get(f"settled_{percentage}_seconds"))} |')
            lines += ['', f'全条件停稳时间：{elapsed(closure.get("settling_seconds"))}。'
                '从该时点起，每个 120 ms 窗口均有至少 100 ms / 20 样本覆盖、速度 RMS ≤0.04 rad/s、'
                '速度峰值 ≤0.15 rad/s；各轴剩余位置范围 ≤0.001 rad、'
                '最终虚拟角差始终 ≤0.01 rad，并至少保留 1 秒确认。',
                '计时基准：' + analysis['time_reference'] + '；不是仅一次越过阈值的到位时间。', '']
        if analysis['failures']:
            lines += ['全程分析未通过：', ''] + ['- ' + reason for reason in analysis['failures']] + ['']
        lines += ['[逐窗口分析与判据](motion_analysis.json)', '']
    default_reason = ('相对角差闭合、全程可分辨振荡检查及尾段停稳通过；不要求每轴固定运动份额。'
                      if angle_test and analysis and analysis.get('acceptance_policy') == 'relative_motion' else
                      '双轴有效运动、全程可分辨振荡检查及尾段停稳通过；不代替人工接触或任意负载验收。'
                      if angle_test else '小扰动响应和尾段停稳通过；不代替人工接触或任意负载验收。')
    lines += [f'说明：{report.get("error", default_reason)}', '',
        '[完整参数与判定](report.json) · [逐帧反馈](feedback.csv) · [状态与事件](state.jsonl)', '']
    if (directory / 'motion.svg').exists():
        lines += ['![运动波形](motion.svg)', '']
    if (directory / 'response.svg').exists():
        lines += ['![仅过渡及观察段的原始运动波形](response.svg)', '']
    (directory / 'report.md').write_text('\n'.join(lines))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    env=parser.add_mutually_exclusive_group(required=True)
    env.add_argument('--hardware',action='store_true');env.add_argument('--simulation',action='store_true')
    parser.add_argument('--controller',default='/bilateral_control')
    parser.add_argument('--motor-a',default='/motor_1');parser.add_argument('--motor-b',default='/motor_2')
    parser.add_argument('--axis',choices=('a','b'),default='a')
    parser.add_argument('--torque',type=float,default=.2)
    parser.add_argument('--pulse-duration',type=float,default=.1)
    parser.add_argument('--angle-offset-deg',type=float,
        help='Use five seconds at rest, inject this virtual relative angle, then observe five seconds')
    parser.add_argument('--transition-time',type=float,default=2.,
        help='Angle-offset quintic transition duration; hardware requires 2..5 seconds')
    parser.add_argument('--observe-seconds',type=float,default=5.)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    if not all(math.isfinite(v) for v in (args.torque,args.pulse_duration,args.observe_seconds)) or not 0<abs(args.torque)<=.3 or not .04<=args.pulse_duration<=.2 or not 5<=args.observe_seconds<=10:
        parser.error('Require 0<|torque|<=0.3 Nm, pulse 0.04..0.2 s, and observation 5..10 s')
    args.test_kind = 'angle_offset' if args.angle_offset_deg is not None else 'pulse'
    args.offset_radians = 0.
    if args.test_kind == 'angle_offset':
        if not math.isfinite(args.angle_offset_deg) or not 0 < abs(args.angle_offset_deg) <= 30:
            parser.error('Angle offset must be finite and within +/-30 degrees, excluding zero')
        if not math.isfinite(args.transition_time) or not (2 if args.hardware else 0) <= args.transition_time <= 5:
            parser.error('Angle transition requires 2..5 s on hardware, 0..5 s in simulation')
        if args.observe_seconds > 9:
            parser.error('Angle observation must be 5..9 s to fit the independent hold deadline')
        args.offset_radians = math.radians(args.angle_offset_deg)
    for key in ('controller','motor_a','motor_b'):setattr(args,key,namespace(getattr(args,key)))
    if args.motor_a==args.motor_b:parser.error('Motor namespaces must differ')
    args.output=(args.output or Path.home()/'.ros/dm_motor_bilateral_control/auto-tests'/
        (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])).resolve()
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error('Output directory must be empty; prior experiments are never overwritten')
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node=AutomaticTest(args)
    def interrupt(_signum,_frame):node.interrupted=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        report=node.execute()
        summarize(args.output,report)
        write_report(args.output, report)
        write_json(args.output / 'progress.json', report)
        print(json.dumps(dict(result=report['result'],report=str(args.output/'report.json'),stop_confirmed=report['stop_confirmed']),ensure_ascii=False),flush=True)
        return 0 if report['result']=='passed' else 2
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    sys.exit(main())
