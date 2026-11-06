"""Independent, stop-only monitor for the bilateral automatic acceptance CLI."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Trigger
from dm_motor_interfaces.msg import BilateralState, MotorFeedback, MotorRequest, MotorState
from dm_motor_bilateral_control.velocity_bandwidth import bandwidth_evidence, required_bandwidth

PHASES = {'preflight', 'homing', 'handoff', 'coupling', 'baseline', 'offset_transition',
          'disturbance', 'manual_contact', 'observation', 'stopping'}
COUPLED = {'coupling', 'baseline', 'offset_transition', 'disturbance', 'manual_contact', 'observation'}
HANDOFF_GRACE = 0.25
OFFSET_EPSILON = 1e-6
# Allows the parent's 50-ms status/IPC round trip, never extending ACTIVE's deadline.
MANUAL_PHASE_ACK_GRACE = 0.25
MANUAL_PHASE_SECONDS = 5.0


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return str(value) if isinstance(value, float) and not math.isfinite(value) else value


def validate(config):
    required_bandwidth(config.get('required_velocity_bandwidth', 0.0))
    if config.get('test_kind', 'pulse') not in ('pulse', 'angle_offset', 'manual'):
        raise ValueError('test_kind must be pulse, angle_offset or manual')
    if config['environment'] not in ('hardware', 'simulation'):
        raise ValueError('environment must be hardware or simulation')
    for key in ('motor_a', 'motor_b', 'controller'):
        if not isinstance(config[key], str) or not config[key].startswith('/') or config[key].endswith('/'):
            raise ValueError(key + ' must be an absolute ROS name without a trailing slash')
    if config['motor_a'] == config['motor_b']:
        raise ValueError('Distinct motor namespaces required')
    for key in ('total_timeout', 'active_timeout', 'max_speed', 'max_torque', 'max_temperature',
                'feedback_timeout', 'max_test_travel', 'home_margin', 'parent_timeout'):
        if isinstance(config[key], bool) or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(key + ' must be positive and finite')
    if not config['active_timeout'] <= config['total_timeout'] <= 60:
        raise ValueError('Require active_timeout <= total_timeout <= 60 seconds')
    for axis in (config['motor_a'], config['motor_b']):
        if not math.isfinite(config['initial_positions'][axis]):
            raise ValueError('Finite initial positions required')
    if config.get('test_kind') == 'angle_offset':
        for key in ('offset_radians', 'transition_time', 'baseline_seconds', 'observe_seconds'):
            if isinstance(config[key], bool) or not math.isfinite(config[key]):
                raise ValueError(key + ' must be finite')
        minimum_transition = 2 if config['environment'] == 'hardware' else 0
        if not 0 < abs(config['offset_radians']) <= math.pi / 6 \
                or not minimum_transition <= config['transition_time'] <= 5 \
                or config['baseline_seconds'] < 5 or not 5 <= config['observe_seconds'] <= 9:
            raise ValueError('Require abs(offset)<=30 degrees, a permitted transition and at least five seconds per observation')
        if config['baseline_seconds'] + config['transition_time'] + config['observe_seconds'] >= config['active_timeout']:
            raise ValueError('ACTIVE deadline must include both observations and the offset transition')
    elif config.get('test_kind') == 'manual':
        fixed = dict(baseline_seconds=5., contact_seconds=5., observe_seconds=5.,
            active_timeout=16.5, max_speed=.8, max_torque=.7, max_temperature=45.,
            feedback_timeout=.05, max_test_travel=.08, home_margin=.08, parent_timeout=.25,
            required_velocity_bandwidth=100.)
        if any(isinstance(config.get(key), bool) or config.get(key) != value
               for key, value in fixed.items()) or config['total_timeout'] > 53:
            raise ValueError('Manual test requires fixed 5/5/5-second phases, V_BW100 and bounded guards')


class Guard(Node):
    def __init__(self, config):
        super().__init__('bilateral_auto_test_guard_' + uuid.uuid4().hex[:8])
        self.config = config
        self.angle_test = config.get('test_kind') == 'angle_offset'
        self.manual_test = config.get('test_kind') == 'manual'
        self.axes = {'a': config['motor_a'], 'b': config['motor_b']}
        self.directory = Path(config['output_directory'])
        self.directory.mkdir(parents=True, exist_ok=True)
        self.events = (self.directory / 'state.jsonl').open('w', buffering=65536)
        self.feedback_file = (self.directory / 'feedback.csv').open('w', newline='', buffering=65536)
        self.writer = csv.writer(self.feedback_file)
        self.writer.writerow(('host_time', 'axis', 'phase', 'receive_time', 'position', 'velocity', 'torque', 'feedback_count'))
        self.phase, self.armed, self.ready, self.done = 'preflight', False, False, False
        self.phase_history = [{'name': 'preflight', 'time': time.monotonic()}]
        self.failed, self.reason, self.stop_reason = False, '', ''
        self.arm_time = self.active_time = self.stop_time = self.confirmed_since = None
        self.stop_confirmed, self.parent_closed, self.logging_failed = False, False, False
        self.states, self.state_times, self.feedback, self.follow = {}, {}, {}, None
        self.follow_time, self.parent_time = 0.0, time.monotonic()
        self.last_status = self.last_disable = self.last_service_stop = 0.0
        self.test_origins, self.home_sources, self.stop_counts = {}, {}, {}
        self.handoff_time, self.handoff_closed = None, set()
        self.baseline = {a: dict(t_first=None, t_last=None, count=0, sumv2=0.0, qmin=None, qmax=None)
                         for a in self.axes}
        self.offset_transition_time, self.previous_offset_id, self.bound_offset_id = None, None, None
        self.angle_offset = {}
        self.manual_reference = None
        self.manual_timing = dict(baseline_started=None, contact_started=None,
            scheduled_release=None, observation_started=None, release_cue_monotonic=None,
            release_physically_verified=False, phase_ack_grace=MANUAL_PHASE_ACK_GRACE)
        self.velocity_bandwidth = {}
        self.counts = {'a': 0, 'b': 0, 'motor_states': 0, 'bilateral_states': 0}
        self.peak = {a: {'speed': 0.0, 'torque': 0.0, 'temperature': 0.0, 'test_travel': 0.0} for a in self.axes}
        self.observation = {'started': None, 'samples': {'a': 0, 'b': 0}}
        self.buffer, self.sequence, self.stop_future = b'', 0, None
        self.client = 'bilateral_guard_' + uuid.uuid4().hex
        self.control_publishers = {a: self.create_publisher(MotorRequest, ns + '/control', 64) for a, ns in self.axes.items()}
        self.stop_client = self.create_client(Trigger, config['controller'] + '/stop')
        self.observers = []
        self.attach_observers()
        os.set_blocking(sys.stdin.fileno(), False)
        os.set_blocking(sys.stdout.fileno(), False)

    def attach_observers(self, minimal=False):
        qos = QoSProfile(depth=1024, reliability=ReliabilityPolicy.RELIABLE)
        for axis, namespace in self.axes.items():
            self.observers.append(self.create_subscription(MotorState, namespace + '/state',
                lambda msg, a=axis: self.motor_state(a, msg, minimal), qos))
            self.observers.append(self.create_subscription(MotorFeedback, namespace + '/feedback',
                lambda msg, a=axis: self.motor_feedback(a, msg, minimal), qos))
        if not minimal:
            self.observers.append(self.create_subscription(BilateralState, self.config['controller'] + '/state', self.bilateral, qos))

    def record(self, kind, value):
        if self.logging_failed:
            return
        try:
            self.events.write(json.dumps(clean({'host_time': time.monotonic(), 'phase': self.phase,
                'kind': kind, 'data': value}), allow_nan=False) + '\n')
        except (OSError, ValueError, TypeError) as error:
            self.logging_failed = True
            self.stop('recording_failed: ' + str(error), True)

    def disable(self):
        if not self.armed:
            return
        now = time.monotonic()
        for publisher in self.control_publishers.values():
            self.sequence += 1
            request = MotorRequest(client_id=self.client, request_id=self.sequence,
                kind=MotorRequest.OPERATION, operation=MotorRequest.DISABLE)
            request.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(request)
        self.last_disable = now
        if now - self.last_service_stop >= 0.2 and self.stop_client.service_is_ready():
            if self.stop_future is None or self.stop_future.done():
                self.stop_future = self.stop_client.call_async(Trigger.Request())
                self.last_service_stop = now

    def stop(self, reason, failure=False):
        if failure and not self.failed:
            self.failed, self.reason = True, reason
        if self.stop_time is not None:
            return
        self.stop_reason = reason
        if not self.armed:
            return  # Preflight rejection never disables someone else's active motor.
        self.stop_time, self.phase = time.monotonic(), 'stopping'
        self.stop_counts = {a: max(self.feedback[a].feedback_count if a in self.feedback else 0,
            self.states[a].feedback_count if a in self.states else 0) for a in self.axes}
        self.disable()  # Never make the emergency output depend on successful logging.
        self.record('stop', {'reason': reason, 'failure': failure, 'counts': self.stop_counts})

    def motor_state(self, axis, msg, minimal=False):
        self.states[axis], self.state_times[axis] = msg, time.monotonic()
        self.counts['motor_states'] += 1
        if self.armed and self.stop_time is None:
            if msg.host_fault or msg.device_fault or msg.phase == MotorState.FAULT or msg.error:
                self.stop('motor_fault: ' + axis + ': ' + msg.error, True)
            elif msg.simulated != (self.config['environment'] == 'simulation') or not msg.absolute_position_valid or not msg.ready or msg.mode != 1:
                self.stop('motor_identity_or_position_invalid: ' + axis, True)
            elif msg.target_active:
                if self.phase == 'homing':
                    if not msg.task_active or not msg.active_source:
                        self.stop('non_task_source_during_home: ' + axis, True)
                    elif axis in self.home_sources and msg.active_source != self.home_sources[axis]:
                        self.stop('home_source_changed: ' + axis, True)
                    else:
                        self.home_sources[axis] = msg.active_source
                elif self.phase in COUPLED:
                    source = getattr(self.follow, 'controller_source', '')
                    if not source or msg.active_source != source:
                        self.stop('bilateral_source_changed: ' + axis, True)
                elif self.phase == 'handoff':
                    # Task completion and state snapshots use different transports.
                    # Admit only the already observed home task's queued snapshots.
                    if (axis in self.handoff_closed or not msg.task_active or not msg.active_source
                            or msg.active_source != self.home_sources.get(axis)
                            or self.handoff_time is None
                            or time.monotonic() - self.handoff_time >= HANDOFF_GRACE):
                        self.stop('unexpected_active_output: handoff: ' + axis, True)
                else:
                    self.stop('unexpected_active_output: ' + self.phase + ': ' + axis, True)
            if self.phase == 'handoff' and msg.phase == MotorState.DISABLED and msg.status == 0 \
                    and not msg.target_active and not msg.task_active:
                self.handoff_closed.add(axis)
        if not minimal:
            self.record('motor_' + axis, message_to_ordereddict(msg))

    def motor_feedback(self, axis, msg, minimal=False):
        self.feedback[axis] = msg
        self.counts[axis] += 1
        if self.armed and self.stop_time is None:
            values = (msg.receive_time, msg.position, msg.velocity, msg.torque)
            if not all(math.isfinite(x) for x in values) or not msg.absolute_position_valid:
                self.stop('invalid_feedback: ' + axis, True)
            else:
                peak = self.peak[axis]
                for name, value in (('speed', abs(msg.velocity)), ('torque', abs(msg.torque)),
                                    ('temperature', max(msg.mos_temperature, msg.rotor_temperature))):
                    peak[name] = max(peak[name], value)
                    limit = self.config['max_' + name]
                    if (value >= limit if name == 'temperature' else value > limit):
                        self.stop(name + '_limit: ' + axis, True)
                if self.test_origins and self.phase in COUPLED:
                    travel = abs(msg.position - self.test_origins[self.axes[axis]])
                    peak['test_travel'] = max(peak['test_travel'], travel)
                    travel_limit = .08 if self.angle_test and self.phase in ('coupling', 'baseline') \
                        else self.config['max_test_travel']
                    if travel > travel_limit:
                        self.stop('test_travel_limit: ' + axis, True)
                else:
                    start = self.config['initial_positions'][self.axes[axis]]
                    margin = self.config['home_margin']
                    if not min(start, 0) - margin <= msg.position <= max(start, 0) + margin:
                        self.stop('home_corridor_limit: ' + axis, True)
                if self.phase == 'observation':
                    self.observation['samples'][axis] += 1
                if (self.angle_test or self.manual_test) and self.phase == 'baseline' and self.stop_time is None:
                    sample = self.baseline[axis]
                    if sample['t_first'] is None:
                        sample['t_first'], sample['qmin'], sample['qmax'] = msg.receive_time, msg.position, msg.position
                    sample['t_last'] = msg.receive_time
                    sample['count'] += 1
                    sample['sumv2'] += msg.velocity * msg.velocity
                    sample['qmin'], sample['qmax'] = min(sample['qmin'], msg.position), max(sample['qmax'], msg.position)
        if not minimal and not self.logging_failed:
            try:
                self.writer.writerow((time.monotonic(), axis, self.phase, msg.receive_time,
                    msg.position, msg.velocity, msg.torque, msg.feedback_count))
            except OSError as error:
                self.logging_failed = True
                self.stop('feedback_recording_failed: ' + str(error), True)

    def bilateral(self, msg):
        self.follow, self.follow_time = msg, time.monotonic()
        self.counts['bilateral_states'] += 1
        if self.angle_test:
            self.check_angle_offset(msg)
        if getattr(self, 'manual_test', False) and self.armed and self.phase in COUPLED \
                and self.stop_time is None:
            reason = self.manual_injection_failure(msg)
            if reason:
                self.stop(reason, True)
        if self.armed and self.phase in COUPLED and self.stop_time is None:
            if msg.phase == BilateralState.ACTIVE and self.active_time is None:
                self.active_time = time.monotonic()
            if msg.phase == BilateralState.ACTIVE:
                self.velocity_bandwidth = bandwidth_evidence(
                    self.config.get('required_velocity_bandwidth', 0.0), msg)
                if not self.velocity_bandwidth['matches_requirement']:
                    self.stop('velocity_bandwidth: ' + self.velocity_bandwidth['error'], True)
            if msg.phase in (BilateralState.FAULT, BilateralState.STOPPING) or (
                    self.active_time is not None and msg.phase != BilateralState.ACTIVE):
                self.stop('bilateral_left_active: ' + msg.message, True)
            elif msg.phase == BilateralState.ACTIVE and (msg.quality != BilateralState.NORMAL or not msg.operator_guarded):
                self.stop('bilateral_quality_or_operator_guard', True)
        self.record('bilateral', message_to_ordereddict(msg))

    def check_angle_offset(self, msg):
        value = dict(id=msg.angle_offset_id, active=msg.angle_offset_active,
            target=msg.angle_offset_target, applied=msg.angle_offset_applied,
            ramping=msg.angle_offset_ramping, remaining=msg.angle_offset_remaining, ready=False)
        previous = self.angle_offset
        self.angle_offset = value
        if not self.armed or self.phase not in COUPLED or self.stop_time is not None:
            return  # STOP may clear the offset; it must never become another excitation.
        numbers = (value['target'], value['applied'], value['remaining'],
                   msg.position_error, msg.virtual_position_error)
        if not all(math.isfinite(x) for x in numbers) or value['remaining'] < 0 \
                or abs(msg.virtual_position_error - msg.position_error - value['applied']) > OFFSET_EPSILON:
            self.stop('invalid_angle_offset_telemetry', True)
            return
        if msg.disturbance_active or msg.disturbance_torque != 0 \
                or msg.disturbance_torque_a != 0 or msg.disturbance_torque_b != 0:
            self.stop('unexpected_torque_disturbance', True)
            return
        if self.phase in ('coupling', 'baseline'):
            if value['active'] or abs(value['applied']) > OFFSET_EPSILON:
                self.stop('unexpected_angle_offset_before_transition', True)
            return
        if self.phase not in ('offset_transition', 'observation'):
            self.stop('unexpected_angle_test_phase', True)
            return
        target = self.config['offset_radians']
        if self.bound_offset_id is None:
            if not value['active'] and value['id'] == self.previous_offset_id \
                    and abs(value['applied']) <= OFFSET_EPSILON:
                return  # Parent waits for this phase's acknowledgement before submitting the request.
            if not value['active'] or value['id'] != self.previous_offset_id + 1:
                self.stop('unexpected_angle_offset_id', True)
                return
            self.bound_offset_id = value['id']
        signed_applied = math.copysign(1, target) * value['applied']
        if not value['active'] or value['id'] != self.bound_offset_id \
                or abs(value['target'] - target) > OFFSET_EPSILON or value['remaining'] <= 0 \
                or not -OFFSET_EPSILON <= signed_applied <= abs(target) + OFFSET_EPSILON:
            self.stop('angle_offset_canceled_or_changed', True)
            return
        if previous.get('id') == self.bound_offset_id and previous.get('active') \
                and math.copysign(1, target) * (value['applied'] - previous['applied']) < -OFFSET_EPSILON:
            self.stop('angle_offset_reversed', True)
            return
        value['ready'] = not value['ramping'] and abs(value['applied'] - target) <= OFFSET_EPSILON
        if self.phase == 'observation' and not value['ready']:
            self.stop('angle_offset_not_held_during_observation', True)

    def baseline_failure(self):
        for axis, sample in self.baseline.items():
            if sample['count'] < 100 or sample['t_first'] is None \
                    or sample['t_last'] - sample['t_first'] < self.config['baseline_seconds'] - .15:
                return 'baseline_coverage: ' + axis
            if math.sqrt(sample['sumv2'] / sample['count']) > .04 or sample['qmax'] - sample['qmin'] > .002:
                return 'baseline_motion: ' + axis
        return ''

    def manual_injection_failure(self, msg, *, bind=False):
        """Historical completed event ids/offset target are allowed, never renewed."""
        values = (msg.disturbance_torque, msg.disturbance_torque_a, msg.disturbance_torque_b,
            msg.disturbance_remaining, msg.angle_offset_target, msg.angle_offset_applied,
            msg.angle_offset_remaining, msg.position_error, msg.virtual_position_error)
        if not all(math.isfinite(value) for value in values):
            return 'manual_invalid_injection_telemetry'
        if msg.disturbance_active or any(value != 0 for value in values[:4]):
            return 'manual_unexpected_torque_disturbance'
        if msg.angle_offset_active or msg.angle_offset_ramping or msg.angle_offset_applied != 0 \
                or msg.angle_offset_remaining != 0 or msg.virtual_position_error != msg.position_error:
            return 'manual_unexpected_angle_offset'
        reference = (msg.disturbance_id, msg.angle_offset_id, msg.angle_offset_target)
        if bind:
            self.manual_reference = reference
        elif self.manual_reference is None or reference != self.manual_reference:
            return 'manual_injection_reference_changed'
        return ''

    def manual_phase_failure(self, phase, request, now):
        if phase == 'stopping':
            return ''  # A stop is permitted from every phase, including early aborts.
        expected = dict(preflight='homing', homing='handoff', handoff='coupling',
            coupling='baseline', baseline='manual_contact', manual_contact='observation')
        if phase != expected.get(self.phase):
            return 'manual_invalid_phase_transition'
        if phase not in COUPLED:
            return ''
        if not self.fresh():
            return 'manual_phase_stale'
        if phase == 'coupling':
            return self.manual_injection_failure(self.follow, bind=True)
        if self.follow.phase != BilateralState.ACTIVE or self.follow.quality != BilateralState.NORMAL \
                or not self.follow.operator_guarded or self.follow.recovery_scale != 1:
            return 'manual_controller_not_normal'
        reason = self.manual_injection_failure(self.follow)
        if reason:
            return reason
        if phase == 'baseline':
            self.manual_timing['baseline_started'] = now
        elif phase == 'manual_contact':
            began = self.manual_timing['baseline_started']
            if began is None or now - began < MANUAL_PHASE_SECONDS:
                return 'manual_baseline_too_short'
            if now - began > MANUAL_PHASE_SECONDS + MANUAL_PHASE_ACK_GRACE:
                return 'manual_baseline_deadline'
            reason = self.baseline_failure()
            if reason:
                return reason
            self.manual_timing['contact_started'] = now
            self.manual_timing['scheduled_release'] = now + MANUAL_PHASE_SECONDS
        elif phase == 'observation':
            release = self.manual_timing['scheduled_release']
            if release is None or now < release:
                return 'manual_contact_too_short'
            if now > release + MANUAL_PHASE_ACK_GRACE:
                return 'manual_contact_deadline'
            cue = request.get('release_cue_monotonic')
            if cue is not None and (isinstance(cue, bool) or not isinstance(cue, (int, float))
                    or not math.isfinite(cue)):
                return 'manual_invalid_release_cue'
            self.manual_timing['release_cue_monotonic'] = cue
            self.manual_timing['observation_started'] = now
        return ''

    def manual_deadline_failure(self, now):
        key = {'baseline': 'baseline_started', 'manual_contact': 'contact_started',
               'observation': 'observation_started'}.get(self.phase)
        began = self.manual_timing[key] if key else None
        return ('manual_' + self.phase.removeprefix('manual_') + '_deadline') if began is not None and \
            now - began > MANUAL_PHASE_SECONDS + MANUAL_PHASE_ACK_GRACE else ''

    def manual_ready_for_baseline(self):
        """Publish this observer's readiness; parent/ROS arrival order is independent."""
        if not self.manual_test or not self.armed or self.failed or self.stop_time is not None \
                or self.phase != 'coupling' or not self.fresh():
            return False
        msg = self.follow
        return bool(msg.phase == BilateralState.ACTIVE and msg.quality == BilateralState.NORMAL
            and msg.operator_guarded and msg.recovery_scale == 1
            and bandwidth_evidence(self.config['required_velocity_bandwidth'], msg)['matches_requirement']
            and not self.manual_injection_failure(msg))

    def fresh(self, include_controller=True):
        now, timeout = time.monotonic(), self.config['feedback_timeout']
        for axis in self.axes:
            feedback, state = self.feedback.get(axis), self.states.get(axis)
            if feedback is None or state is None:
                return False
            if not math.isfinite(feedback.receive_time) or not 0 <= now - feedback.receive_time <= timeout:
                return False
            if not state.feedback_fresh or not math.isfinite(state.feedback_age) or state.feedback_age < 0:
                return False
            if state.feedback_age + now - self.state_times[axis] > timeout:
                return False
        return not include_controller or (self.follow is not None and now - self.follow_time <= timeout)

    def disabled(self, newer=False):
        return self.fresh(False) and all(self.states[a].phase == MotorState.DISABLED and self.states[a].status == 0
            and not self.states[a].target_active and not self.feedback[a].target_active
            and (not newer or min(self.feedback[a].feedback_count, self.states[a].feedback_count) > self.stop_counts[a])
            for a in self.axes)

    def input(self):
        try:
            block = os.read(sys.stdin.fileno(), 65536)
            if block:
                self.buffer += block
            elif not self.parent_closed:
                self.parent_closed = True
                if self.armed and not self.stop_confirmed:
                    self.stop('parent_pipe_closed', True)
                elif not self.armed:
                    self.done = True
        except BlockingIOError:
            pass
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            request = json.loads(line)
            self.parent_time = time.monotonic()
            operation = request['op']
            if operation == 'arm':
                if self.armed or not self.ready or self.failed:
                    self.stop('arm_preflight_rejected', True)
                else:
                    self.armed, self.arm_time = True, time.monotonic()
                    self.record('arm', self.config)
            elif operation == 'phase':
                phase = request['name']
                if phase not in PHASES or not self.armed or self.stop_time is not None:
                    self.stop('invalid_phase_transition', True)
                    continue
                if self.manual_test:
                    reason = self.manual_phase_failure(phase, request, time.monotonic())
                    if reason:
                        self.stop(reason, True)
                        continue
                elif phase == 'manual_contact':
                    self.stop('manual_phase_in_automatic_test', True)
                    continue
                elif self.angle_test:
                    reason = ''
                    if phase == 'disturbance':
                        reason = 'torque_phase_in_angle_test'
                    elif phase == 'baseline' and self.phase != 'coupling':
                        reason = 'invalid_baseline_transition'
                    elif phase == 'offset_transition':
                        reason = 'invalid_offset_transition' if self.phase != 'baseline' else self.baseline_failure()
                        if not reason and not self.fresh():
                            reason = 'offset_baseline_stale'
                        if not reason:
                            self.previous_offset_id = self.follow.angle_offset_id
                            self.offset_transition_time = time.monotonic()
                    elif phase == 'observation':
                        expected_id = request.get('offset_id', self.bound_offset_id)
                        if self.phase != 'offset_transition' or not self.fresh() \
                                or not self.angle_offset.get('ready') or expected_id != self.bound_offset_id:
                            reason = 'angle_offset_observation_not_ready'
                    if reason:
                        self.stop(reason, True)
                        continue
                elif phase in ('baseline', 'offset_transition'):
                    self.stop('angle_phase_in_pulse_test', True)
                    continue
                if phase in COUPLED and not self.test_origins:
                    if not self.fresh():
                        self.stop('coupling_baseline_stale', True)
                        continue
                    self.test_origins = {ns: self.feedback[a].position for a, ns in self.axes.items()}
                self.phase = phase
                if phase == 'handoff' and self.handoff_time is None:
                    self.handoff_time = time.monotonic()
                    self.handoff_closed = {a for a, s in self.states.items() if s.phase == MotorState.DISABLED
                        and s.status == 0 and not s.target_active and not s.task_active}
                self.phase_history.append({'name': phase, 'time': time.monotonic()})
                if phase == 'observation' and self.observation['started'] is None:
                    self.observation['started'] = time.monotonic()
                if phase == 'stopping':
                    self.stop('parent_stopping', False)
                self.record('phase', {'name': phase, 'test_origins': self.test_origins,
                    'offset_id': self.bound_offset_id})
            elif operation == 'stop':
                self.stop(request.get('reason', 'parent_stop'), request.get('failure', False))
            elif operation == 'finish':
                if self.armed and not self.stop_confirmed:
                    self.stop('finish_before_stop_confirmed', True)
                else:
                    self.done = True
            elif operation != 'ping':
                self.stop('unknown_parent_operation', True)

    def status(self):
        return dict(kind='status', ready=self.ready, armed=self.armed, phase=self.phase, failed=self.failed,
            reason=self.reason if self.failed else self.stop_reason, stop_confirmed=self.stop_confirmed,
            counts=self.counts, peak=self.peak, test_origins=self.test_origins, observation=self.observation,
            baseline=self.baseline, angle_offset=self.angle_offset,
            velocity_bandwidth=getattr(self, 'velocity_bandwidth', {}),
            arm_time=self.arm_time, active_time=self.active_time, stop_time=self.stop_time,
            phase_history=self.phase_history,
            **({'manual_timing': self.manual_timing,
                'manual_ready_for_baseline': self.manual_ready_for_baseline()} if self.manual_test else {}))

    def tick(self):
        now = time.monotonic()
        if not self.armed:
            self.ready = not self.failed and self.disabled() and self.fresh() and self.follow.phase == BilateralState.IDLE \
                and self.follow.stop_confirmed and all(s.simulated == (self.config['environment'] == 'simulation')
                and s.ready and s.absolute_position_valid and s.mode == 1 and not s.host_fault and not s.device_fault and not s.error
                for s in self.states.values())
        elif self.failed:
            self.ready = False
        self.input()
        if self.armed and self.stop_time is None:
            if not self.fresh():
                self.stop('feedback_or_controller_stale', True)
            elif now - self.parent_time > self.config['parent_timeout']:
                self.stop('parent_heartbeat_lost', True)
            elif now - self.arm_time >= self.config['total_timeout']:
                self.stop('total_deadline', True)
            elif self.phase == 'handoff' and self.handoff_time is not None \
                    and now - self.handoff_time >= HANDOFF_GRACE \
                    and any(s.target_active or s.task_active for s in self.states.values()):
                self.stop('home_handoff_deadline', True)
            elif self.angle_test and self.phase == 'offset_transition' and self.offset_transition_time is not None \
                    and self.bound_offset_id is None and now - self.offset_transition_time >= .5:
                self.stop('angle_offset_activation_deadline', True)
            elif self.angle_test and self.phase == 'offset_transition' and self.offset_transition_time is not None \
                    and now - self.offset_transition_time >= self.config['transition_time'] + .5 \
                    and not self.angle_offset.get('ready'):
                self.stop('angle_offset_transition_deadline', True)
            elif self.active_time is not None and now - self.active_time >= self.config['active_timeout']:
                self.stop('active_deadline', True)
            elif self.manual_test:
                reason = self.manual_deadline_failure(now)
                if reason:
                    self.stop(reason, True)
        if self.stop_time is not None and not self.stop_confirmed:
            if now - self.last_disable >= 0.04:
                self.disable()
            self.confirmed_since = (self.confirmed_since or now) if self.disabled(True) else None
            self.stop_confirmed = self.confirmed_since is not None and now - self.confirmed_since >= 0.1
        if now - self.last_status >= 0.05:
            self.last_status = now
            try:
                os.write(sys.stdout.fileno(), (json.dumps(clean(self.status()), allow_nan=False) + '\n').encode())
            except (BrokenPipeError, BlockingIOError):
                pass
            if not self.logging_failed:
                try:
                    self.events.flush()
                    self.feedback_file.flush()
                except OSError as error:
                    self.logging_failed = True
                    self.stop('recording_flush_failed: ' + str(error), True)
        if self.parent_closed and self.stop_confirmed:
            self.done = True

    def recover_callbacks(self, error):
        self.stop('guard_callback_exception: ' + repr(error), True)
        for subscription in self.observers:
            self.destroy_subscription(subscription)
        self.observers.clear()
        self.attach_observers(minimal=True)

    def close(self):
        (self.directory / 'monitor.json').write_text(json.dumps(clean(self.status()), indent=2, allow_nan=False) + '\n')
        self.events.close()
        self.feedback_file.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    validate(config)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node, interrupted = Guard(config), False
    def signal_handler(signum, frame):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        while rclpy.ok() and not node.done:
            if interrupted:
                if node.armed:
                    node.stop('guard_signal', True)
                    if node.stop_confirmed:
                        node.done = True
                else:
                    node.done = True
            try:
                rclpy.spin_once(node, timeout_sec=0.002)
                node.tick()
            except Exception as error:
                node.recover_callbacks(error)
                if not node.armed:
                    node.done = True
    finally:
        if node.armed and not node.stop_confirmed:
            node.disable()
        try:
            node.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == '__main__':
    sys.exit(main())
