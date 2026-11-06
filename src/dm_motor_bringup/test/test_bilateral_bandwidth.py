"""Production START V_BW readback on isolated virtual devices only.

This tests register policy, not a physical relationship between V_BW and simulated lag.
"""
import copy
import json
import os
from pathlib import Path
import shutil
import struct
import time

import test_bilateral_shared_reference as _shared
from dm_motor_interfaces.msg import BilateralState, MotorReply, MotorRequest, MotorState
from rcl_interfaces.srv import DescribeParameters, SetParametersAtomically
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


def raw_float(value):
    return struct.unpack('<I', struct.pack('<f', value))[0]


def decoded_float(raw):
    return struct.unpack('<f', struct.pack('<I', raw))[0]


LATE_REPLY_LAUNCH = '''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common.config import ConfigManager
from pathlib import Path

def scene(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    config = ConfigManager(value('config_file'))
    profile = ('dm_motor_bringup', 'bilateral_control.launch.py')
    parameters = config.node_parameters('dm_motor_bilateral_control', 'bilateral_control_node',
                                       profile, variant='simulation')
    parameters.update(config_file=str(config.path), motor_a='motor_1', motor_b='motor_2')
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
        get_package_share_directory('dm_motor_bringup')) / 'launch/dual_motor.launch.py')),
        launch_arguments=dict(config_file=value('config_file'), simulation='true',
            namespace=value('namespace'), config_package=profile[0], config_launch=profile[1]).items()),
        Node(package='dm_motor_bilateral_control', executable='bilateral_control_node',
             name='bilateral_control', namespace=value('namespace'), output='screen',
             parameters=[parameters],
             remappings=[('motor_2/control_result', 'bandwidth_proxy/motor_2_result')])]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file'), DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('simulation', default_value='true'), OpaqueFunction(function=scene)])
'''


class BilateralBandwidthIntegration(_shared.BilateralSharedReferenceIntegration):
    test_positive_30_whole_motion_both_axes_and_fresh_stop = None
    test_driver_capability_rejects_unscaled_native_gain_before_enable = None
    test_shared_angle_expiry_and_explicit_stop_never_replay = None

    def configure_scene(self, document, profile):
        super().configure_scene(document, profile)
        profile['nodes']['dm_motor_bilateral_control']['bilateral_control_node'][
            'required_velocity_bandwidth'] = 100.0
        profile['nodes']['dm_motor_control']['motor_node']['startup_velocity_bandwidth'] = 100.0
        path = Path(self.directory.name) / 'fixture.json'
        fixture = json.loads(path.read_text())
        fixture.update(controller=profile['nodes']['dm_motor_bilateral_control']['bilateral_control_node'],
                       startup_velocity_bandwidth=100.0,
                       purpose='Register/START lifecycle only; no angle or pulse. V_BW does not retune virtual physics.')
        path.write_text(json.dumps(fixture, indent=2) + '\n')

    def start(self, name, command):
        if name == 'system' and 'late_reply' in self._testMethodName:
            launcher = Path(self.directory.name) / 'bandwidth_proxy.launch.py'
            launcher.write_text(LATE_REPLY_LAUNCH)
            command = ['ros2', 'launch', str(launcher), *command[4:]]
        return super().start(name, command)

    def setUp(self):
        self.held_replies, self.gate_events = [], []
        self.hold_readback = False
        super().setUp()
        self.assertTrue(self.namespace.startswith('/physics_test_'))
        self.assertTrue(all(s.simulated for s in self.states.values()))
        self.assertTrue(all(s.virtual_bus_confirmed for s in self.physics.values()))
        self.proxy = None
        if 'late_reply' in self._testMethodName:
            self.proxy = self.node.create_publisher(MotorReply, 'bandwidth_proxy/motor_2_result', 256)
        for axis in ('motor_1', 'motor_2'):
            self.subscriptions.append(self.node.create_subscription(MotorReply, axis + '/control_result',
                lambda message, axis=axis: self.observe_reply(axis, message), 256))
        self.assertEqual(self.follow.required_velocity_bandwidth, 100.)

    def observe_request(self, axis, message):
        super().observe_request(axis, message)
        if self.capture() is not None:
            self.capture()['requests'][-1].update(client_id=message.client_id, request_id=message.request_id,
                source_time=self.source_time(message.header), kind=message.kind, write=message.write,
                addresses=list(message.addresses), raw_values=list(message.raw_values), operation_id=message.operation_id)

    def observe_motor(self, axis, message):
        super().observe_motor(axis, message)
        if self.capture() is not None:
            self.capture()['motors'][-1].update(ready=message.ready,
                activity_available=message.activity_available, maintenance_available=message.maintenance_available)

    def on_follow(self, message):
        super().on_follow(message)
        if self.capture() is not None:
            self.capture()['controller'][-1].update(message=message.message,
                required_velocity_bandwidth=message.required_velocity_bandwidth,
                velocity_bandwidth_verified=message.velocity_bandwidth_verified,
                motor_a_velocity_bandwidth=message.motor_a_velocity_bandwidth,
                motor_b_velocity_bandwidth=message.motor_b_velocity_bandwidth)

    def observe_reply(self, axis, message):
        if self.capture() is not None:
            self.capture().setdefault('replies', []).append(dict(axis=axis, host_time=time.monotonic(),
                source_time=self.source_time(message.header), client_id=message.client_id,
                request_id=message.request_id, kind=message.kind, success=message.success,
                operation_id=message.operation_id, operation_ids=list(message.operation_ids),
                state=message.state, address=message.address, raw_value=message.raw_value))
        if self.proxy is None or axis != 'motor_2':
            return
        if (self.hold_readback and self.follow is not None and message.client_id == self.follow.controller_source and
                message.kind == MotorRequest.OPERATION_RESULT and message.state == MotorReply.CONFIRMED and
                message.address == 32):
            self.held_replies.append(copy.deepcopy(message))
            self.gate_events.append(dict(event='held_original_reply', time=time.monotonic(),
                client_id=message.client_id, request_id=message.request_id, operation_id=message.operation_id,
                raw_value=message.raw_value, source_time=self.source_time(message.header)))
        else:
            self.proxy.publish(message)

    def export_artifacts(self):
        super().export_artifacts()
        output = os.environ.get('DM_BILATERAL_AUTO_TEST_OUTPUT')
        if output:
            destination = Path(output) / self._testMethodName
            shutil.copy2(Path(__file__), destination / Path(__file__).name)
            (destination / 'reply-gate.json').write_text(json.dumps(self.gate_events, indent=2) + '\n')

    def register32(self, axis, value=None):
        self.wait(lambda: self.states[axis].maintenance_available)
        values = dict(write=False, addresses=[32]) if value is None else dict(
            write=True, addresses=[32], raw_values=[raw_float(value)])
        reply = self.call(axis, 'registers', MotorRequest(**values))
        self.assertEqual(len(reply.operation_ids), 1)
        result = self.operation_result(axis, reply.operation_ids[0])
        self.assertEqual(result.address, 32)
        self.assertEqual(result.state, MotorReply.CONFIRMED)
        if value is not None:
            self.assertEqual(result.raw_value, raw_float(value))
            return self.register32(axis)  # Independent post-write readback.
        return decoded_float(result.raw_value)

    def stop_fresh(self):
        marker = time.monotonic()
        counts = {name: state.feedback_count for name, state in self.states.items()}
        self.operator_session, self.operator_future = '', None
        self.service(self.stop_client, Trigger.Request())
        self.wait(lambda: self.follow.stop_confirmed and all(state.feedback_fresh and state.status == 0 and
            state.phase == MotorState.DISABLED and state.feedback_count > counts[name] and
            state.feedback_receive_time > marker for name, state in self.states.items()), timeout=3)
        self.settle(.12)
        self.assert_fresh_stop_evidence(self.current_capture, marker)

    def confirmed_reads(self, name):
        source = self.follow.controller_source
        capture = self.captures[name]
        requests = {(r['axis'], r['client_id'], r['request_id']): r for r in capture['requests']}
        queued = {(r['axis'], operation) for r in capture.get('replies', [])
            if r['client_id'] == source and r['kind'] == MotorRequest.REGISTERS and r['success']
            for operation in r['operation_ids']}
        confirmed = [dict(r) for r in capture.get('replies', []) if r['client_id'] == source and
            r['kind'] == MotorRequest.OPERATION_RESULT and r['success'] and
            r['state'] == MotorReply.CONFIRMED and r['address'] == 32]
        for reply in confirmed:
            request = requests[(reply['axis'], reply['client_id'], reply['request_id'])]
            self.assertEqual(request['kind'], MotorRequest.OPERATION_RESULT)
            # OPERATION_RESULT leaves reply.operation_id at0. The exact matched
            # request carries the polled ID; retain the raw reply value as well.
            reply['polled_operation_id'] = request['operation_id']
            self.assertGreater(reply['polled_operation_id'], 0)
            self.assertIn((reply['axis'], reply['polled_operation_id']), queued)
        return confirmed

    def assert_verified_before_enable(self, name):
        capture, source = self.captures[name], self.follow.controller_source
        requests = [r for r in capture['requests'] if r['client_id'] == source]
        reads = [r for r in requests if r['kind'] == MotorRequest.REGISTERS and 32 in r['addresses']]
        self.assertEqual({r['axis'] for r in reads}, {'motor_1', 'motor_2'})
        self.assertTrue(all(not r['write'] and r['addresses'] == [32] for r in reads))
        confirmed = self.confirmed_reads(name)
        self.assertEqual({r['axis'] for r in confirmed}, {'motor_1', 'motor_2'})
        self.assertTrue(all(r['raw_value'] == raw_float(100.) for r in confirmed))
        enabled = [r for r in requests if r['kind'] == MotorRequest.OPERATION and r['operation'] == MotorRequest.ENABLE]
        self.assertEqual({r['axis'] for r in enabled}, {'motor_1', 'motor_2'})
        first_enable = min(r['source_time'] for r in enabled)
        read_barrier = max(r['source_time'] for r in confirmed)
        self.assertLess(read_barrier, first_enable)
        post_read_samples = {}
        for axis in ('motor_1', 'motor_2'):
            rows = [r for r in capture['motors'] if r['axis'] == axis and read_barrier < r['receive_time'] < first_enable and
                r['phase'] == MotorState.DISABLED and r['fresh'] and r['status'] == 0 and not r['target_active'] and
                r['ready'] and r['activity_available']]
            self.assertTrue(rows, (axis, read_barrier, first_enable))
            post_read_samples[axis] = rows[0]['feedback_count']
        active = [r for r in capture['controller'] if r['phase'] == BilateralState.ACTIVE]
        self.assertTrue(active)
        self.assertTrue(all(r['velocity_bandwidth_verified'] and r['required_velocity_bandwidth'] == 100. and
            r['motor_a_velocity_bandwidth'] == 100. and r['motor_b_velocity_bandwidth'] == 100. for r in active))
        capture['bandwidth_proof'] = dict(read_barrier=read_barrier, first_enable=first_enable,
            post_read_disabled_counts=post_read_samples, confirmed_operations=confirmed)
        self.assert_no_coordinate_writes(name)
        self.assertFalse(any(r['kind'] == MotorRequest.OPERATION and r['operation'] == MotorRequest.SAVE_PARAMETERS for r in requests))
        return {(r['axis'], r['polled_operation_id']) for r in confirmed}

    def reject_bandwidth_start(self):
        previous_commands = self.follow.command_count
        self.service(self.start_client, Trigger.Request())
        self.wait(lambda: self.follow.phase == BilateralState.FAULT and self.follow.stop_confirmed, timeout=4)
        self.settle(.1)
        self.assertEqual(self.follow.command_count, previous_commands)
        self.assertIn('Actual V_BW=40', self.follow.message)
        self.assertIn('required 100', self.follow.message)
        self.assert_never_enabled(self.current_capture)

    def cleanup_stop(self):
        self.operator_session, self.operator_future = '', None
        if not (self.follow.stop_confirmed and self.follow.phase in (BilateralState.IDLE, BilateralState.FAULT) and
                all(s.feedback_fresh and s.phase == MotorState.DISABLED and s.status == 0 for s in self.states.values())):
            self.stop_fresh()

    def test_fresh_bandwidth_read_each_start_rejects_ram_mismatch(self):
        try:
            descriptor = self.node.create_client(DescribeParameters, 'bilateral_control/describe_parameters')
            reply = self.service(descriptor, DescribeParameters.Request(names=['required_velocity_bandwidth']))
            self.assertTrue(reply.descriptors[0].read_only)
            reply = self.service(self.parameter_client, SetParametersAtomically.Request(parameters=[
                Parameter('required_velocity_bandwidth', value=40.).to_parameter_msg()]))
            self.assertFalse(reply.result.successful)
            self.assertEqual(self.register32('motor_1'), 100.)
            self.assertEqual(self.register32('motor_2'), 100.)
            self.begin_capture('first100')
            self.guarded_start()
            self.settle(.15)
            first_ids = self.assert_verified_before_enable('first100')
            self.stop_fresh()
            self.assertEqual(self.register32('motor_1'), 100.)
            self.assertEqual(self.register32('motor_2'), 100.)  # Ordinary STOP must not restore40.
            self.assertEqual(self.register32('motor_1', 40.), 40.)
            self.begin_capture('reject40')
            self.reject_bandwidth_start()
            self.assert_never_enabled('reject40')
            self.assertFalse(self.follow.velocity_bandwidth_verified)
            self.assertIn(raw_float(40.), [r['raw_value'] for r in self.confirmed_reads('reject40')])
            self.assertEqual(self.register32('motor_1', 100.), 100.)
            self.begin_capture('fresh100')
            self.guarded_start()
            self.settle(.15)
            second_ids = self.assert_verified_before_enable('fresh100')
            self.assertTrue(first_ids.isdisjoint(second_ids), 'Each START must create new register operations')
            self.stop_fresh()
        finally:
            self.cleanup_stop()

    def test_stop_during_read_late_reply_cannot_resume(self):
        try:
            self.begin_capture('cancel_read')
            self.hold_readback = True
            self.service(self.start_client, Trigger.Request())
            self.wait(lambda: bool(self.held_replies) and self.follow.phase == BilateralState.ACQUIRING, timeout=3)
            self.assertEqual(self.follow.phase, BilateralState.ACQUIRING)
            self.assertFalse(self.follow.velocity_bandwidth_verified)
            marker = time.monotonic()
            self.service(self.stop_client, Trigger.Request())
            self.wait(lambda: self.follow.phase == BilateralState.STOPPING, timeout=.2)
            released = time.monotonic()
            self.assertLess(released - self.gate_events[0]['source_time'], .2,
                            'Release must be still timely; a naturally expired reply is not the intended test')
            self.hold_readback = False
            for message in self.held_replies:
                self.proxy.publish(message)  # Identical original header/IDs/value, only delayed delivery.
            self.gate_events.append(dict(event='released_after_STOP', time=released,
                                         stop_requested=marker, count=len(self.held_replies)))
            self.wait_fresh_disabled()
            self.settle(.3)
            self.assert_never_enabled('cancel_read')
            self.assertFalse(self.follow.velocity_bandwidth_verified)
            self.assert_fresh_stop_evidence('cancel_read', marker)
            stale_ids = {(r['axis'], r['polled_operation_id']) for r in self.confirmed_reads('cancel_read')}
            self.begin_capture('explicit_new_start')
            self.guarded_start()
            self.settle(.15)
            new_ids = self.assert_verified_before_enable('explicit_new_start')
            self.assertTrue(stale_ids.isdisjoint(new_ids))
            self.stop_fresh()
        finally:
            self.hold_readback = False
            self.cleanup_stop()
