"""Production controller START gate with fake states in a private loopback domain.

No driver, CAN process, transport, or plant is created. simulated=False is only
a scripted identity bit. Lease requests are recorded without granting them.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dm_motor_interfaces.msg import (
    BilateralState, MotorFeedback, MotorReply, MotorRequest, MotorSetpoint, MotorState,
)
from std_srvs.srv import Trigger

from bilateral_fixture import configure_test_environment


def test_differential_start_requires_simulation_or_explicit_hardware_permission(tmp_path):
    binary = os.environ.get('DM_BILATERAL_TEST_NODE')
    assert binary and Path(binary).is_file(), 'Set the production candidate executable explicitly'
    binary = str(Path(binary).resolve())
    configure_test_environment(tmp_path)
    config = Path(__file__).resolve().parents[3] / 'config/dm_motors.yaml'
    parameters = tmp_path / 'empty_calibration.json'
    parameters.write_text(json.dumps({'version': 1, 'motors': {}}))
    evidence = dict(fake_messages_only=True, physical_performance_test=False,
                    domain=int(os.environ['ROS_DOMAIN_ID']),
                    localhost_only=os.environ['ROS_LOCALHOST_ONLY'] == '1',
                    fastdds_profile=os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'],
                    binary=binary, binary_sha256=hashlib.sha256(Path(binary).read_bytes()).hexdigest(), runs=[])
    rclpy.init()
    node = Node('differential_start_fixture', enable_rosout=False, start_parameter_services=False)
    reliable = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    best_effort = QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT)

    def exercise(case, simulated, allow_hardware=False):
        namespace = f'/differential_gate_{os.getpid()}_{case}'
        axes = [namespace + '/a', namespace + '/b']
        states, requests, targets = [], [], []
        state_pub = [node.create_publisher(MotorState, axis + '/state', state_qos) for axis in axes]
        feedback_pub = [node.create_publisher(MotorFeedback, axis + '/feedback', reliable) for axis in axes]
        # Discovery is required by the production commands_ready gate, but no
        # replies are published: this fixture intentionally never grants a lease.
        reply_pub = [node.create_publisher(MotorReply, axis + '/control_result', reliable) for axis in axes]
        subscriptions = [node.create_subscription(BilateralState, namespace + '/controller/state',
            lambda msg: states.append((time.monotonic(), msg)), reliable)]
        for i, axis in enumerate(axes):
            subscriptions.append(node.create_subscription(MotorRequest, axis + '/control',
                lambda msg, i=i: requests.append(dict(time=time.monotonic(), axis=i,
                    kind=int(msg.kind), operation=int(msg.operation))), reliable))
            subscriptions.append(node.create_subscription(MotorSetpoint, axis + '/setpoint',
                lambda msg, i=i: targets.append((i, msg)), best_effort))
        start = node.create_client(Trigger, namespace + '/controller/start')
        stop = node.create_client(Trigger, namespace + '/controller/stop')
        log_path = tmp_path / f'controller-{case}.log'
        command = [binary, '--ros-args', '-r', '__node:=controller', '-r', '__ns:=' + namespace]
        overrides = dict(config_file=str(config), system_parameters_file=str(parameters),
            motor_a=axes[0], motor_b=axes[1], control_mode='differential_torque',
            required_velocity_bandwidth='0.0',
            friction_compensation_enabled='false', rate='500.0', stiffness='50.0',
            coupling_damping='0.4975', local_damping='0.005', max_torque='0.0')
        if allow_hardware:
            overrides['allow_differential_hardware'] = 'true'
        for key, value in overrides.items():
            command += ['-p', key + ':=' + value]
        run = dict(case=case, simulated=simulated, allow_hardware=allow_hardware,
                   permission_explicit=allow_hardware, command=command, requests=requests)
        evidence['runs'].append(run)
        log = log_path.open('w')
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        count, next_feedback, next_state = 0, 0., 0.

        def pump(shutdown=False):
            nonlocal count, next_feedback, next_state
            if not shutdown:
                assert child.poll() is None, log_path.read_text()
            now = time.monotonic()
            if now >= next_feedback:
                next_feedback = now + .002
                count += 1
                for i, publisher in enumerate(feedback_pub):
                    msg = MotorFeedback()
                    msg.motor_id, msg.feedback_count = i+1, count
                    msg.receive_time, msg.absolute_position_valid = now-.001, True
                    msg.mos_temperature = msg.rotor_temperature = 25
                    publisher.publish(msg)
            if now >= next_state:
                next_state = now + .01
                for i, publisher in enumerate(state_pub):
                    msg = MotorState()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.motor_id, msg.master_id, msg.mode = i+1, 0x11+i, 1
                    msg.ready = msg.feedback_fresh = msg.absolute_position_valid = True
                    msg.simulated, msg.phase, msg.status = simulated, MotorState.DISABLED, 0
                    msg.feedback_count, msg.feedback_age, msg.feedback_receive_time = count, .001, now-.001
                    msg.pmax = msg.position_limit = 12.5
                    msg.vmax = msg.velocity_limit = 50.
                    msg.tmax = msg.torque_limit = 10.
                    msg.kp_limit, msg.kd_limit = 500., 1.
                    msg.direction, msg.position_min, msg.position_max = 1., -12.5, 12.5
                    msg.control_rate = msg.actual_control_rate = 500.
                    msg.mos_temperature = msg.rotor_temperature = 25
                    msg.activity_available = True
                    publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=.0005)

        def wait(predicate, timeout=4.):
            deadline = time.monotonic() + timeout
            while not predicate():
                assert time.monotonic() < deadline, str(states[-1][1] if states else '') + log_path.read_text()
                pump()

        def hold(duration):
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                pump()

        def call(client):
            future = client.call_async(Trigger.Request())
            wait(future.done)
            return future.result()

        def enables():
            return [r for r in requests if r['kind'] == MotorRequest.OPERATION and r['operation'] == MotorRequest.ENABLE]

        try:
            wait(lambda: start.service_is_ready() and stop.service_is_ready() and states and
                 all(p.get_subscription_count() for p in state_pub + feedback_pub) and
                 states[-1][1].commands_ready and states[-1][1].paired)
            hold(.05)
            run['start_requested'] = time.monotonic()
            response = call(start)
            run.update(start_success=response.success, start_message=response.message)
            if simulated or allow_hardware:
                assert response.success, response.message
                wait(lambda: any(s.phase == BilateralState.ACQUIRING for _, s in states) and
                     {r['axis'] for r in requests if r['kind'] == MotorRequest.LEASE and
                      r['operation'] == MotorRequest.ACQUIRE} == {0, 1})
                assert not enables(), 'A lease was never granted; ENABLE must not be sent'
                run['acquiring_observed'] = True
            else:
                assert not response.success and 'allow_differential_hardware' in response.message
                hold(.15)
                assert not any(r['kind'] == MotorRequest.LEASE for r in requests), requests
                assert not enables(), requests
                assert not any(s.phase == BilateralState.ACQUIRING for _, s in states)
                run['refused_before_lease_or_enable'] = True
            run['start_checks_finished'] = time.monotonic()
            assert call(stop).success
            stop_response_ns = node.get_clock().now().nanoseconds
            wait(lambda: states[-1][1].phase == BilateralState.IDLE and states[-1][1].stop_confirmed and
                 states[-1][1].header.stamp.sec*1_000_000_000 + states[-1][1].header.stamp.nanosec > stop_response_ns)
            assert not enables(), requests
            assert not targets, 'No lease was granted, so no target stream is permitted'
            run['stop_confirmed'] = True
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                deadline = time.monotonic() + 4
                while child.poll() is None and time.monotonic() < deadline:
                    pump(shutdown=True)
                if child.poll() is None:
                    child.kill()
                child.wait(timeout=3)
            run['exit_code'] = child.returncode
            run['feedback_published_per_axis'] = count
            run['last_published_feedback_clock'] = next_feedback - .002
            run['last_published_state_clock'] = next_state - .01
            run['fixture_end_clock'] = time.monotonic()
            run['phases'] = [int(msg.phase) for _, msg in states]
            run['enable_request_count'] = len(enables())
            run['target_count'] = len(targets)
            log.close()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            for publisher in state_pub + feedback_pub + reply_pub:
                node.destroy_publisher(publisher)
            node.destroy_client(start)
            node.destroy_client(stop)

    try:
        for index, (case, simulated, permission) in enumerate((
                ('real_default', False, False), ('real_authorized', False, True),
                ('simulated_default', True, False))):
            if index:
                # Do not reuse destroyed subscriptions/wait sets across controllers.
                node.destroy_node()
                node = Node('differential_start_fixture_' + case, enable_rosout=False,
                            start_parameter_services=False)
            exercise(case, simulated, permission)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        output = os.environ.get('DM_DIFFERENTIAL_TEST_OUTPUT')
        if output:
            Path(output).write_text(json.dumps(evidence, indent=2) + '\n')
