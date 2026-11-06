"""Actual controller STOP regression using fake MotorState only, no motor or CAN node."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dm_motor_interfaces.msg import MotorState, BilateralState
from std_srvs.srv import Trigger

from bilateral_fixture import configure_test_environment


def test_stop_survives_driver_restart_and_rejects_old_or_active_feedback(tmp_path):
    candidate = os.environ.get('DM_BILATERAL_TEST_NODE')
    assert candidate and Path(candidate).is_file(), 'Test must name the candidate executable'
    before = os.environ.get('DM_BILATERAL_BEFORE_NODE')
    configure_test_environment(tmp_path)
    logs = tmp_path / 'ros-logs'; logs.mkdir()
    os.environ['ROS_LOG_DIR'] = str(logs)
    rclpy.init()
    node = Node('stop_counter_reset_probe', enable_rosout=False, start_parameter_services=False)
    evidence = {'domain': os.environ['ROS_DOMAIN_ID'], 'fake_states_only': True, 'runs': []}
    config = Path(__file__).resolve().parents[3] / 'config' / 'dm_motors.yaml'
    qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def exercise(binary, expect_reset_pass, index):
        namespace = f'/stop_counter_reset_{os.getpid()}_{index}'
        axes = [namespace+'/a', namespace+'/b']
        # Match motor_node's latest-state channel; a deep writer history can
        # retain samples beyond feedback_stop while waiting for UDP acknowledgments.
        publishers = [node.create_publisher(MotorState, a+'/state', state_qos) for a in axes]
        states = []; mode = 'old_counter'; acquisition = None; next_publish = 0.
        subscription = node.create_subscription(BilateralState, namespace+'/controller/state',
            lambda msg: states.append((time.monotonic(), msg.phase, msg.stop_confirmed)), qos)
        client = node.create_client(Trigger, namespace+'/controller/stop')
        log = (tmp_path/f'controller-{index}.log').open('w')
        child = subprocess.Popen([binary, '--ros-args', '-r', '__node:=controller', '-r', '__ns:='+namespace,
            '-p', 'config_file:='+str(config), '-p', 'motor_a:='+axes[0], '-p', 'motor_b:='+axes[1],
            '-p', 'friction_compensation_enabled:=false'], stdout=log, stderr=subprocess.STDOUT)
        run = dict(binary=binary, expected_counter_reset_pass=expect_reset_pass, checks=[])
        evidence['runs'].append(run)

        def pump():
            nonlocal next_publish
            assert child.poll() is None, 'Controller exited; inspect '+str(log.name)
            now = time.monotonic()
            if now >= next_publish:
                for publisher in publishers:
                    state = MotorState(); state.header.stamp = node.get_clock().now().to_msg()
                    state.phase = MotorState.ENABLED if mode == 'active' else MotorState.DISABLED
                    state.status = 1 if mode == 'active' else 0
                    state.feedback_count = 1000000 if mode == 'old_counter' else 1
                    state.feedback_fresh = mode != 'stale'
                    state.feedback_age = .2 if mode == 'stale' else .001
                    state.feedback_receive_time = (acquisition if mode == 'old_acquisition' else now-state.feedback_age)
                    state.ready = True; state.pmax = 12.5; state.vmax = 50.; state.tmax = 10.
                    state.simulated = True
                    publisher.publish(state)
                next_publish = now+.005
            rclpy.spin_once(node, timeout_sec=.001)

        def wait(predicate, seconds=4):
            deadline = time.monotonic()+seconds
            while not predicate():
                assert time.monotonic() < deadline, 'Timed out awaiting controller state'
                pump()

        def hold(seconds):
            until = time.monotonic()+seconds
            while time.monotonic() < until: pump()

        def stop():
            requested = time.monotonic(); future = client.call_async(Trigger.Request())
            wait(future.done); assert future.result().success
            wait(lambda: states and states[-1][1] == BilateralState.STOPPING)
            return requested, len(states)

        try:
            # DDS endpoint discovery does not prove that both MotorState streams
            # have reached the controller. Establish the old-counter baseline
            # from its own fresh-disabled confirmation before requesting STOP.
            wait(lambda: client.service_is_ready() and all(p.get_subscription_count() for p in publishers)
                 and states and states[-1][1] == BilateralState.IDLE and states[-1][2])
            hold(.15)  # Ensure the controller has received the large previous-process counter.
            requested, marker = stop(); mode = 'fresh_reset'
            hold(.85)
            observed = states[marker:]
            idle = [t for t,phase,confirmed in observed if phase == BilateralState.IDLE and confirmed]
            if expect_reset_pass:
                assert idle and idle[0]-requested >= .6
            else:
                assert not idle and states[-1][1] == BilateralState.STOPPING
            run['checks'].append(dict(kind='counter_reset', old_count=1000000, new_count=1,
                stop_requested=requested, first_confirmed_idle=idle[0] if idle else None,
                final_phase=states[-1][1], elapsed_observation=time.monotonic()-requested))
            if expect_reset_pass:
                for invalid in ('old_acquisition', 'stale', 'active'):
                    mode = 'old_counter'; hold(.1)
                    requested, marker = stop(); acquisition = requested-.01; mode = invalid
                    hold(.85)
                    assert not any(phase == BilateralState.IDLE for _,phase,_ in states[marker:])
                    assert states[-1][1] == BilateralState.STOPPING
                    mode = 'fresh_reset'
                    wait(lambda: states[-1][1] == BilateralState.IDLE and states[-1][2])
                    run['checks'].append(dict(kind=invalid, rejected_for_at_least_seconds=.85,
                        stop_requested=requested, confirmed_after_fresh=time.monotonic()))
            run['phase_changes'] = [states[0]] + [states[i] for i in range(1,len(states)) if states[i][1:] != states[i-1][1:]]
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
            run['shutdown_exitcode'] = child.returncode
            log.close(); node.destroy_subscription(subscription); node.destroy_client(client)
            for publisher in publishers: node.destroy_publisher(publisher)

    try:
        if before: exercise(before, False, 0)
        exercise(candidate, True, 1)
    finally:
        node.destroy_node(); rclpy.shutdown()
        output = os.environ.get('DM_STOP_TEST_OUTPUT')
        if output: Path(output).write_text(json.dumps(evidence,indent=2)+'\n')
