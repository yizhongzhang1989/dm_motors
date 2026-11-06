"""Offline boundary, wrong-axis and trajectory checks; never initialize ROS."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("independent_axis_runner", HERE / "run_axis_sweep.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.fixture
def config(tmp_path):
    registry = tmp_path / "registry.yaml"
    registry.write_text("schema: 1\n")
    return dict(schema=1, output=str(tmp_path / "trial"), axis=1, speed=13.2, direction=-1,
                role="training", repeat=1, native_binary=str(tmp_path / "not-built"),
                env={"PATH": "/usr/bin:/bin"}, expected_bandwidth_hz=100.,
                guards=dict(speed=15.9, torque=3.8, travel=20.1, temperature=48.),
                files=dict(system_parameters_file=str(registry), config_file=str(registry)))


def cli(config, tmp_path, *extra):
    path = tmp_path / "input.json"
    path.write_text(json.dumps(config))
    return subprocess.run([sys.executable, "-S", str(HERE / "run_axis_sweep.py"),
                           "--config", str(path), *extra], text=True, capture_output=True, timeout=5)


def test_plan_works_without_ros_or_binary_and_creates_no_output(config, tmp_path):
    result = cli(config, tmp_path)
    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)
    assert not plan["execute"]
    assert plan["gains"] == {"kp": 10., "kd": .6, "torque": 0.}
    assert plan["geometry"]["duration"] == pytest.approx(5.77)
    assert not Path(config["output"]).exists()


@pytest.mark.parametrize("key,value", [("axis", 2), ("axis", True), ("direction", 0),
    ("direction", 1.0), ("speed", float("nan")), ("speed", 13.4), ("speed", .1),
    ("role", "mixed"), ("repeat", True), ("expected_bandwidth_hz", 500),
    ("expected_bandwidth_hz", 499.999999), ("env", {"PATH": None})])
def test_bad_plan_is_rejected_before_ros_or_output(config, tmp_path, key, value):
    config[key] = value
    result = cli(config, tmp_path)
    assert result.returncode == 2
    assert "rclpy" not in result.stderr
    assert not Path(config["output"]).exists()


def test_guard_bounds_cannot_be_relaxed_past_supported_limits(config):
    for key, value in [("speed", 13.2), ("speed", 19), ("torque", 5.1),
                       ("travel", 19.5), ("travel", 26), ("temperature", 49)]:
        changed = copy.deepcopy(config)
        changed["guards"][key] = value
        with pytest.raises(ValueError):
            runner.validate_config(changed)


@pytest.mark.parametrize("age", [0., .5, 2., 7., 7.999999])
def test_native_graph_ready_does_not_allow_early_enable(age):
    assert runner.native_start_ready(100., 100. + age) is False


@pytest.mark.parametrize("age", [8., 8.000001, 9., 9.999999])
def test_native_discovery_window_is_absolute_from_launch(age):
    assert runner.native_start_ready(100., 100. + age) is True


@pytest.mark.parametrize("age", [10., 10.000001, 12., 18.])
def test_native_expired_go_window_cannot_enable(age):
    with pytest.raises(TimeoutError, match="pre-enable GO deadline expired"):
        runner.native_start_ready(100., 100. + age)


def test_native_discovery_waits_with_pump_and_never_uses_ready_as_new_origin():
    now, observations = 100.5, []
    while not runner.native_start_ready(100., now):
        observations.append(now)  # Represents the guarded pump, not a blocking sleep.
        now += .5
    assert observations[0] == 100.5
    assert observations[-1] == 107.5
    assert now == 108.
    # A late READY is evaluated against launch+10, not granted another 8 s.
    with pytest.raises(TimeoutError):
        runner.native_start_ready(100., 110.)


def test_existing_trial_is_preserved(config, tmp_path):
    output = Path(config["output"])
    output.mkdir()
    sentinel = output / "keep.txt"
    sentinel.write_text("untouched\n")
    result = cli(config, tmp_path)
    assert result.returncode == 2
    assert sentinel.read_text() == "untouched\n"


def test_wrong_native_build_fails_before_ros_or_output(config, tmp_path):
    binary = tmp_path / "fake-native"
    binary.write_text('#!/bin/sh\nprintf \'{"source_sha256":"wrong","cmake_sha256":"wrong"}\\n\'\n')
    binary.chmod(0o700)
    config["native_binary"] = str(binary)
    result = cli(config, tmp_path, "--execute")
    assert result.returncode == 2
    assert "build does not match" in result.stderr
    assert "rclpy" not in result.stderr
    assert not Path(config["output"]).exists()


def test_only_completed_matching_vbw_register_can_confirm_readback():
    raw = struct.unpack("<I", struct.pack("<f", 40.))[0]
    assert runner.decode_bandwidth_result(0, 0, 0) is None
    assert runner.decode_bandwidth_result(1, 32, raw) == 40.
    for state, address, value in [(1, 31, raw), (2, 32, raw), (1, 32, 0), (1, 32, 0x7fc00000)]:
        with pytest.raises(ValueError):
            runner.decode_bandwidth_result(state, address, value)


def controller():
    return SimpleNamespace(phase=0, stop_confirmed=True, command_count=123,
        disturbance_active=False, angle_offset_active=False, angle_offset_ramping=False,
        angle_offset_applied=0., disturbance_torque=0., disturbance_torque_a=0., disturbance_torque_b=0.)


def test_idle_controller_device_stop_flag_changes_when_independent_axis_is_enabled():
    b = controller()
    runner.check_controller(b, mode="disabled", command_count=123)
    b.stop_confirmed = False
    for mode in ("enabling", "active"):
        runner.check_controller(b, mode=mode, command_count=123)
    with pytest.raises(RuntimeError, match="STOP is unconfirmed"):
        runner.check_controller(b, mode="disabled", command_count=123)


@pytest.mark.parametrize("phase", [1, 2, 3, 4, 5])
@pytest.mark.parametrize("confirmed", [True, False])
def test_nonidle_bilateral_controller_is_rejected_regardless_of_device_stop_flag(phase, confirmed):
    b = controller()
    b.phase, b.stop_confirmed = phase, confirmed
    for mode in ("disabled", "enabling", "active"):
        with pytest.raises(RuntimeError, match="remain IDLE"):
            runner.check_controller(b, mode=mode, command_count=123)


def test_idle_bilateral_command_or_injection_changes_are_rejected():
    for field, value in [("command_count", 124), ("disturbance_active", True),
                         ("angle_offset_active", True), ("angle_offset_ramping", True),
                         ("angle_offset_applied", .1), ("disturbance_torque", .1),
                         ("disturbance_torque_a", .1), ("disturbance_torque_b", -.1)]:
        for mode in ("disabled", "enabling", "active"):
            b = controller()
            setattr(b, field, value)
            with pytest.raises(RuntimeError):
                runner.check_controller(b, mode=mode, command_count=123)


def observations(axis=1):
    state = SimpleNamespace(ready=True, absolute_position_valid=True, simulated=False,
        motor_id=axis, mode=1, host_fault=False, device_fault=False, task_active=False,
        feedback_receive_time=9.99, position=8., velocity=0., torque=0., phase=0, status=0,
        active_source="", target_active=False, rejected_commands=5)
    feedback = SimpleNamespace(absolute_position_valid=True, receive_time=9.99, position=8.,
        velocity=0., torque=0., mos_temperature=25, rotor_temperature=25, target_active=False)
    return state, feedback


def check(state, feedback, *, axis=1, selected=1, mode="disabled"):
    runner.check_axis(state, feedback, axis=axis, origin=8., selected=selected, mode=mode,
        source="native_test_identity", enable_source="enable_test_identity", rejected=5, now=10.,
        guards=dict(speed=15.9, torque=3.8, travel=20.1, temperature=48.))


def test_nonselected_axis_must_stay_disabled_and_stationary():
    state, feedback = observations(3)
    check(state, feedback, axis=3, selected=1, mode="active")
    for changes in [dict(phase=2, status=1), dict(active_source="foreign"), dict(target_active=True),
                    dict(task_active=True), dict(position=8.021)]:
        changed = copy.copy(state)
        for field, value in changes.items():
            setattr(changed, field, value)
        with pytest.raises(RuntimeError):
            check(changed, feedback, axis=3, selected=1, mode="active")
    feedback.position = 8.021
    with pytest.raises(RuntimeError, match="moved"):
        check(state, feedback, axis=3, selected=1, mode="active")


def test_selected_active_requires_exact_source_freshness_and_no_rejection():
    state, feedback = observations()
    state.phase, state.status, state.active_source, state.target_active = 2, 1, "native_test_identity", True
    check(state, feedback, mode="active")
    for field, value in [("active_source", "other"), ("target_active", False), ("rejected_commands", 6),
                         ("feedback_receive_time", 9.8), ("host_fault", True), ("velocity", 16.)]:
        changed = copy.copy(state)
        setattr(changed, field, value)
        with pytest.raises(RuntimeError):
            check(changed, feedback, mode="active")
    feedback.torque = float("nan")
    with pytest.raises(RuntimeError, match="nonfinite"):
        check(state, feedback, mode="active")


def test_enable_accepts_only_own_enable_and_target_sources_during_transition():
    state, feedback = observations()
    for phase, status in [(0, 0), (1, 0), (2, 1)]:
        state.phase, state.status = phase, status
        for source in ("", "enable_test_identity", "native_test_identity"):
            state.active_source = source
            check(state, feedback, mode="enabling")
        state.active_source = "another_operator"
        with pytest.raises(RuntimeError, match="enable transition"):
            check(state, feedback, mode="enabling")
    state.active_source, state.target_active = "enable_test_identity", True
    with pytest.raises(RuntimeError, match="active source"):
        check(state, feedback, mode="active")
    state.active_source = "native_test_identity"
    check(state, feedback, mode="active")


@pytest.fixture(scope="module")
def native_math(tmp_path_factory):
    directory = tmp_path_factory.mktemp("independent_native_math")
    source = (HERE.parent / "bilateral/native_stream/stream.cpp").read_text()
    body = source[source.index("struct Reference"):source.index("int main")]
    path = directory / "math.cpp"
    path.write_text('#include <cmath>\n#include <algorithm>\n#include <string>\n#include <array>\n'
        '#include <vector>\n#include <iostream>\n#include <iomanip>\n' + body +
        '\nint main(int argc,char**argv){(void)argc;double t=std::stod(argv[1]),q=std::stod(argv[2]),'
        'v=std::stod(argv[3]);int direction=std::stoi(argv[4]);'
        'auto r=directed_reference(t,q,v,"sweep",direction);'
        'std::cout<<std::setprecision(17)<<r.q<<" "<<r.v<<" "<<r.phase<<" "'
        '<<valid_envelope(q,v*(std::max(.25,v/15)+.6),direction);}\n')
    binary = directory / "math"
    subprocess.run(["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                    str(path), "-o", str(binary)], check=True)
    return binary


@pytest.mark.parametrize("q0,direction", [(8.0372251552, -1), (-10.6273107315, 1)])
@pytest.mark.parametrize("speed", [.6, 1.5, 3., 6., 9., 13.2])
def test_native_direction_round_trip_windows_and_envelope(native_math, q0, direction, speed):
    g = runner.geometry(speed)
    envelope = runner.check_envelope(q0, speed, direction)
    points = [0., .3, .3 + g["ramp"] / 2, .3 + g["leg"], g["duration"]]
    points.extend((a + b) / 2 for a, b in g["windows"])
    for t in points:
        result = subprocess.check_output([str(native_math), str(t), str(q0), str(speed), str(direction)], text=True).split()
        q, v, phase, valid = float(result[0]), float(result[1]), result[2], int(result[3])
        assert valid
        assert envelope["minimum"] - 1e-12 <= q <= envelope["maximum"] + 1e-12
        if t in (0., g["duration"]):
            assert q == pytest.approx(q0, abs=1e-12)
            assert v == 0.
        for index, (a, b) in enumerate(g["windows"]):
            if a <= t <= b:
                assert phase == "steady"
                assert v == pytest.approx(speed * direction * (1 if index == 0 else -1))


def test_boundary_touch_or_wrong_starting_direction_cannot_move(native_math):
    for q0, direction in [(8.037, 1), (-10.627, -1), (-12.4, 1), (12.4, -1)]:
        with pytest.raises(ValueError, match="envelope"):
            runner.check_envelope(q0, 13.2, direction)
        result = subprocess.check_output([str(native_math), "0", str(q0), "13.2", str(direction)], text=True).split()
        assert result[-1] == "0"


def test_high_speed_protocol_has_acceleration_and_duration_bounds():
    for speed in [.6, 1.5, 3., 6., 9., 13.2]:
        g = runner.geometry(speed)
        assert speed * math.pi / (2 * g["ramp"]) <= 15 * math.pi / 2 + 1e-12
        assert g["duration"] < 6.
        assert all(b - a == pytest.approx(.3) for a, b in g["windows"])


def test_timed_out_child_is_terminated_then_killed_before_return():
    class StuckProcess:
        def __init__(self):
            self.calls = []

        def wait(self, timeout):
            self.calls.append(("wait", timeout))
            if not any(call[0] == "kill" for call in self.calls):
                raise subprocess.TimeoutExpired("fake", timeout)

        def terminate(self):
            self.calls.append(("terminate",))

        def kill(self):
            self.calls.append(("kill",))

    child = object.__new__(runner.Child)
    child.name, child.process = "fake", StuckProcess()
    with pytest.raises(RuntimeError, match="terminated"):
        child.reap()
    assert child.process.calls == [("wait", 1.), ("terminate",), ("wait", 1.), ("kill",), ("wait", 1.)]
