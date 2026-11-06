"""Bounded low-speed acquisition through the production node and virtual CAN.

This module contains no replacement motion controller.  The shared Scene owns
processes, while CalibrationNode owns targets, guards, returns and disable.
Importing the module does not initialize ROS or start any process.
"""

import csv
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
import rclpy
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
import yaml

from common import ConfigManager
from dm_motor_calibration.dashboard import Dashboard
from dm_motor_calibration.data import Trace
from dm_motor_calibration.report_data import load_report_data
from dm_motor_calibration.ros_node import disabled_feedback
from dm_motor_control.topic_client import motor_topics
from dm_motor_interfaces.msg import MotorRequest
from test_ros_calibration import Scene


REPO = Path(__file__).resolve().parents[3]
PLANT_PROFILE = REPO / "config/bilateral_identification_20261104.yaml"
TERMINAL = ("completed", "completed_unqualified", "failed")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def read_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        assert fields and len(fields) == len(set(fields)), str(path)
        rows = list(reader)
    assert all(None not in row and all(v is not None for v in row.values()) for row in rows), str(path)
    return fields, rows


def configure_scene(scene, axis, *, profile=PLANT_PROFILE):
    """Use the declared Nov4 plant; only the experiment recipe is new."""
    from dm_motor_calibration.low_speed import LowSpeedSettings

    assert axis in ("A", "B")
    candidate = yaml.safe_load(Path(profile).read_text())
    packages = candidate["packages"]
    variant = packages["dm_motor_bringup"]["launches"]["bilateral_control.launch.py"]["variants"]["simulation"]
    physics = dict(packages["dm_motor_control"]["nodes"]["sim_motor_node"])
    physics.update(variant["devices"]["motor_1" if axis == "A" else "motor_2"])
    physics.update(initial_position=0.0, initial_velocity=0.0, persistence_file="")
    scene.config["packages"]["dm_motor_control"]["nodes"]["sim_motor_node"].update(physics)
    driver = dict(variant["nodes"]["dm_motor_control"]["motor_node"])
    driver.update(control_rate=500.0, state_rate=500.0, require_virtual_bus=True,
                  can_bus_namespace=scene.motor + "/can", motor_id=1 if axis == "A" else 2)
    motor_launch = scene.config["packages"]["dm_motor_control"]["launches"]["motor.launch.py"]
    motor_launch["variants"]["simulation"]["nodes"]["dm_motor_control"]["motor_node"].update(driver)
    scene.config["packages"]["can_bus"]["nodes"]["can_bus_node"].update(backend="virtual", port="")
    low = asdict(LowSpeedSettings())
    low["repetitions"] = 1  # Lifecycle coverage only; never model qualification.
    scene.options.update(experiment="low_speed", rate=200.0, target_validity=0.1,
                         allow_hardware=False, allow_multiturn_calibration=False,
                         calibration_file="", quick_excitation="fixed",
                         max_torque=0.4, max_speed=1.0, max_travel=0.7,
                         max_temperature=48.0, feedback_timeout=0.08, max_gap=0.03,
                         collect_warmup=False, collect_static=False,
                         collect_training=True, collect_validation=True,
                         **{"low_speed_" + name: value for name, value in low.items()})
    scene.write_config()
    resolved = ConfigManager(scene.config_path)
    calibration = resolved.node_parameters("dm_motor_calibration", "calibration_node",
                                           ("dm_motor_calibration", "calibration.launch.py"))
    driver = resolved.node_parameters("dm_motor_control", "motor_node",
                                     ("dm_motor_control", "motor.launch.py"), variant="simulation")
    expected_calibration = {name: calibration[name] for name in (
        "experiment", "rate", "target_validity", "allow_hardware", "allow_multiturn_calibration",
        "max_torque", "max_speed", "max_travel", "max_temperature", "feedback_timeout", "max_gap")}
    expected_calibration.update({"low_speed_" + name: calibration["low_speed_" + name] for name in low})
    expected_physics = {name: value for name, value in physics.items()
                        if name not in ("motor_name", "motor_id", "master_id", "mode", "can_bus_namespace")}
    return {"axis": axis, "motor_id": 1 if axis == "A" else 2,
            "physics": expected_physics, "driver": driver, "calibration": expected_calibration,
            "profile": str(profile), "profile_sha256": sha256(profile),
            "qualification": "One repetition; acquisition/lifecycle diagnostic, not model qualification"}


def parameter_value(value):
    fields = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value",
              5: "byte_array_value", 6: "bool_array_value", 7: "integer_array_value",
              8: "double_array_value", 9: "string_array_value"}
    assert value.type in fields, f"Unset/unknown parameter type {value.type}"
    result = getattr(value, fields[value.type])
    return list(result) if value.type >= 5 else result


def expected_parameter_type(value):
    if type(value) is bool:
        return 1
    if type(value) is int:
        return 2
    if type(value) is float:
        return 3
    if type(value) is str:
        return 4
    if isinstance(value, (tuple, list)) and value:
        types = {expected_parameter_type(item) for item in value}
        assert len(types) == 1, "Mixed array parameter types"
        return {1: 6, 2: 7, 3: 8, 4: 9}[types.pop()]
    raise AssertionError(f"Unsupported expected parameter value: {value!r}")


def check_parameters(scene, node_name, expected):
    """A truncated or NOT_SET response fails before START; zip alone is unsafe."""
    client = scene.node.create_client(GetParameters, node_name + "/get_parameters")
    try:
        scene.wait(client.service_is_ready)
        names = list(expected)
        future = client.call_async(GetParameters.Request(names=names))
        scene.wait(future.done)
        response = future.result()
        assert response is not None and len(response.values) == len(names), (node_name, len(names), response)
        actual = {}
        for name, value in zip(names, response.values):
            wanted = list(expected[name]) if isinstance(expected[name], tuple) else expected[name]
            assert value.type == expected_parameter_type(wanted), (node_name, name, value.type, wanted)
            actual[name] = parameter_value(value)
            assert actual[name] == wanted, (node_name, name, actual[name], wanted)
        assert set(actual) == set(names)
        return actual
    finally:
        scene.node.destroy_client(client)


def owned_programs(scene):
    """Record actual child PIDs/images from this Scene's process groups only."""
    from ament_index_python.packages import get_package_prefix

    groups = {process.pid for process in scene.processes.values()}
    processes = []
    for directory in Path("/proc").iterdir():
        if not directory.name.isdigit():
            continue
        try:
            stat = (directory / "stat").read_text().rsplit(")", 1)[1].split()
            if int(stat[2]) not in groups:
                continue
            command = (directory / "cmdline").read_bytes().decode().strip("\0").split("\0")
            image = directory / "exe"
            processes.append({"pid": int(directory.name), "start_ticks": int(stat[19]),
                              "process_group": int(stat[2]), "argv": command,
                              "executable": str(image.resolve()), "image_sha256": sha256(image)})
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    result = []
    specs = (("can_bus", "can_bus_node", scene.motor + "/can"),
             ("dm_motor_control", "motor_node", scene.motor),
             ("dm_motor_control", "sim_motor_node", scene.motor),
             ("dm_motor_calibration", "calibration_node.py", scene.prefix + "/calibration"))
    for package, name, namespace in specs:
        path = Path(get_package_prefix(package)) / "lib" / package / name
        expected = str(path.resolve())
        if name.endswith(".py"):
            matches = [row for row in processes if any(
                arg.startswith("/") and str(Path(arg).resolve()) == expected for arg in row["argv"])]
        else:
            matches = [row for row in processes if row["executable"] == expected]
        assert len(matches) == 1, (name, expected, matches)
        row = dict(matches[0])
        assert "__ns:=" + namespace in row["argv"], (name, namespace, row["argv"])
        if not name.endswith(".py"):
            assert row["image_sha256"] == sha256(path), (name, "Running image differs from selected executable")
        row.update(package=package, program=name, selected_path=str(path), selected_sha256=sha256(path),
                   namespace=namespace)
        result.append(row)
    return result


def launch_scene(scene, case):
    scene.launch("motor", ["ros2", "launch", "dm_motor_control", "motor.launch.py",
        "simulation:=true", f"namespace:={scene.motor}", f"motor_id:={case['motor_id']}",
        f"config_file:={scene.config_path}", f"can_bus_namespace:={scene.motor}/can",
        "backend:=virtual", "require_virtual_bus:=true", "dashboard_port:=''",
        "control_rate:=500", "state_rate:=500"])
    # No port CLI token: a literal pair of quotes is a nonempty ROS argument.
    scene.wait(lambda: scene.state is not None and scene.state.ready and scene.state.feedback_fresh
               and scene.state.simulated and disabled_feedback(scene.state), timeout=20)
    assert scene.state.control_rate == 500.0
    assert scene.state.motor_id == case["motor_id"]
    assert (scene.state.pmax, scene.state.vmax, scene.state.tmax) == tuple(
        case["physics"][name] for name in ("mapping_position", "mapping_velocity", "mapping_torque"))
    assert scene.state.direction == case["driver"]["direction"]
    assert scene.state.software_offset == case["driver"]["software_offset"]
    loaded = {"bus": check_parameters(scene, scene.motor + "/can/can_bus", {"backend": "virtual", "port": ""}),
              "physics": check_parameters(scene, scene.motor + "/simulated_device", {
                  **case["physics"], "motor_id": case["motor_id"], "master_id": 0, "mode": 1})}
    # Production motor_controller intentionally disables ROS parameter services.
    # Read its actual public capabilities; require_virtual_bus is frozen input,
    # while backend/simulated/rates below are independent runtime observations.
    endpoint = motor_topics(scene.node, scene.motor).endpoint("get_capabilities")
    scene.wait(endpoint.service_is_ready)
    future = endpoint.call_async(MotorRequest())
    scene.wait(future.done)
    capabilities = future.result()
    assert capabilities is not None and capabilities.success and capabilities.ready
    assert capabilities.simulated and capabilities.backend == "virtual"
    assert capabilities.profile == case["driver"]["profile"]
    assert capabilities.control_rate == 500.0
    assert capabilities.configuration.control_rate == 500.0 and capabilities.configuration.state_rate == 500.0
    loaded["driver"] = {"simulated": capabilities.simulated, "backend": capabilities.backend,
                        "profile": capabilities.profile, "control_rate": capabilities.control_rate,
                        "state_rate": capabilities.configuration.state_rate,
                        "require_virtual_bus_scope": "True in frozen launch argv and resolved input; not exposed by capabilities"}
    scene.launch_collector()
    loaded["calibration"] = check_parameters(scene, scene.prefix + "/calibration/calibration_node", case["calibration"])
    assert scene.status["identity"]["simulated"] is True
    write_json(scene.directory / "LOADED_PARAMETERS.json", loaded)
    write_json(scene.directory / "ACTUAL_PROGRAMS.json", owned_programs(scene))
    return loaded


def audit_output(scene, output):
    """Verify recorded callbacks and report recovery, not device acknowledgements."""
    report = json.loads((output / "report.json").read_text())
    assert report["experiment"] == "low_speed" and report["qualified"] is False
    assert report["status"] in ("completed", "completed_unqualified"), report
    assert report["acquisition_complete"] is True
    assert report["stop_confirmed"] is True and report["stop_feedback_time"] is not None
    assert report["identity"]["simulated"] is True
    assert report["configuration"]["rate"] == scene.options["rate"]
    assert not (output / "calibration.json").exists()
    assert not report.get("fit") and not report["low_speed"].get("candidate_model")
    progress = report["low_speed_progress"]
    assert progress["done"] and progress["completed_cells"] == progress["required_cells"] == 10
    assert progress["settings"]["repetitions"] == 1
    scene.wait(lambda: scene.state is not None and scene.state.simulated and disabled_feedback(scene.state)
               and scene.state.feedback_receive_time >= report["stop_feedback_time"], timeout=4)
    manifest = json.loads((output / "raw_recording.json").read_text())
    assert manifest["recording_complete"] and manifest["stop_confirmed"]
    assert not manifest["recording_errors"] and not manifest["acquisition_error"]
    streams = {}
    contents = {}
    for name, spec in manifest["files"].items():
        path = output / name
        assert sha256(path) == spec["sha256"] and path.stat().st_size == spec["bytes"]
        if name == "position_observation.csv" and spec["rows"] == 0:
            # This optional journal stays empty when native observation is off.
            case = json.loads((scene.directory / "CASE.json").read_text())
            assert case["driver"]["native_position_observation_rate"] == 0.0
            observation = manifest["position_observation"]
            assert observation["enabled"] is True and observation["file"] == name
            assert observation["enqueued_callbacks"] == 0 and observation["status"] == "not_received"
            assert spec["fields"] == [] and spec["bytes"] == 0
            assert spec["sha256"] == hashlib.sha256(b"").hexdigest()
            fields, rows = [], []
        else:
            fields, rows = read_csv(path)
        assert len(rows) == spec["rows"] and fields == spec["fields"]
        streams[name] = {"rows": len(rows), "sha256": spec["sha256"]}
        contents[name] = rows
    for spec in manifest["sources"]:
        assert sha256(output / spec["path"]) == spec["sha256"]
    feedback = contents["feedback.csv"]
    commands = contents["commands.csv"]
    assert feedback and commands
    times = [float(row["receive_time"]) for row in feedback]
    counts = [int(row["feedback_count"]) for row in feedback]
    assert all(b >= a for a, b in zip(times, times[1:]))
    assert all(b >= a for a, b in zip(counts, counts[1:]))
    sequences = [int(row["sequence"]) for row in commands]
    assert all(b > a for a, b in zip(sequences, sequences[1:]))
    assert max(float(row["valid_for"]) for row in commands) == scene.options["target_validity"]
    trace = Trace.read(output / "samples.csv")
    assert len(trace.columns["time"]) == report["samples"] and report["samples"] > 0
    low = Trace.read(output / "low_speed_samples.csv")
    assert {"analysis_cell", "analysis_role", "analysis_valid", "in_window"} <= low.columns.keys()
    assert set(low.columns["analysis_role"]) <= {-1.0, 0.0, 1.0}
    matched = 0
    for row in zip(*(low.columns[name] for name in (
            "receive_time", "low_speed_command_time", "low_speed_valid_until", "analysis_valid",
            "analysis_cell", "low_speed_cell", "analysis_role", "low_speed_role"))):
        observed, issued, expiry, valid, cell, tagged_cell, role, tagged_role = row
        if valid == 1:
            matched += 1
            assert issued <= observed < expiry and cell == tagged_cell and role == tagged_role
    assert matched > 0
    for name in ("samples.csv", "low_speed_samples.csv"):
        assert sha256(output / name) == report["sample_files"][name]["sha256"]
    # Read saved data without starting another dashboard or making an HTTP request.
    proxy = SimpleNamespace(root=lambda: output.parent,
                            artifact=lambda run, name: output.parent / run / name)
    restored = Dashboard.run_details(proxy, output.name)
    assert restored["report"] == report and restored["fit"] is None
    assert restored["raw_samples"] == report["samples"] and restored["trace"]
    assert not restored["model_error"] and not restored["trace_error"]
    html_inputs = load_report_data(output)
    assert html_inputs.report == report and html_inputs.fit is None
    registry = yaml.safe_load(Path(scene.config["system_parameters_file"]).read_text())
    assert registry == {"version": 1, "motors": {}}, registry
    # Lifecycle/record recovery alone must not qualify a controller candidate
    # for real acquisition. Keep the one-repetition model unqualified, while
    # requiring every nominal physical-simulation cell to meet the old gates.
    failed_cells = [{"index": cell["index"], "issues": cell["quality_issues"]}
                    for cell in report["low_speed"]["cells"] if not cell["quality_passed"]]
    assert not failed_cells, {"nominal_low_speed_cell_quality_failed": failed_cells}
    return {"passed": True, "qualified_model": False, "cell_quality_passed": True,
            "cell_quality_count": len(report["low_speed"]["cells"]), "raw_streams": streams,
            "samples": report["samples"], "low_speed_rows": len(low.columns["time"]),
            "causally_compatible_rows": matched,
            "source_counter_gaps": sum(max(0, b - a - 1) for a, b in zip(counts, counts[1:])),
            "command_sequence_scope": "Software targets, not device ACKs or raw CAN proof",
            "fresh_disabled_feedback_time": scene.state.feedback_receive_time,
            "stop_feedback_time": report["stop_feedback_time"], "dashboard_restore": True,
            "report_loader_restore": True, "registry_unchanged": True}


def execute_case(scene, case):
    launch_scene(scene, case)
    scene.call(scene.start, Trigger.Request())
    scene.wait(lambda: scene.status["phase"] in TERMINAL and scene.status["stop_confirmed"], timeout=300)
    output = Path(scene.status["output_directory"])
    result = audit_output(scene, output)
    result.update(report_directory=str(output), report_sha256=sha256(output / "report.json"))
    return result


@pytest.mark.parametrize("values,accepted", [
    ([SimpleNamespace(type=3, double_value=500.0), SimpleNamespace(type=1, bool_value=True)], True),
    ([SimpleNamespace(type=3, double_value=500.0)], False),
    ([SimpleNamespace(type=3, double_value=500.0), SimpleNamespace(type=0)], False),
    ([SimpleNamespace(type=2, integer_value=500), SimpleNamespace(type=1, bool_value=True)], False),
    ([SimpleNamespace(type=3, double_value=200.0), SimpleNamespace(type=1, bool_value=True)], False),
])
def test_parameter_readback_is_complete_and_typed(values, accepted):
    destroyed = []
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(values=values))
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: future)
    node = SimpleNamespace(create_client=lambda *args: client, destroy_client=destroyed.append)
    def wait(predicate):
        assert predicate()
    scene = SimpleNamespace(node=node, wait=wait)
    if accepted:
        assert check_parameters(scene, "/fake", {"rate": 500.0, "virtual": True}) == {
            "rate": 500.0, "virtual": True}
    else:
        with pytest.raises(AssertionError):
            check_parameters(scene, "/fake", {"rate": 500.0, "virtual": True})
    assert destroyed == [client]


@pytest.mark.parametrize("reference_limited", (False, True))
def test_projected_low_speed_reference_keeps_saturation_in_published_metadata(reference_limited):
    # No ROS context, node or transport: exercise production message construction.
    # A reference projection must remain a quality failure even when the final
    # projected command is small enough to preserve the requested native gains.
    from collections import OrderedDict
    from builtin_interfaces.msg import Time
    from dm_motor_calibration.precision import Drive, Sample
    from dm_motor_calibration.ros_node import CalibrationNode

    sent, recorded = [], []
    drive = Drive(position=.001, velocity=0., kp=20., kd=.8, damping_priority=True)
    engine = SimpleNamespace(control=lambda now: drive, done=False, phase="sample",
        latest=Sample(1., 0., 0., 0., 25.), reference_limited=reference_limited,
        tags=lambda: {"low_speed_cell": 0., "low_speed_reference_limited": float(reference_limited)})
    owner = SimpleNamespace(low_speed_engine=engine, settings=SimpleNamespace(max_torque=.4),
        sequence=0, topics=SimpleNamespace(client_id="offline_projection_test"), validity=.1,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))),
        commands=OrderedDict(), started=0., origin=0., targets=SimpleNamespace(publish=sent.append),
        record_message=lambda *args, **kwargs: recorded.append((args, kwargs)))
    state = SimpleNamespace(kp_limit=100., kd_limit=1., feedback_receive_time=1.,
        position=0., velocity=0., pmax=12.5, vmax=50., tmax=10., direction=1., software_offset=0.)
    CalibrationNode.low_speed_control(owner, 1., state)
    assert len(sent) == 1 and sent[0].target.kp == 20. and sent[0].target.kd == .8
    assert owner.commands[1]["saturated"] == float(reference_limited)
    assert engine.last_saturated is reference_limited
    assert recorded[0][1]["low_speed_reference_limited"] == float(reference_limited)


@pytest.mark.parametrize("position_kp", (0., 50.))
def test_idle_low_speed_node_does_not_build_an_unused_multisine_plan(tmp_path, monkeypatch, position_kp):
    # No motor process or START: exercise the real node's first status callback,
    # which previously revalidated an unrelated quick plan and killed the node.
    from rclpy.parameter import Parameter
    from dm_motor_calibration.data import CalibrationError
    from dm_motor_calibration.ros_node import CalibrationNode

    monkeypatch.setenv("ROS_DOMAIN_ID", os.environ.get("DM_MOTOR_TEST_DOMAIN", str(195 + os.getpid() % 30)))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    config = yaml.safe_load((REPO / "config/dm_motors.yaml").read_text())
    config["system_parameters_file"] = str(tmp_path / "unused_registry.yaml")
    (tmp_path / "unused_registry.yaml").write_text("version: 1\nmotors: {}\n")
    config["packages"]["dm_motor_calibration"]["nodes"]["calibration_node"].update(
        experiment="low_speed", amplitude=1.2, max_travel=.7, max_speed=1.,
        low_speed_position_kp=position_kp, low_speed_velocity_ki=0. if position_kp else 2.)
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(config))
    rclpy.init()
    controller = None
    try:
        controller = CalibrationNode(parameter_overrides=[Parameter("config_file", value=str(path))])
        status = controller.status_message()
        assert status["phase"] == "idle" and status["experiment"] == "low_speed"
        assert status["quick_calibration_plan"] is None
        assert status["low_speed_settings"]["speeds"] == (.05, .1, .2)
        assert status["low_speed_settings"]["position_kp"] == position_kp
        assert not controller.motion_requested and not controller.run_open
        controller.publish_status()
        with pytest.raises(CalibrationError, match="Excitation must fit"):
            controller.validated_settings({"experiment": "calibrate"})
        with pytest.raises(CalibrationError, match="travel"):
            controller.validated_settings({"max_travel": .5})
    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("axis", ("A", "B"))
def test_nominal_low_speed_acquisition_and_stop(tmp_path, monkeypatch, axis):
    monkeypatch.setenv("ROS_DOMAIN_ID", os.environ.get("DM_MOTOR_TEST_DOMAIN", str(195 + os.getpid() % 30)))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    scene = Scene(tmp_path)
    try:
        case = configure_scene(scene, axis)
        write_json(tmp_path / "CASE.json", case)
        result = execute_case(scene, case)
        write_json(tmp_path / "RESULT.json", result)
    finally:
        try:
            if scene.stop.service_is_ready():
                fence = time.monotonic()
                scene.call(scene.stop, Trigger.Request())
                scene.wait(lambda: scene.state is not None and scene.state.simulated and disabled_feedback(scene.state)
                           and scene.state.feedback_receive_time > fence, timeout=5)
        finally:
            try:
                scene.close()
            finally:
                rclpy.shutdown()
