from collections import deque
import json
from pathlib import Path
import shutil
import subprocess
from threading import RLock
from types import SimpleNamespace

from common import ConfigManager
import pytest
from rclpy.logging import get_logger

from dm_motor_bilateral_control.dashboard import ApiError, Dashboard, EDITABLE, validate_settings


def settings():
    defaults = ConfigManager().node_parameters("dm_motor_bilateral_control", "bilateral_control_node")
    return {key: defaults[key] for key in EDITABLE}


def test_configured_defaults_are_valid_without_dashboard_overrides():
    configured = settings()
    mode = ConfigManager().node_parameters("dm_motor_bilateral_control", "bilateral_control_node")["control_mode"]
    assert validate_settings(configured, control_mode=mode) == configured


def test_zero_local_damping_requires_an_explicit_parameter_choice():
    result = validate_settings({**settings(), "local_damping": 0.0})
    assert result["local_damping"] == 0


@pytest.mark.parametrize("overrides", [
    {"direction": True}, {"direction": 0}, {"stiffness": float("nan")},
    {"stiffness": -0.01}, {"stiffness": 500.01}, {"stiffness": True},
    {"coupling_damping": 0.75, "local_damping": 0.5}, {"max_torque": -0.01},
    {"max_torque": True}, {"max_torque": float("inf")}, {"max_feedback_torque": 0},
    {"feedback_age": 0.1, "feedback_stop": 0.08}, {"pair_skew": 0.11},
    {"recovery_time": 0}, {"warning_temperature": 60, "max_temperature": 55},
    {"motion_limits_enabled": "false"}, {"max_travel": float("inf")},
    {"friction_compensation_enabled": 1}, {"friction_compensation_enabled": "false"},
    {"friction_compensation_fraction": -0.01}, {"friction_compensation_fraction": 1.01},
    {"friction_compensation_fraction": True}, {"friction_compensation_fraction": float("nan")},
    {"friction_torque_limit": 0}, {"friction_torque_limit": 0.50001},
    {"friction_torque_limit": float("inf")},
])
def test_invalid_parameters_are_explicitly_rejected(overrides):
    with pytest.raises(ApiError):
        validate_settings({**settings(), **overrides})


def test_simple_bilateral_parameters_are_not_accepted():
    value = settings()
    value["kp"] = value.pop("stiffness")
    with pytest.raises(ApiError):
        validate_settings(value)


@pytest.mark.parametrize("stiffness", [0, 100, 500])
def test_zero_application_budget_preserves_requested_gains_and_other_limits(stiffness):
    configured = {**settings(), "max_torque": 0, "stiffness": stiffness}
    assert validate_settings(configured) == configured


@pytest.mark.parametrize("fraction", [0, 1])
def test_friction_compensation_accepts_fraction_endpoints_and_maximum_limit(fraction):
    result = validate_settings({**settings(), "friction_compensation_fraction": fraction,
                                "friction_torque_limit": 0.5})
    assert result["friction_compensation_fraction"] == fraction
    assert result["friction_torque_limit"] == 0.5


def idle_snapshot():
    return {
        "connected": True, "configuration_fresh": True,
        "configuration": settings(),
        "state": {"phase": Dashboard.state_type.IDLE, "stop_confirmed": True, "control_mode": "native",
                  "motor_a": "/motor_1", "motor_b": "/motor_2", "commands_ready": True,
                  "paired": True, "pair_age": settings()["feedback_age"] / 2,
                  "pair_skew": settings()["pair_skew"] / 2},
        "motors": {name: {"connected": True, "ready": True, "feedback_fresh": True,
                          "kp_limit": 500., "kd_limit": 1.}
                   for name in ("/motor_1", "/motor_2")},
    }


@pytest.mark.parametrize("budget", [0., .5])
def test_shared_effective_gains_map_to_native_limits_without_changing_requested_values(budget):
    configured = {**settings(), "stiffness": 250., "coupling_damping": .4975,
                  "local_damping": .005, "max_torque": budget}
    assert validate_settings(configured, control_mode="shared_reference") == configured
    for changes in ({"stiffness": 250.01}, {"coupling_damping": .5}, {"local_damping": .006}):
        with pytest.raises(ApiError, match="2\\*Kc and 2\\*Dc\\+B"):
            validate_settings({**configured, **changes}, control_mode="shared_reference")


@pytest.mark.parametrize("stiffness", [0., 50., 500.])
@pytest.mark.parametrize("budget", [0., .5])
def test_differential_host_stiffness_is_independent_of_native_position_cap(stiffness, budget):
    configured = {**settings(), "stiffness": stiffness, "coupling_damping": .375,
                  "local_damping": .05, "max_torque": budget}
    assert validate_settings(configured, control_mode="differential_torque",
                             kp_limit=2., kd_limit=.8) == configured
    for changes in ({"stiffness": -0.01}, {"stiffness": 500.01},
                    {"coupling_damping": .3751}, {"local_damping": .0501}):
        with pytest.raises(ApiError, match="driver gain limits"):
            validate_settings({**configured, **changes}, control_mode="differential_torque",
                              kp_limit=2., kd_limit=.8)


@pytest.mark.parametrize("mode,stiffness,dc", [("native", 100., .75), ("shared_reference", 50., .375)])
def test_effective_bounds_use_both_driver_caps(mode, stiffness, dc):
    configured = {**settings(), "stiffness": stiffness, "coupling_damping": dc, "local_damping": .05}
    assert validate_settings(configured, control_mode=mode, kp_limit=100., kd_limit=.8) == configured
    for changes in ({"stiffness": stiffness+.01}, {"coupling_damping": dc+.01}):
        with pytest.raises(ApiError, match="driver gain limits"):
            validate_settings({**configured, **changes}, control_mode=mode, kp_limit=100., kd_limit=.8)


@pytest.mark.parametrize("mode", [None, "", "shared_midpoint", "unknown"])
def test_unknown_control_mode_is_not_silently_treated_as_native(mode):
    with pytest.raises(ApiError, match="Unknown bilateral control mode"):
        validate_settings(settings(), control_mode=mode)


def test_mode_cannot_be_changed_through_dashboard_settings():
    with pytest.raises(ApiError, match="exactly"):
        validate_settings({**settings(), "control_mode": "shared_reference"})
    assert "control_mode" not in Dashboard.parameters


def test_hardware_trial_permission_is_read_only_startup_configuration():
    assert "allow_differential_hardware" in Dashboard.parameters
    assert "allow_differential_hardware" not in EDITABLE
    with pytest.raises(ApiError, match="exactly"):
        validate_settings({**settings(), "allow_differential_hardware": True})
    html = (Path(__file__).resolve().parents[1] / "web/index.html").read_text()
    assert "allow_differential_hardware" not in html


def test_dashboard_reads_current_state_mode_and_smaller_driver_limit(monkeypatch):
    snapshot = idle_snapshot()
    snapshot["state"]["control_mode"] = "shared_reference"
    snapshot["motors"]["/motor_2"].update(kp_limit=100., kd_limit=.8)
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    configured = {**settings(), "stiffness": 50., "coupling_damping": .375, "local_damping": .05}
    assert dashboard.validate_settings(configured) == configured
    with pytest.raises(ApiError, match="driver gain limits"):
        dashboard.validate_settings({**configured, "stiffness": 50.01})
    # The current read-only state, not a cached default or a client field, decides the mapping.
    snapshot["state"]["control_mode"] = "native"
    assert dashboard.validate_settings({**configured, "stiffness": 100.})["stiffness"] == 100.


@pytest.mark.parametrize("mode", ["shared_reference", "differential_torque"])
def test_control_mode_arrives_through_existing_state_subscription(mode):
    dashboard = Dashboard.__new__(Dashboard)
    dashboard.lock = RLock()
    dashboard.state = None
    dashboard.running = dashboard.seen_guard = False
    dashboard.motor_subscriptions = {"/motor_1": None, "/motor_2": None}
    dashboard.event = lambda *_args, **_kwargs: None
    dashboard.on_state(Dashboard.state_type(control_mode=mode,
        motor_a="/motor_1", motor_b="/motor_2", phase=Dashboard.state_type.IDLE))
    assert dashboard.state["control_mode"] == mode


def test_dashboard_accepts_fresh_differential_mode_and_uses_both_damping_caps(monkeypatch):
    snapshot = idle_snapshot()
    snapshot["state"]["control_mode"] = "differential_torque"
    snapshot["motors"]["/motor_2"].update(kp_limit=2., kd_limit=.8)
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    configured = {**settings(), "stiffness": 500., "coupling_damping": .375, "local_damping": .05}
    assert dashboard.validate_settings(configured) == configured
    with pytest.raises(ApiError):
        dashboard.validate_settings({**configured, "coupling_damping": .4})


@pytest.mark.parametrize("simulated", [False, None, 1])
@pytest.mark.parametrize("permission", [False, True])
def test_differential_start_rejects_mixed_or_unknown_identity(monkeypatch, simulated, permission):
    snapshot = idle_snapshot()
    snapshot["state"]["control_mode"] = "differential_torque"
    for motor in snapshot["motors"].values():
        motor["simulated"] = True
    snapshot["configuration"]["allow_differential_hardware"] = permission
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    dashboard.require_start()
    snapshot["motors"]["/motor_2"]["simulated"] = simulated
    dashboard.require_idle()  # Configuration remains editable without enabling.
    with pytest.raises(ApiError, match="two simulated motors") as error:
        dashboard.require_start()
    assert error.value.status == 409
    for mode in ("native", "shared_reference"):
        snapshot["state"]["control_mode"] = mode
        dashboard.require_start()


@pytest.mark.parametrize("permission", [None, False, 0, 1, "true", True])
def test_differential_real_start_requires_strict_true_startup_permission(monkeypatch, permission):
    snapshot = idle_snapshot()
    snapshot["state"]["control_mode"] = "differential_torque"
    for motor in snapshot["motors"].values():
        motor["simulated"] = False
    if permission is not None:
        snapshot["configuration"]["allow_differential_hardware"] = permission
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    dashboard.require_idle()
    if permission is True:
        dashboard.require_start()
        snapshot["configuration_fresh"] = False
        with pytest.raises(ApiError, match="stale") as error:
            dashboard.require_start()
        assert error.value.status == 503
    else:
        with pytest.raises(ApiError, match="allow_differential_hardware") as error:
            dashboard.require_start()
        assert error.value.status == 409


def test_differential_simulated_start_does_not_require_hardware_permission(monkeypatch):
    snapshot = idle_snapshot()
    snapshot["state"]["control_mode"] = "differential_torque"
    for motor in snapshot["motors"].values():
        motor["simulated"] = True
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    dashboard.require_start()


def test_velocity_bandwidth_state_is_visible_but_cannot_be_written_from_dashboard():
    dashboard = Dashboard.__new__(Dashboard)
    dashboard.lock = RLock()
    dashboard.state = None
    dashboard.running = dashboard.seen_guard = False
    dashboard.motor_subscriptions = {"/motor_1": None, "/motor_2": None}
    dashboard.event = lambda *_args, **_kwargs: None
    fields = dict(required_velocity_bandwidth=100., velocity_bandwidth_verified=True,
                  motor_a_velocity_bandwidth=100., motor_b_velocity_bandwidth=100.)
    dashboard.on_state(Dashboard.state_type(control_mode="shared_reference", **fields,
        motor_a="/motor_1", motor_b="/motor_2", phase=Dashboard.state_type.ACTIVE))
    assert all(dashboard.state[name] == value for name, value in fields.items())
    for name in fields:
        assert name not in EDITABLE and name not in Dashboard.parameters
        with pytest.raises(ApiError) as error:
            validate_settings({**settings(), name: fields[name]})
        assert error.value.status == 400
    html = (Path(__file__).resolve().parents[1] / "web/index.html").read_text()
    assert '<p id="velocity-bandwidth"' in html
    assert '<input id="velocity-bandwidth"' not in html and '<select id="velocity-bandwidth"' not in html


@pytest.mark.parametrize("missing", ["mode", "controller", "configuration", "driver", "kp_limit", "kd_limit"])
def test_dashboard_mapping_requires_fresh_mode_and_capability(monkeypatch, missing):
    snapshot = idle_snapshot()
    if missing == "mode": snapshot["state"].pop("control_mode")
    elif missing == "controller": snapshot["connected"] = False
    elif missing == "configuration": snapshot["configuration_fresh"] = False
    elif missing == "driver": snapshot["motors"]["/motor_2"]["feedback_fresh"] = False
    else: snapshot["motors"]["/motor_2"][missing] = float("nan")
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    with pytest.raises(ApiError) as error:
        dashboard.validate_settings(settings())
    assert error.value.status == 503


def test_browser_maps_gains_from_read_only_mode_and_driver_capabilities():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js unavailable for offline JavaScript helper test")
    web = Path(__file__).resolve().parents[1] / "web"
    source = (web / "dashboard.js").read_text()
    helpers = source[source.index("  function nativeGains("):source.index("  function notice(")]
    snapshot = idle_snapshot()
    script = "const assert = require('node:assert/strict'); let data = " + json.dumps(snapshot) + ";\n" + helpers + """
      assert.deepEqual(nativeGains({stiffness:50,coupling_damping:.7,local_damping:.005}, 'native'), {kp:50,kd:.705});
      for (const max_torque of [0,.5]) {
        assert.deepEqual(nativeGains({stiffness:50,coupling_damping:.4975,local_damping:.005,max_torque}, 'shared_reference'), {kp:100,kd:1});
      }
      for (const stiffness of [0,50,500]) {
        assert.deepEqual(nativeGains({stiffness,coupling_damping:.4975,local_damping:.005}, 'differential_torque'), {kp:0,kd:1});
      }
      data.state.control_mode='shared_reference';
      data.motors['/motor_2'].kp_limit=300; data.motors['/motor_2'].kd_limit=.8;
      assert.deepEqual(gainCapability(), {mode:'shared_reference',multiplier:2,kp:300,kd:.8});
      assert.equal(stiffnessLimit(gainCapability()),150);
      data.state.control_mode='differential_torque'; data.motors['/motor_2'].kp_limit=2;
      assert.deepEqual(gainCapability(), {mode:'differential_torque',multiplier:2,kp:2,kd:.8});
      assert.equal(stiffnessLimit(gainCapability()),500);
      data.state.control_mode='missing'; assert.equal(gainCapability(), null);
      assert.throws(()=>nativeGains({},'missing'));
      data.state.control_mode='native'; data.motors['/motor_2'].feedback_fresh=false;
      assert.equal(gainCapability(), null);
      assert.match(velocityBandwidthStatus({}, true).text, /未读取/);
      assert.match(velocityBandwidthStatus({}, false).text, /过期/);
      let bandwidth={phase:3, required_velocity_bandwidth:100, velocity_bandwidth_verified:true,
        motor_a_velocity_bandwidth:100, motor_b_velocity_bandwidth:100};
      assert.deepEqual(velocityBandwidthStatus(bandwidth,true), {text:'速度带宽 100 Hz 已核验',warning:false});
      bandwidth.motor_b_velocity_bandwidth=40;
      assert.equal(velocityBandwidthStatus(bandwidth,true).warning,true);
      bandwidth.phase=0;
      assert.match(velocityBandwidthStatus(bandwidth,true).text,/开始前核验/);
      bandwidth.required_velocity_bandwidth=0;
      assert.match(velocityBandwidthStatus(bandwidth,true).text,/未设置核验要求/);
      bandwidth={phase:3, required_velocity_bandwidth:123.456, velocity_bandwidth_verified:true,
        motor_a_velocity_bandwidth:Math.fround(123.456),motor_b_velocity_bandwidth:Math.fround(123.456)};
      assert.match(velocityBandwidthStatus(bandwidth,true).text,/已核验/);
      bandwidth.motor_b_velocity_bandwidth=123.456;
      assert.equal(velocityBandwidthStatus(bandwidth,true).warning,true);
    """
    subprocess.run([node, "-e", script], check=True, capture_output=True, text=True)
    subprocess.run([node, "--check", str(web / "dashboard.js")], check=True, capture_output=True, text=True)
    html = (web / "index.html").read_text()
    assert '<span id="control-mode">' in html
    assert '<select id="control-mode"' not in html and '<input id="control-mode"' not in html
    assert "原生 MIT Kp / Kd" in html


def test_browser_differential_form_start_gate_and_wire_limit_display():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js unavailable for offline JavaScript form test")
    source = (Path(__file__).resolve().parents[1] / "web/dashboard.js").read_text()
    snapshot = idle_snapshot()
    snapshot["state"].update(control_mode="differential_torque", compensation_ready_a=True,
                             compensation_ready_b=True)
    snapshot["configuration"].update(stiffness=500., coupling_damping=.375, local_damping=.05)
    for motor in snapshot["motors"].values():
        motor.update(simulated=True, kp_limit=2., kd_limit=.8)
    functions = source[source.index("  function nativeGains("):source.index("  function notice(")]
    functions += source[source.index("  function controls("):source.index("  function motor(")]
    functions += source[source.index("  function outputStatus("):source.index("  function compensation(")]
    numbers = source[source.index("  const numbers ="):source.index("  const phases =")]
    script = "const assert=require('node:assert/strict'); let data=" + json.dumps(snapshot) + ";\n"
    script += numbers + """
      const elements=new Map();
      const $=id=>{if(!elements.has(id))elements.set(id,{value:'',checked:false,
        classList:{toggle(){}}});return elements.get(id);};
      const format=(value,digits=4)=>Number.isFinite(value)?value.toFixed(digits):'—';
      const document={hidden:false};
      let disconnected=false, initialized=true, busy=false, stopping=false, dirty=false;
      for(const [key,value] of Object.entries(data.configuration)) {
        if(typeof value==='boolean')$(key).checked=value; else $(key).value=String(value);
      }
    """ + functions + """
      controls();
      assert.equal($('control-mode').textContent,'差模力矩');
      assert.match($('gain-help').textContent,/主机弹簧/);
      assert.match($('gain-help').textContent,/Kp=0/);
      assert.equal($('stiffness').max,'500');
      assert.equal($('coupling_damping').max,'0.375');
      assert.equal($('start').disabled,false);
      assert.equal(readSettings().stiffness,500);
      $('stiffness').value='500.01'; assert.throws(readSettings,/Kc/);
      $('stiffness').value='50'; $('coupling_damping').value='.376';
      assert.throws(readSettings,/Kp.*Kd/);
      $('coupling_damping').value='.375';
      data.motors['/motor_2'].simulated=false; controls();
      assert.equal($('start').disabled,true); assert.equal($('apply').disabled,false);
      assert.match($('availability').textContent,/不支持混合连接/);
      data.configuration.allow_differential_hardware=true; controls();
      assert.equal($('start').disabled,true); // Permission cannot authorize mixed identities.
      data.motors['/motor_1'].simulated=false; controls();
      assert.equal($('start').disabled,false);
      assert.equal($('control-mode').textContent,'差模力矩 · 真机试验模式');
      assert.equal($('start').textContent,'开始 · 真机试验');
      assert.match($('availability').textContent,/真机试验模式已由启动配置授权/);
      assert.equal(Object.hasOwn(readSettings(),'allow_differential_hardware'),false);
      for (const permission of [false,undefined,1,'true']) {
        data.configuration.allow_differential_hardware=permission; controls();
        assert.equal($('start').disabled,true);
        assert.equal($('control-mode').textContent,'差模力矩');
      }
      data.configuration.allow_differential_hardware=true;
      data.configuration_fresh=false; controls();
      assert.equal($('start').disabled,true);
      assert.equal($('control-mode').textContent,'差模力矩');
      data.configuration_fresh=true;
      for (const identity of [null,1,undefined]) {
        data.motors['/motor_2'].simulated=identity; controls();
        assert.equal($('start').disabled,true);
      }
      data.motors['/motor_1'].simulated=data.motors['/motor_2'].simulated=true;
      Object.assign(data.state,{phase:3,decoded_kp:0,decoded_kd:.8,gain_scale:1,
        recovery_scale:1,application_torque_limited:false,protocol_saturated_a:true,protocol_saturated_b:false});
      outputStatus(data.state,data.configuration);
      assert.equal($('gains').textContent,'0.000 / 0.800');
      assert.ok($('requested-gains').textContent.includes('原生目标 0.000 / 0.800'));
      assert.match($('protocol-status').textContent,/耦合需求受协议上限限制/);
      assert.match($('gain-reason').textContent,/原生增益未缩小；耦合需求受协议上限限制/);
      driverLimits('a',data.state,data.motors['/motor_1']);
      assert.match($('a-output-status').textContent,/前馈或协议上限限制/);
    """
    result = subprocess.run([node, "-e", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("unready", ["commands_ready", "paired", "pair_age", "pair_skew"])
def test_disabled_configuration_remains_editable_before_start_pairing_is_ready(monkeypatch, unready):
    snapshot = idle_snapshot()
    if unready in ("commands_ready", "paired"):
        snapshot["state"][unready] = False
    else:
        key = "feedback_age" if unready == "pair_age" else "pair_skew"
        snapshot["state"][unready] = snapshot["configuration"][key] * 2
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    dashboard.require_idle()
    with pytest.raises(ApiError, match="fresh paired disabled samples"):
        dashboard.require_start()


@pytest.mark.parametrize("unready", ["controller", "configuration", "stop", "motor"])
def test_configuration_still_requires_fresh_confirmed_disabled_state(monkeypatch, unready):
    snapshot = idle_snapshot()
    if unready == "controller":
        snapshot["connected"] = False
    elif unready == "configuration":
        snapshot["configuration_fresh"] = False
    elif unready == "stop":
        snapshot["state"]["stop_confirmed"] = False
    else:
        snapshot["motors"]["/motor_2"]["feedback_fresh"] = False
    dashboard = Dashboard.__new__(Dashboard)
    monkeypatch.setattr(Dashboard, "snapshot", lambda self: snapshot)
    with pytest.raises(ApiError):
        dashboard.require_idle()
    with pytest.raises(ApiError):
        dashboard.require_start()


def test_fault_and_recovery_events_can_alternate_log_severity():
    logger = get_logger("bilateral_dashboard_event_recovery")
    dashboard = SimpleNamespace(lock=RLock(), events=deque(maxlen=10), notice="",
                                get_logger=lambda: logger)
    messages = [("Controller fault", True), ("Recovered", False),
                ("Controller fault again", True), ("Restart ready", False)]
    for message, error in messages:
        Dashboard.event(dashboard, message, error=error)
    assert dashboard.notice == "Restart ready"
    assert [event["message"] for event in dashboard.events] == [message for message, _ in messages]
