#!/usr/bin/env python3
"""Optional real-browser acceptance of standard bilateral control on virtual devices."""

import json
import os
from pathlib import Path
import unittest
from urllib.error import URLError
from urllib.request import Request, urlopen

from dm_motor_interfaces.msg import BilateralState
from dm_motor_interfaces.srv import SimulationControl
from test_physical_simulation import PhysicalMotorScene
from bilateral_friction_fixture import simulation_calibrations


class BilateralBrowser(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState

    def configure_scene(self, document, profile):
        profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"].update(
            control_mode="native", required_velocity_bandwidth=0.0,
            friction_compensation_enabled=False, stiffness=16.0, coupling_damping=0.35,
            local_damping=0.35, max_torque=8.0)
        profile["nodes"]["dm_motor_control"]["motor_node"]["startup_velocity_bandwidth"] = 0.0
        profile["devices"]["motor_2"]["ambient_temperature"] = 46.0

    def test_operator_dashboard_browser(self):
        from playwright.sync_api import expect, sync_playwright
        origin = f"http://127.0.0.1:{self.controller_port}"
        motor_origin = f"http://127.0.0.1:{self.web_port}"
        def state(base=origin, path="/api/state"):
            with urlopen(base + path, timeout=2) as response:
                return json.load(response)
        def ready():
            try:
                data = state()
                return (data["connected"] and data["configuration_fresh"] and len(data["motors"]) == 2
                        and data["state"]["paired"])
            except URLError:
                return False
        self.wait(ready)
        assert all(motor["simulated"] for motor in state()["motors"].values())
        def force(torque):
            with urlopen(Request(motor_origin + "/api/external-torque",
                data=json.dumps({"torque": torque, "duration": 0.3}).encode(),
                headers={"Origin": motor_origin, "Content-Type": "application/json"}), timeout=2) as response:
                assert response.status == 200
        report = {}
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": 1440, "height": 1000})
            errors, writes = [], []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.on("request", lambda request: writes.append(request.url) if request.method == "POST" else None)
            try:
                page.goto(origin)
                expect(page.locator("#start")).to_be_enabled()
                assert not writes, writes
                assert float(page.locator("#stiffness").input_value()) == state()["configuration"]["stiffness"]
                assert not page.locator("#motion_limits_enabled").is_checked()
                assert not page.locator("#friction_compensation_enabled").is_checked()
                expect(page.locator("#b-temperature")).to_have_class("warning")
                page.locator("#start").click()
                expect(page.locator("#phase")).to_have_text("双边耦合中")
                assert state()["state"]["operator_guarded"]
                assert page.locator("#stiffness").is_disabled()
                report["explicit_start_and_no_auto_enable"] = True
                response = self.service(self.simulation["motor_2"], SimulationControl.Request(
                    operation=SimulationControl.Request.SET_ANCHOR, value=0.0))
                assert response.success
                self.settle(0.7, action=lambda: force(0.18))
                snapshot = state()
                a = snapshot["motors"][snapshot["state"]["motor_a"]]
                b = snapshot["motors"][snapshot["state"]["motor_b"]]
                assert a["torque"] < -0.03 and b["torque"] > 0.03
                assert snapshot["state"]["target_a"]["torque"] == snapshot["state"]["target_b"]["torque"] == 0
                expect(page.locator("#a-torque")).not_to_have_text("—")
                expect(page.locator("#pair-count")).not_to_have_text("—")
                report["physical_anchor_force_reflection_and_zero_feedforward"] = True
                for width in (1440, 1024, 768, 550, 390, 320):
                    page.set_viewport_size({"width": width, "height": 900})
                    assert page.evaluate("document.documentElement.scrollWidth <= innerWidth"), width
                    stop = page.locator("#stop").bounding_box()
                    assert stop is not None and 0 <= stop["x"] and stop["x"] + stop["width"] <= width
                page.set_viewport_size({"width": 1440, "height": 1000})
                output = os.environ.get("DM_BILATERAL_BROWSER_OUTPUT")
                if output:
                    Path(output).mkdir(parents=True, exist_ok=True)
                    page.evaluate("window.scrollTo(0,0); document.activeElement.blur()")
                    page.screenshot(path=str(Path(output) / "bilateral-dashboard.png"), full_page=True)
                report["responsive_start_stop_and_plots"] = True
                force(0.0)
                page.locator("#stop").click()
                expect(page.locator("#start")).to_be_enabled()
                self.service(self.simulation["motor_2"], SimulationControl.Request(
                    operation=SimulationControl.Request.CLEAR_ANCHOR))
                self.wait(lambda: all(abs(motor.velocity) < 0.05 for motor in self.states.values()))

                page.locator("#local_damping").fill("0.8")
                page.locator("#start").click()
                expect(page.locator("#notice")).to_contain_text("Dc+B")
                assert state()["state"]["stop_confirmed"]
                page.locator("#local_damping").fill("0")
                report["invalid_gain_combination_does_not_start"] = True

                delayed = []
                page.route("**/api/start", lambda route: delayed.append(route))
                with page.expect_request(lambda request: request.url.endswith("/api/start")):
                    page.locator("#start").click()
                page.wait_for_timeout(50)
                assert len(delayed) == 1
                with page.expect_response(lambda response: response.url.endswith("/api/stop")) as stopped:
                    page.locator("#stop").click()
                assert stopped.value.ok
                with page.expect_response(lambda response: response.url.endswith("/api/start")) as late:
                    delayed[0].continue_()
                assert late.value.status == 409
                page.unroute("**/api/start")
                expect(page.locator("#start")).to_be_enabled()
                assert state()["state"]["stop_confirmed"]
                report["late_http_start_cannot_undo_stop"] = True

                page.locator("#start").click()
                expect(page.locator("#phase")).to_have_text("双边耦合中")
                page.goto("about:blank")
                self.wait(lambda: state()["state"]["stop_confirmed"], timeout=3)
                page.goto(origin)
                expect(page.locator("#start")).to_be_enabled()
                page.wait_for_timeout(400)
                assert state()["state"]["stop_confirmed"]
                report["page_exit_stops_and_return_does_not_resume"] = True
                assert not errors, errors
                report["page_errors"] = errors
                report["hardware_used"] = False
                print(json.dumps(report, indent=2))
                if output:
                    (Path(output) / "bilateral-browser-results.json").write_text(json.dumps(report, indent=2) + "\n")
            finally:
                browser.close()


class BilateralFrictionBrowser(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState

    def configure_scene(self, document, profile):
        profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"].update(
            control_mode="native", required_velocity_bandwidth=0.0,
            stiffness=50.0, coupling_damping=0.995, local_damping=0.005,
            friction_compensation_enabled=False, friction_compensation_fraction=0.6,
            friction_torque_limit=0.2, max_torque=0.0)
        profile["nodes"]["dm_motor_control"]["motor_node"]["startup_velocity_bandwidth"] = 0.0
        self.friction_registry = simulation_calibrations(self, document, profile)

    def test_operator_dashboard_friction_controls_and_guarded_stop(self):
        from playwright.sync_api import expect, sync_playwright
        origin = f"http://127.0.0.1:{self.controller_port}"
        motor_origin = f"http://127.0.0.1:{self.web_port}"

        def state():
            with urlopen(origin + "/api/state", timeout=2) as response:
                return json.load(response)

        def ready():
            try:
                snapshot = state()
                return snapshot["connected"] and snapshot["configuration_fresh"] and snapshot["state"]["paired"]
            except URLError:
                return False

        def force(torque):
            request = Request(motor_origin + "/api/external-torque",
                data=json.dumps({"torque": torque, "duration": 0.3}).encode(),
                headers={"Origin": motor_origin, "Content-Type": "application/json"})
            with urlopen(request, timeout=2) as response:
                assert response.status == 200

        def stationary():
            snapshot = state()
            return (len(snapshot["motors"]) == 2 and all(
                motor["connected"] and motor["feedback_fresh"] and abs(motor["velocity"]) < 0.05
                for motor in snapshot["motors"].values()))

        def verify_feedforward_components(snapshot):
            for role in ("a", "b"):
                self.assertAlmostEqual(snapshot["target_" + role]["torque"],
                    snapshot["friction_torque_" + role] + snapshot["quantization_torque_" + role], delta=1e-9)
                assert abs(snapshot["friction_torque_" + role]) <= 0.2

        self.wait(ready)
        assert all(motor["simulated"] for motor in state()["motors"].values())
        self.wait(lambda: state()["state"]["compensation_ready_a"]
                  and state()["state"]["compensation_ready_b"], timeout=3)
        initial = state()["state"]
        assert initial["compensation_ready_a"] and initial["compensation_ready_b"], {
            key: value for key, value in initial.items()
            if key.startswith("compensation_") or key == "system_parameters_file"}
        report = {"hardware_used": False}
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": 1440, "height": 1000})
            errors, writes = [], []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.on("request", lambda request: writes.append(request.url) if request.method == "POST" else None)
            try:
                page.goto(origin)
                expect(page.locator("#start")).to_be_enabled()
                assert not writes
                compensation = page.locator("#friction_compensation_enabled")
                assert not compensation.is_checked()
                assert float(page.locator("#max_torque").input_value()) == 0
                compensation.check()
                page.get_by_text("补偿比例与上限", exact=True).click()
                page.locator("#friction_compensation_fraction").fill("0.6")
                page.locator("#friction_torque_limit").fill("0.2")
                page.locator("#start").click()
                expect(page.locator("#phase")).to_have_text("双边耦合中")
                expect(compensation).to_be_disabled()
                self.wait(lambda: state()["state"]["gain_scale"] == 1 and any(
                    abs(state()["state"]["quantization_torque_" + role]) > 1e-5 for role in ("a", "b")))
                snapshot = state()["state"]
                assert snapshot["compensation_ready_a"] and snapshot["compensation_ready_b"]
                assert snapshot["compensation_active"]
                assert snapshot["compensation_model_a"] == snapshot["compensation_model_b"] == "coulomb_viscous"
                assert snapshot["system_parameters_file"] == str(self.friction_registry)
                assert snapshot["friction_torque_a"] == snapshot["friction_torque_b"] == 0
                assert not snapshot["application_torque_limited"]
                verify_feedforward_components(snapshot)
                expect(page.locator("#compensation-summary")).to_contain_text("两轴补偿运行中")
                for role in ("a", "b"):
                    expect(page.locator("#" + role + "-model")).to_have_text("库仑 + 粘性")
                    expect(page.locator("#" + role + "-domain")).to_have_text("-3.000 ～ 3.000")
                    expect(page.locator("#" + role + "-source")).not_to_have_text("未匹配模型来源")
                report["explicit_compensation_start_loads_two_simulation_models"] = True

                self.wait(lambda: state()["state"]["friction_torque_a"] > 0.01,
                          timeout=3, action=lambda: force(0.24))
                snapshot = state()["state"]
                assert 0 < snapshot["friction_torque_a"] <= 0.2
                assert snapshot["gain_scale"] == 1
                assert not snapshot["application_torque_limited"]
                verify_feedforward_components(snapshot)
                expect(page.locator("#a-friction")).not_to_have_text("—")
                expect(page.locator("#a-quantization")).not_to_have_text("—")
                report["measured_motion_produces_bounded_friction_feedforward"] = True
                report["zero_application_budget_preserves_gains_and_separates_encoding_correction"] = True
                for width in (1440, 768, 390, 320):
                    page.set_viewport_size({"width": width, "height": 900})
                    assert page.evaluate("document.documentElement.scrollWidth <= innerWidth"), width
                page.set_viewport_size({"width": 1440, "height": 1000})
                report["model_metadata_and_compensation_controls_fit_mobile"] = True
                output = os.environ.get("DM_BILATERAL_BROWSER_OUTPUT")
                if output:
                    Path(output).mkdir(parents=True, exist_ok=True)
                    page.screenshot(path=str(Path(output) / "bilateral-friction-dashboard.png"), full_page=True)
                force(0.0)
                page.locator("#stop").click()
                expect(page.locator("#start")).to_be_enabled()
                assert state()["state"]["stop_confirmed"]
                expect(compensation).to_be_enabled()

                # Disabling friction leaves the known MIT encoding correction enabled.
                self.wait(stationary)
                compensation.uncheck()
                page.locator("#start").click()
                expect(page.locator("#phase")).to_have_text("双边耦合中")
                self.settle(0.25, action=lambda: force(0.24))
                snapshot = state()["state"]
                assert not snapshot["compensation_active"]
                assert snapshot["friction_torque_a"] == snapshot["friction_torque_b"] == 0
                assert any(abs(snapshot["quantization_torque_" + role]) > 1e-5 for role in ("a", "b"))
                verify_feedforward_components(snapshot)
                report["explicit_friction_disable_preserves_only_encoding_correction"] = True
                force(0.0)
                page.locator("#stop").click()
                expect(page.locator("#start")).to_be_enabled()

                self.wait(stationary)
                compensation.check()
                page.locator("#start").click()
                expect(page.locator("#phase")).to_have_text("双边耦合中")
                self.wait(lambda: state()["state"]["friction_torque_a"] > 0.01,
                          timeout=3, action=lambda: force(0.24))
                page.goto("about:blank")
                self.wait(lambda: state()["state"]["stop_confirmed"], timeout=3)
                snapshot = state()["state"]
                assert snapshot["friction_torque_a"] == snapshot["friction_torque_b"] == 0
                assert snapshot["quantization_torque_a"] == snapshot["quantization_torque_b"] == 0
                assert snapshot["target_a"]["torque"] == snapshot["target_b"]["torque"] == 0
                report["page_exit_clears_active_compensation_and_stops"] = True
                assert not errors, errors
                report["page_errors"] = errors
                print(json.dumps(report, indent=2))
                if output:
                    (Path(output) / "bilateral-friction-browser-results.json").write_text(json.dumps(report, indent=2) + "\n")
            finally:
                browser.close()


if __name__ == "__main__":
    unittest.main()
