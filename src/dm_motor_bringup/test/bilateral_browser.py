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


class BilateralBrowser(PhysicalMotorScene):
    controller_name = "bilateral_control"
    controller_state_type = BilateralState

    def configure_scene(self, document, profile):
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
                assert page.locator("#stiffness").input_value() == "8"
                assert not page.locator("#motion_limits_enabled").is_checked()
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


if __name__ == "__main__":
    unittest.main()
