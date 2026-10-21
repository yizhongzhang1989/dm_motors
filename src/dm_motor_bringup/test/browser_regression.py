#!/usr/bin/env python3
"""Optional real-browser regression; launches only isolated virtual devices."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

import yaml
from common import ConfigManager


def ports(count):
    sockets = []
    try:
        for _ in range(count):
            endpoint = socket.socket()
            endpoint.bind(("127.0.0.1", 0))
            sockets.append(endpoint)
        return [endpoint.getsockname()[1] for endpoint in sockets]
    finally:
        for endpoint in sockets:
            endpoint.close()


def state(origin, path="/api/state"):
    with urlopen(origin + path, timeout=2) as response:
        return json.load(response)


def post(origin, path, body):
    with urlopen(Request(origin + path, data=json.dumps(body).encode(),
                         headers={"Content-Type": "application/json", "Origin": origin}), timeout=3) as response:
        return json.load(response)


def wait(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("Browser regression condition timed out")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional directory for JSON report and screenshots")
    parser.add_argument("--legacy-ui", action="store_true", help="Also run the existing motor/CAN static UI suite")
    args = parser.parse_args()
    try:
        from playwright.sync_api import expect, sync_playwright
    except ImportError as error:
        raise SystemExit("Install optional playwright and its Chromium browser before running this UI test") from error

    can_port, motor_port, second_port, follow_port = ports(4)
    motor_origin = f"http://127.0.0.1:{motor_port}"
    follow_origin = f"http://127.0.0.1:{follow_port}"
    namespace = f"browser_physics_{os.getpid()}"
    environment = dict(os.environ, ROS_DOMAIN_ID=str(170 + os.getpid() % 20), ROS_LOCALHOST_ONLY="1")
    static = None
    if args.legacy_ui:
        source = Path(__file__).resolve().parents[2]
        static = ThreadingHTTPServer(("127.0.0.1", 0), partial(SimpleHTTPRequestHandler, directory=str(source)))
        static_thread = threading.Thread(target=static.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        static_thread.start()
    report = {}
    with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode="w+") as log:
        document = yaml.safe_load(ConfigManager().path.read_text())
        variant = document["packages"]["dm_motor_bringup"]["launches"]["lead_follow.launch.py"]["variants"]["simulation"]
        variant["system"]["motors"] = {"motor_2": {"direction": -1.0, "software_offset": 0.4}}
        config = Path(directory) / "browser.yaml"
        config.write_text(yaml.safe_dump(document))
        process = subprocess.Popen([
            "ros2", "launch", "dm_motor_bringup", "lead_follow.launch.py", "simulation:=true", f"namespace:={namespace}",
            f"config_file:={config}",
            f"can_dashboard_ports:={{main: {can_port}}}",
            f"motor_dashboard_ports:={{motor_1: {motor_port}, motor_2: {second_port}}}",
            f"dashboard_port:={follow_port}", "dashboard_host:=127.0.0.1",
        ], env=environment, stdout=log, stderr=subprocess.STDOUT)
        try:
            def ready():
                if process.poll() is not None:
                    raise RuntimeError("Virtual launch exited before readiness")
                try:
                    for origin in (motor_origin, f"http://127.0.0.1:{second_port}"):
                        snapshot = state(origin)
                        simulation = state(origin, "/api/simulation")
                        if not (snapshot["connected"] and (snapshot.get("capabilities") or {}).get("ready")
                                and (snapshot["state"] or {}).get("simulated")
                                and simulation["connected"] and (simulation["state"] or {}).get("virtual_bus_confirmed")):
                            return False
                    follow = state(follow_origin)
                    bus = state(f"http://127.0.0.1:{can_port}")["status"] or {}
                except (URLError, TimeoutError):
                    return False
                return (follow["connected"] and (follow["state"] or {}).get("stop_confirmed")
                        and bus.get("backend") == "virtual" and bus.get("port") == "")
            wait(ready, 25)
            assert state(motor_origin)["state"]["simulated"]
            assert state(f"http://127.0.0.1:{second_port}")["state"]["simulated"]
            bus = state(f"http://127.0.0.1:{can_port}")["status"]
            assert bus["backend"] == "virtual" and bus["port"] == ""
            report["virtual_scene_ready"] = True
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(headless=True)
                context = browser.new_context(viewport={"width": 1440, "height": 1000})
                page = context.new_page()
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                try:
                    if static is not None:
                        page.goto(f"http://127.0.0.1:{static.server_port}/dm_motor_control/test/browser.html")
                        expect(page.locator("#results")).not_to_have_attribute("data-status", "running", timeout=60000)
                        if page.locator("#results").get_attribute("data-status") != "passed":
                            raise AssertionError(page.locator("#results").inner_text())
                        report["existing_motor_and_can_ui"] = json.loads(page.locator("#results").inner_text())

                    mutations = []
                    page.on("request", lambda request: mutations.append(request.url) if request.method == "POST" else None)
                    page.goto(motor_origin)
                    dial = page.locator("#shaft-disc")
                    expect(dial).to_have_attribute("aria-disabled", "false")
                    assert not mutations, mutations
                    assert page.locator("#tab-simulation").count() == 0
                    report["monitoring_sends_no_commands"] = True

                    def dial_point(angle, radius=0.35):
                        bounds = dial.bounding_box()
                        assert bounds is not None
                        return (bounds["x"] + bounds["width"] * (0.5 + radius * math.sin(angle)),
                                bounds["y"] + bounds["height"] * (0.5 - radius * math.cos(angle)))

                    def drag_shaft(delta):
                        dial.scroll_into_view_if_needed()
                        page.mouse.move(*dial_point(0))
                        page.mouse.down()
                        for step in range(1, 9):
                            page.mouse.move(*dial_point(delta * step / 8))

                    before = state(motor_origin)["state"]["position"]
                    other_before = state(f"http://127.0.0.1:{second_port}")["state"]["position"]
                    target_before = state(motor_origin)["target"]
                    drag_shaft(0.4)
                    wait(lambda: state(motor_origin)["state"]["position"] > before + 0.02)
                    page.mouse.up()
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0)
                    assert state(motor_origin)["state"]["status"] == 0
                    assert state(motor_origin)["target"] == target_before
                    assert abs(state(f"http://127.0.0.1:{second_port}")["state"]["position"] - other_before) < 0.001
                    assert all(url.endswith(("/api/shaft-drag", "/api/external-torque")) for url in mutations)
                    report["drag_moves_disabled_shaft_without_motor_targets"] = True

                    wait(lambda: abs(state(motor_origin)["state"]["velocity"]) < 0.05)
                    drag_shaft(-0.3)
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] < -0.05)
                    dial.dispatch_event("pointercancel", {"pointerId": 1})
                    page.mouse.up()
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0)
                    report["reverse_drag_and_pointer_cancel"] = True

                    for response_delayed in (False, True):
                        delayed = []
                        def delay_force(route):
                            delayed.append((route, route.fetch() if response_delayed else None))
                        page.route("**/api/shaft-drag", delay_force)
                        drag_shaft(0.3)
                        assert len(delayed) == 1
                        with page.expect_response(lambda response: response.url.endswith("/api/external-torque")) as cleared:
                            page.mouse.up()
                        assert cleared.value.ok
                        route, response = delayed[0]
                        if response_delayed:
                            assert response.ok
                            route.fulfill(response=response)
                        else:
                            route.continue_()
                        page.wait_for_timeout(450)
                        assert len(delayed) == 1
                        assert state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0
                        page.unroute("**/api/shaft-drag", delay_force)
                    report["late_request_and_late_success_cannot_restart_drag"] = True

                    def hardware_view(route):
                        response = route.fetch()
                        body = response.json()
                        body["state"]["simulated"] = False
                        body["simulation"]["enabled"] = False
                        route.fulfill(response=response, json=body)
                    page.route("**/api/state", hardware_view)
                    expect(dial).to_have_attribute("aria-disabled", "true")
                    count = len(mutations)
                    drag_shaft(0.4)
                    page.mouse.up()
                    dial.dblclick()
                    page.wait_for_timeout(200)
                    assert len(mutations) == count
                    old_rotation = page.locator("#shaft-rotor").get_attribute("transform")
                    post(motor_origin, "/api/external-torque", {"torque": 0.08, "duration": 0.15})
                    expect(page.locator("#shaft-rotor")).not_to_have_attribute("transform", old_rotation)
                    assert len(mutations) == count
                    page.unroute("**/api/state", hardware_view)
                    expect(dial).to_have_attribute("aria-disabled", "false")
                    report["hardware_view_is_readonly_and_tracks_feedback"] = True

                    wait(lambda: abs(state(motor_origin)["state"]["velocity"]) < 0.05)
                    page.locator("#tab-control").click()
                    page.locator("#enable").click()
                    expect(page.locator("#phase")).to_contain_text("ENABLED")
                    page.locator("#live-targets").uncheck()
                    target = state(motor_origin)["state"]["position"] + 0.05
                    page.locator("#target-position").fill(str(target))
                    page.locator("#target-kp").fill("4")
                    page.locator("#target-kd").fill("0.5")
                    page.locator("#apply").click()
                    wait(lambda: abs(state(motor_origin)["state"]["position"] - target) < 0.02)
                    # A held target makes the load sustained instead of a brief free-rotor force peak.
                    held_target = state(motor_origin)["target"]
                    drag_shaft(2.0)
                    wait(lambda: abs(state(motor_origin, "/api/simulation")["state"]["external_torque"]) > 1.05)
                    assert state(motor_origin)["target"] == held_target
                    page.mouse.up()
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0)
                    report["drag_force_exceeds_previous_one_nm_cap"] = True
                    page.locator("#stop").click()
                    wait(lambda: state(motor_origin)["state"]["status"] == 0)
                    report["production_motor_ui_position_control"] = True

                    for width in (1440, 1366, 1024, 900, 768, 550, 390, 320):
                        page.set_viewport_size({"width": width, "height": 900})
                        assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
                        assert dial.is_visible()
                        target_dial = page.locator("#position-scroll").bounding_box()
                        shaft = dial.bounding_box()
                        position_input = page.locator(".position-input").bounding_box()
                        ring = page.locator("#shaft-disc > circle").first.bounding_box()
                        assert all(box is not None for box in (target_dial, shaft, position_input, ring))
                        assert (target_dial["x"] + target_dial["width"] <= shaft["x"] + 1
                                or target_dial["y"] + target_dial["height"] <= shaft["y"] + 1)
                        assert position_input["y"] >= max(target_dial["y"] + target_dial["height"],
                                                          shaft["y"] + shaft["height"])
                        if width == 1440:
                            assert target_dial["width"] >= 230 and ring["width"] >= 180
                        if width <= 390:
                            assert target_dial["width"] >= 230 and ring["width"] >= 165
                    page.set_viewport_size({"width": 1440, "height": 1000})
                    page.evaluate("window.scrollTo(0, 0); document.activeElement.blur()")
                    page.wait_for_timeout(100)
                    if args.output:
                        args.output.mkdir(parents=True, exist_ok=True)
                        page.screenshot(path=str(args.output / "simulation-dashboard.png"), full_page=True)
                    report["desktop_and_mobile_layout"] = True
                    report["large_dials_above_position_input"] = True

                    second_origin = f"http://127.0.0.1:{second_port}"
                    page.goto(second_origin)
                    expect(dial).to_have_attribute("aria-disabled", "false")
                    second_before = state(second_origin)["state"]["position"]
                    native_before = state(second_origin, "/api/simulation")["state"]["position"]
                    assert state(second_origin)["state"]["direction"] == -1
                    drag_shaft(0.35)
                    wait(lambda: state(second_origin)["state"]["position"] > second_before + 0.02)
                    wait(lambda: state(second_origin, "/api/simulation")["state"]["position"] < native_before - 0.02)
                    page.mouse.up()
                    wait(lambda: state(second_origin, "/api/simulation")["state"]["external_torque"] == 0)
                    report["drag_respects_negative_direction_and_software_offset"] = True
                    wait(lambda: abs(state(second_origin)["state"]["velocity"]) < 0.05)
                    post(second_origin, "/api/external-torque", {"torque": 0.01, "duration": 5})
                    wait(lambda: state(second_origin, "/api/simulation")["state"]["transient_torque"] == 0.01)
                    page.wait_for_timeout(50)
                    target_before_anchor = state(second_origin)["target"]
                    mutation_count = len(mutations)
                    page.mouse.dblclick(*dial_point(0.7))
                    expect(page.locator("#shaft-anchor")).to_have_attribute("display", "inline")
                    assert mutations[mutation_count:] == [second_origin + "/api/shaft-anchor"], (
                        mutations[mutation_count:], state(second_origin, "/api/simulation")["state"])
                    anchored = state(second_origin, "/api/simulation")["state"]
                    assert anchored["anchor_active"] and anchored["transient_torque"] == 0.01
                    assert abs(anchored["anchor_torque"]) < 0.01
                    marker = page.locator("#shaft-anchor-point")
                    bearing = math.atan2(float(marker.get_attribute("cx")) - 140,
                                         140 - float(marker.get_attribute("cy")))
                    assert abs(bearing - 0.7) < 0.01
                    post(second_origin, "/api/external-torque", {"torque": 0.0, "duration": 0.1})
                    rest = anchored["anchor_position"]
                    drag_shaft(0.5)
                    wait(lambda: state(second_origin, "/api/simulation")["state"]["position"] < rest - 0.04)
                    page.mouse.up()
                    wait(lambda: state(second_origin, "/api/simulation")["state"]["transient_torque"] == 0
                         and state(second_origin, "/api/simulation")["state"]["external_remaining"] == 0)
                    released = state(second_origin, "/api/simulation")["state"]
                    assert released["anchor_active"] and released["anchor_torque"] > 0.01
                    assert released["external_remaining"] == 0
                    assert state(second_origin)["target"] == target_before_anchor
                    assert state(second_origin)["state"]["status"] == 0
                    if args.output:
                        page.screenshot(path=str(args.output / "anchor-dashboard.png"), full_page=True)
                    bearing_after = math.atan2(float(marker.get_attribute("cx")) - 140,
                                               140 - float(marker.get_attribute("cy")))
                    assert abs(bearing_after - bearing) < 0.01
                    page.locator("#stop").click()
                    assert state(second_origin, "/api/simulation")["state"]["anchor_active"]
                    page.goto("about:blank")
                    page.wait_for_timeout(350)
                    assert state(second_origin, "/api/simulation")["state"]["anchor_active"]
                    page.goto(second_origin)
                    expect(page.locator("#shaft-anchor")).to_have_attribute("display", "inline")
                    with page.expect_response(lambda response: response.url.endswith("/api/shaft-anchor")) as removed:
                        marker.dblclick()
                    assert removed.value.ok
                    expect(page.locator("#shaft-anchor")).to_have_attribute("display", "none")
                    wait(lambda: not state(second_origin, "/api/simulation")["state"]["anchor_active"])
                    report["double_click_anchor_persists_after_drag_stop_and_page_exit"] = True
                    report["anchor_matches_negative_axis_coordinates"] = True

                    delayed_anchor = []
                    page.route("**/api/shaft-anchor", lambda route: delayed_anchor.append(route))
                    dial.scroll_into_view_if_needed()
                    with page.expect_request(lambda request: request.url.endswith("/api/shaft-anchor")):
                        page.mouse.dblclick(*dial_point(0.7))
                    page.wait_for_timeout(50)
                    assert len(delayed_anchor) == 1, (len(delayed_anchor), page.locator("#shaft-status").inner_text())
                    current = state(second_origin, "/api/simulation")["state"]
                    post(second_origin, "/api/shaft-anchor", {
                        "active": False, "bearing": 0.0, "revision": current["anchor_revision"],
                        "simulation_time": current["simulation_time"],
                    })
                    with page.expect_response(lambda response: response.url.endswith("/api/shaft-anchor")) as late:
                        delayed_anchor[0].continue_()
                    assert late.value.status == 409
                    page.unroute("**/api/shaft-anchor")
                    page.wait_for_timeout(250)
                    assert not state(second_origin, "/api/simulation")["state"]["anchor_active"]
                    report["late_anchor_creation_cannot_undo_removal"] = True
                    page.goto(motor_origin)
                    expect(dial).to_have_attribute("aria-disabled", "false")

                    drag_shaft(0.4)
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] > 0.05)
                    page.goto("about:blank")
                    page.mouse.up()
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0)
                    report["page_exit_clears_force"] = True

                    page.goto(motor_origin)
                    expect(dial).to_have_attribute("aria-disabled", "false")
                    post(motor_origin, "/api/external-torque", {"torque": 0.01, "duration": 2})
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0.01)
                    page.goto("about:blank")
                    page.wait_for_timeout(400)
                    assert state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0.01
                    post(motor_origin, "/api/external-torque", {"torque": 0, "duration": 0.1})
                    report["observer_exit_does_not_cancel_program_force"] = True

                    wait(lambda: abs(state(motor_origin)["state"]["velocity"]) < 0.05)
                    wait(lambda: abs(state(second_origin)["state"]["velocity"]) < 0.05)
                    page.goto(follow_origin)
                    expect(page.locator("#start")).to_be_enabled()
                    page.locator("#start").click()
                    expect(page.locator("#phase")).to_have_text("正在跟随")
                    initial = state(follow_origin)
                    post(motor_origin, "/api/external-torque", {"torque": 0.08, "duration": 0.15})
                    leader = initial["state"]["leader"]
                    follower = initial["state"]["follower"]
                    wait(lambda: state(follow_origin)["motors"][leader]["position"] >
                         initial["motors"][leader]["position"] + 0.015)
                    wait(lambda: state(follow_origin)["motors"][follower]["position"] >
                         initial["motors"][follower]["position"] + 0.015)
                    page.wait_for_timeout(500)
                    following = state(follow_origin)
                    assert following["state"]["phase"] == 3
                    assert abs(following["state"]["following_error"]) < 0.03
                    page.locator("#stop").click()
                    expect(page.locator("#start")).to_be_enabled()
                    assert state(follow_origin)["state"]["stop_confirmed"]
                    report["follow_ui_with_physical_disturbance"] = True
                    page.goto(motor_origin)
                    expect(dial).to_have_attribute("aria-disabled", "false")
                    hands = []
                    page.on("request", lambda request: hands.append(request.post_data_json)
                            if request.url.endswith("/api/shaft-drag") else None)
                    native = state(motor_origin, "/api/simulation")["state"]["position"]
                    drag_shaft(6 * math.pi + 0.3)
                    page.wait_for_timeout(100)
                    page.mouse.up()
                    assert hands and hands[-1]["position"] - native > 4 * math.pi + 1
                    wait(lambda: state(motor_origin, "/api/simulation")["state"]["external_torque"] == 0)
                    report["drag_unwraps_multiple_turns"] = True
                    report["drag_has_no_two_turn_input_cap"] = True
                    assert not errors, errors
                    report["browser_errors"] = errors
                    report["hardware_used"] = False
                finally:
                    browser.close()
            print(json.dumps(report, indent=2, ensure_ascii=False))
            if args.output:
                (args.output / "browser-results.json").write_text(
                    json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        finally:
            children = subprocess.run(["ps", "-o", "pid=", "--ppid", str(process.pid)],
                                      capture_output=True, text=True, check=False)
            child_pids = [int(pid) for pid in children.stdout.split()]
            try:
                if process.poll() is None and report.get("virtual_scene_ready"):
                    for origin, path, body in (
                        (motor_origin, "/api/external-torque", {"torque": 0, "duration": 0.1}),
                        (follow_origin, "/api/stop", {}),
                    ):
                        try:
                            post(origin, path, body)
                        except (URLError, TimeoutError) as error:
                            print(f"Cleanup request failed; stopping the owned virtual processes: {error}", file=sys.stderr)
                    wait(lambda: state(follow_origin)["state"]["stop_confirmed"])
                    for pid in child_pids:
                        command = Path(f"/proc/{pid}/cmdline")
                        try:
                            if b"/lead_follow_node\0" in command.read_bytes():
                                os.kill(pid, signal.SIGINT)
                                wait(lambda: not command.exists(), timeout=5)
                        except FileNotFoundError:
                            pass
                        except ProcessLookupError:
                            pass
            finally:
                for pid in child_pids:
                    try:
                        os.kill(pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        for pid in child_pids:
                            command = Path(f"/proc/{pid}/cmdline")
                            try:
                                if namespace.encode() in command.read_bytes():
                                    os.kill(pid, signal.SIGKILL)
                            except FileNotFoundError:
                                pass
                            except ProcessLookupError:
                                pass
                        process.kill()
                        process.wait(timeout=3)
                if static is not None:
                    static.shutdown()
                    static.server_close()
                    static_thread.join(timeout=1)
                log.seek(0)
                if not report.get("follow_ui_with_physical_disturbance"):
                    print("Virtual launch log:\n" + log.read())


if __name__ == "__main__":
    main()
