"""Light-touch robustness fixture using the production controller and virtual devices.

The inertias and Coulomb terms freeze the 2026-10-30 mechanical calibration for
repeatable simulation. The 2 ms velocity-estimator lag is a robustness assumption,
not an identified hardware parameter. Physical velocity remains independently
observable through SimulationState, including after external torque is removed.
"""

import csv
import json
import math
import os
from pathlib import Path
from statistics import mean
import time

import yaml

from dm_motor_interfaces.msg import BilateralState


TOUCH_AXES = (
    {"inertia": 0.0038742723463633427, "coulomb": 0.1416881637628295,
     "velocity_min": -0.4108284, "velocity_max": 0.39145345},
    {"inertia": 0.0014637501871318296, "coulomb": 0.10164556437387359,
     "velocity_min": -0.32715376, "velocity_max": 0.32279489},
)


def configure_touch_scene(scene, document, profile, *, friction_scale=1.0, velocity_tau=0.002):
    """Use a namespace-bound simulation model created by simulation_calibrations."""
    controller = profile["nodes"]["dm_motor_bilateral_control"]["bilateral_control_node"]
    controller.update(control_mode="native", required_velocity_bandwidth=0.0,
                      rate=500.0, stiffness=50.0, coupling_damping=0.995,
                      local_damping=0.005, max_torque=0.0,
                      friction_compensation_enabled=True, friction_compensation_fraction=0.6,
                      friction_torque_limit=0.2)
    profile["nodes"]["dm_motor_control"]["motor_node"]["startup_velocity_bandwidth"] = 0.0
    document["packages"]["dm_motor_bringup"]["launches"]["bilateral_control.launch.py"][
        "parameters"]["control_rate"] = "500"
    models = yaml.safe_load(scene.friction_registry.read_text())
    for index, name in enumerate(("motor_1", "motor_2")):
        axis = TOUCH_AXES[index]
        physical_coulomb = friction_scale * axis["coulomb"]
        profile["devices"][name].update(
            state_rate=500.0, inertia=axis["inertia"], coulomb_friction=physical_coulomb,
            viscous_friction=0.0, static_friction=(1.2 if friction_scale == 1.0 else 1.0) * physical_coulomb,
            torque_time_constant=0.002, velocity_time_constant=velocity_tau)
        fit = models["motors"][scene.namespace + "/" + name]["simulation"][
            "calibrations"]["mechanical"]["fit"]
        fit["parameters"].update(coulomb=axis["coulomb"], viscous=0.0)
        fit["domain"].update(velocity_min=axis["velocity_min"], velocity_max=axis["velocity_max"])
    scene.friction_registry.write_text(yaml.safe_dump(models, sort_keys=False))


def _reversals(values):
    # Ignore numerical zero and require motion beyond a small physical threshold.
    signs = [1 if value > 0 else -1 for value in values if abs(value) > 0.01]
    return sum(a != b for a, b in zip(signs, signs[1:]))


def release_metrics(rows):
    released = [row for row in rows if row["stage"] == "released"]
    metrics = {"samples": len(rows), "released_samples": len(released)}
    if not released:
        return metrics
    end = released[-1]["time_a"]
    tail = [row for row in released if row["time_a"] >= end - 1.0]
    metrics.update(
        released_duration=released[-1]["time_a"] - released[0]["time_a"],
        tail_velocity_rms=[math.sqrt(mean(row["v_" + a] ** 2 for row in tail)) for a in "ab"],
        tail_velocity_peak=[max(abs(row["v_" + a]) for row in tail) for a in "ab"],
        tail_position_range=[max(row["q_" + a] for row in tail) - min(row["q_" + a] for row in tail)
                             for a in "ab"],
        tail_velocity_reversals=[_reversals([row["v_" + a] for row in tail]) for a in "ab"],
        peak_travel=[max(abs(row["q_" + a] - rows[0]["q_" + a]) for row in rows) for a in "ab"],
        released_external_torque_peak=max(abs(row["external_" + a]) for row in released for a in "ab"),
        tail_energy_mean=mean(row["energy"] for row in tail))
    return metrics


def observe_light_touch(scene, *, fraction, axis=0, push=0.3, duration=0.25, rest=0.25):
    """Observe 3.1 seconds with confirmed zero external torque; never infer stability from ACTIVE."""
    scene.configure(stiffness=50.0, coupling_damping=0.995, local_damping=0.005,
                    max_torque=0.0, friction_compensation_fraction=float(fraction))
    scene.begin()
    names = ("motor_1", "motor_2")
    origins = [scene.physics[name].position for name in names]
    rows, last_step, stage = [], -1, "rest"
    started = time.monotonic()
    try:
        def sample():
            nonlocal last_step, stage
            physics = [scene.physics[name] for name in names]
            if physics[0].integration_steps == last_step:
                return
            last_step = physics[0].integration_steps
            scene.assertEqual(scene.follow.phase, BilateralState.ACTIVE, scene.follow.message)
            if stage == "release_pending" and all(p.external_torque == 0 for p in physics):
                stage = "released"
            error = (physics[0].position - origins[0]) - (physics[1].position - origins[1])
            row = {"elapsed": time.monotonic() - started, "stage": stage, "error": error,
                   "energy": sum(0.5 * TOUCH_AXES[i]["inertia"] * p.velocity ** 2
                                 for i, p in enumerate(physics)) + 25.0 * error ** 2,
                   "pair_age": scene.follow.pair_age, "pair_skew": scene.follow.pair_skew}
            for suffix, name, p in zip("ab", names, physics):
                row.update({"time_" + suffix: p.simulation_time, "q_" + suffix: p.position,
                            "v_" + suffix: p.velocity, "motor_" + suffix: p.motor_torque,
                            "external_" + suffix: p.external_torque,
                            "physical_friction_" + suffix: p.friction_torque,
                            "model_friction_" + suffix: getattr(scene.follow, "friction_torque_" + suffix),
                            "feedback_velocity_" + suffix: scene.states[name].velocity})
                target = scene.states[name].active_target
                for field in ("position", "velocity", "kp", "kd", "torque"):
                    row["target_" + field + "_" + suffix] = getattr(target, field)
            rows.append(row)
            # Virtual-only excursion bounds retain the old 50 Hz failure waveform.
            scene.assertLess(max(abs(p.velocity) for p in physics), 30.0)
            scene.assertLess(max(abs(p.position - origins[i]) for i, p in enumerate(physics)), 1.0)

        scene.settle(rest, action=sample)
        stage = "contact"

        def contact():
            for i, name in enumerate(names):
                scene.force(name, push if i == axis else 0.0, 0.12)
            sample()

        scene.settle(duration, action=contact)
        for name in names:
            scene.force(name, 0.0)
        stage = "release_pending"
        scene.wait(lambda: stage == "released", timeout=0.5, action=sample)
        scene.settle(3.1, action=sample)
        return release_metrics(rows)
    finally:
        for name in names:
            scene.force(name, 0.0)
        if scene.follow.phase == BilateralState.ACTIVE:
            scene.halt()
        output = os.environ.get("BILATERAL_TOUCH_RESULTS_DIR")
        if output and rows:
            directory = Path(output)
            directory.mkdir(parents=True, exist_ok=True)
            stem = scene._testMethodName + "_fraction_" + str(fraction)
            with (directory / (stem + ".csv")).open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            (directory / (stem + ".json")).write_text(json.dumps(release_metrics(rows), indent=2) + "\n")
