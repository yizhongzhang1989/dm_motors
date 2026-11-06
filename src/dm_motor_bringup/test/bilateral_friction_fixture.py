"""Synthetic, namespace-bound simulation models; never use hardware calibration."""

from copy import deepcopy
from pathlib import Path

import yaml


def simulation_calibrations(scene, document, profile, *, missing=False, wrong_identity=False, velocity_limit=3.0):
    """Install two distinct virtual friction models beside this scene's runtime YAML."""
    registry = {"version": 1, "motors": {}}
    for index, name in enumerate(("motor_1", "motor_2"), 1):
        motor = {**document["packages"]["dm_motor_control"]["nodes"]["motor_node"],
                 **profile.get("nodes", {}).get("dm_motor_control", {}).get("motor_node", {}),
                 **profile.get("system", {}).get("motors", {}).get(name, {})}
        coulomb, viscous = (0.06, 0.02) if index == 1 else (0.09, 0.025)
        profile["devices"][name].update(
            coulomb_friction=coulomb, viscous_friction=viscous,
            static_friction=1.2 * coulomb, friction_model="coulomb_viscous")
        entry = {
            "schema": 3, "qualified": True,
            "identity": {"motor_id": index, "mode": 1,
                         "direction": motor.get("direction", 1.0),
                         "software_offset": motor.get("software_offset", 0.0),
                         "pmax": motor.get("pmax", 12.5), "vmax": motor.get("vmax", 50.0),
                         "tmax": motor.get("tmax", 10.0)},
            "fit": {
                "model": "coulomb_viscous",
                # Large non-friction terms make accidental load/inertia compensation observable.
                "parameters": {"inertia": 0.8, "bias": 0.17, "gravity_sin": 0.3,
                               "gravity_cos": -0.2, "stiffness": 0.4,
                               "coulomb": coulomb, "viscous": viscous},
                "units": {"inertia": "kg*m^2", "bias": "Nm", "gravity_sin": "Nm",
                          "gravity_cos": "Nm", "stiffness": "Nm/rad",
                          "coulomb": "Nm", "viscous": "Nm/(rad/s)"},
                "domain": {"velocity_min": -velocity_limit, "velocity_max": velocity_limit},
            },
        }
        if wrong_identity and index == 2:
            entry["identity"]["motor_id"] = 99
        if not (missing and index == 2):
            registry["motors"][scene.namespace + "/" + name] = {
                "simulation": {"calibrations": {"mechanical": entry}},
                # The hardware branch must never rescue a missing simulation model.
                "hardware": {"calibrations": {"mechanical": deepcopy(entry)}},
            }
        elif missing:
            registry["motors"][scene.namespace + "/" + name] = {
                "hardware": {"calibrations": {"mechanical": deepcopy(entry)}}}
    path = Path(scene.directory.name) / "bilateral-simulation-parameters.yaml"
    path.write_text(yaml.safe_dump(registry, sort_keys=False))
    document["system_parameters_file"] = str(path)
    return path
