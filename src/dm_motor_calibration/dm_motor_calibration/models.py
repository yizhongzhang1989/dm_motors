"""Resisting friction in output-shaft SI coordinates; compensation has the same sign."""

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray


MODELS = ("viscous", "coulomb_viscous", "stribeck",
          "asymmetric_coulomb_viscous", "asymmetric_stribeck")
TABLE_MODEL = "tabulated"


def friction_names(model: str) -> tuple[str, ...]:
    if model == TABLE_MODEL:
        return ()
    if model not in MODELS:
        raise ValueError(f"Unknown friction model {model!r}; expected one of {MODELS}")
    names = ("viscous",) if model == "viscous" else ("coulomb", "viscous")
    if "stribeck" in model:
        names += ("static", "stribeck_velocity")
    if model.startswith("asymmetric"):
        return tuple(f"{name}_{direction}" for direction in ("positive", "negative") for name in names)
    return names


@dataclass(frozen=True)
class FrictionModel:
    name: str
    parameters: Mapping[str, float]
    table: tuple[tuple[float, float, float, float], ...] = ()

    def __post_init__(self) -> None:
        expected = friction_names(self.name)
        if set(self.parameters) != set(expected):
            raise ValueError(f"{self.name} requires exactly {expected}")
        if any(isinstance(v, bool) or not math.isfinite(v) or v < 0 for v in self.parameters.values()):
            raise ValueError("Friction parameters must be finite and nonnegative")
        for suffix in ("_positive", "_negative") if self.name.startswith("asymmetric") else ("",):
            if "stribeck" in self.name:
                if self.parameters["stribeck_velocity" + suffix] <= 0:
                    raise ValueError("Stribeck velocity must be positive")
                if self.parameters["static" + suffix] < self.parameters["coulomb" + suffix]:
                    raise ValueError("Static friction must be at least Coulomb friction")
        object.__setattr__(self, "parameters", dict(self.parameters))
        if self.name == TABLE_MODEL:
            if any(isinstance(value, bool) or not isinstance(value, (int, float))
                   for row in self.table for value in row):
                raise ValueError("Friction table values must be numbers, not coerced strings or booleans")
            table = np.asarray(self.table, dtype=float)
            if (table.ndim != 2 or table.shape[0] < 3 or table.shape[1] != 4 or
                    not np.all(np.isfinite(table)) or np.any(table[:, (0, 2)] <= 0) or
                    np.any(np.diff(table[:, 0]) <= 0) or np.any(np.diff(table[:, 2]) <= 0) or
                    np.any(table[:, (1, 3)] < 0)):
                raise ValueError("Tabulated friction needs ordered positive speeds and nonnegative directional torques")
        elif self.table:
            raise ValueError("Only tabulated models may contain friction knots")

    def torque(self, velocity: ArrayLike) -> NDArray[np.float64]:
        v = np.asarray(velocity, dtype=float)
        if not np.all(np.isfinite(v)):
            raise ValueError("Velocity must be finite")
        if self.name == TABLE_MODEL:
            table = np.asarray(self.table)
            positive = np.interp(np.abs(v), table[:, 0], table[:, 1])
            negative = np.interp(np.abs(v), table[:, 2], table[:, 3])
            return np.where(v > 0, positive, np.where(v < 0, -negative, 0.0))
        result = np.zeros_like(v)
        for suffix in ("_positive", "_negative") if self.name.startswith("asymmetric") else ("",):
            p = self.parameters
            dry = np.full_like(v, p.get("coulomb" + suffix, 0.0))
            if "stribeck" in self.name:
                dry += (p["static" + suffix] - p["coulomb" + suffix]) * np.exp(
                    -np.square(v / p["stribeck_velocity" + suffix]))
            branch = np.sign(v) * dry + p["viscous" + suffix] * v
            mask = v >= 0 if suffix == "_positive" else v < 0 if suffix else np.ones_like(v, dtype=bool)
            result += np.where(mask, branch, 0.0)
        return result

    def at(self, velocity: float) -> float:
        return float(self.torque(velocity))
