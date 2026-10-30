"""Measured pre-run evidence, deliberately separate from friction identification."""

from __future__ import annotations

from dataclasses import asdict
import math
from typing import TYPE_CHECKING, Callable, Sequence

from .data import CalibrationError
from .conditioning import ChirpPlan, SineEvidence

if TYPE_CHECKING:
    from .precision import Cell, Sample


class WarmupCell:
    def __init__(self, index: int, cell: Cell, turns: int):
        self.index, self.cell, self.turns = index, cell, turns
        self.first: Sample | None = None
        self.last: Sample | None = None
        self.count = 0
        self.temperature_min = math.inf
        self.temperature_max = -math.inf

    @property
    def complete(self) -> bool:
        return (self.first is not None and self.last is not None and
                self.cell.direction * (self.last.position - self.first.position) >= self.turns * math.tau)

    def ingest(self, sample: Sample) -> None:
        if self.complete:
            raise CalibrationError("Warmup cell contains samples beyond its completed rotation boundary")
        self.first = sample if self.first is None else self.first
        self.last = sample
        self.count += 1
        self.temperature_min = min(self.temperature_min, sample.temperature)
        self.temperature_max = max(self.temperature_max, sample.temperature)

    def report(self) -> dict:
        if self.first is None or self.last is None:
            raise CalibrationError("Warmup cell has no measured samples")
        distance = self.last.position - self.first.position
        duration = self.last.time - self.first.time
        return {**asdict(self.cell), "cell": self.index, "complete": self.complete,
                "required_turns": self.turns, "measured_turns": self.cell.direction * distance / math.tau,
                "angle": abs(distance), "duration": duration, "samples": self.count,
                "mean_speed": distance / duration if duration > 0 else None,
                "start_time": self.first.time, "end_time": self.last.time,
                "temperature_start": self.first.temperature, "temperature_end": self.last.temperature,
                "temperature_range": [self.temperature_min, self.temperature_max]}


class WarmupEvidence:
    def __init__(self, cells: Sequence[Cell], turns: Callable[[float], int], zero: float, tolerance: float,
                 sine_plan: ChirpPlan | None = None):
        self.plan, self.turns, self.zero, self.tolerance = cells, turns, zero, tolerance
        self.cells: dict[int, WarmupCell] = {}
        self.first: Sample | None = None
        self.last: Sample | None = None
        self.samples = 0
        self.temperature_min = math.inf
        self.temperature_max = -math.inf
        self.zero_sample: Sample | None = None
        self.sine = SineEvidence(sine_plan) if sine_plan is not None else None

    def ingest(self, sample: Sample, index: int = -1, *, sine: bool = False) -> bool:
        if not self.plan:
            raise CalibrationError("Unexpected warmup sample without a configured pre-run plan")
        self.first = sample if self.first is None else self.first
        self.last = sample
        self.samples += 1
        self.temperature_min = min(self.temperature_min, sample.temperature)
        self.temperature_max = max(self.temperature_max, sample.temperature)
        if sine:
            if self.sine is None or index >= 0:
                raise CalibrationError("Sine conditioning overlaps a rotation cell or has no configured plan")
            self.sine.ingest(sample)
            return False
        if index < 0:
            return False
        if not 0 <= index < len(self.plan) or (self.cells and index < max(self.cells)):
            raise CalibrationError("Warmup sample membership is outside the ordered pre-run plan")
        if index not in self.cells:
            cell = self.plan[index]
            self.cells[index] = WarmupCell(index, cell, self.turns(cell.speed))
        window = self.cells[index]
        window.ingest(sample)
        return window.complete

    @property
    def completed_cells(self) -> int:
        return sum(cell.complete for cell in self.cells.values())

    def confirm_zero(self, sample: Sample) -> None:
        if (self.completed_cells != len(self.plan) or abs(sample.position - self.zero) >= self.tolerance or
                (self.sine is not None and not self.sine.complete)):
            raise CalibrationError("Warmup cannot finish before all rotations and the measured zero return")
        self.zero_sample = sample

    def report(self, *, include_cells: bool = True) -> dict:
        result = {"enabled": bool(self.plan), "complete": bool(self.plan) and self.completed_cells == len(self.plan) and
                  self.zero_sample is not None and (self.sine is None or self.sine.complete),
                "required_cells": len(self.plan), "completed_cells": self.completed_cells,
                "samples": self.samples, "zero_position": self.zero,
                "zero_return_time": self.zero_sample.time if self.zero_sample is not None else None,
                "zero_return_position": self.zero_sample.position if self.zero_sample is not None else None,
                "start_time": self.first.time if self.first is not None else None,
                "end_time": self.last.time if self.last is not None else None,
                "duration": self.last.time - self.first.time if self.first is not None and self.last is not None else 0.0,
                "temperature_range": [self.temperature_min, self.temperature_max] if self.samples else None,
                "thermal_equilibrium_verified": False,
                "interpretation": "Measured motion coverage is verified; integer temperature feedback alone does not certify thermal equilibrium."}
        if include_cells:
            result["cells"] = [self.cells[index].report() for index in sorted(self.cells)]
        if self.sine is not None:
            result["sine"] = self.sine.report(include_cycles=include_cells)
        return result
