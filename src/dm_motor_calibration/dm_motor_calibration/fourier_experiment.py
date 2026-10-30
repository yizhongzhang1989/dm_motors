"""Finite-Fourier experiment using the shared warmup, baseline, static and stop machinery."""

from dataclasses import asdict, replace
import math
from typing import Mapping

from .conditioning import StageLimits
from .data import CalibrationError, FitResult
from .fourier_identification import PeriodBuffer, analyze_fourier
from .fourier_trajectory import FourierSettings, FourierTrajectory, design_pair
from .identification import MechanicalOptions
from .precision import Drive, PrecisionExperiment, PrecisionSettings, Sample
from .experiment import AcquisitionStages


class FourierExperiment(PrecisionExperiment):
    def __init__(self, settings: PrecisionSettings, origin: float, fourier: FourierSettings,
                 options: MechanicalOptions, *, position_resolution: float = 25 / 65535,
                 trajectories: tuple[FourierTrajectory, FourierTrajectory] | None = None,
                 stages: AcquisitionStages | None = None):
        stages = stages or AcquisitionStages()
        fourier = replace(fourier, training_enabled=stages.collect_training,
                          validation_enabled=stages.collect_validation)
        self.fourier_settings = fourier
        self.mechanical_options = options
        self.trajectories = trajectories or design_pair(fourier, options)
        self.fourier_limits = fourier.limits(settings)
        self.fourier_started: float | None = None
        self.fourier_armed = False
        self.fourier_reference_velocity = 0.0
        self.fourier_speed_limited = False
        self.period_rows = PeriodBuffer()
        self.cycle_counts: dict[int, int] = {}
        super().__init__(settings, origin, position_resolution=position_resolution, stages=stages)
        if settings.time_budget < settings.preparation_minimum_duration + fourier.duration:
            raise CalibrationError("Time budget cannot cover preparation and complete Fourier periods")

    @property
    def required_cells(self) -> int:
        return self.fourier_settings.total_cycles

    @property
    def completed_cells(self) -> int:
        if self.fourier_started is None or self.latest is None:
            return 0
        elapsed = self.latest.time - self.fourier_started
        return sum(index in self.cycle_counts and elapsed >= self.fourier_settings.cycle_start(index + 1)
                   for index in range(self.required_cells))

    def observation_limits(self, timestamp: float | None = None) -> StageLimits | PrecisionSettings:
        if self.warmup_sample(timestamp):
            return self.warm_limits
        return self.fourier_limits if self.fourier_armed else self.settings

    def recording_metadata(self) -> dict:
        return {"kind": "fourier_recording", "format": "fourier_csv_v1", "cells": [],
                "acquisition_stages": asdict(self.stages),
                "control_law": "bounded_position_to_native_velocity",
                "fourier_settings": asdict(self.fourier_settings), "mechanical_options": asdict(self.mechanical_options),
                "fourier_trajectories": [asdict(trajectory) for trajectory in self.trajectories]}

    def recording_annotations(self) -> dict:
        return {"fourier_started": self.fourier_started}

    def additional_membership(self, tag: Mapping[str, float] | None, timestamp: float | None) -> dict[str, int]:
        cycle = -1
        if (self.phase == "periodic" and self.fourier_started is not None and timestamp is not None and
                tag is not None and tag.get("fourier_tracking") == 1):
            cycle = self.fourier_settings.sample_cycle(timestamp - self.fourier_started)
        return {"analysis_fourier_cycle": cycle, "analysis_fourier_role": self.fourier_settings.role(cycle)}

    def ingest(self, sample: Sample, tag: Mapping[str, float] | None = None) -> None:
        membership = self.additional_membership(tag, sample.time)
        super().ingest(sample, tag)
        cycle = membership["analysis_fourier_cycle"]
        if cycle >= 0:
            saturated = int(tag.get("saturated", 0) != 0) if tag is not None else 0
            self.period_rows.append(sample, cycle, saturated)
            self.cycle_counts[cycle] = self.cycle_counts.get(cycle, 0) + 1

    def begin_sweep(self, now: float) -> None:
        if self.phase != "review_static" or self.static_diagnostics is None:
            raise CalibrationError("Static acquisition must be reviewed before Fourier excitation")
        if not self.has_sweep:
            self.transition("done", now)
            return
        self.fourier_armed = True
        self.previous_control = None
        self.command_speed = self.integral = 0.0
        self.transition("home", now)

    def finish_formal_home(self, now: float) -> None:
        self.reference_position = self.origin
        self.command_speed = self.integral = 0.0
        if self.fourier_started is None:
            self.fourier_started = now
            self.transition("periodic", now)
        else:
            self.transition("done", now)

    def home_gains(self) -> tuple[float, float]:
        if self.fourier_armed:
            return self.fourier_settings.kp, self.fourier_settings.kd
        return super().home_gains()

    def control(self, now: float) -> Drive:
        if self.phase != "periodic":
            return super().control(now)
        if self.latest is None or self.fourier_started is None:
            raise CalibrationError("Fourier motion has no fresh measurement or reference origin")
        if self.previous_control is not None and (not math.isfinite(now) or now <= self.previous_control):
            raise CalibrationError("Fourier control clock is not strictly increasing")
        self.previous_control = now
        elapsed = now - self.fourier_started
        cycle = self.fourier_settings.sample_cycle(elapsed)
        if cycle < 0:
            if elapsed < 0:
                raise CalibrationError("Fourier reference clock moved backwards")
            self.command_speed = self.integral = 0.0
            self.transition("home", now)
            return Drive(position=self.origin, kp=self.fourier_settings.kp, kd=self.fourier_settings.kd)
        trajectory = self.trajectories[int(self.fourier_settings.role(cycle) >= 2)]
        position, velocity, _ = trajectory.evaluate(elapsed - self.fourier_settings.cycle_start(cycle))
        scale = self.fourier_settings.cycle_scale(cycle)
        self.reference_position = self.origin + position * scale
        reference_velocity = velocity * scale
        self.fourier_reference_velocity = reference_velocity
        correction = self.fourier_settings.kp / self.fourier_settings.kd * (self.reference_position - self.latest.position)
        limit = self.fourier_settings.command_speed_limit(self.settings)
        self.command_speed = max(-limit, min(limit, reference_velocity + correction))
        self.fourier_speed_limited = self.command_speed != reference_velocity + correction
        return Drive(position=self.reference_position, velocity=self.command_speed, kd=self.fourier_settings.kd)

    def tags(self) -> dict[str, float]:
        tags = super().tags()
        tags.update(fourier_tracking=float(self.phase == "periodic"))
        tags.update(fourier_reference_velocity=self.fourier_reference_velocity,
                    fourier_speed_limited=float(self.fourier_speed_limited))
        if self.fourier_armed:
            tags.update(precision_cell=-1.0, precision_sampling=0.0, precision_speed=self.command_speed)
        return tags

    def analysis(self) -> tuple[FitResult | None, dict]:
        if self.phase != "done" or self.static_diagnostics is None:
            raise CalibrationError("Fourier excitation and independent static acquisition must finish before analysis")
        warmup = self.warmup_report()
        if self.warmup_cells and not warmup["complete"]:
            raise CalibrationError("Configured preconditioning did not complete")
        result, report = analyze_fourier(
            self.period_rows, self.fourier_settings, self.fourier_started, self.offset, self.offset_ci,
            self.mechanical_options, self.static_diagnostics, self.settings.max_gap, self.trajectories, origin=self.origin)
        if not self.baseline_stationary:
            report["quality_issues"].append("Disabled baseline moved; the sensor-zero assumption is uncertain")
            report["qualified"] = False
            result = None
        report.update(
            warmup=warmup, acquisition_stages=asdict(self.stages),
            preparation_settings=asdict(self.settings), baseline_segments=self.baseline_segments,
            baseline_stationary=self.baseline_stationary, temperature_range=[self.temperature_min, self.temperature_max],
            peak_speed=self.peak_speed, peak_measured_torque=self.peak_torque, peak_travel=self.peak_travel,
            elapsed_time=(self.latest.time - self.epoch) if self.latest is not None and self.epoch is not None else 0.0,
            formal_elapsed_time=(self.latest.time - self.formal_epoch)
            if self.latest is not None and self.formal_epoch is not None else 0.0,
            completed_cells=self.completed_cells, measured_cells=len(self.cycle_counts), required_cells=self.required_cells,
            collection_notes=list(self.collection_notes.values()))
        return result, report

    def status(self) -> dict:
        status = super().status()
        current = (self.fourier_settings.sample_cycle(self.latest.time - self.fourier_started)
                   if self.latest is not None and self.fourier_started is not None else -1)
        status.update(cells=self.required_cells, completed_cells=self.completed_cells, measured_cells=len(self.cycle_counts),
                      cell=max(0, current), validation=current >= 0 and self.fourier_settings.role(current) >= 2,
                      target_speed=self.command_speed if self.fourier_armed else status["target_speed"],
                      minimum_duration=self.settings.preparation_minimum_duration + self.fourier_settings.duration,
                      fourier={"cycle": current, "role": self.fourier_settings.role(current),
                               "scale": self.fourier_settings.cycle_scale(current) if current >= 0 else None,
                               "required_cycles": self.required_cells, "completed_cycles": self.completed_cells,
                               "samples": len(self.period_rows), "reference_started": self.fourier_started,
                               "numeric_buffer_bytes": len(self.period_rows.values) * self.period_rows.values.itemsize,
                               "settings": asdict(self.fourier_settings)})
        for name, roles in (("training", (0, 1)), ("validation", (2, 3))):
            indices = [index for index in range(self.required_cells) if self.fourier_settings.role(index) in roles]
            status[name + "_required"] = len(indices)
            status[name + "_completed"] = sum(
                index in self.cycle_counts and self.fourier_started is not None and self.latest is not None and
                self.latest.time - self.fourier_started >= self.fourier_settings.cycle_start(index + 1)
                for index in indices)
        return status
