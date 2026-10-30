"""Explicit, read-only replay of independent recordings into a new calibration result."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, replace
from pathlib import Path
import threading
from typing import Callable

from .data import CalibrationError, Trace, atomic_json, read_json
from .experiment import AcquisitionStages
from .fourier_identification import add_reference, analyze_fourier, combine_folds, fold_role
from .fourier_reidentify import read_fourier_samples, read_trajectories
from .fourier_trajectory import FourierSettings
from .identification import MechanicalOptions
from .precision import PrecisionSettings, analyze_precision, summarize_static
from .recording import environment_versions, save_sources
from .reidentify import file_sha256, replay_recording
from .stage_parameters import TEST_PROFILE_NOTE


def compatible_sources(sources: list[Path]) -> list[dict]:
    if not 1 <= len(sources) <= 16 or len(set(sources)) != len(sources):
        raise CalibrationError("Select 1..16 distinct source recordings")
    manifests = []
    sample_hashes: set[str] = set()
    signature = None
    for source in sources:
        metadata = read_json(source / "recording.json")
        if (not isinstance(metadata, dict) or metadata.get("kind") not in ("precision_recording", "fourier_recording")
                or metadata.get("recording_complete") is not True):
            raise CalibrationError("Only closed, versioned raw recordings can be reanalyzed")
        if metadata.get("resume") is not None:
            raise CalibrationError("Resume bundles contain inherited samples; select independent original recordings to avoid double counting")
        settings = PrecisionSettings.from_mapping(metadata["precision_settings"], steady_budget=False)
        preparation = asdict(settings)
        preparation.pop("warmup_repetitions")
        key = (metadata["kind"], metadata["identity"], preparation, metadata["origin"],
               metadata["position_resolution"], metadata["torque_resolution"])
        if metadata["kind"] == "fourier_recording":
            fourier = FourierSettings.from_mapping(metadata["fourier_settings"])
            key += (asdict(replace(fourier, training_enabled=True, validation_enabled=True)),
                    metadata["mechanical_options"], metadata["fourier_trajectories"])
        if signature is not None and signature != key:
            raise CalibrationError("Recordings must have identical motor identity, coordinates, acquisition and analysis settings")
        signature = key
        fingerprint = metadata["files"]["samples.csv"]["sha256"]
        if fingerprint in sample_hashes:
            raise CalibrationError("Duplicate raw samples cannot count as independent measurements")
        sample_hashes.add(fingerprint)
        manifests.append(metadata)
    return manifests


def merge_recordings(sources: list[Path], output: Path) -> dict:
    sources = [path.expanduser().resolve() for path in sources]
    output = output.expanduser().resolve()
    if output.exists() or any(output == source or output.is_relative_to(source) for source in sources):
        raise CalibrationError("Merged output must be a new directory outside every source recording")
    manifests = compatible_sources(sources)
    hashes = [file_sha256(source / "recording.json") for source in sources]
    fourier = manifests[0]["kind"] == "fourier_recording"
    bundles = [read_fourier_samples(source) for source in sources] if fourier else []
    replays = [bundle[0] for bundle in bundles] if fourier else [replay_recording(source) for source in sources]
    plans = [AcquisitionStages(**metadata.get("acquisition_stages", {})) for metadata in manifests]
    settings = PrecisionSettings.from_mapping(manifests[0]["precision_settings"], steady_budget=False)
    ci = max(replay.precision["baseline_ci95"] for replay in replays)
    static = summarize_static(
        [{**row, "source_index": index} for index, replay in enumerate(replays) if plans[index].collect_static
         for row in replay.precision["static"]["trials"]],
        [{**row, "source_index": index} for index, replay in enumerate(replays) if plans[index].collect_static
         for row in replay.precision["static"].get("conditioning", [])],
        settings, ci, repetitions=settings.static_repetitions * sum(plan.collect_static for plan in plans))
    temperatures = (min(replay.precision["temperature_range"][0] for replay in replays),
                    max(replay.precision["temperature_range"][1] for replay in replays))
    stationary = all(replay.precision["baseline_stationary"] for replay in replays)
    offset = replays[-1].precision["baseline_offset"]
    blockers = []
    if any(metadata["configuration"].get("protocol_mode") == "workflow_test" for metadata in manifests):
        blockers.append(TEST_PROFILE_NOTE)
    for index, replay in enumerate(replays):
        if replay.invalid_samples:
            blockers.append(f"Source {index + 1} contains invalid analysis samples")
        if not manifests[index]["acquisition"].get("stop_confirmed"):
            blockers.append(f"Source {index + 1} has no confirmed disable")
        if manifests[index].get("status") not in ("completed", "completed_unqualified"):
            blockers.append(f"Source {index + 1} did not complete normally")
    if fourier:
        options = MechanicalOptions(**manifests[0]["mechanical_options"])
        base = FourierSettings.from_mapping(manifests[0]["fourier_settings"])
        combined = replace(base, training_enabled=any(plan.collect_training for plan in plans),
                           validation_enabled=any(plan.collect_validation for plan in plans))
        trajectories = read_trajectories(manifests[0], base)
        folded = {1: [], 3: []}
        for index, (replay, rows, local, _options, pair, started) in enumerate(bundles):
            for role, enabled in ((1, local.training_enabled), (3, local.validation_enabled)):
                if not enabled:
                    continue
                if started is None:
                    raise CalibrationError("Selected Fourier source has no excitation clock")
                part = fold_role(rows, local, started, role, replay.precision["baseline_offset"],
                                 settings.max_gap, options.min_speed)
                part = (Trace.of(**{**part[0].columns,
                                   "provenance_source": [index] * len(part[0].columns["time"])}),
                        part[1], part[2])
                add_reference(part[2], pair[int(role == 3)], local, manifests[index]["origin"])
                folded[role].append(part)
        separation = max(settings.max_gap, base.period / base.phase_samples * 1.1)
        roles = ((combine_folds(folded[1], separation), combine_folds(folded[3], separation))
                 if folded[1] and folded[3] else None)
        fit, result = analyze_fourier([], combined, 0.0, offset, ci, options, static, settings.max_gap,
                                      trajectories, origin=manifests[0]["origin"], folded_roles=roles)
        required = sum(bundle[2].total_cycles for bundle in bundles)
        measured = sum(len({row.cycle for row in bundle[1]}) for bundle in bundles)
        result.update(required_cycles=required, measured_cycles=measured,
                      required_cells=required, measured_cells=measured, completed_cells=measured,
                      preparation_settings=asdict(settings))
        if not stationary:
            blockers.append("One or more disabled baselines moved")
        if temperatures[1] - temperatures[0] > settings.temperature_span:
            blockers.append("Combined temperature span exceeds the configured calibration band")
    else:
        observations = [{**row, "source_index": index} for index, replay in enumerate(replays)
                        for row in replay.precision["observations"]]
        fit, result = analyze_precision(
            observations, static, offset, ci, settings, source_plans=plans,
            cell_failures=[{**row, "source_index": index} for index, replay in enumerate(replays)
                           for row in replay.precision["cell_failures"]],
            temperature_range=temperatures, baseline_stationary=stationary)
        result.update(required_cells=sum(replay.precision["required_cells"] for replay in replays),
                      measured_cells=len(observations),
                      completed_cells=sum(replay.precision["completed_cells"] for replay in replays))
    for source, digest in zip(sources, hashes):
        if file_sha256(source / "recording.json") != digest:
            raise CalibrationError("Source manifest changed during reanalysis")
    if blockers:
        fit = None
        result["qualified"] = False
        result["quality_issues"].extend(blockers)
        if result.get("candidate_model") is not None:
            result["candidate_model"]["qualified"] = False
    experiment = "fourier" if fourier else "precision"
    result.update(temperature_range=temperatures, baseline_stationary=stationary,
                  baseline_segments=[{**row, "source_index": index} for index, replay in enumerate(replays)
                                     for row in replay.precision["baseline_segments"]],
                  acquisition_complete=all(
                      replay.precision.get("acquisition_complete", True) for replay in replays))
    report = {
        "status": "completed" if fit is not None else "completed_unqualified",
        "experiment": experiment, "operation": "merge_reidentify", "offline": True, "no_motion_performed": True,
        "identity": manifests[0]["identity"], "configuration": manifests[-1]["configuration"],
        "qualified": fit is not None, "samples": sum(replay.samples for replay in replays),
        "stop_confirmed": all(metadata["acquisition"].get("stop_confirmed", False) for metadata in manifests),
        "publication_blockers": blockers, experiment: result,
        "sources": [{"run": source.name, "path": str(source), "manifest_sha256": digest,
                     "samples_sha256": metadata["files"]["samples.csv"]["sha256"], "samples": replay.samples,
                     "stages": asdict(plan), "baseline_offset": replay.precision["baseline_offset"]}
                    for source, digest, metadata, replay, plan in zip(sources, hashes, manifests, replays, plans)],
        "merge_policy": "Replay every source with its own sensor zero; preserve per-source holdouts and independent validation. No cached fitted parameters are averaged.",
    }
    output.mkdir(parents=True, exist_ok=False)
    report["analysis_environment"] = environment_versions()
    report["analysis_sources"] = save_sources(output)
    atomic_json(output / "inputs.json", {"sources": report["sources"], "manifests": manifests})
    atomic_json(output / "static.json", static)
    atomic_json(output / (experiment + ".json"), result)
    if fit is not None:
        fit.save(output / "calibration.json", report["identity"])
        report["fit"] = asdict(fit)
    atomic_json(output / "report.json", report)
    return {"status": "ready", "run": output.name, "qualified": fit is not None, "samples": report["samples"]}


class ReanalysisJobs:
    def __init__(self, log_error: Callable[[str], object]):
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="calibration-reanalysis")
        self.log_error = log_error
        self.current: dict = {"status": "idle"}

    def request(self, sources: list[Path], output: Path) -> dict:
        with self.lock:
            if self.current["status"] == "running":
                raise CalibrationError("A reanalysis is already running")
            self.current = {"status": "running", "run": output.name, "sources": [path.name for path in sources]}
            future = self.executor.submit(merge_recordings, sources, output)
            future.add_done_callback(lambda finished: self.finished(finished, output))
            return dict(self.current)

    def finished(self, future, output: Path) -> None:
        error = "Reanalysis cancelled" if future.cancelled() else future.exception()
        if error is not None:
            message = f"Reanalysis failed: {error}"
            self.log_error(message)
            result = {"status": "failed", "run": output.name, "error": message}
            try:
                output.mkdir(parents=True, exist_ok=True)
                atomic_json(output / "report.json", {
                    "status": "failed", "operation": "merge_reidentify", "offline": True,
                    "no_motion_performed": True, "qualified": False, "error": message})
            except OSError as persist_error:
                result["error"] += f"; cannot persist failure: {persist_error}"
                self.log_error(result["error"])
        else:
            result = future.result()
        with self.lock:
            self.current = result

    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.current)

    def close(self) -> None:
        self.executor.shutdown(wait=True, cancel_futures=False)
