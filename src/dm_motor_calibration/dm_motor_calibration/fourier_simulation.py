"""Independent production-physics validation of the Fourier experiment."""

from pathlib import Path
from typing import Mapping

from .data import FitResult
from .fourier_experiment import FourierExperiment
from .fourier_trajectory import FourierSettings, design_pair
from .identification import MechanicalOptions
from .precision import PrecisionSettings
from .precision_simulation import collect_prepared


def collect_fourier(parameters: Mapping[str, float | str], *, settings: FourierSettings | None = None,
                    preparation: PrecisionSettings | None = None, options: MechanicalOptions | None = None,
                    noise: float = 0.0005, quantized: bool = True, seed: int = 7,
                    output: Path | None = None, executable: Path | None = None) -> tuple[FitResult | None, dict]:
    fourier = settings or FourierSettings()
    preparation = preparation or PrecisionSettings()
    options = options or MechanicalOptions(model="auto", fit_gravity=True)
    fourier.validate_control_rate(200.0)
    trajectories = design_pair(fourier, options)
    return collect_prepared(
        parameters, settings=preparation, noise=noise, quantized=quantized, seed=seed, output=output,
        executable=executable, experiment="fourier", feedback_period=1 / fourier.feedback_rate,
        feedback_after_preparation=True,
        engine_factory=lambda p, origin: FourierExperiment(p, origin, fourier, options, trajectories=trajectories))
