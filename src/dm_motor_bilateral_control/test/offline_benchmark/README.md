This benchmark runs the project's `VirtualMotor`, physical model, codec, bilateral core, and `FrictionCompensator` without starting ROS or opening a transport. It compares shared position control, differential torque control, and a disabled single motor under known virtual hand loads. The explicit model is `config/bilateral_identification_20261104.yaml`; its compensation registry is resolved relative to that profile. No files or libraries under `~/.ros` are required.

Build the package with testing enabled. CMake's `test_offline_benchmark` runs ten short regression cases: fast drag for all three laws, differential rest/touch, low-delay drag/touch, and half-step integration checks. Run the same suite manually from the repository root:

```bash
python3 src/dm_motor_bilateral_control/test/offline_benchmark/run_benchmark.py \
  --binary build/dm_motor_bilateral_control/offline_bilateral_benchmark \
  --output /tmp/bilateral-benchmark
```

Add `--full` for the fixed 88-case matrix: signed A/B touches, ±.03/.2/1.3 rad/s drag, reversal, opposite mapping/direction, opposing loads, transport delays, stagger, K50/80/100, and ±30% J/Fc changes. It is a finite comparison, not every combination of parameter uncertainty. `--profile PATH` selects another explicit model. Each run writes input/model/binary hashes, cases, command records, raw traces, timing events, and metrics.

`--case differential_drag` selects a single short-suite case. For example, combine it with `--profile config/bilateral_optimized_simulation_20261104.yaml` to check the recommended profile. The compensation key follows the selected simulation variant's merged launch namespace; it is not tied to the identification profile's namespace. Early executable failures preserve return codes and stderr in `RESULTS.json`, including when no trace was produced.

The controller and motor driver run at 500Hz; broker polling is 1kHz. Distinct queues preserve controller publication, driver transmission, command delivery/device sample, broker receipt, driverRX service, and controller receipt. Commands and samples hold. Broker timestamps remain truthful; physical sampling timestamps are separate audit fields. The controller takes only decoded compact feedback, startup sensor anchors, and the original 8ms velocity filter. Only the environment and metrics access physical state.

Nominal transport is 1ms command plus 1ms return, with driver B staggered 1ms. Gating and command holding produce a measured mean physical reference age near 4ms. The model uses internal velocity lag 0, actuator lag 2ms, and separately filtered/delayed velocity and torque reports. It is an identified structural candidate, not an independently measured firmware model.

The virtual hand applies `15*(handle_position-q) + .05*(handle_velocity-v)` Nm. A .5s velocity ramp, 3s cruise, .5s ramp-down and .5s hold precede 3s release. The hand uses truth solely to compute a known external load. This differs from the ROS spring API's `K*(anchor-q)-D*v` and its wall-clock input refresh; results cannot be compared as identical inputs. Touches apply ±.3Nm for .2s. Opposing ±.4Nm loads measure apparent static stiffness including friction.

`.csv` samples physical and observed state at 2kHz. `.timing.csv` records the causal chain in 31.25us integer ticks. `.integrals.json` averages every environment tick to avoid aliasing native P's held-command sawtooth. Its drag window is 2–4.2s. `hand_common` is summed known applied hand torque; `motor_common` is electromagnetic torque. Friction, coupling, and encoded residual are separate columns. Positive hand drag can coexist with motor assistance because shaft friction remains.

Half-step checks use 15.625us physical integration with identical event times and held external inputs. Regression limits test useful physical conclusions: differential drag materially below the shared baseline, comparable to a disabled motor, real motion during touch, settled release, and close relative-motion/force metrics. Free common position is weakly constrained and quantization can shift its final resting point; exact sample-by-sample equality is not asserted. These tests do not authorize hardware operation.

An optional `angle` scenario applies the production `AngleOffset` quintic and `apply_angle_offset` to the same sensor-only control path. It supplies **no external hand torque**. The unchanged ten/88-case lists do not select this scenario; two additional pytest cases check +5 and -5 degrees. A case may specify `friction_fraction` in [0,1] (default .6); this changes compensation only, not the physical motor friction or the .2Nm compensation limit.

Create an independent case and model snapshot, then invoke the executable directly:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 - <<'PY'
import json
from pathlib import Path
import sys
sys.path.insert(0, 'src/dm_motor_bilateral_control/test/offline_benchmark')
from run_benchmark import DEFAULT_PROFILE, load_profile
out = Path('/tmp/bilateral-angle')
out.mkdir(exist_ok=False)
models, registry, namespace = load_profile(DEFAULT_PROFILE)
(out / 'MODELS.json').write_text(json.dumps(models, indent=2))
case = dict(law='differential', scenario='angle', angle_degrees=5,
            angle_baseline=5, angle_transition=2, angle_hold=5,
            angle_disabled_observation=1, friction_fraction=.6,
            calibration_file=str(registry), calibration_namespace=namespace)
(out / 'CASE.json').write_text(json.dumps(case, indent=2))
PY
build/dm_motor_bilateral_control/offline_bilateral_benchmark \
  /tmp/bilateral-angle/MODELS.json /tmp/bilateral-angle/CASE.json \
  /tmp/bilateral-angle/angle.csv
```

`angle_degrees` accepts nonzero values up to ±30 degrees, including ±5/±30. Other defaults are shown above: five seconds quiet, two seconds ramp, five seconds holding/observing the full offset, followed by queued simulated STOP and one second disabled observation. These phase durations are seconds; ramp and hold obey production `AngleOffset` bounds and the 2ms controller grid. `law='shared'` is also supported; `disabled` is rejected. Existing case fields (K, mapping, direction and transport delays) remain available. An explicit `duration` must include the complete sequence; omitting it computes the final time from the two driver phases and STOP transmission delay (13.002s with default queues). A positive offset asks for `(qA-originA)-direction*(qB-originB) ≈ -offset`. It does not prescribe equal motion by both axes.

Only angle traces append `angle_applied` (radians, held between controller ticks), `virtual_error` (the last observed relative position minus captured origins plus applied offset), `origin_a/b`, `angle_active`, `simulated_stop_requested` and `simulated_disabled_a/b`. Existing `relative_error`, `qa/qb`, `va/vb` remain physical metrics, never controller inputs. The `.integrals.json` window is the last ACTIVE second, before STOP. A `.stop.json` sidecar records integer request, driver transmission and device receipt ticks, decoded FD reply/final status, last ordinary command times, offset cancellation and disabled observation duration. Its `tick_seconds` converts ticks to seconds. STOP withdraws queued targets and stops ordinary transmission; each FD traverses the existing c2d/driver/wire path. Both simulated devices must report status0 before the offset is canceled. After STOP, observed columns hold the last ordinary feedback and must not be interpreted as fresh measurements; `virtual_error` after cancellation is no longer an active tracking objective.

The fixed ±5-degree tests require relative closure ≥95%, final physical virtual error <.004rad, genuine motion, last ACTIVE second speed <.02rad/s, full-run speed <1rad/s, zero external load, no protocol clipping and a completed simulated STOP/disabled sequence. They do not require a half-angle move by each axis. They preserve command records and physical metrics before asserting gates. These are finite application regressions, not stiffness/drag qualification, oscillation spectral acceptance or evidence of hardware STOP. A failed executable can leave a partial trace without a successful STOP sidecar; preserve the nonzero exit and partial files.
