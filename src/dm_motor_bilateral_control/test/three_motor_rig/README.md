This is a finite **offline** three-motor fixture. It starts no ROS process, opens no
transport, and invokes no hardware operation. It compiles the production
`VirtualMotor`, `JointPhysics`, MIT codec, register/target helpers, `bilateral.cpp`,
and `FrictionCompensator`. The existing benchmark's `controller.hpp` remains the
controller boundary. Generated ROS message headers are required by the shared
compensator header at compile time; the executable links no ROS runtime library.

Run from the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 src/dm_motor_bilateral_control/test/three_motor_rig/run_fixture.py \
  --speed .3 .6 13.2 --fraction 0 .6
```

The default output is
`~/.ros/dm_motor_bilateral_control/three-motor-optimization-20261105/simulation-review`.
`--output` must remain outside the repository. Build products, hashes, input
snapshots, event records, traces, and results go there. `--skip-build` reuses
`OUTPUT/three_motor_fixture`; `--binary PATH` uses an existing CMake-built fixture
without compiler/header discovery. Both reuse modes record the binary hash and
current source snapshot, but do not claim that snapshot proves how the existing
binary was built. `--profile` defaults to the source-controlled
`config/bilateral_optimized_simulation_20261104.yaml`; the existing
`offline_benchmark.run_benchmark.load_profile` merges its motor models and resolves
its registry/namespace. No installed profile is substituted.

With `BUILD_TESTING=ON`, the package builds `three_motor_fixture` and registers
`test_three_motor_coupling` and `test_three_motor_rig`. After building the package
and its current dependencies, run only these offline tests:

```bash
ctest --test-dir build/dm_motor_bilateral_control \
  --output-on-failure -R '^test_three_motor_(coupling|rig)$'
```

The coupling test checks signs, passive power balance, backlash and step-size
convergence, including Release builds. The pytest entry runs two short .3 rad/s
cases, including an explicit distinct C model, nondefault K/D/torque cap, and
synchronized application-delay case. It checks motion/following, friction,
settling, reporting separation, causal timing, production unwrapping, and
rejection of invalid reference files. It does not run the six-case matrix. Both tests
produce ament/colcon-readable XML results. A standalone invocation is:

```bash
DM_THREE_MOTOR_FIXTURE=/absolute/path/to/three_motor_fixture \
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m pytest -q -p no:cacheprovider \
  src/dm_motor_bilateral_control/test/three_motor_rig/test_three_motor_fixture.py
```

Direct builds use `--cxx` (default `$CXX` or `c++`), explicit repeated
`--include-dir` roots, and activated `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH`
prefixes. Generated headers may come from the workspace build or an isolated or
merged install. When no prefix is active, a named `ROS_DISTRO`, or one unambiguous
`/opt/ros` installation, supplies the compile headers. Multiple unselected
installations are not mixed. All paths are resolved independently of the current
working directory; quoted paths with spaces are supported. Direct builds target
Linux GCC/Clang with ELF section garbage collection; other build environments
should supply a CMake-built executable through `--binary`.

There are three independent `VirtualMotor` objects. A/id1 and B/id2 use their own
profile parameters. `--models PATH` supplies a complete `motor_1`, `motor_2`, and
optional `motor_3` JSON snapshot. **Without `motor_3`, C/id3 copies A's model as an
assumption**. Explicit C parameters can likewise be an allocation assumption;
their presence does not establish independent identification. All states advance in 31.25 us
steps. At each step the fixture snapshots A and C, computes both coupling torques,
sets all external torques, and only then advances all three motors. It never
recomputes the force after advancing just one axis.

The link is a passive effective compliant path with optional backlash:
`delta = q1 + q3 - offset`, `tau1 = tau3 = -K*extension - D*(v1+v3)` outside
the gap. The torque signs are equal because the two native encoders have opposite
directions. Defaults are K40 Nm/rad, D.02 Nm s/rad, zero gap. These are scenario
parameters, **not identified shaft rigidity**. The preexisting `test_coupling.cpp`
checks the passive link law. Change the scenario with `--shaft-stiffness`,
`--shaft-damping`, and `--shaft-half-gap`.

C alone receives the prescribed MIT P20/D.6 trajectory (including its reference
velocity). A receives only the compliant-link external torque; B has zero
external torque and follows only through the actual differential bilateral law
(default K50, coupling damping .4975, local damping .005). No external position target is
assigned to A or B. A/B compensation uses the real `FrictionCompensator` constructed
from each source registry simulation fit, a default .2 Nm cap, and fraction 0 or .6.
Its fit is used as a model candidate with qualification checks disabled, matching
the existing offline benchmark; that is not hardware calibration approval.

The trajectory matches the finite `physical_drag_v2.py` geometry: .3 s baseline,
raised-cosine velocity ramps of `r=max(.25,speed/15)` seconds, .6 s cruise,
.25 s turnaround, reverse sweep, .5 s settling. Duration is `2*(2*r+.6)+1.05`.
C starts at minus half the travel and A/B at plus half the travel, so the
13.2 rad/s command remains within the native MIT position range. Initial
velocities are zero. The initial-coordinate changes are scenario setup, not
changes to identified plant coefficients.

The controller and per-axis command transmissions are 500 Hz with held targets.
Commands take 1 ms to reach a motor; its raw MIT reply is encoded at device
receipt and takes 1 ms to reach the controller. B's transmission/sampling phase
is offset by 1 ms. Direct CASE files can vary `command_delay`, `feedback_delay`
and `b_phase` on the 31.25 us grid; delays must be positive and B phase <2 ms.
The plant's own velocity and torque reporting dynamics are
preserved. Only decoded feedback populates `latest`; only that feedback and
startup feedback origins enter `bench_control::compute`. Physics truth is used
solely for the physical link and logging. Production peer-velocity filtering and
bin centering remain intact. This is a fixed-delay fixture, not a reconstruction
of ROS scheduling jitter or a complete operator-session state machine.

Traces default to 2 kHz; CASE `trace_rate` can select 500, 1000 or 2000 Hz without
changing physics or command cadence. `q/v/t_truthN` are physical position, speed, and
electromagnetic torque; `q/v/t_reportedN` hold the most recently delivered decoded
feedback. `sample_timeN` is the corresponding physical sample time.
`command_*N` is the decoded MIT command currently applied at the device.
`friction_requestedN` is the latest compensator output;
`friction_appliedN` and `bilateral_torqueN` are controller torque components
carried with the command through its delay. They do not include MIT rounding
residuals or actuator dynamics; `command_tN` and `t_truthN` expose those later
stages. `plant_frictionN` is the physical signed friction torque.
The trace also records reference, link extension/energy/dissipation/torque,
filtered speeds, A/B relative error, and clipping. The event file records raw
command/reply hex and all source/controller/send/sample/delivery integer ticks.
Multiply ticks by 1/32000 seconds.

`RESULTS.json` reports both steady cruise windows, peak physical speed/torque,
final state, and basic physical invariants. These runs are model predictions.
**No high-speed fit, high-speed validation, or hardware safety claim follows from
running 13.2 rad/s**; the inherited A/B model and unmeasured C/link assumptions
must be checked against independent data before those claims are justified.

For offline parameter sweeps, CASE fields `stiffness`, `coupling_damping`,
`local_damping`, and `friction_torque_limit` are explicit, with corresponding
runner CLI options. The friction limit uses production Settings' `(0,.5]` Nm
range. `--calibration-file` and `--calibration-namespace` select a separate
simulation compensation registry without changing physical models or writing a
production registry.

`config/three_motor_identified_simulation_20261105.json` is an explicit three-axis
physical candidate. It changes only Fc/b from the original profile. B uses the
frozen independent effective estimate; A/C split the identified sum equally as
an assumption. Inertia, static friction, friction law and reporting dynamics
remain priors. The file records training, holdout evidence and limitations and
does not replace any runtime default.

`compare_identified.py --fit FIT.json --binary FIXTURE --output OUT` constructs
the frozen Fc/b candidate and compares it with the original plant using the same
unchanged compensation registry. It verifies `FROZEN_STATIC.json`, hashes every
real input, preserves whole-trial train/validation membership, and aligns only by
`receive_time - result.started`. A 1 ms causal hold masks reports older than
20 ms. Position origins come from the fixed .02–.2 s baseline; position-derived
speed uses a fixed 51 ms cubic derivative. Scores include both predeclared cruise
windows, the full .3 s-to-end transient, settling, report torque/velocity,
position-derived velocity and detrended ripple. No fitted time shift is used.

The optional direct CASE `scenario` values are:

- `trajectory`: the original C P20/D.6 trajectory and A/C link.
- `touch`: zero C P/D/feedforward, a finite external C torque pulse, then release
  while A/C remain coupled.
- `touch_pair`: C disconnected throughout; a finite external A pulse and then
  free A/B response. There is no environment position target.
- `release_pair`: C drives the original trajectory until `release_time`, then
  its link torque becomes identically zero. A/B subsequently evolve independently
  of C, so C friction cannot hide active damping in the pair. C's independent
  trajectory continues only as a logged disconnected rotor.

Probe fields are `probe_torque`, `probe_start`, and `probe_duration`. Traces expose
`probe_torque1`, `probe_torque3`, and `coupling_active`.
Removing C is a simulation operation, not a hardware procedure. These cases do
not qualify an A compensation model or authorize production changes.

Direct CASE fields `c_kp` and `c_kd` default to20/.6.
`c_velocity_bandwidth` writes and verifies
simulated register32 while disabled; **VirtualMotor stores that register without
changing dynamics**. A separate MODELS C `velocity_time_constant` override is
therefore an explicit model assumption, not identified firmware behavior.

`position_feedback_mode=common_time` is a position-only diagnostic against the
default `latest`: interpolate delivered decoded position histories at the older
latest receive timestamp, with no extrapolation or physical truth. Raw velocity,
velocity filtering, torque and friction remain on their latest feedback path.
`control_positionN` and `control_position_timeN` expose the exact positions used.
This diagnostic alone is not a production synchronization implementation.

`feedback_pause_axis` (0 disables,1/2 selects A/B), `feedback_pause_start`, and
`feedback_pause_duration` can delay one application subscription while preserving
every physical sample, encoded frame and original transport receive timestamp.
Other axes continue normally. Delayed frames drain in timestamp order when the
pause ends. Event `feedback_delivery_tick` remains transport arrival;
`application_delivery_tick` and trace `application_delivery_timeN` separately
record consumption by the controller. This finite schedule is a sensitivity
case, not a replay of real host scheduling.

`position_feedback_mode=synchronized` consumes the production
`FeedbackPairs::common_time_positions(now,.020,.005)` result in
`symmetric_targets`' optional spring-position input. It preserves the latest
velocity/damping path. Missing or stale brackets abort the offline run as
STOP-required; there is no latest-position fallback. CASE
`synchronization_max_age` and `synchronization_max_bracket_gap` expose those bounds.
The fixture's default bracket limit is 5 ms; the later 8 ms cases set `.008`
explicitly and retain the same 20 ms age limit.

CASE `extra_viscous_1`, `extra_viscous_2`, and `extra_viscous_velocity_limit`
default to0 and call the actual production compensator's explicit additional
viscous term. Its calibrated domain remains unchanged; cap, filter, reversal
reset and slew are shared.

Feedback positions now use the **production Bus absolute-position path**.
`feedback_runtime.hpp` provides an in-memory Transport only: startup Bus queries,
including register80 anchoring and enable, go to the same three VirtualMotors
while their physical clock remains0. During motion the Bus receives each original
MIT frame at its transport receive timestamp and applies its real rollover/gap
validation. No Bus-generated motion commands are allowed after startup. The
fixture's existing delayed wire-command path still drives all three plants.

The Bus snapshot enters a separate application-delivery queue. Pausing an
application subscriber therefore does not pause driver unwrapping or rewrite
receive timestamps. `q_reportedN` is continuous absolute position, matching the
production motor-node position field. `native_positionN` preserves the raw MIT
decoded angle, and `position_wrapsN` records the runtime rollover count. Physical
truth never enters this feedback conversion. A fixed clock epoch accounts for
the driver's completed initialization; relative receive and delivery times keep
their original meaning.

`FIXTURE --test-feedback-wrap` checks both directions through native boundaries
and verifies common-time positions remain continuous. This small sensor-path
regression is included in CTest, alongside the short plant cases.

For a continuous variable-speed protocol, a direct trajectory CASE can specify
`reference_file`, an absolute CSV path with exact header `time,q,v`. Rows begin
at zero, advance exactly 2 ms, contain finite values, keep position strictly
inside (-12.4,12.4) rad and speed within ±13.3 rad/s, and begin/end at zero speed.
Adjacent positions must respect that speed envelope. The explicit CASE duration
must match the last row. Only this file mode permits up to 360 s; ordinary cases
retain the 60 s limit. The table sets C's initial position and held C reference;
A/B receive no environmental position targets.

`check_variable_reference.py` imports the external sender preparation's pure
`Protocol`, generates its exact 500 Hz table, and checks one six-speed group or
all 17 groups. It uses the frozen three-axis physical candidate, original
compensation registry, B-only80% extra assistance, C P10/D.6 with V_BW40 stored,
and the production synchronized/Bus feedback paths with an explicit 8 ms bracket
limit. There are no internal holds between speed legs. The full sequence has
301.24 s of motion and 302.04 s total duration. It saves the source, reference,
model and executable hashes, a 500 Hz trace, and every raw command/reply event.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
python3 src/dm_motor_bilateral_control/test/three_motor_rig/check_variable_reference.py \
  --binary /absolute/path/to/three_motor_fixture \
  --protocol-file /absolute/path/to/long-variable-preparation/protocol.py \
  --groups 1 --output /outside/repository/short-sequence
```

Use `--groups 17` only after reviewing the short run. Existing output directories
are never reused for simulation. `--score-existing OUT` scores a completed run
without running its executable; it checks the archived input hashes and records
the scorer separately from the original simulation source snapshot. The frozen
gates cover speed, clipping, link/follower error, settling, repeated-group drift
and exact wire cadence. Model temperatures and report-based C torque estimates
are diagnostics, not identified firmware or thermal limits. These offline
checks do not establish that a physical five-minute run has completed.

Receive collection is separate from application delivery. CASE `rx_batch_axis`
(0 disables, 1/2 selects A/B), `rx_batch_at`, `rx_batch_window`, and `rx_batch_gap`
delay actual raw replies **before production Bus processing**. The pre-window
reply receives at `at-gap`; later replies through `at` receive together at `at`.
Their original encoded payloads and physical sample times remain unchanged.
Every frame advances the Bus count; the production helper's existing equal-time
rule keeps the newest sample at a shared receive timestamp. The event file adds
`nominal_feedback_tick`; `.rx.csv` records actual Bus consumption and count.
This explicitly models receive latency, not a reconstruction of a hardware run.

`controller_phase` defaults to0 and may select another physics-grid phase within
the2 ms control period. Wire transmissions keep their existing phases. A1 ms
control phase lets a B collection flush coincide with a control tick whose
latest A sample falls inside B's wide gap. Before the first valid synchronized
bracket, A/B retain neutral MIT targets for at most20 ms, matching the production
startup wait. After activation, any missing valid common-time evidence still
aborts as STOP-required. `.control.csv` records each active helper result and
the rejection that ends a case. No interruption during active control is treated
as a new startup period.

A receive timestamp can be substantially newer than its payload's sample.
The traces record both ages without correcting that uncertainty from physical truth.
