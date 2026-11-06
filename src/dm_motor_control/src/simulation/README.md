# Virtual motor dynamics

`VirtualMotor` runs the joint dynamics and its internal servo at every
`integration_step` (default 0.5 ms). CAN command frequency and published state
frequency do not set this integration rate.

`velocity_time_constant` optionally models a first-order internal velocity
estimate. It accepts finite values from 0 to 0.1 seconds. The default 0 uses the
physical velocity exactly, preserving the existing ideal-sensor model. A positive
value updates the estimate after each physics substep. Internal velocity feedback,
including MIT damping, uses that estimate. The encoded CAN velocity uses it by
default; the reporting source and dynamics can be selected independently below.
`simulation/state.velocity` continues to report the actual physical velocity.
The estimate starts at `initial_velocity` and remains continuous through disable,
device zero, and virtual power-cycle operations.

This setting and `torque_time_constant` describe different dynamics: the former
delays the velocity estimate, while the latter delays actuator torque. Nonzero
velocity-estimator settings are robustness-test assumptions unless measured on
the corresponding hardware. A simulation failure at a chosen value demonstrates
sensitivity to that assumption; it does not identify a real motor's time constant.

## Independent feedback reporting

These optional parameters affect the encoded feedback only. They never alter the
physical position, velocity, motor torque, temperature, or the internal MIT servo.
`simulation/state` remains ground truth. Missing keys in an older configuration
use the defaults below.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `feedback_velocity_source` | `internal` | `internal` uses the existing internal estimate; `physical` uses physical velocity. |
| `feedback_velocity_time_constant` | 0 s | First-order filtering of the selected reporting source. |
| `feedback_velocity_delay` | 0 s | Delay after the reporting velocity filter. |
| `feedback_torque_time_constant` | 0 s | First-order filtering of the actual physical motor torque for reporting. |
| `feedback_torque_delay` | 0 s | Delay after the reporting torque filter. |

All four time constants/delays must be finite and in `[0, 0.1]` seconds. Reporting
filters update at each integration substep. Delay history interpolates fractional
substeps and holds the initial value before enough simulated time has elapsed.
The history is limited to the configured delay and 20,002 samples; excessive
substep density faults the simulation instead of growing memory without bound.
Repeated reads at the same simulated time do not advance the filters or delays.
Disable, device zero and virtual power cycle preserve reporting state and history.

The legacy defaults produce the same feedback as before: internal velocity with
no additional reporting dynamics, and physical motor torque. Selecting `physical`
allows a measured reporting response to be modeled independently of an uncertain
internal estimate; selecting `internal` adds the reporting filter after that
estimate, so the two time constants must not accidentally count the same filter
twice.

For example, the 2026-10-30 recorded motor experiments support this *reporting*
scenario (explicit overrides, not new defaults):

```yaml
feedback_velocity_source: physical
feedback_velocity_time_constant: 0.0045
feedback_velocity_delay: 0.0015
feedback_torque_time_constant: 0.012
feedback_torque_delay: 0.002  # 0.003 is another observed composite timing candidate.
```

The velocity response cross-validates against an independent long MOVE. The
torque response describes CAN-reported torque and includes command snapshot and
transport timing uncertainty. It does **not** establish a 12 ms actuator response;
do not copy it into `torque_time_constant`. Likewise, the records do not uniquely
establish whether the MIT damping loop uses the reported velocity estimate.
Keep `velocity_time_constant`, physical `torque_time_constant`, inertia and friction
as separate uncertainty cases. If command transport delay is modeled elsewhere,
do not duplicate the composite 2–3 ms timing entirely in the reporting path.

## Optional elastic LuGre friction

`friction_model: lugre` selects a symmetric, one-coordinate LuGre branch with
bristle damping `sigma1=0`. The existing viscous term remains separate. The
default remains `coulomb_viscous`; existing viscous, Coulomb and Stribeck branches
do not use the bristle state. No fitted motor parameters or configuration defaults
are replaced by selecting this additional model capability.

| Parameter | Default | LuGre meaning |
| --- | --- | --- |
| `friction_displacement_scale` | 0.001 rad | Positive displacement scale `delta`; stiffness `k=Fs/delta`. |
| `initial_bristle_displacement` | 0 rad | Initial elastic state `z0`, with `abs(z0)<=delta`. |
| `coulomb_friction` / `static_friction` | Existing defaults | `0 < Fc <= Fs`. |
| `stribeck_velocity` | Existing default | Positive velocity scale `vs`. |

The equations are

```text
g(v) = Fc + (Fs-Fc) exp(-(v/vs)^2)
J dv/dt = U - k z - b v
dz/dt = v - abs(v) k z / g(v)
```

Here `U` is the frozen motor plus external torque minus constant `load_torque`
for the physics step, and `b` is `viscous_friction`. When `Fs=Fc`, the Stribeck
velocity has no effect and the branch reduces to full-bristle Dahl friction.
There is no parallel set-valued dry-friction threshold: a subthreshold force
can move the rotor and load the elastic state. A nonzero `z0` is a preload, not
a promise of static equilibrium; it can relax even with zero input.

The supported range is symmetric (`friction_negative_scale=1`), without gravity
or a joint spring. Nonzero `gravity_torque` or `joint_stiffness` is rejected.
A constant `load_torque` is supported with its existing `load_torque*q`
potential energy. The initial position must lie strictly inside the limits;
a substep that reaches either limit is rejected instead of silently switching
to the unverified contact model. External torque is an input held over the
outer physics step; this does not prove a discretization of an external spring
or another coupled system preserves its own total energy.

For each accepted midpoint substep of duration `h`, `vm=(v0+v1)/2`,
`zm=(z0+h*vm/2)/(1+h*abs(vm)*k/(2*g(vm)))`, `q1=q0+h*vm` and
`z1=2*zm-z0`. The velocity equation is solved by bracketed bisection. The solver
halves the substep until all of these conditions hold:

```text
C = 1 + Fs/Fc + (2/e) Fs(Fs-Fc)/Fc^2
h^2 k C/(4J) <= 0.5       # strict margin inside the uniqueness bound <1
a = h abs(vm) k/(2g(vm)) <= 1
h sqrt(k/J) <= 0.25
```

The endpoint update is a convex combination and preserves `abs(z)<=delta`.
Neither the state nor dissipated energy is clamped. A physics step accepts at
most 4096 substeps and cannot refine below `dt/4096`; insufficient resolution,
invalid input, a failed energy check or contact produces an exception. State,
thermal values and cumulative work are committed only after the whole step
succeeds. The existing outer `dt` requirement `(0,0.001]` seconds remains.

For kinetic plus bristle storage `H=J*v^2/2+k*z^2/2`, the midpoint identity is

```text
H1-H0 = h U vm - h b vm^2 - h abs(vm) k^2 zm^2/g(vm)
```

Both loss terms are nonnegative. Constant load work is transferred into its
existing potential energy, so the complete budget uses motor plus external
work. `PhysicsState.bristle_drive_work`, `viscous_dissipation` and
`bristle_dissipation` are cumulative diagnostics for this branch only.
`mechanical_energy` keeps its previous kinetic/potential meaning;
`total_storage=mechanical_energy+bristle_energy`. The state also exposes
`bristle_displacement` and `bristle_torque=k*z`; the latter is the restoring
term subtracted in the equation, whereas `friction_torque` remains the net
step-averaged friction torque. The three fields `bristle_displacement`,
`bristle_energy` and `total_storage` are published on `simulation/state`,
not encoded into motor feedback.

STOP and ENABLE do not reset position, velocity or bristle state. During
disabled time, zero motor torque still allows physical relaxation. A virtual
firmware power cycle also preserves that physical state; constructing a new
`JointPhysics`/`VirtualMotor` instance uses the configured initial state.
Bristle state is not a saved device register or a calibration value.

This is a numerically verified optional model, not an identified motor model.
The bounded recorded-data investigation matched some stopped displacement
endpoints but did not reproduce the timing of MOVE position bursts. Reported
velocity contains quantization and stationary estimator variation; sparse
position-register reads do not identify its high-frequency waveform. Those
limitations remain and no observation-noise model is added here. The LuGre
model family is described by DOI `10.1109/9.376053`; the midpoint algorithm and
the finite subdivision conditions above are separately checked numerical
choices, not claims about the firmware implementation.
