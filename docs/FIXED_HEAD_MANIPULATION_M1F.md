# M1f: fixed-head, position-and-axis coupon manipulation

Status: executable F1-reduced simulation; **not hardware qualification**.

M1f removes the M1e observer's automatic ROI repositioning and adds executable
axis corrections. One fixed camera/projector observes one arm acquiring a
0.40 mm peg, transferring it, correcting its direction, inserting it into the
existing 0.125 mm radial-clearance socket, releasing, and retreating. Roll of
the circular peg remains unobservable. M1e remains the default schema-v1
regression baseline; M1f is selected explicitly with a schema-v2 scenario.

## Fixed physical arrangement

`scenarios/observed_manipulation_m1f_v2.json` is authoritative. Distances are
metres, times seconds, angles radians, and vectors are in the cell frame C.

| Datum or parameter | M1f candidate |
| --- | --- |
| Nominal pickup centre | [20, -4, 0] mm |
| Nominal socket centre | [20, +4, 0] mm |
| Fixed optical target-plane centre | [21.8, 0, 1.3] mm |
| Working distance / camera-projector baseline | 30 / 24 mm |
| Target-plane field / image dimensions | 9 × 12 mm / 1920 × 2560 pixels |
| Camera and projector body envelope radius | 3 mm each |
| Mount envelope radius | 1 mm |
| Pattern sequence | 8 exposures at 60 Hz |
| Processing / settling / maximum measurement age | 20 / 12 / 160 ms |
| Correlated calibration floor | 3 µm, applied once |
| Maximum translation correction / repeatable floor | 400 / 5 µm |
| Maximum axis correction / repeatable angular floor | 20 / 1 mrad |
| Axis convergence / observed capture limit | 9 / 100 mrad |
| Correction axis speed / acceleration limits | 0.12 rad/s / 0.50 rad/s² |
| Transfer standoff | 3.5 mm |

The versioned unit view vector and image-axis vector determine the rigid head
orientation. Camera and projector entrance pupils are offset by half the
baseline; their optical axes converge on the fixed target-plane centre. Opaque
body spheres sit behind the apertures. The mount is fixed relative to their
midpoint. These same solids participate in mechanical preflight and optical
occlusion. They are packaging envelopes, not complete lens or bracket CAD.

The pickup and socket sit side by side in the image. A layout along the viewing
direction allowed the socket to obscure the retracted tool. The side-by-side
layout and a more distant transfer path were necessary to satisfy the fixed
visibility and rail-clearance guards. Requests for other ROIs change the
requested feature set; they cannot move, rotate, or zoom the observer. The
calibration-health point is fixed and uses the cell's occlusion scene too.

Pixel pitch at the target is 4.6875 µm in both image directions. This candidate
does **not** inherit M1d's 3.0/3.4 µm precision values. Its estimator propagates
the changed geometry and localization noise. A 60 Hz sequence takes 134
integer 1 ms ticks, with the oldest measurement available at age 153 ms after
the configured processing delay. The 160 ms gate admits that stopped burst;
it does not permit continuous blind near-contact tracking.

The approximately 4.9 MP / 60 Hz acquisition rate, projector synchronization,
full-field focus, and throughput remain unqualified component requirements.
No particular low-cost camera, lens, or projector is asserted to meet them.
The 1 mrad angular correction floor is a scenario hypothesis requiring a
loaded-arm coupon measurement, just as the 5 µm translation floor does.

## Executed control and motion

`MachineCommand::SetToolAxisTarget` constrains the TCP position and directed
local-Z axis. The existing position-only command retains its original policy.
The new solver uses five coordinates: carriage Z, carriage azimuth, shoulder
yaw, shoulder pitch, and elbow pitch. It preserves the wrist-roll coordinate;
that does not create a measured or controlled world-frame roll.

The deterministic damped least-squares solver scales rail translation by arm
reach, clamps every trial to joint/travel limits, backtracks failed trials,
and independently checks final FK position and axis residuals. It tries the
current command state and a position-IK seed, with at most 100 iterations each.
An unreachable constraint returns an error. It never submits its best partial
solution. The resulting joint plan uses the existing synchronized smoothstep,
limit-derived duration, tendon boundary, command sequence, Stop behaviour,
arm collision preflight, carried-body preflight, and sampling-work limit.

The observed controller applies a bounded minimum rotation from the measured
moving axis toward the measured target axis to the **commanded** tool axis.
It does not assign the simulated peg orientation or read the configured
latent initial errors. Pick and socket correction loops require position and
axis convergence. Translation corrections retain the commanded axis. Rotation
of an attached peg follows the plant-owned grasp transform.

The translation controller can use one full 5 µm step on the largest error axis
when gain-scaled components all fall below the actuator floor, provided that step
strictly reduces observed squared error. The convergence tolerance stays at 9 µm.

After every correction the executive settles and reacquires measurements.
Each correction phase has a 32-iteration budget. Retraction and carried transfer
are divided into at most 128 segments, each no longer than the configured
translation correction magnitude, with fresh tool and peg
observations between segments. The final approach reduces the remaining
translation error through bounded steps. Angular corrections occur outside
the insertion lead-in. Insertion still requires fresh relative estimates,
bounded increments, contact evidence, and the independent force interlock.

## Collision and uncertainty contract

The previous uncertainty-inflated head/fixture, jaw/socket, and carried-peg
checks remain active. M1f additionally gives both socket target rails physical
capsule envelopes and checks them before motion:

- With a fresh socket estimate, the rails use its position and axis bounds.
  Before socket reacquisition they use the surveyed nominal datum with an
  explicit 50 µm position bound and 50 mrad axis bound. Scenario admission
  rejects latent socket initialization outside those declared bounds.
- The controller encloses all possible unobserved rail roll in a finite
  annular keep-out. Separate axial envelopes cover jaws, tool targets,
  recessed terminal tube, and carried peg. Mean tilt, uncertainty, full
  commanded angular excursion, carried-offset rotation, and FK departure
  from the endpoint chord enlarge the swept envelopes.
- The terminal tube's front plane and axis rotate as one rigid volume. The
  support calculation preserves that relation. A detached centre bound
  would count the same orientation uncertainty twice and incorrectly reject
  the validated recess clearance.
- Interval displacement bounds supplement the samples; sampling alone is
  not presented as continuous clearance proof. The two more distant arm
  links use conservative spherical enclosures sampled from command/FK, with
  a 120 µm provisional model margin and a bound on intersample travel.
- Independent plant scoring checks jaw/target/peg/link/tube overlaps with
  the physical rails at every fixed step. Palm/peg overlap is also checked.
  An intended peg/socket contact exemption cannot exempt the rails.

The nominal run must complete with every applicable controller gate passing,
release confirmed, seat tolerances satisfied by terminal evaluation, and zero
unplanned penetration. Terminal faults issue Stop and hold. No unobserved
automatic retreat is added. Truth scoring remains unavailable during control
and excluded from the controller trace hash.

## Reproduction and robustness evidence

```sh
cargo test --locked --workspace
cargo build --locked --release -p pipe_sim_cli --bin pipe-observed-manipulation
target/release/pipe-observed-manipulation \
  --scenario scenarios/observed_manipulation_m1f_v2.json --compact
python3 scripts/check_m1f_robustness.py \
  --binary target/release/pipe-observed-manipulation \
  --report out/m1f_robustness.json
```

`scenarios/robustness_m1f_v1.json` defines 44 cases: required nominal completion
with byte-identical replay; a 27-case grid spanning three seeds, initial peg
X errors of 55/65/75 µm, and three two-axis socket tilts; separate backlash,
calibration-bias and latency sweeps; and five required fault classifications.
The peg Y/Z offsets stay at -45/+30 µm. The socket tilt pairs are
[-28,+24], [0,0], and [+28,-24] mrad.

Exploration records completed cycles, guarded stops, and scenario-admission
refusals separately, with exact reasons and the terminal physical score.
It does not label a stop as successful manipulation. Acceptance requires the
nominal replay, the specified fault outcomes, no safety violation in any
executed case, and at least 15 completed grid cases. The completed grid points
describe an empirical modeled operating set; they establish neither a
continuous guaranteed envelope nor statistical hardware yield.

CI runs the matrix and uploads the compact matrix report and full nominal
report. Scenario and report schema 2 preserve axis-command intent and the
fixed-head contract. The existing WASM `fromScenarioJson` path accepts M1f;
`observedManipulationSupportedSchemaVersionsJson` advertises versions 1 and 2.
The historical singular schema getters still describe the default M1e cycle.

The checked-in [44-case report](evidence/m1f_robustness_v1.json) records the
native Rust 1.98.1, x86-64 Linux run. All 27 seed/offset/tilt grid cases complete.
Across all 44 cases, 34 complete, six stop safely, and four are refused during
scenario admission. The nominal report replays byte for byte. Every executed
case has zero unplanned penetration and zero stale near-contact commands.
Validation also passes all 310 workspace tests in the release profile, formatting,
Clippy with warnings denied, and the release `wasm32-unknown-unknown` build.

| Terminal modeled metric | Nominal | Maximum across 34 completed cycles |
| --- | ---: | ---: |
| Peg lateral seat error | 1.068 µm | 9.604 µm |
| Peg axial seat error | 7.361 µm | 12.320 µm |
| Peg direction error | 4.279 mrad | 6.726 mrad |

The nominal cycle executes three observed axis corrections and takes 151.532 s
of simulation time. Its peak grip/insertion force proxies are 0.126368 N and
0.007942 N. These are model outputs, not measured accuracy, force, or throughput.
The 6 µm calibration-bias case stops at the calibration gate. Backlash values
of 36/72 µm exceed the declared pickup-capture budget; processing latencies of
40/100 ms exceed the stopped-burst timing contract. Those four admission
refusals do not provide runtime recovery evidence. The five injected fault
profiles stop for their exact declared reasons.

## Remaining fidelity boundaries and next gate

Features are labelled geometric observations, not detections from images.
Lens MTF, focus, correspondence errors, transparent-tube refraction/glare,
thermal calibration drift, and measured timing distributions remain absent.
Pad and insertion forces are uncalibrated proxies. Gravity, slip, breakaway,
and frictional grasp dynamics remain absent. The capsule peg, square socket,
and conservative packaging proxies do not constitute manufacturing drawings.
Local IK and conservative corridors are not a general obstacle-avoiding
planner or a multi-arm reservation system.

Native replay and WASM compilation do not demonstrate actual JavaScript/WASM
versus native golden execution parity. M1f does not yet populate the general
browser scene with estimates. Hardware claims still require the independent
measurements in `HARDWARE_COUPON_M1E.md`, extended with loaded angular correction,
fixed-field visibility, and acquisition-throughput measurements for this
candidate. The next software milestone is observed two-arm handoff, with
explicit grasp ownership and loss-of-observation handling.
