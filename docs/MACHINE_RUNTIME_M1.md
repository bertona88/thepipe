# Machine runtime M1 — architecture decision and implementation contract

Status: M1a and M1b implemented; simulation baseline, not hardware-qualified

This note records the machine-architecture reset and the amendments made while
turning it into an executable milestone. It is subordinate to the safety and
qualification requirements in `REQUIREMENTS.md`.

## Decision

Rust is the sole owner of physical machine state. The browser receives a
versioned description and frame projection and renders the supplied poses. It
does not calculate rail positions, arm forward kinematics, part trajectories,
contacts, or grasp ownership.

The runtime path is:

```text
validated machine config
  -> sequenced MachineCommand
  -> bounded carriage/joint/gripper targets
  -> fixed-step plant and tendon-arm projection
  -> collision/kinematic state
  -> SceneFrame
  -> WASM JSON
  -> renderer
```

`MachineBackend` is the plant-facing boundary. The deterministic simulator is
its first implementation; a hardware adapter can implement the same command
contract later without giving the planner or UI direct access to actuators.

## Scope amendment

The original M1 mixed four risks: state ownership, actuator integration, IK,
and a browser viewer. They are split so each claim has a clean acceptance gate.

| Milestone | Evidence | Status |
| --- | --- | --- |
| M1a — authoritative machine state | Canonical configuration, bounded direct-axis commands, named FK poses, collision capsules, versioned scene export, snapshot-driven browser | Implemented |
| M1b — one-arm point motion | Dedicated calibration target, tool-position IK, time-parameterized path, numeric target error, replay trace | Implemented |
| M1c — simple manipulation | Pick and place a calibration peg; grasp ownership and held-part pose come from the plant | Planned |
| M2 — two-arm handoff | Collision-aware dual grasp, transfer, release, and retreat | Planned |
| M3 — observed-state control | Timestamped estimates and uncertainty drive the controller; truth is evaluation-only | Planned |
| M4 — gearbox integration | Gearbox executive issues real machine goals after the preceding gates pass | Deferred |

The existing gearbox scenario remains an explicitly labelled F1-reduced
software-integration scaffold. Its logical part plant is not evidence that the
new manipulators can execute the assembly.

## Frozen simulation baseline

`scenarios/machine_baseline_v1.json` is the canonical M0 machine input. It is
embedded into native and WASM builds, parsed with unknown-field rejection,
checked for internal consistency, and content-hashed into `SceneDescription`.
All runtime dimensions are SI.

| Item | Baseline |
| --- | ---: |
| Tube inside diameter | 160 mm |
| Tube working length | 320 mm |
| Central qualified-work radius | 40 mm |
| Manipulator count | 4 |
| Shoulder datum radius | 72 mm |
| Axial travel | -150 to +150 mm |
| Maximum axial speed | 30 mm/s |
| Maximum circumferential speed | 30 deg/s |
| Maximum axial acceleration | 100 mm/s² |
| Maximum circumferential acceleration | 120 deg/s² |
| Arm link lengths | 32 / 30 / 15 mm |
| Gripper opening | 0.08 to 2.8 mm |
| Fixed physics step used by the reference app | 1 ms |

The payload, force, metrology, TCP-error, and feature-size values in the file
are qualification targets, not measured capability. The loader rejects any
change that removes the explicit `simulation_baseline_not_hardware_qualified`
status.

## Rail topology decision

M0 selects `paired_belt_end_bogies`: each manipulator's axial guide is carried
by a paired bogie at the tube ends; coordinated belt motion changes azimuth,
and carriage motion along that guide changes axial position. This makes `theta`
and `z` independent commanded axes while keeping the small arm local to the
work.

The runtime models the idealized kinematic result plus velocity, acceleration,
and axial limits. Belt stretch, differential skew, service-loop keep-outs,
encoder quantization, friction, brake behavior, and loss-of-power behavior
remain hardware-identification work. The topology name therefore identifies a
design hypothesis, not a validated mechanism.

## Frames and serialization

- `pipe_world` is right-handed.
- `+Z` follows the tube axis.
- `+X` is radial zero.
- positive `theta` rotates from `+X` toward `+Y`.
- persistent translations are metres and angles are radians.
- persistent quaternions are explicitly named and ordered `[x, y, z, w]`.

Forward kinematics exports carriage, shoulder, elbow, wrist, and tool poses,
the world-space tool axis, jaw poses, and link collision capsules. Stable
manipulator and rigid-body IDs join the static and dynamic scene records.

## Scene contract amendment

A single monolithic snapshot would resend topology and geometry every tick.
The implemented contract separates them:

| Object | Lifetime | Contents |
| --- | --- | --- |
| `SceneDescription` | Once per reset/configuration | Schema version, config ID/hash, units, frame convention, tube, rail limits, manipulator geometry, rigid-body shapes, sensor datums |
| `SceneFrame` | Every sampled tick | Tick/time, physical truth, optional estimate, commanded targets, full poses, velocities, tendon telemetry, gripper state, bodies, grasp ownership, contacts |

`SceneFrame` keeps `truth`, `estimate`, and `commanded` as separate fields.
Simulation/debug builds may expose truth. Hardware and closed-loop acceptance
must populate estimates from timestamped observations and must not silently
copy truth into the estimate field.

Live step responses carry the current `SceneFrame`; the version-1 acceptance
report keeps its compact control/telemetry trace and does not duplicate a full
scene into every stored record. A dedicated compact replay stream remains a
later interface milestone.

The JSON schema is versioned independently from the existing report schema.
Breaking field, unit, quaternion-order, or semantic changes require a scene
schema increment.

## Command and motion semantics

The M1a command vocabulary is:

- `MoveCarriageZ`
- `MoveCarriageTheta`
- `SetJointTargets`
- `SetGripperOpening`
- `Stop`

Targets are validated before mutation. Out-of-range and non-finite values are
rejected rather than silently clamped. Accepted commands receive a monotonic
sequence number and issue tick. Motion is deterministic, velocity-limited, and
acceleration-limited; circumferential motion follows the shortest wrapped path.
`Stop` captures current axis and jaw positions, zeros velocities, and preserves
an existing grasp instead of opening the tool.

`SetToolPoseTarget` now provides M1b Cartesian point motion. Its target is a
world-space position; tool orientation remains unconstrained and the current
wrist roll is preserved. The deterministic carriage-first solver aligns rail
azimuth to the point, puts carriage Z as close as its limits allow, and solves
the shoulder/elbow pair with explicit reach and joint-limit rejection.

Accepted solutions are executed with one synchronized cubic smoothstep over
carriage and joint coordinates. Duration is derived from every configured
velocity and acceleration limit and uses the baseline commissioning speed
scale. A preflight samples the complete configuration-space path at no more
than 1 degree or 0.25 mm between checks and rejects arm/body, inter-arm, and
non-adjacent self collisions before target state is mutated. Plans requiring
more than 4,096 samples are rejected instead of weakening those bounds. This
is bounded sampled collision checking, not yet a continuous swept-volume proof.

`PointMotionRuntime` isolates this acceptance path from the legacy gearbox
executive. Its default calibration point is `[0.020, 0.000, 0.000]` m. Native
and WASM callers receive authoritative scene frames plus a fixed-step trace
containing target/actual position, TCP error, progress, rail coordinates, and
joint positions. The final sample lands exactly on the solved state.

## Browser boundary

The operator console fetches `SceneDescription` once at reset and consumes the
`SceneFrame` attached to each Rust step. The active renderer maps supplied
poses to drawing primitives. If the WASM runtime or scene schema is absent, it
shows the machine scene as unavailable instead of inventing a physical pose.

Display interpolation is allowed later if it is clearly a visual interpolation
between authoritative frames. It must never feed back into contacts, task
guards, or command state.

## Verification gate

M1a is accepted when all of the following pass:

- configuration parsing, status, hashing, and CAD-datum parity;
- rail limit, wrap, speed, acceleration, and deterministic-trace tests;
- command validation, sequencing, stop behavior, and fixed-step projection;
- named FK frames, jaw poses, collision geometry, bodies, contacts, and stable IDs in the scene;
- native and WASM code compile against the same scene types;
- browser bridge rejects unknown scene schema versions;
- the browser renders machine geometry only when Rust scene state is present;
- Cartesian targets reject non-finite, unreachable, joint-limited, and
  collision-bearing requests without changing command sequence or targets;
- the calibration target completes under synchronized velocity/acceleration
  bounds with a deterministic trace and numeric final TCP error;
- native and WASM point-motion runtimes expose the same scene and report data;
- formatting, unit tests, clippy with warnings denied, WASM release build, web verification, and CAD tests pass in CI.

## Explicit remaining risks

- The task executive still maps logical gearbox commands to diagnostic arm
  targets; parts do not yet follow executed gripper trajectories.
- M1b has analytic tool-position IK and a sampled collision-aware joint path,
  but no constrained orientation IK, singularity metric, continuous collision
  checking, obstacle-avoiding planner, or multi-arm carriage optimizer.
- Serial-arm collision contacts are diagnostic and are not yet safety-gating
  the legacy gearbox acceptance result.
- The estimator layer is empty, so `estimate` is `null` and controllers are not
  yet closed around observed machine state.
- Watchdog, brake, force-limit, and commissioning-scale values are contracts;
  they are not yet a complete safety state machine.
- The selected rail topology and all qualification targets require physical
  prototype and calibration evidence.

The next physical experiment should be a calibration peg and socket, not a
gearbox: move one carriage, solve one tool target, approach, grasp or insert at
low speed, report target/contact error, retreat, and replay the Rust frames.
