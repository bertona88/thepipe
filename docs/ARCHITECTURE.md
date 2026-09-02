# Pipe Micro-Assembly Simulator — Technical Architecture

Status: normative target architecture with current boundaries called out, revision 0.1
Normative companion: `docs/REQUIREMENTS.md`

This document describes the intended qualified system. Present-tense descriptions of target
interfaces are not an implementation-status claim; see `IMPLEMENTATION_STATUS.md` for the
executable subset. The current end-to-end simulator is an **F1-reduced integration scaffold**,
not the normative F1 milestone specified below.

## 1. Architecture principles

1. **One simulation core.** Native and WebAssembly reference builds call the same Rust state-transition, reduced mechanics, sensing, and task code. A fuller estimator and optional Rapier scheduler integration remain target work.
2. **CAD is generated, not hand-edited.** build123d owns dimensional geometry and manufacturing exports. The current Rust CLI validates a versioned metadata manifest against its reduced model; runtime mesh ingestion is a later fidelity step.
3. **Estimation is separated from truth.** The target controllers receive timestamped sensor observations and estimates. Only evaluation and debugging may access simulator truth.
4. **Precision is closed-loop.** The target tendon mechanisms are compliant and hysteretic; optics and local force evidence correct them near the work. The current reduced plant does not yet close this loop through executed arm trajectories.
5. **Determinism is designed in.** Fixed steps, stable iteration order, seeded streams, hashed assets, and replay logs are first-class requirements.
6. **Fidelity is selectable and declared.** Fast kinematic studies, default engineering runs, and slow tooth/contact validation share scene data but never masquerade as each other.
7. **The core is headless.** The operator console is an adapter over a versioned Rust scene contract, not the owner of simulation state.

## 2. Proposed repository boundaries

The following layout is a target boundary map; it does not require all crates to exist on day one.

```text
cad/
  pipe_cad/                 # build123d Python package
  parameters/               # cell, arm, tools, gearbox YAML/JSON
  scripts/                  # generate, tessellate, validate, export
  tests/                    # dimensional and interference checks
assets/generated/
  manufacturing/            # STEP plus nominal 2PP exchange meshes
  visual/                   # glTF/GLB meshes, materials
  collision/                # convex hulls and tooth-resolved meshes
  metadata/                 # frames, joints, mass properties, hashes
crates/
  pipe-schema/              # serde schemas, units, IDs, versioning
  pipe-geometry/            # transforms, kinematics, reachability
  pipe-actuation/           # tendon, capstan, motor, backlash models
  pipe-physics/             # Rapier/Parry adapter and contact policy
  pipe-optics/              # cameras, projector, visibility, image noise
  pipe-estimation/          # calibration, tracking, fusion, covariance
  pipe-control/             # IK, visual servo, force and tendon control
  pipe-planning/            # collision-aware motion and reservations
  pipe-assembly/            # task graph, guards, recovery behaviours
  pipe-sim/                 # scheduler and authoritative world transition
  pipe-report/              # metrics, trace, replay, human report
  pipe-cli/                 # native headless executable
  pipe-wasm/                # wasm-bindgen boundary; no UI code
scenarios/
  gearbox_baseline_v1/      # scene, task, calibration, seed, expected gates
tests/
  fixtures/                 # tiny deterministic scenes and golden events
  integration/              # end-to-end and cross-target checks
```

Generated assets shall carry the source parameter hash, generator version, unit declaration, and tessellation settings. They may be checked in for reproducible builds; manual modification of generated files is prohibited.

The current digital-thread gate is manifest-level: a file-backed scenario names a CAD
metadata path relative to itself, and the CLI verifies the schema, canonical parameter
and geometry-facts SHA-256 values, valid-BREP flags, named insertion sequence, an enumerated
dimensional subset, and a pinned whole-document scenario-contract digest except for the relocatable metadata
path before it constructs the compiled F1-reduced scenario. Reports retain separate scenario,
CAD-parameter, CAD-geometry, and combined run hashes. STEP/STL geometry remains an offline
manufacturing and inspection artifact until a later runtime mesh loader is implemented.

## 3. CAD and digital-thread pipeline

### 3.1 build123d model ownership

The build123d package shall generate:

- tube, end frames, service apertures, and camera/projector mounts;
- mobile-base carriage, rollers, cable guides, capstans, and keep-out sweep;
- arm links, pins, joint hard stops, tendon paths, and strain relief;
- each end effector in relevant states;
- passive fixture, trays, calibration target, and gauge artefact;
- housing, involute gears, shafts, cover, inspection datums, and optional external 2PP qualification arrays; and
- assembly-level reference frames and exploded assemblies.

Parameters shall be typed and validated before solid generation. Dimensional values live in one parameter source; mesh scale factors shall never repair a unit error later.

### 3.2 Exports

| Export | Purpose | Rule |
|---|---|---|
| STEP | manufacturing/editable solid interchange | exact B-rep, one named solid per part |
| 3MF or STL | nominal 2PP supplier exchange and collision reference | uncompensated nominal geometry; chord error <= 1 um for gearbox teeth/bores, <= 30 um for cell parts |
| GLB/glTF | visual and optical scene | indexed triangles, normals, material/spectral tags, millimetre metadata |
| Convex/compound mesh | real-time dynamic collision | no visual-only embellishment; verified to conservatively contain the physical part where safety matters |
| Tooth mesh | F2 functional test | high-resolution involute flank and root; independent convergence variants |
| Metadata JSON | joints, frames, density, mass/inertia, tendon attachment/routing, tool points, fiducials, collision filters | schema-versioned and content-hashed |

Mass properties shall be computed from exact CAD volume, nominal density, and explicit stock hardware. The Rust loader shall compare exported mass/inertia to an independent sanity calculation and reject non-positive or implausible values.

### 3.3 Geometry checks

CAD CI shall check disconnected solids, invalid B-reps, duplicate frame names, joint sweep interferences, tube/service access, gearbox centre distances, and the chosen supplier's externally qualified design-rule envelope. A tooth-profile test shall sample the involute and verify pitch/base/addendum/root radii, tooth count, and intended backlash. The CAD pipeline shall not simulate voxel exposure, laser scan paths, polymerization, shrinkage, supports, development, or cure.

build123d is an Open Cascade-based parametric Python CAD library; its official documentation is the implementation reference: <https://build123d.readthedocs.io/>.

## 4. Runtime state and scheduling

### 4.1 Authoritative state

`pipe-sim` owns an immutable-input/mutable-state world containing:

- rigid-body pose, velocity, mass, inertia, and collider handles;
- joints, motors, capstans, tendon states, and tool/grasp states;
- sensor configurations, clocks, pending exposures, and observations;
- calibration parameters and estimator belief states;
- planner reservations, trajectories, and task state;
- truth-only evaluation accumulators; and
- named random streams and event sequence numbers.

Each entity has a stable typed ID. Entity iteration shall be by stable ID, never hash-map insertion order.

The implemented machine-runtime foundation is specified in `MACHINE_RUNTIME_M1.md`.
`pipe-sim-core` now owns cell-level tube, carriage, manipulator-motion, gripper,
safety-target, and qualification-target types plus a sequenced `MachineCommand`
boundary. `scenarios/machine_baseline_v1.json` is parsed and hashed once, and
native/WASM adapters project the same state into a static `SceneDescription`
and dynamic `SceneFrame`. The dynamic schema keeps truth, estimate, and
commanded targets separate. The estimator is not implemented yet, so its field
is absent rather than being populated from truth.

### 4.2 Multi-rate deterministic loop

The scheduler advances in integer microseconds. The baseline cadence is:

| Subsystem | Cadence | Notes |
|---|---:|---|
| Physics/contact | 1,000 Hz | fixed 1 ms step; optional smaller F2 substeps |
| Motor/tendon control | 500 Hz | spool command, tension, friction/hysteresis |
| Joint/Cartesian control | 250 Hz | IK/Jacobian, limits, force guard |
| Camera exposures | 30 or 60 Hz | independent clock, exposure interval, delivery latency |
| Projector codes | 60–240 Hz logical patterns | synchronized sequence with explicit dropped frames |
| Estimator update | event-driven, max 120 Hz | timestamped prediction/update, outlier rejection |
| Planner/task executive | 20–50 Hz plus events | guard evaluation and replanning |
| Log sample | configurable 100–1,000 Hz | binary or compact structured trace |

Order within a physics tick is fixed: deliver due observations, update estimates, advance task guards, compute controls, update tendon/motor forces, step physics, evaluate contacts/safety, sample sensors, then append ordered events. Safety events can pre-empt lower-priority commands at the next tick.

### 4.3 Randomness

Use a specified portable PRNG with an explicit algorithm/version, such as ChaCha8. Derive independent streams from the scenario seed and stable labels (`part_metrology`, `camera/3`, `tendon/arm2`, `contact`) so adding camera noise does not change part perturbations. Every sampled parameter is written to the trace header.

## 5. Physics and collision architecture

### 5.1 Engines and responsibilities

Rapier 3D and Parry are the target rigid-body/contact and proximity engines for higher-fidelity
studies; both support Rust and WebAssembly. The independently tested `pipe-physics` adapter is
not connected to the current reference scheduler. Native and WASM F1-reduced runs instead use
the deterministic analytic collision path in `pipe-sim-core`. Official references:
<https://rapier.rs/docs/> and <https://parry.rs/>.

`pipe-physics` shall wrap engine types behind project-owned interfaces so collision policy, units, deterministic ordering, and logging remain under project control. No other crate talks directly to engine handles.

Normative F1/F2 acceptance builds shall use 64-bit real values and recenter micro-contact computations in a gearbox-local frame before handing them to the narrow phase. All public state remains SI. Contact offsets, permitted penetration, and solver `length_unit`/stabilization parameters shall be explicitly scaled for 0.1 mm gear teeth; engine defaults tuned for metre-scale bodies are not acceptable without a scale test. Native and WebAssembly builds shall use the same scalar type and settings once that engine is integrated.

### 5.2 Collider hierarchy

- Tube wall, fixed mounts, and fixture: static triangle or compound colliders.
- Mobile bases and arm links: conservative compound-convex colliders.
- Cables: swept capsules for collision/visibility; their flexible dynamics may remain reduced-order.
- Gripper fingers and insertion tip: small convex colliders with continuous collision detection.
- Held parts: their own rigid bodies joined by a breakable grasp constraint, never fused invisibly into the tool.
- Gears in F0/F1: conservative body collider plus analytic bore/shaft and gear-mesh relations.
- Gears in F2: tooth-resolved triangle/convex-decomposed collider, reduced time step, convergence check.

Loose gearbox parts shall include gravity and a calibrated linear/quadratic air-drag surrogate. Optional surface-adhesion break force is allowed as an F2/F3 identified parameter because capillary/electrostatic/van-der-Waals effects can dominate microgram part weight; it is a contact surrogate, not a fabrication-process model.

Collision groups shall distinguish permitted contacts (finger–held part, gear–shaft, intended gear mesh) from forbidden ones. “Allowed” changes reporting and constraint handling; it does not disable force or penetration measurement.

### 5.3 Contact material table

Materials are referenced by ID and provide density, static/dynamic friction distribution, restitution, contact stiffness/damping surrogate, optical tags, and uncertainty source. Initial 2PP-polymer/polymer and 2PP-polymer/steel values are broad priors. F3 values come from physical incline, pull, insertion, and torque measurements.

### 5.4 Tendon and actuator model

The target normative F1 tendon is a reduced-order transmission:

1. motor electrical/torque-speed/current limit;
2. gear train or stepper quantization and backlash;
3. capstan rotation to paid-out length;
4. routing-dependent capstan/Bowden friction using direction and wrap angle;
5. elastic tendon stretch plus pretension;
6. joint moment from tendon moment arm; and
7. direction-dependent deadband/hysteresis state.

F2 may discretize long cable paths as massless spring segments with guide contact, but full finite-element cable dynamics are not required for gearbox acceptance. Each model reports tension and energy; negative tension becomes slack, never compressive force.

### 5.5 Gear functional models

The current F1-reduced report constructs a stateful analytic gear train after the task loop and
only after insertion, mesh and latch gates pass. It maps angle through tooth ratios and consumes
backlash on reversal; it is not a rotary-tool/vision measurement executed by the task executive.
Its force and torque values remain separate uncalibrated gate surrogates. Normative F1 shall add
the in-loop measurement, identified efficiency, friction torque and a jam criterion without
pretending cylinders are tooth-resolved contacts.

F2 disables the analytic constraint for the final validation and uses actual involute tooth contacts. The run shall be repeated with at least two mesh chord errors and two physics substeps. If output angle or peak torque changes more than 5%, the result is numerically unconverged and cannot pass the F2 gate.

## 6. Optical simulator

### 6.1 Two backends

The target `pipe-optics` boundary shall expose one observation schema with two interchangeable backends:

- **Deterministic geometric backend (acceptance/default):** CPU triangle ray queries or analytic projection for fiducial corners, depth samples, visibility, projected codes, distortion, blur/error surrogate, outliers, and timing. It is fast, headless, reproducible, and works in WebAssembly.
- **Rendered backend (diagnostic/F2):** offscreen GPU rasterization through `wgpu` or a native reference renderer, producing intensity, depth, normal, and object-ID buffers before camera noise/ISP effects. It validates glare/contrast and non-fiducial tracking assumptions, but GPU pixels are not the determinism oracle.

The optics crate currently implements the geometric primitives, camera/projector triangulation,
noise, fiducial, drift and covariance utilities. The F1-reduced integration uses the locked
60 mm/±106 mm global layout and rigid 12 mm macro pair numerically in a commanded
active-component-local sensing frame; it does not attach those cameras to fixed CAD world or arm
transforms. It ray-tests active/placed part spheres plus an optional synthetic occluder and requires
returns from distinct cameras. Neighboring rays are averaged within each camera before
distinct-view information fusion, with a 3 µm correlated calibration floor so pixel count cannot
manufacture certainty. Successful rays gate a pose observation synthesized from the latent
component pose error; the nominal shaft length, gear tip radius or cover corner radius is
used only as a lever arm for orientation uncertainty. This is not an image-to-CAD 6D estimator.
Nominal Brown–Conrady distortion is zero, and the reference run does not ray-test the complete
tube, arms, tools, tendons or CAD mesh scene.

The rendered backend may approximate the transparent tube with measured contrast loss and reflection layers if physically based refraction is unavailable. Any such approximation is recorded in the report.

### 6.2 Camera observation pipeline

For each exposure:

1. sample body poses across the exposure interval;
2. transform labelled optical features into camera coordinates;
3. clip and test full-scene occlusion;
4. project using calibrated intrinsics and Brown–Conrady distortion;
5. apply point-spread/pixel integration, exposure, shot/read noise, clipping, and quantization;
6. run simulated detection, including missed/false/outlier data;
7. timestamp at the camera clock and queue with transfer/processing latency; and
8. deliver an observation containing measurements, uncertainty, IDs when decoded, and detector quality—not truth pose.

Cable collision meshes participate in visibility. A visual scene that omits tendons and service loops is invalid for observability results.

### 6.3 Structured light and local sensing

The projector is represented as an inverse camera with pattern/time state. Gray-code or phase-shift correspondence produces rays; camera/projector triangulation returns a point cloud with confidence and invalid regions. The default task uses sparse structured-light inspection for empty seats and cover gap, rather than attempting a dense reconstruction every control tick.

Macro tool cameras use the same model with a 4 × 3 mm or smaller field of view, 10–25 mm working distance, and a simultaneous stereo/projector baseline. They qualify successive views inside the 12 × 12 × 8 mm micro-assembly zone, not the whole global work volume at once. Force sensing may initially be optical deflection of a compliant tool flexure; its bandwidth, <= 0.5 mN nominal resolution, noise, saturation, and bias must be explicit. Tendon-current inference alone is not sufficient for the 5 mN gear-placement gate.

## 7. Calibration, estimation, and truth firewall

### 7.1 Calibration data flow

Calibration is a versioned input artifact with provenance:

```text
raw target detections
  -> camera/projector intrinsics
  -> multi-view extrinsics and tube frame
  -> arm kinematic/tendon identification
  -> tool-centre-point transforms
  -> independent gauge validation
  -> signed calibration bundle
```

The same algorithms shall operate on simulated observations and physical CSV/image detections. Simulation-only shortcuts must be confined to test fixtures.

### 7.2 Estimator

The estimator shall fuse encoder/spool state, tendon model prediction, fiducial corners, part features, structured-light points, and tool force evidence. An error-state EKF is adequate for body tracking; batch bundle adjustment handles calibration. All outputs include timestamp and covariance.

Outlier rejection shall use reprojection residuals and geometry, not object truth. Covariance growth during occlusion is required. The task executive uses covariance gates from the requirements, so an apparently correct but unobservable move pauses.

### 7.3 Truth firewall

Crate boundaries enforce access:

- `pipe-sim` and `pipe-report::evaluator` can read truth;
- sensor backends receive truth only to synthesize observations;
- `pipe-estimation` consumes observations and commands;
- planner, control, and assembly crates accept `EstimatedWorld`, not `SimWorld`;
- debug APIs are feature-gated and acceptance binaries refuse that feature.

An integration test shall deliberately offset a part from its nominal scene pose and prove the controller follows the observation, not the scene declaration.

## 8. Planning, control, and assembly executive

### 8.1 Motion planning

The first gearbox sequence does not require a general autonomous factory planner. It requires a reliable motion service with:

- damped-least-squares IK with joint/velocity/force limits;
- deterministic RRT-Connect or equivalent collision-free free-space planning;
- Cartesian approach/insert/retreat primitives;
- time parameterization with base/arm limits;
- multi-arm space-time reservations and a 0.5 mm predicted-separation guard;
- held-part collision geometry; and
- replanning from the latest estimated state.

The planner shall return a reason code on failure: unreachable, collision, singularity, uncertainty, time reservation, or limit violation.

### 8.2 Closed-loop control

Free-space motion follows joint/tendon trajectories using encoder plus model feedback. Within a configurable capture distance, image-based or pose-based visual servo closes the final alignment loop. Insertion uses hybrid position/force control with velocity and force guards. Tendon compensation may feed forward, but visual and force residuals decide task progress.

### 8.3 Multi-arm executive

`pipe-assembly` represents `gearbox_baseline_v1` as a versioned task graph. Each node contains preconditions, resource locks, observation requirements, action, success gate, timeout, retry budget, cleanup, and failure code. Arms, QWV sectors, cameras, tools, and held parts are explicit resources.

Priority order is safety, part retention, observability, fixture stability, then throughput. The executive may move an observer or wait for visibility; it may not skip a required inspection or functional gate to claim success.

The cover handoff is a dedicated two-arm protocol:

1. giver establishes stable grasp and presents a known handoff pose;
2. receiver visually aligns and closes with low force;
3. both grasps are verified;
4. load transfers while the observer confirms cover pose;
5. giver releases only after receiver margin and visibility pass; and
6. any disagreement returns the cover to a safe tray or dual-held pose.

## 9. Scenario and configuration schemas

Persistent files shall be human-diffable JSON, JSON5, TOML, or YAML, with a canonical serialized form for hashing. Schemas include:

- asset manifest and frame tree;
- body, joint, collider, material, tendon, motor, and tool definitions;
- cameras, projector, lights, clocks, and noise models;
- calibration bundle and covariance;
- part trays, fixture, and initial-state distributions;
- task graph and acceptance gates;
- fidelity selection and numeric solver settings; and
- named output metrics.

All dimensional fields include units in the field name or use a strongly typed tagged value. Unknown schema versions and missing hashes are fatal in acceptance mode.

## 10. Outputs and replay

Every run produces:

- `run_manifest.json`: build/version, target, feature flags, assets/hashes, scenario, fidelity, seed, sampled parameters;
- `events.jsonl`: ordered task, safety, contact, observation-quality, retry, and failure events;
- `telemetry` stream: commands, estimates/covariances, truth, tendon tensions, forces, poses, and timing;
- `metrics.json`: every requirement gate with observed value, limit, unit, and pass/fail;
- `report.md`: concise human-readable outcome and qualification limits;
- optional camera/depth frames and contact snapshots; and
- a replay file sufficient to reproduce visualization without rerunning planning.

Truth fields are labelled and physically separated from estimated fields. Reports show both estimator error and the estimate that drove the controller.

## 11. Native, WebAssembly, and later frontend boundary

### 11.1 Native reference

The current native CLI runs one compiled baseline headlessly, validates a file-backed CAD
manifest over a defined field subset, and emits one structured JSON report. The target native
tooling shall additionally own batch sweeps, F2 high-resolution contact runs, calibration
fitting, richer CAD asset validation, and run-directory artifacts.

### 11.2 WebAssembly adapter

The current `pipe_sim_wasm` adapter constructs a compiled scenario by name, advances one or many
bounded cycles, and returns JSON snapshots/reports. It also exposes the versioned static machine
description and current truth/estimate/commanded scene frame used by the browser renderer. It does
not load the file-backed CAD manifest.
The target adapter shall additionally expose:

- load validated asset/scenario bytes;
- initialize with fidelity and seed;
- advance a bounded number of fixed ticks;
- submit pause/resume/fault commands;
- read compact state snapshots and ordered events; and
- export the run report/replay.

The adapter shall avoid blocking loops, filesystem assumptions, native threads, and nondeterministic browser clocks. JavaScript never mutates Rust world state directly. Large geometry is loaded once; snapshots use typed arrays or a compact binary representation.

The website may interpolate display transforms and render pretty meshes, but it cannot determine
rail or arm kinematics, contacts, grasp ownership, task gates, or success.

## 12. Verification strategy

### 12.1 Unit and property tests

- transform/frame composition and unit conversions;
- forward/inverse kinematics and Jacobians against finite differences;
- capstan length, tendon tension, slack, backlash, and energy bounds;
- camera projection/distortion round trips;
- triangulation uncertainty versus analytic cases;
- collision group policy and grasp break thresholds;
- gear involute dimensions, ratios, backlash, and direction;
- schema migration/rejection and content hashing; and
- seeded-stream independence.

### 12.2 Golden deterministic scenes

Small scenes isolate free fall, a sliding block, pendulum, tendon joint, jaw grasp, vacuum break, shaft insertion, camera occlusion, structured-light plane, two-gear mesh, and multi-arm crossing. Golden tests compare discrete events and bounded metrics, not raw cross-platform float bytes.

### 12.3 End-to-end tests

1. CAD assets regenerate and hashes match.
2. F0 reach/visibility maps cover the QWV.
3. Synthetic calibration recovers planted parameters within expected covariance.
4. Estimator never accesses truth; an offset-world test demonstrates it.
5. Canonical gearbox sequence passes at F1 on native and WebAssembly.
6. Same-seed event sequences match across targets.
7. All listed fault injections stop/recover with the required reason.
8. The 100-run robustness suite emits statistics and preserves every failing seed.
9. F2 gear test demonstrates time-step and mesh convergence.

### 12.4 Hardware correlation hooks

Physical controller and simulator logs shall share observation, command, estimate, and event schemas. Identification tools fit tendon stiffness/hysteresis, base slip, camera timing/noise, insertion friction, and gear torque from measured logs. Correlation reports show residuals and the tested envelope; calibration does not overwrite the original prior data.

## 13. Performance budgets

For F1 on a current laptop-class CPU:

- one simulated second shall execute in <= 2 wall seconds natively without saving image frames;
- the WebAssembly build shall sustain at least 20 simulated physics ticks per animation frame when called in chunks, or report that it is behind without dropping physics steps;
- scenario plus collision/visual assets shall remain below 150 MB uncompressed and 40 MB transferred for the later browser build;
- a 12-minute run at normal telemetry settings shall remain below 500 MB; and
- batch mode may disable rendered optics but not geometric visibility, timing, noise, or estimator logic.

F2 tooth-resolved contact and rendered optics are allowed to run slower than real time. Reports include wall time, physics step, solver iterations, mesh level, and dropped sensor frames.

## 14. Initial build order

1. Freeze schemas, frames, units, scenario seed, and report gates.
2. Generate build123d gearbox, fixture, simple arm solids, and collision metadata.
3. Implement fixed-step `pipe-sim`, kinematics, F0 collision, trace, and CLI.
4. Add tendon transmission and F1 rigid contact; validate on small golden scenes.
5. Add geometric cameras, latency/noise, calibration, estimator, and truth firewall.
6. Implement guarded pick/insert/mesh primitives and the deterministic task graph.
7. Pass the canonical F1 assembly and fault-injection suite natively.
8. Compile the same core to WebAssembly and compare event traces.
9. Add F2 tooth contacts, compliant insertion refinements, rendered optical diagnostics, and convergence tests.
10. Send the nominal gearbox through external 2PP process qualification, inspect delivered parts, and feed metrology/contact measurements into the F3 correlation layer before physical assembly trials.

This order deliberately puts the gearbox, collision bodies, sensor error, and task gates ahead of visualization. A later frontend can consume stable replay and state interfaces without changing the engineering result.
