# The Pipe — hardware-first micro-assembly simulator

This repository is the engineering core for a tube-shaped robotic micro-assembly cell. It
contains low-cost tendon-arm models, printable machine fixtures, idealized gearbox parts,
reduced collision checks, structured-light/multi-camera primitives, and a deterministic guarded
gearbox task. Rust now also owns a canonical machine configuration, bounded carriage/arm command
state, and a versioned physical scene consumed by the WebAssembly operator console. M1e adds a
separate observed-state single-arm coupon runtime: timestamped macro camera/projector feature
measurements feed an uncertainty-bearing axisymmetric pose estimator, bounded stop-and-look
corrections, guarded grasp/contact transitions, incremental insertion, and fail-closed recovery.
The current end-to-end gearbox run remains a reduced observed-part surrogate; it does not yet
execute real task-space held-part trajectories or couple parts to the four arms. M1c remains as a
deliberately truth-visible mechanics baseline, while M1e is the smallest closed-loop test of the
external-metrology thesis.

This is an engineering simulator, not a photorealistic animation and not yet a validated
predictor of a particular manufacturing process or actuator. Every pass/fail result declares its model
fidelity and assumptions. Replace nominal machine parameters with measured coupon and calibration
data before using it to release hardware.

## Start here

- [What is needed first](docs/NEEDS_FIRST.md)
- [Engineering requirements](docs/REQUIREMENTS.md)
- [Architecture and fidelity boundaries](docs/ARCHITECTURE.md)
- [Machine runtime M1 decision and implementation contract](docs/MACHINE_RUNTIME_M1.md)
- [M1c simple-manipulation acceptance contract](docs/MACHINE_RUNTIME_M1C.md)
- [M1d optical/robot co-design and precision budget](docs/OPTICAL_CODESIGN_M1D.md)
- [M1e observed-state single-arm manipulation](docs/OBSERVED_STATE_MANIPULATION_M1E.md)
- [M1e one-arm hardware coupon qualification](docs/HARDWARE_COUPON_M1E.md)
- [Implemented fidelity versus future work](docs/IMPLEMENTATION_STATUS.md)
- [CAD package](cad/README.md)

## Repository map

| Path | Responsibility |
| --- | --- |
| `crates/pipe_sim_core` | units, poses, tendon-arm mechanics, bodies, contacts and fixed-step state |
| `crates/pipe_optics` | calibrated cameras/projector, visibility, ray/depth noise and observation quality |
| `crates/pipe_physics` | optional f64 Rapier backend with CCD, contact groups and deterministic snapshots |
| `crates/pipe_planner` | guarded gearbox assembly state machine, recovery and acceptance metrics |
| `crates/pipe_sim` | reduced gearbox plant plus the isolated M1c and observed-state M1e single-arm runtimes |
| `crates/pipe_sim_cli` | deterministic headless run and machine-readable report |
| `crates/pipe_sim_wasm` | browser-neutral WebAssembly boundary consumed by the operator console |
| `web` | engineering operator console, viewport, telemetry and report export |
| `cad` | build123d source models, printable exports and dimension manifest |
| `scenarios` | versioned machine, gearbox, optical co-design, and M1e coupon acceptance inputs |
| `scripts` | reproducible build, CAD and verification entry points |

## Browser operator console

The dependency-free operator console lives in `web/`. It connects to the generated
Rust/WASM wrapper and renders the versioned Rust `SceneDescription`/`SceneFrame` contract.
Without that wrapper it can preview controls and telemetry, but it marks the physical machine
scene unavailable instead of synthesizing rail, arm, or part poses.

```bash
./scripts/build_wasm.sh
cd web
npm run check
npm test
npm run build
npm run preview
```

Open `http://localhost:4173/web/` after starting the preview server.

## Quick verification

With stable Rust, Python 3.11+ and build123d installed:

```bash
cargo test --locked --workspace
cargo run --locked -p pipe_sim_cli -- --scenario scenarios/gearbox_acceptance.json --report out/run.json
cargo run --locked -p pipe_sim_cli --bin pipe-manipulation -- --compact
cargo run --locked -p pipe_sim_cli --bin pipe-optical-codesign -- --compact
cargo run --locked -p pipe_sim_cli --bin pipe-observed-manipulation -- --compact
python -m pytest cad/tests
PYTHONPATH=cad python -m pipe_cad.cli gearbox --output cad/out --stl-tolerance 0.01
./scripts/build_wasm.sh
```

For a clean environment, `scripts/bootstrap.sh` creates the Python environment and installs
the Rust WebAssembly target. CAD export is intentionally separate from the real-time loop:
build123d/OCP is used for nominal geometry and exact BREP/dimension checks; the Rust runtime
uses deterministic reduced collision shapes and analytic gear checks suitable for WASM.
For file-backed acceptance runs, the CLI resolves `cad_metadata_path` relative to the
scenario, rejects any scenario-value drift outside that relocatable path, and requires the CAD
manifest schema, parameter and geometry-facts SHA-256 values, BREP validity, part names,
insertion order, and enumerated CAD/runtime dimensions to agree. Reports preserve the separate
scenario, CAD-parameter, CAD-geometry, and combined run hashes. This v1 gate does not construct
runtime bodies or sensing geometry from the manifest, and it does not ingest STEP/STL collision
meshes. The optional Rapier adapter is tested independently for
richer rigid-body studies; the F1-reduced reference run deliberately stays on the same compact
analytic collision path in native and WASM builds. The WASM adapter currently selects compiled
scenario names rather than loading the file-backed CAD manifest.

## Gearbox acceptance test

The initial gearbox is deliberately small enough to expose the hard problems without hiding
behind a large robot: an approximately 6 x 4 x 2 mm housing, three shafts, three 0.10-module
involute spur gears, lead-in chamfers, and explicit backlash. The task executive runs logical
locate, pick, reorient, align, insert, mesh, and rotation-check phases against the reduced plant.
It fails closed when modeled uncertainty consumes the clearance budget; arm motion planning and
held-part trajectory execution remain future work.

The gearbox solids are idealized for assembly studies; eventual 2PP process behavior is an
external measured input. The output is reproducible and engineering-facing: it contains
geometry-derived dimensions, deterministic sensor errors, actuator compliance/backlash,
proxy occlusion and collision clearance, guarded insertion, recovery, and quantitative metrics.
Insertion forces, mesh torque, latch response, and component motion are explicitly uncalibrated
reduced-order surrogates. The reference optical scene uses idealized spheres for active/placed
parts rather than the complete CAD cell. Ray returns gate observation availability and uncertainty,
with neighboring rays averaged inside each camera before distinct-view information fusion and a
3 µm correlated calibration floor. The reported component pose still starts from a synthetic latent
pose error; a nominal feature-size lever arm converts position uncertainty to orientation uncertainty. It is not an image-derived 6D
pose estimate. The final ratio/backlash result is a post-run analytic check rather than a modeled
rotary-tool measurement. It does not claim fluid, thermal, photometric wave-optics, polymer cure,
tooth-contact FEA, physical yield, or normative F1 fidelity.

## M1c calibration manipulation

`SimpleManipulationRuntime` is isolated from the legacy gearbox executive. Manipulator 1 opens,
approaches a 0.40 mm-diameter calibration peg, closes until the reduced compliant-pad model
reports bilateral contact, acquires plant-owned grasp state, retracts, transfers, enters a
four-wall calibration socket, opens until geometric contact is lost, and retreats. Cartesian
preflight sweeps the attached peg as well as the arm links. The native `pipe-manipulation` binary
and the WASM `SimpleManipulationSimulator` expose the same Rust runtime and structured report.

This is an F0 geometry/M1c simulation baseline. The socket has deliberate 0.125 mm radial
clearance so the current unqualified point-motion planner can safety-gate the complete path.
Insertion and retreat follow the tilted socket's local axis rather than a world-coordinate axis.
The runtime has zero gravity, rigid attachment, no gripper/tool collision mesh, no contact-derived
insertion force, and no estimator. Its scene therefore exposes simulation truth while keeping
`estimate` empty. It proves software ownership and sequencing, not micrometre insertion or
hardware performance.

## M1d optical/robot co-design

`scenarios/optical_codesign_m1d.json` freezes a reviewable two-scale optical candidate and its
uncertainty assumptions. The native CLI and WASM static export use one analytic implementation to
predict global/macro precision, sweep macro field width versus baseline, and solve the residual
loaded arm-control allocation for every manipulation phase. A feasible report is explicitly
`model_feasible_hardware_qualification_required`; it is not measured accuracy. See the M1d note
for the proposed camera/projector layout, current 7.9 µm tightest lateral residual allocation, and
the coupon sequence required before camera replication or arm architecture freeze.

## M1e observed-state single-arm manipulation

`scenarios/observed_manipulation_m1e_v1.json` drives a deterministic vertical slice with one
tendon arm, a nominal 0.40 mm calibration peg, one socket coupon, and one local macro
camera/projector head. The sensor boundary produces timestamped labelled feature measurements
with finite field of view, opaque-proxy occlusion/self-shadow checks, localization noise, dropout,
latency, drift, and a correlated calibration floor. Symmetric `+/-0.800 mm` tool targets and
`+/-1.000 mm` external socket targets are observed independently; measured midpoints constrain the
axis without subtracting a latent-pose offset. A deterministic weighted least-squares estimator fits the
translation and axis direction that the circular peg/socket geometry can actually constrain.
Rotation about that axis is deliberately unobservable, so the result is 5-DoF rather than a
fabricated image-derived 6D pose.

The M1e executive enters a calibrated capture volume, stops and settles, acquires observation
bursts, gates covariance/age/residual/visibility, and issues bounded reproducible corrections.
Observed geometry plus bilateral compliant-pad evidence gates grasp; the transfer propagates
held-transform uncertainty; socket and peg features are reacquired before incremental guarded
insertion. Estimated swept envelopes include conservative tool/gripper and carried-peg geometry.
Invalid, stale, occluded, inconsistent, over-uncertain, non-converging, colliding, or excessive
force-proxy states fail closed with an explicit report reason. Recoverable contact may use a fresh,
preflighted in-task reverse move; a terminal failure issues Stop and holds rather than inventing an
unobserved retreat. The fault suite covers all of those decision paths.

The native `pipe-observed-manipulation` CLI and the WASM
`ObservedManipulationSimulator` class expose the same Rust executive and report schema. Host-side
wrapper replay is deterministic and CI compiles that core for `wasm32-unknown-unknown`; an actual
JavaScript/WASM-versus-native golden execution comparison remains deferred and is not claimed as
demonstrated cross-target parity. The WASM surface runs a full cycle and reports status/hash, but
does not yet populate the general browser `SceneFrame` with M1e estimates.

This is an **F1-reduced modeled M1e coupon result, not hardware qualification**. It uses labelled
geometric features rather than rendered images or a detector, opaque primitives rather than clear-
tube refraction/glare, an uncalibrated reduced contact-force proxy, and an uncertain kinematic
attachment without gravity, frictional slip, or breakaway dynamics. Its idealized macro head is
retiled about requested regions without modeling a head actuator, repositioning error/time, or
collision envelope. The approximately 3.0 µm
lateral / 3.4 µm depth M1d optical values and approximately 7.9 µm guarded-insertion residual
allocation remain modeled hypotheses. `docs/HARDWARE_COUPON_M1E.md` defines the measurements that
must replace scenario assumptions before any precision, latency, compliance, force, or yield
claim can refer to hardware.

## Safety boundary

The generated machine is a research fixture. Before energizing hardware, add physical travel
stops, current limits, tendon guards, an enclosure, an emergency stop, and a low-speed
commissioning mode. Never infer safe forces solely from this simulation.
