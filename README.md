# The Pipe — hardware-first micro-assembly simulator

This repository is the engineering core for a tube-shaped robotic micro-assembly cell. It
contains low-cost tendon-arm models, printable machine fixtures, idealized gearbox parts,
reduced collision checks, structured-light/multi-camera primitives, and a deterministic guarded
gearbox task. Rust now also owns a canonical machine configuration, bounded carriage/arm command
state, and a versioned physical scene consumed by the WebAssembly operator console. The current
end-to-end gearbox run still advances a reduced observed-part plant; it does not yet execute
task-space trajectories or couple parts to the four arms. A separate M1c calibration runtime now
executes one plant-owned peg grasp, carry, socket insertion, release, and retreat; that bounded
coupon cycle is not yet wired into the gearbox executive.

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
- [Implemented fidelity versus future work](docs/IMPLEMENTATION_STATUS.md)
- [CAD package](cad/README.md)

## Repository map

| Path | Responsibility |
| --- | --- |
| `crates/pipe_sim_core` | units, poses, tendon-arm mechanics, bodies, contacts and fixed-step state |
| `crates/pipe_optics` | calibrated cameras/projector, visibility, ray/depth noise and observation quality |
| `crates/pipe_physics` | optional f64 Rapier backend with CCD, contact groups and deterministic snapshots |
| `crates/pipe_planner` | guarded gearbox assembly state machine, recovery and acceptance metrics |
| `crates/pipe_sim` | reduced observed-volume plant, sensor boundary and end-to-end reference run |
| `crates/pipe_sim_cli` | deterministic headless run and machine-readable report |
| `crates/pipe_sim_wasm` | browser-neutral WebAssembly boundary consumed by the operator console |
| `web` | engineering operator console, viewport, telemetry and report export |
| `cad` | build123d source models, printable exports and dimension manifest |
| `scenarios` | versioned machine and gearbox acceptance inputs |
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

## Safety boundary

The generated machine is a research fixture. Before energizing hardware, add physical travel
stops, current limits, tendon guards, an enclosure, an emergency stop, and a low-speed
commissioning mode. Never infer safe forces solely from this simulation.
