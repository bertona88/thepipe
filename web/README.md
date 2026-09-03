# The Pipe operator console

Engineering UI for the deterministic Rust/WebAssembly gearbox simulator.

## Run locally

From `web/`:

```bash
npm run check
npm test
npm run build
npm run preview
```

Then open `http://localhost:4173/web/`. The interface looks for the generated
WASM wrapper at `dist/wasm/pipe_sim_wasm.js` relative to the repository root.
Generate it first with:

```bash
./scripts/build_wasm.sh
```

The static build copies that wrapper into `web/dist/wasm/` when it is present.
If the wrapper is absent, the console enters a clearly labelled deterministic
UI-preview mode. That preview exercises controls and telemetry only; the
physical viewport remains unavailable rather than inventing machine poses.

The wrapper supplies a static `SceneDescription` after reset and a dynamic
`SceneFrame` on each step. The renderer consumes the supplied carriage,
shoulder, elbow, wrist, tool, jaw, body, and contact state; JavaScript does not
run machine forward kinematics.

The generated module also exports `PointMotionSimulator`, the isolated M1b
Cartesian calibration runtime. `setCalibrationTarget()` runs the canonical
20 mm radial point, while `setToolTarget(manipulatorId, xM, yM, zM)` accepts a
custom world-space point. `stepJson()`, `runUntilSettledJson()`, and
`reportJson()` expose the authoritative scene and deterministic TCP-error
trace without letting the gearbox executive overwrite the command.
`runCalibrationCycleJson()` executes the canonical approach, touch, and
retreat sequence and returns one trace per leg.

## Keyboard controls

- `Space`: run or pause
- `→`: advance one executive cycle
- `R`: reset the selected scenario
- `1`, `2`, `3`: switch 3D, top, and macro views
