# Implementation status and fidelity contract

This repository contains an executable engineering foundation, not the entire qualified
machine described in `REQUIREMENTS.md`. The table below prevents an implemented proxy from
being mistaken for a higher-fidelity claim.

| Capability | Current implementation | What remains for a hardware-qualified claim |
| --- | --- | --- |
| Machine and part CAD | Parametric build123d cell, serial tendon arms, jaw grippers, CAD/scenario-locked global sensor datums, a CAD-modeled rigid 12 mm wrist macro head, fixture, ideal involute gearbox, parametric vacuum/probe/rotary/calibration tool solids, and a named export manifest | Runtime STEP/STL collision-mesh ingestion, runtime tool behavior/tool changing, operational projector/macro extrinsic validation, joint sweep optimization, cable/service-loop keep-outs, independent physical mass/inertia checks and hardware drawings |
| Tendon mechanics | Deterministic capstan displacement, pretension, stiffness, one 0.018 mm differential lost-motion/backlash parameter, force limits; serial and continuum arm models exercised separately from assembly motion | Identify parameters from measured actuators, add motor electrical/thermal limits and routing-dependent friction, then couple arm/TCP trajectories to held parts |
| Runtime physics | Fixed-step f64 core used by native/WASM, analytic micro-gear clearance, plus an independently tested f64 Rapier adapter with collision groups and CCD | Connect Rapier to the reference scheduler, tooth-resolved F2 convergence, breakable grasps, calibrated friction/contact compliance and multi-arm trajectory execution |
| Optical sensing | The optics crate implements Brown–Conrady cameras, ray occlusion, camera/projector triangulation, photon/read/quantization/dropout noise, fiducials, drift and covariance fusion; the reference run ray-gates a synthetic latent component pose over multi-part sphere proxies, requires distinct camera views, and uses nominal feature size only as an orientation-uncertainty lever arm | Image-derived 6D pose, fixed tube-world camera transforms coupled to the CAD cell, complete cell/tool/tendon visibility geometry, nonzero calibrated distortion/drift in the reference run, full intensity rendering, transparent-tube/glare correlation, timing/ISP calibration and physical macro-rig validation |
| Assembly executive | Logical guarded locate/pick/handoff/align/insert/mesh/verify sequence over a reduced observed-part/force plant, with retry budgets, safety aborts and injected failures; task-loop verification motion remains a command-driven surrogate | Task-space arm and held-part execution, general collision-free motion planning, multi-arm space-time reservations, physical force-loop integration and estimator-driven hardware commands |
| Gearbox article | Ideal nominal 0.10-module 12/18/24-tooth train, three shafts, housing and cover; the reported forward/reverse ratio and backlash acceptance is calculated analytically after the task loop | A modeled rotary-tool/vision measurement in the task loop, optional metrology-driven perturbation sweeps, external 2PP feasibility/cleaning/metrology and measured friction/wear/stiction data |
| Interfaces | One headless compiled-baseline native run, structured JSON report with separate scenario/CAD-parameter/CAD-geometry/combined hashes, a pinned normalized whole-document scenario digest plus strict manifest gates in the CLI, and a browser-neutral WASM adapter using compiled scenario names | Scenario-driven runtime construction, file-backed manifest loading in WASM, batch/robustness tools, pause/resume control, binary replay/telemetry compression, the later website and native/WASM event-golden comparison in a browser runner |

## Fidelity labels

- **F0 geometry:** kinematics, reach, conservative collision envelopes and visibility.
- **F1-reduced integration scaffold (current):** deterministic reduced tendon/contact/optics
  models, command-driven component state, ray-gated synthetic pose observations, and analytic
  gear constraints. The final ratio/backlash check runs after the task loop. This exercises
  software integration but does not meet the normative F1 gate in `REQUIREMENTS.md`.
- **F1 engineering acceptance (target):** integrated arm/held-part trajectories, rigid contact,
  estimator state, breakable grasps, complete relevant visibility geometry and the other F1
  evidence required by `REQUIREMENTS.md`.
- **F2 detailed verification:** tooth-resolved contact and rendered optical diagnostics with
  time-step/mesh convergence. Interfaces and CAD assets are prepared, but this tier is not yet
  implemented end to end.
- **F3 correlated hardware:** parameters fitted to calibration coupons, camera captures, force
  curves and completed gearbox tests. No F3 claim is possible before hardware exists.

Reports must name their fidelity. An F1-reduced pass means only that the configured proxies and
numeric guards passed. It is not a normative F1 pass and does not certify 2PP fabrication,
machine safety, physical yield, or feasibility of the multi-arm motion.
