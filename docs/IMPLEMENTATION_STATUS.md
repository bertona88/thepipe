# Implementation status and fidelity contract

This repository contains an executable engineering foundation, not the entire qualified
machine described in `REQUIREMENTS.md`. The table below prevents an implemented proxy from
being mistaken for a higher-fidelity claim.

| Capability | Current implementation | What remains for a hardware-qualified claim |
| --- | --- | --- |
| Machine and part CAD | Parametric build123d cell, serial tendon arms, jaw grippers, CAD/scenario-locked global sensor datums, a CAD-modeled rigid 12 mm wrist macro head, fixture, ideal involute gearbox, parametric vacuum/probe/rotary/calibration tool solids, and a named export manifest | Runtime STEP/STL collision-mesh ingestion, runtime tool behavior/tool changing, operational projector/macro extrinsic validation, joint sweep optimization, cable/service-loop keep-outs, independent physical mass/inertia checks and hardware drawings |
| Machine runtime | Canonical hashed SI machine configuration; explicit paired-belt/end-bogie topology; bounded and sequenced carriage, joint, gripper, stop, and Cartesian point commands; carriage-first tool-position IK; synchronized limit-derived trajectories; sampled arm and carried-part collision preflight; numeric TCP error and deterministic replay; named FK frames; stable IDs; link collision capsules; standalone M1c calibration-peg manipulation | Constrained-orientation IK, continuous/obstacle-avoiding path planning, carriage optimization, watchdog/safety state machine, identified rail errors, and a hardware backend |
| Tendon mechanics | Deterministic capstan displacement, pretension, stiffness, one 0.018 mm differential lost-motion/backlash parameter, force limits; bounded serial-arm target motion now projects through the tendon/FK model but is not coupled to assembly parts | Identify parameters from measured actuators, add motor electrical/thermal limits and routing-dependent friction, then couple arm/TCP trajectories to held parts |
| Runtime physics | Fixed-step f64 core used by native/WASM, analytic micro-gear clearance, serial-arm collision queries, contact-conditioned reduced jaw grasp/release, kinematic held-part attachment and carried-part path sweeps, plus an independently tested f64 Rapier adapter with collision groups and CCD | Connect Rapier to the reference scheduler, add gripper/tool collision solids and contact-derived insertion forces, tooth-resolved F2 convergence, breakable grasps, calibrated friction/contact compliance and multi-arm trajectory execution |
| Optical sensing | The optics crate implements Brown–Conrady cameras, ray occlusion, camera/projector triangulation, photon/read/quantization/dropout noise, fiducials, drift and covariance fusion; M1d adds a versioned two-scale camera/projector candidate, shared analytic precision propagation, macro field/baseline sweep, and phase arm-residual budgets; the reference run still ray-gates a synthetic latent component pose over multi-part sphere proxies | Exact camera/lens/projector and trigger downselect, image-derived 6D pose, fixed tube-world camera transforms coupled to the CAD cell, complete cell/tool/tendon visibility geometry, nonzero calibrated distortion/drift in the reference run, full intensity rendering, transparent-tube/glare correlation, timing/ISP calibration, loaded-arm hold/correction data and physical macro-rig validation |
| Assembly executive | Logical guarded locate/pick/handoff/align/insert/mesh/verify sequence over a reduced observed-part/force plant, with retry budgets, safety aborts and injected failures; its commands now exercise diagnostic bounded arm state, but part motion remains a separate surrogate | Task-space arm and held-part execution, general collision-free motion planning, multi-arm space-time reservations, physical force-loop integration and estimator-driven hardware commands |
| Gearbox article | Ideal nominal 0.10-module 12/18/24-tooth train, three shafts, housing and cover; the reported forward/reverse ratio and backlash acceptance is calculated analytically after the task loop | A modeled rotary-tool/vision measurement in the task loop, optional metrology-driven perturbation sweeps, external 2PP feasibility/cleaning/metrology and measured friction/wear/stiction data |
| Interfaces | One headless compiled-baseline gearbox run; standalone native/WASM M1b point-motion and M1c simple-manipulation runtimes; native/WASM M1d optical co-design report; structured JSON reports, fixed-step Cartesian and phase-boundary manipulation replay; strict CLI manifest gates; and independently versioned static `SceneDescription` plus dynamic truth/estimate/commanded `SceneFrame`; the operator console renders the Rust poses | General direct machine-command controls, scenario-driven runtime construction, file-backed manifest loading in WASM, batch/robustness tools, compact binary replay/telemetry, estimate population, and native/WASM scene-golden comparison in a browser runner |

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
