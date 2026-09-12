# Implementation status and fidelity contract

This repository contains an executable engineering foundation, not the entire qualified
machine described in `REQUIREMENTS.md`. The table below prevents an implemented proxy from
being mistaken for a higher-fidelity claim.

## Current integration decision

Steering applied against main at `4ccbd72c954fa8dd699b43b1eb900428942cfdbb`.
This is a direction change, not a new simulation or hardware result.

One observed gear-on-shaft operation in the existing 160 mm ID machine is the
common experiment. The lead engineer coordinating it owns agreement between
geometry, observations, commands, and outcome. The complete gearbox, including
an executed functional check, remains the goal.

| Current evidence | Consequence for the shared operation |
| --- | --- |
| First command probe rejects approach with `ToolPathCollision` at tick 0; static gear/wrist envelopes overlap by about 1.54 mm | Diagnose distal geometry, grasp and envelope fidelity together; this is a conservative-model conflict, not proven hardware interference |
| Zero of 16 candidates passes precision admission; favorable synthetic errors are about 3.7 µm RMS while the conservative relative bound is about 7 µm against a 5 µm gate | Improve feature calibration, marker support/layout or justified relative uncertainty; do not relax the gate to match favorable samples |
| Cylindrical metrology and M1e/M1f still use different sensing paths; standalone optics is 100 mm ID | Integrate through the existing observation and motion boundaries using one explicitly identified physical candidate |
| Gearbox execution remains a reduced surrogate; no hardware-qualified result | Demonstrate arm-owned observed execution and obtain targeted bench evidence before claiming buildability or physical precision |

See [the gear study](GEAR_OBSERVABILITY.md) for reproduction and coupon limitations.
Review contributions by what was learned, which design/model changed, and which
important assumption remains untested. Preserve earlier regressions and report
completed operations separately from controlled refusals.

## Observed gear pickup candidate

The [pickup and return implementation](ARM_PICKUP_RETURN.md) adds an opt-in
5 mm distal standoff, shared CAD/runtime palm/finger/pad solids, physically
located printed reference patches, independently observed gear state and a
modeled fixed support nest. It retains the 160 mm machine and the existing
authoritative runtime. Pickup admission is separate from the unchanged 5 µm
insertion limit. Its clean-revision run completes pickup and return
through all nine phases, with 2,400 recorded samples and independent CAD/runtime
frame agreement. The [committed summary](evidence/arm-pickup-return-f805ce5.json)
records completion and four verified controlled-refusal cases.
Rigid attachment, zero gravity and geometric support-gap release remain
explicit limitations. It does not establish shaft insertion or hardware
buildability. The earlier gear-study failures above remain historical evidence
for the earlier configuration.

## Capability inventory

| Capability | Current implementation | What remains for a hardware-qualified claim |
| --- | --- | --- |
| Machine and part CAD | Parametric build123d cell, serial tendon arms, jaw grippers, CAD/scenario-locked global sensor datums, a CAD-modeled rigid 12 mm wrist macro head, fixture, ideal involute gearbox, parametric vacuum/probe/rotary/calibration tool solids, and a named export manifest | Runtime STEP/STL collision-mesh ingestion, runtime tool behavior/tool changing, operational projector/macro extrinsic validation, joint sweep optimization, cable/service-loop keep-outs, independent physical mass/inertia checks and hardware drawings |
| Machine runtime | Canonical hashed SI machine configuration; explicit paired-belt/end-bogie topology; bounded and sequenced carriage, joint, gripper, stop, and Cartesian point commands; carriage-first tool-position IK; synchronized limit-derived trajectories; sampled arm and carried-part collision preflight; numeric TCP error and deterministic replay; named FK frames; stable IDs; link collision capsules; standalone M1c calibration-peg manipulation; and M1e bounded stop-and-look corrections executed through the same tendon/FK command boundary with configurable backlash, correction floor, loaded hold disturbance, settling, and held-transform uncertainty | Constrained-orientation IK, continuous/obstacle-avoiding path planning, carriage optimization, watchdog/safety state machine, identified rail and loaded-arm distributions, angular correction execution, and a hardware backend |
| Tendon mechanics | Deterministic capstan displacement, pretension, stiffness, one 0.018 mm differential lost-motion/backlash parameter, and force limits; bounded serial-arm target motion projects through the tendon/FK model. M1e couples its one peg to that motion after guarded acquisition through an uncertain kinematic attachment; the gearbox surrogate remains uncoupled | Identify parameters from measured actuators, add motor electrical/thermal limits and routing-dependent friction, replace the coupon attachment with calibrated breakable grasp/load dynamics, then couple real arm/TCP trajectories to gearbox parts |
| Runtime physics | Fixed-step f64 core used by native/WASM, analytic micro-gear clearance, serial-arm collision queries, contact-conditioned reduced jaw grasp/release, kinematic held-part attachment and carried-part path sweeps, plus an independently tested f64 Rapier adapter with collision groups and CCD. M1e adds conservative tool/gripper/peg envelopes, bilateral compliant-pad grasp evidence, uncertain kinematic attachment, raw timestamped pad/contact/force-proxy packets, per-tick force interlocks, and controller-side contact-state classification using fresh observed relative geometry | Connect Rapier to the reference scheduler, ingest complete gripper/tool/coupon collision solids (including the currently optical-only external socket rails), derive insertion evidence from calibrated sensors/contact dynamics, add gravity/friction/slip and breakable grasps, demonstrate tooth-resolved F2 convergence, and execute multi-arm trajectories |
| Optical sensing | The optics crate implements Brown–Conrady cameras, ray occlusion, camera/projector triangulation, photon/read/quantization/dropout noise, fiducials, drift and covariance fusion; M1d adds a versioned two-scale camera/projector candidate, shared analytic precision propagation, macro field/baseline sweep, and phase arm-residual budgets. M1e uses the same optics boundary to observe explicit labelled centreline features with finite field of view, camera/projector visibility, carrier self-shadow probes, timestamps/latency, quality/dropout, calibration drift/bias, and a once-only correlated floor. Its symmetric paired tool/socket targets are independently observed and reduced by measured midpoint. The legacy gearbox still ray-gates a synthetic latent component pose over sphere proxies | Exact camera/lens/projector and trigger downselect, detector/correspondence simulation or real detections, roll-constraining features where tasks need 6D pose, a fixed or actuated physical macro-head model (the current ROI retile has no actuator/pose/sweep dynamics), fixed tube/tool-world transforms coupled to the CAD cell, complete cell/tool/tendon visibility geometry, rendered intensity diagnostics, transparent-tube refraction/glare correlation, timing/ISP calibration, and physical macro-rig validation |
| Estimation and observed control | M1e implements deterministic weighted least-squares fusion for axisymmetric peg/socket/tool 5-DoF states, timestamp/age tracking, explicit covariance, view/head/ray counts, residual and innovation gates, deterministic outlier rejection, command-only prediction, uncertainty growth, bounded stop-and-look correction, phase-specific fail-closed guards, and truth-firewall/replay tests. It does not label unobservable roll as estimated | Hardware-backed feature extraction and calibration provenance, cross-object/camera correlation, angular tool control, identified process noise and bias distributions, continuous tracking between stopped bursts, more general 6D objects, and estimator-driven gearbox execution |
| Assembly executive | The legacy gearbox executive runs logical guarded locate/pick/handoff/align/insert/mesh/verify over a reduced observed-part/force surrogate; its bounded arm state remains diagnostic rather than the owner of held-part motion. Separately, M1e executes one observed-state peg acquire/transfer/reacquire/guarded-insert/release/retreat cycle through the authoritative single-arm runtime. Recoverable contact may use a fresh preflighted reverse move; terminal faults issue Stop+hold with explicit reasons across ten injected profiles | Real task-space held-part trajectories in the gearbox executive, a general collision-free planner, multi-arm space-time reservations/handoff, calibrated physical force-loop integration, and estimator-driven hardware commands |
| Gearbox article | Ideal nominal 0.10-module 12/18/24-tooth train, three shafts, housing and cover; the reported forward/reverse ratio and backlash acceptance is calculated analytically after the task loop | A modeled rotary-tool/vision measurement in the task loop, optional metrology-driven perturbation sweeps, external 2PP feasibility/cleaning/metrology and measured friction/wear/stiction data |
| Interfaces | One headless compiled-baseline gearbox run; standalone native/WASM M1b point-motion and M1c simple-manipulation runtimes; native/WASM M1d optical co-design report; native M1e scenario/fault CLI plus a full-cycle WASM `ObservedManipulationSimulator` using the same structured decision report and controller hash; fixed-step Cartesian and phase-boundary replay; strict CLI manifest/scenario gates; and independently versioned static `SceneDescription` plus dynamic truth/estimate/commanded `SceneFrame`; offline recorded-state inspector (the legacy website and synthetic preview remain removed) | General direct machine-command controls, file-backed manifest loading in WASM, M1e stepwise scene visualization and browser golden comparison, batch/robustness tools, compact binary replay/telemetry, and estimator population in the general scene contract |

## M1f fixed-head extension

The capability table above describes the M1e baseline and legacy gearbox. Explicit
schema-v2 scenarios now select the [M1f extension](FIXED_HEAD_MANIPULATION_M1F.md):
fixed cell-frame macro geometry; shared optical and mechanical head/mount envelopes;
physical target rails with observed/surveyed-datum swept guards; bounded TCP position
and directed-axis IK through the authoritative tendon runtime; observed angular
corrections; and segmented, reobserved transfer and retraction. A versioned 44-case
matrix reports completion, controlled refusal, exact failure reasons, and terminal
physical scoring. It does not establish a continuous guaranteed envelope or yield.

This closes the ROI-retile and absent angular-actuation gaps for the isolated coupon.
General orientation planning, world-frame roll control, multi-arm trajectories,
image-derived detections, hardware timing, calibrated contact and breakable grasps
remain open. The new camera field and burst timing are a separate modeled candidate;
M1d precision values are not inherited. M1f accepts scenario/report schema 2 through
the same native and WASM runtime, while M1e remains schema 1 and the default.

## Engineering replay and stationary M1g handoff gate

An opt-in recorder now exports exact M1e/M1f geometry and command samples with the
unchanged controller report. The offline inspector displays three orthographic
projections, separates truth/estimates/commands, gates stale or invalid estimates,
and rejects broken replay mappings. General `SceneFrame.estimate` remains empty;
5-DoF estimates retain their own report schema rather than becoming full poses.

The isolated stationary handoff coupon now executes receiver closure, an atomic
single-owner exchange, donor opening and fresh receiver-retention checks. It
preserves the current owner on observation loss and tests physical transaction
rejection without donor release. Its arms start prepositioned; sensing is injected
at the labelled measurement-packet boundary. It is F0 protocol evidence, not a
fixed-head two-arm optical or trajectory result. See
[the replay and handoff contract](ENGINEERING_REPLAY_AND_HANDOFF_M1G.md).

## Calibrated cylindrical metrology

The [optical metrology subsystem](OPTICAL_METROLOGY.md) adds a parameterized
100 mm ID / 25 mm-zone candidate, six-view distorted-pixel triangulation,
six-DOF rigid distal fiducials and TCP covariance, timed hybrid Gray/phase
decoding, reduced defocus/diffraction, explicit calibration/thermal/reference
health, separate occupancy and surface APIs, independent geometry verification
and a stopped acquisition/precision-admission adapter over the existing machine.
It does not replace M1e/M1f sensing or qualify the legacy CAD. The 160 mm runtime
scale is explicitly distinguished from the standalone candidate. Incomplete
structured-light coverage and repeatability above target are reported as failures,
even where accepted points have micrometre-scale error. Physical accuracy,
intrinsic/extrinsic calibration fitting, full optical image formation and complete
cell occlusion geometry remain unvalidated or outside this implementation.

## Fidelity labels

- **F0 geometry:** kinematics, reach, conservative collision envelopes and visibility.
- **F1-reduced integration scaffold (current):** deterministic reduced tendon/contact/optics
  models and analytic gear constraints. The legacy gearbox still uses ray-gated synthetic pose
  observations and calculates final ratio/backlash after the task loop. The isolated M1e coupon
  instead closes a timestamped explicit-feature -> 5-DoF estimate -> correction/contact -> new
  observation loop without controller truth access. Neither executable meets the normative F1
  gearbox or hardware-correlation gates in `REQUIREMENTS.md`.
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
machine safety, physical yield, achieved micrometre precision, calibrated insertion force, or
feasibility of the multi-arm motion. In particular, the modeled M1d values of approximately
3.0 µm lateral RMS and 3.4 µm depth RMS and the approximately 7.9 µm guarded-insertion
closed-loop residual allocation are hypotheses until the M1e hardware coupon replaces the
scenario distributions with held-out measurements.

## Transparent-part observability experiment

The [gear/shaft study](GEAR_OBSERVABILITY.md) adds marker-inferred feature geometry
with independent relation uncertainty and provenance, 16 camera/projector/marker
candidates, static articulated visibility/contact checks and an actual bounded
command probe. No candidate passes the precision contract. The first approach is
collision-rejected; sampled optical endpoints are not an executed assembly.
Transparent surface reconstruction, physical marker supports, distal clearance
and contact insertion remain unqualified.
