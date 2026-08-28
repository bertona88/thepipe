# Pipe Micro-Assembly Simulator — Engineering Requirements

Status: normative target design baseline, revision 0.1
Scope: hardware-first simulator and a buildable low-cost reference cell. A web UI is explicitly out of scope for this phase.

This document specifies the evidence required for qualified fidelity tiers; it is not a list of
features already implemented. The executable subset and its exclusions are recorded in
`IMPLEMENTATION_STATUS.md`. The current end-to-end result is labeled **F1-reduced** and does not
yet satisfy the normative F1 requirements below.

## 1. Purpose and success claim

The simulator shall answer a concrete question: **can inexpensive, tendon-driven manipulators mounted inside a cylindrical cell assemble a two-photon-polymerization-scale spur gearbox when precision comes primarily from continuous optical observation?**

The target reference result is not a photorealistic animation. It shall be a reproducible engineering run containing:

- manufacturable CAD and mass/collision properties;
- actuator, tendon, contact, sensor, calibration, estimation, and controller models;
- a collision-free, observable assembly trace;
- tolerance and uncertainty sweeps;
- a functional gearbox test after assembly; and
- explicit reasons for every failed run.

The default simulation shall run headlessly. Native Rust is the reference executable; the same core shall compile to WebAssembly for a later browser presentation.

## 2. Assumptions and honest limits

### 2.1 Reference-build assumptions

| Item | Baseline assumption |
|---|---|
| Gearbox fabrication | Polymer housing, cover, and gears are eventually supplied by an external two-photon-polymerization (2PP) process; 0.35 mm precision wire shafts are the baseline hybrid stock parts |
| Gearbox geometry in simulation | Target canonical acceptance uses ideal nominal B-rep-derived solids; optional robustness runs use explicit metrology-derived surface/dimension perturbations, never a simulated 2PP process. The current F1-reduced run uses compiled nominal envelopes and does not ingest the B-reps. |
| Small cell stock parts | 0.8–1.5 mm arm joint pins, miniature bearings only where necessary, M2/M3 frame hardware |
| Structure | Clear acrylic or polycarbonate tube plus printed carriers and external motor deck |
| Compute | User-supplied laptop/desktop; no GPU is required for acceptance tests |
| Environment | 20–25 °C, diffuse enclosure lighting, vibration-isolated tabletop |
| Assembly material | Dry, clean rigid 2PP polymer parts plus steel shafts; no uncured polymer, debris, or electrostatic-control model in the baseline |
| Human intervention | Loading trays and replacing tools is allowed before a run; no intervention is allowed during the acceptance assembly |

2PP voxel exposure, scan strategy, support generation, polymerization, shrinkage, development, and cure are outside simulator scope. Fabrication feasibility and dimensional capability shall be qualified separately with the selected 2PP supplier/process. Normative F1/F2 shall consume nominal CAD plus an optional inspection/metrology bundle for delivered parts; the current F1-reduced run accepts ideal compiled geometry only.

### 2.2 Limits on predictive accuracy

Rigid-body simulation can reject impossible geometry and tune clearances, but it cannot by itself predict 2PP polymer wear, microscopic residue, stiction, electrostatic adhesion, or tendon creep. Until correlated hardware data exist, the simulator shall report contact-force and success-probability results as estimates with parameter ranges, not as guaranteed physical performance.

The target baseline optical model shall predict visibility, geometric reconstruction error, quantization, distortion, exposure noise, and occlusion. It does not claim wave-optics accuracy, sub-pixel surface metrology, 2PP-polymer subsurface scattering, or a complete camera ISP model. The current F1-reduced integration exercises only a smaller geometric subset: rays through sphere proxies gate a synthetic latent-pose observation, while nominal feature size supplies an orientation-uncertainty lever arm. It does not estimate 6D pose from images or ray-return features.

## 3. Coordinate system and units

- The cell frame `C` shall be right-handed: `+Z` along the tube axis; `+X` and `+Y` span the radial plane.
- Cylindrical base coordinates shall use `theta = atan2(Y, X)` and axial coordinate `z`.
- CAD interchange and human-readable dimensions shall use millimetres, grams, seconds, degrees, and newtons.
- The solver shall convert at one boundary to SI units: metres, kilograms, seconds, radians, and newtons.
- Every pose shall name its parent and child frame. Unlabelled 4x4 transforms are forbidden in persistent files.
- Quaternions shall be normalized and stored as `[x, y, z, w]`; angles shall not be persisted as Euler triples unless intended for display.

## 4. Reference cell geometry

### 4.1 Tube and qualified volume

| Parameter | Required baseline | Permitted design range |
|---|---:|---:|
| Clear tube inside diameter | 160.0 ± 0.5 mm | 140–220 mm |
| Tube usable length | 320 mm | 250–500 mm |
| Tube wall | 3.0 ± 0.5 mm | 2–5 mm |
| Qualified work volume (QWV) | cylinder, 80 mm diameter × 160 mm long | no smaller |
| QWV axial extent | `-80 <= z <= +80 mm` | — |
| Micro-assembly zone (MAZ) | 12 × 12 × 8 mm cuboid around the registered fixture datum, nominally centred at `(0, 0, 0)` | must remain inside QWV |
| Keep-out from tube wall | 8 mm, excluding bases and routed tendons | no smaller |
| Service aperture | one opening at least 80 × 100 mm or a removable half-shell | — |

Coarse tracking/manipulation performance applies inside the QWV. The 10–20 um final-alignment claims apply only inside the macro-imaged MAZ. Parts may be stored outside the QWV, but the simulator shall mark sensing and manipulation claims outside it as unqualified.

### 4.2 Mobile bases

The reference simulation shall contain four independently commanded mobile arm bases. Geometry shall leave room for six. Each base has:

- axial travel of at least 220 mm;
- angular travel of at least 300° with an explicit cable/service-loop keep-out sector;
- commanded base speed limited to 30 mm/s axially and 30°/s angularly;
- base acceleration limited to 100 mm/s² and 120°/s²;
- measured closed-loop base error no greater than 0.15 mm RMS axially and 0.15° RMS angularly under nominal load;
- a passive brake or non-backdrivable hold mode so loss of power does not drop a tool into the QWV; and
- a collision body including carriage, cable bundle, strain relief, fasteners, and not merely the cosmetic shell.

The low-cost physical reference is a printed, three-roller preloaded inner-wall carriage driven in `z` and `theta` by antagonistic capstan tendons from motors outside the work volume. A rigid rail or belt carriage is a valid substitute if it meets the same envelopes and error model. The simulator shall not assume a mechanically perfect cylindrical joint.

## 5. Tendon-driven arm module

### 5.1 Kinematics

Each of the four baseline arms shall provide six commanded pose degrees of freedom before gripper actuation:

1. mobile-base axial translation;
2. mobile-base rotation about the tube axis;
3. shoulder pitch;
4. shoulder yaw;
5. elbow pitch; and
6. wrist roll.

The first implementation may add wrist pitch as a seventh pose axis, but the gearbox test shall not depend on it. The shoulder datum shall lie at tube radius 72 mm or less. A valid baseline arm has 32 mm upper link, 30 mm forearm, 15 mm wrist/tool offset, and at least 75 mm usable radial TCP reach from its shoulder datum; this is enough to cross the axis of the 160 mm-ID tube with margin. Joint ranges shall be no smaller than:

| Joint | Range | Loaded speed limit | Repeatability target after visual correction |
|---|---:|---:|---:|
| Shoulder pitch | -70° to +95° | 90°/s | 0.20° RMS |
| Shoulder yaw | -55° to +55° | 90°/s | 0.20° RMS |
| Elbow pitch | 0° to 135° | 120°/s | 0.25° RMS |
| Wrist roll | ±170° | 180°/s | 0.35° RMS |

At least two arms shall be able to reach every point in the QWV, and at least three arms shall be able to view or illuminate every point without entering a hard collision. The simulator shall generate a sampled reachability/observability map at 2 mm spatial resolution.

### 5.2 Construction and tendons

The physical-reference links shall be printable resin shells with 0.8–1.5 mm steel pins, hard stops, and replaceable low-friction tendon guides. The baseline tendon model and CAD shall use:

- 0.15–0.25 mm braided UHMWPE line;
- four differential tendon loops, one each for shoulder yaw, shoulder pitch, elbow pitch, and
  wrist roll, with 2–5 N total pretension per loop;
- 4–8 mm capstan diameter;
- remote stepper, smart-servo, or geared-DC actuation outside the QWV;
- a tendon working load no greater than 25% of its measured break load; and
- replaceable PTFE guide liners where a routing bend is tighter than 10 mm radius.

The simulation shall model tendon stretch, joint stiffness, capstan radius, spool quantization, backlash/deadband, Coulomb friction, pretension, and at least one hysteresis term. Nominal parameters and uncertainty bounds shall be stored in a versioned calibration file, not embedded in controller code.

### 5.3 Manipulation performance

With a calibrated cell and a part of 2 g or less:

- coarse tool-centre-point (TCP) error throughout the QWV shall be no greater than 0.20 mm RMS;
- final macro-visual-servo alignment inside the MAZ shall be no greater than 0.010 mm RMS in the image-resolved lateral axes and 0.020 mm RMS in depth;
- TCP speed during free motion shall be limited to 40 mm/s;
- TCP speed within 2 mm of a part shall be limited to 3 mm/s;
- normal contact force during gear placement shall be limited to 0.005 N;
- shaft insertion force shall be limited to 0.050 N; and
- unintended contact above 0.010 N or penetration above 0.005 mm shall be a hard failure.

The arm shall carry at least 5 g at 60 mm extension and exert 0.050 N for 2 s along a tool axis without exceeding tendon or printed-link limits. Higher-force operations require a separately qualified braced tool path rather than silently increasing tendon capacity.

## 6. End effectors and fixtures

The tool interface shall have a repeatable kinematic coupling or keyed bayonet with a simulated and measured tool transform. Tool-change repeatability target is 50 um translational and 0.3° rotational, 3-sigma.

The minimum tool set is:

| Tool | Required working range | Acceptance role |
|---|---|---|
| Compliant parallel jaw | 0.10–3.0 mm opening; 0.002–0.10 N commanded grip | shaft handling, cover handling, handoff |
| Vacuum micro-pick | 0.15–0.30 mm pulled-capillary/soft tip; -20 to -60 kPa sensed vacuum; leak detection | flat gear pickup and placement |
| Axial insertion probe | 0–0.10 N optically read compliant flexure; <= 0.5 mN resolution; 0.10 mm travel | seating shafts and cover |
| Rotary drive blade | 0.080 mm nominal blade; 0–0.050 mN·m optically read torsion flexure; <= 0.002 mN·m resolution | pre-cover mesh check and post-cover ratio/torque test |
| Calibration pointer | conical metal tip with characterised 5–10 um apex | TCP and macro-camera calibration |

Jaw pads shall be replaceable TPU/silicone parts and shall have collision geometry in both open and closed states. Vacuum pickup shall fail when effective seal area, pressure, acceleration, or part porosity produces less than a 3x holding-force margin.

The gearbox nest shall be a passive cell-scale printed fixture with three datum contacts, a compliant clamp, and an inset micro-nest sized to the 6 × 4 mm housing. It shall constrain the housing without using gear-critical internal features as datums. The fixture and parts tray shall carry machine-readable global fiducials plus 2PP-scale local alignment marks that remain visible to a macro camera during the acceptance run.

## 7. Optical sensing and calibration

### 7.1 Sensor layout

The baseline cell shall simulate and provide mounts for:

- six monochrome global-shutter camera modules, 1280 × 800 or greater, 30 fps minimum;
- three cameras near each tube end, with azimuths separated by roughly 120° and triangulation baselines of 60–130 mm across the QWV;
- one structured-light source (DLP/pico-projector or scanned line) with switchable coded and dark frames;
- diffuse white/near-IR illumination with independently controlled banks; and
- at least two tool-local macro cameras shared among the four arms or mounted on designated observer arms; the acceptance move shall have a simultaneous stereo pair or one macro camera plus a calibrated projector view, 4 × 3 mm or smaller field of view, 10–25 mm working distance, and 6–20 mm effective triangulation baseline.

The locked reference packaging uses two clocked three-camera triplets at a 60 mm front-face
radius and `z = -106/+106 mm` from the work datum, plus a rigid 12 mm stereo macro head on one
wrist. The current F1-reduced runtime uses those numbers in an active-component-local sensing
frame rather than executing the CAD wrist or fixed tube-world camera transforms.

Rolling-shutter cameras may be simulated as a cost-down option but shall not satisfy the dynamic acceptance test unless their row timing is included in estimation. Transparent tube reflections shall be present in the high-fidelity optical scene or represented by a measured false-detection/contrast-loss model.

### 7.2 Measurement model

The optical simulator shall model, at minimum:

- calibrated pinhole intrinsics;
- Brown–Conrady radial and tangential lens distortion;
- finite pixel sampling and a configurable point-spread blur;
- shot noise, read noise, black level, saturation, and exposure;
- camera timestamp offset and jitter;
- occlusion by complete robot/tool/cable collision geometry;
- projector intrinsics/extrinsics and code correspondence errors; and
- false negative/outlier observations, never only zero-mean Gaussian pose noise.

The default geometric targets inside the QWV are:

| Metric | Nominal | Required worst case for acceptance |
|---|---:|---:|
| Fiducial centre reprojection RMS | <= 0.15 px | <= 0.30 px |
| Static triangulated point error | <= 25 um RMS lateral, <= 50 um RMS depth | <= 50 um lateral, <= 100 um depth |
| Tracked rigid-body pose update | 60 Hz | >= 30 Hz |
| Sensor-to-estimate latency | 25 ms | <= 60 ms |
| Time synchronization residual | 0.25 ms RMS | <= 1 ms RMS |
| Macro-camera object-space sampling | <= 2.0 um/px | <= 3.0 um/px |
| Macro stereo/structured depth error | <= 8 um RMS | <= 15 um RMS |

These are design targets to be verified on a calibration artefact; they are not inferred from camera resolution alone.

### 7.3 Calibration and observability

The system shall support a fully scripted calibration with:

1. per-camera intrinsic calibration using a printed/glass target;
2. joint camera/projector bundle adjustment;
3. tube-frame registration from at least six surveyed wall fiducials;
4. robot kinematic/tendon parameter identification from a calibration pointer observed at no fewer than 40 configurations per arm;
5. tool-centre-point calibration for every installed end effector; and
6. validation against a separate gauge object not used in fitting.

No controller or planner may read ground-truth part or robot poses during an acceptance run. Ground truth is available only to the simulator, metrics recorder, and optional debug visualizer. Loss of two-view observability for more than 100 ms during near-contact motion shall cause a controlled pause or retreat.

## 8. Physics, collision, and part-variation fidelity

### 8.1 Required contact behaviour

The default simulator shall provide:

- broad-phase and narrow-phase collision detection for every moving body;
- continuous collision detection for the TCP, jaws, held part, and nearby fixture during insertion;
- Coulomb friction with separately configurable static/dynamic coefficients;
- restitution, contact compliance, damping, and solver iteration controls;
- rigid-body mass and inertia calculated from CAD volume plus inserted hardware;
- grasp constraints that can break based on normal force, friction cone, vacuum margin, or impulse;
- joint/tendon limits and hard-stop contact; and
- collision layers for robot/robot, robot/cell, robot/part, part/part, and allowed grasp contacts.

Static visual meshes shall not be silently reused as all-purpose dynamic colliders. Dynamic links require convex or compound-convex colliders. Gear teeth require tooth-resolved collision only for the final functional test at the highest fidelity tier; lower tiers may use an analytic gear constraint after verified placement.

### 8.2 Part-realization distributions

The canonical run uses ideal nominal gearbox solids. Robustness mode shall support deterministic perturbations representing an inspected delivered-part population, including at least:

- generic local normal surface offset: zero nominal, user-supplied distribution, default bounded at ±5 um for sensitivity analysis only;
- gear-bore diameter deviation: zero nominal, default bounded at ±3 um;
- housing shaft-centre location deviation: zero nominal, default bounded at ±3 um per axis;
- housing/cover flatness deviation: zero nominal, default bounded at ±3 um;
- shaft diameter: 0.350 mm nominal, ±2 um;
- shaft straightness: 3 um over 1.55 mm;
- shaft-position error after seating: ±10 um XY and ±0.25° tilt;
- coefficient of friction ranges for 2PP-polymer/polymer and 2PP-polymer/steel; and
- camera, base, tool, and tendon calibration residuals.

These defaults are engineering sensitivity bounds, not claims about 2PP process capability. Distributions shall be replaceable with actual inspection maps or measured histograms. No voxel, laser-path, polymerization, shrinkage, support, or development simulation belongs in this layer. Every run shall log the metrology-bundle ID, seed, and sampled values.

## 9. Gearbox acceptance article

### 9.1 Geometry

The acceptance article is a single-stage, three-gear spur train in an approximately 6 × 4 × 2 mm covered housing. Its polymer parts target external 2PP fabrication. The assembly simulator receives them as cleaned, separated, nominal solids unless an explicit metrology bundle is selected.

| Part | Required nominal geometry |
|---|---|
| Housing | 6.00 × 4.00 × 1.60 mm body/tray; 0.25 mm floor; three 0.340 mm split-compliant shaft seats × 0.250 mm deep; external three-point datum; two cover latches |
| Cover | 6.00 × 4.00 × 0.20 mm; shaft-capture reliefs; 0.75 mm input-driver window; 1.0 mm output-observation window; two compliant latches; assembled envelope <= 6.05 × 4.05 × 1.85 mm |
| Input gear G1 | 12 teeth, module 0.10, 25° pressure angle, 1.20 mm pitch dia., 1.40 mm outside dia., 0.35 mm tooth face; 0.55 mm OD coaxial hub gives 1.30 mm total part height and carries a 0.10 mm drive slot |
| Idler gear G2 | 18 teeth, module 0.10, 25° pressure angle, 1.80 mm pitch dia., 2.00 mm outside dia., 0.35 mm tooth face; 0.55 mm OD coaxial hub gives 1.30 mm total part height |
| Output gear G3 | 24 teeth, module 0.10, 25° pressure angle, 2.40 mm pitch dia., 2.60 mm outside dia., 0.35 mm tooth face; 0.55 mm OD coaxial hub gives 1.30 mm total part height and carries unequal optical phase marks visible through cover |
| Gear bores | 0.420 mm nominal, 0.025 mm entry chamfer × 45° both sides |
| Shafts S1–S3 | 0.350 mm diameter × 1.55 mm long precision steel wire/dowel; <= 5 um end chamfer/deburr envelope |
| Shaft centres | G1 `(0.750, 2.000) mm`; G2 `(2.250, 2.000) mm`; G3 `(4.350, 2.000 mm)` in housing XY |
| Centre distances | G1–G2 1.500 mm; G2–G3 2.100 mm; G1–G3 outside-circle clearance 1.60 mm |
| Tooth thickness/backlash | 0.1471 mm nominal tooth thickness at pitch circle (0.0100 mm thinning per gear); 0.020 mm nominal circumferential backlash per mesh |
| Gear axial clearance | 0.04–0.08 mm at each gear's upper hub after cover closes |

The nominal input-to-output reduction ratio is `24/12 = 2.0000:1`, so the output/input speed
ratio is `0.5`; with two meshes, input and output rotate in the same direction. The 25° pressure
angle is intentional: it avoids the full-depth involute undercut expected for a 12-tooth, 20°
pinion while retaining nominal centre distances. CAD generation shall use converged involute
profile sampling and verify transverse contact ratio greater than 1.20 for both meshes. A
separate 2PP qualification design may contain tooth, bore, clearance, and latch arrays, but that
process-qualification artefact is not part of the assembly simulation.

### 9.2 Deterministic initial state

The canonical acceptance run is named `gearbox_baseline_v1`, uses random seed `0x504950455F474258`, a fixed 1 ms physics step, and single-threaded deterministic event ordering. The initial state shall be loaded from versioned scene data:

- fixture registered in the cell frame with its housing datum at `(0, 0, 0)`;
- empty housing in the fixture with initial pose error sampled but bounded by ±1.0 mm and ±2°;
- shafts in three labelled tray grooves;
- gears flat in separate pockets with tooth index orientation deliberately unknown to the controller;
- cover in its tray pocket;
- all arm tools clear of the QWV and all cameras at nominal calibration plus the seeded residuals; and
- no hidden truth pose exposed to planning, estimation, or control.

### 9.3 Required assembly sequence

The following sequence is deterministic at the task level. Motion paths may be replanned, but action order and pass/fail gates may not change.

| Step | Required action | Gate to proceed |
|---:|---|---|
| 0 | Home bases/arms, acquire fiducials, estimate housing and trays | all cameras valid; housing pose covariance <= `(0.020 mm)^2` lateral and `(0.10°)^2` angular |
| 1 | Observer arm inspects empty housing and shaft seats with macro structured light | no obstruction > 0.010 mm; all three seats localized <= 0.005 mm lateral |
| 2 | Jaw arm picks S1, verifies grasp, inserts it into input seat | insertion depth `0.250 ± 0.015 mm`, tilt <= 0.5°, peak force 0.001–0.050 N |
| 3 | Repeat for S2 and S3 in that order | same limits; no housing displacement > 0.010 mm |
| 4 | Vacuum arm picks G3, checks vacuum margin, visually centres bore, and lowers it over S3 | gear seated, temporary free axial clearance >= 0.030 mm, no force > 0.005 N |
| 5 | Vacuum arm installs G2 on S2 while a second arm holds the housing datum | G2–G3 circumferential backlash 0.010–0.040 mm at pitch circle; gears turn 20° without bind |
| 6 | Install G1 on S1; observer performs tooth/mesh inspection | both meshes engaged; all gears within 0.020 mm of intended Z |
| 7 | Probe arm engages G1's hub slot and rotates it through one full turn | G3 turns `180.0° ± 3.0°` in same direction; peak input torque <= 0.020 mN·m; no skip |
| 8 | Two arms perform a deliberate cover handoff, align bearing holes, and lower cover | handoff does not lose observability; pin-to-hole lateral error <= 0.010 mm before descent |
| 9 | Insertion tool closes the two latch features sequentially while the other arm stabilizes the nest | each latch emits force/displacement signature; cover gap <= 0.030 mm at four inspection points |
| 10 | Through the cover windows, drive G1's hub slot ten turns at 30 rpm equivalent, then reverse two turns while macro vision tracks G3's phase marks | output `5.000 ± 0.080` turns forward then `1.000 ± 0.040` turns reverse; no jam, loss, or collision |
| 11 | Release fixture, image all sides possible in situ, and emit the run report | all metrics, estimates, truth-only errors, contacts, sampled tolerances, and artifacts present |

For Step 7, one input revolution ideally produces `12/24 = 0.5` output revolution, or 180°. For Step 10, ten input revolutions ideally produce five output revolutions.

Steps 7 and 10 are normative target actions. The current F1-reduced task loop uses a
command-driven verification surrogate; after the task completes, the report separately applies a
stateful analytic ratio/backlash calculation gated by insertion, mesh and latch observations. It
does not model the rotary blade engaging G1 or macro vision measuring G3 rotation.

### 9.4 Baseline pass criteria

A run passes only if all step gates pass and:

- no non-permitted collision exceeds 0.005 mm penetration or 0.010 N estimated normal force;
- no part leaves the designated capture volume;
- every near-contact action is based on sensor-derived state with valid covariance;
- aggregate assembly time is <= 12 minutes of simulated wall time;
- the final cover is retained under a 0.020 N axial proof load;
- the final gear train completes the forward/reverse functional test;
- estimated poses and forces are recorded separately from truth values; and
- replaying the same binary, assets, configuration, and seed produces the same task events and pass/fail result.

Floating-point traces need not be bit-identical between native CPU architectures and WebAssembly, but metrics shall remain within declared numerical tolerances and discrete task events shall match.

### 9.5 Robustness suite

After the canonical run passes, 100 seeded tolerance runs shall satisfy:

- at least 95 complete assemblies without human intervention;
- zero high-energy robot/cell collisions;
- no more than five controlled aborts;
- no uncontrolled part ejection; and
- a Wilson 95% lower confidence bound reported alongside observed success rate.

This is a simulation robustness target, not a physical yield claim. Hardware success shall be reported separately after at least 30 physical trials.

## 10. Fidelity tiers

| Tier | Intended decision | Required models | Explicitly not claimed |
|---|---|---|---|
| F0 — geometry | Reach, gross collision, cell layout | exact kinematics; simplified convex collision; ideal grasp; perfect pose | forces, sensor error, physical success |
| F1 — engineering default | Controller and assembly-sequence feasibility | rigid dynamics/contact; tendon compliance/deadband; noisy timestamped cameras; occlusion; state estimation; grasp failure; optional metrology-derived part perturbations; analytic gear constraint | 2PP process outcome, micro-wear, polymer deformation, absolute yield |
| F2 — contact/optical validation | Clearance, insertion, tooth mesh, difficult visibility | tooth-resolved collision; compliant contacts; calibrated friction ranges; capstan/Bowden friction; full distortion/exposure/projector model; flexible jaw/fixture surrogate | definitive lifetime or fatigue |
| F3 — hardware-correlated | Predict measured prototype outcomes | F2 plus identified parameters and residual/error models from physical logs; hardware-in-loop option | extrapolation outside measured envelope |

`F1-reduced` is an implementation label for the current scaffold, not an additional qualified
tier. The gearbox acceptance report shall always state its fidelity. F0 and F1-reduced are never
sufficient for a “successful physical assembly” or normative F1 claim. Normative F1 is the
minimum software milestone; F2 is required before freezing nominal gearbox clearances for
external 2PP qualification.

## 11. Failure modes and required responses

| Failure mode | Detection | Simulator behaviour | Controller response |
|---|---|---|---|
| Tendon slack/break | tension below threshold; encoder/model mismatch | change stiffness or disconnect tendon | stop affected arm, secure/retract others |
| Tendon hysteresis/creep | persistent direction-dependent residual | bias/deadband evolves with load/time | re-estimate, approach final pose from known direction |
| Base carriage slip | base fiducial disagrees with command | friction-limited base motion | pause, relocalize, reduce acceleration |
| Lost/occluded fiducial | no inlier observations or covariance growth | remove observation, retain dynamics | pause near contact; move observer/illuminator |
| Camera/projector desync | timestamp residual/reprojection spike | shift timestamps/drop frames | disable structured frame, fall back or retreat |
| Transparent-wall glare | contrast/outlier rate | inject false negatives/outliers | switch illumination bank/exposure |
| Jaw or vacuum mis-pick | force/vacuum/part-pose check fails | grasp constraint absent or weak | return tool, reacquire, retry once |
| Two parts lifted together | observed mass/outline mismatch | coupled or second loose body | place back, separate, reacquire |
| Shaft jams or buckles | force rises before insertion depth | compliant contact and shaft tilt | stop below 0.050 N, withdraw, inspect, retry once |
| Gear bridges shaft | axial force/height mismatch | edge contact at bore chamfer | spiral search <= 0.030 mm radius, then abort |
| Tooth-on-tooth placement | gear height high, no small rotation | tooth contact | lift 0.020 mm and phase gear <= half tooth pitch |
| Part adheres to tray/tool | commanded release but part pose/force does not change | configurable breakaway adhesion surrogate | peel/roll release at low acceleration, then retry once |
| Gear binds | torque high or insufficient output rotation | friction/contact resists motion | identify mesh, reopen before cover |
| Dropped part | grasp break and ballistic/contact motion | full free body | freeze unrelated arms, track, recover only if visible |
| Arm-arm collision risk | predicted separation below 0.5 mm | continuous collision forecast | time-parameterize/retreat lower-priority arm |
| Calibration drift | check fiducial residual > threshold | extrinsic bias evolves | pause and recalibrate |
| Solver instability | energy/penetration/nonnumeric guard | mark invalid run | fail closed; never report assembly success |

Each listed mode shall have at least one deterministic fault-injection test.

## 12. BOM classes and cost ceiling

Costs are budget classes, not vendor quotes, and exclude the external 2PP gearbox run, computer, taxes, and hand tools. The low-cost target applies to the reusable assembly cell; it does not imply that 2PP fabrication is inexpensive.

| Class | Examples | Four-arm target cost (USD) | Simulator data required |
|---|---|---:|---|
| Cell/frame | clear tube, end plates, printed carriers, bearings/rollers, fasteners | $80–160 | geometry, material, mass, tolerances |
| Base motion | capstans/belts, steppers or geared motors, drivers, encoders/markers | $100–220 | torque/speed curve, backlash, friction |
| Arms/tendons | printed links, pins, UHMWPE, liners, springs, remote actuators | $180–360 | routing, stiffness, pretension, limits |
| Tools | jaw, vacuum tips, pump/valves, insertion spring/load sensing | $60–150 | grasp/contact/force calibration |
| Global vision | six OV9281-class mono global-shutter modules, lenses, mounts, sync electronics | $160–320 | intrinsics, timing, noise, spectral response surrogate |
| Local vision/light | two macro modules, LEDs, filters, coded projector/line source | $80–220 | intrinsics/extrinsics, photometric settings |
| Control/electrical | microcontroller, USB hub, power supplies, wiring, E-stop | $80–180 | latency, rates, current/thermal limits |
| Calibration/consumables | target, gauge artefact, spare capillary tips/tendon | $40–100 | measured compensation and uncertainty |

The budget-selected four-arm reference build target is **$780–1,200**, excluding external 2PP parts and host computer. The independent upper bounds in the table describe stretch substitutions and sum to roughly $1,700; they are not the low-cost configuration. A minimum three-arm proof-of-concept may omit two global cameras, one local camera, and some independent base hardware, but its reduced observability shall be visible in the simulator and it does not satisfy the full acceptance requirement.

## 13. Software and evidence requirements

The software shall:

- compile the core for native Rust and `wasm32-unknown-unknown` without separate physics logic;
- run headlessly from a versioned scenario file;
- use a fixed simulation step and seeded, named pseudo-random streams;
- validate CAD/scene schema versions and content hashes before a run;
- log commands, observations, estimates, truth, contacts, force limits, task events, and random samples;
- emit machine-readable JSON plus a concise human-readable report;
- export a replay independent of a live UI;
- contain unit, property, integration, determinism, and fault-injection tests; and
- refuse to label a run successful if required evidence is missing or the numeric solver reports invalid state.

## 14. Phase gates

1. **CAD gate:** all cell, arm, tools, fixture, and nominal gearbox parts regenerate from parameters; assemblies have no impossible interferences; manufacturing exchange files and collision proxies are exported.
2. **F0 gate:** reachability and multi-arm collision analysis shows the full deterministic task has a geometric solution.
3. **F1 sensing gate:** calibration and estimator meet QWV error/latency requirements without truth leakage.
4. **F1 assembly gate:** `gearbox_baseline_v1` passes natively and in WebAssembly with matching discrete events.
5. **F2 gate:** tooth-resolved functional test and tolerance sweep pass; critical results receive convergence checks.
6. **2PP qualification gate (external to simulation):** the selected process/supplier demonstrates tooth, bore, clearance, and latch capability; delivered parts are inspected and their metrology bundle is available to optional perturbation runs.
7. **Correlation gate:** measured prototype logs update F3 parameters; software and hardware yields are reported separately.

No frontend work is required to satisfy these gates.
