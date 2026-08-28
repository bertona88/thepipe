# What is needed first

The website and the 2PP printing process are intentionally out of scope. Gearbox parts are ideal
nominal solids. The first hardware milestone is **one measured arm, one measured mobile base, one
rigid macro metrology head, and real tools on a clamped fixture**. Do not buy or build four copies
until that cell proves its force, travel, optical, and repeatability budgets.

The current software is an F1-reduced integration scaffold. It is useful for locking interfaces,
exercising the guarded recipe, and finding contradictions. It is not yet the normative F1
hardware-feasibility simulator because arms do not carry the simulated parts, force and torque are
uncalibrated surrogates, and the acceptance optics do not estimate 6D pose from CAD imagery.

## 1. Freeze and measure one arm/base channel

Reference geometry already represented in CAD and Rust:

- 160 mm tube ID and 320 mm usable length;
- shoulder datum at 72 mm radius;
- 32 + 30 + 15 mm serial links;
- four differential tendon joints per arm;
- 1.65 mm joint tendon moment arm, distinct from the 3.0 mm motor capstan radius;
- 12 mm total usable tendon payout, 1.2 N nominal pretension, 4 N tension limit; and
- 2.8 mm jaw opening with a 0.15 N hardware force ceiling.

The first physical channel needs a real Z/azimuth base mechanism, not just a CAD rail. Freeze:

- Z and azimuth motor/gear/belt choice, continuous and peak torque, speed, duty cycle, and brake;
- absolute or homed position sensing, end switches, hard stops, and a safe holding state;
- capstan groove and opposed winding, tensioner, fairleads, bend radii, and service-loop routing;
- motor encoder, current measurement, driver, supply voltage, thermal sensor, and fuse; and
- a calibration fixture that measures payout, tendon tension, joint angle, backlash, hysteresis,
  stiffness, repeatability, and temperature over the whole joint range.

Pass before replication: all commanded spans fit inside the 12 mm payout; no tendon exceeds 4 N;
the arm carries 5 g at 60 mm; it applies 50 mN for 2 s; and measured TCP repeatability and lost
motion are good enough for the macro camera to close the remaining error. The current serial-arm
model accepts tendon offsets directly and therefore does not prove motor travel or thermal limits.

## 2. Freeze one rigid macro metrology head

Do not use two independent compliant arms as an assumed micrometre stereo baseline. Put the two
macro cameras on one characterized rigid head, or continuously estimate their relative transform.
The reference optical contract is:

- six 1280 x 800 global views on two clocked triplets, front-face radius 60 mm, Z offsets
  -106/+106 mm, and 68 degree horizontal field of view;
- a rigid two-camera macro head with 12 mm baseline;
- 2048 x 1536 macro sampling over a nominal 4 x 3 mm field at 10–25 mm working distance;
- one coded projector or scanned-line source with an explicit camera/projector baseline;
- hardware trigger, common clock, exposure control, dark frames, and independently switched light;
- at least six surveyed cell fiducials plus rail, tool, nest, and tray fiducials; and
- a separate traceable gauge that is not used to fit the calibration.

Before hardware is frozen, select an actual camera, lens, projector/laser, wavelength, optical
filter, and controller bandwidth. Measure intrinsics, distortion, camera/projector extrinsics,
timestamp error, drift versus temperature, depth error versus range/incidence/material, dropout,
and false returns through the clear tube. The acceptance controller must pause or retreat when two
independent views cannot support its uncertainty gate.

The optics crate already ray-traces projector visibility, triangulation, quantization, photon/read
noise, and covariance. The end-to-end reference run currently uses those rays to gate a synthetic
active-feature pose sensor over sphere proxies; it is not yet a full image-to-CAD pose pipeline.

## 3. Build functional tools and a repeatable coupling

Every tool needs a keyed or kinematic coupling, a measured TCP transform, and collision geometry in
all operating states. Minimum first set:

| Tool | Physical target | Calibration evidence |
|---|---|---|
| Jaw | replaceable soft pads, 2.8 mm opening, <=0.10 N commanded | force versus tendon/current, slip and breakaway |
| Vacuum pick | 0.15–0.30 mm soft/capillary tip, -20 to -60 kPa | pressure sensor, leak test, >=3x holding margin |
| Axial probe | approximately 0.5–1.5 N/mm bending flexure, >=0.10 mm travel | optical displacement target, <=0.5 mN resolution |
| Rotary blade | 0.080 mm blade for the 0.100 mm G1 slot | torsion flexure, <=0.002 mN·m resolution, runout |
| Calibration pointer | characterized 5–10 um metal apex | independent microscope/gauge measurement |

CAD now contains buildable envelopes and corrected tip/blade/flexure dimensions, but tool changing,
vacuum plumbing, force readout, rotary drive, and runtime tool behavior still need implementation.

## 4. Clamp and instrument the workpiece

The fixture must be a real 3-2-1 restraint, not a loose pocket. Add a compliant side clamp, three
primary supports, two secondary contacts, one tertiary contact, and a measured release force. The
housing must not use gear-critical bores as datums. Add global fiducials and macro-scale local marks
to both nest and insertion-order tray. Measure fixture repeatability, clamp-induced housing strain,
and visibility in every required arm approach.

Ideal gearbox contract for the simulator:

- 6.00 x 4.00 mm housing, three 0.35 mm shafts;
- module 0.10, 25 degree, 12/18/24-tooth gears;
- 0.020 mm **per-mesh** circular backlash (0.010 mm tooth thinning per gear);
- 0.420 mm bores, 0.35 mm tooth face, 1.30 mm total gear height; and
- three shafts, G3, G2, G1, then the two-latch cover.

2PP supplier selection, exposure strategy, cleaning, shrinkage, and yield are later external gates.
They do not block software work with ideal parts.

## 5. Electrical and safety architecture

Before powered multi-axis testing, provide:

- a 24+ axis power/current budget for four arm joints, eight base axes, grippers, tools, and optics;
- hardwired E-stop/contactor, per-channel current and thermal protection, and safe holding brakes;
- homing and end-limit inputs independent of the high-level controller;
- watchdog behavior for camera, encoder, network, or controller loss;
- tendon whip guards and needle/tool guarding;
- laser/VCSEL classification, opaque beam stops, and an interlock appropriate to a clear tube; and
- low-speed commissioning modes with force, travel, and workspace limits.

The present cost tables are planning envelopes, not supplier quotes. Re-quote after the one-arm,
metrology-head, tool, base-drive, controller-bandwidth, and safety downselects; those choices dominate
cost more than the idealized gearbox parts.

## 6. Software gates to reach normative F1

The next implementation order is:

1. derive runtime bodies and sensor extrinsics from the versioned manifest rather than a compiled
   subset;
2. couple actuator states to tendon joints, IK, arm/TCP trajectories, held-part constraints, and
   breakable jaw/vacuum grasps;
3. put all robot, tool, cable keep-out, fixture, and part geometry in the collision/safety query;
4. reconstruct and fuse 3D features into timestamped 6D estimates without controller truth access;
5. calibrate insertion force, latch signatures, mesh torque, friction, and compliance from hardware;
6. execute the one-turn pre-cover and ten-forward/two-reverse post-cover rotary-tool tests inside
   the task loop; and
7. establish matching native/WASM discrete-event golden traces and measured fault tests.

Current reports include active-part pose error, ray/uncertainty summaries, analytic clearances,
force/torque surrogates, retries, failures, configuration hashes, and an explicit fidelity boundary.
They do not yet include arm-driven assembly trajectories, full covariance histories, or a calibrated
physical gearbox measurement.

## Gate for this phase

The F1-reduced nominal run must place three shafts, G3/G2/G1, and the cover through observation-driven
guards; remain inside the modeled part/fixture clearance and force gates; and pass an explicitly
labeled post-run analytic ratio/backlash check. Occlusion and bounded insertion-force faults must
trigger recovery; an injected collision must fail closed. CAD export must reproduce the canonical
names, dimensions, BREP validity, geometry hash, and per-mesh backlash contract.

That software pass is permission to build and calibrate the one-arm bench—not evidence that a
four-arm machine or 2PP gearbox is already hardware-qualified.
