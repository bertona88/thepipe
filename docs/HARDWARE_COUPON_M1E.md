# M1e hardware coupon and bench qualification

Status: bench protocol and data contract; no hardware-qualified result exists

This document defines the minimum physical bridge from the modeled M1d optical/robot candidate to
M1e observed-state single-arm manipulation. It is intended to qualify one measurement and manipulation channel,
not the full Pipe. The build is intentionally limited to:

- one tendon-driven arm and its real base/drive channel;
- one compliant parallel-jaw tool carrying an approximately 0.40 mm calibration peg or pointer;
- one characterized socket coupon on a rigid fixture;
- two temporary global views for coarse acquisition; and
- one rigid local camera/projector macro observer.

The result is a versioned set of measured distributions suitable for replacing M1d/M1e scenario
assumptions. It is not permission to claim micrometre arm accuracy, camera accuracy, production
yield, or four-arm feasibility. Six global cameras, replicated macro heads, a final projector SKU,
and production wrist packaging remain downstream decisions.

## 1. Qualification question and claim boundary

The coupon shall answer one question:

> Can a loaded inexpensive tendon arm repeatedly enter a local optical capture volume, stop long
> enough to be measured through the intended optical path, execute corrections as small as 5--50
> micrometres, grasp a 0.40 mm peg using observable contact evidence, and perform guarded insertion
> without using an unobserved pose or exceeding a declared force limit?

Three evidence classes must remain distinct in every record and report:

| Evidence class | Permitted use |
| --- | --- |
| Commanded | Controller request, encoder/motor state, trigger request, jaw request, or planned pose. It is not distal-tool position. |
| Observed/estimated | Timestamped image features, structured-light correspondences, jaw/contact measurements, fused state, covariance, residual, age, and validity. These may drive the controller. |
| Reference/evaluation only | Independent gauge, reference microscope/displacement probe, force calibration, surveyed geometry, or manually adjudicated outcome. These may score a run but shall not be available to the controller. |

Simulator truth has no physical counterpart on the coupon. Reference instruments shall therefore
be wired or logged on an evaluation-only path, and software shall demonstrate that removing their
stream does not change controller commands or state transitions. A camera calibration reprojection
error is not an independent accuracy result. Macro performance is accepted only on a separate
held-out gauge and on manipulation trials not used to fit the calibration.

The optical qualification is geometric and radiometric at the feature-detection level. It does not
qualify diffraction, coherent speckle, complete wave optics, transparent-polymer subsurface
scattering, or a full camera ISP. Contact results are reduced compliance/force-envelope data, not a
calibrated friction law or a general insertion-force prediction.

The default terminal response is **Stop and hold**. An automatic unload or retreat is permitted
only as an in-task recovery while fresh observed state, bounded force evidence, and a
collision-preflighted reverse corridor remain valid. Once the executive has declared a terminal
fault, it must not invent an axis or execute an unobserved retreat; a human-controlled safing
procedure then removes stored energy and recovers the coupon.

## 2. Frames, state, and units

All machine-readable values use SI units: metres, seconds, radians, kelvin, kilograms, newtons,
amperes, and volts. Millimetres and micrometres may be used in drawings and tables for readability,
but the raw-data field name must carry its SI suffix. Timestamps are signed or unsigned integer
nanoseconds on a named clock, never floating-point wall time.

Compound-unit suffixes shall spell out the operation. Translational stiffness is `_n_per_m`, while
torque is `_n_m`; `_n_m` shall never be used to mean newtons per metre. Likewise, a random-walk
process amplitude is `_m_per_sqrt_s`, not `_m`. The M1e scenario therefore names
`contact.lateral_stiffness_proxy_n_per_m`, `contact.axial_stiffness_proxy_n_per_m`,
`estimator.loaded_hold_process_sigma_m_per_sqrt_s`, and
`estimator.free_prediction_sigma_m_per_m` explicitly. Its repeatable loaded translation is
`motion.loaded_hold_error_world_m`, expressed in `C`. Every exported calibration bundle shall use
the same names and include `world_frame_id: C`; a numeric value without its unit/frame mapping is
invalid.

### 2.1 Named frames

| Frame | Definition |
| --- | --- |
| `C` | Cell/bench frame. Right-handed; `+Z_C` follows the Pipe tube axis and `+X_C,+Y_C` span the radial plane, matching `REQUIREMENTS.md`. |
| `H` | Rigid macro-head mechanical datum. It is surveyed relative to `C`; it is not silently equated to a camera frame. |
| `K` | Macro camera optical frame: `+x_K` image right, `+y_K` image down, `+z_K` forward along the optical axis. |
| `P` | Projector pinhole/entrance-pupil frame, using the same axis convention as `K`. |
| `G0`, `G1` | The two temporary global-camera optical frames. |
| `W` | Curved clear-wall coupon datum, fixed to three surveyed mounting contacts. |
| `F` | Fixture datum fixed by the coupon's 3-2-1 restraint. |
| `S` | Socket geometric-centre frame, matching the runtime estimate. `+z_S` points from entrance to seat; the 0.800 mm guide puts entrance at `z_S=-0.400 mm` and seat at `z_S=+0.400 mm`. |
| `T` | Tool centre frame. The origin is midway between the unloaded inner pad surfaces at the declared reference opening; `+x_T` is the jaw-closing axis and `+z_T` is the nominal peg/insertion axis. |
| `B` | Peg capsule-centre frame, matching the runtime estimate. `+z_B` points toward the leading tip; for the 0.700 mm half-segment and 0.200 mm cap radius, the tip is at `z_B=+0.900 mm` and the tail at `z_B=-0.900 mm`. |
| `R` | Independent reference-metrology frame. It is evaluation-only. |

Persistent poses shall be named as `parent_from_child`; for example, `C_from_H` maps coordinates in
`H` into `C`. Store translation as `[x,y,z]` and a normalized quaternion as `[x,y,z,w]`. Unlabelled
matrices and persisted Euler triples are forbidden. Calibration files shall state covariance order,
frame, and whether orientation errors are left- or right-multiplicative.

### 2.2 Honest observable state

The macro head shall carry an asymmetric, surveyed fiducial group so that `C_from_H` is constrained
in six degrees of freedom. Independent qualification metrology should likewise use asymmetric tool
and fixture marks to survey `C_from_T` and `C_from_S` in six degrees of freedom. The M1e controller,
however, intentionally consumes only the translation and axis obtained from symmetric paired
tool/socket targets. It does not use those evaluation marks to manufacture a roll estimate.

A plain cylindrical peg does **not** constrain rotation about its own axis. Its default mating state
is therefore the five-dimensional vector

`[x_B, y_B, z_B, delta_theta_x, delta_theta_y]`,

where the two small-angle components tilt the peg axis and roll about `+z_B` is absent. Its
covariance is 5 x 5 in that declared order. If a later peg carries a surveyed asymmetric mark and
the detector uses it, a six-dimensional body state may be reported under a new schema/version.
Round-peg roll shall not be invented from commanded tool roll. For insertion, the directly useful
relative state is lateral offset in `S`, axial tip-to-seat distance, and two-axis tilt; the tool pose
and held transform remain separate estimates.

The runtime/bench conversion is therefore explicit:

`B_tip = B + 0.900 mm z_B`, `S_entrance = S - 0.400 mm z_S`, and
`S_seat = S + 0.400 mm z_S`.

Never compare a peg-centre/socket-centre offset directly with a tip-to-seat tolerance.

## 3. Physical coupon

### 3.1 Arm, tool, and peg

The arm coupon shall preserve the current reference geometry unless a versioned as-built file says
otherwise:

| Item | Coupon nominal | Required as-built record |
| --- | ---: | --- |
| Upper link / forearm / wrist-tool offset | 32 / 30 / 15 mm | Pivot-to-pivot dimensions and uncertainty |
| Joint tendon moment arm | 1.65 mm | Each direction and joint angle |
| Motor capstan radius | 3.00 mm | Groove effective radius under working tension |
| Tendon payout span | 12 mm | Usable range before hard stop/slack |
| Tendon pretension / limit | 1.2 / 4.0 N | Per tendon, with instrument and temperature |
| Gripper opening | 0.080--2.800 mm | Commanded, encoder-inferred, and optically measured opening |
| Jaw half-extents represented by M1e | 0.100 x 0.250 x 0.200 mm | Actual collision envelope of each jaw/pad, open and closed; the legacy M1c 0.600 mm axial half-length is not the M1e coupon geometry |
| Replaceable per-pad working compliance | 12 micrometres per pad, initial model | Measured per-pad force-displacement curve and hysteresis; distinguish it from total diametral jaw closure |
| Provisional configured grip-force ceiling | 0.15 N modeled/configured, not hardware-qualified | Measured safe load-path/contact limit; a lower per-test software limit is mandatory |
| Peg diameter | 0.400 mm nominal | Diameter at no fewer than three axial stations and four azimuths |
| Peg overall capsule length | 1.800 mm nominal | Tip, end form, straightness, cylindricity, material, and mass |
| Peg straight cylindrical segment | 1.400 mm nominal | Axial extent and surface finish/condition; capsule half-segment is 0.700 mm |

The nominal peg dimensions match the M1e `0.200 mm` radius and `0.700 mm` capsule half-segment:
the leading-tip offset from `B` is 0.900 mm and the idealized overall capsule is 1.800 mm long. A
real wire or pin need not have hemispherical ends, but its measured end
geometry must replace that envelope before physical contact results are compared with simulation.
Use a corrosion-resistant metal gauge pin/wire or a metallized calibration pointer whose diameter
and straightness can be independently inspected. Material and supplier are not frozen by this
protocol.

The tool must include both jaws, pads, fasteners, strain relief, local fiducials, and any pointer
collar in its collision envelope. The simulated M1e tool has a central palm recessed to
`z_T=-0.350 mm` and two 0.100 mm-radius, 0.700 mm-long coded rails centred at
`x_T=+/-0.800 mm`; their four axial stations are `[-0.250,-0.225,0.225,0.250] mm`.
Both rails must be detected independently and only their measured midpoint may define the
controller's tool-axis point. Fiducials shall not bridge a compliant joint whose deflection is
being inferred. A separate asymmetric qualification mark should break 180-degree and mirror
ambiguity, but that mark is evaluation/calibration evidence unless the controller schema explicitly
models it. Ordinary printed targets may aid acquisition but shall not establish micrometre truth.

The modeled tail grasp uses `B = T + 0.750 mm z_B`. With a 0.700 mm peg half-segment and a
0.200 mm M1e jaw axial half-extent, this leaves 0.150 mm nominal straight-shaft/jaw overlap; the
controller requires at least 0.100 mm after observed pose and uncertainty margins. The bench shall
measure this overlap and may not substitute the longer legacy M1c jaw.

### 3.2 Socket coupon and fixture

The first socket coupon is a removable, inspectable square guide that retains the reduced M1c
lateral opening/clearance but uses the explicit M1e centre/entrance/seat convention, rather than
pretending to be a final gearbox bore:

| Feature | Starting nominal |
| --- | ---: |
| Square entrance width | 0.650 mm |
| Wall thickness around opening | 0.200 mm |
| Straight guide/seat depth along `+z_S` | 0.800 mm |
| Entrance edge break | 0.010--0.025 mm at 45 degrees, measured rather than assumed |
| Flat coupon body | at least 6 x 6 x 2 mm |
| Fixture datum repeatability target | less than 5 micrometres translation and 0.05 degrees rotation, verified |

The 0.800 mm value is the **full** guide depth: relative to the runtime socket centre, the entrance
is at -0.400 mm and the seat at +0.400 mm. The 0.650 mm opening gives 0.125 mm nominal radial clearance to a centred 0.400 mm peg and is a
software-correlation coupon, not a precision-fit claim. The coupon body should also accept a
replaceable measured insert with a tighter 0.420--0.450 mm circular or square entrance if contact
sensitivity must be increased. Results from different inserts are separate datasets with separate
geometry IDs; they shall never be pooled as though they were the same contact.

The simulated visibility design adds two external 0.100 mm-radius coded rails centred at
`x_S=+/-1.000 mm`, each extending `z_S=+/-0.350 mm` and carrying the same four axial stations
`[-0.250,-0.225,0.225,0.250] mm`. Their as-built positions and covariance must be surveyed. Both
rails must be independently visible; the controller derives the socket-axis point from their
measured midpoint, not from the fixture's latent/survey truth. The socket shall be fixed by a 3-2-1
arrangement with a compliant side clamp. The entrance, guide,
seat, datum surfaces, and asymmetric qualification fiducial group shall be measured after assembly. Gear-critical
features are not used as fixture datums. A removable coupon is preferred so its entrance and seat
can be inspected between trials for debris or damage.

The physical rail mounts are part of the fixture collision envelope. The current simulator renders
the rails for optical occlusion but does not instantiate them as separate mechanics bodies, so the
bench shall add surveyed rail-mount keep-outs to planning before automatic motion; nominal spacing
alone is not a collision qualification.

### 3.3 Clear-wall coupon

Use material cut from the intended tube lot: nominal 160 mm inside diameter and `3.0 +/- 0.5 mm`
wall thickness. Record polymer, extrusion/casting process if known, lot, actual inner/outer radii,
thickness map, surface condition, stress/birefringence observation, and mounting preload. The wall
section shall span the complete camera and projector apertures with at least 10 mm margin and retain
the production curvature.

The optical bench shall support two repeatable configurations:

1. `direct_reference`: no wall in either ray path; and
2. `through_intended_wall`: the surveyed curved wall section installed at its intended incidence
   and orientation.

The direct case diagnoses the head. The through-wall case is the operational qualification
configuration. Removing and
reinstalling the wall must not move the head or target; verify this with independent datums. A flat
sheet is not a substitute for the curved production coupon. If the final macro head is inside the
tube and has no wall in its operational path, through-wall results remain a global-view and
contingency dataset and must not be charged to the macro model.

## 4. Optical and timing configuration

### 4.1 Two-view global acquisition coupon

Build only two views from one locked M1d global triplet. The starting transforms place the
entrance pupils at a 60 mm radius, `z_C = -106 mm`, and azimuths 0 and 120 degrees, aimed at the
fixture datum. This produces the M1d nominal 103.923 mm baseline and approximately 121.803 mm range
to the origin. Each camera starts with 1280 x 800 monochrome global-shutter sampling and a 68-degree
horizontal field. Survey and calibrate the actual transforms; do not force measured geometry back
to the nominal values.

The pair is intended to qualify coarse acquisition and entry into the macro capture volume. It does not prove
six-view QWV coverage. Translate the fixture over the portion of the QWV reachable by the bench and
record both-view, one-view, and zero-view regions. Near-contact operation shall not be enabled from
these coarse views alone.

### 4.2 Rigid macro head

The M1d geometry remains the starting philosophy, but the executable M1e scenario currently widens
the modeled field to cover the external paired socket rails:

| Parameter | Starting requirement, not achieved performance |
| --- | ---: |
| Macro camera | 1280 x 800 monochrome global shutter |
| Object field at nominal plane | 3.000 x 2.500 mm for M1e; retain a 2.500 x 1.5625 mm M1d comparison capture |
| Perpendicular working distance | 15.000 mm |
| Effective camera/projector entrance-pupil baseline | 12.000 mm |
| Nominal included triangulation angle | 43.603 degrees |
| Object sampling at nominal plane | 2.344 micrometres/pixel on the 1280-pixel axis; 3.125 micrometres/pixel on the 800-pixel axis for the M1e field |
| Structured-light burst | 8 patterns |
| Required pattern rate | 240 patterns/s |
| Nominal pattern exposure time | measured for selected hardware |
| M1d conservative sensor-to-estimate budget | 53.3 ms |
| M1d hard candidate gate | no more than 60 ms at the selected latency statistic |

The 12 mm dimension is an **effective entrance-pupil baseline**, not circuit-board spacing. The
camera, lens, projector/scanned source, illumination, wavelength/filter, and trigger electronics
remain bench downselects. Mount them on one stiff, thermally instrumented structure. Record
`H_from_K` and `H_from_P` with covariance at calibration temperature and after warm-up. A flexible
relative transform must be estimated continuously or the head fails this protocol.

The head is one rigid observer, not an unqualified moving sensor. The simulator synthesizes a head
pose about each requested region of interest and does not model the actuator, relocation error,
relocation time, cable sweep, or collision envelope. For this coupon, either keep `H` fixed and
index the pickup/socket fixture through its qualified field, or translate the entire rigid head on
a surveyed stage. Every indexed fixture or head position is a separate calibrated state with
settling and uncertainty evidence; none may inherit the simulator's ideal retile operation.

The simulator's ring observer tests a front surface arc against finite-field and camera/projector
occlusion, including carrier self-shadowing, and only then triangulates a virtual feature centre
with that carrier removed. The bench must replace this virtual-centre boundary with actual image
detection: preserve the raw surface pixels, fitted arc/ellipse residual, visible angular span,
self-/external-occlusion labels, and rejected fits. A reported rail midpoint is valid only when both
physical rails were independently detected in the same accepted temporal envelope.

At 10, 15, 20, and 25 mm perpendicular distance, record field size, focus position, slanted-edge
MTF, distortion residual, saturation margin, structured-light contrast, and working clearance.
Only 15 mm is the initial M1e operating plane; other distances establish capture and degradation.
Full-housing claims require registered tiles and are outside this coupon.

### 4.3 Trigger and timebase instrumentation

All devices shall share a named monotonic timebase or have an estimated clock transform with
offset, drift, and uncertainty. Instrument at least:

- controller trigger-request edge;
- projector pattern-valid light using a fast photodiode in a small reference patch;
- camera exposure-active/strobe signal, or a validated optical exposure marker;
- camera hardware timestamp and frame counter;
- host frame-receive timestamp;
- detector start/end timestamps;
- estimator update publication timestamp; and
- command acceptance and motion-start timestamps.

Use a logic analyser or oscilloscope with no worse than 1 microsecond resolution. The sensor itself
must expose a usable global-exposure timing signal or be optically characterized; a host API call
time is not exposure time. Log dropped/duplicated frames, pattern ID, frame ID, queue depth, and
clock resynchronization.

`capture_to_estimate_latency_s` starts at the midpoint of the first exposure whose data contribute
to an estimate and ends when that estimate is atomically visible to the controller. Also report
last-exposure-to-estimate latency and total trigger-to-estimate latency. A latency figure without
this definition is invalid.

## 5. Required gauges and instrumentation

Use capability requirements rather than committing to expensive production hardware:

| Item | Minimum coupon capability |
| --- | --- |
| Calibration target | Rigid asymmetric dot/corner target spanning the 3.0 x 2.5 mm M1e macro field (and the retained 2.5 x 1.5625 mm M1d comparison field), with independently measured feature coordinates and less than one-third of the intended lateral-error budget uncertainty |
| Held-out 3D gauge | Separate from the calibration target; at least 12 identifiable points/edges over three depths, surveyed with uncertainty less than one-third of the intended depth-error budget |
| Length/scale artefact | Traceable pitch/line or gauge spacing over 0.1--3.0 mm; certificate or in-house comparison and uncertainty recorded |
| Independent lateral metrology | Evaluation-only microscope, interferometric/capacitive probe, or calibrated stage encoder with resolution no worse than 0.25 micrometres and expanded uncertainty no worse than 0.5 micrometres over a 50 micrometre move when claiming a 5 micrometre MRC |
| Independent axial metrology | Evaluation-only displacement measurement with resolution no worse than 0.25 micrometres and expanded uncertainty no worse than 0.5 micrometres over a 50 micrometre move when claiming a 5 micrometre MRC |
| Force/compliance gauge | Independently resolved axial and lateral channels with calibrated range at least 0.20 N in every tested force sign, resolution no worse than 0.5 mN, overload protection, and current calibration; the range must cover the provisional 0.15 N grip ceiling with margin |
| Jaw-opening and per-pad deflection gauges | Independent left/right pad channels plus total-opening measurement, each resolving 0.25 micrometres or better with expanded uncertainty no worse than 0.5 micrometres over 0.35--0.70 mm |
| Timing gauge | Logic analyser/oscilloscope and photodiode, 1 microsecond resolution or better |
| Environment | Head, wall, fixture, motor, and ambient temperature sensing to 0.2 K; humidity recorded; vibration indicator or accelerometer strongly preferred |

The same macro camera/projector estimate may be the controller sensor, but it may not be its own
reference for correction accuracy, hold jitter, socket geometry, or seating. A borrowed or
time-shared metrology instrument is acceptable; purchasing a production CMM is not a prerequisite.
Gauge uncertainty shall be propagated into every reported residual and shall not be subtracted from
observed error unless a documented uncertainty model supports the operation.

The capability values above are claim gates, not purchasing requirements. A lower-capability gauge
may be used to explore a condition, but that condition is `reference_metrology_incapable` and cannot
establish the MRC, pad-contact threshold, or force threshold it cannot resolve. Gauge range,
resolution, expanded uncertainty, calibration date, and uncertainty contributors shall be checked
against every claimed test magnitude rather than quoted only at full scale.

## 6. Calibration and held-out validation

### 6.1 Preparation

1. Inspect and clean the peg, jaw pads, socket, targets, and wall with a recorded procedure.
2. Assemble the arm, fixture, macro head, and two global cameras. Torque or preload fasteners to
   recorded values.
3. Warm the cameras, projector, motors, and illumination for at least 30 minutes or until each head
   temperature changes less than 0.1 K over 5 minutes. Record whichever condition takes longer.
4. Survey `C_from_H`, `C_from_F`, `F_from_S`, `H_from_K`, `H_from_P`, `C_from_G0`, and `C_from_G1`
   with uncertainty. Survey wall installation datums separately.
5. Freeze camera settings: exposure, analogue/digital gain, black level, lens focus/aperture,
   illumination current, projector sequence, and any ISP controls. Automatic exposure, focus,
   white balance, sharpening, denoising, and geometric correction shall be disabled or fully logged.
6. Generate a dataset manifest and calibration ID before taking validation data.

### 6.2 Three-way acquisition partition and lock

Before viewing outcomes, allocate immutable physical acquisition blocks to three disjoint
partitions. A block includes its run IDs, target poses, wall installation cycle, thermal cycle, and
fixture installation; adjacent frames from one unmoved setup may not be split across partitions.

1. `fit`: fit camera/projector geometry, clock transforms, feature models, and calibration terms.
2. `tune`: choose detector thresholds, outlier gates, covariance floors, contact-class thresholds,
   seating rules, and policy limits. It may reject a proposed model but may not be reported as final
   qualification evidence.
3. `qualification`: untouched trials used once to score the completely locked calibration,
   detector, estimator, contact classifier, and policy. It shall contain distinct physical run and
   wall-installation IDs and shall not merely be a withheld random subset of frames from a fit or
   tuning pose.

Hash the partition manifest before calibration. Record any excluded block with a reason fixed
without reference to its error or outcome. If a qualification result changes any parameter or
threshold, that result becomes tuning data and a newly acquired untouched qualification partition
is required. Repartitioning existing runs after their outcomes are known is forbidden.

### 6.3 Calibration fit

Calibrate macro-camera intrinsics and Brown--Conrady distortion, projector intrinsics, camera to
projector extrinsics, pattern correspondence, camera clock mapping, and `H` transforms. Use no fewer
than 30 accepted calibration-target poses spanning the field, depth, and incidence range. Retain
raw rejected images and their rejection reasons. Fit direct and through-wall models separately if
the wall effect cannot be represented by a stable calibrated transform/distortion term.

Calibration acceptance shall include parameter covariance/conditioning, not only mean reprojection
error. Repeated frames of one unmoved target do not create independent calibration information and
must not divide away a shared calibration floor.

Provide a rigid calibration-health feature in the same optical path and time envelope as each
manipulation burst. The simulator currently rejects a burst above the modeled 8 micrometre
`optics.maximum_calibration_reference_residual_m`, but its reference scene is otherwise empty and
does not qualify occlusion. On the bench, select this threshold only from tuning blocks, record the
accepted/missing sample count and residual for every burst, and score false acceptance/rejection on
untouched wall/field/temperature qualification blocks. The reference feature must not become an
evaluation-truth pose fed back to the controller.
The current modeled gate requires at least two accepted reference samples in a burst; replace that
count only from measured detection/dropout and correlation evidence, not convenience.

### 6.4 Untouched qualification geometry test

Lock calibration, detector, estimator, and decision parameters before exposing qualification-gauge
results. The qualification gauge shall not share fitted feature coordinates with the calibration
target, and no qualification image may have been used to tune a threshold. At minimum acquire:

- nominal working distances of 10, 15, 20, and 25 mm;
- centre and at least eight edge/corner positions within the macro field;
- surface/feature incidence of 0, plus and minus 10, and plus and minus 20 degrees where visible;
- direct and through-intended-wall configurations;
- the actual peg, socket edge/seat, tool fiducials, and a matte gauge material; and
- three repeat blocks separated by wall removal/reinstallation and a full head cool-down/warm-up.

Acquire at least 30 complete bursts per condition. Store every exposure, including dropout,
ambiguous decode, saturation, and outlier cases. Report signed bias, RMS, median absolute error,
95th and 99th absolute error, covariance, normalized residual, valid-return fraction, false-return
fraction, triangulation condition, and results by field position/range/incidence/material/temperature.
For dropout, false return, false grasp, false seat, and every other categorical outcome, report the
numerator, denominator, and a one-sided exact 95% binomial confidence bound. Zero observed failures
shall never be written as zero failure probability; for zero failures in `n` independent trials the
reported upper bound is `1 - 0.05^(1/n)`. Independence assumptions and any grouping by installation
or thermal block shall be stated.

For five-dimensional peg estimates, report translation and two-axis direction error. Do not report
axial roll. For socket/tool six-dimensional estimates, report translation plus geodesic rotation
error and the feature lever arm used. A result is qualification-held-out only if neither its images,
surveyed feature coordinates, labels, nor outcomes changed the fitted calibration, detector,
estimator, contact classifier, covariance floor, or policy thresholds.

## 7. Transparent-wall bias and dropout test

The through-wall test shall use paired acquisitions at unchanged `C_from_H` and `C_from_target`:

1. acquire a direct-reference burst;
2. install the wall on surveyed kinematic contacts without touching the head/target;
3. acquire the through-wall burst;
4. remove and reinstall the wall; and
5. repeat for at least 30 installation cycles.

Repeat at the centre and field edges, at the tested working distances, and at wall-ray incidence
angles of 0, 10, 20, and 30 degrees where the intended mechanics permit. Rotate the curved coupon
to sample at least three azimuthal sectors. Test the clean/dry intended state and one documented
representative surface-contamination state; the latter is a fault/maintenance boundary, not part of
the nominal accuracy pool.

For each camera feature and structured-light return, compute paired signed lateral/depth change,
contrast/SNR change, covariance change, decode residual, false-return rate, and dropout rate. A
wall-induced bias that repeats across frames is correlated calibration error, not random noise.
Spatially or thermally varying bias must be represented as a condition-dependent floor or a
declared invalid region. If two-view visibility is lost, the controller must stop and hold. It may
execute a previously validated, freshly observed retreat only while still inside its explicit
recovery policy; it may not reuse the last direct-view calibration indefinitely.

## 8. Capture-to-estimate latency

Collect at least 10,000 macro bursts and 10,000 global frames with normal logging and controller
load. Include startup, steady state, and a deliberately loaded but supported host condition. For
each stream report minimum, median, 90th, 95th, 99th, 99.9th, and maximum latency; clock offset and
drift; exposure/pattern jitter; frame/pattern loss; and estimator queue depth.

Run a separate stale-data fault by delaying delivery beyond the configured near-contact maximum.
The command trace must show that no new near-contact increment is accepted from the stale estimate.
The intended action is Stop and hold. Force unload and retreat are allowed only before terminal
failure when fresh geometry and the reverse swept corridor pass the M1e recovery policy.

The M1d 53.3 ms value is only an initial modeled allocation. The candidate passes the timing gate
only if the statistic selected by the versioned M1e scenario (initially the 99th percentile) is no
more than 60 ms and the measured tail is represented. A faster median does not excuse an unsafe
tail. If this gate fails, update the scenario latency distribution and staleness threshold; do not
round the measured result down to 53.3 ms.

## 9. Loaded hold jitter and drift

Install the final coupon jaw, pad, fiducial, and peg/pointer. Record the full moving payload mass,
centre of mass, and arm extension. At minimum test the real peg payload and a versioned worst-case
ballast up to the current 2 g manipulation envelope at 60 mm extension; the 5 g structural hold
test in `NEEDS_FIRST.md` remains a separate strength gate.

For each of at least five arm poses spanning the macro approach and transfer workspace:

1. approach from both signs of each active joint/tendon direction;
2. stop, engage the intended hold mode, and wait each candidate settling interval (50, 100, 200,
   and 500 ms);
3. acquire the complete eight-pattern burst while all motion commands remain constant;
4. continue observation for 2 seconds to expose creep;
5. repeat at least 100 times per pose/payload/direction; and
6. repeat cold, thermally steady, and after 30 minutes of representative motor duty.

Use simultaneous stationary fixture features to remove only demonstrable head/common-mode motion.
Report within-burst RMS and peak-to-peak displacement, burst-to-burst mean shift, 2-second drift,
axis cross-covariance, spectral peaks where sample rate permits, temperature slope, and approach-
direction dependence. The evaluation-only reference instrument determines distal motion; encoders
and tendon payout are explanatory variables.

The current schema maps the random-walk-equivalent component to
`estimator.loaded_hold_process_sigma_m_per_sqrt_s` and the repeatable signed loaded offset to
`motion.loaded_hold_error_world_m`. A later conditioned schema
calibration bundle is required to preserve per-axis, pose-, direction-, temperature-, and
payload-conditioned distributions rather than collapsing them into these scalar/vector inputs.
The current 2 micrometre lateral/depth allocation during macro bursts is a target, not an assumed
result. Failure closes the relevant phase budget and requires longer settling, a stiffer/shorter
pose, improved hold, shorter burst, or a new co-design calculation.

## 10. Commanded 5, 10, 20, and 50 micrometre corrections

At each of the five arm poses, command tool-frame corrections of 5, 10, 20, and 50 micrometres in
both signs of `x_T`, `y_T`, and `z_T`. Randomize magnitude/sign order with a recorded seed. For each
combination perform at least 50 trials after a positive-direction preload and 50 after a negative-
direction preload. Use the same bounded velocity, acceleration, settling, and stop-and-look policy
intended for M1e. Record motor/encoder motion, tendon state, tool estimate before/after, independent
reference displacement, settling trace, and any rejected command. Interleave at least one
randomized no-command hold control for every ten commanded trials so drift and analysis choices
cannot turn stationary noise into a small-correction pass.

For requested correction vector `q` and reference-measured displacement `d`, report:

- signed axial gain `(d dot q) / (q dot q)`;
- vector residual `d - q` in `T` and `C`;
- orthogonal/cross-axis motion;
- settle time to the declared band;
- reversal versus same-direction residual;
- no-motion, wrong-sign, saturation, and overshoot counts; and
- temperature, pose, payload, and tendon-direction conditioning.

### 10.1 Minimum repeatable correction extraction

For each axis/pose/payload, the minimum repeatable correction (MRC) is the smallest tested magnitude
for which all of the following hold separately in both command signs and both preload directions:

1. at least 95% of trials produce a reference displacement with the commanded sign;
2. median signed gain is between 0.5 and 1.5;
3. `1.4826 * MAD` of signed displacement error is no more than one third of the command magnitude;
4. the 95th percentile orthogonal displacement is no more than one half of the command magnitude;
5. the lower 95% confidence bound on median signed displacement exceeds the independently measured
   no-command hold displacement;
6. the reference gauge's expanded uncertainty at that magnitude is no greater than one third of the
   permitted robust-scale bound in item 3 (`q/9`, or 0.56 micrometres for a 5 micrometre command);
   and
7. no motion, collision, force, or tendon limit is violated.

Report `greater_than_50_um` rather than extrapolating if no magnitude passes. Also report the full
direction-conditioned residual distributions; MRC alone is insufficient for simulation. Backlash
is the separation of same-magnitude opposite-approach median endpoints. Hysteresis is reported as
the closed-loop path separation over the randomized forward/reverse sequence. Do not infer distal
deadband from motor encoder resolution. If item 6 cannot be met, retain the measurements as
characterization but report `reference_metrology_incapable` for that candidate MRC.

For the M1d guarded-insertion candidate, the combined measured correction residual must leave no
more than the currently modeled 7.87 micrometre lateral arm allocation after optics, hold, latency,
and contact terms are recomputed. Passing a 10 micrometre command does not itself establish a 7.87
micrometre closed-loop residual.

## 11. Grasp compliance and evidence

Calibrate the force instrument and independent left/right pad-deflection channels first and protect
them with a mechanical stop. With the 0.400 mm peg at known offsets, close the jaws in increments no
larger than 2 micrometres through first contact and the intended 12 micrometre **per-pad** working
region. `commanded_pad_compression_m` in the schema-v1 scenario is total diametral closure beyond
the nominal peg diameter; it is shared between the two pads and is not a per-pad deflection. The
schema-v1 minimum and maximum bilateral deflection gates apply separately to each recorded pad.
For the symmetric centred nominal only, 20 micrometres total closure corresponds to approximately
10 micrometres per pad; measured asymmetry shall not be overwritten with that expectation.
Test at least:

- centred peg;
- lateral closing-axis offsets of plus/minus 5, 10, 20, and 40 micrometres;
- axial/depth offsets of plus/minus 50, 100, 250, and 400 micrometres;
- peg-axis tilts of 0, plus/minus 0.5, and plus/minus 1.0 degrees;
- new and run-in pads; and
- both jaw closing directions at cold and thermally steady conditions.

Acquire at least 30 trials per nominal condition and 10 per boundary/fault condition. Measure each
jaw's contact onset on independently sampled left/right channels, total opening, each pad's
deflection, force versus closure, unloading hysteresis, lateral centring shift, slip force under
axial/lateral pull, and peg pose before and after lift. Bilateral contact is unqualifiable if either
pad channel is absent, shares an undocumented aggregate threshold with the other pad, saturates, or
lacks a traceable calibration over the tested range. The initial commissioning software force limit
shall be 0.010 N and may be increased only from observed margin; it shall never exceed the
provisional 0.15 N configured ceiling. That number is not a hardware-qualified force limit until
coupon strength, load path, and calibrated measurements support it.

A guarded grasp is accepted only when the controller-visible record contains bilateral contact,
peg geometry inside the measured capture region, closure/deflection inside the calibrated curve,
force inside the configured band, a valid fresh peg/tool estimate, and a post-lift observation
consistent with ownership. Evaluation reference data may score false acceptance but may not assert
ownership for the controller. An outside-capture or unilateral-contact trial must be rejected for
that reason rather than becoming a rigid attachment.

After grasp, measure `T_from_B` over repeated acquisitions and lifts. Store its mean plus full
covariance conditioned on jaw opening, force, approach direction, pad ID, and payload. This is the
held-part transform uncertainty used by M1e. Kinematic attachment remains an acceptable simulation
reduction only inside this observed uncertainty and tested acceleration envelope. It does not prove
a non-slip rigid grasp.

### 11.1 Loaded lift and transfer envelope

Qualify the acceleration envelope rather than inferring it from a static grasp. Use the actual peg
and the versioned worst-case manipulation ballast, each at every approach/transfer arm pose used by
the end-to-end coupon. Exercise the exact M1e lift and transfer path at 25%, 50%, and 100% of the
scenario velocity and acceleration limits, with both motion directions and a reversal, for at least
30 trials per payload/speed fraction. Include the configured settle-and-observe dwell before lift,
after lift, at the transfer endpoint, and after reversal.

For every trial record the requested and accepted trajectory, actual velocity/acceleration trace,
payload mass and centre of mass, tendon/hold state, pad channels, force channels, and observed and
evaluation-only `T_from_B` before grasp, after lift, at the transfer endpoint, after reversal, and
after final dwell. Report translation/tilt change, creep, slip/loss counts, covariance coverage, and
their exact binomial confidence bounds. An acceleration condition enters the qualified envelope
only if controller-visible ownership remains valid, reference motion remains inside its predeclared
bound, contact channels do not saturate, and no force, collision, or tendon limit is exceeded.
Outside this matrix, the M1e kinematic attachment remains an unqualified simulation reduction.

## 12. Guarded insertion trials

Measure the as-built peg and socket geometry before and after the series. Independently resolved,
time-synchronized axial and lateral contact-force channels are mandatory and shall be calibrated in
the assembled load path. If either required channel is absent, saturated, outside calibration, or
not synchronized to the pose and command trace, guarded insertion and recoverable-contact behavior
are unqualifiable rather than assumed from motor current. Use stop-and-look axial increments no
larger than 10 micrometres within 0.20 mm of expected lead-in; begin at no more than 0.10 mm/s and
increase only within the versioned safe envelope. A hard mechanical/software stop shall prevent
force above 0.050 N. Initial commissioning shall use a 0.010 N stop threshold until force paths and
coupon strength are known. The simulator's configured force proxy is not a hardware force claim.

Run the following starting matrix in both direct and operational through-wall optical conditions:

| Condition | Lateral offset at entrance | Tilt | Minimum repeats | Expected controller outcome |
| --- | ---: | ---: | ---: | --- |
| Nominal | 0, 5, 10 micrometres | 0 degrees | 30 each | Free/lead-in, converge, observed seated state |
| Recoverable lead-in | 25 and 75 micrometres | 0 and 0.5 degrees | 10 each | Detect contact, unload, observed lateral correction, continue only if uncertainty and force recover |
| Current recovery-limit challenge | 110 micrometres | 0 and 0.5 degrees | 10 each | Exceeds the schema-v1 95 micrometre modeled recoverable bound: unload and retreat; it may be reclassified only by a new versioned threshold selected on tuning data and passed on untouched qualification trials |
| Interference boundary | 130 and 160 micrometres | 0 and 1.0 degrees | 10 each | Reject before contact or detect excessive interference and retreat |
| Axis-tolerance boundary | 0 and 10 micrometres | 1.25 and 1.60 degrees | 10 each | The 1.25 degree case may continue only inside all gates; the 1.60 degree case exceeds the schema-v1 0.025 radian (approximately 1.43 degree) seating bound and shall be rejected or unloaded/retreated |
| Axial false-seat | centred with debris-free temporary depth stop 25 and 50 micrometres above seat | 0 degrees | 10 each | Do not declare seated; hold/unload/retreat |
| Reproducible jam coupon | centred, with versioned removable non-damaging jam insert | 0 and 0.5 degrees | 10 each | Detect force/contact growth without seated evidence, stop before the configured limit, unload, and retreat as `jam` |
| Observability fault | nominal geometry with socket or peg mating feature intentionally occluded | 0 degrees | 10 each | No near-contact increment; hold then retreat/reacquire |

For a tighter replaceable insert, scale offsets to its measured radial clearance and preserve at
least three points inside, one near, and two outside the recoverable boundary. Do not reuse the
0.650 mm opening's labels.

The jam insert shall be a measured, removable shim, wedge, or deliberately undersized sacrificial
guide that creates repeatable contact before the seat without relying on debris or damaging the
production-intent coupon. Give each insert an as-built geometry ID and predeclare its expected
contact station. Preserve the full force/deflection trace through stop, unload, and retreat. After
every jam trial, return to a documented reset pose, remove load, and inspect/record peg, insert,
socket, and pads before the next repeat; any permanent set or damage ends that geometry's series.
The controller shall identify `jam` from stalled observed progress plus the calibrated contact trend
before the hard force ceiling. Merely tripping the force limit or timing out does not pass the jam
fault trial.

Classify each increment using controller-visible evidence as `free_motion`, `lead_in_contact`,
`recoverable_lateral_contact`, `excessive_interference`, `seated`, or `jam`. Log observed relative
pose/covariance/age, commanded increment, measured flexure/force proxy, inferred contact state,
clearance prediction, and chosen action. A seating decision requires all of:

- fresh valid peg/socket features or a documented seat feature that remains observable;
- relative lateral/tilt state inside the scenario seating tolerance;
- observed axial position consistent with the measured seat depth;
- a repeatable contact/deflection signature inside the calibrated envelope;
- no further permitted axial increment without rising contact evidence; and
- stability during a final observation burst after force unload.

Force alone cannot prove seating. Pose alone cannot prove contact. A high force, stale estimate,
hidden mating feature, unexpected contact location, or lack of convergence shall stop increments
and hold. Unloading along `-z_S` and retreating are permitted only if a fresh estimate and
preflighted reverse corridor authorize that in-task recovery; otherwise the run remains stopped for
human safing. No result may be labelled a calibrated friction or insertion-force
prediction outside the measured material, speed, geometry, temperature, and surface-condition
envelope.

## 13. End-to-end coupon sequence and fault evidence

After component tests pass, execute the complete M1e loop at least 30 times without changing the
calibration:

1. coarse two-global-view acquisition;
2. bounded move into macro capture range;
3. settle, burst, estimate, covariance/residual gate, and iterative correction;
4. guarded observed grasp and lift verification;
5. bounded transfer with held-transform uncertainty propagation;
6. macro reacquisition of peg and socket mating features;
7. local alignment and guarded insertion;
8. observed/contact-supported seating verification;
9. geometric/contact-supported release; and
10. collision-free retreat.

Run separately labelled fault trials for optical dropout, calibration bias, delayed/stale delivery,
correction deadband, outside-capture grasp, insertion jam, occluded mating feature, carried-peg
keep-out collision, inconsistent/outlier observation, and deliberately impossible convergence.
Each shall stop for the injected reason. An in-task unload/retreat is acceptable only under the
validated recovery policy; terminal faults remain Stop+hold. A generic timeout is not a pass.
No near-contact command may occur while the required estimate is invalid, stale, above covariance
budget, missing required features, or based on an unaccepted innovation.

## 14. Raw-data contract

All raw inputs, including rejected observations, shall be immutable and content-hashed. Store lossless
or raw sensor frames where the device permits, plus a manifest linking every derived table to input
hashes, calibration ID, detector/estimator version, repository commit, and scenario hash. CSV or
Parquet tables are acceptable; JSON is preferred for per-run summaries. Schema changes require a
version increment.

### 14.1 Run and hardware metadata

Required fields include:

- `schema_version`, `dataset_id`, `run_id`, `trial_id`, `test_type`, and `condition_id`;
- `partition` (`fit`, `tune`, or `qualification`), `partition_manifest_sha256`, physical
  `acquisition_block_id`, thermal-cycle ID, fixture-installation ID, wall-installation-cycle ID,
  randomized order index, and randomization seed;
- `repository_commit_sha`, `scenario_id`, `scenario_sha256`, `controller_build_sha256`, and seed;
- arm/base/tool/peg/socket/wall/head/camera/lens/projector IDs and serial/lot numbers;
- firmware, driver, OS, detector, estimator, contact-classifier, policy, and calibration versions;
- all as-built dimensions and uncertainty-source IDs;
- `C_from_*` surveyed transforms and covariance order;
- payload mass/centre of mass, pad ID/wear count, tendon IDs/pretension, and fixture clamp setting;
- every reference/contact/displacement/force/timing gauge ID, calibration certificate/version/date,
  calibrated range, resolution, expanded uncertainty and coverage factor, sample rate, and filter;
- exposure time in seconds, gain, focus/aperture, illumination, pattern sequence/rate, and trigger
  mode;
- ambient/head/wall/motor temperatures in kelvin, humidity fraction, and vibration summary; and
- operator, setup notes, cleaning state, damage/debris inspection, and deviation approval.

### 14.2 Per-exposure and observation fields

Required fields include:

- `native_clock_id` on every event; `trigger_request_time_ns`, `pattern_valid_time_ns`,
  `exposure_start_time_ns`, `exposure_end_time_ns`, `exposure_midpoint_time_ns`,
  `camera_hardware_time_ns`, `host_receive_time_ns`, `detector_start_time_ns`,
  `detector_end_time_ns`, `estimate_publish_time_ns`, `command_request_time_ns`,
  `command_acceptance_time_ns`, `motion_start_time_ns`, and `motion_stop_time_ns`;
- `clock_transform_id`, source/destination clock IDs, `valid_from_time_ns`, `valid_to_time_ns`,
  `offset_ns`, `drift_s_per_s`, offset/drift covariance with units/order, synchronization
  sample/sequence ID, fit residual in seconds, `last_sync_time_ns`, and
  reset/wrap/discontinuity flags;
- logic-analyser/oscilloscope/photodiode trace URI and SHA-256, channel ID, sample rate in hertz,
  trigger polarity/threshold, exact samples used to determine each edge, and edge-fit uncertainty;
- source/view/frame/burst/pattern IDs, sequence counters, queue depth, dropped/duplicate flags;
- raw-image URI and SHA-256, bit depth, black/saturation counts, ROI, `exposure_s`, and gain;
- feature ID/type, pixel coordinates, pixel covariance, detector score, SNR/contrast, code/correspondence,
  ray residual, triangulation condition, and accepted/rejected status with enumerated reason;
- paired-target entity and side (`positive`/`negative`), surveyed rail offset/covariance, both raw
  reconstructed rail points, their cross-correlation where known, midpoint/covariance, and a flag
  proving that both sides contributed;
- visibility/occluder IDs, valid source/view count, calibration ID, drift/bias state, and wall mode;
- `wall_pair_id`, direct/through-wall member, `wall_installation_cycle_id`, pair order index,
  `C_from_W` with covariance/order, `wall_sector_id`, `wall_azimuth_rad`, `ray_incidence_rad`,
  `surface_condition_id`, mounting preload in newtons, paired target-pose ID, and head/wall/target
  temperatures in kelvin;
- observation measurement vector, named frame, covariance order, timestamp, latency, and quality; and
- outlier/inconsistency test statistic and threshold.

### 14.3 Estimator, command, mechanics, and contact fields

Required fields include:

- entity/state ID; reduced or six-dimensional state type; `parent_from_child`; position/quaternion
  or reduced-axis components; covariance; prediction/update timestamps; age; source/view count;
- innovation vector/covariance, normalized residual, validity/degradation state, rejection reason,
  and correlated-floor ID;
- task phase, correction iteration, guard inputs, guard result, policy action, retry count, and
  stable decision/event sequence number;
- requested correction vector in its named tool frame; accepted `C_from_T` target; preload direction;
  randomized sequence index/seed; command/no-command-control ID; requested and measured velocity and
  acceleration traces; configured settle interval/band; first in-band and accepted-settled times;
  post-run link IDs to evaluation-reference samples immediately before and after the command (never
  mapped into the controller process); and rejection reason;
- requested and accepted tool/joint/tendon/jaw state; velocity/acceleration limit; command
  request/acceptance and motion start/stop timestamps; controller mode; and rejection reason;
- motor position/current/voltage, encoder position, capstan payout, tendon tension where measured,
  brake/hold state, total gripper opening/velocity, total commanded diametral compression, and
  separate left/right pad deflection;
- for every contact sample: sample ID, native timestamp and clock ID, channel/sensor ID, jaw side or
  axial/lateral axis, raw ADC count and/or voltage, engineering unit, calibration matrix/curve and
  calibration ID, sample rate, analogue and digital filter settings, tare/zero value and time,
  saturation/over-range/health flags, controller threshold and hysteresis state, and provenance;
- contact pair/location/normal, left/right contact bits and onset times, calibrated per-pad
  deflections, resolved axial/lateral force estimates, force-proxy state, interference/clearance
  prediction, and collision/swept-volume result;
- controller-inferred ownership and contact class, clearly labelled as hypotheses rather than plant
  or reference truth, plus `T_from_B` estimate/covariance;
- lift/transfer trajectory ID, payload condition, speed/acceleration fraction, reversal ID, dwell
  interval, observation station, and ownership/slip decision at grasp, lift, endpoint, reversal, and
  final dwell; and
- stale/uncertainty/visibility/collision/force/convergence safety flags and commanded recovery.

Calibration-bundle exports shall provide canonical
`loaded_hold_process_sigma_m_per_sqrt_s`, `free_prediction_sigma_m_per_m`,
`lateral_stiffness_proxy_n_per_m`, and `axial_stiffness_proxy_n_per_m` values. They shall also export
`loaded_hold_error_world_m` with `world_frame_id: C`.
`left_pad_deflection_m` and `right_pad_deflection_m` are per-pad measurements;
`commanded_pad_compression_m` is the total commanded diametral closure. These quantities shall not be
pooled under an ambiguous `pad_compression_m` column.

### 14.4 Evaluation-only fields

Independent reference values shall live under a visibly separate `evaluation_only` namespace or
file. Each exact sample shall include sample/event/link IDs; native and mapped timestamps; reference
clock ID and clock-transform ID; raw reading; engineering unit; calibrated value; `R_from_*` pose or
displacement with covariance/order; gauge ID and calibration ID; range, resolution, uncertainty and
coverage factor; sample rate; tare; analogue/digital filter; interpolation method and bracketing
sample IDs; synchronization residual; and saturation, clipping, dropout, health, and extrapolation
flags. It shall link explicitly to the exposure, estimator update, contact sample, command, motion,
or state transition it scores rather than storing only one final value per trial.

This namespace also contains manually adjudicated contact/seating outcome, inspection/damage state,
final measured errors, and acceptance result, including assessor and method. The controller process
shall not map this stream, and controller ownership shall never be populated from the reference
label. Reports must be reproducible with the evaluation file withheld, except for post-run scoring
fields.

## 15. Replacing modeled scenario assumptions

Never copy the best run or an unqualified RMS into a scenario. Preserve raw distributions and
derive a conservative, versioned calibration bundle. Unless a later qualification plan supersedes
it, use the upper 95% confidence bound of the named statistic and stratify by pose, direction,
temperature, wall mode, and payload. Do not pool a bimodal or condition-dependent result into one
optimistic Gaussian.

| Existing modeled input/behavior | Coupon evidence | Replacement rule |
| --- | --- | --- |
| `coupon.peg_diameter_m` and `coupon.peg_half_segment_m` | Multi-station diameter, straight segment, cap/end-form, straightness, and tip-location survey | Populate an as-built geometry ID and conservative dimensions; if the end is not a capsule, replace the plant/contact geometry under a new compatible schema rather than forcing the measurement into a false radius. |
| `coupon.socket_depth_m`, `socket_wall_thickness_m`, and `socket_fiducial_*` | Surveyed entrance, seat, wall, and both external target rails with covariance | Populate the measured centre-frame geometry and paired-target offsets/radius/extent; keep entrance at `-depth/2` and seat at `+depth/2`, or version the frame convention explicitly. |
| Macro/global localization sigma in pixels | Held-out signed pixel residuals by feature, field, incidence, SNR | Use robust within-condition scale (`1.4826 * MAD`) or a heavier-tailed empirical table; retain outliers/dropout separately. |
| Correlated calibration floor | Held-out group means across calibration repeats, wall installations, field sectors, and temperature | Use the covariance/upper bound of group bias once per fused estimate; never divide it by frame or feature count. |
| `optics.maximum_calibration_reference_residual_m` | Independent reference-target residual distributions, including wall, field, and temperature blocks | Freeze the threshold on tuning data, verify false-accept/reject rates on untouched qualification blocks, and reject rather than subtract an over-limit residual. |
| Surface/depth allocation | Peg/socket/material held-out axial residual | Replace with material/incidence-conditioned signed bias plus residual distribution. |
| Field of view and visibility | Accepted/rejected features over surveyed poses with complete tool/wall geometry | Populate measured capture bounds and occlusion map; untested regions are invalid, not interpolated pass regions. |
| Dropout/outlier rate | All bursts, including rejected returns | Store empirical probability plus confidence bound by condition and a deterministic seeded sampling table. |
| Sensor/processing latency | 10,000-burst timestamp trace | Use measured empirical distribution or conservative selected quantiles; preserve tails and clock uncertainty. |
| Drift/bias | Cool-down/warm-up and temperature/wall repeat blocks | Fit only supported temperature/time dependence; otherwise use bounded correlated floor and shorten recalibration interval. |
| `estimator.loaded_hold_process_sigma_m_per_sqrt_s` and `motion.loaded_hold_error_world_m` | Loaded within-burst and post-stop reference motion | Convert the random component to a conservative m/sqrt(s) equivalent and place the repeatable signed offset in the declared world frame; use a later conditioned distribution when one scalar/vector cannot represent the evidence. |
| Minimum correction / backlash / hysteresis | Randomized 5/10/20/50 micrometre bidirectional trials | Set correction floor to the worst qualified MRC and use direction-conditioned residual/deadband distributions, not encoder resolution. |
| `estimator.held_transform_sigma_m` | Post-grasp/lift `T_from_B` measurements | The current scalar takes a conservative translational bound; use a schema-v2 full measured covariance/mixture conditioned on force, jaw direction, pad and pose, and reject outside the tested capture region. |
| Per-pad compliance / grasp force proxy | Total-opening, separate left/right force-opening-deflection, and slip tests | Fit a monotone piecewise curve per pad only over the tested range, with residual bounds and break/reject limits. Preserve total commanded diametral compression separately from each pad's deflection. |
| Contact-process allocation and `*_stiffness_proxy_n_per_m` | Misalignment/increment/resolved-force insertion matrix | Use measured pose-change and force-envelope distributions per contact class; retain stiffness units as N/m, never torque, and do not identify Coulomb friction unless separately excited and fitted. |
| Seating/contact thresholds | Observed pose plus calibrated flexure signature and reference outcome | Select thresholds only on `tune`; freeze them before `qualification`. Report dangerous-false-seat numerator/denominator and a one-sided exact 95% binomial upper bound on untouched qualification trials. Zero observed events is not a zero-rate estimate, and the qualification sample size must support the predeclared rate gate. |

After substitution, rerun the executable M1d phase budget and every M1e nominal/fault scenario. If
the budget does not close, the hardware does not pass by relabelling the measured distribution.
Change settling, observer geometry, mechanics, passive lead-in, correction policy, or phase target
and repeat the affected qualification.

## 16. Acceptance gates

The coupon is accepted for continued M1e development only when all applicable gates pass:

1. **Traceability:** all as-built geometry, transforms, calibration inputs, raw frames, timings,
   controller build, scenario, data-processing code, and reports are versioned and hashed.
2. **No circular truth:** controller outputs are unchanged when evaluation-only streams are removed,
   and no controller-visible field is populated from a reference pose or post-run label.
3. **Optical geometry:** the measured macro field contains required peg/tool/socket features at the
   declared 15 mm plane; the measured sampling and held-out uncertainty close the executable phase
   budgets. The current approximately 3.02 micrometre lateral and 3.43 micrometre depth RMS values
   remain modeled until replaced.
4. **Covariance honesty:** held-out normalized residuals and interval coverage do not show material
   overconfidence; correlated floors remain correlated during burst fusion.
5. **Wall qualification:** bias, dropout, false returns, and invalid sectors through the intended
   clear wall are measured and represented. Required feature loss produces Stop+hold or an
   explicitly authorized, fresh/preflighted in-task retreat.
6. **Timing:** the selected latency statistic initially meets 60 ms, stale delivery is detected,
   and no stale estimate drives a near-contact increment.
7. **Loaded hold:** measured burst motion fits each phase's replaced hold allocation after the
   configured settling interval.
8. **Correction:** all three local axes have a reported MRC and direction-conditioned residuals;
   the recomputed guarded-insertion lateral arm residual fits the phase allocation (currently 7.87
   micrometres modeled), and reference-metrology capability closes at every claimed magnitude.
9. **Grasp:** nominal guarded grasps provide independently measured left/right contact and post-lift
   ownership evidence; outside-capture/unilateral cases are rejected; force remains below the
   configured limit; held transform uncertainty is populated; and the actual lift/transfer
   acceleration envelope is qualified. A missing or aggregate-only pad channel cannot pass.
10. **Insertion:** all nominal trials seat within declared observed tolerances without force-limit
    or collision violation; synchronized resolved axial/lateral contact channels remain calibrated;
    recoverable cases unload/correct; and recovery-limit, axis-limit, interference, false-seat,
    occlusion, and measured jam-coupon cases fail closed for their intended reasons.
11. **Release/retreat:** release requires observed loss of bilateral holding contact with the peg
    still seated, and the tool retreats without moving the peg or violating swept clearance.
12. **Repeatability:** at least 30 unchanged-calibration end-to-end nominal trials are reported,
    with no hidden exclusions. Failures remain failures and are analysed by explicit reason. Every
    categorical safety outcome includes its numerator, denominator, and exact binomial confidence
    bound; zero observed failures is not reported as a zero failure probability.
13. **Fidelity label:** reports state `hardware_coupon_reduced_contact_not_machine_qualified` until
    a later correlated F3 gate exists.

An optical or correction miss is not automatically a request for costlier arms or cameras. The
first co-design responses are to inspect calibration bias, shorten/retime the burst, improve rigid
mounting and settling, reduce field, change view incidence, add passive lead-in/compliance, or use a
direction-aware correction. Any improvement must return through the same held-out protocol.

## 17. Required report package

The bench deliverable shall contain:

- as-built drawings and frame diagram;
- calibration and held-out-gauge certificates/uncertainty statements;
- immutable fit/tune/qualification partition manifest and a register of threshold/model changes;
- immutable raw images, timing traces, reference streams, controller-visible streams, and manifests;
- direct and through-wall optical residual/dropout summaries;
- capture-to-estimate latency distributions;
- loaded hold, drift, correction, backlash, and MRC results;
- grasp force/compliance/capture-region, per-pad evidence, held-transform, and loaded-transfer-envelope
  results;
- guarded-insertion traces with synchronized axial/lateral force, pose, covariance, decisions,
  reference outcome, recovery/axis boundary trials, jam-insert geometry and post-trial inspection;
- categorical outcome counts and exact binomial confidence bounds;
- end-to-end nominal and fault reports;
- the generated calibration/distribution bundle proposed for the M1e scenario;
- executable budget results before and after substitution; and
- an exceptions/failures register with no deleted trials.

The package shall end with a one-page claim statement listing what was measured, the tested
envelope, what remains modeled, every failed gate, and the exact statement allowed in project
documentation. Until all gates close, the only permitted conclusion is that a particular coupon
configuration was measured; it is not a hardware-qualified Pipe precision claim.
