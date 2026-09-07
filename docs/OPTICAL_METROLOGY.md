# Optical metrology subsystem

`pipe_optics::metrology` implements the cylindrical metrology specification as
an observation-only reconstruction layer, an explicitly synthetic sensor
frontend, and independent verification. Its numerical target is **3–5 µm RMS
of the 3-D error vector**, with ≤2 µm RMS repeatability. These are local
measurement objectives. Neither the whole tube nor the present hardware is
qualified to these values.

The baseline is a **candidate**, not a successful hardware design. The isolated
volume experiment already exposes incomplete structured-light coverage and
repeatability above target. Rejecting those conditions is part of the result.

## Implemented boundaries

| Boundary | Implemented behavior |
| --- | --- |
| Synthetic acquisition | Actual sensor poses, thermal expansion, calibration perturbations, primitive/triangle occlusion, surface response, reduced optical blur, seeded image-feature noise and encoded intensities generate observations. Only this boundary receives plant geometry or truth poses. |
| Pixel reconstruction | Distorted calibrated rays initialize a weighted nonlinear multi-view solve. Finite-difference distorted projection Jacobians produce a full 3×3 covariance. Negative-depth, inadequate-angle, inconsistent, duplicate and temporally incompatible observations fail. |
| Rigid tool pose | All compatible feature observations constrain one six-DOF body. Different cameras may see different subsets. At least three non-collinear features with triangulatable observations initialize the solve; subsequent single-view features also contribute. Collinear constellations do not invent roll. |
| TCP transform | `tcp_from_fiducial` is **T_fiducial_to_TCP**, mapping fiducial coordinates into TCP coordinates. `world_from_tcp = world_from_fiducial * inverse(tcp_from_fiducial)`. A 6×6 Jacobian propagates orientation and position uncertainty through the TCP lever arm. |
| Identity | A separate identity code, arm ID and tool type accompany calibrated metrology features. Image-feature association is supplied to the solver; no QR/AprilTag raster detector is claimed. Identity print dimensions are not used as the dimensional standard. |
| Observed world | `ObservedWorld` contains distinct observed entities, health, uncertainty and acquisition provenance. Acquisition replaces previous observations. It is separate from command, encoder/FK prediction and plant state. |
| Control | `PrecisionContract` requires valid finite covariance, fresh available tool and target measurements, a precision-zone location, sufficient real camera support, calibration, reference residuals and thermal validity. Ideal synthetic data cannot authorize a precision operation. A physical contract also requires physical calibration and reference provenance. |
| Verification | `verify_point` compares a reconstructed estimate with independently known geometry excluded from calibration by artifact identity. No alignment to verification points is used to remove measured bias. Repeatability, pixel residuals, covariance and independent geometry error are different fields. |

The solver's initial tool pose comes from reconstructed fiducial geometry. It
does not accept a true pose, commanded pose, encoder pose or nominal FK as a
precision observation or initialization shortcut.

## Four sensing products

| Product | Interface | Target and current boundary |
| --- | --- | --- |
| Global occupancy | `reconstruct_occupancy` consumes calibrated backlight silhouettes and builds a bounded, tiled voxel visual hull. Unknown pixels/views remain unknown; hidden concavities remain occupied. | Approximately 50–200 µm is the design accuracy objective, not a claim derived from voxel pitch. Segmentation is an external input. Edge uncertainty and voxel extent dilate masks; planning needs separately validated segmentation and clearance margins. |
| Tool/gripper pose | `estimate_tool_pose` yields directly observed fiducial/TCP poses, full covariance and supporting pixels. | 3–5 µm local precision target; tracking uses a distinct larger localization-noise model. Distal marker geometry, sizes, normals and TCP calibration are configurable. |
| Critical features | `reconstruct_point` and `fit_axis_feature`; the latter consumes separately measured section centres. | 3–5 µm local target. Shaft/bore axes can be represented by measured centres without reconstructing a complete surface. Raster circle/edge fitting and arbitrary CAD feature detection remain external inputs. |
| General surface geometry | `acquire_surface_scan` samples real scene intersections solely to generate camera/projector intensities, decodes them and returns reconstructed points. All pixels belong to the same timed pattern sequence. | 10–30 µm is a secondary objective. Stride controls surface sampling density, not claimed accuracy. Low signal, occluded rays and decode failure remain missing points. |

Tracking is configured at 60 Hz, with a 30–100 Hz intended design range. This
does not benchmark a camera, decoder or real-time throughput. Precision scans
are slower and require stopped or independently bounded motion. Occupancy and
dense scanning are headless APIs, not fabricated full-machine maps in a viewer.

## Inspectable candidate

`scenarios/metrology_cylindrical_v1.json` is the inspectable baseline. All lengths
are metres, time is seconds, angles are radians, temperature is kelvin, and
pixel quantities have `_px` suffixes. `a_from_b` always maps coordinates in b
into a. Cameras/projectors use +Z forward, +X right, +Y down. The authoritative
machine frame has Z along the tube. `world_from_tube` can represent a separate
metrology frame. Arm, fiducial, TCP, part and feature transforms remain explicit.

| Parameter | Baseline |
| --- | --- |
| Tube | 100 mm ID, 160 mm working length |
| Precision zone | 25 × 25 × 25 mm centred on the tube origin |
| Cameras | Six circumferential views, alternating ±10 mm axial offsets |
| Sensor | 4096 × 3000, 3 µm pitch, 12-bit monochrome, global shutter, raw access and hardware trigger |
| Sampling | 9 µm/pixel at the camera focus plane; spatial changes are reported |
| Camera optics | Fixed/locked focus, 2 mm aperture, calibrated Brown–Conrady distortion; effective focal distance approximately 17 mm |
| Projector | One calibrated inverse camera, 1920 × 1200, 25 µm/pixel at focus |
| Illumination | 450 nm baseline; spectral response also represents red/NIR; diffuse, backlight, structured, grazing and dark acquisition states; optional parallel/crossed linear polarization |
| Acquisition | 200 µs exposure, 1 µs trigger sigma, 5 µs admitted inter-camera skew, 5 ms processing latency |
| Hybrid scan | Complementary Gray bits for both axes, four phase steps per axis, dark/white frames: 54 frames at 2 kHz; 26.7 ms from first exposure start to last exposure end |
| Motion limits | 1 µm during an exposure; 0.5 µm over an encoded precision scan |
| Thermal structure | 23 × 10⁻⁶/K expansion, explicit anchor, structure/tube/reference/device temperatures and focal-temperature coefficients |

Sensor dimensions and effective focal distance must agree with pixel intrinsics.
The nominal thin-lens focal length is separate from the sensor-to-principal-plane
distance at macro focus. Defocus uses thin-lens image distance and the aperture's
circle of confusion. A Gaussian approximation to the Airy central lobe contributes
photon-limited localization noise; both optical paths attenuate structured phase
modulation. The configured 30 mm admissible focus interval is only a model domain,
**not** 30 mm of demonstrated sharp depth of field. Telecentric optics are an
explicit unsupported candidate, so they cannot silently execute as pinholes.

The implementation keeps camera positions, orientations, intrinsics, distortion,
sensor dimensions, focus, aperture, projector configuration, illumination,
thermal state, pattern parameters, timing and noise editable. Changing camera
positions changes baseline, angle, Jacobian covariance and actual ray visibility.
Changing aperture, focus or wavelength changes modeled blur/modulation. Marker
size affects usable contrast; marker position and tool geometry affect visibility
and TCP covariance. None of these controls is tuned to force a passing target.

## Error and calibration semantics

Pixel covariance is independent localization/photometric uncertainty in square
pixels. Reconstruction uses `(Jᵀ W J)⁻¹`, solved with diagonally equilibrated
Cholesky; an ill-conditioned/unobservable system fails rather than returning an
arbitrary covariance. For a parallel stereo test, depth sigma agrees with
`Z²/(f_px B) * sigma_disparity_px` and scales inversely with baseline.

The configuration separately names camera/projector calibration, residual
distortion, structural drift, thermal residual, surface interaction,
reconstruction, marker manufacturing and TCP calibration errors. The non-pixel
position terms are **total 3-D RMS priors**, divided by √3 for isotropic per-axis
sigma. They are retained once after fusion. Geometry/manufacturing terms also
retain a conservative angular floor based on the marker span. These priors do
not replace empirical verification and cannot average away with more frames.
`sqrt(trace(covariance))` is used for 3-D RMS; the legacy crate's per-axis RMS
convention is not substituted for this objective.

Synthetic acquisition perturbs actual sensor extrinsics, distortion and thermal
focus before nominal-calibration reconstruction. Surface-centroid and residual
algorithm biases are explicitly reduced random effects, not rendered BRDFs.
Manufacturing and TCP errors change the synthetic optical geometry. Seeded
common terms stay correlated between observations as documented in code. This
does not claim a fully identified joint distribution across objects and devices.

Calibration records distinguish sensor model, camera intrinsics/distortion/
extrinsics, projector intrinsics/distortion/extrinsics, world frame, tube
references, fiducial geometry, fiducial-to-TCP calibration and part features.
Each records its revision, frame names, validity and fit artifact IDs. Calibration
parameters are **imported**, not estimated by an intrinsic/extrinsic calibration
optimizer in this change. Synthetic nominal calibration is explicitly distinct
from a physical artifact fit.

Permanent references are measured through the same optical path. Their residuals
are evaluated in the existing machine frame, without a best-fit registration that
could conceal drift. Missing, stale or excessive residuals inhibit precision
admission. Thermal expansion is applied around an explicit structural anchor;
100 mm of aluminium expands by 2.3 µm/K in this model. Temperature limits are also
checked directly at command admission, not only in a cached health flag.

## Connection to the authoritative machine

`PointMotionRuntime::acquire_metrology` observes the **same** `Simulation` that
executes machine commands. All enabled rigid bodies, live serial-arm capsules
and jaw boxes supply occlusion. Box faces are triangles; gears conservatively
fill tooth gaps and bores. Additional CAD triangle geometry can enter through
`extra_occluders`. Self-tag exemptions are not used to see through tools.

The existing baseline machine is **160 mm ID**, outside the new preferred
80–150 mm study range. `metrology_candidate()` explicitly adapts camera mounts,
working distance and focal distance to those existing dimensions; a mismatched
100 mm optical configuration is rejected. The standalone 100 mm optical study
does not alter existing arm dimensions or silently assert a matching CAD design.

The machine adapter currently acquires stopped precision observations, advances
the authoritative clock through exposure/latency, and checks motion. A requested
`submit_precision_target` must pass the optical contract and existing mechanical
command/collision gates. The capture is consumed after command submission; a
prior machine command or changed configuration invalidates admission. This is
an executable optical admission boundary, not a new complete visual-servo task.
The M1e/M1f manipulation runtimes continue to use their separately documented
reduced optical pipeline. No existing acceptance result is relabelled as an
achievement of this subsystem.

Rail solids, tendons, marker mounts, reference supports, camera housings and
transparent-wall refraction are not complete in the current runtime geometry.
The adapter reports these omissions. Its visibility cannot qualify camera mounts
or prove assembled hardware clearance until the missing design geometry and
optical path are supplied. CAD is not treated as immutable.

## Reproduction and evidence

Run from an exact committed source state:

```bash
cargo run --locked -p pipe_sim_cli --bin pipe-metrology -- --print-config
cargo run --locked -p pipe_sim_cli --bin pipe-metrology -- \
  --config scenarios/metrology_cylindrical_v1.json \
  --source-revision "$(git rev-parse HEAD)" --repeats 12 --summary
cargo run --locked -p pipe_sim_cli --bin pipe-metrology -- \
  --machine --source-revision "$(git rev-parse HEAD)"
cargo test --locked -p pipe_optics --test metrology
cargo test --locked -p pipe_sim --test metrology
```

The same headless configuration and verification are exported to WASM as
`opticalMetrologyConfigJson` and `opticalMetrologyReportJson`.

Omit `--summary` to retain every independent reference, reconstructed coordinate,
error, covariance and supporting pixel. Summary evidence in
`docs/evidence/metrology_v1.json` preserves the exact source revision and
configuration hash. The 27 test points span 80% of each configured zone dimension;
12 repeats test 324 acquisitions. The boundary shell and arbitrary intervening
geometries are not thereby certified. The tool test uses 24 acquisitions of one
asymmetric constellation and orientation.

Sensitivity cases increase localization noise tenfold, raise structural
temperature by 5 K, or increase camera calibration perturbations tenfold. Reports
retain unsuccessful acquisitions and rejection reasons. Errors on accepted
points alone are not a coverage success. NEES and ellipsoid coverage quantify
covariance consistency; fixed-calibration repeated frames are correlated, so
their count is not treated as independent hardware confidence evidence.

The recorded baseline result is:

| Product | Accepted / attempted | 3-D error RMS | Repeatability RMS | Finding |
| --- | --- | --- | --- | --- |
| Passive critical points | 323 / 324 | 2.51 µm | 1.52 µm | One acquisition rejected for timing; no complete-volume qualification |
| Hybrid structured light | 124 / 324 | 3.76 µm | 2.72 µm | 176 low-signal and 24 field failures; repeatability misses target |
| Distal TCP, one pose | 24 / 24 | 2.02 µm | 1.06 µm | Synthetic local result only, not full machine or physical accuracy |

The separate live-machine sample has 132 occluding primitives. It reconstructs
one of four distal tools; three lack sufficient views in that arm configuration.
This is evidence that the optical/mechanical layout needs co-design, not an
excuse to substitute nominal FK for missing optical poses.

## Physical validation boundary

The supported eventual claim remains: independently characterized 3-D geometry,
excluded from calibration, reconstructed with ≤5 µm RMS within the declared
precision volume and operating conditions. A physical verification record must
identify the independent artifact, finite characterization uncertainty, physical
observations and physical calibration provenance. This implementation makes no
such claim. Camera/lens selection, measured MTF, actual detections, exposure/
trigger measurements, field-dependent calibration, material response, thermal
stability and independent artifact measurements remain required evidence.

No raster image formation, full BRDF/interreflection transport, autofocus,
transparent-wall refraction, general motion-compensated phase decoding, or
arbitrary CAD-feature detector is implemented. Dark/specular/translucent/
interreflecting, saturated, poorly textured and grazing cases have explicit
reduced failure behavior. Polarization changes the reduced signal model but
does not remove all reflection or material limitations. Gray transitions are
still idealized discrete classification; phase modulation has optical blur.

The calibration convention follows the primary
[OpenCV calibration model](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
Complementary Gray encoding is documented by
[OpenCV's GrayCodePattern reference](https://docs.opencv.org/5.0/extra_modules/classcv_1_1structured__light_1_1GrayCodePattern.html).
The separation of instrument errors, surface conditions and independent artifact
evaluation is informed by [NIST's structured-light error study](https://www.nist.gov/publications/sources-errors-structured-light-3d-scanners).
These sources support the model structure; they do not validate this candidate.
