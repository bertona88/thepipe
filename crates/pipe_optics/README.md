# pipe_optics

Dependency-free optical metrology simulation for native Rust and
`wasm32-unknown-unknown`.

The physical path is explicit:

1. A detector pixel is unprojected through the **drifted physical camera**.
2. The closest primitive surface is ray-traced.
3. The point must be inside the drifted projector frustum and have an unoccluded
   projector path.
4. Lambertian incidence, range, reflectance, shot noise, ambient signal, read
   noise and dropout determine whether a return exists.
5. Noisy, quantized camera/projector coordinates are unprojected through the
   **nominal calibration** and triangulated.
6. The result reports the true/measured point, covariance, ray residual,
   triangulation condition, SNR and a compact confidence score.

The raster `DepthSample` above is a sensor-debug result and includes truth for
post-run evaluation. Observed-state controllers should instead use
`StructuredLightRig::observe_feature_points`. Its input feature geometry is
simulation truth consumed only while generating camera/projector measurements;
its `FeaturePointObservation` output contains measured pixels, a nominal-ray
triangulated point, covariance, quality and an explicit missing reason, with no
true point, true range or truth-relative error. One camera/projector pair is one
triangulation head, not two statistically independent views. A configured
correlated calibration floor enters covariance once and is not averaged away.

Callers can populate a scene with already-triangulated geometry by supplying
`Geometry::Triangle` primitives; this crate does not load CAD, STEP, or STL files.
Spheres, boxes and finite capped cylinders are provided for fast fixture/arm proxies. The crate
does not claim that a cheap camera is micron metrology: pixel footprint,
baseline angle, speckle floor, occlusion and calibration drift remain visible in
each sample's covariance and failure reason. In particular, a shallow baseline
or coarse macro image can be tens to hundreds of micrometres uncertain, which is
material for 0.10 mm-module gears.

## Integration boundaries

The legacy `pipe_sim` gearbox runtime supplies active and placed parts as sphere proxies and samples
them from distinct camera indices. Successful ray returns gate availability,
confidence, range bias and covariance; neighboring samples are averaged per camera,
then distinct views are information-fused with a correlated floor. They do not themselves solve component pose.
The observation starts from a synthetic latent component pose error, and nominal
feature size is used only as a lever arm that converts position uncertainty to
orientation uncertainty. Image-to-CAD feature detection and 6D pose estimation remain
future work. The runtime also applies its camera numbers in a commanded
active-component-local frame rather than ingesting fixed tube/wrist transforms from CAD.

M1e uses the explicit feature-point path instead. Known labelled features on its peg, socket, tool,
and datum are projected and independently visibility-tested for the camera and projector, then
returned as controller-safe timestamped measurements. The tool's symmetric `+/-0.800 mm` targets
and socket's external `+/-1.000 mm` rails are each observed on both sides before their measured
midpoint is exposed as a centreline point; no latent-pose offset is subtracted after measurement.
Surface-arc probes retain carrier self-shadowing. Only after that gate passes does the caller remove
the feature's own carrier to triangulate a virtual fitted centre. A downstream deterministic
weighted least-squares estimator fits only the translation and axis direction constrained by those
features. Rotation about the circular coupon axis remains unobservable, so M1e reports a 5-DoF
axisymmetric pose rather than inventing 6D information. A camera/projector pair is one
triangulation head with two calibrated rays; repeated frames do not average away the configured
correlated calibration floor.

Explicit feature-point observations model geometric projection, finite fields of
view, opaque primitive occlusion, Lambertian/retroreflective signal, centroid and
depth noise, quantization, dropout, calibration drift and two-ray triangulation.
They do not model images, blur/MTF, the actual arc/ellipse fit, feature detection, correspondence failures
beyond configured dropout, transparent-wall refraction, photometric rendering,
wave optics, or pose inference. Multiple known geometrically separated features
are therefore required before a downstream estimator can claim the corresponding
observable pose components.

All M1e optical precision, covariance, visibility, and dropout results are modeled and
hardware-unqualified. The path assumes known feature identities and does not demonstrate that an
actual camera/projector and detector can recover them through the intended clear tube. The
qualification bridge must measure transparent-wall bias/dropout, timing, held-out reconstruction
error, and capture-to-estimate latency before scenario values can support a hardware claim.
