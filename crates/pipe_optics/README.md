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

Callers can populate a scene with already-triangulated geometry by supplying
`Geometry::Triangle` primitives; this crate does not load CAD, STEP, or STL files.
Spheres, boxes and finite capped cylinders are provided for fast fixture/arm proxies. The crate
does not claim that a cheap camera is micron metrology: pixel footprint,
baseline angle, speckle floor, occlusion and calibration drift remain visible in
each sample's covariance and failure reason. In particular, a shallow baseline
or coarse macro image can be tens to hundreds of micrometres uncertain, which is
material for 0.10 mm-module gears.

## Reference integration boundary

`pipe_sim` currently supplies active and placed parts as sphere proxies and samples
them from distinct camera indices. Successful ray returns gate availability,
confidence, range bias and covariance; neighboring samples are averaged per camera,
then distinct views are information-fused with a correlated floor. They do not themselves solve component pose.
The observation starts from a synthetic latent component pose error, and nominal
feature size is used only as a lever arm that converts position uncertainty to
orientation uncertainty. Image-to-CAD feature detection and 6D pose estimation remain
future work. The runtime also applies its camera numbers in a commanded
active-component-local frame rather than ingesting fixed tube/wrist transforms from CAD.
