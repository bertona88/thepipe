# External optics and local geometric truth

**Chosen direction:** retain useful external metrology scale while reducing the mechanisms that enter the scene.

The optical system observes tools, parts, fixtures and their changing relationships. The inherited ambition is 3–5 µm RMS local 3D position accuracy under calibrated conditions. This atlas does not claim that value for the redesigned machine. Smaller arms may help visibility, while smaller features, new windows and changed working distance create a new measurement problem.

## Large enough to be useful

In an ideal rectified stereo model, depth uncertainty is approximately σ_Z ≈ Z²σ_d/(fB), with f and disparity uncertainty σ_d expressed in pixels, and Z and baseline B in consistent length units. This explains why preserving useful baseline is attractive. It does not mean the largest possible baseline is always best: common field of view, surface visibility, optical resolution and correspondence constrain the geometry.

The cell diameter does not itself determine the camera baseline. Lens working distance, magnification, numerical aperture, depth of field, illumination and window design must agree about the same working volume.

## What must be observable

Useful measurements include tool pose, jaw state, object pose, fixture pose, local flexure deformation where it is used as a signal, and separation between donor and object. These can require different views. Angular uncertainty matters when placing a small gear or inserting a shaft even if the object's centre is well located.

## Illumination as a design variable

Controlled illumination is necessary; structured light is an option. Passive multi-view imaging, silhouette information, transmitted light, directional illumination and fiducials are candidate tools. Projected texture does not automatically solve transparent or specular surfaces because the light can transmit or reflect away from the assumed surface.

## The chamber is in the optical path

Curved walls can introduce refractive effects and distortions. Flat windows or purpose-designed ports are candidates. Calibration should include the actual optical path, relevant temperature range and the usable depth region. Observability must survive the presence of the arms, receiving fixture and part together.

The complementary document on [fiducials and transparent parts](fiducials-and-transparent-parts.md) describes how printing can improve the scene rather than demanding the cameras solve arbitrary geometry unaided.

## Browse this folder

- [Fiducials and transparent parts](fiducials-and-transparent-parts.md)

---

[Start](../../README.md) · [Parent folder](../README.md) · [Complete directory](../../DIRECTORY.md)
