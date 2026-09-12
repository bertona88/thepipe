# Scaling laws and their limits

Let s be the new-to-old linear size ratio. The following comparisons assume geometric similarity and unchanged material properties unless otherwise stated. They describe an idealised mechanism, not the full selectively scaled machine.

| Quantity | Ideal scaling | At s = 1/7 |
| --- | --- | --- |
| Length | s | 1/7 |
| Area | s² | 1/49 |
| Mass at fixed density | s³ | 1/343 |
| Rotational inertia | s⁵ | 1/16,807 |
| Thermal length change at fixed α and ΔT | s | 1/7 |
| Bending stiffness k of a geometrically similar beam | s | 1/7 |
| Absolute self-weight cantilever deflection | s² | 1/49 |
| Ideal structural natural frequency | 1/s | 7× |
| Surface-to-volume ratio | 1/s | 7× |

The beam statements follow from I ∝ L⁴ and k ∝ EI/L³. For self-weight, the distributed load scales with cross-sectional area, giving the stated absolute deflection scaling. The frequency estimate follows from √(k/m). Damping, tendon lengths, external actuator dynamics and fluid loading do not necessarily scale with the arm.

## What becomes easier

Less moving mass and inertia can reduce actuation effort and kinetic energy for a given tip speed. Smaller critical lengths reduce absolute thermal expansion for a given material and temperature change. Geometrically similar structures sag less under their own weight. Slender tools may reduce local occlusion.

These are opportunities, not automatic performance multipliers. A tiny arm can still be dominated by tendon preload or an external tube. A deliberate high-compliance design can have a lower useful bandwidth than a stiffer scaled version.

## What does not disappear

For the same imposed tip contact force, a uniformly scaled beam has lower absolute stiffness, so its deflection can increase. For a stress-limited similar structure, useful force scales approximately with area. The part's allowable contact stress and the tool's strain limit matter more than the arm's low weight.

External motors can deliver substantial force through a light tendon. At the same speed, distal kinetic energy falls with mass, while stored tendon and flexure energy depends on their stiffness and deformation. Low mass is therefore not a sufficient collision-safety argument.

## Surface forces relative to gravity

Weight scales with L³. Some idealised capillary and curved-contact adhesion models have force terms proportional to L; pressure over a similar sealed area scales with L² at fixed pressure difference. The corresponding ratios to weight can scale as 1/L² or 1/L.

These are model-specific comparisons. Meniscus volume, contact angle, surface roughness, charge distribution and deformation can break geometric similarity. Electrostatic force has no universal exponent applicable to every charging condition. Do not assign all adhesion one L or L² rule.

## Why selective scaling is attractive

The distal tool can gain the mass and access advantages while the camera baseline, external sensors and actuators retain useful dimensions. This creates a mixed-scale machine whose transmission, support and environment must be evaluated at their actual sizes. A macro demonstrator therefore preserves topology more readily than it preserves microcontact physics.

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
