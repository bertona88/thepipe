# Joint and arm families

## Candidate mechanisms

| Family | Useful behaviour | Principal tradeoff |
| --- | --- | --- |
| Leaf or notch flexure | Local bending in a preferred direction | Concentrated strain and parasitic translation |
| Crossed or paired leaves | More controlled rotational behaviour | Additional geometry and possible interference through travel |
| Parallel compliant linkage | Guided translation or jaw symmetry | Finite travel and sensitivity to link/flexure matching |
| Serial flexure chain | Compact local pose adjustment | Tendon coupling, cumulative compliance and limited orientation reach |
| Continuum or distributed compliant arm | Smooth curvature and obstacle accommodation | Shape estimation and coupled actuation become more important |
| Compliant wrist on a simpler positioner | Local accommodation at the interaction | Useful orientation and force response remain bounded |
| Bistable element | Two stable configurations or holding without continuous effort | Snap-through can inject energy into the part |

None of these is the selected arm CAD. They describe a vocabulary from which an arm can be designed.

## Small beam intuition

For an ideal slender rectangular beam bending in its thickness direction, I = bt³/12. With a cantilever approximation, small-deflection transverse stiffness is k ≈ 3EI/L³. Thickness is therefore a strong lever: halving t reduces this model's stiffness by a factor of eight.

For approximately uniform bending curvature, outer-fibre strain is ε ≈ tκ/2. If a bend angle φ is distributed over length L, κ ≈ φ/L gives ε ≈ tφ/(2L). These formulas are screening relations, not a validation of a large-deflection printed hinge. Notches, channels, print orientation and stress concentration change the result.

## Which stiffness matters

The useful question is directional. A gripper can be stiff in transport shear, compliant in a seating direction and easy to peel away along a release motion. Compliance that helps insertion may make optical force inference possible. Compliance in the wrong direction can make a tendon pull rotate the part unexpectedly.

## Stops and fail behaviour

A printed stop can limit travel, but contact with the stop creates its own high-stress or adhesive interface. A passive return can open a jaw when tension is lost; that may be desirable for avoiding overloading yet undesirable when a part would become unsupported. A holding latch has the opposite tradeoff. There is no universal safest relaxed posture independent of the operation.

The [control documents](../control/README.md) treat tension loss and uncertain retention as explicit physical situations rather than assuming a generic spring return always resolves them.

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
