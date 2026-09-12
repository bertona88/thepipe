# Printed interfaces inside an arm

2PP makes the internal layout part of the mechanism design. An arm can include tendon guides, anchors, channels, contact textures and observable markers. The useful question is whether each feature can be fabricated, cleaned, loaded and inspected in its final form.

## Tendon anchoring

Candidate anchors include through-eyes, winding posts, wedge seats, mechanical pockets and accessible tie features. An anchor should spread load into a sufficiently robust region and avoid a sharp edge cutting the fibre. Retensioning and replacement may matter more than a perfectly hidden joint.

The first printed structure can remain one material while using a separately supplied tendon. A tendon captured during or after fabrication is a construction choice, not proof that its strength or friction is suitable.

## Channels through compliant regions

A channel within a link can be attractive. Carrying it through a flexure is more demanding: it removes section material, changes strain distribution, may collapse or kink, and can couple internal pressure into bending. A path near a neutral bending region is a candidate, not a guarantee that these effects disappear.

Alternatives include an external flexible tube crossing the joint or channels confined to rigid sections. The tube adds restoring force and possible occlusion, but may be easier to clean or replace. [Vacuum and fluidics](../end-effectors/vacuum-and-fluidics.md) compares the functional consequences.

## Printed sensing features

Pairs of markers can reveal deflection across a known compliant element. A nearby reference can distinguish local jaw opening from global arm motion. Markers should occupy faces visible during the critical operation, and their geometry must be resolvable by the actual optical system.

## Tool interfaces

A monolithic gripper is the simplest structural starting point. Later replaceable fingertips or tool cartridges could use keyed seats, compliant clips or preload features. The dock introduces another compliance and contamination interface, so its seated pose and retention need observation.

## Access is part of the geometry

Printing an enclosed lumen is not the same as producing a usable open channel. Uncured material needs an exit path, wash fluid needs access, and thin parts need a practical release procedure. The [fabrication documents](../fabrication/README.md) keep these operations attached to the design.

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
