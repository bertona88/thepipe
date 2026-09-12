# Mechanical grippers and supported release

A printed compliant tweezer is a natural first tool because geometry, contact and actuation can be inspected directly. Its useful behaviour does not depend on a particular jaw count or symmetry.

## Grasp geometry

Opposed jaws can apply normal loading to create tangential holding capacity. Concave or shaped contacts can also supply partial form constraint, reducing dependence on high friction. A handling tab, recess or edge can make object orientation more observable and establish a reproducible grip location.

Normal force must stay compatible with the part and the contact geometry. Sparse mesas concentrate load. Thin gears, compliant components or polished surfaces may be damaged well before a generic “grip strength” limit is reached.

## Opening paths are a design variable

Symmetric opening may leave a component attached to either jaw. Candidate alternatives include one jaw opening first, a jaw rotating through a peel trajectory, or a shaped receiving fixture holding the part while the jaw withdraws.

A mechanical ejector could move relative to the donor contact, gently encouraging detachment while the receiver provides support. It introduces a third contact whose own adhesion and motion need consideration. An ejector is useful only if its load and withdrawal do not create a new uncontrolled attachment.

## Compliance placement

The gripping spring can establish a predictable relationship between deformation and load over a measured range. A separate compliant wrist can accommodate alignment errors. Combining them saves parts but may make jaw force harder to infer because wrist deformation, grip closure and tendon friction become coupled.

## Role of the other arms

The receiver need not overpower the donor with a larger squeeze. It can constrain the part while the donor reduces its retention through opening, peeling or a change of load direction. This is especially useful for fragile parts: transfer is a controlled change in interface state, not a contest of peak forces.

Successful release requires evidence that the part remains with its receiver or destination and that the donor has separated. The [handoff description](../control/verified-handoff.md) specifies the conceptual sequence without choosing a controller implementation.

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
