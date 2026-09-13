# Release and retention

The part is a mechanical body with contact forces, constraints and moments. It does not literally belong to whichever interface has the largest scalar force. The “strongest engaged interface” phrase is a useful intuition, but actual transfer depends on load direction, torque, compliance, contact history and how the donor is disengaged.

## A workable transfer balance

The receiver should provide enough support for the residual donor force and torque plus disturbances during unloading. The donor should reduce its attachment in a controlled direction. The part should stay within acceptable translation, rotation and contact stress limits throughout the transition.

This is why peel and directional unloading are interesting. Progressive separation can change the active contact region and lower the required peak load in a suitable interface. It is not a universal guarantee: pad geometry, stiffness and the part's shape determine the actual release path.

## Retention mechanisms can be combined

A fixture may constrain rotation while a jaw prevents translation. A receiver can support a face while the donor vents a pressure interface. An adhesive pad can hold during travel and peel into a mechanical pocket. These combinations are especially useful when no single interface can provide both secure holding and gentle release alone.

## What “verified” means

Verification is evidence sufficient for the declared operation, under a known measurement capability. It may include observed receiver-relative pose, a donor separation gap and response to a bounded test motion. It cannot mean knowing every molecular interaction from a camera image.

Release probability must be accompanied by final placement error. A tool that detaches reliably by launching the object has not solved controlled assembly. Retention must also cover the intended transport and orientation loads, not merely the ability to lift against gravity.

## Failed release is informative

The object may remain on one jaw, rotate during peel, follow a residual liquid bridge or slip from both interfaces. These are distinct outcomes with different remedies. A surface comparison should preserve them instead of reducing all failures to one boolean.

The [handoff states](../02-subsystems/control/verified-handoff.md) connect this physical picture to coordination. The [coupon atlas](../05-evidence/coupon-atlas.md) connects it to measurements.

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
