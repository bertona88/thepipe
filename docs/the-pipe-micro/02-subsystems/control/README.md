# Observed cooperative control

**Chosen principle:** external observation supplies geometric truth; inexpensive mechanics supply motion and controlled interaction. This describes desired behaviour without selecting a software implementation.

The system acts on an estimate of tool, object and fixture relationships. It also has actuator state, transmission state, environment observations and uncertainty. The object can be in contact with several bodies at once, so a single “owner” label is not a complete physical state.

## Nested physical responsibilities

Transport brings a local workspace into reach. Tendon actuation changes the arm and tool. Optical feedback corrects observed geometry. Compliance handles bounded interaction. A manipulation procedure decides whether enough evidence exists to acquire, carry, regrasp, insert or withdraw.

Those responsibilities are connected: a base move may change tendon length, jaw loading can deform a marker frame, and a humidity transition may invalidate an empirical release model. The controller should use the current operating conditions rather than assuming these channels are independent.

## Facts, estimates and commands

“Jaw opening commanded” is a command. “Jaw gap observed” is an estimate. “Part follows receiver under the qualified verification motion” is retention evidence. “Donor separated and destination pose maintained” is evidence of a completed release. They should remain distinguishable in any implementation or visualisation.

The desired deterministic behaviour is reproducible state evolution given declared inputs and model assumptions. Simulated contact outcomes are useful only within those assumptions. A rigid attachment model can demonstrate sequencing, while actual release behaviour requires a richer model or measurements.

## Useful abstraction

A manipulation can be described by the intended geometric relation, the current supporting interfaces, the permitted load/motion envelope and the observation needed to advance. This leaves room for mechanical, vacuum and capillary tools to share high-level coordination while preserving their different physical conditions.

The [handoff document](verified-handoff.md) gives the central example; [collision and recovery](collision-and-recovery.md) covers interruptions and unexpected contact.

## Browse this folder

- [Collision, disturbance and recovery](collision-and-recovery.md)
- [Verified handoff and release](verified-handoff.md)

---

[Start](../../README.md) · [Parent folder](../README.md) · [Complete directory](../../DIRECTORY.md)
