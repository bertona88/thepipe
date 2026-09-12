# Sharing the interior

Smaller arms may create room for more viewpoints and more cooperating tools. That benefit depends on the whole geometry: shoulder supports, tendons, channels, fixtures and windows can still occupy much of the scene.

## Organise by task regions

Useful regions include assembly, acquisition, tool parking, inspection, handoff and transfer-chamber access. These are functional regions that may overlap at different times. A regrasp requires a place where two tools can approach the same object, establish independent support and separate without pulling the object through an obstruction.

A station close to the cylinder wall can be perfectly useful if multiple nearby arms and cameras can reach it. A central assembly zone is attractive but is not compulsory. The [reach analysis](scale-and-workspace.md) should govern its location.

## Reserve visible approach paths

A planned manipulation has an approach volume, an interaction volume and a withdrawal volume. A camera may need to see a gap from a side angle to verify separation, even if a frontal image is adequate for positioning. A view that only sees the marker on the arm cannot necessarily establish the relationship between the tip and a transparent component.

Arm posture can be used to improve observation. One arm may park while another establishes a critical contact. A receiver may choose a handling tab that keeps the final locating surfaces visible. This is optical and mechanical co-design at the operation level.

## Include the services

Tendons can cross even when rigid arms do not. A hose loop can block a camera or touch a neighbouring tool. Finite circumferential travel can require unwinding. A carriage order along a shared path can constrain who can reach a station.

Describe these as physical constraints rather than relying on an ideal point moving on a cylinder. The specific controller or collision library is outside this document's scope; the desired behaviour is described in [collision and recovery](../02-subsystems/control/collision-and-recovery.md).

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
