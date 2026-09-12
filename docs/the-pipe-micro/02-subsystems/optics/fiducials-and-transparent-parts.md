# Fiducials and transparent parts

The user raised an important distinction: the machine's arms are under our design control, but the handled parts may be smooth and transparent. A good concept should support both designed-for-observation parts and parts whose functional surfaces cannot be changed.

## Printed features on tools

A rigid cluster of distinguishable features can identify a tool and provide pose information. Features near a jaw can reveal opening, while a separate cluster on the arm distinguishes whole-body motion from local deformation. Asymmetry can help avoid rotational ambiguity.

Relief marks, edges, holes and patterned silhouettes can all be candidates. Calling a mark a QR code does not establish that the camera can resolve or decode it at the needed viewpoint and magnification. A simpler geometric constellation may supply more reliable information.

## Features on parts

When the part design is controllable, handling tabs or nonfunctional regions can carry fiducials. A transparent gear might have an asymmetric hub feature while leaving teeth and bearing surfaces unchanged. A removable handling feature is also possible, but removal becomes an additional physical operation with debris and force consequences.

The same feature should not silently be treated as optically neutral and mechanically neutral. Surface texture can change adhesion, stress concentration, balance and fit.

## Observing unmodified surfaces

Candidate cues include silhouette, known shape constraints, multiple illumination directions and refractive or reflective features calibrated for the imaging setup. A transparent edge can be visible without providing an unambiguous point on the true surface. Model-based pose estimation must retain that uncertainty rather than treating every image feature as a direct surface measurement.

## Evidence for separation

Tool and object markers moving together can suggest retention; they do not by themselves establish a force margin. A visible gap from an appropriate view, combined with independent receiver support and bounded motion, can support a release conclusion. A liquid bridge or very fine connection may remain below optical resolution, so verification is always tied to the qualified measurement capability.

See [verified handoff](../control/verified-handoff.md) and [optical sensing opportunities](../../04-2pp-opportunities/optical-and-mechanical-sensing.md).

---

[Start](../../README.md) · [Parent folder](README.md) · [Complete directory](../../DIRECTORY.md)
