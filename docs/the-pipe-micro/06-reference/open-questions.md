# Open questions

This is a map of design freedom, not a ranked backlog.

| Question | Why the answer matters | Detailed home |
| --- | --- | --- |
| Where do shoulders sit relative to the working region? | Small distal reach must close the physical gap | [Scale and workspace](../01-architecture/scale-and-workspace.md) |
| Which base motions are independent between arms? | A shared ring couples arm locations | [Mobility](../01-architecture/mobility-and-reach.md) |
| What actually guides and constrains each base? | Cable effort alone does not define all unwanted motion | [Transport options](../02-subsystems/transport/mechanism-options.md) |
| How much circumferential travel is useful? | Services can wrap or constrain passage | [Tendon routing](../02-subsystems/tendons/routing-and-preload.md) |
| Which local tool poses does the first operation need? | Determines arm topology and role of regrasp | [Arm families](../02-subsystems/flexure-arms/joint-and-arm-families.md) |
| Where should compliance live? | Controls load response, observability and correction range | [Flexure arms](../02-subsystems/flexure-arms/README.md) |
| How is a tendon attached and retensioned? | Anchor strength, installation and creep affect function | [Printed interfaces](../02-subsystems/flexure-arms/printed-interfaces.md) |
| Which texture improves the actual material pair? | Geometry can increase or decrease adhesion | [Contact microarchitecture](../02-subsystems/end-effectors/contact-microarchitecture.md) |
| Which release direction is least disturbing? | Peel, opening and venting produce different loads | [Release physics](../03-physics/release-and-retention.md) |
| Does a useful vacuum seal survive contamination? | Ideal pressure force alone does not establish a tool | [Vacuum](../02-subsystems/end-effectors/vacuum-and-fluidics.md) |
| Can a channel remain functional through bending? | Links geometry, process and pressure response | [Integrated mechanisms](../04-2pp-opportunities/integrated-and-fluidic-mechanisms.md) |
| What humidity policy improves manipulation? | Charge, wetting and polymer response are coupled | [Humidity](../02-subsystems/environment/humidity-and-electrostatics.md) |
| What does a transfer chamber need to control? | Incoming contamination and disturbances affect the working region | [Load locks](../02-subsystems/environment/contamination-and-load-locks.md) |
| Which tool/part features remain observable in contact? | Local accuracy and release verification require usable views | [Fiducials](../02-subsystems/optics/fiducials-and-transparent-parts.md) |
| What receiving support proves sufficient retention? | Avoids confusing contact with successful handoff | [Verified handoff](../02-subsystems/control/verified-handoff.md) |
| What is the suitable hold state after a fault? | Passive return may release a part or reduce a damaging load | [Recovery](../02-subsystems/control/collision-and-recovery.md) |
| Which process makes each feature usefully? | Resolution, throughput and cleaning jointly determine practicality | [Fabrication](../02-subsystems/fabrication/README.md) |
| Which later arm component could the first machine assemble? | Turns the bootstrapping idea into a concrete operation | [Successor arms](../04-2pp-opportunities/assembling-the-next-arms.md) |

An answer can be a deliberate simplification. Limited travel, one contact family, a particular fixture or one stable environmental condition can all be coherent embodiments if their limits are explicit. The full concept remains broader than any one such embodiment.

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
