# Where the discussion's ideas live

The supplied conversation is the primary source for this atlas. This map preserves its intent and distinguishes it from engineering elaboration.

| Discussion idea or preference | Where it is developed | Treatment |
| --- | --- | --- |
| Concern that integration into the current Pipe could lose the spirit | [Vision](../00-vision/README.md) and [architecture](../01-architecture/README.md) | Standalone concept; no migration plan |
| Big optics with tiny manipulators | [Scale and workspace](../01-architecture/scale-and-workspace.md) | Central architectural principle |
| Flexures as the exciting arm direction | [Flexure arms](../02-subsystems/flexure-arms/README.md) | Chosen starting architecture |
| Cylinder inspired by O'Neill cylinders | [Vision](../00-vision/README.md) | Spatial inspiration, no artificial-gravity mechanism |
| Rail provides axial shift and motion around z | [Mobility](../01-architecture/mobility-and-reach.md) | z/θ roles preserved; tool roll distinguished |
| Little experience with literal rails | [Transport options](../02-subsystems/transport/mechanism-options.md) | Mechanism remains open |
| All motors outside | [Tendons](../02-subsystems/tendons/README.md) | External actuation direction |
| Prefer no metals inside | [Design choices](../00-vision/design-choices.md) and [environment](../02-subsystems/environment/README.md) | Preserved as a preference with electrical implications |
| Seal the cell and introduce objects through chambers | [Contamination and load locks](../02-subsystems/environment/contamination-and-load-locks.md) | Functional chamber concept |
| Use humidity cycles to address static | [Humidity and electrostatics](../02-subsystems/environment/humidity-and-electrostatics.md) | Preserved exploration, no invented recipe |
| Very smooth surfaces as well as strong texturing; 2GL mentioned | [Contact microarchitecture](../02-subsystems/end-effectors/contact-microarchitecture.md) | Both branches retained; no unprovided 2GL process specification assumed |
| Gecko feet and anti-adhesion surfaces | [Directional adhesion](../04-2pp-opportunities/directional-adhesion.md) | Distinct adhesive and low-contact branches |
| Use capillary effects deliberately | [Wet tools](../04-2pp-opportunities/wet-tools-and-capillary-transfer.md) | Exploratory tool family |
| Do not rely on gravity for release | [Release physics](../03-physics/release-and-retention.md) and [handoff](../02-subsystems/control/verified-handoff.md) | Cooperative support, unloading and separation evidence |
| Vacuum and channels in the arms, especially with flexures | [Vacuum](../02-subsystems/end-effectors/vacuum-and-fluidics.md) and [printed interfaces](../02-subsystems/flexure-arms/printed-interfaces.md) | Candidate with channel-through-flexure question explicit |
| First arm should probably use one material | [Fabrication](../02-subsystems/fabrication/README.md) | One printed structural resin initially |
| First machine can assemble later arms with other materials | [Assembling the next arms](../04-2pp-opportunities/assembling-the-next-arms.md) | Preserved future capability |
| Fluorination interesting, texture first | [Design choices](../00-vision/design-choices.md) | Coatings optional later |
| Shrinkage is not a major concern | [Materials and process](../02-subsystems/fabrication/materials-and-process.md) | Routine geometry compensation, proportionate attention |
| 300 µm–1 mm is a suitable start | [Parameter register](parameter-register.md) | Accepted initial range |
| Prototype printed at macroscale | [Macro and micro prototypes](../05-evidence/macro-and-micro-prototypes.md) | Topology evidence paired with actual micro specimens |
| High-level documents should lead to lower-level details | [DIRECTORY.md](../DIRECTORY.md) | Folder indexes, parent navigation and crosslinks throughout |

## Clarifications introduced while documenting

The atlas makes several physical distinctions more precise than the informal conversation: enclosure radius versus shoulder radius; circumferential travel versus independent tool roll; structural mass versus tendon preload; pressure force versus complete vacuum retention; receiving contact versus measured retention; and logical ownership versus actual donor separation.

These clarifications support the concept. They do not constitute newly selected mechanisms or a requirement to integrate the idea into the current codebase.

---

[Start](../README.md) · [Parent folder](README.md) · [Complete directory](../DIRECTORY.md)
