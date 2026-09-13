# Subsystems

These documents describe what each subsystem contributes and the design freedom around it. They are connected by physical relationships, rather than by a repository structure.

| Subsystem | Contribution | Most consequential connection |
| --- | --- | --- |
| [Transport](transport/README.md) | Places shoulders around and along the cell | Support geometry must make the distal workspace reachable |
| [Flexure arms](flexure-arms/README.md) | Local motion and designed compliance | Tendon preload, useful strain and tip observability |
| [Tendons](tendons/README.md) | Transfers effort from external motors | Routing friction, elastic stretch and carriage loads |
| [End effectors](end-effectors/README.md) | Acquires, holds and releases objects | Contact geometry, part properties and receiving support |
| [Optics](optics/README.md) | Observes actual geometry and relationships | Windows, occlusion, transparent objects and calibration |
| [Environment](environment/README.md) | Makes contact conditions more reproducible | Charge, moisture, particles and flow disturbance |
| [Control](control/README.md) | Coordinates observed manipulation | Retention evidence and safe state transitions |
| [Fabrication](fabrication/README.md) | Makes integrated small geometry practical | Process access, actual material behaviour and inspectability |

The same printed feature may serve several roles: a flexure can be a joint and a force indicator; a fingertip can include contact mesas, an optical marker and a fluid aperture. Integration is useful when each function remains understandable and verifiable.

## Browse this folder

- [Observed cooperative control](control/README.md)
- [End effectors — interfaces with selectable behaviour](end-effectors/README.md)
- [A controlled assembly chamber](environment/README.md)
- [Fabrication — use resolution where function needs it](fabrication/README.md)
- [Flexure arms](flexure-arms/README.md)
- [External optics and local geometric truth](optics/README.md)
- [Remote tendon actuation](tendons/README.md)
- [Transport and shoulder support](transport/README.md)

---

[Start](../README.md) · [Complete directory](../DIRECTORY.md)
