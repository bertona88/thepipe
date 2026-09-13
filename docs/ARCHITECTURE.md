# One active Pipe µ architecture

`pipe_micro.contracts` admits only the versioned near-wall machine and native-resin
transfer operation. Required fields have no mechanical defaults. Independent enclosure,
shoulder, transport, flexure, contact, fixture, optical and environment records define
the same candidate. Schema changes break admission explicitly.

`plant` owns physical state: base position, bounded tool deformation, jaw opening,
quasistatic tendon tension/energy, specimen position, support set and environment.
It executes bounded commands. Contact support uses declared seeded capacity/pull-off
bounds, without adding a second adhesion force. A rigid retention shortcut is labeled
synthetic; it cannot qualify real retention under load. Loaded dual-support movement
is refused. Stop freezes the modeled mechanisms while preserving their current load;
this response is recorded, but may be unsafe on uncharacterized physical hardware.

`sensor` alone converts plant state into synthetic observation records. Independent
part/tool centroid samples, covariance, ray-box visibility, timing and configuration
identity cross the boundary. Its contact evidence is explicitly synthetic. `controller`
imports neither plant nor sensor and cannot inspect evaluation truth. It guards every
phase and separately records commanded release, separation and retention.

`runtime` coordinates fixed steps and records pre/post physical state, observation,
command, responsibility, geometry and events. Reports include their complete machine
and operation and a deterministic digest. Replay reexecutes the operation and rejects
any changed observation, geometry, outcome or incomplete provenance.

`geometry` supplies conservative boxes to collision admission, optical ray tests and
`cad/export_micro.py`. The CAD export is an inspection representation of an executed
state. It is not BREP fabrication geometry, a full tendon route, or a continuum solution.
Discrete collision checks do not prove continuous swept-solid clearance. Shared frame
consistency across sampled states is tested; broader physical parity remains unverified.

The Rust `pipe_sim_core` now exposes only units and math. `pipe_optics` retains its
independent metrology library/tests with their own domain. Neither library is linked to
the active Python operation. The former planner, simulation, CLI, WASM and machine CAD
are removed, not quarantined as a supported alternate product. No legacy loader is called.
