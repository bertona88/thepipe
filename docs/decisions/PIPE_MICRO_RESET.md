# Adopt Pipe µ as the sole active architecture

Adopted 2026-09-12 from the user's instruction to implement
`pipe-micro-project-reset-plan.md`. The full supplied plan is retained in this folder.
The old gearbox goal, dimensions, nominal forces, serial joints, defaults and
qualification claims are retired. Historical reference: commit
`4ccbd72c954fa8dd699b43b1eb900428942cfdbb`, tag `legacy/pre-pipe-micro-20260912`.
Atlas source: `f99bfeba5626c70689dd229540093e14272f3ac2`.

The authority chain is explicit user direction/adopted decisions, Pipe µ
requirements, embodiment/operation contracts, implementation, then scoped evidence.
The design atlas preserves alternatives; illustrative numbers are not requirements.

The active executable is now a dependency-free Python fixed-step experimental
runtime. This is an intentional implementation choice, not a mechanical requirement.
The old Rust runtime's serial mechanism and owner-based attachments are removed.
Independent Rust SI/math and optics libraries remain separately tested studies;
they are not imported by this operation. Their historical baseline constructors
cannot populate Pipe µ data. The WASM product API is retired, not silently emulated.
A future native port must reproduce the new records and pass the same guards.

Repository authority and default execution can be cut over before physical
qualification. This change implements a synthetic cooperative operation with
explicit limits; it does not complete P3 physical feasibility, P4 prototypes, P6
calibrated camera reconstruction, full P7 service integration, or P9 qualification.
The remaining work is recorded in IMPLEMENTATION_STATUS, without legacy fallbacks.

Recoverable history preserves original code and results. Archived evidence retains
its original bytes, configuration identity and limitations. No old passing result
is Pipe µ qualification. Reintroducing a retired assumption requires a new decision.
