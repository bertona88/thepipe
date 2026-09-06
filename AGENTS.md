# The Pipe engineering rules

Prioritize physical correctness, deterministic simulation, explicit units,
collision safety, testability, and honest fidelity boundaries.

## Visuals must be backed by implemented code

- The legacy website and synthetic UI preview have been removed. Do not restore
  them from old branches or Git history.
- Render machine geometry and motion only from versioned Rust scene output or
  reproducible build123d-generated CAD. Preserve stable IDs, frame conventions,
  units, and configuration identity. CAD geometry is a nominal design model,
  not evidence of executed motion or hardware capability.
- Drive telemetry, sensor/visibility overlays, contacts, grasp state, task
  progress, and pass/fail indicators from the implemented runtime or its recorded
  reports. Any displayed derived quantity must have an explicit, reproducible
  definition grounded in those inputs.
- Missing, stale, malformed, unsupported, or disconnected data must show an
  unavailable/error state. Do not generate substitute readings, animation-driven
  poses, scripted assembly progress, fake sensor views, or success fallbacks.
- Keep simulation truth, observed estimates, and commanded targets visibly
  distinct; never substitute truth for a missing estimate. M1e/M1f estimate data
  currently resides in structured reports, not the general SceneFrame.
- Replays must identify the source revision, scenario/configuration hash, and
  simulation tick/time. Record generation commands and parameters for CAD and
  report artifacts. Clearly label model fidelity and unqualified hardware claims.
- Display interpolation may only interpolate authoritative samples and must be
  labelled. It must never determine physics, contacts, control, task gates, or
  acceptance results.
- Before adding a viewer, verify its data mapping against real runtime output
  and verify that missing/invalid data fails closed. Keep the engineering core
  headless and authoritative.
