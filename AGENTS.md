# The Pipe engineering rules

## Goal and way of working

Build a low-cost tube-shaped micro-assembly cell whose precision comes primarily
from optical observation, local feedback, and guarded contact. The reference
demonstration is assembling and checking a miniature gearbox with mobile
tendon-driven arms. Gearbox parts are idealized; the cell is the design problem.

Work on one evolving machine. CAD, mechanics, optics, controls, and software are
different views of that machine, and changes should be considered across them.

- Design by articulating and observing the proposed mechanism. Use motion,
  clearance, contact, and visibility studies to revise geometry as well as control.
  Rough CAD and reduced models are useful when their assumptions are clear.
- Keep runtime dimensions, joints, frames, tools, and sensor mounts connected to
  the intended mechanical design. Reuse shared definitions where practical, and
  call out discrepancies and their consequences instead of letting models drift.
- Use small operations such as a peg insertion as integrated experiments. Explain
  what each result establishes and what its approximations leave unanswered about
  the complete machine.
- Let simulation and physical measurements inform each other. Treat unmeasured
  performance as a hypothesis, and revise the design or model when evidence disagrees.
- Keep documentation focused on the goal, current decisions, evidence, and open
  questions. Choose work around the uncertainty that matters to the machine;
  keep planning lightweight rather than accumulating prescribed milestone sequences.

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
