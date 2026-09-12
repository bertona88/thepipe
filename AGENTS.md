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

## Current integration focus

The common design problem is one optically observed gear-on-shaft operation:
acquire, carry, align the bore and shaft, seat under guarded contact, release,
withdraw, and verify. Start with one active arm and a fixed fixture. Keep the
complete executed and functionally checked gearbox as the destination.

- Use the existing 160 mm ID machine for integration unless an explicit co-design
  decision changes it. Keep the 100 mm optical candidate separate; do not transfer
  precision claims between configurations.
- Use the gear observability experiment as the starting evidence, not an assembly
  success. State how its coupon represents the reference gearbox and which
  clearance, grasp, visibility, and release difficulties it omits.
- Resolve distal-tool clearance together with grasp, marker supports, optical
  views, seating, release, and withdrawal. Change the tool or fixture when needed.
  Refine conservative collision envelopes only when physical geometry justifies
  it; never suppress relevant checks to obtain a pass.
- Close the loop through the authoritative runtime and a common observation
  boundary: observe, assess uncertainty and clearance, move a bounded distance,
  observe again, assess contact and retention. Reuse guards and replay machinery.
  Synthetic and eventual image-derived detections must preserve the same contract,
  measured versus inferred provenance, and genuinely unobservable degrees of freedom.
- Optimize observations for relative mating geometry. Keep point/TCP/feature RMS,
  relative uncertainty, bias, tilt, motion after observation, and held-part error
  distinct. Support any shared-error cancellation with covariance evidence.
  Structured light and dense reconstruction must resolve a demonstrated task need;
  neither is a prerequisite for this operation.
- Use focused bench measurements of the same proposed arrangement to replace
  consequential assumptions. Preserve held-out geometric verification and the
  evaluation-only measurement path. Include mounts, triggering, illumination,
  calibration equipment, cabling, and tools in the buildable cost accounting.
- The lead engineer coordinating the operation is its integration owner and is
  accountable for configuration consistency across contributions. Each change
  should explain what was learned, what design/model changed, and what important
  assumption remains untested. Use existing reports and the engineering inspector.
- Report completed operations separately from controlled refusals. Preserve
  regression demonstrations; defer broader tool libraries, multi-arm choreography,
  presentation, and added physics unless they address this operation's limiting
  uncertainty. Do not add a prescribed milestone plan.

Current evidence and open constraints are in
[docs/GEAR_OBSERVABILITY.md](docs/GEAR_OBSERVABILITY.md) and
[docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md).

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
