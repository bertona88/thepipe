# Prepared publication and backlog changes

Status: local implementation committed; no remote branch, tag, PR or issue changes
were completed in this session. Automatic approval review rejected publishing the
branch and legacy tag because it interpreted the request as local implementation.
The changes below are prepared for explicit publication approval.

## Pull request

Title: Reset The Pipe to Pipe µ with an isolated synthetic cooperative operation

The old gearbox architecture supplied machine defaults, acceptance criteria and
attachment semantics incompatible with the adopted Pipe µ direction. This change
replaces active authority, executable selection, scenarios, CAD export and CI with
one explicit Pipe µ candidate and operation; original history and evidence remain
recoverable and distinctly historical.

The active experimental runtime is dependency-free Python. Independent Rust SI/math
and optics libraries remain tested separately; legacy planner, machine, CLI and WASM
paths are removed. The new operation executes bounded shared z/θ positioning, pickup,
transfer, dual support, unloading, separation verification, withdrawal and independent
part inspection. It records physical state, synthetic observations and control state
separately. One configuration drives reduced collision geometry, ray tests and CAD.

Validation: 11 local tests, nominal completion, 14 specific fault refusals with executed
stop, deterministic replay/tamper checks and multi-state CAD envelope correspondence.
The clean-source evidence matrix is docs/evidence/pipe_micro_v1.json. Rust tooling was
unavailable locally; the independent Rust CI job remains to run after publication.

Limits: this is a synthetic translational/contact model with assumed loads and rigid
retention, not a qualified physical machine. Resin selection, fabrication-ready CAD,
full service routing, continuous deformed-solid clearance, calibrated camera observations
and P4/P9 hardware evidence remain open. The detailed candidate and status identify
these gaps. Path/symbol/parameter/test dispositions record removed contracts and
partial property coverage. No legacy defaults enter the new runtime.

## Existing work dispositions

- Close PR #12 as superseded by the adopted Pipe µ reset, not completed.
- Close draft PR #17 as superseded after selective property review; preserve its branch
  and evidence. No wholesale merge. Generic properties recovered are documented in
  migration/README.md; calibrated surface-marker integration remains open.
- Close issues #13–#16 with state reason `not_planned`, explicitly superseded by the
  following Pipe µ work. Do not mark their gearbox deliverables completed.

## Replacement issue: qualify the shared physical candidate

Select and record one resin/process, native surface and specimen identity. Resolve the
near-wall supports, flexure geometry, tendon anchors/guides, bounded ring motion, service
routing, chamber and transfer access against the same task poses. Compare reduced
geometry with fabrication solids across articulated/deformed states; check complete
motion sweeps and view occlusion. The present envelope model does not close this gate.
Acceptance: one jointly reachable, collision-admissible, observable and service-feasible
candidate with explicit uncertainty and unmeasured assumptions. Supersedes geometry
properties from #13/#14 without retaining their machine dimensions.

## Replacement issue: build and measure targeted Pipe µ prototypes

Fabricate an enlarged routing/assembly prototype and actual-scale native-resin flexure
and contact coupons once process identity is selected. Record all nonuniform scaling.
Measure relevant travel, directional stiffness, dwell-dependent creep/hysteresis,
anchor behavior and pickup/retention/release under logged conditions. Compare smooth
pads with one useful geometry variant. Record failures separately and replace only
assumptions supported in the tested domain. No prototype or measurement is claimed yet.

## Replacement issue: integrate measured contact and optical evidence

Replace synthetic contact-state evidence and centroid generation with candidate-bound
observation interfaces that retain part/tool independence, covariance, age, calibration,
source identity and unobservable axes. Observe separation and verify retention without
ownership inference. Validate material-specific support-dependent stop/recovery behavior,
including receiver slip, donor adhesion and residual bridges. Preserve current fault
refusals and explicitly requalify any changed thresholds. Supersedes useful #15 properties.

## Replacement issue: qualify the actual-scale cooperative operation

After geometry and measured models support it, execute the complete bounded z/θ and
cooperative operation in the actual environment. Predeclare acceptance/uncertainty limits;
record repeated nominal and failure outcomes, process/environment/wear history and
independent final pose/separation measurements. Keep synthetic, calibrated and hardware
claims separate. Replays must preserve exact source/configuration identity and reject
corrupt/missing data. This remains open P9 work, retaining useful #16 evidence properties.
