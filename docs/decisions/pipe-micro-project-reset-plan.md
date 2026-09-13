# The Pipe µ — project reset and migration plan

**Decision proposed:** Make The Pipe µ the only active product architecture. Preserve reusable engineering infrastructure, but remove the old gearbox machine as the source of defaults, requirements, success criteria, and implementation constraints.

**Audit date:** 12 September 2026.

**Repository inspected:** `bertona88/thepipe`. The design-atlas branch was at `f99bfeba5626c70689dd229540093e14272f3ac2`; its base and current `main` were `4ccbd72c954fa8dd699b43b1eb900428942cfdbb`. The branch comparison showed one documentation-only commit adding the atlas. Open PR #12, draft PR #17, and open issues #13–#16 were also inspected. This is a static planning audit of the central design documents, selected implementation interfaces, repository structure, CAD documentation, configuration, and CI—not a completed line-by-line audit or a newly executed test campaign. No repository files, branches, issues, or pull requests have been changed.

## 1. Executive recommendation

Do not introduce a `micro` mode into a gearbox-led project. Perform a controlled architectural reset within the existing repository.

The reusable asset is the engineering discipline: explicit units and frames, observation-only control, guarded motion, reproducible simulation, independently attributable evidence, and honest visualization. The old machine's dimensions, mechanical topology, nominal performance, benchmark, and attachment semantics are not reusable simply because their implementation already exists.

The pivot should satisfy two different completion conditions:

1. **Repository cutover:** the active requirements, dependencies, defaults, CAD, scenarios, reports, and acceptance tests describe Pipe µ. Legacy behavior is no longer reachable as an implicit fallback.
2. **Physical qualification:** a specified Pipe µ embodiment has demonstrated an operation under recorded conditions with measured uncertainty and failure outcomes.

The first can finish before the second. An incomplete but honestly identified Pipe µ implementation is preferable to an apparently complete product that silently executes the previous machine.

## 2. What the audit found

| Contamination source | Observed legacy assumption | Consequence for the migration |
|---|---|---|
| Root authority | `AGENTS.md`, `README.md`, and normative `docs/REQUIREMENTS.md` make gearbox assembly the goal. | Replace the governing contract; adding an atlas underneath it does not change authority. |
| Machine JSON | `machine_baseline_v1.json` fixes four manipulators, a 160 mm-ID tube, 320 mm working length, a 72 mm shoulder radius, and 32/30/15 mm serial links. | Create a new model identity and schema; do not resize this baseline in place. |
| Performance defaults | The baseline includes 1.2 N tendon pretension, 4 N maximum tension, 0.15 N maximum grip force, 5 g payload, and 50 mN tool-force targets. | Retire these numbers. Any new value needs an operation-specific rationale and evidence status. |
| Runtime topology | `machine_config.rs` only accepts `paired_belt_end_bogies`; `machine.rs` embeds `SerialArmConfig` and requires arm reach at least the carriage radius. | Separate transport, shoulder support, distal mechanism, and task-workspace feasibility. |
| CAD | The nominal CAD uses pinned 32/30/15 mm links, four differential capstans, six global cameras, and a wrist macro head. | Preserve CAD tooling, not this assembly as the new starting geometry. |
| Alternative old arm model | `crates/pipe_sim_core/src/arm.rs` is a three-tendon, piecewise constant-curvature continuum model. | It is not automatically a model of a localized printed-flexure arm. Admit it only against a matching mechanism and test domain. |
| Handoff | The existing controller requests atomic ownership transfer, donor opening, and receiver verification using reduced evidence. | Preserve observation guards, but replace physical support and separation semantics. |
| Default execution | `ScenarioSpec::named("nominal")` resolves to the gearbox baseline; `crates/pipe_sim/src/lib.rs` directly imports the gearbox executive. | Remove legacy aliases and dependency paths, not just visible documentation. |
| CI | CI runs gearbox acceptance and named M1c/M1e/M1f/M1g scenarios. | Preserve the tested properties while replacing the old machine's acceptance contracts. |
| Active work | PR #12 reinforces the 160 mm gearbox machine. PR #17 builds an opt-in pickup-return candidate and includes changes to root guidance. | Supersede the old steering and selectively recover useful implementation improvements. |

Source basis: [S1]–[S12].

## 3. The new governing contract

### 3.1 Commitments to preserve

Pipe µ is a cylindrical, controlled microassembly workspace with small printed compliant manipulators, useful external optical infrastructure, remote tendon actuation, and cooperative manipulation. Begin with objects in the 300 µm–1 mm domain. The first printed distal arm/gripper structure uses one identified structural resin and its native surface. Geometry is the first lever for engineering contact and release. The near-manipulation volume has a metal-free design preference; external motors and instrumentation can contain metal.

Preserve longitudinal z and circumferential θ base positioning in the architecture. Preserve small transfer chambers and environmental conditioning. Release is a separately observed outcome, not a synonym for jaw opening or changing a software owner field. [S1, S2, S13]

### 3.2 Choices that must remain explicit and open

| Topic | Status in the atlas | Migration rule |
|---|---|---|
| 500 µm reference object | Illustrative reference | Convenient first specimen, not a universal part size. |
| 10–15 mm distal reach | Architectural reference | Select against required tool poses and strain/travel constraints. |
| 80–120 mm enclosure diameter | Illustrative | Do not turn 100 mm into an unexplained normative requirement. |
| Shoulder position and assembly station | Unresolved | Compare inward supports, internal supports, and near-wall stations. |
| Number of arms and local degrees of freedom | Unselected | Two active hands are a proposed demonstration choice, not the final product count. |
| Shared versus independent base motion | Unselected | Represent coupling honestly; one moving ring is not several independent stages. |
| Transport mechanism | Candidate space | No inherited paired-belt requirement, continuous rotation, or passing assumption. |
| Optical count, modality, and layout | Unselected | Select from operation visibility and uncertainty needs. |
| 3–5 µm local optical accuracy | Inherited ambition | Re-establish for the new scene; do not inherit qualification. |
| Vacuum, wet tools, coatings, directional adhesion | Explorations | Preserve as extension options, not first-machine prerequisites. |
| Humidity cycling | Active idea, not a recipe | Begin with recorded stable conditions unless evidence supports a cycle. |
| Shrinkage | Ordinary fabrication/calibration concern | Measure functional dimensions; do not create an unrelated prerequisite programme. |

Sources: [S1, S2, S14, S15].

### 3.3 Authority and change control

Use a single precedence chain:

**Explicit user direction and adopted decisions → Pipe µ requirements → selected embodiment/operation contracts → configuration and code → evidence about that configuration.**

The atlas preserves intent and design freedom. Code describes what is implemented; it does not acquire the authority to redefine the machine. An old passing test cannot veto an explicitly adopted architectural change.

Write one reset decision record and rewrite the active requirements rather than appending contradictory paragraphs. Keep decision provenance distinct from evidence confidence: a user preference, an engineering elaboration, a numerical example, a candidate, and a measurement are different things.

## 4. Disposition rules

**Keep** means a method or implementation is demonstrably independent of the retired architecture. It does not preserve all the file's defaults.

**Adapt** means a component has reusable structure but may enter the new runtime only after its assumptions and tests have been rewritten.

**Retire/delete from active code** means it must disappear from the active workspace, imports, defaults, public selection paths, and release gates. Git history or a pinned legacy tag preserves recoverability.

**Archive evidence** means retain the original artifact, revision, configuration, commands, and limitations. Do not relabel old results as Pipe µ evidence or rewrite failures to match the new narrative.

Every tracked path should receive a disposition in the migration manifest. Grouped decisions below are the initial audit, not a claim that every call site has already been enumerated.

## 5. Documentation, backlog, and guidance

| Paths or work items | Decision | Required action |
|---|---|---|
| `docs/the-pipe-micro/**` | Keep | Preserve the atlas as intent. Maintain chosen/candidate/illustrative/exploration/measured distinctions. |
| `AGENTS.md` | Rewrite | Make Pipe µ the sole goal; prohibit implicit legacy reuse; retain truth/estimate/command separation, explicit units, fail-closed data handling, reproducibility, and non-fabricated visualization. |
| Root `README.md` | Rewrite | State the new machine, actual implementation status, one active default operation, and current limitations. Move historical walkthroughs out of the main onboarding route. |
| `docs/REQUIREMENTS.md` | Replace active content | Remove old normative dimensions, pin joints, prescribed six pose axes, payload, tools, camera layout, and gearbox success claim. Introduce operation-specific requirements and evidence status. |
| `docs/ARCHITECTURE.md` | Replace active content | Describe transport/support/distal/contact/observation/environment roles and implemented interfaces. Do not retain two competing normative architectures. |
| `docs/IMPLEMENTATION_STATUS.md` | Rewrite | Track Pipe µ capabilities as absent, modeled, empirically calibrated, or hardware-demonstrated, with scope and links. Keep old achievements in a historical record. |
| M1/M1c/M1d/M1e/M1f/M1g and gear-study documents | Extract, then archive | Recover general estimator, safety, replay, and experimental methods. Retire their old machine/milestone authority. |
| `docs/HARDWARE_COUPON_M1E.md` | Selectively adapt | Recover measurement and independent-verification methods; replace hardware, geometry, force, and optical assumptions. Do not make the complete old protocol a prerequisite. |
| `docs/evidence/**` | Preserve provenance | Separate historical evidence from active qualification. Keep failure records and exact identity. Use a short archive index rather than duplicating the entire old corpus into active agent context. |
| PR #12 | Supersede | Close as superseded, not completed. Its integrated-engineering discipline can be adopted without its 160 mm gear-on-shaft direction. |
| Draft PR #17 | Split/salvage | Do not merge wholesale. Review generic improvements to tool frames, supported geometry, independent observations, executed stop behavior, release authorization, and replay integrity for separate adoption. Retire the old scenario and copied steering. |
| Issues #13–#16 | Supersede and replace | Preserve useful acceptance properties in new Pipe µ issues. Do not mark old deliverables complete just because the product direction changed. |

Sources: [S3, S4, S10–S12, S16].

## 6. Runtime and software disposition

Paths below are relative to the repository root.

| Component | Decision | New contract / removal requirement |
|---|---|---|
| `crates/pipe_sim_core/src/{units,math}.rs` | Keep after unit review | Explicit units, vectors, poses, transforms, numerical validity; no machine dimensions in generic constructors. |
| `crates/pipe_sim_core/src/{geometry,collision}.rs` | Keep primitives; adapt coverage | Include deformed-arm envelopes, full jaws and supports, carried objects, fixtures, shoulders, carriages, and relevant services. Keep intended contact narrowly identified rather than disabling collision classes. |
| `crates/pipe_sim_core/src/actuator.rs` | Adapt | Retain rate/saturation/state concepts. Re-establish tendon routing, pretension, compliance, slack, friction, and stored-energy behavior for the selected embodiment. |
| `crates/pipe_sim_core/src/machine.rs` and `crates/pipe_sim/src/machine_config.rs` | Replace machine-specific contract | Separate enclosure radius, shoulder transform, transport coupling, distal model, contact tool, and operation workspace. Remove legacy `Default` fallbacks and the sole-topology assumption. |
| `crates/pipe_sim_core/src/serial_arm.rs` | Retire as Pipe µ model | Extract generic frame or sweep utilities when independently useful. Do not preserve the fixed yaw/pitch/elbow/roll topology merely to minimize changes. |
| `crates/pipe_sim_core/src/arm.rs` | Quarantine; conditional reuse only | Constant-curvature continuum kinematics are a distinct model, not the new default. Reuse only if the chosen physical arm and evidence justify them. |
| `crates/pipe_sim_core/src/gripper.rs` | Substantial rewrite | Separate jaw geometry, contact evidence, retention state, adhesion/release behavior, and control responsibility. Avoid automatic acquisition or release from an ID update. |
| `crates/pipe_sim_core/src/simulation.rs` | Adapt | Preserve fixed stepping, command execution, interlocks, and records. Remove serial-only construction and single-owner attachment assumptions from the new physical contract. Geometry-only attachment may remain an explicitly labeled test model. |
| `crates/pipe_optics` | Preserve metrology capabilities; adapt scene | Keep timestamped observations, covariance, calibration identity, validity, and observability rules. Rebuild scene geometry, markers, windows, head layout, and qualification for the actual operation. |
| `crates/pipe_sim/src/metrology.rs` | Adapt | Derive optics and occluders from the selected embodiment. Track tool and part independently when verifying retention/release. |
| `crates/pipe_sim/src/observed_manipulation/{controller,estimator,runtime,plant,scenario}.rs` | Extract/rework | Keep observation-only control, freshness gates, bounded correction, and refusal reasons. Remove mandatory peg/socket, old arm, fixed layout, and insertion-specific release assumptions. |
| `crates/pipe_sim/src/handoff/**` | Rework | Model simultaneous physical supports, donor unloading, observable separation, receiver retention, and ambiguous outcomes. Control ownership is separate bookkeeping. |
| `crates/pipe_sim/src/point_motion.rs` | Adapt selectively | Preserve bounded commands and conservative path checking, but replace the specific mechanism and required pose assumptions. |
| `crates/pipe_sim/src/{scene.rs,observed_manipulation/report.rs,observed_manipulation/replay.rs}` | Adapt | Retain authoritative samples and provenance. Add deformation, supports, release status, environment, and per-claim evidence class. Unknown data stays unknown. |
| `crates/pipe_planner` gearbox executive/model/scenario | Retire from active runtime | Extract genuinely generic policy utilities before deletion. Build a small cooperative-operation executive without preserving a gearbox ontology. |
| Gearbox plant in `crates/pipe_sim/src/lib.rs`, `gear_observability.rs`, old `simple_manipulation.rs` | Retire active demonstrations | Preserve useful historical negative cases and extracted properties; remove default aliases, compiled gearbox hashes, and old scene creation. |
| `crates/pipe_sim_cli` and `crates/pipe_sim_wasm` | Keep boundaries; replace exposed defaults | The same active Pipe µ operation and provenance should drive available frontends. Remove old implicit `nominal` behavior and stale exports; version breaking schemas. |
| `crates/pipe_physics` | Optional, scoped retention | A rigid-body backend can support a defined study. It is not automatically a microscale adhesion, release, or flexure solver, and must not determine the architecture. |

A fresh module is often safer than an adapter when the old abstraction is wrong. Do not construct a general robotics framework to avoid choosing a first physical embodiment.

Sources: [S5–S9, S13, S17–S21].

## 7. CAD, fabrication, scenarios, and tooling

| Area | Decision | Required action |
|---|---|---|
| `cad/pipe_cad/{digital_thread,export,records}.py` | Keep infrastructure; adapt metadata | Preserve deterministic exports, hashes, geometry validity, units, part identity, and repeatable artifact generation. Remove mandatory gearbox metadata and insertion ordering. |
| `cad/pipe_cad/{params,arm,kinematics,structure,assemblies}.py` | Rebuild active machine definition | Introduce selected shoulder support, compliant geometry, tendon anchors/guides, native-surface contact, chamber access, and useful optical paths. Replace pinned-joint geometry and old kinematics. |
| `cad/pipe_cad/{sensing,tooling}.py` | Selectively adapt | Recover generic mounts, fiducial construction, fixture concepts, and export helpers; redesign dimensions and support relations for Pipe µ. |
| `cad/pipe_cad/{gearbox,gear_math}.py`, gearbox-only export scripts, `cad/baseline/gearbox.metadata.json` | Retire from active product | Preserve in the legacy snapshot. A future gear task needs a fresh rationale and cannot define the present architecture. |
| `cad/tests/**` | Adapt by property | Keep BREP validity, frame consistency, unit conversion, geometry hashes, and collision correspondence. Replace exact old dimensions and old assembly expectations. |
| Legacy machine and M1 scenario JSON files | Retire active defaults | Preserve original identity in history. Introduce new schema/model identities; never load old values into missing new fields. |
| `scenarios/gearbox_acceptance.json` and gear observability scenario | Retire active acceptance | Replace with operation and failure scenarios centered on native-resin manipulation and verified support transitions. |
| Replay/build/bootstrap scripts | Keep reusable machinery | Remove hard-coded scenario paths and artifact names. Keep clean-revision recording, reproducible commands, and explicit report validation. |
| `.github/workflows/ci.yml`, workspace manifests, locks and frontend exports | Adapt | Remove transitive old-machine dependencies and release artifacts after extraction. Preserve build, lint, test, CAD, provenance, and appropriate cross-target checks. |

The existing CAD/runtime manifest checks are useful but do not establish full geometry parity. The new design should generate or consume a common set of physical definitions and verify the CAD, collision, and optical interpretations at multiple articulated and deformed states. Full STEP/STL ingestion is not a prerequisite if reduced geometry and its margins are explicitly derived and checked. [S8, S9, S20]

## 8. Minimum new architecture

### 8.1 One selected embodiment and operation identity

Proposed new artifacts:

- `docs/decisions/PIPE_MICRO_RESET.md`: adopted direction, retired assumptions, allowed decision process.
- `docs/migration/pipe_micro_disposition.yaml`: per-path/symbol/parameter dispositions and completion evidence.
- `designs/pipe_micro/<candidate_id>/machine.json`: a versioned physical embodiment description.
- `scenarios/pipe_micro/<operation_id>.json`: operation, specimen/tool/fixture references, policies, seed, and evidence scope.

These are proposed paths, not existing files.

The embodiment description should identify the chamber; support/frame graph; shared versus independent z/θ motion; flexure geometry/model; tendon routes and anchors; tool/contact surfaces; fixtures; observation features and mounts; and relevant environmental interfaces. Keep operation limits in the operation contract, with units and provenance.

For material or contact assumptions, record a value or distribution, evidence status, applicable domain, and source. An unknown value may support an explicitly exploratory study with declared bounds; it must not silently acquire a legacy constant or support a hardware qualification claim.

### 8.2 Three independently represented kinds of state

**Physical state:** mechanism deformation, tendon loading, contact/support relationships, object motion, and environment within the model's declared coverage.

**Observed state:** timestamped measurements and estimates, covariance, observable degrees of freedom, source/calibration identity, and missing/invalid reasons.

**Control state:** authorized commands, logical responsibility, phase, safety decisions, and requested recovery.

The controller must not read evaluation truth as a substitute for an observation. A logical owner is not a physical attachment. The visualization consumes records; it does not manufacture evidence.

### 8.3 Contact and release without premature over-modeling

Use the simplest model that answers the experiment. A geometry-only attachment can test choreography. A seeded empirical contact model can represent measured pull-off, slip, and release distributions. A mechanistic model is justified when it resolves a concrete uncertainty.

Do not double-count adhesion through both empirical thresholds and additional uncalibrated force terms. Keep model class explicit in each claim and report. No single successful simulation is a physical reliability result. [S22]

A minimal support-transition vocabulary is:

`DonorRetained → ReceiverContact → DualSupported → DonorUnloading → SeparationObserved → ReceiverRetained → DonorWithdrawn → FinalPoseVerified`

Ambiguous separation or slipping must lead to an appropriate hold/inspection/unloading response, not success. Record `release_commanded`, `separation_observed`, and `retention_verified` as separate events with evidence identities. A synthetic observation only verifies the corresponding synthetic model. [S13, S23]

## 9. Execution packages and acceptance gates

The integration lead owns cross-domain consistency. Role labels below identify responsibility; they do not assign people or imply work has begun.

| Package | Owner role | Deliverables | Exit gate / dependency |
|---|---|---|---|
| P0 — Preserve and supersede | Integration lead | Pinned pre-pivot snapshot, evidence index, branch/issue dispositions, complete initial file/symbol/default inventory. | No unreviewed old-direction PR may merge into the new baseline. Nothing historically useful is lost. |
| P1 — Replace authority | Integration lead | Reset decision, new `AGENTS.md`, README, requirements, architecture scope, status and source precedence. | A new contributor sees one active product; unimplemented Pipe µ capabilities are explicitly absent. Depends on P0. |
| P2 — Extract and isolate | Runtime lead + verification | Generic primitives and tests separated; reviewed PR #17 extractions; old machine imports/defaults isolated and removed from active selection. | The new dependency graph cannot enter legacy execution. Old schemas/aliases fail explicitly. Depends on P0/P1. |
| P3 — Select a coherent physical candidate | Mechanics/CAD + optics + controls | One operation, shoulder/station arrangement, task-pose set, base-motion coupling, tendon/service routing, shared manifest and uncertainty budget. | The required poses are jointly reachable, observable, collision-admissible and service-feasible. No dimensional scaling shortcut. |
| P4 — Build coupled prototypes and coupons | Fabrication + experimental lead | Enlarged flexure/tendon prototype plus actual-scale contact/flexure specimens; recorded specimen/process identity. | Results identify what topology works and which physical assumptions remain unresolved. Starts with P3; feeds later packages. |
| P5 — New compliant/contact plant | Runtime + mechanics | Bounded flexure/tendon model, physical supports, intended contacts, declared adhesion/slip/release model, state-dependent safe responses. | Faults cannot become successful support transitions through ownership bookkeeping. Depends on P2/P3 and uses P4 evidence. |
| P6 — New observation integration | Optics + controls | Scene and feature identities tied to the candidate; part/tool observations; covariance/age/configuration guards; explicit unobservable axes. | Occlusion, stale calibration, wrong identities, or unobservable required motion stop admission. No old retiled-head or truth fallback. |
| P7 — Cooperative vertical slice | Integration + controls | Executed approach, pickup, transfer, dual support, donor unloading, separation, withdrawal, and destination inspection. Include a bounded z/θ positioning operation and relevant services. | Nominal completion and declared fault refusals are reproducible; all geometry, commands, evidence and reports share one identity. Depends on P5/P6. |
| P8 — Repository cutover and deletion | Verification + integration | Pipe µ default CLI/replay/CAD/CI; removal of remaining legacy active files, aliases, imports, fixtures and normative links; completed disposition manifest. | Clean checkout has one architecture. Old artifacts remain only explicitly historical. No compatibility shim is required by the new runtime. |
| P9 — Physical operation qualification | Experimental + integration | Actual-scale operation evidence, independent outcome measurement, environment/process history, repeated outcomes, updated parameter domains. | Claims meet predeclared operation-specific acceptance and uncertainty limits. Follows available P4/P7 work; distinct from repository cutover. |

Do not defer prototypes until all software is finished. Do not defer the new architecture until every coupon is characterized. Use the same candidate to couple design, observation, simulation, and measurements.

## 10. First demonstration proposal

Use two printed compliant distal tools and a receiving fixture to manipulate one selected part in the initial size domain. Approximately 500 µm is a convenient proposal, not a requirement. Select a shape and material pair with observable task-relevant orientation and a clear retention/release experiment.

The first local demonstration may keep bases fixed while testing support transfer. It must not be presented as a completed mobile cell. The subsequent integrated slice must execute the selected bounded z/θ motion with the actual shoulder support and service routing.

Use one known structural resin, native contact surfaces, and a stable recorded environment. Compare a smooth control with a small number of useful patterned/contact geometries. Do not require vacuum, coatings, wet tools, all texture families, continuous rotation, or self-manufacture to complete this first operation.

### 10.1 Geometry and macro prototype

Resolve enclosure radius, shoulder location, and cooperative workspace independently. A 10–15 mm arm at the wall of a roughly 100 mm-diameter enclosure cannot reach its axis; putting shorter numbers into the old central-workspace model is not a valid design. Compare candidate supports and station locations using the actual task pose set, not only a reach sphere. [S14]

The enlarged prototype tests assembly of tendons, routing changes, anchors, access, cooperative postures, jaw movement, and observation geometry. Record departures from uniform scaling. It does not validate microscale adhesion, charge behavior, or placement accuracy. [S24]

### 10.2 Actual-scale mechanics and contact

Measure only what the selected operation needs: usable flexure travel, relevant directional stiffness, settling/hysteresis/creep over its dwell and load range, tendon-anchor behavior, and pickup/retention/release outcomes.

For contact comparisons, record resin/batch, process, geometry, critical measured dimensions, part material, preload, dwell, approach/separation path and speed, RH, temperature, and wear/contamination history. Record failure mechanisms separately: stuck donor, one-jaw retention, receiver slip, rotation/jump, residual connection, damage, and verified release. A modest targeted set is preferable to a mandatory exhaustive coupon atlas. [S15, S25]

### 10.3 Observation and independent outcome verification

Re-evaluate field of view, feature placement, support occlusion, window effects, relative pose uncertainty, and the confidence of donor separation in the selected scene. Preserve unobservable pose components rather than inventing six-axis information.

Measure final placement independently enough to distinguish actual part motion from tool commands or tool-derived part estimates. Do not treat old 3–5 µm ambitions, camera sampling, or historical coupon passes as qualification of this scene. [S14, S19, S22]

### 10.4 Environmental and transfer integration

Carry sealed-chamber and small-transfer-chamber interfaces from the start. The integrated embodiment must include their physical access and disturbance implications, even if early specimens are tested in a simpler enclosure.

Begin with controlled, logged RH and temperature. Humidity cycling becomes an operation only after its effect on the selected resin/contact pair is established. Metal-free construction does not demonstrate zero charge; returning to the same RH does not automatically restore the prior surface state. Stopping or unloading tendons must depend on current support and stored energy, not on a universal release-all or hold-all rule. [S2, S23, S26, S27]

Retain physical emergency stopping, travel stops, bounded current/tension/energy, enclosure protection, and guarded low-speed commissioning as safety requirements. Re-derive their values for the actual embodiment. Do not energize hardware on the strength of simulated safe-force claims alone.

## 11. Tests that prevent a halfway result

| Test family | Required behavior |
|---|---|
| Legacy admission | Old machine schemas, historical model IDs, implicit gearbox aliases, and missing new physical parameters cannot enter the active Pipe µ runtime. |
| Authority consistency | Root guidance, default examples, active requirements, selected model and operation agree. Historical sources cannot become normative by link traversal. |
| Reach and services | Reject an impossible shoulder/workspace combination; reject unsupported shared/independent motion, prohibited passing, wrap, or route changes. |
| Deformation and collision | Include full physical tools, supports, flexures, payload, and relevant services. Reject blocked jaw opening, separation, and withdrawal. |
| Observability | Missing/stale/wrong-configuration observations or unobservable required motion block the next step. Preserve the truth firewall. |
| Retention | Apparent co-location, a closed jaw, or an owner ID alone cannot establish sufficient support. |
| Release | Donor adhesion, receiver slip, residual bridge, ambiguous gap, or missing destination evidence cannot become completion. |
| Safe response | Execute and verify the permitted fault response for the particular support/tool state; no blind retreat or universal tension dump. |
| Evidence class | Geometry-only, synthetic-contact, empirically calibrated, and hardware-observed results cannot inherit each other's qualification. Zero-gravity/rigid-attachment shortcuts cannot qualify physical retention or release under real loads. |
| Reproducibility | Identical revision/configuration/seed produces the declared deterministic comparison; reports preserve event and source identity. |
| Replay integrity | Missing/invalid geometry, provenance, or estimates yields rejection/unavailable, never replacement animation or telemetry. |
| CAD/runtime/optics agreement | Verify shared frames and physical definitions over several poses and deformations; matching one nominal hash is insufficient. |

Retire tests that assert an obsolete architecture. Preserve and port the safety, numerical, provenance, and observation properties they exercised. Every removed test needs a disposition: obsolete contract, replaced property coverage, or explicitly recorded gap.

## 12. Anti-contamination controls

### 12.1 Positive reuse admission

A retained component requires: its original assumptions, proposed new role, required changes, valid operating domain, new dependency path, and tests/evidence supporting admission. “It exists” and “its old tests pass” are not admission criteria.

### 12.2 Parameter provenance

The migration manifest should account for prior dimensions, loads, optical layouts, force limits, control timing, tolerances, material properties, and acceptance thresholds. Mark each as retired, re-derived, or explicitly readopted with justification. Avoid copying a constant into a new file and losing its origin.

### 12.3 Dependency and default checks

Use CI checks on active imports, scenario selection, generated metadata and schema/model identities—not a blanket ban on words such as `gear` or numeric values such as `0.080`. A future legitimate gear task or chamber dimension should be possible through an explicit decision, not accidental inheritance.

No new production path may call the old baseline loader or use an old `Default` constructor to fill missing design choices. Break incompatible schemas explicitly.

### 12.4 Agent and review context

The active `AGENTS.md` should require reading the reset decision and selected operation before implementation. Historical requirements are reference material only. Do not automatically load the full legacy documentation corpus into implementation context. Reintroducing a retired design decision requires a new decision record.

Each PR should identify the changed physical assumption, shared configuration impact, evidence class, safety consequences, test dispositions, and any legacy dependency it adds or removes. Cross-domain review is mandatory when geometry changes observation, contact, routing, or control.

### 12.5 No permanent quarantine product

A temporary quarantine is a migration mechanism, not a supported second architecture. P8 removes it from the active workspace. Keep original history and reproducible artifacts, not indefinitely maintained compatibility wrappers that force new abstractions to fit old ones.

### 12.6 Scope restraint

Do not replace old rigidity with new accidental dogma. Do not freeze six axes, a particular rail, universal humidity settings, a universal resin modulus, coatings, or all-2PP manufacture. Do not require a general FEM, electrostatic solver, or giant materials programme before trying a useful operation. Tie every added model and experiment to an uncertainty in the selected machine.

## 13. Definition of done

The repository pivot is complete when:

- Every active requirement and default belongs to Pipe µ, and every migrated path/parameter has an explicit disposition.
- The active dependency graph, CLI, WASM surface where retained, CAD export, scenario loader, replay, and CI cannot silently execute the retired machine.
- The selected embodiment separates enclosure, shoulder support, base motion, compliant mechanism, contact, optics, and environment coherently.
- A new executed cooperative operation and its failure cases use one physical/configuration identity, with release command, observed separation, and retention distinguished.
- The remaining unimplemented or unmeasured capabilities are visible rather than supplied by the old baseline.
- Historical evidence is preserved, accurately labeled, and excluded from new qualification claims.

Physical success requires the additional P9 evidence. Architectural alignment is not a hardware-performance claim.

**Governing question:** Would we choose this component or assumption for Pipe µ today if the old repository did not exist? If the answer is no, keeping it requires a concrete, bounded justification—not familiarity or sunk cost.

## Source index

All paths below are in `bertona88/thepipe`. Unless otherwise stated, the reviewed file revision is `f99bfeba5626c70689dd229540093e14272f3ac2`, which differs from the reviewed `main` only by the added atlas documents.

- **S1:** `docs/the-pipe-micro/README.md`.
- **S2:** `docs/the-pipe-micro/00-vision/design-choices.md`.
- **S3:** `AGENTS.md`.
- **S4:** `docs/REQUIREMENTS.md`, reviewed opening sections through the start of end-effector requirements.
- **S5:** `scenarios/machine_baseline_v1.json`.
- **S6:** `crates/pipe_sim/src/machine_config.rs`, reviewed schema/loading and initial construction code.
- **S7:** `crates/pipe_sim_core/src/machine.rs`, reviewed configuration/default/validation definitions.
- **S8:** `cad/README.md`.
- **S9:** `crates/pipe_sim/src/lib.rs`, reviewed exports, dependencies and scenario aliases.
- **S10:** `.github/workflows/ci.yml`.
- **S11:** Open PR #12, “Apply integrated gear-on-shaft project steering,” inspected 12 September 2026.
- **S12:** Draft PR #17, “Integrate an observed arm pickup-and-return operation,” metadata/body and changed-file list inspected 12 September 2026; head `f478ae48af27034959050ce1ca4b9a6ad655d9ba`. PR-reported results were not re-executed for this audit.
- **S13:** `docs/the-pipe-micro/02-subsystems/control/verified-handoff.md`.
- **S14:** `docs/the-pipe-micro/01-architecture/scale-and-workspace.md`.
- **S15:** `docs/the-pipe-micro/05-evidence/coupon-atlas.md`.
- **S16:** Open issues #13–#16, bodies inspected 12 September 2026.
- **S17:** `crates/pipe_sim_core/src/arm.rs`, reviewed constant-curvature model definitions.
- **S18:** `crates/pipe_sim/src/handoff/controller.rs`.
- **S19:** `crates/pipe_optics/README.md`.
- **S20:** Git trees for root, docs, CAD, scenarios, `pipe_sim_core`, and `pipe_sim`, and atlas directory; path-level inventory evidence does not imply full source review of every file.
- **S21:** `docs/the-pipe-micro/02-subsystems/flexure-arms/README.md`.
- **S22:** `docs/the-pipe-micro/05-evidence/measurement-and-models.md`.
- **S23:** `docs/the-pipe-micro/02-subsystems/control/collision-and-recovery.md`.
- **S24:** `docs/the-pipe-micro/05-evidence/macro-and-micro-prototypes.md`.
- **S25:** `docs/the-pipe-micro/02-subsystems/fabrication/materials-and-process.md`.
- **S26:** `docs/the-pipe-micro/02-subsystems/environment/humidity-and-electrostatics.md`.
- **S27:** `docs/the-pipe-micro/02-subsystems/tendons/routing-and-preload.md`.
