# The Pipe µ engineering rules

Build the cylindrical controlled microassembly workspace described by the adopted
[reset decision](docs/decisions/PIPE_MICRO_RESET.md). Read that decision, active
[requirements](docs/REQUIREMENTS.md), the [selected candidate](docs/decisions/NEAR_WALL_V1.md),
and its machine/operation contracts before implementation.

Explicit user direction and adopted decisions → Pipe µ requirements → selected
embodiment and operation contracts → configuration/code → evidence about that configuration.
The atlas preserves intent and open choices. Historical documents have no authority.
The gearbox machine, serial-arm topology and old milestones are retired; restoring
any assumption requires an explicit new decision, not a compatibility fallback.

Work on one machine across mechanics, CAD, optics, control, contact and environment.
Cross-domain review is required whenever geometry changes visibility, services,
contact or control. Prioritize physical correctness, explicit SI units and frames,
determinism, guarded commands, testability and honest fidelity boundaries.

- Keep physical state, observations and commands distinct. Controllers consume only
  observations; software responsibility never establishes physical attachment.
- Treat release commands, observed separation and verified retention as different
  events. Missing, stale, wrong-identity, uncertain or unobservable data fails closed.
- Derive defaults from the selected operation, with provenance and valid domains.
  Unknown material properties remain unknown; synthetic bounds cannot qualify hardware.
- Preserve z/θ positioning, support geometry, service coupling and sealed/transfer
  chamber interfaces. Do not assume independent stages, passing or continuous rotation.
- Preserve the single-native-resin initial direction without inventing a measured
  material batch. Hardware use is blocked until the specimen/process is identified.
- Visuals consume executed records or reproducible CAD. Never invent telemetry,
  animation, estimates or success. Record source revision, tree/configuration hashes,
  ticks, units and fidelity. Missing geometry means unavailable, never a fallback.
- Each PR records changed physical assumptions, shared configuration impact, evidence
  class, safety consequences, test dispositions and added/removed legacy dependencies.
- Run `bash scripts/verify.sh`; retained Rust libraries additionally require workspace
  tests, formatting and Clippy. Record unavailable tooling honestly.
- Physical stopping, current/tension/energy bounds, travel stops and guarded commissioning
  need embodiment-specific hardware verification; a simulated hold is not certification.

No active code may import historical machine implementations or load their schemas.
No permanent legacy compatibility product. Git history preserves recoverability.
