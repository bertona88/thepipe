# Migration record

Baseline `4ccbd72c954fa8dd699b43b1eb900428942cfdbb` is pinned as
`legacy/pre-pipe-micro-20260912`. Atlas documents were recovered unchanged from
`f99bfeba5626c70689dd229540093e14272f3ac2`.

`pipe_micro_disposition.yaml` is JSON-compatible YAML: one entry per baseline tracked
path, original byte hash, disposition and archive destination. Original evidence and
historical methods retain exact bytes under `docs/history/pre-micro/`; original code
and scenarios remain reproducible from the tag. Historical links stay historical and
may refer to code at that revision. They do not confer active requirements.

`legacy_inventory.json` records declared symbols, test names and numeric source lines,
and every scalar JSON parameter, at the pinned revision. This is a syntactic inventory,
not a claim that every literal is an independently meaningful engineering parameter.
All old machine values are retired; kept library values retain only their prior study
domain. New candidate values are independently explained in NEAR_WALL_V1.md.

PR #12's gearbox steering is superseded. PR #17 was inspected at its branch diff:
tool standoff/frame coverage, full tool collision shapes, independent surface marker
acquisition, executed stop and support-gated release are useful properties. Its serial
kinematics, 5 mm tool extension, fixture protocol and legacy optional fallbacks are
not admitted. No code was cherry-picked wholesale. The new implementation covers
separate tools/part, full reduced envelopes, stop execution, independent observations
and release guards. Calibrated surface-marker acquisition remains a library integration
gap. The branch stays recoverable in Git.

See TEST_DISPOSITIONS.md for removed test treatment and remaining safety coverage.
No obsolete deliverable is reported complete simply because direction changed.

Remote publication and backlog actions are pending approval; no remote state was
changed. Exact prepared PR/backlog text is in PUBLICATION.md.
