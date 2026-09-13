# Test and reuse disposition

The exact baseline test/symbol inventory is in legacy_inventory.json. Each removed
file is mapped by pipe_micro_disposition.yaml. The mapping below applies to every
test within the named family, including embedded Rust tests; it does not claim that
new coverage is equivalent to hardware validation.

| Removed family | Disposition | New coverage or recorded gap |
|---|---|---|
| Planner/gearbox/gear observability | Obsolete task contract | No active gearbox acceptance; historical negative results preserved |
| Serial arm, continuum arm and old machine configuration | Obsolete mechanism contract | New schema, shoulder reach, coupling and strain/load admission tests |
| Actuator/simulation dynamics | Replaced partially; gap | Fixed-step bounded execution and stop tests; dynamic friction/slack/creep not modeled |
| Gripper/simple manipulation/handoff | Replaced property coverage | Physical support sets, separate release/separation/retention, adhesion/slip/bridge failures |
| Observed controller/estimator truth firewall | Replaced property coverage | Controller has no plant import, requires identified fresh observation and covariance |
| Metrology integration and M1f optical scenes | Retired scene; gap | New ray-box synthetic scene; full reconstruction/calibration remains unintegrated |
| Collision/geometry/point motion | Replaced partially; gap | Full reduced tool/support/fixture/service boxes; continuous swept/deformed collision not proven |
| Replay/CLI/WASM | Replaced Python CLI/replay; WASM retired | Deterministic reexecution, tamper rejection, source/configuration identity; no WASM product API |
| Old CAD parameter/BREP tests | Obsolete assembly; BREP gap | Shared envelope consistency over executed states; no manufacturable solid validity claim |
| SI/math and independent optics tests | Kept | Original domains only, checked separately in Rust CI |

Positive reuse admission: SI quantities and rigid transform algebra assume finite SI
inputs, not a machine size; their new role is an independent utility library. Optics
retains its own documented camera/scene/measurement assumptions and tests; its new
role is an independent study library, with no production dependency path into Pipe µ.
Any future integration must replace its historical scene/default configuration and
show the selected candidate's visibility and uncertainty. Existing tests alone do not
admit its baseline constants into the new operation.
