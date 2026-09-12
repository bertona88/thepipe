# Observed gear pickup and return candidate

The candidate connects distal geometry, independently observed tool and gear
references, bounded arm motion and a fixed return fixture in the existing
160 mm ID machine. It addresses the previously inadmissible pickup before
attempting bore/shaft alignment. It does not establish gearbox assembly or
hardware performance. Execution status must come from the generated report;
implementation of the operation alone is not a completed-cycle claim.

## Shared geometry

The machine configuration is
[`machine_gear_pickup_v1.json`](../scenarios/machine_gear_pickup_v1.json).
Baseline configurations retain their previous zero-standoff tool behavior.

| Item | Candidate dimensions and convention |
| --- | --- |
| Arm links | 32 + 30 + 15 mm; existing mobile base and four tendon axes |
| Distal frame | Runtime TCP; jaws close along X, approach along +Z |
| Wrist/TCP separation | 5 mm along the final wrist axis; physical wrist endpoint at TCP Z = −5 mm |
| Palm | 3.6 × 2.6 × 2.0 mm, centre (0, 0, −4) mm |
| Fingers | 0.2 × 2.2 mm cross-section; Z from −3 to −0.6 mm |
| Contact pads | 0.2 × 2.2 × 1.2 mm, centres X = ±(opening/2 + 0.1 mm), Z = 0 |
| Opening | 0.08–2.8 mm |
| Gear coupon | 18 teeth, module 0.1 mm, 2 mm outer diameter, 0.8 mm thickness, 0.52 mm bore |
| Return contacts | Three 0.15 mm-radius balls at 0.6 mm radial distance, 120° apart; centre Z = +0.55 mm in the nominal gear frame |
| Fixture supports | 0.1 mm-radius capsule stems and 1.8 × 1.8 × 0.3 mm fixed plate; present in runtime collision geometry |

The five distal solids replace an implicit empty tool extension. Their dimensions
come from the same configuration used by runtime collision queries. The CAD
candidate explicitly distinguishes the wrist endpoint from TCP; it does not
silently reuse the legacy CAD gripper's local +X frame. CAD export preserves the
legacy catalogue. See [CAD reproduction](../cad/README.md).

The coupon preserves a small gear's grasp and optical obstruction problem. It
omits the gearbox housing, neighbouring gears, shaft approach and constrained
seating/release. The reference gearbox remains the destination.

## Physically located optical references

All coordinates below are millimetres in the relevant tool or gear frame.
The signs in paired coordinates independently enumerate their combinations.
Markers are nominal 0.1 mm printed patches on existing substrates, not floating
points on unmodeled tabs. The operation checks nominal marker surface attachment
and intersection against modeled bodies.

| Substrate | Marker centres | Outward normals |
| --- | --- | --- |
| Palm front face, four spots | (±1.65, ±1.15, −3) | +Z |
| Palm side faces, four spots | (±1.8, ±0.8, −3.3) | Corresponding ±X |
| Gear face, four spots | (±0.5, ±0.4, −0.4) | −Z |
| Gear rim, six spots | (cos θ, sin θ, ±0.25), θ = 80°, 90°, 100° | (cos θ, sin θ, 0) |

The front palm spots move beyond finger-root occlusion. The rim constellation
retains useful gear references with closed jaws. These changes follow modeled
visibility failures. The rim substrate is the conservative cylindrical gear
envelope; a real tooth surface and a real printing/marking process need separate
qualification. Surface endpoint uncertainty is tangent to the nominal substrate;
normal substrate form errors are not simulated. Ray occlusion and camera noise
are simulated, not rendered-image detection or measured camera performance.

Tool and part references are independently acquired through the common metrology
adapter. The controller does not synthesize a part observation from the held
transform. Printed-marker-to-feature calibration and optical errors remain
modeled assumptions, and geometry truth stays separately labeled in replay.

## Operation and admission

The runtime implements approach, grasp, short lift, observed hold, return,
supported release and withdrawal. Fresh observation and conservative relative
uncertainty gate the pickup. Its 100 µm engagement margin is a pickup clearance
budget; it is not the unchanged 5 µm insertion precision limit. The report
explicitly records that bore/shaft alignment is not evaluated in this operation.

The three-ball release condition uses geometrically evaluated support gaps,
allowing up to 20 µm positive gap. With zero gravity this cannot demonstrate
settling, load transfer or physical support contact. The held part uses a rigid
attachment and pad compliance gives a force proxy, not a calibrated force sensor.
Missing or stale observations, motion timeout and unsupported release have
explicit refusal paths. Refusal is reported separately from completion.

## Executed evidence

The clean run at `f805ce58addcb39e4f406656c8a28a9448a543d4` reports `completed_pickup_return`, no refusal,
and all nine phases through `withdraw_and_verify`. It records 2,400 authoritative
samples over 46.802 simulated seconds (final tick 46,802). Independent CAD FK
checks every recorded articulated sample, including TCP orientation; maximum
position discrepancy is 5.99 × 10⁻¹⁷ m. The recorded fixture exports as seven
valid CAD solids. Final release gaps are +1.687, +1.482 and +1.728 µm; these
are positive geometric separations in zero gravity, not physical settling
or measured support loads. The [committed evidence summary](evidence/arm-pickup-return-f805ce5.json)
records exact source/configuration identities and report hashes. All 2,400 runtime
samples reproduce the development run exactly. Four injected faults refuse for
their declared reasons with stationary terminal states; unsupported release
retains the gear. The recorder checks reason and ownership, so an unrelated early
refusal cannot satisfy a fault case.

The experiment first exposed 50 µm clearance between the held gear and a
fixture stem, causing a guarded lift refusal. Revised stems preserve their
ball/plate attachments while providing 200 µm nominal clearance to the gear.
The completed run uses that revised geometry; collision guards
remain active. Completion establishes this reduced pickup/return experiment,
not physical support transfer, insertion precision or gearbox assembly.

## Reproduction and evidence

```sh
python3 scripts/record_gear_pickup.py --fault-suite
python3 scripts/render_gear_pickup.py out/gear-pickup/nominal.json \
  --output out/gear-pickup/arm-operation.mp4 --fps 12 --duration 16
python cad/scripts/export_arm_candidate.py scenarios/machine_gear_pickup_v1.json \
  --output build/cad/arm_candidate
```

Run the recorder from a clean checkout. It builds the executable and records the
actual source revision and SHA256 of `git ls-tree -r --full-tree HEAD` output bytes.
The report includes these identities, machine and operation configuration
hashes, explicit fidelity statements, completed phases, refusal reason and
recorded scene samples. Fault arguments are `missing-part`, `stale`, `timeout`
and `unsupported-release`.

Animations must use those recorded samples, retain simulation tick/time and
truth/estimate distinctions, and display the report's actual outcome. CAD
`verify_recorded_arm_frames` independently checks articulated frame agreement;
`make_recorded_fixture` exports fixture solids from the recorded description and
poses rather than maintaining a second geometry definition.

No actuator selection, guide mechanism, fastener design, strength validation,
loaded correction calibration, gravity/slip dynamics, real marker calibration
or hardware-qualified precision is established by this candidate. A build-ready
arm still requires those consequential design and bench results.
