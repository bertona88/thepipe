# Pipe µ implementation status

All active operation evidence is **synthetic_contact**; hardware qualification is false.

| Capability | Status | Scope / remaining gap |
|---|---|---|
| Sole Pipe µ authority and default | Implemented | Legacy execution removed; pinned history and per-path dispositions retained |
| Explicit candidate/operation schema | Modeled | Near-wall two-tool candidate; physical embodiment not qualified |
| Shared bounded z/θ positioning | Modeled | Executed unloaded positioning; service envelopes, not manufactured routing |
| Local compliant tool motion | Modeled | Bounded linear translation/strain screen; no calibrated beam/tendon mechanics |
| Contact and cooperative transfer | Modeled | Seeded unmeasured retention/pull-off law with rigid retention; dual support and separate release outcomes |
| Observation-only controller | Modeled | Independent synthetic centroids and contact-state evidence; no actual sensor acquisition |
| Optical scene | Modeled | Two-view ray/box checks and uncertainty gates; no selected camera/window calibration |
| Orientation | Absent | Constrained in model; departures refuse; no six-axis claim |
| Collision and CAD | Modeled | Shared conservative envelopes at executed states; no continuous swept-solid or BREP proof |
| Replay/provenance | Implemented | Deterministic rerun, schema/hash/data validation, source identity |
| Fault stop | Modeled | Executed freeze with preserved load; not hardware-safe commissioning evidence |
| Chamber and transfer access | Candidate interface | Logged environment and reserved access; seal/flow/disturbance design absent |
| Actual resin/process/batch | Absent | R1 explicitly unselected; fabrication cannot proceed on assumed material data |
| Enlarged routing prototype (P4) | Absent | Requires fabrication and physical inspection |
| Actual-scale contact/flexure coupons (P4) | Absent | No measured stiffness, travel, hysteresis, creep, pull-off, slip or wear |
| Calibrated optics/contact (P5/P6) | Absent | Synthetic software boundary exists; empirical evidence remains required |
| Integrated physical feasibility (P3/P7) | Incomplete | Local operation runs; full supports/services/visibility/deformation not qualified |
| Hardware operation qualification (P9) | Absent | Repeated independently measured outcomes required |

`bash scripts/verify.sh` exercises nominal execution, all declared fault refusals,
truth-firewall properties, schema/reach/service/uncertainty admission, replay tampering,
geometry correspondence and preservation of historical evidence. Passing these checks
is software evidence only. Independent Rust tests run separately in CI; Rust was not
installed in the implementation session and local Rust execution was unavailable.

The repository authority/default cutover is implemented. The plan's full physical
candidate, prototype, integrated feasibility and qualification exit gates remain open;
this status deliberately does not equate architectural alignment with P9 completion.
