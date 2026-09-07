# Transparent gear / shaft observability experiment

The question is whether optical observations can support aligning a small smooth,
transparent part while the machine's own arms and jaws obstruct the views. This
experiment separates direct surface measurement, pose inferred from markers, and
independent knowledge of the mating geometry. It does not claim a completed assembly
or qualified physical accuracy.

## Executable configuration

`scenarios/gear_observability_v1.json` specifies a 2 mm diameter, 0.8 mm thick
annular gear envelope with a 0.52 mm bore and a 0.50 mm shaft. It samples approach,
grasp, reorientation, precontact and insertion design states in the 25 mm precision
zone. The five states are independently articulated with the authoritative runtime
FK; they are **not executed assembly progress**. The part is fixed at the pickup
datum before grasp; the shaft datum remains fixed across states. Truth transforms
are explicitly separate from optical estimates.

The study uses the existing **160 mm ID** machine, not the standalone 100 mm
optical candidate. Other arms remain in their runtime parking configurations.
The shared link capsules and jaw boxes cast optical shadows. The optical gear is
a faceted annulus with an open bore; transparent bulk is conservatively blocking,
so rays refracted through it cannot masquerade as direct views. The shaft has the
same capsule envelope in optics and mechanics. Static physical checks use the
runtime's gear envelope and report contact IDs and penetration separately.

Six cameras, the existing sensor/lens/noise/thermal configuration, permanent
reference observations and the precision contract are retained. The sweep varies:

| Parameter | Candidates |
| --- | --- |
| Camera axial positions | Alternating ±10 mm / ±25 mm |
| Projector circumferential position | −45° / 135° |
| Part and shaft marker offset from datum | 3 mm / 6 mm |
| Independent marker-to-feature characterization | 1 µm / 8 µm 3D RMS |
| Repeated acquisitions per state | 4 |

There are 16 combinations and 320 state/exposure samples. Optical configuration
hashes, machine configuration hash, seed and source revision accompany the report.
Marker features use the existing five-point rigid constellation; these are nominal
handling-tab features, with no manufactured marker assembly or tab support geometry
yet qualified. Identity associations are provided to the reduced feature frontend;
there is no image rasterization or tag decoder.

## Measurements and admission

`infer_part_feature` propagates the complete marker pose covariance through
`J = [I, -[R p]×]` to the bore or shaft datum. Independent dimensional
characterization uncertainty is added once. Axis uncertainty includes marker
orientation and relation characterization. Provenance explicitly says **inferred**,
not directly observed surface geometry. Nominal CAD relations cannot pass
`require_inferred_precision`; physical operation additionally requires physical
relation characterization and the existing physical optical provenance checks.

Fixed independent dimensional offsets are introduced only in verification truth.
They are not fitted to the marker observations and do not vary between repeated
exposures. This makes good repeatability compatible with poor feature accuracy.
The test checks that changing this independent geometry leaves the optical pose
fit unchanged.

Relative alignment uncertainty is conservatively bounded by the sum of the two
marginal 3D RMS values. This avoids inventing independence or cancellation of
shared camera calibration errors. It is an upper bound, **not a joint covariance**.
The existing tool/target/reference/timing gate and a 5 µm relative-alignment bound
must pass. Orientation uncertainty is also compared with radial mating clearance.
Optical admission does not override the separate mechanical checks.

The same geometry is acquired with diffuse marker/feature illumination and hybrid
Gray/phase structured light. An opaque witness on the nominal gear face tests
whether the projector could observe that location even if material response were
favorable. It is not substituted for a transparent-surface measurement. Transparent
silhouette extraction and refractive reconstruction remain unsupported.

## Evidence and implications

The committed summary is `docs/evidence/gear_observability_v1.json`; its
`source_revision` identifies the implementation used to generate it. All 320
samples retain tool, part-marker and shaft-marker pose estimates. Partial marker
occlusion remains present; minimum part support is five cameras in this sampled
layout. This establishes observability of these nominal marker positions only,
not coverage of the complete machine or a physical marker mount.

For the −45° projector candidate (passive results are identical at 135°):

| Camera axial offset | Marker offset | Relation RMS | Available-estimate alignment error, 3D RMS | Maximum conservative alignment RMS bound |
| --- | --- | --- | --- | --- |
| 10 mm | 3 mm | 1 µm | 3.742 µm | 7.058 µm |
| 25 mm | 3 mm | 1 µm | 3.734 µm | 7.047 µm |
| 10 mm | 6 mm | 1 µm | 3.731 µm | 11.329 µm |
| 25 mm | 6 mm | 1 µm | 3.649 µm | 11.326 µm |
| 10 mm | 3 mm | 8 µm | 13.068 µm | 17.373 µm |
| 25 mm | 3 mm | 8 µm | 13.054 µm | 17.368 µm |

These are synthetic errors against geometry excluded from the marker fit, under
one seed and a small fixed set of systematic perturbations. They are not physical
accuracy, a calibrated probability of success, or validation of the entire volume.
Repeated bore-centre scatter remains below 1.6 µm RMS in this sample even with the
8 µm relation uncertainty: repeatability does not reveal the dimensional bias.

**Zero precision admissions** result. Increasing marker offset reduces some
occlusions but amplifies orientation uncertainty at the bore. Changing camera axial
spacing has little effect on this particular error budget. Neither projector position
returns a valid surface measurement at the chosen witness location in these states;
occlusion and unavailable reconstruction prevent treating structured light as a
solution for this task. This is not evidence that all projector layouts or transparent
measurement methods are ineffective.

Four of five static states contain envelope contacts. In particular, gear body 700
intersects the active wrist capsule (body 3221225478) by approximately **1.54 mm**
when placed at the nominal TCP. This is a conflict in the current conservative
runtime representation, not a measurement of actual hardware interference.
Moving pickup off the original tool position did not make the approach admissible.

A separate **actual command probe** inserts gear/shaft collision envelopes into the
runtime and attempts approach, pickup, gripper closure, grasp, transfer and precontact
through the existing bounded command API. The default first approach is rejected
by `ToolPathCollision` at tick 0. A stop request is recorded, no grasp or insertion is
reported, and no optical endpoint samples are mislabeled as executed motion.
The probe deliberately has no contact-insertion or optical-servo success fallback.

The unresolved design constraints are distal-tool clearance, physically supported
marker placement, characterized marker-to-mating-feature geometry, transparent
surface observability and shared-error covariance for relative alignment. Endpoint
optical visibility alone is insufficient to qualify the proposed operation.

## Reproduction

```sh
cargo run --locked -p pipe_sim_cli --bin pipe-gear-observability -- --print-config
cargo run --locked -p pipe_sim_cli --bin pipe-gear-observability -- \
  --config scenarios/gear_observability_v1.json \
  --source-revision <full-implementation-commit-sha> --summary
```

Omit `--summary` to retain individual truth/estimate poses, supporting pixels,
full covariances, acquisition timing, rejection reasons and visibility attempts.
The summary retains state aggregates and deduplicates optical configurations by
hash. The CLI accepts an asserted source SHA; the caller must run the corresponding
clean checkout. The source SHA is provenance, not an automatic attestation.

Validation covers full covariance/lever-arm propagation, rejection of invalid and
nominal geometry, deterministic replay, fixed fixture frames, exclusion of
independent geometry from pose fitting, transparent-surface refusal and the
existing optical/machine metrology regressions.
