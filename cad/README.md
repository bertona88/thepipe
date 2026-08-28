# Pipe assembly-cell CAD

This package is the nominal hardware geometry for the observed-volume assembly
cell.  It is deliberately engineering-first: precise BREP parts, deterministic
dimensions, assembly poses, and simulator metadata are generated before any web
presentation layer.

The nominal reference scene is a **160 mm-ID, 320 mm usable-length tube with four
independent z+theta arm bases, six global cameras, one rigid 12 mm-baseline wrist
macro stereo head, and a micro-gearbox assembly benchmark**. The cell hardware is
kept DIY-friendly; the ideal nominal benchmark parts are intentionally much smaller
and target eventual two-photon polymerization (2PP), which is not simulated here.

## What is needed first

1. Python 3.11+ and `build123d` 0.10 or 0.11 (OpenCascade is pulled in by it).
2. A CAD-dimension source of truth: `pipe_cad/params.py`.  The checked-in
   acceptance scenario points to its exported metadata JSON; the Rust CLI validates
   its schema/hash/BREP flags and an enumerated dimensional subset before starting
   the separately compiled reduced-order model.
3. Build and instrument one rail/base/arm channel first; the four-arm export is
   a replication target, not a procurement instruction. Each carriage deck places
   its shoulder datum at 72.0 mm radius while the rail body is at 76.9 mm.
4. The first arm needs four measured actuator channels with 3.0 mm capstans,
   1.65 mm joint moment arms, 12 mm usable payout, encoders/current/temperature
   sensing, tensioners, and opposed 0.20 mm UHMWPE line winding. Replicate to
   sixteen channels only after its travel/force/repeatability gates pass.
5. Six triggerable global-shutter board cameras, two close-focus wrist cameras,
   one shared calibrated coded-projector or laser-line module, matte
   fiducials, a sync pulse source, and stable camera mounts.
6. For the later physical benchmark, not for the current ideal-part software gate:
   select and independently qualify a 2PP vendor/process against the current 8-20
   micrometre design-rule assumptions, add three 0.35 x 1.55 mm dowel shafts, and
   inspect gear bores and shaft center distance.  The nominal gearbox is
   6.00 x 4.00 x 1.83 mm closed.
7. Prototype the current loose-pocket gearbox carrier and insertion-order parts
   tray; the carrier still needs explicit restraint contacts and a clamp before
   it is a fixture. Fit the arms with a vacuum micro-pick, compliant insertion
   probe, replaceable rotary drive blade, and three-point calibration pointer
   from `tooling.py`.
8. Before physical assembly, calibrate camera intrinsics, global extrinsics,
   wrist-camera extrinsics, each rail's theta axis, each carriage's z scale, and
   the arm joint/tendon maps.  The visual tracker remains the absolute reference.

## Nominal mechanism snapshot

- Tube: 80 mm inner radius, 3 mm wall, 330 mm physical length, 5 mm end margins.
- Base motion: each carbon rail is carried by its own paired belt-driven end
  bogies on common circular tracks (theta); its sleeve carriage provides z.
  The exported scene is one static pose.  Motion limits and collision scheduling
  belong in the simulator.
- Arms: 32 + 30 + 15 mm serial links, 5.6 x 2.4 mm beam section, 1.2 mm pins,
  paired 0.20 mm UHMWPE lines, and four shared differential capstans for
  shoulder yaw, shoulder pitch, elbow pitch, and wrist roll.  Each capstan uses
  one motor/spool for both line sides; the wrist terminates in a two-jaw gripper.
  The CAD contains a distinct +Z shoulder-yaw turntable, yaw-rotated +Y
  shoulder-pitch and elbow-pitch yokes/pins, and a final-link +X wrist-roll
  sleeve/rotor.  Wrist roll changes tool orientation; it is not approximated as
  a fourth planar bend.
- Sensing: the static CAD locks two clocked 120-degree global triplets at 60 mm
  front-face radius and z = 59/271 mm in the tube frame (-106/+106 mm from the work
  datum). A modeled rear crossbar overlaps both macro-pod shells at a rigid 12 mm
  baseline and includes a keyed wrist tongue in the CAD packaging pose. The gearbox
  manifest cross-checks the global front-face layout and macro
  baseline/raster parameters, not the operational wrist pose. The reduced runtime uses
  the same layout numbers in an active-component-local sensing frame, not these fixed
  tube-world and wrist transforms, and does not ingest the full-cell CAD geometry.
- Gearbox: module 0.10 mm, 25 degree pressure angle, 0.020 mm backlash,
  12/18/24 teeth, 1.2/1.8/2.4 mm pitch diameters, centers at x=0.75/2.25/4.35
  mm, 0.35 mm gear faces, 1.30 mm total gear/hub height, 0.35 x 1.55 mm shafts,
  0.420 mm running bores, and a 2:1 reduction ratio.  The body is 1.60 mm tall with a
  0.25 mm floor; the 0.20 mm latched cover leaves 0.05 mm above the gear tops.
  Blind split-compliant shaft seats are 0.340 mm diameter x 0.250 mm deep.
  The cover has 0.75 mm input-driver and 1.0 mm output-observation windows; G1
  has a 0.10 mm drive slot and G3 has unequal optical phase marks.  Three bottom
  pads define the external datum.  Every gear bore has 0.025 mm x 45 degree
  entry chamfers at both ends; each shaft has 0.005 mm maximum end deburr
  chamfers without changing its 1.55 mm envelope.
- Tooling: a loose-pocket gearbox carrier prototype, sequenced parts tray, 0.30 mm-tip vacuum
  pick, connected four-leaf fixed-guided insertion probe, 0.080 mm rotary blade for the
  0.10 mm G1 slot, and a 10 um-apex calibration pointer. These are simple
  parametric solids and collision envelopes, not metrology-grade process models.
  Their presence in CAD does not mean the reduced runtime changes tools or engages
  the rotary blade during its post-run analytic gearbox check.
  A seated gear spans z = 0.25 to 1.55 mm in the housing frame, so its solid
  center is z = 0.90 mm; scenario approach/travel values are not solid-center poses.

The theta rail arrangement is a low-cost prototype architecture, not a final
production bearing system.  A future scheduler must prevent bogie/rail crossings and
reserve azimuth corridors. Camera shells and motor bodies exist as CAD solids, but the
current runtime does not ingest them for collision or occlusion. PCB connectors,
wiring, most fasteners/bearings, belt teeth, service loops, and flexing tendon wrap are
not complete keep-out models and must be added before a hardware collision claim.

## Install and validate

```bash
cd cad
python -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

The kernel-free checks run even without build123d:

```bash
python scripts/validate_parameters.py
python -m unittest discover -s tests -p 'test_parameters.py' -v
```

## Export

```bash
# All assemblies plus an individual printable-parts catalog
python -m pipe_cad.cli all --output build/cad

# Closed gearbox and every insertion item as STEP/STL
python scripts/export_gearbox.py --output build/cad

# Optional coarser/finer preview mesh; exact STEP is unaffected
python scripts/export_gearbox.py --output build/cad --stl-tolerance 0.004

# Exploded insertion-order view
python scripts/export_gearbox_exploded.py --output build/cad

# Fixture, parts tray, and each swappable tool as individual STEP/STL
python scripts/export_tooling.py --output build/cad

# Full nominal reference cell with the completed benchmark at workspace center
python scripts/export_assembly.py --output build/cad
```

Every assembly produces `NAME.step`, `NAME.stl`, and
`NAME.metadata.json`.  Metadata includes the complete parameter tree, BREP
validity, bounding boxes, volume, material/process hints, and the locked
insertion sequence `S1, S2, S3, G3, G2, G1, cover`; the housing is already held
in the assembly fixture.  The final operation closes both cover latches.
`parameter_sha256` is SHA-256 of sorted, compact, ASCII JSON for the full
parameter tree; `geometry_facts_sha256` fingerprints ordered record geometry.
Homogeneous records with an explicit nominal density include mass, center of
mass, and centroidal inertia.  Mixed or stock assemblies deliberately report
unknown physical mass/inertia while retaining unit-density geometric moments.
Catalog mode additionally emits an origin-centered STEP and STL for every
unique fabricated part and tool envelope.

The canonical acceptance input references `cad/baseline/gearbox.metadata.json`.
The CLI resolves that path relative to the scenario, recomputes the manifest's
canonical parameter and geometry-facts SHA-256 values, locks every other scenario value,
and checks BREP validity, insertion order, and an enumerated subset of CAD/runtime dimensions.
The report records those source hashes separately from its combined run hash. This is a
consistency gate, not full CAD/runtime parity: runtime geometry and sensing remain compiled,
and STEP/STL triangles are not loaded as live collision or optical geometry.

## Package map

- `params.py` -- immutable millimetre-scale design parameters and physical checks.
- `gear_math.py` -- kernel-free analytical involute profile sampling.
- `structure.py` -- tube, rail, z carriage, theta track, and theta bogie.
- `arm.py` -- four physical joint stages, links, 3-D tendon preview paths,
  motor-spool bank, and gripper.
- `kinematics.py` -- kernel-free four-axis forward kinematics and named frames.
- `sensing.py` -- global/macro camera assemblies and printable shells, projector
  assembly/shell, and relief fiducial.
- `gearbox.py` -- gears, housing, shaft, cover, and nominal/exploded assembly.
- `tooling.py` -- nest, tray, vacuum pick, insertion probe, driver, and pointer.
- `digital_thread.py` -- kernel-free canonical JSON and SHA-256 helpers.
- `assemblies.py` -- nominal reference world pose, benchmark/tool poses, and catalog.
- `export.py` -- STEP/STL and deterministic JSON metadata writer.

## Manufacturing boundary

Desktop MSLA/FDM settings in `ManufacturingParams` apply only to the cell-scale
brackets, carriages, and pods.  `GearboxParams` carries separate 2PP minimum
feature, running-clearance, and tolerance-perturbation values.  The CAD does not
simulate the 2PP process or apply automatic shrink compensation to the
micro-gears.  Run a vendor-specific
tolerance study by perturbing bores, shaft seats, center distances, and backlash
after the nominal geometry and optical/collision simulation agree. Those
perturbations are not yet consumed by the Rust acceptance runtime.
