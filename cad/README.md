# Pipe µ reduced CAD export

Generate a verified operation replay, then run:

```sh
python3 cad/export_micro.py out/pipe_micro.json --tick 100 --output out/pipe_micro_tick100
```

The SCAD and metadata contain the same metre-valued conservative envelopes used by
the runtime and optical ray tests; SCAD converts metres to millimetres explicitly.
Each part preserves its ID and replay/configuration identity. Missing or tampered
replay data fails closed. Tests compare several executed states.

This is inspection CAD, not fabrication-ready geometry. Actual flexure profiles,
tendon anchors/guides, chamber construction, seals and manufacturable tool structure
remain to be designed and checked against the same candidate. The old pinned-joint
CAD is available only at the pre-pivot tag and is not an active starting assembly.
