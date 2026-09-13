# The Pipe µ

A cylindrical controlled microassembly workspace with small printed compliant tools,
remote tendon actuation, external observation and cooperative handling of 300 µm–1 mm
objects. The Pipe µ is the sole active architecture.

The implemented default is a **synthetic native-surface coupon transfer**, including
bounded shared z/θ positioning, pickup, dual support, donor unloading, observed
separation, withdrawal and destination inspection. Physical, observed and control
state are separate. No hardware accuracy or reliability has been demonstrated.

```sh
python3 -m pipe_micro --report out/pipe_micro.json
python3 -m pipe_micro --verify-replay out/pipe_micro.json
python3 cad/export_micro.py out/pipe_micro.json --output out/pipe_micro_envelopes
bash scripts/verify.sh
```

Python 3.12 standard library is sufficient. Fault studies use, for example,
`python3 -m pipe_micro --fault donor_adhesion --report out/stuck.json` and return
exit code 2 on a modeled refusal. Invalid configuration/replay returns 1.
There is one default operation; old scenarios and aliases are rejected.

Read [requirements](docs/REQUIREMENTS.md), [architecture](docs/ARCHITECTURE.md),
[implementation status](docs/IMPLEMENTATION_STATUS.md), and the
[candidate decision](docs/decisions/NEAR_WALL_V1.md). The [design atlas](docs/the-pipe-micro/README.md)
retains the broader design space. The [reset decision](docs/decisions/PIPE_MICRO_RESET.md)
and [migration record](docs/migration/README.md) explain the cutover.

Current limits: reduced translational flexures and contact law; no calibrated resin,
full tendon routing, fabricated assembly, camera reconstruction, measured orientation,
or hardware qualification. CAD export contains conservative inspection envelopes,
not fabrication solids. The Rust SI/math and optics crates remain independent
libraries; `cargo test --locked --workspace` tests their own historical domains.
There is no active WASM frontend or legacy machine runtime.
