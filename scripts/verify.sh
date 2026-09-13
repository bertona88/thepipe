#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m unittest discover -s tests -v
python3 -m pipe_micro --report out/pipe_micro.json
python3 -m pipe_micro --verify-replay out/pipe_micro.json
python3 cad/export_micro.py out/pipe_micro.json --output out/pipe_micro_envelopes
