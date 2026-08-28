#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

cargo fmt --all -- --check
cargo test --locked --workspace
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo build --locked --release --target wasm32-unknown-unknown -p pipe_sim_wasm

python_bin="python3"
if [[ -x .venv/bin/python ]]; then
  python_bin=".venv/bin/python"
fi

"$python_bin" -m pytest cad/tests
mkdir -p cad/out
"$python_bin" cad/scripts/validate_parameters.py > cad/out/manifest.json
cargo run --locked -p pipe_sim_cli -- \
  --scenario scenarios/gearbox_acceptance.json \
  --report out/gearbox_acceptance.json
