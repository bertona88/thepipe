#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

cargo fmt --all -- --check
cargo test --locked -p pipe_sim \
  simple_manipulation::tests::m1c_cycle_grasps_carries_inserts_releases_and_retreats \
  -- --exact --nocapture
cargo test --locked -p pipe_sim \
  observed_manipulation::runtime::tests::nominal_m1e_completes_from_observations \
  -- --exact --nocapture
cargo test --locked -p pipe_sim \
  observed_manipulation::runtime::tests::every_injected_fault_stops_for_its_declared_reason \
  -- --exact --nocapture
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
mkdir -p out
cargo run --locked -p pipe_sim_cli -- \
  --scenario scenarios/gearbox_acceptance.json \
  --report out/gearbox_acceptance.json
cargo run --locked -p pipe_sim_cli --bin pipe-manipulation -- \
  --compact > out/m1c_manipulation.json
cargo run --locked -p pipe_sim_cli --bin pipe-optical-codesign -- \
  --compact > out/m1d_optical_codesign.json
cargo run --locked -p pipe_sim_cli --bin pipe-observed-manipulation -- \
  --scenario scenarios/observed_manipulation_m1e_v1.json \
  --compact > out/m1e_observed_state.json
cargo run --locked -p pipe_sim_cli --bin pipe-observed-manipulation -- \
  --scenario scenarios/observed_manipulation_m1e_v1.json \
  --compact > out/m1e_observed_state_replay.json
cmp out/m1e_observed_state.json out/m1e_observed_state_replay.json
