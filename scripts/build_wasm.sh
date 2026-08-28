#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

out_dir="${1:-dist/wasm}"
mkdir -p "$out_dir"

cargo build --locked --release --target wasm32-unknown-unknown -p pipe_sim_wasm
wasm_path="target/wasm32-unknown-unknown/release/pipe_sim_wasm.wasm"

if command -v wasm-bindgen >/dev/null 2>&1; then
  wasm-bindgen \
    --target web \
    --typescript \
    --out-dir "$out_dir" \
    "$wasm_path"
else
  cp "$wasm_path" "$out_dir/pipe_sim_wasm.wasm"
  echo "wasm-bindgen CLI not installed; emitted the raw module only" >&2
fi
