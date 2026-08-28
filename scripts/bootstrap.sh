#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if ! command -v rustup >/dev/null 2>&1; then
  echo "rustup is required: install stable Rust from https://rustup.rs" >&2
  exit 2
fi

rustup toolchain install stable --profile minimal --component rustfmt --component clippy
rustup target add wasm32-unknown-unknown --toolchain stable

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r cad/requirements.txt

echo "Toolchains ready. Run ./scripts/verify.sh"
