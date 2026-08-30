#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

expected_bindgen="0.2.127"
site_dir="${1:-dist/chatgpt-site}"
wasm_dir="$repo_root/dist/wasm"

if ! command -v wasm-bindgen >/dev/null 2>&1; then
  echo "error: wasm-bindgen CLI is required for a production browser bundle" >&2
  echo "install with: cargo install wasm-bindgen-cli --version ${expected_bindgen} --locked" >&2
  exit 1
fi

actual_bindgen="$(wasm-bindgen --version | awk '{print $2}')"
if [[ "$actual_bindgen" != "$expected_bindgen" ]]; then
  echo "error: wasm-bindgen CLI ${actual_bindgen} does not match crate pin ${expected_bindgen}" >&2
  exit 1
fi

npm --prefix web run verify
./scripts/build_wasm.sh "$wasm_dir"

for required in \
  "$wasm_dir/pipe_sim_wasm.js" \
  "$wasm_dir/pipe_sim_wasm_bg.wasm"; do
  if [[ ! -f "$required" ]]; then
    echo "error: production WASM binding missing: $required" >&2
    exit 1
  fi
done

rm -rf "$site_dir"
mkdir -p "$site_dir/web" "$site_dir/dist"

for file in index.html styles.css app.js simulator-bridge.mjs; do
  cp "$repo_root/web/$file" "$site_dir/web/$file"
done
cp -R "$wasm_dir" "$site_dir/dist/wasm"
touch "$site_dir/.nojekyll"

cat > "$site_dir/index.html" <<'HTML'
<!doctype html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <meta name="robots" content="noindex" />
    <title>The Pipe — Microassembly Simulator</title>
    <meta http-equiv="refresh" content="0; url=./web/" />
    <script>
      const target = new URL("./web/", window.location.href);
      target.search = window.location.search;
      target.hash = window.location.hash;
      window.location.replace(target);
    </script>
  </head>
  <body>
    <p><a href="./web/">Open The Pipe microassembly simulator</a></p>
  </body>
</html>
HTML

node --check "$site_dir/web/app.js"
node --check "$site_dir/web/simulator-bridge.mjs"

if ! grep -Fq '../dist/wasm/pipe_sim_wasm.js' "$site_dir/web/app.js"; then
  echo "error: packaged layout no longer matches the operator console WASM import path" >&2
  exit 1
fi

printf 'Built ChatGPT Sites/static-host bundle at %s\n' "$site_dir"
find "$site_dir" -type f -print | sort
