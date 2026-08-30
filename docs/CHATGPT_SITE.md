# ChatGPT Sites / static deployment

The operator console is a static browser application backed by the same Rust reference simulator compiled to WebAssembly. No long-running application server is required for the interactive simulator.

## Build the production bundle

The Rust crate pins `wasm-bindgen = 0.2.127`, so use the matching CLI:

```bash
cargo install wasm-bindgen-cli --version 0.2.127 --locked
./scripts/build_chatgpt_site.sh
```

The build fails if the generated JavaScript bindings or WebAssembly binary are missing. The output is:

```text
dist/chatgpt-site/
├── index.html              # root redirect into the preserved /web layout
├── .nojekyll
├── web/
│   ├── index.html
│   ├── styles.css
│   ├── app.js
│   └── simulator-bridge.mjs
└── dist/wasm/
    ├── pipe_sim_wasm.js
    ├── pipe_sim_wasm_bg.wasm
    └── generated type/support files
```

The `web/` + `dist/wasm/` directory relationship is deliberate: the current operator console imports `../dist/wasm/pipe_sim_wasm.js`. Preserving that relationship avoids falling back to the visualization-only preview when the site is hosted from a static root.

## CI artifact

`.github/workflows/chatgpt-site.yml` builds the production bundle on this deployment branch and on manual dispatch, then uploads `the-pipe-chatgpt-site` as a GitHub Actions artifact.

## Publish with ChatGPT Sites

ChatGPT Sites creation and publishing happens from the Sites experience in ChatGPT Work on the web, or Work/Codex in the desktop app. Use this repository/branch and the generated `dist/chatgpt-site` bundle as the source, review the preview, then publish from the Sites sharing controls. The Sites publisher generates the production URL.

The repository workflow prepares and verifies the application bundle; it does not impersonate or bypass the account/workspace-level ChatGPT Sites publish control.

## Fidelity boundary

A successful browser load should show `RUST / WASM` in the MODEL badge. `UI PREVIEW` means the real WebAssembly wrapper did not load and the page is running the deterministic visualization fallback, which must not be treated as an acceptance result.
