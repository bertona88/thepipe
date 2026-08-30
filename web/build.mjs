import { cp, mkdir, readFile, rm } from "node:fs/promises";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

const source = resolve(fileURLToPath(new URL(".", import.meta.url)));
const output = resolve(source, "dist");

await rm(output, { recursive: true, force: true });
await mkdir(output, { recursive: true });

for (const file of ["index.html", "styles.css", "app.js", "simulator-bridge.mjs"]) {
  await cp(resolve(source, file), resolve(output, file));
}

let copiedWasm = false;
try {
  await cp(resolve(source, "..", "dist", "wasm"), resolve(output, "wasm"), { recursive: true });
  copiedWasm = true;
} catch (error) {
  if (error.code !== "ENOENT") throw error;
}

const index = await readFile(resolve(output, "index.html"), "utf8");
for (const expected of ["styles.css", "app.js", "simulator-canvas", "run-toggle"]) {
  if (!index.includes(expected)) {
    throw new Error(`build validation failed: index.html is missing ${expected}`);
  }
}

console.log(`Built static operator console in ${output}${copiedWasm ? " with Rust/WASM wrapper" : " (UI preview only; Rust/WASM wrapper not found)"}`);
