#!/usr/bin/env python3
"""Build a clean revision and record the actual observed arm experiment."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("out/gear-pickup"))
    parser.add_argument("--config", default="scenarios/machine_gear_pickup_v1.json")
    parser.add_argument("--fault-suite", action="store_true")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if git(root, "status", "--porcelain", "--untracked-files=normal").strip():
        raise SystemExit("Record from a clean checkout; commit the implementation first.")
    git(root, "ls-files", "--error-unmatch", args.config)
    revision = git(root, "rev-parse", "HEAD").decode().strip()
    # Exact definition: SHA-256 of Git's recursive full-tree listing, including
    # modes, types, blob IDs and paths. This is not a hash of a rendered image.
    tree_hash = hashlib.sha256(git(root, "ls-tree", "-r", "--full-tree", "HEAD")).hexdigest()
    subprocess.run(["cargo", "build", "--locked", "--release", "-p", "pipe_sim_cli",
                    "--bin", "pipe-gear-pickup"], cwd=root, check=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    binary = root / "target/release/pipe-gear-pickup"
    faults = [None]
    if args.fault_suite:
        faults += ["missing-part", "stale", "timeout", "unsupported-release"]
    def record_case(fault):
        name = fault or "nominal"
        path = output / f"{name}.json"
        command = [str(binary), str(root / args.config), revision, tree_hash, str(path)]
        if fault:
            command.append(fault)
        subprocess.run(command, cwd=root, check=True)
        content = path.read_bytes()
        report = json.loads(content)
        return {
            "case": name, "status": report["status"], "refusal": report["refusal"],
            "completed_phases": report["completed_phases"],
            "sample_count": len(report["samples"]),
            "final_tick": report["samples"][-1]["tick"],
            "simulated_duration_s": report["samples"][-1]["time_s"],
            "machine_config_sha256": report["machine_config_sha256"],
            "configuration_sha256": report["configuration_sha256"],
            "report_sha256": hashlib.sha256(content).hexdigest(),
            "release_support_gaps_m": next((s["support_gaps_m"] for s in report["samples"]
                if s["phase"] == "supported_release"), None),
            "insertion_precision_rejections": sorted({s["insertion_precision_rejection"]
                for s in report["samples"] if s.get("insertion_precision_rejection")}),
            "report_path": path.name, "fidelity": report["fidelity"],
        }
    # Independent processes share only the immutable binary and source. Preserve
    # case order in the summary while limiting simultaneous large replay files.
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(record_case, faults))
    fidelity = results[0].pop("fidelity")
    for result in results[1:]:
        if result.pop("fidelity") != fidelity:
            raise SystemExit("Fault case fidelity differs from nominal configuration")
    summary = {"source_revision": revision, "source_tree_sha256": tree_hash,
               "source_tree_hash_definition": "SHA256(git ls-tree -r --full-tree HEAD output bytes)",
               "configuration": args.config, "results": results,
               "fidelity": fidelity}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    if results[0]["status"] != "completed_pickup_return":
        raise SystemExit("Nominal operation refused; evidence retained, success not claimed.")
    if any(r["status"] != "controlled_refusal" for r in results[1:]):
        raise SystemExit("Fault suite did not produce controlled refusals.")


if __name__ == "__main__":
    main()
