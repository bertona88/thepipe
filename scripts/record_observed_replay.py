#!/usr/bin/env python3
"""Build the current clean Git revision, record M1f, and create its inspector."""
import argparse
import json
from pathlib import Path
import subprocess

from inspect_observed_replay import render, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("out/observed-inspection"))
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--fault")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    try:
        status = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=root, text=True)
        if status:
            raise ValueError("source revision is dirty; commit the intended source before recording provenance")
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
        version = subprocess.check_output(["rustc", "-vV"], cwd=root, text=True)
        host = next(line.removeprefix("host: ") for line in version.splitlines() if line.startswith("host: "))
        target_dir = root / "target"
        subprocess.run(["cargo", "build", "--locked", "--release", "--target", host,
                        "--target-dir", str(target_dir), "-p", "pipe_sim_cli", "--bin", "pipe-observed-replay"], cwd=root, check=True)
        executable = "pipe-observed-replay.exe" if "windows" in host else "pipe-observed-replay"
        command = [str(target_dir / host / "release" / executable), "--source-revision", revision]
        if args.scenario:
            command += ["--scenario", str(args.scenario.resolve())]
        if args.fault:
            command += ["--fault", args.fault]
        result = subprocess.run(command, cwd=root, stdout=subprocess.PIPE, check=False)
        if result.returncode not in (0, 2):
            raise ValueError(f"runtime refused recording (exit {result.returncode})")
        if subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=root, text=True) or subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip() != revision:
            raise ValueError("source changed during recording; provenance is unavailable")
        data = validate(json.loads(result.stdout))
        inspector = render(data)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "replay.json").write_bytes(result.stdout)
        output = args.output_dir / "inspector.html"
        output.write_text(inspector)
        print(f"Recorded {revision}: {data['report']['status']}\nOpen {output.resolve()}")
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Recording unavailable: {error}\n")


if __name__ == "__main__":
    main()
