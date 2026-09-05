#!/usr/bin/env python3
"""Execute the versioned M1f grid through the authoritative native CLI.

Only terminal evaluation scores can determine physical success. Exploratory
refusals remain visible and are never counted as successful manipulation.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def run_case(case, baseline, binary, directory):
    scenario = copy.deepcopy(baseline)
    for path, value in case["patch"].items():
        keys = path.split(".")
        cursor = scenario
        for key in keys[:-1]:
            cursor = cursor[key]
        if keys[-1] not in cursor:
            raise ValueError(f"Unknown scenario field: {path}")
        cursor[keys[-1]] = value
    scenario["id"] = f"m1f_robustness_{case['id']}"
    path = directory / f"{case['id']}.json"
    path.write_text(json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    command = [str(binary), "--scenario", str(path), "--compact"]
    if case.get("fault"):
        command += ["--fault", case["fault"]]
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=300)
    if result.returncode == 1 and "invalid scenario:" in result.stderr:
        return {
            "id": case["id"], "expectation": case["expect"], "patch": case["patch"],
            "fault": case.get("fault", "none"), "status": "rejected_before_execution",
            "terminal_reason": result.stderr.strip().split("invalid scenario:", 1)[1].strip(),
            "completed": False, "safe_stop": False,
            "acceptance_passed": case["expect"] in ("explore", "invalid_scenario"),
            "replay_equal": None,
        }
    if result.returncode not in (0, 2):
        raise RuntimeError(f"{case['id']}: CLI error {result.returncode}: {result.stderr}")
    report = json.loads(result.stdout)
    scoring = report["evaluation_only_truth"]
    gates = {gate["id"]: gate for gate in report["acceptance_gates"]}
    safe = (
        report["schema_version"] == 2
        and report["timing"]["stale_near_contact_command_count"] == 0
        and scoring["maximum_unplanned_penetration_m"] == 0.0
        and all(gates[key]["passed"] for key in
                ("no_controller_truth_access", "force_proxy_limit", "explicit_fidelity_contract"))
        and scoring["peak_grip_force_proxy_n"] <= scenario["grasp"]["maximum_grip_force_n"]
        and scoring["peak_insertion_force_proxy_n"] <= scenario["contact"]["maximum_force_proxy_n"]
    )
    complete = (
        safe and report["status"] == "complete" and result.returncode == 0
        and all(gate["passed"] for gate in gates.values())
        and scoring["within_declared_seat_tolerances"] and scoring["physical_release_verified"]
    )
    safe_stop = safe and report["status"] == "failed_safe" and any(
        decision["action"] == "fail_closed_hold" and decision["command_sequence"] is not None
        for decision in report["decisions"]
    )
    expectation = case["expect"]
    passed = complete if expectation == "complete" else (
        complete or safe_stop if expectation == "explore" else
        safe_stop and report["terminal_reason"] == expectation and result.returncode == 0
    )
    replay_equal = None
    if case.get("replay"):
        replay = subprocess.run(command, capture_output=True, text=True, check=False, timeout=300)
        replay_equal = replay.returncode == result.returncode and replay.stdout == result.stdout
        passed = passed and replay_equal
    metrics = report["metrics"]
    return {
        "id": case["id"], "expectation": expectation, "patch": case["patch"],
        "fault": case.get("fault", "none"), "status": report["status"],
        "terminal_reason": report["terminal_reason"], "completed": complete,
        "safe_stop": safe_stop, "acceptance_passed": passed, "replay_equal": replay_equal,
        "scenario_sha256": report["scenario_sha256"],
        "controller_report_sha256": report["controller_report_sha256"],
        "full_report_sha256": hashlib.sha256(result.stdout.encode()).hexdigest(),
        "elapsed_simulation_s": report["decisions"][-1]["time_s"],
        "axis_correction_count": sum(d["action"] == "axis_correction" for d in report["decisions"]),
        "maximum_accepted_position_sigma_m": metrics["maximum_accepted_position_sigma_m"],
        "maximum_accepted_axis_sigma_rad": metrics["maximum_accepted_axis_sigma_rad"],
        "maximum_grip_force_proxy_n": metrics["maximum_grip_force_proxy_n"],
        "maximum_insertion_force_proxy_n": metrics["maximum_insertion_force_proxy_n"],
        "evaluation_only_truth": scoring,
    }


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, default=root / "scenarios/robustness_m1f_v1.json")
    parser.add_argument("--binary", type=Path, default=root / "target/debug/pipe-observed-manipulation")
    parser.add_argument("--report", type=Path, default=root / "out/m1f_robustness.json")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 8:
        parser.error("--jobs must be between 1 and 8")
    matrix = json.loads(args.matrix.read_text(encoding="utf-8"))
    baseline_path = args.matrix.resolve().parent / matrix["baseline_scenario"]
    if args.report.resolve() in (args.matrix.resolve(), baseline_path.resolve()):
        parser.error("report path cannot replace a source scenario or matrix")
    if matrix["schema_version"] != 1:
        parser.error("unsupported matrix schema")
    cases = matrix["cases"]
    if not cases or len(cases) > 1000 or len({c["id"] for c in cases}) != len(cases):
        parser.error("matrix must contain bounded, uniquely named cases")
    if any(not c["id"] or any(x not in "abcdefghijklmnopqrstuvwxyz0123456789_+-" for x in c["id"]) for c in cases):
        parser.error("case IDs must be simple filenames")
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="m1f-", dir=args.report.parent) as temporary:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            rows = list(pool.map(lambda case: run_case(case, baseline, args.binary.resolve(), Path(temporary)), cases))
    grid = [r for r in rows if r["id"].startswith("grid_")]
    completed_grid = sum(r["completed"] for r in grid)
    passed = all(r["acceptance_passed"] for r in rows) and completed_grid >= matrix["minimum_completed_grid_cases"]
    report = {
        "schema_version": 1, "fidelity": "F1_reduced_M1f_not_hardware_qualified",
        "matrix_sha256": hashlib.sha256(args.matrix.read_bytes()).hexdigest(),
        "baseline_sha256": hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
        "interpretation": matrix["interpretation"], "acceptance_passed": passed,
        "case_count": len(rows), "completed_count": sum(r["completed"] for r in rows),
        "safe_stop_count": sum(r["safe_stop"] for r in rows),
        "rejected_before_execution_count": sum(r["status"] == "rejected_before_execution" for r in rows),
        "grid_case_count": len(grid), "grid_completed_count": completed_grid,
        "minimum_completed_grid_cases": matrix["minimum_completed_grid_cases"], "cases": rows,
    }
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"M1f: {report['completed_count']} completed, {report['safe_stop_count']} safe stops, {report['rejected_before_execution_count']} scenario refusals; grid {completed_grid}/{len(grid)}; acceptance={passed}")
    for row in rows:
        if not row["completed"]:
            print(f"  {row['id']}: {row['terminal_reason']} (gate={row['acceptance_passed']})")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
