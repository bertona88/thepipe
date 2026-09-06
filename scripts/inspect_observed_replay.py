#!/usr/bin/env python3
"""Validate a Rust replay and create an offline engineering inspector (stdlib only)."""
import argparse
import json
import math
from pathlib import Path
import re


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite_tree(value):
    if isinstance(value, float):
        require(math.isfinite(value), "nonfinite number")
    elif isinstance(value, dict):
        for item in value.values():
            finite_tree(item)
    elif isinstance(value, list):
        for item in value:
            finite_tree(item)


def vector(value, size):
    require(isinstance(value, list) and len(value) == size, "invalid vector")
    require(all(type(x) in (int, float) and math.isfinite(x) for x in value), "invalid vector element")


def pose(value):
    vector(value["translation_m"], 3)
    vector(value["rotation_xyzw"], 4)
    require(abs(sum(x*x for x in value["rotation_xyzw"])-1) < 1e-8, "nonunit quaternion")


def shape(value):
    kind = value["kind"]
    require(kind in ("box", "sphere", "capsule"), "unsupported geometry")
    if kind == "box":
        vector(value["half_extents_m"], 3)
        require(all(x > 0 for x in value["half_extents_m"]), "invalid box")
    else:
        require(value["radius_m"] > 0, "invalid radius")
        if kind == "capsule":
            require(value["half_segment_m"] >= 0, "invalid capsule")


def validate(data):
    finite_tree(data)
    require(data["schema_version"] == 1, "unsupported replay schema")
    require(re.fullmatch(r"[0-9a-fA-F]{40}", data["source_revision"]), "missing source revision")
    require(bool(data["generation_command"].strip()), "missing generation command")
    require(data["coordinate_frame"] == "pipe_world_right_handed_Z_tube_axis", "unsupported frame")
    require([data[k] for k in ("length_unit", "angle_unit", "time_unit")] == ["m", "rad", "s"], "unsupported units")
    require(type(data["sample_every_ticks"]) is int and data["sample_every_ticks"] > 0, "invalid sample stride")
    report = data["report"]
    require(report["schema_version"] in (1, 2), "unsupported report schema")
    require(report["schema_version"] == report["scenario_schema_version"], "mismatched report/scenario schema")
    for key in ("scenario_sha256", "machine_config_sha256", "controller_report_sha256"):
        require(re.fullmatch(r"[0-9a-f]{64}", report[key]), "missing configuration identity")
    require(report["status"] in ("complete", "failed_safe"), "nonterminal recording")
    require(report["evaluation_only_truth"] is not None, "missing terminal evaluation")
    require(report["roll_observable"] is False, "unsupported roll estimate")
    require((report["status"] == "complete" and report["terminal_reason"] is None)
            or (report["status"] == "failed_safe" and isinstance(report["terminal_reason"], str)), "terminal status/reason mismatch")
    dt = report["timing"]["fixed_step_s"]
    require(dt > 0 and report["timing"]["maximum_measurement_age_s"] > 0, "invalid timing")
    frames = data["frames"]
    require(isinstance(frames, list) and len(frames) >= 2, "missing replay frames")
    previous = -1
    for frame in frames:
        scene = frame["scene"]
        require(scene["schema_version"] == 1, "unsupported scene schema")
        tick = scene["tick"]
        require(type(tick) is int and tick > previous, "unordered or duplicate tick")
        previous = tick
        require(abs(scene["time_s"] - tick*dt) < 1e-7, "time/tick mismatch")
        require(scene["truth"] is not None and scene["estimate"] is None, "invalid truth/estimate mapping")
        pose(frame["physical_tool_pose"])
        pose(frame["socket_pose"])
        vector(frame["commanded_tool_position_world_m"], 3)
        if frame["commanded_tool_axis_world"] is not None:
            vector(frame["commanded_tool_axis_world"], 3)
        require(frame["contact_packet"]["captured_at_tick"] == tick, "stale contact packet")
        ids = set()
        for collider in frame["bodies"] + frame["physical_jaws"]:
            require(collider["geometry_id"] not in ids, "duplicate geometry identity")
            ids.add(collider["geometry_id"])
            pose(collider["pose"])
            shape(collider["shape"])
        body_map = {entry["body_id"]: entry for entry in frame["bodies"]}
        for body in scene["truth"]["rigid_bodies"]:
            require(body["id"] in body_map, "missing body geometry")
            require(body["pose"] == body_map[body["id"]]["pose"] and body["geometry_id"] == body_map[body["id"]]["geometry_id"], "geometry/state mismatch")
        for arm in scene["truth"]["manipulators"]:
            for collider in arm["link_colliders"]:
                pose(collider["pose"])
                shape(collider["shape"])
    require(frames[0]["scene"]["tick"] == 0, "missing initial state")
    require(report["decisions"] and report["decisions"][-1]["tick"] == previous, "missing terminal state")
    previous_update_tick = -1
    for index, update in enumerate(report["estimator_updates"]):
        estimate = update["estimate"]
        require(update["sequence"] == index, "unordered estimate update")
        require(previous_update_tick <= estimate["controller_tick"] <= previous, "estimate outside run or unordered")
        previous_update_tick = estimate["controller_tick"]
        if estimate["pose"] is not None:
            vector(estimate["pose"]["center_world_m"], 3)
            vector(estimate["pose"]["axis_world_unit"], 3)
            require(estimate["pose"]["roll_observable"] is False, "fabricated roll")
        if estimate["uncertainty"] is not None:
            vector(estimate["uncertainty"]["center_sigma_m"], 3)
            require(all(x >= 0 for x in estimate["uncertainty"]["center_sigma_m"]), "negative uncertainty")
    return data


def render(data):
    validate(data)
    template = Path(__file__).with_name("observed_replay_inspector.html").read_text()
    # Escape HTML delimiters even in diagnostic strings and source filenames.
    payload = json.dumps(data, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")
    return template.replace("__REPLAY_JSON__", payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    try:
        data = validate(json.loads(args.replay.read_text()))
        if not args.validate_only:
            require(args.output is not None, "--output is required")
            require(args.output.resolve() != args.replay.resolve(), "output would overwrite source")
            args.output.write_text(render(data))
        print(f"Validated {len(data['frames'])} exact frames; task status: {data['report']['status']}")
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Replay unavailable: {error}\n")


if __name__ == "__main__":
    main()
