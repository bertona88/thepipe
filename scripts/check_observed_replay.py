#!/usr/bin/env python3
"""Exercise inspector admission against an actual Rust-generated recording."""
import copy
import json
from pathlib import Path
import subprocess
import sys

from inspect_observed_replay import render, validate


def main():
    source = Path(sys.argv[1])
    data = json.loads(source.read_text())
    validate(data)
    mutations = {
        "unsupported schema": lambda d: d.update(schema_version=99),
        "wrong units": lambda d: d.update(length_unit="mm"),
        "missing truth": lambda d: d["frames"][0]["scene"].update(truth=None),
        "truth in estimate": lambda d: d["frames"][0]["scene"].update(estimate=d["frames"][0]["scene"]["truth"]),
        "missing geometry": lambda d: d["frames"][0].update(bodies=[]),
        "unsupported geometry": lambda d: d["frames"][0]["bodies"][0]["shape"].update(kind="imaginary"),
        "duplicate tick": lambda d: d["frames"][1]["scene"].update(tick=0),
        "wrong timestamp": lambda d: d["frames"][0]["scene"].update(time_s=1),
        "stale contact": lambda d: d["frames"][-1]["contact_packet"].update(captured_at_tick=0),
        "missing final frame": lambda d: d["frames"].pop(),
        "unsupported report": lambda d: d["report"].update(schema_version=999),
        "nonterminal": lambda d: d["report"].update(status="running"),
        "unproven roll": lambda d: d["report"].update(roll_observable=True),
        "nonfinite coordinate": lambda d: d["frames"][0]["physical_tool_pose"]["translation_m"].__setitem__(0,float("nan")),
    }
    for name, mutate in mutations.items():
        corrupted = copy.deepcopy(data)
        mutate(corrupted)
        try:
            validate(corrupted)
        except (ValueError, KeyError, TypeError):
            pass
        else:
            raise AssertionError(f"admitted {name}")
    text = copy.deepcopy(data)
    text["generation_command"] = '</script><script>alert("bad")</script>'
    assert text["generation_command"] not in render(text)
    subprocess.run(["node", str(Path(__file__).with_name("check_observed_replay_js.cjs")), str(source)], check=True)
    print(f"Replay admission: {len(mutations)} malformed cases refused; embedded text escaped")


if __name__ == "__main__":
    main()
