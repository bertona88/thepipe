"""Command-line export entry point."""

from __future__ import annotations

import argparse
from pathlib import Path

from .assemblies import (
    benchmark_records,
    cell_records,
    gearbox_records,
    printable_catalog_records,
    tooling_records,
)
from .export import export_assembly
from .params import default_design


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "assembly",
        choices=("cell", "gearbox", "gearbox-exploded", "benchmark", "tooling", "catalog", "all"),
        nargs="?",
        default="all",
    )
    result.add_argument("--output", type=Path, default=Path("build/cad"))
    result.add_argument("--individual", action="store_true")
    result.add_argument(
        "--stl-tolerance",
        type=float,
        default=None,
        help="override linear STL deflection in mm; STEP remains exact",
    )
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    config = default_design()
    jobs = {
        "cell": (cell_records, 0.02),
        "gearbox": (lambda c: gearbox_records(c, False), 0.003),
        "gearbox-exploded": (lambda c: gearbox_records(c, True), 0.003),
        "benchmark": (benchmark_records, 0.005),
        "tooling": (tooling_records, 0.01),
        "catalog": (printable_catalog_records, 0.003),
    }
    selected = jobs if args.assembly == "all" else {args.assembly: jobs[args.assembly]}
    for name, (builder, tolerance) in selected.items():
        paths = export_assembly(
            builder(config),
            config,
            args.output,
            name,
            tolerance=args.stl_tolerance if args.stl_tolerance is not None else tolerance,
            individual=args.individual or name == "catalog",
        )
        print(f"{name}: " + ", ".join(f"{kind}={path}" for kind, path in paths.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
