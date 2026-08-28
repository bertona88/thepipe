#!/usr/bin/env python3
"""Export the closed nominal 2PP gearbox and all individual items."""

from __future__ import annotations

import sys
from pathlib import Path

CAD_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CAD_ROOT))

from pipe_cad.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main(["gearbox", "--individual", *sys.argv[1:]]))
