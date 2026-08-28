#!/usr/bin/env python3
"""Export an exploded insertion-order view of the gearbox."""

from __future__ import annotations

import sys
from pathlib import Path

CAD_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CAD_ROOT))

from pipe_cad.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main(["gearbox-exploded", *sys.argv[1:]]))
