"""Lightweight assembly records used for exports and metadata."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class PartRecord:
    name: str
    category: str
    shape: Any
    material: str
    process: str
    printable: bool = True
    quantity: int = 1
    notes: tuple[str, ...] = field(default_factory=tuple)
    # Set only for genuinely homogeneous records.  Mixed/purchased assemblies
    # remain None so metadata never invents a validated mass.
    density_g_mm3: float | None = None
