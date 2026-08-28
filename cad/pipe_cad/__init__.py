"""Parametric CAD for the observed-volume tube assembly cell.

The top-level package deliberately has no build123d import.  Parameter and
gear-profile checks therefore remain usable on controllers that do not carry
the OpenCascade runtime.
"""

from .gear_math import GearOutline, involute_gear_outline
from .params import GEARBOX_INSERTION_SEQUENCE, DesignConfig, default_design

__all__ = [
    "DesignConfig",
    "GearOutline",
    "GEARBOX_INSERTION_SEQUENCE",
    "default_design",
    "involute_gear_outline",
]
