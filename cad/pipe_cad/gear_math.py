"""Kernel-free involute spur-gear profile generation."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan, cos, pi, radians, sin, sqrt


Point2 = tuple[float, float]


@dataclass(frozen=True)
class GearOutline:
    points: tuple[Point2, ...]
    tooth_count: int
    module: float
    pressure_angle_deg: float
    backlash: float
    pitch_tooth_thickness: float
    pitch_radius: float
    base_radius: float
    root_radius: float
    tip_radius: float


def _polar(radius: float, angle: float) -> Point2:
    return radius * cos(angle), radius * sin(angle)


def _linspace(start: float, stop: float, count: int) -> list[float]:
    if count <= 1:
        return [start]
    return [start + (stop - start) * i / (count - 1) for i in range(count)]


def involute_gear_outline(
    tooth_count: int,
    module: float,
    pressure_angle_deg: float = 25.0,
    backlash: float = 0.020,
    flank_samples: int = 6,
    arc_samples: int = 3,
) -> GearOutline:
    """Return a CCW, non-repeated outline for an external spur gear.

    The flank is an analytical involute sampled radially.  Below the base
    circle the profile uses a conservative radial root transition; this avoids
    a fragile trochoid in tiny printed gears and intentionally leaves extra
    root clearance. ``backlash`` is the desired assembled-pair circular
    backlash at the pitch circle. Each mating gear contributes half of the
    total thinning, so a pair generated with the same value has that value of
    play rather than twice that value.
    """

    if tooth_count < 8:
        raise ValueError("tooth_count must be at least 8")
    if module <= 0:
        raise ValueError("module must be positive")
    if not 10.0 <= pressure_angle_deg <= 35.0:
        raise ValueError("pressure angle must be between 10 and 35 degrees")
    if backlash < 0 or backlash >= pi * module / 2:
        raise ValueError("backlash must be non-negative and less than half pitch")
    if flank_samples < 3 or arc_samples < 2:
        raise ValueError("insufficient profile samples")

    pressure = radians(pressure_angle_deg)
    pitch_radius = module * tooth_count / 2
    base_radius = pitch_radius * cos(pressure)
    root_radius = pitch_radius - 1.25 * module
    tip_radius = pitch_radius + module
    pitch_angle = 2 * pi / tooth_count
    tooth_thickness = pi * module / 2 - backlash / 2
    pitch_half_angle = tooth_thickness / (2 * pitch_radius)

    def involute_angle(radius: float) -> float:
        parameter = sqrt(max((radius / base_radius) ** 2 - 1, 0.0))
        return parameter - atan(parameter)

    pitch_involute = involute_angle(pitch_radius)
    base_half_angle = pitch_half_angle + pitch_involute
    tip_half_angle = pitch_half_angle + pitch_involute - involute_angle(tip_radius)
    if tip_half_angle <= 0:
        raise ValueError("backlash and tooth geometry collapse the tooth tip")
    if 2 * base_half_angle >= pitch_angle:
        raise ValueError("adjacent tooth roots overlap")

    flank_radii = _linspace(base_radius, tip_radius, flank_samples)
    points: list[Point2] = []
    for tooth in range(tooth_count):
        center = tooth * pitch_angle

        # Root-to-base transition at the leading flank.
        points.append(_polar(root_radius, center - base_half_angle))
        for radius in flank_radii:
            half_angle = pitch_half_angle + pitch_involute - involute_angle(radius)
            points.append(_polar(radius, center - half_angle))

        # Rounded-by-faceting crest between the two involute flanks.
        for angle in _linspace(center - tip_half_angle, center + tip_half_angle, arc_samples)[1:]:
            points.append(_polar(tip_radius, angle))

        for radius in reversed(flank_radii[:-1]):
            half_angle = pitch_half_angle + pitch_involute - involute_angle(radius)
            points.append(_polar(radius, center + half_angle))
        points.append(_polar(root_radius, center + base_half_angle))

        # Root land ends at the leading transition of the following tooth.
        next_leading = center + pitch_angle - base_half_angle
        for angle in _linspace(center + base_half_angle, next_leading, arc_samples)[1:]:
            points.append(_polar(root_radius, angle))

    return GearOutline(
        points=tuple(points),
        tooth_count=tooth_count,
        module=module,
        pressure_angle_deg=pressure_angle_deg,
        backlash=backlash,
        pitch_tooth_thickness=tooth_thickness,
        pitch_radius=pitch_radius,
        base_radius=base_radius,
        root_radius=root_radius,
        tip_radius=tip_radius,
    )


def transverse_contact_ratio(
    driver_teeth: int,
    driven_teeth: int,
    module: float,
    pressure_angle_deg: float = 25.0,
) -> float:
    """Standard-centre transverse contact ratio for two external gears."""

    if min(driver_teeth, driven_teeth) < 8 or module <= 0:
        raise ValueError("invalid external gear pair")
    pressure = radians(pressure_angle_deg)
    pitch_driver = module * driver_teeth / 2
    pitch_driven = module * driven_teeth / 2
    base_driver = pitch_driver * cos(pressure)
    base_driven = pitch_driven * cos(pressure)
    tip_driver = pitch_driver + module
    tip_driven = pitch_driven + module
    center_distance = pitch_driver + pitch_driven
    path_of_contact = (
        sqrt(tip_driver**2 - base_driver**2)
        + sqrt(tip_driven**2 - base_driven**2)
        - center_distance * sin(pressure)
    )
    base_pitch = pi * module * cos(pressure)
    return path_of_contact / base_pitch
