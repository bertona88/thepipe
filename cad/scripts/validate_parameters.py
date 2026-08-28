#!/usr/bin/env python3
"""Run kernel-free dimension and gear-profile checks."""

from __future__ import annotations

import json
import sys
from pathlib import Path

CAD_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CAD_ROOT))

from pipe_cad.gear_math import involute_gear_outline  # noqa: E402
from pipe_cad.params import GEARBOX_INSERTION_SEQUENCE, default_design  # noqa: E402


def main() -> int:
    config = default_design()
    g = config.gearbox
    profiles = [
        involute_gear_outline(
            teeth,
            g.module,
            g.pressure_angle_deg,
            g.backlash,
        )
        for teeth in (g.input_teeth, g.idler_teeth, g.output_teeth)
    ]
    print(
        json.dumps(
            {
                "status": "ok",
                "tube_inner_diameter_mm": 2 * config.tube.inner_radius,
                "usable_tube_length_mm": config.tube.usable_length,
                "mobile_arm_count": config.tube.rail_count,
                "serial_arm_lengths_mm": list(config.arm.link_lengths),
                "differential_capstan_channels": list(config.arm.actuator_axes),
                "tendon_line_diameter_mm": config.arm.tendon_diameter,
                "capstan_radius_mm": config.arm.spool_radius,
                "global_camera_count": config.sensing.global_camera_count,
                "simultaneous_macro_views": config.sensing.simultaneous_macro_view_count,
                "structured_light_projector_count": (
                    config.sensing.structured_light_projector_count
                ),
                "gear_pitch_diameters_mm": [
                    round(2 * profile.pitch_radius, 6) for profile in profiles
                ],
                "gearbox_ratio": g.ratio,
                "gear_pressure_angle_deg": g.pressure_angle_deg,
                "gear_backlash_mm": g.backlash,
                "gear_bore_mm": g.bore_diameter,
                "shaft_diameter_x_length_mm": [g.shaft_diameter, g.shaft_length],
                "gear_centers_x_mm": [
                    g.input_center_x,
                    g.idler_center_x,
                    g.output_center_x,
                ],
                "gearbox_envelope_mm": [
                    g.housing_length,
                    g.housing_width,
                    g.housing_height + g.lid_thickness + g.datum_pad_height,
                ],
                "gearbox_body_floor_cover_mm": [
                    g.housing_height,
                    g.housing_floor,
                    g.lid_thickness,
                ],
                "gear_face_total_height_mm": [
                    g.gear_thickness,
                    g.total_gear_height,
                ],
                "cover_under_clearance_mm": g.cover_under_clearance,
                "shaft_seat_diameter_x_depth_mm": [
                    g.shaft_seat_diameter,
                    g.shaft_seat_depth,
                ],
                "cover_windows_mm": [
                    g.input_driver_window_diameter,
                    g.output_observation_window_diameter,
                ],
                "insertion_sequence": list(GEARBOX_INSERTION_SEQUENCE),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
