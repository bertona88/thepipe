"""Single source of truth for all physical dimensions.

Units are millimetres and degrees.  Dataclasses are frozen so one export run
cannot accidentally mutate a value half-way through the assembly.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import cos, pi, radians
from typing import Any


GEARBOX_INSERTION_SEQUENCE = ("S1", "S2", "S3", "G3", "G2", "G1", "cover")


@dataclass(frozen=True)
class ManufacturingParams:
    process: str = "405_nm_MSLA_resin"
    xy_pixel: float = 0.035
    layer_height: float = 0.025
    minimum_wall: float = 0.45
    minimum_printed_gap: float = 0.08
    minimum_supported_feature: float = 0.18
    stl_linear_tolerance: float = 0.01
    stl_angular_tolerance_deg: float = 0.15
    resin_shrink_scale: float = 1.006
    nominal_resin_density_g_mm3: float = 0.00118
    nominal_petg_density_g_mm3: float = 0.00127


@dataclass(frozen=True)
class TubeParams:
    inner_radius: float = 80.0
    wall: float = 3.0
    length: float = 330.0
    rail_count: int = 4
    end_margin: float = 5.0
    material: str = "clear_acrylic_tube_or_printed_quadrants"

    @property
    def outer_radius(self) -> float:
        return self.inner_radius + self.wall

    @property
    def usable_length(self) -> float:
        return self.length - 2 * self.end_margin


@dataclass(frozen=True)
class RailParams:
    radial_depth: float = 3.0
    tangential_width: float = 6.0
    wall_standoff: float = 0.6
    carriage_clearance: float = 0.25
    material: str = "3x6_mm_pultruded_carbon_or_printed_PETG"


@dataclass(frozen=True)
class CarriageParams:
    radial_depth: float = 5.0
    tangential_width: float = 9.0
    axial_length: float = 42.0
    deck_reach: float = 10.0
    deck_width: float = 8.0
    deck_thickness: float = 3.0
    pivot_hole_diameter: float = 1.30
    fastener_hole_diameter: float = 1.70
    material: str = "tough_MSLA_resin"


@dataclass(frozen=True)
class ArmParams:
    link_count: int = 3
    link_lengths: tuple[float, ...] = (32.0, 30.0, 15.0)
    link_width: float = 5.6
    link_thickness: float = 2.4
    joint_radius: float = 3.4
    hinge_pin_diameter: float = 1.20
    hinge_bore_clearance: float = 0.10
    actuator_count: int = 4
    actuator_axes: tuple[str, ...] = (
        "shoulder_yaw",
        "shoulder_pitch",
        "elbow_pitch",
        "wrist_roll",
    )
    # The shoulder axis datum is intentionally independent of the rail-body
    # centerline.  The carriage deck bridges the 4.9 mm radial offset in the
    # qualified 160 mm-ID cell.
    shoulder_root_radius: float = 72.0
    shoulder_yaw_stage_radius: float = 4.8
    shoulder_yaw_stage_height: float = 3.0
    pitch_yoke_span: float = 7.2
    pitch_yoke_thickness: float = 1.2
    wrist_roll_outer_diameter: float = 4.8
    wrist_roll_length: float = 5.0
    tendon_diameter: float = 0.20
    tendon_channel_diameter: float = 0.90
    tendon_offset: float = 1.65
    spool_radius: float = 3.0
    spool_width: float = 4.2
    usable_tendon_payout: float = 12.0
    motor_diameter: float = 6.0
    motor_length: float = 15.0
    gearhead_diameter: float = 7.0
    gearhead_length: float = 5.5
    actuator_spacing: float = 10.0
    # Axis order is actuator_axes: yaw about +Z, both pitch axes about the
    # yaw-rotated +Y axis, and wrist roll about the final link +X axis.
    default_joint_angles_deg: tuple[float, ...] = (0.0, -12.0, 22.0, 0.0)
    material: str = "tough_MSLA_resin"
    tendon_material: str = "0.20_mm_UHMWPE_line"


@dataclass(frozen=True)
class GripperParams:
    palm_length: float = 6.0
    palm_width: float = 6.0
    palm_thickness: float = 2.4
    jaw_length: float = 8.0
    jaw_width: float = 1.4
    jaw_thickness: float = 2.0
    jaw_opening: float = 2.8
    tip_width: float = 0.65
    pivot_diameter: float = 1.0
    tendon_channel_diameter: float = 0.75
    material: str = "tough_MSLA_resin_with_silicone_tip_dip"


@dataclass(frozen=True)
class SensingParams:
    global_camera_count: int = 6
    simultaneous_macro_view_count: int = 2
    structured_light_projector_count: int = 1
    global_image_width_px: int = 1280
    global_image_height_px: int = 800
    global_horizontal_fov_deg: float = 68.0
    macro_image_width_px: int = 2048
    macro_image_height_px: int = 1536
    macro_field_width: float = 4.0
    macro_field_height: float = 3.0
    macro_pixel_scale: float = 0.002
    depth_quantization: float = 0.00025
    pixel_sigma_px: float = 0.18
    dropout_probability: float = 0.002
    # Optical front-face datums in the tube frame. The two triplets are
    # expressed relative to the central gearbox datum at tube length / 2.
    global_camera_front_radius: float = 60.0
    global_camera_end_offsets: tuple[float, float] = (-106.0, 106.0)
    global_camera_triplet_azimuths: tuple[tuple[float, ...], ...] = (
        (0.0, 120.0, 240.0),
        (60.0, 180.0, 300.0),
    )
    projector_front_radius: float = 60.0
    projector_azimuth_deg: float = 90.0
    projector_z_offset: float = 0.0
    macro_stereo_baseline: float = 12.0
    macro_mount_arm_index: int = 1
    macro_mount_normal_offset: float = 11.0
    camera_board_depth: float = 2.0
    camera_board_width: float = 24.0
    camera_board_height: float = 24.0
    camera_pod_depth: float = 8.0
    camera_pod_width: float = 27.0
    camera_pod_height: float = 27.0
    camera_lens_diameter: float = 8.0
    camera_aperture_diameter: float = 6.6
    projector_depth: float = 10.0
    projector_width: float = 16.0
    projector_height: float = 12.0
    projector_aperture_diameter: float = 5.2
    macro_pod_depth: float = 6.0
    macro_pod_width: float = 10.0
    macro_pod_height: float = 10.0
    macro_lens_diameter: float = 3.2
    macro_aperture_diameter: float = 2.6
    case_wall: float = 1.0
    fiducial_size: float = 12.0
    fiducial_thickness: float = 0.7
    fiducial_cells: int = 6
    fiducial_relief: float = 0.16
    material: str = "black_MSLA_resin_with_matte_white_inlay"


@dataclass(frozen=True)
class GearboxParams:
    # Idealized 2PP target: these values are deliberately below commodity
    # desktop-MSLA limits.  The surrounding cell remains DIY-scale.
    module: float = 0.10
    pressure_angle_deg: float = 25.0
    backlash: float = 0.020
    input_teeth: int = 12
    idler_teeth: int = 18
    output_teeth: int = 24
    gear_thickness: float = 0.35
    total_gear_height: float = 1.30
    bore_diameter: float = 0.420
    bore_entry_chamfer: float = 0.025
    bore_entry_chamfer_angle_deg: float = 45.0
    shaft_diameter: float = 0.35
    shaft_length: float = 1.55
    shaft_deburr_chamfer: float = 0.005
    hub_diameter: float = 0.55
    housing_length: float = 6.00
    housing_width: float = 4.00
    housing_height: float = 1.60
    housing_wall: float = 0.030
    housing_floor: float = 0.25
    lid_thickness: float = 0.20
    gear_floor_clearance: float = 0.00
    gear_side_clearance: float = 0.018
    input_center_x: float = 0.75
    center_y: float = 2.00
    two_photon_min_feature: float = 0.008
    two_photon_running_clearance: float = 0.020
    tolerance_perturbation: float = 0.010
    shaft_seat_diameter: float = 0.340
    shaft_seat_depth: float = 0.250
    shaft_seat_split_width: float = 0.040
    shaft_seat_cap_thickness: float = 0.010
    input_driver_window_diameter: float = 0.75
    output_observation_window_diameter: float = 1.00
    input_drive_slot_width: float = 0.10
    output_phase_mark_long_length: float = 0.18
    output_phase_mark_short_length: float = 0.10
    output_phase_mark_width: float = 0.040
    output_phase_mark_depth: float = 0.030
    latch_count: int = 2
    datum_count: int = 3
    datum_pad_diameter: float = 0.30
    datum_pad_height: float = 0.03
    material_density_g_mm3: float = 0.00120
    shaft_density_g_mm3: float = 0.00785
    material: str = "2PP_low_shrink_photopolymer"
    shaft_material: str = "0.35_mm_hardened_steel_or_tungsten_dowel"

    @property
    def input_pitch_radius(self) -> float:
        return self.module * self.input_teeth / 2

    @property
    def idler_pitch_radius(self) -> float:
        return self.module * self.idler_teeth / 2

    @property
    def output_pitch_radius(self) -> float:
        return self.module * self.output_teeth / 2

    @property
    def idler_center_x(self) -> float:
        return self.input_center_x + self.input_pitch_radius + self.idler_pitch_radius

    @property
    def output_center_x(self) -> float:
        return (
            self.idler_center_x
            + self.idler_pitch_radius
            + self.output_pitch_radius
        )

    @property
    def ratio(self) -> float:
        return self.output_teeth / self.input_teeth

    @property
    def gear_z(self) -> float:
        return self.housing_floor + self.gear_floor_clearance

    @property
    def cover_under_clearance(self) -> float:
        return self.housing_height - (self.gear_z + self.total_gear_height)


@dataclass(frozen=True)
class ToolingParams:
    """Low-cost fixture and end-effector geometry for the benchmark cell."""

    nest_length: float = 12.0
    nest_width: float = 9.0
    nest_height: float = 2.5
    nest_pocket_clearance: float = 0.06
    nest_pocket_depth: float = 0.80
    nest_fastener_diameter: float = 2.20
    tray_length: float = 34.0
    tray_width: float = 24.0
    tray_height: float = 2.40
    tray_pocket_depth: float = 1.00
    tray_gear_clearance: float = 0.12
    vacuum_holder_diameter: float = 6.0
    vacuum_holder_length: float = 18.0
    vacuum_nozzle_outer_diameter: float = 0.90
    vacuum_nozzle_inner_diameter: float = 0.45
    vacuum_nozzle_length: float = 16.0
    vacuum_tip_outer_diameter: float = 0.30
    vacuum_tip_inner_diameter: float = 0.18
    probe_base_length: float = 8.0
    probe_base_width: float = 6.0
    probe_base_height: float = 2.0
    probe_flexure_length: float = 9.0
    probe_flexure_width: float = 0.60
    probe_flexure_thickness: float = 0.45
    probe_tip_diameter: float = 0.30
    probe_tip_length: float = 4.0
    rotary_shank_diameter: float = 3.175
    rotary_shank_length: float = 18.0
    rotary_blade_width: float = 0.080
    rotary_blade_thickness: float = 0.080
    rotary_blade_length: float = 0.45
    pointer_base_length: float = 12.0
    pointer_base_width: float = 10.0
    pointer_base_height: float = 2.0
    pointer_rod_diameter: float = 1.2
    pointer_rod_length: float = 20.0
    pointer_tip_diameter: float = 0.010
    pointer_tip_length: float = 4.0
    printed_material: str = "PETG_or_tough_MSLA_resin"
    tool_material: str = "stainless_steel_or_tungsten_with_printed_holder"
    printed_density_g_mm3: float = 0.00120


PROBE_FLEXURE_LEAF_COUNT = 4
PROBE_NOMINAL_MODULUS_N_MM2 = 2_000.0


def nominal_probe_stiffness_n_per_mm(tool: ToolingParams) -> float:
    """Return the four-leaf fixed-guided bending stiffness.

    Each modeled leaf has length ``L`` along X, width ``w`` along Y, and
    bending thickness ``t`` along Z.  A fixed-guided leaf therefore contributes
    ``12 E I / L^3 = E w t^3 / L^3`` in the probe's Z direction.
    """

    return (
        PROBE_FLEXURE_LEAF_COUNT
        * PROBE_NOMINAL_MODULUS_N_MM2
        * tool.probe_flexure_width
        * tool.probe_flexure_thickness**3
        / tool.probe_flexure_length**3
    )


@dataclass(frozen=True)
class DesignConfig:
    schema_version: str = "pipe-cad/0.1"
    tube: TubeParams = field(default_factory=TubeParams)
    rail: RailParams = field(default_factory=RailParams)
    carriage: CarriageParams = field(default_factory=CarriageParams)
    arm: ArmParams = field(default_factory=ArmParams)
    gripper: GripperParams = field(default_factory=GripperParams)
    sensing: SensingParams = field(default_factory=SensingParams)
    gearbox: GearboxParams = field(default_factory=GearboxParams)
    tooling: ToolingParams = field(default_factory=ToolingParams)
    manufacturing: ManufacturingParams = field(default_factory=ManufacturingParams)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def validation_errors(self) -> list[str]:
        """Return physical contradictions without importing a CAD kernel."""

        e: list[str] = []
        t, r, c, a, g, s, tool, m = (
            self.tube,
            self.rail,
            self.carriage,
            self.arm,
            self.gearbox,
            self.sensing,
            self.tooling,
            self.manufacturing,
        )
        if t.inner_radius <= 0 or t.wall < m.minimum_wall or t.length <= 0:
            e.append("tube dimensions are non-physical or below minimum wall")
        if abs(t.inner_radius - 80.0) > 1e-9:
            e.append("qualified target requires a 160 mm tube ID")
        if abs(t.usable_length - 320.0) > 1e-9:
            e.append("qualified target requires exactly 320 mm usable tube length")
        if t.rail_count != 4:
            e.append("qualified target requires four independently driven rail axes")
        if r.radial_depth + r.wall_standoff >= t.inner_radius:
            e.append("rail consumes the central workspace")
        if c.radial_depth <= r.radial_depth + 2 * r.carriage_clearance:
            e.append("carriage outer radial depth does not enclose the rail")
        if c.tangential_width <= r.tangential_width + 2 * r.carriage_clearance:
            e.append("carriage outer width does not enclose the rail")
        if len(a.link_lengths) != a.link_count:
            e.append("one serial length is required per arm link")
        if a.link_lengths != (32.0, 30.0, 15.0):
            e.append("qualified target requires 32+30+15 mm serial arm links")
        if len(a.default_joint_angles_deg) != a.actuator_count:
            e.append("one default angle is required per physical arm axis")
        if a.actuator_count != 4 or len(a.actuator_axes) != a.actuator_count:
            e.append("qualified target requires four differential capstan channels")
        if a.actuator_axes != (
            "shoulder_yaw",
            "shoulder_pitch",
            "elbow_pitch",
            "wrist_roll",
        ):
            e.append("arm actuator axes diverge from the locked tendon-loop baseline")
        if abs(a.shoulder_root_radius - 72.0) > 1e-9:
            e.append("qualified arm shoulder datum must be 72 mm from the tube axis")
        if abs(a.tendon_diameter - 0.20) > 1e-9 or abs(a.spool_radius - 3.0) > 1e-9:
            e.append("qualified tendon loop requires 0.20 mm line and 3.0 mm capstans")
        if abs(a.tendon_offset - 1.65) > 1e-9:
            e.append("reference joint tendon moment arm must be 1.65 mm")
        if a.usable_tendon_payout < 12.0:
            e.append("actuator payout cannot cover the reference joint spans")
        if any(length <= 2 * a.joint_radius for length in a.link_lengths):
            e.append("arm link center distance must exceed two joint radii")
        if a.tendon_channel_diameter < a.tendon_diameter + m.minimum_printed_gap:
            e.append("tendon channel has insufficient running clearance")
        if a.hinge_pin_diameter + a.hinge_bore_clearance >= 2 * a.joint_radius:
            e.append("hinge bore removes the whole arm knuckle")
        if s.case_wall < m.minimum_wall:
            e.append("sensor case wall is below manufacturing minimum")
        if s.global_camera_count != 6:
            e.append("qualified target requires six global cameras")
        if s.simultaneous_macro_view_count != 2:
            e.append("qualified target requires two simultaneous macro views")
        if s.structured_light_projector_count != 1:
            e.append("qualified target requires one shared structured-light projector")
        if (s.global_image_width_px, s.global_image_height_px) != (1280, 800):
            e.append("reference global image raster must be 1280 x 800")
        if abs(s.global_horizontal_fov_deg - 68.0) > 1e-9:
            e.append("reference global horizontal field of view must be 68 degrees")
        if (s.macro_image_width_px, s.macro_image_height_px) != (2048, 1536):
            e.append("reference macro image raster must be 2048 x 1536")
        if abs(s.macro_field_width - 4.0) > 1e-9 or abs(s.macro_field_height - 3.0) > 1e-9:
            e.append("reference macro field must be 4 x 3 mm")
        if abs(s.macro_pixel_scale - 0.002) > 1e-9:
            e.append("reference macro object sampling must be 0.002 mm/px")
        if abs(s.global_camera_front_radius - 60.0) > 1e-9:
            e.append("reference optical front-face radius must be 60 mm")
        if s.global_camera_end_offsets != (-106.0, 106.0):
            e.append("reference camera triplets must be at +/-106 mm")
        if s.global_camera_triplet_azimuths != (
            (0.0, 120.0, 240.0),
            (60.0, 180.0, 300.0),
        ):
            e.append("reference camera triplet azimuths changed")
        if abs(s.macro_stereo_baseline - 12.0) > 1e-9:
            e.append("reference macro stereo baseline must be 12 mm")
        if not 0 <= s.macro_mount_arm_index < t.rail_count:
            e.append("macro stereo head must mount to an existing arm")
        if s.global_camera_front_radius + max(
            s.camera_pod_depth, s.projector_depth
        ) >= t.inner_radius:
            e.append("sensor pod depth intersects the tube wall")
        if g.bore_diameter <= g.shaft_diameter:
            e.append("gear bore must run freely on the shaft")
        qualified_gear_values = (
            abs(g.module - 0.10) <= 1e-9
            and abs(g.pressure_angle_deg - 25.0) <= 1e-9
            and abs(g.backlash - 0.020) <= 1e-9
            and abs(g.bore_diameter - 0.420) <= 1e-9
            and abs(g.bore_entry_chamfer - 0.025) <= 1e-9
            and abs(g.bore_entry_chamfer_angle_deg - 45.0) <= 1e-9
            and abs(g.shaft_length - 1.55) <= 1e-9
            and 0.0 < g.shaft_deburr_chamfer <= 0.005 + 1e-12
            and abs(g.input_center_x - 0.75) <= 1e-9
            and abs(g.idler_center_x - 2.25) <= 1e-9
            and abs(g.output_center_x - 4.35) <= 1e-9
        )
        if not qualified_gear_values:
            e.append("gearbox parameters diverge from the qualified micro-assembly target")
        article_values = (
            abs(g.housing_height - 1.60) <= 1e-9
            and abs(g.housing_floor - 0.25) <= 1e-9
            and abs(g.lid_thickness - 0.20) <= 1e-9
            and abs(g.gear_thickness - 0.35) <= 1e-9
            and abs(g.total_gear_height - 1.30) <= 1e-9
            and abs(g.shaft_seat_diameter - 0.340) <= 1e-9
            and abs(g.shaft_seat_depth - 0.250) <= 1e-9
            and abs(g.input_driver_window_diameter - 0.75) <= 1e-9
            and abs(g.output_observation_window_diameter - 1.00) <= 1e-9
            and abs(g.input_drive_slot_width - 0.10) <= 1e-9
            and g.latch_count == 2
            and g.datum_count == 3
        )
        if not article_values:
            e.append("gearbox geometry diverges from the acceptance article")
        if g.output_phase_mark_long_length <= g.output_phase_mark_short_length:
            e.append("output optical phase marks must have unequal lengths")
        if not 0.04 <= g.cover_under_clearance <= 0.08:
            e.append("gear top must clear the cover underside by 0.04 to 0.08 mm")
        if g.shaft_seat_diameter >= g.shaft_diameter:
            e.append("split shaft seat must retain diametral interference")
        if abs(g.shaft_seat_depth - g.housing_floor) > 1e-9:
            e.append("blind shaft seat depth must match the qualified floor depth")
        if (
            g.housing_height + g.lid_thickness + g.datum_pad_height
            > 1.85 + 1e-9
        ):
            e.append("closed gearbox including external datums exceeds 1.85 mm")
        if g.backlash < g.two_photon_min_feature:
            e.append("gear backlash is below the 2PP minimum feature")
        if g.bore_diameter - g.shaft_diameter < g.two_photon_running_clearance:
            e.append("gear bore has insufficient 2PP running clearance")
        if min(g.input_teeth, g.idler_teeth, g.output_teeth) < 8:
            e.append("gear tooth count below supported involute approximation")
        if g.bore_entry_chamfer >= g.total_gear_height / 2:
            e.append("gear bore entry chamfers overlap")
        if g.shaft_deburr_chamfer >= min(g.shaft_length / 2, g.shaft_diameter / 2):
            e.append("shaft deburr chamfer consumes the shaft")

        if tool.rotary_blade_width >= g.input_drive_slot_width:
            e.append("rotary drive blade must clear the G1 drive slot")
        if tool.nest_pocket_depth <= 0 or tool.nest_pocket_depth >= tool.nest_height:
            e.append("gearbox nest pocket depth is invalid")
        if tool.tray_pocket_depth <= 0 or tool.tray_pocket_depth >= tool.tray_height:
            e.append("parts-tray pocket depth is invalid")
        if tool.vacuum_nozzle_inner_diameter >= tool.vacuum_nozzle_outer_diameter:
            e.append("vacuum nozzle bore consumes its wall")
        if not 0.15 <= tool.vacuum_tip_outer_diameter <= 0.30:
            e.append("vacuum pickup tip must be 0.15 to 0.30 mm OD")
        if tool.vacuum_tip_inner_diameter >= tool.vacuum_tip_outer_diameter:
            e.append("vacuum pickup tip bore consumes its wall")
        if abs(tool.rotary_blade_width - 0.080) > 1e-9:
            e.append("rotary drive blade must be the 0.080 mm reference")
        if tool.pointer_tip_diameter > 0.010:
            e.append("calibration pointer apex exceeds 10 um")
        nominal_probe_stiffness_n_mm = nominal_probe_stiffness_n_per_mm(tool)
        if not 0.5 <= nominal_probe_stiffness_n_mm <= 1.5:
            e.append("probe leaf-flexure stiffness misses the 0.10 N/0.10 mm class")

        max_output_x = g.output_center_x + g.module * (g.output_teeth + 2) / 2
        if max_output_x + g.gear_side_clearance > g.housing_length - g.housing_wall:
            e.append("output gear collides with the housing end wall")
        min_input_x = g.input_center_x - g.module * (g.input_teeth + 2) / 2
        if min_input_x - g.gear_side_clearance < g.housing_wall:
            e.append("input gear collides with the housing end wall")
        output_outer_radius = g.module * (g.output_teeth + 2) / 2
        cavity_half_width = (g.housing_width - 2 * g.housing_wall) / 2
        if output_outer_radius + g.gear_side_clearance > cavity_half_width:
            e.append("output gear collides with a housing side wall")
        gear_top = g.gear_z + g.total_gear_height
        if gear_top + g.gear_floor_clearance > g.housing_height:
            e.append("gear stack is taller than housing cavity")

        for teeth in (g.input_teeth, g.idler_teeth, g.output_teeth):
            pitch = g.module * teeth / 2
            base = pitch * cos(radians(g.pressure_angle_deg))
            root = pitch - 1.25 * g.module
            if root <= g.bore_diameter / 2 + 3 * g.two_photon_min_feature:
                e.append(f"{teeth}-tooth gear has insufficient root material")
            if base <= 0 or root <= 0:
                e.append(f"{teeth}-tooth gear radii are non-physical")
        return e

    def validate(self) -> None:
        errors = self.validation_errors()
        if errors:
            raise ValueError("; ".join(errors))


def default_design() -> DesignConfig:
    """Construct and validate the baseline low-cost prototype."""

    config = DesignConfig()
    config.validate()
    return config


def gear_circular_pitch(module: float) -> float:
    return pi * module
