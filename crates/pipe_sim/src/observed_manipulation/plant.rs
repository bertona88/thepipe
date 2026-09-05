//! Private truth-owning plant and optical-adapter boundary for M1e.
//!
//! `ObservedPlant` is the only M1e type in `pipe_sim` allowed to own a
//! [`Simulation`] or construct truth-space optical features.  Its normal API
//! exposes command acknowledgements, fixed-step timing, labelled feature
//! measurements, and reduced contact packets that have plausible hardware
//! counterparts.  Exact poses are available only through the deliberately
//! named `evaluation_*` methods at the bottom of the implementation.

use std::collections::BTreeSet;

use pipe_optics::{
    BrownConrady, CalibratedCamera, CalibrationDrift, CameraIntrinsics, Covariance3, FeaturePoint,
    FeaturePointObservation, FeaturePointSample, Geometry as OpticalGeometry, ImageSize,
    Mat3 as OpticalMat3, Material as OpticalMaterial, MissingFeaturePoint, PinholeCamera,
    Primitive as OpticalPrimitive, QualityMetrics, RigidTransform, ScanConfig,
    Scene as OpticalScene, Sphere as OpticalSphere, StructuredLightRig, Vec3 as OpticalVec3,
};
use pipe_sim_core::{
    query_pair, wrap_angle_pi, ArmId, BodyId, CollisionFilter, MachineCommand, MachineCommandError,
    ManipulatorId, ManipulatorMotionState, MotionType, Pose, Quat, RigidBody, Shape, Simulation,
    SimulationError, ToolMotionPlan, ToolMotionStatus, Vec3,
};
use serde::Serialize;

use super::controller::{ContactPacket, PlanningObstacle};
use super::estimator::{FeatureMeasurement, KnownAxialFeature};
use super::scenario::{M1eFault, ObservedManipulationScenario};
use crate::{machine_config, SimError};

pub(super) const ACTIVE_ARM_ID: ArmId = ArmId(1);
pub(super) const ACTIVE_MANIPULATOR_ID: ManipulatorId = ManipulatorId(1);
pub(super) const PEG_OBJECT_ID: u32 = 10_001;
pub(super) const SOCKET_OBJECT_ID: u32 = 10_002;
pub(super) const TOOL_OBJECT_ID: u32 = 10_010;
pub(super) const CALIBRATION_DATUM_OBJECT_ID: u32 = 10_020;

const SOCKET_BODY_IDS: [BodyId; 4] = [
    BodyId(10_002),
    BodyId(10_003),
    BodyId(10_004),
    BodyId(10_005),
];
const CARRIED_COLLISION_BODY_ID: BodyId = BodyId(10_030);
const PEG_COLLISION_GROUP: u32 = 0b0010;
const SOCKET_COLLISION_GROUP: u32 = 0b0100;
const FAULT_OBSTACLE_COLLISION_GROUP: u32 = 0b1000;
const FEATURE_IDS: [u32; 4] = [1, 2, 3, 4];
// A fitted ring center is emitted only after at least three of these seven
// surface probes are measured and the usable probes bracket the nominal
// viewing meridian by at least 40 degrees.  This is a deterministic geometric
// visibility proxy for a coded-ring/arc fit, not rendered-image extraction.
const RING_ARC_PROBE_ANGLES_RAD: [f64; 7] = [
    -core::f64::consts::PI / 3.0,
    -2.0 * core::f64::consts::PI / 9.0,
    -core::f64::consts::PI / 9.0,
    0.0,
    core::f64::consts::PI / 9.0,
    2.0 * core::f64::consts::PI / 9.0,
    core::f64::consts::PI / 3.0,
];
const MINIMUM_USABLE_RING_PROBES: usize = 3;
const MINIMUM_USABLE_RING_ARC_RAD: f64 = 2.0 * core::f64::consts::PI / 9.0;
const OPTICAL_TAG_PEG: u32 = 300;
const OPTICAL_TAG_SOCKET: u32 = 310;
const OPTICAL_TAG_SOCKET_FIDUCIAL_POSITIVE: u32 = 311;
const OPTICAL_TAG_SOCKET_FIDUCIAL_NEGATIVE: u32 = 312;
const OPTICAL_TAG_GRIPPER: u32 = 320;
const OPTICAL_TAG_TOOL_LINK: u32 = 330;
const OPTICAL_TAG_TOOL_FIDUCIAL_POSITIVE: u32 = 340;
const OPTICAL_TAG_TOOL_FIDUCIAL_NEGATIVE: u32 = 341;
const OPTICAL_TAG_ARM_BASE: u32 = 400;
const OPTICAL_TAG_FIXTURE_BASE: u32 = 500;
const OPTICAL_TAG_FAULT_OCCLUDER: u32 = 900;
const PEG_FEATURE_STATIONS_M: [f64; 4] = [-0.350e-3, -0.325e-3, 0.025e-3, 0.050e-3];
const CALIBRATION_FAULT_BIAS_M: f64 = 60.0e-6;
const OUTLIER_FAULT_OFFSET_M: f64 = 120.0e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum MotionClass {
    Transit,
    Correction,
    Insertion,
    Retreat,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum MotionStatus {
    Idle,
    Active,
    Stopped,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub(super) struct MotionCapabilities {
    pub maximum_correction_m: f64,
    pub minimum_reproducible_correction_m: f64,
    pub differential_backlash_m: f64,
    pub maximum_correction_velocity_m_s: f64,
    pub maximum_correction_acceleration_m_s2: f64,
    pub insertion_increment_m: f64,
    pub settling_ticks: u64,
}

/// Calibrated, truth-free coupon geometry used to guard the jaw/socket sweep.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub(super) struct JawSocketClearanceGeometry {
    pub jaw_axial_half_length_m: f64,
    pub closed_jaw_transverse_radius_m: f64,
    pub open_jaw_transverse_radius_m: f64,
    pub central_palm_forward_plane_tool_z_m: f64,
    pub central_palm_radial_extent_m: f64,
    pub side_target_center_axial_offset_m: f64,
    pub side_target_center_transverse_distance_m: f64,
    pub side_target_cross_section_radius_m: f64,
    pub side_target_axial_half_extent_m: f64,
    pub side_target_forward_extent_m: f64,
    pub side_target_transverse_radius_m: f64,
    pub socket_depth_m: f64,
    pub socket_inner_half_width_m: f64,
    pub socket_outer_half_width_m: f64,
    pub peg_tip_from_center_m: f64,
    pub tool_to_peg_axial_offset_m: f64,
    pub minimum_clearance_m: f64,
    pub nominal_seated_axial_clearance_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(super) struct CommandReceipt {
    pub command_sequence: u64,
    pub issued_at_tick: u64,
    pub command_kind: &'static str,
    pub motion_class: Option<MotionClass>,
    pub target_position_world_m: Option<[f64; 3]>,
    pub target_gripper_opening_m: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum ForceInterlockChannel {
    Gripper,
    Insertion,
}

impl ForceInterlockChannel {
    pub const fn id(self) -> &'static str {
        match self {
            Self::Gripper => "gripper",
            Self::Insertion => "insertion",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub(super) struct ForceInterlockEvent {
    pub tick: u64,
    pub channel: ForceInterlockChannel,
    pub measured_force_proxy_n: f64,
    pub limit_force_proxy_n: f64,
    pub motion_was_active: bool,
    pub stop_command_sequence: Option<u64>,
    pub packet: ContactPacket,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(super) struct MissingFeatureMeasurement {
    pub object_id: u32,
    pub feature_id: u32,
    pub capture_tick: u64,
    pub reason: String,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub(super) struct ObservationBurst {
    pub sequence: u32,
    pub capture_start_tick: u64,
    pub capture_end_tick: u64,
    pub available_tick: u64,
    pub requested_object_ids: Vec<u32>,
    pub measurements: Vec<FeatureMeasurement>,
    pub missing: Vec<MissingFeatureMeasurement>,
    pub triangulation_head_count: u32,
    pub calibrated_rays_per_observed_point: u32,
    pub calibration_reference_residual_m: Option<f64>,
    pub calibration_reference_sample_count: u32,
    pub calibration_reference_valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub(super) struct PlantEvaluationMetrics {
    pub final_peg_tip_to_socket_seat_error_m: f64,
    pub final_peg_lateral_error_m: f64,
    pub final_peg_axial_error_m: f64,
    pub final_peg_axis_error_rad: f64,
    pub final_tool_center_to_socket_center_distance_m: f64,
    pub physical_grasp_attachment_present: bool,
    pub grasp_committed: bool,
    pub release_committed: bool,
    pub maximum_unplanned_penetration_m: f64,
    pub peak_grip_force_proxy_n: f64,
    pub peak_insertion_force_proxy_n: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum PlantFailure {
    InvalidCommand,
    CorrectionMagnitudeLimit,
    InsertionIncrementLimit,
    CorrectionFloorTooLarge,
    MotionCollisionRisk,
    CarriedPartCollisionRisk,
    ImpossibleGeometry,
    MotionDurationLimit,
    MotionNotStopped,
    ObservationBeforeSettled,
    GraspOutsideCaptureRegion,
    GraspForceOutOfBounds,
    PegNotHeld,
    ReleaseConditionNotMet,
    ContactForceLimit,
    MechanicsFailure,
}

impl PlantFailure {
    pub const fn reason(self) -> &'static str {
        match self {
            Self::InvalidCommand => "invalid_plant_command",
            Self::CorrectionMagnitudeLimit => "correction_magnitude_limit",
            Self::InsertionIncrementLimit => "insertion_increment_limit",
            Self::CorrectionFloorTooLarge => "correction_floor_too_large",
            Self::MotionCollisionRisk => "predicted_swept_geometry_collision_risk",
            Self::CarriedPartCollisionRisk => "carried_part_collision_risk",
            Self::ImpossibleGeometry => "impossible_geometry",
            Self::MotionDurationLimit => "motion_duration_limit",
            Self::MotionNotStopped => "observation_while_motion_active",
            Self::ObservationBeforeSettled => "observation_before_settled",
            Self::GraspOutsideCaptureRegion => "grasp_outside_capture_region",
            Self::GraspForceOutOfBounds => "grasp_force_out_of_bounds",
            Self::PegNotHeld => "peg_not_held",
            Self::ReleaseConditionNotMet => "release_condition_not_met",
            Self::ContactForceLimit => "contact_force_limit",
            Self::MechanicsFailure => "plant_mechanics_failure",
        }
    }
}

impl core::fmt::Display for PlantFailure {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(self.reason())
    }
}

impl std::error::Error for PlantFailure {}

/// M1e's private plant.  Keep this type private to the observed-manipulation
/// module: widening it would make accidental controller access to truth much
/// easier during later milestones.
pub(super) struct ObservedPlant {
    scenario: ObservedManipulationScenario,
    fault: M1eFault,
    machine_config_id: String,
    machine_config_source_sha256: String,
    calibrated_tool_geometry: machine_config::CalibratedToolGeometry,
    mechanics: Simulation,
    socket_pose: Pose,
    calibrated_socket_axis_world: [f64; 3],
    calibrated_macro_view_direction_world: [f64; 3],
    physical_tool_axis_tilt: Quat,
    commanded_tool_position_world_m: [f64; 3],
    previous_motion_direction: [i8; 3],
    last_motion_complete_tick: u64,
    burst_sequence: u32,
    maximum_unplanned_penetration_m: f64,
    peak_grip_force_proxy_n: f64,
    peak_insertion_force_proxy_n: f64,
    grasp_capture_fault_applied: bool,
    insertion_jam_fault_applied: bool,
    active_motion_class: Option<MotionClass>,
    active_gripper_closing: bool,
    last_force_interlock_event: Option<ForceInterlockEvent>,
    grasp_committed: bool,
    release_committed: bool,
}

impl ObservedPlant {
    pub fn new(scenario: &ObservedManipulationScenario, fault: M1eFault) -> Result<Self, SimError> {
        scenario
            .validate_fault(fault)
            .map_err(|error| SimError::InvalidScenario(error.to_string()))?;
        let loaded = machine_config::load_m1e_coupon_machine_config()?;
        if scenario.machine_config_id != loaded.id {
            return Err(SimError::InvalidScenario(format!(
                "M1e scenario machine_config_id '{}' does not match loaded '{}'",
                scenario.machine_config_id, loaded.id
            )));
        }
        let calibrated_tool_geometry = loaded.tool_geometry.ok_or_else(|| {
            SimError::InvalidScenario(
                "M1e machine config requires calibrated tool_geometry".to_owned(),
            )
        })?;
        let mut mechanics = machine_config::build_baseline_machine(&loaded)?;
        let calibrated_gripper = mechanics
            .serial_arm(ACTIVE_ARM_ID)
            .ok_or_else(|| SimError::Mechanics("M1e active arm 1 missing".to_owned()))?
            .gripper_config;
        let peg_tip_from_center_m =
            scenario.coupon.peg_half_segment_m + 0.5 * scenario.coupon.peg_diameter_m;
        let required_tool_envelope_m = open_gripper_corner_radius_m(calibrated_gripper)
            .max(scenario.grasp.tool_to_peg_axial_offset_m + peg_tip_from_center_m)
            .max(side_target_bounding_radius_m(calibrated_tool_geometry));
        if scenario.safety.tool_envelope_radius_m + 1.0e-15 < required_tool_envelope_m {
            return Err(SimError::Mechanics(format!(
                "M1e tool_envelope_radius_m {:.9e} is smaller than calibrated open-jaw radius {:.9e}",
                scenario.safety.tool_envelope_radius_m, required_tool_envelope_m
            )));
        }
        let required_carried_peg_envelope_m = peg_tip_from_center_m;
        if scenario.safety.carried_peg_envelope_radius_m + 1.0e-15 < required_carried_peg_envelope_m
        {
            return Err(SimError::Mechanics(format!(
                "M1e carried_peg_envelope_radius_m {:.9e} is smaller than capsule bound {:.9e}",
                scenario.safety.carried_peg_envelope_radius_m, required_carried_peg_envelope_m
            )));
        }
        let tool_forward_extent_m = calibrated_gripper
            .jaw_half_extents_m
            .z
            .max(side_target_forward_extent_m(calibrated_tool_geometry));
        let seated_axial_clearance_m = nominal_seated_tool_socket_clearance_m(
            scenario.coupon.socket_depth_m,
            peg_tip_from_center_m,
            scenario.grasp.tool_to_peg_axial_offset_m,
            tool_forward_extent_m,
        )
        .ok_or_else(|| SimError::Mechanics("M1e tool/socket geometry is invalid".to_owned()))?;
        if seated_axial_clearance_m + 1.0e-15 < scenario.safety.minimum_obstacle_clearance_m {
            return Err(SimError::Mechanics(format!(
                "M1e nominal seated tool/socket clearance {:.9e} m is below required {:.9e} m",
                seated_axial_clearance_m, scenario.safety.minimum_obstacle_clearance_m
            )));
        }
        let calibrated_socket_pose = solved_tool_pose(
            &mechanics,
            array_vec3(scenario.coupon.socket_center_nominal_world_m),
        )?;
        let calibrated_socket_axis = calibrated_socket_pose
            .transform_vector(Vec3::Z)
            .normalized_or(Vec3::Z);
        // M1e keeps M1's authoritative 1 ms fixed scheduler and zero-gravity
        // coupon baseline.  Gravity/part slip remain an explicit fidelity gap.
        let pick_nominal = array_vec3(scenario.coupon.pick_peg_center_nominal_world_m);
        let initial_commanded_tool = pick_nominal
            - calibrated_socket_axis * scenario.motion.pick_capture_start_axial_standoff_m;
        let initial_tool_truth =
            initial_commanded_tool + array_vec3(scenario.coupon.initial_tool_command_error_m);
        set_initial_tool_pose(&mut mechanics, initial_tool_truth)?;

        let peg_position = pick_nominal + array_vec3(scenario.coupon.initial_peg_error_m);
        let peg_pose = solved_tool_pose(&mechanics, peg_position)?;
        let peg_tilt = two_axis_tilt(scenario.coupon.initial_peg_axis_tilt_rad);
        let mut peg = RigidBody::new(
            BodyId(PEG_OBJECT_ID),
            Shape::Capsule {
                radius_m: 0.5 * scenario.coupon.peg_diameter_m,
                half_segment_m: scenario.coupon.peg_half_segment_m,
            },
            Pose::new(peg_position, (peg_pose.rotation * peg_tilt).normalized()),
            MotionType::Dynamic,
        );
        peg.collision_filter = CollisionFilter {
            group: PEG_COLLISION_GROUP,
            // M1e task-contact and fault-obstacle decisions are made from the
            // controller's covariance-inflated planning scene and sanitized
            // contact packets.  Disable core truth-scene task preflight so its
            // latent poses cannot become a controller side channel.
            mask: 0,
        };
        mechanics
            .add_body(peg)
            .map_err(|error| SimError::Mechanics(format!("M1e peg: {error:?}")))?;

        let socket_position = array_vec3(scenario.coupon.socket_center_nominal_world_m)
            + array_vec3(scenario.coupon.initial_socket_error_m);
        let mut socket_pose = solved_tool_pose(&mechanics, socket_position)?;
        socket_pose.rotation = (socket_pose.rotation
            * two_axis_tilt(scenario.coupon.initial_socket_axis_tilt_rad))
        .normalized();
        add_socket_coupon(&mut mechanics, scenario, socket_pose)?;
        add_calibrated_planning_obstacles(&mut mechanics, scenario)?;

        if fault == M1eFault::CarriedPartCollision {
            add_carried_collision_obstacle(&mut mechanics, scenario)?;
        }

        Ok(Self {
            scenario: scenario.clone(),
            fault,
            machine_config_id: loaded.id,
            machine_config_source_sha256: loaded.source_sha256,
            calibrated_tool_geometry,
            mechanics,
            socket_pose,
            calibrated_socket_axis_world: vec3(calibrated_socket_axis),
            // Fixed surveyed head azimuth: look through the finger-depth axis,
            // not along the parallel jaws' closure axis. The actual tool pose
            // is not consulted when generating this calibrated datum.
            calibrated_macro_view_direction_world: vec3(
                calibrated_socket_pose
                    .transform_vector(Vec3::Y)
                    .normalized_or(Vec3::Y),
            ),
            physical_tool_axis_tilt: two_axis_tilt(scenario.coupon.initial_tool_axis_tilt_rad),
            commanded_tool_position_world_m: vec3(initial_commanded_tool),
            previous_motion_direction: [0; 3],
            last_motion_complete_tick: 0,
            burst_sequence: 0,
            maximum_unplanned_penetration_m: 0.0,
            peak_grip_force_proxy_n: 0.0,
            peak_insertion_force_proxy_n: 0.0,
            grasp_capture_fault_applied: false,
            insertion_jam_fault_applied: false,
            active_motion_class: None,
            active_gripper_closing: false,
            last_force_interlock_event: None,
            grasp_committed: false,
            release_committed: false,
        })
    }

    pub fn now_tick(&self) -> u64 {
        self.mechanics.step_index
    }

    pub fn fixed_dt_s(&self) -> f64 {
        self.mechanics.config.fixed_dt_s
    }

    pub fn commanded_tool_position_world_m(&self) -> [f64; 3] {
        self.commanded_tool_position_world_m
    }

    pub fn machine_config_id(&self) -> &str {
        &self.machine_config_id
    }

    pub fn machine_config_source_sha256(&self) -> &str {
        &self.machine_config_source_sha256
    }

    /// Last sanitized protective-interlock event. This is a modeled hardware
    /// packet, not geometric truth or a contact-state classification.
    pub fn force_interlock_event(&self) -> Option<ForceInterlockEvent> {
        self.last_force_interlock_event
    }

    /// Peak values of the two raw hardware-plausible load channels sampled at
    /// the fixed plant rate. These are observations, not contact-state labels.
    pub fn force_channel_peaks(&self) -> (f64, f64) {
        (
            self.peak_grip_force_proxy_n,
            self.peak_insertion_force_proxy_n,
        )
    }

    /// Nominal socket axis derived only from the versioned coupon datum and
    /// calibrated baseline kinematics.  The actual socket pose is deliberately
    /// not exposed; local optical estimates replace this datum near contact.
    pub fn calibrated_socket_axis_world(&self) -> [f64; 3] {
        self.calibrated_socket_axis_world
    }

    pub fn maximum_gripper_opening_m(&self) -> f64 {
        self.active_arm().gripper_config.max_opening_m
    }

    pub fn jaw_socket_clearance_geometry(&self) -> JawSocketClearanceGeometry {
        let gripper = self.active_arm().gripper_config;
        let closed_opening_m =
            self.scenario.coupon.peg_diameter_m - self.scenario.grasp.commanded_pad_compression_m;
        let inner_half_width_m = 0.5 * self.scenario.coupon.peg_diameter_m
            + self.scenario.coupon.socket_radial_clearance_m;
        let peg_tip_from_center_m =
            self.scenario.coupon.peg_half_segment_m + 0.5 * self.scenario.coupon.peg_diameter_m;
        let side_target_forward_extent_m =
            side_target_forward_extent_m(self.calibrated_tool_geometry);
        let side_target_transverse_radius_m =
            side_target_transverse_radius_m(self.calibrated_tool_geometry);
        let [target_x_m, target_y_m, target_z_m] = self
            .calibrated_tool_geometry
            .side_target_center_offset_tool_m;
        let tool_forward_extent_m = gripper
            .jaw_half_extents_m
            .z
            .max(side_target_forward_extent_m);
        JawSocketClearanceGeometry {
            jaw_axial_half_length_m: gripper.jaw_half_extents_m.z,
            closed_jaw_transverse_radius_m: closed_jaw_transverse_radius_m(
                closed_opening_m,
                gripper.jaw_half_extents_m,
            ),
            open_jaw_transverse_radius_m: closed_jaw_transverse_radius_m(
                gripper.max_opening_m,
                gripper.jaw_half_extents_m,
            ),
            central_palm_forward_plane_tool_z_m: self
                .calibrated_tool_geometry
                .palm_forward_plane_tool_z_m,
            central_palm_radial_extent_m: gripper.jaw_half_extents_m.y,
            side_target_center_axial_offset_m: target_z_m,
            side_target_center_transverse_distance_m: target_x_m.hypot(target_y_m),
            side_target_cross_section_radius_m: core::f64::consts::SQRT_2
                * self.calibrated_tool_geometry.side_target_radius_m,
            side_target_axial_half_extent_m: self
                .calibrated_tool_geometry
                .side_target_axial_half_extent_m,
            side_target_forward_extent_m,
            side_target_transverse_radius_m,
            socket_depth_m: self.scenario.coupon.socket_depth_m,
            socket_inner_half_width_m: inner_half_width_m,
            socket_outer_half_width_m: inner_half_width_m
                + self.scenario.coupon.socket_wall_thickness_m,
            peg_tip_from_center_m,
            tool_to_peg_axial_offset_m: self.scenario.grasp.tool_to_peg_axial_offset_m,
            minimum_clearance_m: self.scenario.safety.minimum_obstacle_clearance_m,
            nominal_seated_axial_clearance_m: nominal_seated_tool_socket_clearance_m(
                self.scenario.coupon.socket_depth_m,
                peg_tip_from_center_m,
                self.scenario.grasp.tool_to_peg_axial_offset_m,
                tool_forward_extent_m,
            )
            .expect("validated M1e tool/socket geometry remains finite"),
        }
    }

    /// Static, calibrated planning keep-outs available to the controller. The
    /// collision fault also inserts its obstacle into the private plant, but no
    /// exact live pose or collision-query result crosses this boundary.
    pub fn calibrated_planning_obstacles(&self) -> Vec<PlanningObstacle> {
        // These are versioned surveyed-datum proxies, not live truth queries.
        // Stable ID ordering keeps native/WASM controller reports identical.
        // The socket mouth stays out of this phase-independent set because its
        // intended contact is governed by the insertion/contact state machine.
        let mut obstacles = self
            .scenario
            .safety
            .planning_obstacles
            .iter()
            .map(|obstacle| PlanningObstacle {
                id: obstacle.id,
                center_world_m: obstacle.center_world_m,
                conservative_radius_m: obstacle.conservative_radius_m,
                position_sigma_m: obstacle.position_sigma_m,
            })
            .collect::<Vec<_>>();
        if self.fault == M1eFault::CarriedPartCollision {
            let pick = array_vec3(self.scenario.coupon.pick_peg_center_nominal_world_m);
            let socket = array_vec3(self.scenario.coupon.socket_center_nominal_world_m);
            obstacles.push(PlanningObstacle {
                id: CARRIED_COLLISION_BODY_ID.0,
                center_world_m: vec3(pick.lerp(socket, 0.5)),
                conservative_radius_m: self.scenario.safety.carried_peg_envelope_radius_m + 0.10e-3,
                position_sigma_m: self.scenario.optics.correlated_calibration_sigma_m,
            });
        }
        obstacles.sort_by_key(|obstacle| obstacle.id);
        obstacles
    }

    pub fn motion_capabilities(&self) -> MotionCapabilities {
        let configured_floor = self.scenario.motion.minimum_reproducible_correction_m;
        let minimum_reproducible_correction_m = if self.fault == M1eFault::CorrectionFloorTooLarge {
            (2.5 * self.scenario.motion.correction_convergence_m).max(configured_floor)
        } else {
            configured_floor
        };
        MotionCapabilities {
            maximum_correction_m: self.scenario.motion.maximum_correction_m,
            minimum_reproducible_correction_m,
            differential_backlash_m: self.scenario.motion.differential_backlash_m,
            maximum_correction_velocity_m_s: self.scenario.motion.maximum_correction_velocity_m_s,
            maximum_correction_acceleration_m_s2: self
                .scenario
                .motion
                .maximum_correction_acceleration_m_s2,
            insertion_increment_m: self.scenario.motion.insertion_increment_m,
            settling_ticks: ticks_ceil(
                self.scenario.optics.settling_interval_s,
                self.mechanics.config.fixed_dt_s,
            ),
        }
    }

    pub fn feature_model(&self, object_id: u32) -> Vec<KnownAxialFeature> {
        let half_span = if object_id == TOOL_OBJECT_ID {
            self.calibrated_tool_geometry
                .side_target_feature_half_span_m
        } else {
            0.5 * self.scenario.coupon.minimum_feature_axial_span_m
        };
        axial_feature_model(object_id, half_span)
    }

    pub fn command_tool_position(
        &mut self,
        target_position_world_m: [f64; 3],
        motion_class: MotionClass,
    ) -> Result<CommandReceipt, PlantFailure> {
        let requested = array_vec3(target_position_world_m);
        self.validate_tool_motion_request(requested, motion_class)?;
        let commanded_delta = requested - array_vec3(self.commanded_tool_position_world_m);

        let actual_current = self.active_arm().tool_pose().translation;
        let effective_target =
            self.effective_motion_target(actual_current, requested, motion_class);
        self.inject_insertion_jam_fault_if_needed(motion_class, effective_target)?;
        let nominal_speed_scale = self.active_arm().tool_motion_speed_scale;
        let bounded_speed_scale = matches!(
            motion_class,
            MotionClass::Correction | MotionClass::Insertion | MotionClass::Retreat
        )
        .then(|| self.near_contact_tool_motion_speed_scale())
        .transpose()?;
        if let Some(speed_scale) = bounded_speed_scale {
            self.mechanics
                .serial_arm_mut(ACTIVE_ARM_ID)
                .expect("M1e active serial arm remains present")
                .tool_motion_speed_scale = speed_scale;
        }
        let issued_at_tick = self.now_tick();
        let submission = self
            .mechanics
            .submit_machine_command(MachineCommand::SetToolPoseTarget {
                manipulator: ACTIVE_MANIPULATOR_ID,
                target_position_world_m: effective_target,
            });
        // The plan captures its duration at submission, so restoring the
        // commissioning scale cannot alter the active bounded plan and avoids
        // leaking a correction scale into a later transit or retreat.
        if bounded_speed_scale.is_some() {
            self.mechanics
                .serial_arm_mut(ACTIVE_ARM_ID)
                .expect("M1e active serial arm remains present")
                .tool_motion_speed_scale = nominal_speed_scale;
        }
        let sequence = submission.map_err(|error| self.sanitize_command_error(error))?;
        self.commanded_tool_position_world_m = target_position_world_m;
        self.update_motion_directions(commanded_delta);
        self.active_motion_class = Some(motion_class);
        self.active_gripper_closing = false;
        Ok(CommandReceipt {
            command_sequence: sequence,
            issued_at_tick,
            command_kind: "set_tool_position",
            motion_class: Some(motion_class),
            target_position_world_m: Some(target_position_world_m),
            target_gripper_opening_m: None,
        })
    }

    /// Exact deterministic duration of the plan that `command_tool_position`
    /// would submit from the current authoritative commanded/mechanical state.
    /// Duration is a sanitized hardware-plausible planning result; no target
    /// pose, joint solution, or private collision geometry is returned.
    pub fn preview_tool_motion_duration_s(
        &self,
        target_position_world_m: [f64; 3],
        motion_class: MotionClass,
    ) -> Result<f64, PlantFailure> {
        Ok(self
            .preview_tool_motion_plan(target_position_world_m, motion_class)?
            .duration_s)
    }

    /// Conservative rigid-tool orientation excursion for the complete
    /// joint-interpolated point motion.  Each revolute coordinate follows one
    /// monotone smoothstep, so the sum of absolute coordinate travel is a
    /// triangle-inequality bound on rotation of any tool-fixed vector.  This
    /// is a sanitized command/model preview, not a latent distal-pose query.
    pub fn preview_tool_axis_change_bound_rad(
        &self,
        target_position_world_m: [f64; 3],
        motion_class: MotionClass,
    ) -> Result<f64, PlantFailure> {
        let plan = self.preview_tool_motion_plan(target_position_world_m, motion_class)?;
        let start_joints = plan.start.tendon_joint_angles();
        let goal_joints = plan.goal.tendon_joint_angles();
        let tendon_rotation_bound_rad = start_joints
            .into_iter()
            .zip(goal_joints)
            .map(|(start, goal)| (goal - start).abs())
            .sum::<f64>();
        let base_rotation_bound_rad =
            wrap_angle_pi(plan.goal.base_theta_rad - plan.start.base_theta_rad).abs();
        Ok(base_rotation_bound_rad + tendon_rotation_bound_rad)
    }

    /// Certified one-sided bound on TCP departure from the endpoint chord for
    /// the commanded joint-interpolated path. The sampled maximum is enlarged
    /// by a kinematic path-length bound for the interval between samples, so
    /// the result is not merely a visualization approximation.
    pub fn preview_tool_path_deviation_bound_m(
        &self,
        target_position_world_m: [f64; 3],
        motion_class: MotionClass,
    ) -> Result<f64, PlantFailure> {
        const SAMPLE_INTERVALS: usize = 128;
        const SMOOTHSTEP_MAX_SLOPE: f64 = 1.5;

        let plan = self.preview_tool_motion_plan(target_position_world_m, motion_class)?;
        let arm = self.active_arm();
        let mut candidate = arm.arm.clone();
        candidate
            .set_positions(plan.start)
            .map_err(|_| PlantFailure::ImpossibleGeometry)?;
        let chord_start = candidate.forward_kinematics().tool_pose.translation;
        candidate
            .set_positions(plan.goal)
            .map_err(|_| PlantFailure::ImpossibleGeometry)?;
        let chord_end = candidate.forward_kinematics().tool_pose.translation;

        let mut sampled_max_m: f64 = 0.0;
        for sample in 0..=SAMPLE_INTERVALS {
            let progress = sample as f64 / SAMPLE_INTERVALS as f64;
            candidate
                .set_positions(plan.sample(progress))
                .map_err(|_| PlantFailure::ImpossibleGeometry)?;
            let point = candidate.forward_kinematics().tool_pose.translation;
            sampled_max_m =
                sampled_max_m.max(point_segment_distance_m(point, chord_start, chord_end));
        }

        let start_joints = plan.start.tendon_joint_angles();
        let goal_joints = plan.goal.tendon_joint_angles();
        let joint_delta: [f64; 4] =
            core::array::from_fn(|index| (goal_joints[index] - start_joints[index]).abs());
        let theta_delta_rad =
            wrap_angle_pi(plan.goal.base_theta_rad - plan.start.base_theta_rad).abs();
        let z_delta_m = (plan.goal.base_z_m - plan.start.base_z_m).abs();
        let upper_m = arm.arm.config.upper_arm_length_m;
        let distal_m = arm.arm.config.forearm_length_m + arm.arm.config.wrist_length_m;
        let total_path_length_bound_m = z_delta_m
            + arm.carriage_config.rail_radius_m * theta_delta_rad
            + upper_m * (theta_delta_rad + joint_delta[0] + joint_delta[1])
            + distal_m * (theta_delta_rad + joint_delta[0] + joint_delta[1] + joint_delta[2]);
        let intersample_bound_m =
            SMOOTHSTEP_MAX_SLOPE * total_path_length_bound_m / SAMPLE_INTERVALS as f64;
        let bound_m = sampled_max_m + intersample_bound_m;
        if bound_m.is_finite() {
            Ok(bound_m)
        } else {
            Err(PlantFailure::ImpossibleGeometry)
        }
    }

    fn preview_tool_motion_plan(
        &self,
        target_position_world_m: [f64; 3],
        motion_class: MotionClass,
    ) -> Result<ToolMotionPlan, PlantFailure> {
        let requested = array_vec3(target_position_world_m);
        self.validate_tool_motion_request(requested, motion_class)?;
        let actual_current = self.active_arm().tool_pose().translation;
        let effective_target =
            self.effective_motion_target(actual_current, requested, motion_class);
        let arm = self.active_arm();
        let speed_scale = if matches!(
            motion_class,
            MotionClass::Correction | MotionClass::Insertion | MotionClass::Retreat
        ) {
            self.near_contact_tool_motion_speed_scale()?
        } else {
            arm.tool_motion_speed_scale
        };
        let start = arm.motion.positions();
        let solution = arm
            .arm
            .solve_tool_position(effective_target, start)
            .map_err(|_| PlantFailure::ImpossibleGeometry)?;
        let plan = ToolMotionPlan::new(
            effective_target,
            start,
            solution.positions,
            arm.carriage_config,
            arm.motion_config,
            speed_scale,
        );
        let maximum_duration_s =
            f64::from(self.scenario.motion.maximum_steps_per_motion) * self.fixed_dt_s();
        if !plan.duration_s.is_finite() || plan.duration_s > maximum_duration_s + 1.0e-12 {
            return Err(PlantFailure::MotionDurationLimit);
        }
        Ok(plan)
    }

    pub fn command_gripper(
        &mut self,
        target_opening_m: f64,
    ) -> Result<CommandReceipt, PlantFailure> {
        if !target_opening_m.is_finite() {
            return Err(PlantFailure::InvalidCommand);
        }
        self.inject_grasp_capture_fault_if_needed(target_opening_m)?;
        let closing = target_opening_m + 1.0e-15 < self.active_arm().gripper.opening_m;
        let issued_at_tick = self.now_tick();
        let sequence = self
            .mechanics
            .submit_machine_command(MachineCommand::SetGripperOpening {
                manipulator: ACTIVE_MANIPULATOR_ID,
                target_opening_m,
            })
            .map_err(|error| self.sanitize_command_error(error))?;
        self.active_motion_class = None;
        self.active_gripper_closing = closing;
        Ok(CommandReceipt {
            command_sequence: sequence,
            issued_at_tick,
            command_kind: "set_gripper_opening",
            motion_class: None,
            target_position_world_m: None,
            target_gripper_opening_m: Some(target_opening_m),
        })
    }

    pub fn command_stop(&mut self) -> Result<CommandReceipt, PlantFailure> {
        let issued_at_tick = self.now_tick();
        let sequence = self
            .mechanics
            .submit_machine_command(MachineCommand::Stop {
                manipulator: Some(ACTIVE_MANIPULATOR_ID),
            })
            .map_err(|error| self.sanitize_command_error(error))?;
        self.last_motion_complete_tick = issued_at_tick;
        self.active_motion_class = None;
        self.active_gripper_closing = false;
        Ok(CommandReceipt {
            command_sequence: sequence,
            issued_at_tick,
            command_kind: "stop",
            motion_class: None,
            target_position_world_m: None,
            target_gripper_opening_m: None,
        })
    }

    pub fn motion_status(&self) -> MotionStatus {
        let arm = self.active_arm();
        let gripper_active = (arm.gripper.opening_m - arm.gripper.command_opening_m).abs()
            > 1.0e-12
            || arm.gripper.opening_velocity_m_s.abs() > 1.0e-12;
        if gripper_active
            || arm
                .motion
                .tool_motion
                .is_some_and(|plan| plan.status == ToolMotionStatus::Active)
        {
            MotionStatus::Active
        } else if arm.motion.stopped
            || arm
                .motion
                .tool_motion
                .is_some_and(|plan| plan.status == ToolMotionStatus::Stopped)
        {
            MotionStatus::Stopped
        } else {
            MotionStatus::Idle
        }
    }

    pub fn advance_one(&mut self) -> Result<(), PlantFailure> {
        let was_active = self.motion_status() == MotionStatus::Active;
        let report = self
            .mechanics
            .step()
            .map_err(|_| PlantFailure::MechanicsFailure)?;
        let packet = self.sample_contact_packet();
        self.peak_grip_force_proxy_n = self.peak_grip_force_proxy_n.max(packet.grip_force_proxy_n);
        self.peak_insertion_force_proxy_n = self
            .peak_insertion_force_proxy_n
            .max(packet.insertion_force_proxy_n);
        // Accumulate every private collision diagnostic before selecting the
        // deterministic terminal failure. A same-tick force stop must not hide
        // penetration in the evaluation-only safety record.
        let tool_socket_penetration_m = self.private_tool_socket_max_penetration_m();
        self.maximum_unplanned_penetration_m = self
            .maximum_unplanned_penetration_m
            .max(tool_socket_penetration_m);
        let mut collision_failure =
            (tool_socket_penetration_m > 1.0e-12).then_some(PlantFailure::MotionCollisionRisk);
        for contact in report.contacts {
            if is_intended_socket_pair(contact.body_a, contact.body_b) {
                continue;
            }
            self.maximum_unplanned_penetration_m = self
                .maximum_unplanned_penetration_m
                .max(contact.penetration_depth_m);
            let candidate = if contact.body_a == CARRIED_COLLISION_BODY_ID
                || contact.body_b == CARRIED_COLLISION_BODY_ID
            {
                PlantFailure::CarriedPartCollisionRisk
            } else {
                PlantFailure::MotionCollisionRisk
            };
            if collision_failure.is_none() || candidate == PlantFailure::CarriedPartCollisionRisk {
                collision_failure = Some(candidate);
            }
        }
        // The insertion load-path channel is meaningful whenever a held peg
        // reaches the socket neighborhood, regardless of the executive's
        // motion label. This prevents a transit/correction excursion from
        // bypassing the same overload interlock used during insertion.
        let force_violation = if self.active_gripper_closing
            && packet.grip_force_proxy_n > self.scenario.grasp.maximum_grip_force_n
        {
            Some((
                ForceInterlockChannel::Gripper,
                packet.grip_force_proxy_n,
                self.scenario.grasp.maximum_grip_force_n,
            ))
        } else if was_active
            && packet.insertion_force_proxy_n > self.scenario.contact.maximum_force_proxy_n
        {
            Some((
                ForceInterlockChannel::Insertion,
                packet.insertion_force_proxy_n,
                self.scenario.contact.maximum_force_proxy_n,
            ))
        } else {
            None
        };
        if let Some((channel, measured_n, limit_n)) = force_violation {
            let _ = self.trip_force_interlock(channel, measured_n, limit_n, was_active, packet);
            return Err(collision_failure.unwrap_or(PlantFailure::ContactForceLimit));
        }
        if let Some(failure) = collision_failure {
            return Err(failure);
        }
        if was_active && self.motion_status() != MotionStatus::Active {
            self.last_motion_complete_tick = self.now_tick();
            self.active_motion_class = None;
            self.active_gripper_closing = false;
        }
        Ok(())
    }

    fn trip_force_interlock(
        &mut self,
        channel: ForceInterlockChannel,
        measured_force_proxy_n: f64,
        limit_force_proxy_n: f64,
        motion_was_active: bool,
        packet: ContactPacket,
    ) -> Result<(), PlantFailure> {
        let tick = self.now_tick();
        let stop_command_sequence = self
            .mechanics
            .submit_machine_command(MachineCommand::Stop {
                manipulator: Some(ACTIVE_MANIPULATOR_ID),
            })
            .ok();
        self.last_motion_complete_tick = tick;
        self.active_motion_class = None;
        self.active_gripper_closing = false;
        self.last_force_interlock_event = Some(ForceInterlockEvent {
            tick,
            channel,
            measured_force_proxy_n,
            limit_force_proxy_n,
            motion_was_active,
            stop_command_sequence,
            packet,
        });
        Err(PlantFailure::ContactForceLimit)
    }

    pub fn advance_ticks(&mut self, ticks: u64) -> Result<(), PlantFailure> {
        for _ in 0..ticks {
            self.advance_one()?;
        }
        Ok(())
    }

    pub fn advance_until_idle(&mut self, maximum_steps: u32) -> Result<(), PlantFailure> {
        for _ in 0..maximum_steps {
            if self.motion_status() != MotionStatus::Active {
                return Ok(());
            }
            self.advance_one()?;
        }
        Err(PlantFailure::MechanicsFailure)
    }

    /// Synchronously execute one stationary coded burst.  Capture and latency
    /// consume whole fixed ticks.  Each requested feature is sampled on three
    /// (scenario-configurable) distinct exposure ticks; all samples become
    /// visible together after the processing-latency interval.
    pub fn acquire_observation_burst(
        &mut self,
        object_ids: &[u32],
        roi_center_world_m: [f64; 3],
    ) -> Result<ObservationBurst, PlantFailure> {
        if self.motion_status() == MotionStatus::Active {
            return Err(PlantFailure::MotionNotStopped);
        }
        let settling_ticks = self.motion_capabilities().settling_ticks;
        if self
            .now_tick()
            .saturating_sub(self.last_motion_complete_tick)
            < settling_ticks
        {
            return Err(PlantFailure::ObservationBeforeSettled);
        }
        let roi = array_optical_vec3(roi_center_world_m);
        if !roi.is_finite() {
            return Err(PlantFailure::InvalidCommand);
        }
        let mut requested_object_ids = object_ids.to_vec();
        requested_object_ids.sort_unstable();
        requested_object_ids.dedup();
        if requested_object_ids
            .iter()
            .any(|id| !matches!(*id, PEG_OBJECT_ID | SOCKET_OBJECT_ID | TOOL_OBJECT_ID))
        {
            return Err(PlantFailure::InvalidCommand);
        }

        let capture_duration_ticks = ticks_ceil(
            f64::from(self.scenario.optics.pattern_count) / self.scenario.optics.pattern_rate_hz,
            self.fixed_dt_s(),
        )
        .max(1);
        let latency_ticks =
            ticks_ceil(self.scenario.optics.processing_latency_s, self.fixed_dt_s());
        let capture_start_tick = self.now_tick();
        let capture_end_tick = capture_start_tick + capture_duration_ticks.saturating_sub(1);
        let stale_extra_ticks = if self.fault == M1eFault::StaleObservation {
            ticks_ceil(
                self.scenario.optics.maximum_measurement_age_s,
                self.fixed_dt_s(),
            ) + 1
        } else {
            0
        };
        let available_tick = capture_end_tick + latency_ticks + stale_extra_ticks;
        let frame_count = u64::from(self.scenario.optics.burst_frame_count.max(1));
        let frame_ticks = distinct_frame_ticks(capture_start_tick, capture_end_tick, frame_count);
        let sequence = self.burst_sequence;
        let rig = self.build_macro_rig(roi, sequence);
        let mut measurements = Vec::new();
        let mut missing = Vec::new();

        for &capture_tick in &frame_ticks {
            if capture_tick > self.now_tick() {
                self.advance_ticks(capture_tick - self.now_tick())?;
            }
            let mut optical_scene = self.build_optical_scene();
            self.inject_fault_optical_geometry(&rig, &mut optical_scene);
            for &object_id in &requested_object_ids {
                let observations = if self.fault == M1eFault::OpticalDropout {
                    let features = self.truth_ring_center_points(object_id, roi)?;
                    features
                        .iter()
                        .map(|feature| FeaturePointObservation::Missing {
                            object_id: feature.object_id,
                            feature_id: feature.feature_id,
                            head: pipe_optics::TriangulationHead {
                                camera_id: self.scenario.optics.camera_id,
                                projector_id: self.scenario.optics.projector_id,
                            },
                            reason: MissingFeaturePoint::StochasticDropout,
                        })
                        .collect::<Vec<_>>()
                } else {
                    self.observe_fitted_ring_centers(
                        &rig,
                        &optical_scene,
                        object_id,
                        roi,
                        u64::from(sequence) << 32 | capture_tick,
                    )?
                };
                self.sanitize_feature_observations(
                    capture_tick,
                    available_tick,
                    observations,
                    &mut measurements,
                    &mut missing,
                );
            }
        }
        if self.now_tick() < available_tick {
            self.advance_ticks(available_tick - self.now_tick())?;
        }
        measurements.sort_by(|a, b| {
            (
                a.object_id,
                a.feature_id,
                a.head_id,
                a.capture_tick,
                a.available_tick,
            )
                .cmp(&(
                    b.object_id,
                    b.feature_id,
                    b.head_id,
                    b.capture_tick,
                    b.available_tick,
                ))
        });
        missing.sort_by(|a, b| {
            (a.object_id, a.feature_id, a.capture_tick, &a.reason).cmp(&(
                b.object_id,
                b.feature_id,
                b.capture_tick,
                &b.reason,
            ))
        });
        let (calibration_reference_residual_m, calibration_reference_sample_count) =
            self.observe_calibration_reference(&rig, roi, sequence, &frame_ticks);
        let calibration_reference_valid = calibration_reference_sample_count
            >= self.scenario.optics.minimum_calibration_reference_samples
            && calibration_reference_residual_m.is_some_and(|residual_m| {
                residual_m
                    <= self
                        .scenario
                        .optics
                        .maximum_calibration_reference_residual_m
            });
        self.burst_sequence = self.burst_sequence.saturating_add(1);
        Ok(ObservationBurst {
            sequence,
            capture_start_tick,
            capture_end_tick,
            available_tick,
            requested_object_ids,
            triangulation_head_count: u32::from(!measurements.is_empty()),
            calibrated_rays_per_observed_point: 2,
            calibration_reference_valid,
            calibration_reference_residual_m,
            calibration_reference_sample_count,
            measurements,
            missing,
        })
    }

    /// Hardware-plausible raw channels only. The plant deliberately does not
    /// assign grasp, insertion, jam, or seating states; the controller combines
    /// this packet with fresh estimator-derived relative geometry.
    pub fn contact_packet(&mut self) -> ContactPacket {
        let packet = self.sample_contact_packet();
        self.peak_grip_force_proxy_n = self.peak_grip_force_proxy_n.max(packet.grip_force_proxy_n);
        self.peak_insertion_force_proxy_n = self
            .peak_insertion_force_proxy_n
            .max(packet.insertion_force_proxy_n);
        packet
    }

    fn sample_contact_packet(&self) -> ContactPacket {
        let (
            left_pad_contact,
            right_pad_contact,
            left_deflection_m,
            right_deflection_m,
            grip_force_proxy_n,
        ) = self.private_jaw_contact_channels();
        let (contact_detected, insertion_force_proxy_n) = self.private_insertion_contact();
        ContactPacket {
            captured_at_tick: self.now_tick(),
            contact_detected,
            left_pad_contact,
            right_pad_contact,
            left_pad_deflection_m: left_deflection_m,
            right_pad_deflection_m: right_deflection_m,
            grip_force_proxy_n,
            insertion_force_proxy_n,
        }
    }

    /// Transactional plant mutation after the controller has authorized a
    /// grasp from fresh observations and a classified raw contact packet. The
    /// private recheck prevents a stale authorization from forcing attachment.
    pub fn commit_grasp(&mut self) -> Result<(), PlantFailure> {
        let palm_peg_penetration_m = self.private_palm_peg_penetration_m();
        self.maximum_unplanned_penetration_m = self
            .maximum_unplanned_penetration_m
            .max(palm_peg_penetration_m);
        if palm_peg_penetration_m > 1.0e-12 {
            return Err(PlantFailure::GraspOutsideCaptureRegion);
        }
        let (left_contact, right_contact, _, _, force_proxy_n) =
            self.private_jaw_contact_channels();
        if !(left_contact && right_contact) {
            return Err(PlantFailure::GraspOutsideCaptureRegion);
        }
        if force_proxy_n < self.scenario.grasp.minimum_grip_force_n
            || force_proxy_n > self.scenario.grasp.maximum_grip_force_n
        {
            return Err(PlantFailure::GraspForceOutOfBounds);
        }
        self.mechanics
            .grasp_body_serial_with_partial_axial_overlap(
                ACTIVE_ARM_ID,
                BodyId(PEG_OBJECT_ID),
                self.scenario.grasp.minimum_axial_grasp_overlap_m,
            )
            .map_err(|_| PlantFailure::GraspOutsideCaptureRegion)?;
        self.apply_held_transform_disturbance();
        self.peak_grip_force_proxy_n = self.peak_grip_force_proxy_n.max(force_proxy_n);
        self.grasp_committed = true;
        Ok(())
    }

    /// Transactional detach after controller-side seating authorization. This
    /// rechecks only raw load-path evidence and attachment ownership; it does
    /// not reconstruct a truth-derived seating verdict.
    pub fn commit_release(&mut self) -> Result<(), PlantFailure> {
        if !self.peg_physically_held() {
            return Err(PlantFailure::PegNotHeld);
        }
        let (contact_detected, insertion_force_proxy_n) = self.private_insertion_contact();
        if !contact_detected
            || insertion_force_proxy_n > self.scenario.contact.maximum_force_proxy_n
        {
            return Err(PlantFailure::ReleaseConditionNotMet);
        }
        self.mechanics
            .release_body_serial(ACTIVE_ARM_ID)
            .map_err(|_| PlantFailure::MechanicsFailure)?;
        self.release_committed = true;
        Ok(())
    }

    /// Truth-only metrics for acceptance evaluation.  Never pass these values
    /// into an estimator, planner, guard, or task-state transition.
    pub fn evaluation_metrics(&self) -> PlantEvaluationMetrics {
        let peg = self.peg_body();
        let peg_tip_world = peg.pose.transform_point(
            Vec3::Z
                * (self.scenario.coupon.peg_half_segment_m
                    + 0.5 * self.scenario.coupon.peg_diameter_m),
        );
        let tip_in_socket = self.socket_pose.inverse_transform_point(peg_tip_world);
        let local = tip_in_socket - Vec3::Z * (0.5 * self.scenario.coupon.socket_depth_m);
        let peg_axis = peg.pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
        let socket_axis = self
            .socket_pose
            .transform_vector(Vec3::Z)
            .normalized_or(Vec3::Z);
        let axis_error = peg_axis
            .cross(socket_axis)
            .length()
            .atan2(peg_axis.dot(socket_axis).clamp(-1.0, 1.0));
        let tool = self.active_arm().tool_pose().translation;
        PlantEvaluationMetrics {
            final_peg_tip_to_socket_seat_error_m: local.length(),
            final_peg_lateral_error_m: local.x.hypot(local.y),
            final_peg_axial_error_m: local.z.abs(),
            final_peg_axis_error_rad: axis_error,
            final_tool_center_to_socket_center_distance_m: (tool - self.socket_pose.translation)
                .length(),
            physical_grasp_attachment_present: self.peg_physically_held(),
            grasp_committed: self.grasp_committed,
            release_committed: self.release_committed,
            maximum_unplanned_penetration_m: self.maximum_unplanned_penetration_m,
            peak_grip_force_proxy_n: self.peak_grip_force_proxy_n,
            peak_insertion_force_proxy_n: self.peak_insertion_force_proxy_n,
        }
    }

    fn active_arm(&self) -> &pipe_sim_core::SerialArmInstance {
        self.mechanics
            .serial_arm(ACTIVE_ARM_ID)
            .expect("M1e plant owns active arm 1")
    }

    /// Physical distal tool axis after the modeled mounting/compliance tilt.
    /// Position commands still terminate at the authoritative FK flange; M1e
    /// has no orientation actuator and must observe/gate this mismatch.
    fn physical_tool_pose(&self) -> Pose {
        let flange = self.active_arm().tool_pose();
        Pose::new(
            flange.translation,
            (flange.rotation * self.physical_tool_axis_tilt).normalized(),
        )
    }

    fn peg_body(&self) -> &RigidBody {
        self.mechanics
            .body(BodyId(PEG_OBJECT_ID))
            .expect("M1e plant owns calibration peg")
    }

    fn peg_physically_held(&self) -> bool {
        self.active_arm().gripper.held_body == Some(BodyId(PEG_OBJECT_ID))
    }

    fn validate_tool_motion_request(
        &self,
        requested: Vec3,
        motion_class: MotionClass,
    ) -> Result<(), PlantFailure> {
        if !requested.is_finite() {
            return Err(PlantFailure::InvalidCommand);
        }
        let commanded_distance =
            (requested - array_vec3(self.commanded_tool_position_world_m)).length();
        if motion_class == MotionClass::Correction
            && commanded_distance > self.scenario.motion.maximum_correction_m + 1.0e-15
        {
            return Err(PlantFailure::CorrectionMagnitudeLimit);
        }
        if motion_class == MotionClass::Insertion
            && commanded_distance > self.scenario.motion.insertion_increment_m + 1.0e-15
        {
            return Err(PlantFailure::InsertionIncrementLimit);
        }
        if motion_class == MotionClass::Correction
            && self.motion_capabilities().minimum_reproducible_correction_m
                > 2.0 * self.scenario.motion.correction_convergence_m
        {
            return Err(PlantFailure::CorrectionFloorTooLarge);
        }
        Ok(())
    }

    fn effective_motion_target(
        &self,
        actual_current: Vec3,
        requested: Vec3,
        motion_class: MotionClass,
    ) -> Vec3 {
        let requested_delta = requested - actual_current;
        let magnitude = requested_delta.length();
        let floor = self.motion_capabilities().minimum_reproducible_correction_m;
        let executed_magnitude = if magnitude <= 1.0e-15 {
            0.0
        } else {
            (magnitude / floor).round() * floor
        };
        let mut effective = if magnitude > 1.0e-15 {
            actual_current + requested_delta * (executed_magnitude / magnitude)
        } else {
            actual_current
        };
        for axis in 0..3 {
            let direction = sign_i8(requested_delta.component(axis));
            if direction != 0
                && self.previous_motion_direction[axis] != 0
                && direction != self.previous_motion_direction[axis]
            {
                let lost_motion = 0.5 * self.scenario.motion.differential_backlash_m;
                effective = effective.with_component(
                    axis,
                    effective.component(axis) - f64::from(direction) * lost_motion,
                );
            }
        }
        if self.peg_physically_held() {
            effective += array_vec3(self.scenario.motion.loaded_hold_error_world_m);
        }
        if self.fault == M1eFault::NonConvergence && motion_class == MotionClass::Correction {
            // A deterministic lost-motion fault: the command is acknowledged,
            // but the distal tool does not respond.  The next observation, not
            // this private branch, is what lets the controller diagnose it.
            actual_current
        } else {
            effective
        }
    }

    /// Conservative Cartesian time-scaling for stop-and-look corrections,
    /// insertion increments, and contact-unloading retreats. The authoritative cubic tool plan scales joint
    /// speed by `s` and acceleration by `s^2`, hence its duration by `1/s`.
    /// These whole-chain bounds include carriage rotation and every upstream
    /// translating joint. Wrist roll is excluded because the tool point lies
    /// on its roll axis. Selecting `s` only tightens the machine limits.
    fn near_contact_tool_motion_speed_scale(&self) -> Result<f64, PlantFailure> {
        let arm = self.active_arm();
        let carriage = arm.carriage_config;
        let motion = arm.motion_config;
        let config = arm.arm.config;
        let theta_speed = carriage.max_theta_speed_rad_s;
        let theta_acceleration = carriage.max_theta_accel_rad_s2;
        let yaw_speed = motion.max_joint_speed_rad_s[0];
        let shoulder_speed = motion.max_joint_speed_rad_s[1];
        let elbow_speed = motion.max_joint_speed_rad_s[2];
        let yaw_acceleration = motion.max_joint_accel_rad_s2[0];
        let shoulder_acceleration = motion.max_joint_accel_rad_s2[1];
        let elbow_acceleration = motion.max_joint_accel_rad_s2[2];
        let distal_length_m = config.forearm_length_m + config.wrist_length_m;
        let unity_velocity_bound_m_s = carriage.max_z_speed_m_s
            + carriage.rail_radius_m * theta_speed
            + config.upper_arm_length_m * (theta_speed + yaw_speed + shoulder_speed)
            + distal_length_m * (theta_speed + yaw_speed + shoulder_speed + elbow_speed);
        let upper_acceleration_bound_rad_s2 = angular_chain_acceleration_bound(
            &[theta_speed, yaw_speed, shoulder_speed],
            &[theta_acceleration, yaw_acceleration, shoulder_acceleration],
        );
        let distal_acceleration_bound_rad_s2 = angular_chain_acceleration_bound(
            &[theta_speed, yaw_speed, shoulder_speed, elbow_speed],
            &[
                theta_acceleration,
                yaw_acceleration,
                shoulder_acceleration,
                elbow_acceleration,
            ],
        );
        let unity_acceleration_bound_m_s2 = carriage.max_z_accel_m_s2
            + carriage.rail_radius_m * (theta_acceleration + theta_speed * theta_speed)
            + config.upper_arm_length_m * upper_acceleration_bound_rad_s2
            + distal_length_m * distal_acceleration_bound_rad_s2;
        if !unity_velocity_bound_m_s.is_finite()
            || unity_velocity_bound_m_s <= 0.0
            || !unity_acceleration_bound_m_s2.is_finite()
            || unity_acceleration_bound_m_s2 <= 0.0
        {
            return Err(PlantFailure::ImpossibleGeometry);
        }
        let scale = arm
            .tool_motion_speed_scale
            .min(self.scenario.motion.maximum_correction_velocity_m_s / unity_velocity_bound_m_s)
            .min(
                (self.scenario.motion.maximum_correction_acceleration_m_s2
                    / unity_acceleration_bound_m_s2)
                    .sqrt(),
            );
        if !scale.is_finite() || scale <= 0.0 {
            return Err(PlantFailure::InvalidCommand);
        }
        Ok(scale.min(1.0))
    }

    fn update_motion_directions(&mut self, delta: Vec3) {
        for axis in 0..3 {
            let direction = sign_i8(delta.component(axis));
            if direction != 0 {
                self.previous_motion_direction[axis] = direction;
            }
        }
    }

    fn sanitize_command_error(&self, error: SimulationError) -> PlantFailure {
        match error {
            SimulationError::InvalidMachineCommand(MachineCommandError::ToolPathCollision) => {
                PlantFailure::MotionCollisionRisk
            }
            SimulationError::InvalidMachineCommand(
                MachineCommandError::ToolTargetUnreachable
                | MachineCommandError::ToolTargetJointLimits
                | MachineCommandError::ToolPathSamplingLimit,
            ) => PlantFailure::ImpossibleGeometry,
            SimulationError::InvalidMachineCommand(_) => PlantFailure::InvalidCommand,
            _ => PlantFailure::MechanicsFailure,
        }
    }

    fn build_macro_rig(&self, roi: OpticalVec3, sequence: u32) -> StructuredLightRig {
        let optics = &self.scenario.optics;
        let half_baseline = 0.5 * optics.camera_projector_baseline_m;
        let long_axis = array_optical_vec3(self.calibrated_socket_axis_world)
            .normalized()
            .unwrap_or(OpticalVec3::Z);
        let view_direction = array_optical_vec3(self.calibrated_macro_view_direction_world)
            .normalized()
            .unwrap_or(OpticalVec3::Y);
        let baseline_direction = view_direction
            .cross(long_axis)
            .normalized()
            .unwrap_or(OpticalVec3::X);
        let head_midpoint = roi + view_direction * optics.working_distance_m;
        let camera_origin = head_midpoint - baseline_direction * half_baseline;
        let projector_origin = head_midpoint + baseline_direction * half_baseline;
        let slant_range_m = optics.working_distance_m.hypot(half_baseline);
        let focal_x_px =
            slant_range_m * f64::from(optics.image_size_px[0]) / optics.field_size_at_target_m[0];
        let focal_y_px =
            slant_range_m * f64::from(optics.image_size_px[1]) / optics.field_size_at_target_m[1];
        let image = ImageSize::new(optics.image_size_px[0], optics.image_size_px[1]);
        let intrinsics = CameraIntrinsics::new(
            focal_x_px,
            focal_y_px,
            0.5 * (f64::from(image.width) - 1.0),
            0.5 * (f64::from(image.height) - 1.0),
        );
        let drift_world = array_optical_vec3(optics.calibration_bias_m)
            + array_optical_vec3(optics.drift_per_burst_m) * f64::from(sequence);
        let fault_world = if self.fault == M1eFault::ExcessiveCalibrationBias {
            OpticalVec3::new(CALIBRATION_FAULT_BIAS_M, 0.0, 0.0)
        } else {
            OpticalVec3::ZERO
        };
        let camera_nominal = PinholeCamera::new(
            image,
            intrinsics,
            BrownConrady::NONE,
            optical_look_at_with_long_axis(camera_origin, roi, long_axis),
        );
        let projector_nominal = PinholeCamera::new(
            image,
            intrinsics,
            BrownConrady::NONE,
            optical_look_at_with_long_axis(projector_origin, roi, long_axis),
        );
        let camera_local_drift =
            camera_nominal.world_from_camera.rotation.transpose() * (drift_world + fault_world);
        let projector_local_drift =
            projector_nominal.world_from_camera.rotation.transpose() * (drift_world + fault_world);
        let mut camera = CalibratedCamera::new(optics.camera_id, camera_nominal);
        camera.drift = CalibrationDrift {
            translation_m: camera_local_drift,
            ..CalibrationDrift::ZERO
        };
        let mut projector = CalibratedCamera::new(optics.projector_id, projector_nominal);
        projector.drift = CalibrationDrift {
            translation_m: projector_local_drift,
            ..CalibrationDrift::ZERO
        };
        let config = ScanConfig {
            camera_pixel_sigma_floor_px: optics.camera_localization_sigma_px,
            projector_pixel_sigma_floor_px: optics.projector_localization_sigma_px,
            photon_centroid_coefficient_px: 2.0,
            depth_quantization_m: 0.5e-6,
            speckle_axial_sigma_m: 1.5e-6,
            // The estimator adds this same-head correlated floor exactly once.
            correlated_calibration_sigma_m: 0.0,
            // Transparent-wall throughput is applied once in each feature's
            // optical material. Keeping the clear-air reference unscaled avoids
            // accidentally squaring the declared attenuation.
            reference_photoelectrons: 18_000.0,
            reference_range_m: slant_range_m,
            ambient_photoelectrons: 120.0,
            read_noise_electrons: 4.0,
            base_dropout_probability: optics.base_dropout_probability,
            grazing_dropout_probability: optics.grazing_dropout_probability,
            minimum_feature_confidence: optics.minimum_confidence,
            maximum_ray_separation_m: 120.0e-6,
            ..ScanConfig::default()
        };
        StructuredLightRig::new(vec![camera], projector, config, self.scenario.seed)
    }

    fn build_optical_scene(&self) -> OpticalScene {
        let mut scene = OpticalScene::default();
        let neutral = optical_material(self.scenario.optics.clear_wall_signal_scale, 1.0);
        let fiducial = optical_material(self.scenario.optics.clear_wall_signal_scale, 2.5);

        let peg = self.peg_body();
        if let Shape::Capsule {
            radius_m,
            half_segment_m,
        } = peg.shape
        {
            push_optical_capsule(
                &mut scene,
                peg.pose,
                radius_m,
                half_segment_m,
                fiducial,
                OPTICAL_TAG_PEG,
            );
        }
        for socket_id in SOCKET_BODY_IDS {
            if let Some(wall) = self.mechanics.body(socket_id) {
                if let Shape::Box { half_extents_m } = wall.shape {
                    push_optical_box(
                        &mut scene,
                        wall.pose,
                        half_extents_m,
                        neutral,
                        OPTICAL_TAG_SOCKET,
                    );
                }
            }
        }
        // Two sparse external coded rails establish socket centerline points
        // by measured midpoint. They sit outside the opaque square coupon, so
        // a rigid side-looking macro head need not see through a socket wall.
        for (sign, tag) in [
            (1.0, OPTICAL_TAG_SOCKET_FIDUCIAL_POSITIVE),
            (-1.0, OPTICAL_TAG_SOCKET_FIDUCIAL_NEGATIVE),
        ] {
            let center_local =
                Vec3::X * (sign * self.scenario.coupon.socket_fiducial_lateral_offset_m);
            let start = optical_vec(self.socket_pose.transform_point(
                center_local - Vec3::Z * self.scenario.coupon.socket_fiducial_axial_half_extent_m,
            ));
            let end = optical_vec(self.socket_pose.transform_point(
                center_local + Vec3::Z * self.scenario.coupon.socket_fiducial_axial_half_extent_m,
            ));
            scene.push(OpticalPrimitive::new(
                OpticalGeometry::Cylinder(pipe_optics::Cylinder {
                    start,
                    end,
                    radius_m: self.scenario.coupon.socket_fiducial_radius_m,
                    capped: true,
                }),
                fiducial,
                tag,
            ));
        }
        for (index, obstacle) in self.scenario.safety.planning_obstacles.iter().enumerate() {
            scene.push(OpticalPrimitive::new(
                OpticalGeometry::Sphere(OpticalSphere {
                    center: array_optical_vec3(obstacle.center_world_m),
                    radius_m: obstacle.conservative_radius_m,
                }),
                neutral,
                OPTICAL_TAG_FIXTURE_BASE + index as u32,
            ));
        }
        let arm = self.active_arm();
        let capsule_count = arm.kinematics.collision_capsules.len();
        for (index, (pose, shape)) in arm.kinematics.collision_capsules.iter().enumerate() {
            if let Shape::Capsule {
                radius_m,
                half_segment_m,
            } = *shape
            {
                let is_terminal = index + 1 == capsule_count;
                if is_terminal {
                    // Mechanics deliberately sweeps a rounded capsule, whose
                    // radius is a conservative safety envelope rather than a
                    // literal optical mesh. Represent the calibrated terminal
                    // tube plus a small, laterally offset coded target spanning
                    // the flange. This makes feature placement physical, keeps
                    // it clear of the main-link/jaw shadows, and does not alter
                    // the larger collision envelope enforced by mechanics.
                    push_optical_terminal_tool(
                        &mut scene,
                        TerminalToolOpticalInput {
                            terminal_link_pose: *pose,
                            physical_tool_pose: self.physical_tool_pose(),
                            collision_envelope_radius_m: radius_m,
                            half_segment_m,
                            fiducial_boss_radius_m: arm.gripper_config.jaw_half_extents_m.y,
                            tool_geometry: self.calibrated_tool_geometry,
                            link_material: neutral,
                            fiducial_material: fiducial,
                        },
                    );
                } else {
                    push_optical_capsule(
                        &mut scene,
                        *pose,
                        radius_m,
                        half_segment_m,
                        neutral,
                        OPTICAL_TAG_ARM_BASE + index as u32,
                    );
                }
            }
        }
        for (pose, shape) in arm
            .gripper
            .jaw_poses(self.physical_tool_pose(), arm.gripper_config)
            .into_iter()
            .zip(pipe_sim_core::GripperState::jaw_shapes(arm.gripper_config))
        {
            let Shape::Box { half_extents_m } = shape else {
                continue;
            };
            push_optical_box(
                &mut scene,
                pose,
                half_extents_m,
                neutral,
                OPTICAL_TAG_GRIPPER,
            );
        }
        scene
    }

    fn inject_fault_optical_geometry(&self, rig: &StructuredLightRig, scene: &mut OpticalScene) {
        if self.fault != M1eFault::OccludedMatingFeature {
            return;
        }
        let socket_center = optical_vec(self.socket_pose.translation);
        let camera_center = rig
            .cameras
            .first()
            .map(|camera| camera.actual().center_world())
            .unwrap_or(socket_center + OpticalVec3::Y);
        let toward_camera = (camera_center - socket_center)
            .normalized()
            .unwrap_or(OpticalVec3::Y);
        // Opaque optical-only coupon flag on the socket-to-camera center ray.
        // It is deliberately not a mechanics body: this injected fault is a
        // lost view rather than a collision. The radius covers both symmetric
        // +/-1 mm coded rails plus their fitted-ring probes, so ordinary
        // camera visibility rays reject every socket feature.
        scene.push(OpticalPrimitive::new(
            OpticalGeometry::Sphere(OpticalSphere {
                center: socket_center + toward_camera * 1.5e-3,
                radius_m: 1.30e-3,
            }),
            optical_material(1.0, 1.0),
            OPTICAL_TAG_FAULT_OCCLUDER,
        ));
    }

    fn truth_ring_center_points(
        &self,
        object_id: u32,
        roi: OpticalVec3,
    ) -> Result<Vec<FeaturePoint>, PlantFailure> {
        let (pose, half_span, self_tag) = match object_id {
            PEG_OBJECT_ID => (
                self.peg_body().pose,
                0.5 * self.scenario.coupon.minimum_feature_axial_span_m,
                OPTICAL_TAG_PEG,
            ),
            SOCKET_OBJECT_ID => (
                self.socket_pose,
                0.5 * self.scenario.coupon.minimum_feature_axial_span_m,
                OPTICAL_TAG_SOCKET,
            ),
            TOOL_OBJECT_ID => (
                self.physical_tool_pose(),
                self.calibrated_tool_geometry
                    .side_target_feature_half_span_m,
                OPTICAL_TAG_TOOL_FIDUCIAL_POSITIVE,
            ),
            _ => return Err(PlantFailure::InvalidCommand),
        };
        let head_midpoint = roi
            + array_optical_vec3(self.calibrated_macro_view_direction_world)
                * self.scenario.optics.working_distance_m;
        let material = optical_material(self.scenario.optics.clear_wall_signal_scale, 3.0);
        let model = axial_feature_model(object_id, half_span);
        Ok(model
            .into_iter()
            .map(|feature| {
                let point = pose.transform_point(Vec3::Z * feature.axial_coordinate_m);
                let point = optical_vec(point);
                FeaturePoint {
                    object_id,
                    feature_id: feature.feature_id,
                    point_world: point,
                    normal_world: (head_midpoint - point)
                        .normalized()
                        .unwrap_or(OpticalVec3::Y),
                    material,
                    // The feature is a fitted ring-center result.  Its own
                    // carrier is ignored, but every other robot/fixture solid
                    // remains an occluder.
                    self_occlusion_tag: Some(self_tag),
                }
            })
            .collect())
    }

    /// Apply the explicit fitted-ring visibility proxy before producing any
    /// centerline measurement consumed by the 5-DoF estimator.
    ///
    /// Seven surface probes cover a 120-degree arc about the head-facing
    /// meridian.  At least three usable probes must span 40 degrees and must
    /// bracket that meridian.  The carrier itself is ignored only for its own
    /// probes.  Peg, socket, gripper, terminal-link, and proximal-arm tags stay
    /// distinct, so every other local actor remains a real camera/projector
    /// occluder.  Once the arc gate passes, the virtual ring center is measured
    /// through the same structured-light path in a scene with only its own
    /// carrier removed.  This models a fitted geometric feature; it does not
    /// claim image segmentation or a photometric ellipse fit.
    fn observe_fitted_ring_centers(
        &self,
        rig: &StructuredLightRig,
        full_scene: &OpticalScene,
        object_id: u32,
        roi: OpticalVec3,
        frame_index: u64,
    ) -> Result<Vec<FeaturePointObservation>, PlantFailure> {
        let centers = self.truth_ring_center_points(object_id, roi)?;
        let feature_model = self.feature_model(object_id);
        let (axis, carrier_tag) = self.ring_probe_axis_and_carrier(object_id)?;
        let head_midpoint = roi
            + array_optical_vec3(self.calibrated_macro_view_direction_world)
                * self.scenario.optics.working_distance_m;
        let mut result = Vec::with_capacity(centers.len());
        for (center, feature) in centers.into_iter().zip(feature_model) {
            let probe_radius_m = self.ring_probe_radius(object_id, feature.axial_coordinate_m)?;
            if matches!(object_id, TOOL_OBJECT_ID | SOCKET_OBJECT_ID) {
                // Two independently observed target rails are mounted at
                // symmetric local offsets. Their *measured* midpoint is the
                // object-axis feature. No true/current pose-derived offset is
                // subtracted after sensor reconstruction, so roll remains
                // genuinely unobservable in this reduced 5-DoF model.
                let (offset_world, positive_tag, negative_tag) = if object_id == TOOL_OBJECT_ID {
                    (
                        optical_vec(
                            self.physical_tool_pose().transform_vector(array_vec3(
                                self.calibrated_tool_geometry
                                    .side_target_center_offset_tool_m,
                            )),
                        ),
                        OPTICAL_TAG_TOOL_FIDUCIAL_POSITIVE,
                        OPTICAL_TAG_TOOL_FIDUCIAL_NEGATIVE,
                    )
                } else {
                    (
                        optical_vec(self.socket_pose.transform_vector(
                            Vec3::X * self.scenario.coupon.socket_fiducial_lateral_offset_m,
                        )),
                        OPTICAL_TAG_SOCKET_FIDUCIAL_POSITIVE,
                        OPTICAL_TAG_SOCKET_FIDUCIAL_NEGATIVE,
                    )
                };
                let mut positive = center;
                positive.point_world += offset_world;
                let mut negative = center;
                negative.point_world -= offset_world;
                let positive_observation = self.observe_one_fitted_ring_center(
                    rig,
                    full_scene,
                    positive,
                    axis,
                    probe_radius_m,
                    positive_tag,
                    head_midpoint,
                    frame_index ^ 0x51DE_0001,
                );
                let negative_observation = self.observe_one_fitted_ring_center(
                    rig,
                    full_scene,
                    negative,
                    axis,
                    probe_radius_m,
                    negative_tag,
                    head_midpoint,
                    frame_index ^ 0x51DE_0002,
                );
                result.push(midpoint_paired_tool_observations(
                    positive_observation,
                    negative_observation,
                ));
            } else {
                result.push(self.observe_one_fitted_ring_center(
                    rig,
                    full_scene,
                    center,
                    axis,
                    probe_radius_m,
                    carrier_tag,
                    head_midpoint,
                    frame_index,
                ));
            }
        }
        Ok(result)
    }

    #[allow(clippy::too_many_arguments)]
    fn observe_one_fitted_ring_center(
        &self,
        rig: &StructuredLightRig,
        full_scene: &OpticalScene,
        mut center: FeaturePoint,
        axis: OpticalVec3,
        probe_radius_m: f64,
        carrier_tag: u32,
        head_midpoint: OpticalVec3,
        frame_index: u64,
    ) -> FeaturePointObservation {
        let head = pipe_optics::TriangulationHead {
            camera_id: self.scenario.optics.camera_id,
            projector_id: self.scenario.optics.projector_id,
        };
        center.normal_world = (head_midpoint - center.point_world)
            .normalized()
            .unwrap_or(OpticalVec3::Y);
        // Surface probes retain the carrier in the scene so rear/grazing arcs
        // can self-shadow. Only the virtual fitted center below removes that
        // carrier after the visible-arc gate has passed.
        center.self_occlusion_tag = None;
        let probes = ring_arc_probes(center, axis, probe_radius_m, head_midpoint);
        let probe_frame = frame_index
            ^ (u64::from(center.object_id) << 16)
            ^ (u64::from(center.feature_id) << 8)
            ^ 0xA2C0_0000_u64;
        let probe_observations = rig.observe_feature_points(full_scene, probe_frame, &probes);
        let (usable, occlusion_reason) = usable_ring_arc_geometry(&probe_observations);
        if !usable {
            return FeaturePointObservation::Missing {
                object_id: center.object_id,
                feature_id: center.feature_id,
                head,
                reason: occlusion_reason.unwrap_or(MissingFeaturePoint::InsufficientQuality),
            };
        }

        let center_scene = OpticalScene::new(
            full_scene
                .primitives
                .iter()
                .copied()
                .filter(|primitive| primitive.tag != carrier_tag)
                .collect(),
        );
        center.self_occlusion_tag = None;
        let center_frame = frame_index
            ^ (u64::from(center.object_id) << 24)
            ^ (u64::from(center.feature_id) << 12)
            ^ 0xC37E_0000_u64;
        rig.observe_feature_points(&center_scene, center_frame, &[center])
            .into_iter()
            .next()
            .expect("one fitted center produces one head observation")
    }

    fn ring_probe_axis_and_carrier(
        &self,
        object_id: u32,
    ) -> Result<(OpticalVec3, u32), PlantFailure> {
        let (pose, carrier_tag) = match object_id {
            PEG_OBJECT_ID => (self.peg_body().pose, OPTICAL_TAG_PEG),
            SOCKET_OBJECT_ID => (self.socket_pose, OPTICAL_TAG_SOCKET),
            // Coded circumferential bands on the terminal member. The jaws are
            // separate occluders, not part of this carrier.
            TOOL_OBJECT_ID => (
                self.physical_tool_pose(),
                OPTICAL_TAG_TOOL_FIDUCIAL_POSITIVE,
            ),
            _ => return Err(PlantFailure::InvalidCommand),
        };
        Ok((
            optical_vec(pose.transform_vector(Vec3::Z))
                .normalized()
                .unwrap_or(OpticalVec3::Z),
            carrier_tag,
        ))
    }

    fn ring_probe_radius(
        &self,
        object_id: u32,
        axial_coordinate_m: f64,
    ) -> Result<f64, PlantFailure> {
        match object_id {
            PEG_OBJECT_ID => {
                let cylinder_radius_m = 0.5 * self.scenario.coupon.peg_diameter_m;
                capsule_latitude_radius(
                    cylinder_radius_m,
                    self.scenario.coupon.peg_half_segment_m,
                    axial_coordinate_m,
                )
                .ok_or(PlantFailure::ImpossibleGeometry)
            }
            SOCKET_OBJECT_ID => Ok(self.scenario.coupon.socket_fiducial_radius_m),
            // The coded tool stations are on the calibrated side target, not
            // on the conservative 1.8 mm-radius terminal collision envelope.
            TOOL_OBJECT_ID => Ok(self.calibrated_tool_geometry.side_target_radius_m),
            _ => Err(PlantFailure::InvalidCommand),
        }
    }

    fn sanitize_feature_observations(
        &self,
        capture_tick: u64,
        available_tick: u64,
        observations: Vec<FeaturePointObservation>,
        measurements: &mut Vec<FeatureMeasurement>,
        missing: &mut Vec<MissingFeatureMeasurement>,
    ) {
        for observation in observations {
            match observation {
                FeaturePointObservation::Observed(sample) => {
                    let mut point = [
                        sample.measured_point.x,
                        sample.measured_point.y,
                        sample.measured_point.z,
                    ];
                    if self.fault == M1eFault::InconsistentObservation
                        && matches!(sample.feature_id, 1 | 2)
                    {
                        point[0] += OUTLIER_FAULT_OFFSET_M;
                    }
                    let covariance = sample.covariance.matrix_m2.m;
                    // FeatureMeasurement intentionally carries only a diagonal.
                    // Copying the raw diagonal would discard triangulation
                    // correlations and can understate a rotated worst direction.
                    // An isotropic diagonal at the maximum absolute Gershgorin
                    // row sum upper-bounds every eigenvalue of the symmetric
                    // 3x3 covariance, preserving conservative uncertainty.
                    let Some(variance_upper_bound_m2) =
                        conservative_covariance_variance_bound(covariance)
                    else {
                        missing.push(MissingFeatureMeasurement {
                            object_id: sample.object_id,
                            feature_id: sample.feature_id,
                            capture_tick,
                            reason: missing_feature_reason(
                                MissingFeaturePoint::InvalidConfiguration,
                            )
                            .to_owned(),
                        });
                        continue;
                    };
                    measurements.push(FeatureMeasurement {
                        object_id: sample.object_id,
                        feature_id: sample.feature_id,
                        head_id: self.scenario.optics.head_id,
                        calibrated_ray_count: sample.calibrated_ray_count,
                        capture_tick,
                        available_tick,
                        measured_point_world_m: point,
                        covariance_diagonal_m2: [variance_upper_bound_m2; 3],
                        confidence: sample.quality.confidence,
                    });
                }
                FeaturePointObservation::Missing {
                    object_id,
                    feature_id,
                    reason,
                    ..
                } => missing.push(MissingFeatureMeasurement {
                    object_id,
                    feature_id,
                    capture_tick,
                    reason: missing_feature_reason(reason).to_owned(),
                }),
            }
        }
    }

    fn observe_calibration_reference(
        &self,
        rig: &StructuredLightRig,
        roi: OpticalVec3,
        sequence: u32,
        capture_ticks: &[u64],
    ) -> (Option<f64>, u32) {
        // A rigid reference point in the macro-head target plane is measured
        // through exactly the same calibrated camera/projector reconstruction
        // path as task features.  The controller sees only this residual/health
        // packet, never the known point or the rig's physical drift.
        let Some(camera) = rig.cameras.first() else {
            return (None, 0);
        };
        let camera_center = camera.actual().center_world();
        let feature = FeaturePoint {
            object_id: CALIBRATION_DATUM_OBJECT_ID,
            feature_id: 1,
            point_world: roi,
            normal_world: (camera_center - roi).normalized().unwrap_or(OpticalVec3::Y),
            material: optical_material(self.scenario.optics.clear_wall_signal_scale, 3.0),
            self_occlusion_tag: None,
        };
        let mut residuals = capture_ticks
            .iter()
            .filter_map(|capture_tick| {
                let frame_index = (u64::from(sequence) << 32) ^ *capture_tick ^ 0xCA11_BA7E_u64;
                match rig
                    .observe_feature_points(&OpticalScene::default(), frame_index, &[feature])
                    .into_iter()
                    .next()?
                {
                    FeaturePointObservation::Observed(sample) => {
                        Some((sample.measured_point - roi).norm())
                    }
                    FeaturePointObservation::Missing { .. } => None,
                }
            })
            .collect::<Vec<_>>();
        residuals.sort_by(f64::total_cmp);
        let sample_count = residuals.len() as u32;
        (residuals.get(residuals.len() / 2).copied(), sample_count)
    }

    fn apply_held_transform_disturbance(&mut self) {
        let sigma = self.scenario.estimator.held_transform_sigma_m;
        let disturbance = Vec3::new(0.50 * sigma, -0.25 * sigma, 0.375 * sigma);
        if let Some(arm) = self.mechanics.serial_arm_mut(ACTIVE_ARM_ID) {
            if let Some(local_pose) = arm.held_body_local_pose.as_mut() {
                local_pose.translation += disturbance;
            }
        }
    }

    /// Reduced hardware-plausible pad channels derived from the instantaneous
    /// jaw/peg capture geometry.  Attachment ownership is intentionally not an
    /// input: a real pad does not stop sensing contact merely because the M1e
    /// kinematic-attachment bookkeeping has transitioned to released.
    fn private_jaw_contact_channels(&self) -> (bool, bool, f64, f64, f64) {
        let arm = self.active_arm();
        let candidate = arm.gripper.evaluate_partial_axial_overlap_candidate(
            self.physical_tool_pose(),
            self.peg_body(),
            arm.gripper_config,
            self.scenario.grasp.minimum_axial_grasp_overlap_m,
        );
        if !candidate.within_finger_depth {
            return (false, false, 0.0, 0.0, 0.0);
        }
        let total_compression_m = (-candidate.jaw_clearance_m).max(0.0);
        let center_offset_m = candidate.center_error_m.x;
        let (left_deflection_m, right_deflection_m) = if total_compression_m > 0.0 {
            (
                (0.5 * total_compression_m - center_offset_m).max(0.0),
                (0.5 * total_compression_m + center_offset_m).max(0.0),
            )
        } else {
            // Center offset can redistribute real compression, but cannot turn
            // positive jaw clearance into one-sided contact across free space.
            (0.0, 0.0)
        };
        let left_contact = left_deflection_m > 0.0;
        let right_contact = right_deflection_m > 0.0;
        let force_proxy_n = if arm.gripper_config.pad_compliance_m > 0.0 {
            arm.gripper_config.max_grip_force_n
                * (total_compression_m / (2.0 * arm.gripper_config.pad_compliance_m))
                    .clamp(0.0, 1.0)
        } else if total_compression_m > 0.0 {
            arm.gripper_config.max_grip_force_n
        } else {
            0.0
        };
        (
            left_contact,
            right_contact,
            left_deflection_m,
            right_deflection_m,
            force_proxy_n,
        )
    }

    fn inject_grasp_capture_fault_if_needed(
        &mut self,
        target_opening_m: f64,
    ) -> Result<(), PlantFailure> {
        if self.fault != M1eFault::GraspOutsideCapture
            || self.grasp_capture_fault_applied
            || self.peg_physically_held()
            || target_opening_m >= self.scenario.coupon.peg_diameter_m
        {
            return Ok(());
        }
        // Fault coupon: an unmodeled support disturbance shifts the free peg
        // during jaw approach, after the last stop-and-look exposure. Move the
        // actual body rather than fabricating contact channels so the ensuing
        // unilateral/missing-pad evidence follows the same geometry as nominal.
        let jaw_axis = self
            .physical_tool_pose()
            .transform_vector(Vec3::X)
            .normalized_or(Vec3::X);
        let disturbance_m = 2.5 * self.scenario.grasp.maximum_center_offset_m;
        self.mechanics
            .body_mut(BodyId(PEG_OBJECT_ID))
            .ok_or(PlantFailure::MechanicsFailure)?
            .pose
            .translation += jaw_axis * disturbance_m;
        self.grasp_capture_fault_applied = true;
        Ok(())
    }

    fn inject_insertion_jam_fault_if_needed(
        &mut self,
        motion_class: MotionClass,
        effective_target_world_m: Vec3,
    ) -> Result<(), PlantFailure> {
        if self.fault != M1eFault::InsertionJam
            || self.insertion_jam_fault_applied
            || motion_class != MotionClass::Insertion
            || !self.peg_physically_held()
        {
            return Ok(());
        }
        let held_local_pose = self
            .active_arm()
            .held_body_local_pose
            .ok_or(PlantFailure::PegNotHeld)?;
        let predicted_tool_pose = solved_tool_pose(&self.mechanics, effective_target_world_m)
            .map_err(|_| PlantFailure::ImpossibleGeometry)?;
        let predicted_peg_pose = predicted_tool_pose * held_local_pose;
        let predicted_peg_tip_world = predicted_peg_pose.transform_point(
            Vec3::Z
                * (self.scenario.coupon.peg_half_segment_m
                    + 0.5 * self.scenario.coupon.peg_diameter_m),
        );
        let tip_in_socket = self
            .socket_pose
            .inverse_transform_point(predicted_peg_tip_world);
        let axial_remaining_m = 0.5 * self.scenario.coupon.socket_depth_m - tip_in_socket.z;
        if axial_remaining_m > self.scenario.contact.lead_in_start_m {
            return Ok(());
        }
        // One-shot attachment disturbance after the stopped observation and
        // before the first insertion increment. It moves the physical peg,
        // not a reported estimate or contact label; the next optical burst and
        // raw load-path packet are the only controller-visible consequences.
        let target_lateral_m = self
            .scenario
            .insertion_jam_lateral_target_m()
            .ok_or(PlantFailure::ImpossibleGeometry)?;
        // Set, rather than add, the socket-frame lateral error. Otherwise the
        // residual left by a particular noise seed can consume the force-limit
        // margin and turn the intended geometric jam into an overload trip.
        let desired_tip_in_socket = Vec3::new(target_lateral_m, 0.0, tip_in_socket.z);
        let correction_in_socket = desired_tip_in_socket - tip_in_socket;
        let correction_world = self.socket_pose.transform_vector(correction_in_socket);
        let local_disturbance = predicted_tool_pose.inverse_transform_vector(correction_world);
        let arm = self
            .mechanics
            .serial_arm_mut(ACTIVE_ARM_ID)
            .ok_or(PlantFailure::MechanicsFailure)?;
        let local_pose = arm
            .held_body_local_pose
            .as_mut()
            .ok_or(PlantFailure::PegNotHeld)?;
        local_pose.translation += local_disturbance;
        self.insertion_jam_fault_applied = true;
        Ok(())
    }

    fn private_insertion_contact(&self) -> (bool, f64) {
        if !self.peg_physically_held() {
            return (false, 0.0);
        }
        let peg = self.peg_body();
        let peg_tip_world = peg.pose.transform_point(
            Vec3::Z
                * (self.scenario.coupon.peg_half_segment_m
                    + 0.5 * self.scenario.coupon.peg_diameter_m),
        );
        let tip_in_socket = self.socket_pose.inverse_transform_point(peg_tip_world);
        let tip_to_seat = Vec3::Z * (0.5 * self.scenario.coupon.socket_depth_m) - tip_in_socket;
        let lateral_m = tip_to_seat.x.hypot(tip_to_seat.y);
        let axial_remaining_m = tip_to_seat.z;
        // A signed axial projection alone does not establish contact with the
        // coupon.  In particular, the pickup station can project "past" the
        // seat on the socket axis while remaining millimetres away laterally.
        // Bound the reduced contact sensor to the physical radial reach of the
        // socket wall plus the peg radius before converting axial error into a
        // load proxy.
        let peg_radius_m = 0.5 * self.scenario.coupon.peg_diameter_m;
        let socket_outer_half_width_m = peg_radius_m
            + self.scenario.coupon.socket_radial_clearance_m
            + self.scenario.coupon.socket_wall_thickness_m;
        if lateral_m > socket_outer_half_width_m + peg_radius_m {
            return (false, 0.0);
        }
        let peg_axis = peg.pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
        let socket_axis = self
            .socket_pose
            .transform_vector(Vec3::Z)
            .normalized_or(Vec3::Z);
        let axis_error_rad = peg_axis
            .cross(socket_axis)
            .length()
            .atan2(peg_axis.dot(socket_axis).clamp(-1.0, 1.0));
        if axial_remaining_m > self.scenario.contact.lead_in_start_m {
            return (false, 0.0);
        }
        let lateral_deflection_m = (lateral_m - self.scenario.contact.seat_lateral_tolerance_m)
            .max(0.0)
            + axis_error_rad.sin().abs() * self.scenario.coupon.peg_half_segment_m;
        // Reduced compliant lead-in: load rises continuously from zero at the
        // declared lead-in boundary to one seat-tolerance of pad/flexure
        // deflection at the nominal seat. This is a force proxy, not a
        // calibrated friction or insertion-force prediction.
        let axial_deflection_m = (self.scenario.contact.lead_in_start_m - axial_remaining_m)
            .max(0.0)
            * (self.scenario.contact.seat_axial_tolerance_m
                / self.scenario.contact.lead_in_start_m);
        let force_proxy_n = lateral_deflection_m
            * self.scenario.contact.lateral_stiffness_proxy_n_per_m
            + axial_deflection_m * self.scenario.contact.axial_stiffness_proxy_n_per_m;
        if force_proxy_n > self.scenario.contact.maximum_force_proxy_n {
            return (true, force_proxy_n);
        }
        (force_proxy_n > 0.0, force_proxy_n)
    }

    /// Hidden physical enforcement for non-peg tool/socket overlap. Only the
    /// generic collision failure and aggregate evaluation penetration cross
    /// the plant boundary; live socket, jaw, or side-target proximity never
    /// enters controller state.
    fn private_tool_socket_max_penetration_m(&self) -> f64 {
        let (jaw_penetration_m, side_target_penetration_m) =
            self.private_tool_socket_component_penetrations_m();
        jaw_penetration_m.max(side_target_penetration_m)
    }

    fn private_tool_socket_component_penetrations_m(&self) -> (f64, f64) {
        let arm = self.active_arm();
        let mut maximum_jaw_penetration_m = 0.0_f64;
        for (jaw_index, (pose, shape)) in arm
            .gripper
            .jaw_poses(self.physical_tool_pose(), arm.gripper_config)
            .into_iter()
            .zip(pipe_sim_core::GripperState::jaw_shapes(arm.gripper_config))
            .enumerate()
        {
            let jaw = RigidBody::new(
                BodyId(10_090 + jaw_index as u32),
                shape,
                pose,
                MotionType::Kinematic,
            );
            for socket_id in SOCKET_BODY_IDS {
                let Some(socket_wall) = self.mechanics.body(socket_id) else {
                    continue;
                };
                maximum_jaw_penetration_m =
                    maximum_jaw_penetration_m.max(private_penetration_depth_m(&jaw, socket_wall));
            }
        }

        // The physical coded side targets are represented optically as capped
        // cylinders. The oriented boxes below contain those cylinders exactly
        // and provide conservative collision proxies supported by the core
        // narrow phase. They share the same hashed calibration values as the
        // optical scene and controller clearance envelope.
        let mut maximum_side_target_penetration_m = 0.0_f64;
        let target_offset = array_vec3(
            self.calibrated_tool_geometry
                .side_target_center_offset_tool_m,
        );
        for (target_index, sign) in [1.0, -1.0].into_iter().enumerate() {
            let target = RigidBody::new(
                BodyId(10_092 + 2 * target_index as u32),
                Shape::Box {
                    half_extents_m: Vec3::new(
                        self.calibrated_tool_geometry.side_target_radius_m,
                        self.calibrated_tool_geometry.side_target_radius_m,
                        self.calibrated_tool_geometry
                            .side_target_axial_half_extent_m,
                    ),
                },
                self.physical_tool_pose() * Pose::from_translation(target_offset * sign),
                MotionType::Kinematic,
            );
            for socket_id in SOCKET_BODY_IDS {
                let Some(socket_wall) = self.mechanics.body(socket_id) else {
                    continue;
                };
                maximum_side_target_penetration_m = maximum_side_target_penetration_m
                    .max(private_penetration_depth_m(&target, socket_wall));
            }
        }
        (maximum_jaw_penetration_m, maximum_side_target_penetration_m)
    }

    /// Independent physical recheck for the recessed central palm. The proxy
    /// is a box containing the tapered cylindrical boss immediately behind the
    /// palm plane; the open jaw channel ahead of that plane is intentionally
    /// empty except for the jaws themselves.
    fn private_palm_peg_penetration_m(&self) -> f64 {
        let palm_radius_m = self.active_arm().gripper_config.jaw_half_extents_m.y;
        let boss_length_m = 6.0 * palm_radius_m;
        let palm_center_z_m =
            self.calibrated_tool_geometry.palm_forward_plane_tool_z_m - 0.5 * boss_length_m;
        let palm = RigidBody::new(
            BodyId(10_093),
            Shape::Box {
                half_extents_m: Vec3::new(palm_radius_m, palm_radius_m, 0.5 * boss_length_m),
            },
            self.physical_tool_pose() * Pose::from_translation(Vec3::Z * palm_center_z_m),
            MotionType::Kinematic,
        );
        private_penetration_depth_m(&palm, self.peg_body())
    }
}

/// Pair the two symmetric physical target returns into one logical tool-axis
/// station. Requiring both sides prevents a one-sided dropout from silently
/// reintroducing an unknown roll-dependent mount offset.
fn midpoint_paired_tool_observations(
    first: FeaturePointObservation,
    second: FeaturePointObservation,
) -> FeaturePointObservation {
    let (first, second) = match (first, second) {
        (FeaturePointObservation::Observed(first), FeaturePointObservation::Observed(second)) => {
            (first, second)
        }
        (missing @ FeaturePointObservation::Missing { .. }, _) => return missing,
        (_, missing @ FeaturePointObservation::Missing { .. }) => return missing,
    };
    if first.object_id != second.object_id
        || first.feature_id != second.feature_id
        || first.head != second.head
        || first.calibrated_ray_count == 0
        || second.calibrated_ray_count == 0
    {
        return FeaturePointObservation::Missing {
            object_id: first.object_id,
            feature_id: first.feature_id,
            head: first.head,
            reason: MissingFeaturePoint::InvalidFeatureGeometry,
        };
    }
    let Some(first_variance_bound_m2) =
        conservative_covariance_variance_bound(first.covariance.matrix_m2.m)
    else {
        return FeaturePointObservation::Missing {
            object_id: first.object_id,
            feature_id: first.feature_id,
            head: first.head,
            reason: MissingFeaturePoint::InvalidConfiguration,
        };
    };
    let Some(second_variance_bound_m2) =
        conservative_covariance_variance_bound(second.covariance.matrix_m2.m)
    else {
        return FeaturePointObservation::Missing {
            object_id: first.object_id,
            feature_id: first.feature_id,
            head: first.head,
            reason: MissingFeaturePoint::InvalidConfiguration,
        };
    };
    // No cross-target covariance is carried by the optics DTO. The triangle
    // inequality therefore gives this arbitrary-correlation-safe midpoint
    // bound; equal marginal samples do not claim a fictitious sqrt(2) gain.
    let midpoint_sigma_bound_m =
        0.5 * (first_variance_bound_m2.sqrt() + second_variance_bound_m2.sqrt());
    let midpoint = 0.5 * (first.measured_point + second.measured_point);
    FeaturePointObservation::Observed(FeaturePointSample {
        object_id: first.object_id,
        feature_id: first.feature_id,
        head: first.head,
        calibrated_ray_count: first
            .calibrated_ray_count
            .saturating_add(second.calibrated_ray_count),
        observed_camera_pixel: (first.observed_camera_pixel + second.observed_camera_pixel) * 0.5,
        observed_projector_pixel: (first.observed_projector_pixel
            + second.observed_projector_pixel)
            * 0.5,
        measured_point: midpoint,
        covariance: Covariance3::isotropic(midpoint_sigma_bound_m),
        quality: QualityMetrics {
            signal_to_noise: first
                .quality
                .signal_to_noise
                .min(second.quality.signal_to_noise),
            triangulation_angle_rad: first
                .quality
                .triangulation_angle_rad
                .min(second.quality.triangulation_angle_rad),
            ray_separation_m: first
                .quality
                .ray_separation_m
                .max(second.quality.ray_separation_m),
            reprojection_error_px: first
                .quality
                .reprojection_error_px
                .max(second.quality.reprojection_error_px),
            estimated_axial_sigma_m: midpoint_sigma_bound_m,
            condition_number: first
                .quality
                .condition_number
                .max(second.quality.condition_number),
            confidence: first.quality.confidence.min(second.quality.confidence),
        },
        signal_photoelectrons: first
            .signal_photoelectrons
            .min(second.signal_photoelectrons),
    })
}

fn ring_arc_probes(
    center: FeaturePoint,
    axis: OpticalVec3,
    radius_m: f64,
    head_midpoint: OpticalVec3,
) -> Vec<FeaturePoint> {
    let to_head = head_midpoint - center.point_world;
    let head_radial = to_head - axis * to_head.dot(axis);
    let radial = head_radial.normalized().unwrap_or_else(|| {
        let reference = if axis.z.abs() < 0.9 {
            OpticalVec3::Z
        } else {
            OpticalVec3::Y
        };
        axis.cross(reference).normalized().unwrap_or(OpticalVec3::X)
    });
    let tangent = axis.cross(radial).normalized().unwrap_or(OpticalVec3::Y);
    RING_ARC_PROBE_ANGLES_RAD
        .into_iter()
        .enumerate()
        .map(|(probe_index, angle_rad)| {
            let normal = radial * angle_rad.cos() + tangent * angle_rad.sin();
            FeaturePoint {
                object_id: center.object_id,
                feature_id: center
                    .feature_id
                    .saturating_mul(100)
                    .saturating_add(probe_index as u32 + 1),
                point_world: center.point_world + normal * radius_m,
                normal_world: normal,
                material: center.material,
                // Surface probes must be occluded by the far side of their
                // own carrier. The ray-origin epsilon already admits the
                // front surface; ignoring the carrier would make back-facing
                // arcs see through solid peg/socket/target material.
                self_occlusion_tag: None,
            }
        })
        .collect()
}

fn usable_ring_arc_geometry(
    observations: &[FeaturePointObservation],
) -> (bool, Option<MissingFeaturePoint>) {
    // The M1e rig has exactly one camera/projector head.  A fitted arc cannot
    // be assembled from disjoint isolated points, so track contiguous probe
    // runs in deterministic angle order and require one run to bracket the
    // viewing meridian with the declared span.
    if observations.len() != RING_ARC_PROBE_ANGLES_RAD.len() {
        return (false, None);
    }
    let mut run_count = 0_usize;
    let mut run_start_angle = 0.0;
    let mut usable = false;
    let mut occlusion_reason = None;
    for (angle_rad, observation) in RING_ARC_PROBE_ANGLES_RAD.iter().copied().zip(observations) {
        match observation {
            FeaturePointObservation::Observed(_) => {
                if run_count == 0 {
                    run_start_angle = angle_rad;
                }
                run_count += 1;
                let half_required_arc = 0.5 * MINIMUM_USABLE_RING_ARC_RAD;
                usable |= run_count >= MINIMUM_USABLE_RING_PROBES
                    && angle_rad - run_start_angle >= MINIMUM_USABLE_RING_ARC_RAD
                    && run_start_angle <= -half_required_arc
                    && angle_rad >= half_required_arc;
            }
            FeaturePointObservation::Missing { reason, .. } => {
                run_count = 0;
                if *reason == MissingFeaturePoint::CameraOccluded {
                    occlusion_reason = Some(MissingFeaturePoint::CameraOccluded);
                } else if *reason == MissingFeaturePoint::ProjectorOccluded
                    && occlusion_reason.is_none()
                {
                    occlusion_reason = Some(MissingFeaturePoint::ProjectorOccluded);
                }
            }
        }
    }
    (usable, occlusion_reason)
}

/// Radius of a circular latitude on the capsule surface at an axial station.
/// The cylindrical span has constant radius; stations on either hemispherical
/// end use the exact spherical cross-section rather than floating probes in
/// free space outside the 0.400 mm peg.
fn capsule_latitude_radius(
    radius_m: f64,
    half_segment_m: f64,
    axial_coordinate_m: f64,
) -> Option<f64> {
    if !radius_m.is_finite()
        || !half_segment_m.is_finite()
        || !axial_coordinate_m.is_finite()
        || radius_m <= 0.0
        || half_segment_m < 0.0
    {
        return None;
    }
    let cap_axial_m = (axial_coordinate_m.abs() - half_segment_m).max(0.0);
    if cap_axial_m >= radius_m {
        return None;
    }
    Some((radius_m * radius_m - cap_axial_m * cap_axial_m).sqrt())
}

fn axial_feature_model(object_id: u32, half_span_m: f64) -> Vec<KnownAxialFeature> {
    // The peg rings sit forward of the short M1e jaws but remain on the
    // approach side of the through-socket entrance at physical seating. Their
    // 0.400 mm extreme span has a 0.1883 mm mean-centred RMS lever arm, above
    // the configured 0.180 mm gate. A known off-center feature model lets the
    // estimator recover the capsule center without treating the visible-ring
    // centroid as the object origin.
    // Tool features are four coded stations on the calibrated side target
    // spanning +/-0.25 mm about the flange. Their fitted centers are mapped
    // through the known rigid mount offset to the tool axis before exposure to
    // the estimator. Socket rings retain the same symmetric layout.
    let stations = match object_id {
        PEG_OBJECT_ID => PEG_FEATURE_STATIONS_M,
        TOOL_OBJECT_ID => [
            -half_span_m,
            -0.9 * half_span_m,
            0.9 * half_span_m,
            half_span_m,
        ],
        _ => [
            -half_span_m,
            -0.9 * half_span_m,
            0.9 * half_span_m,
            half_span_m,
        ],
    };
    FEATURE_IDS
        .into_iter()
        .zip(stations)
        .map(|(feature_id, axial_coordinate_m)| KnownAxialFeature {
            feature_id,
            axial_coordinate_m,
        })
        .collect()
}

fn set_initial_tool_pose(mechanics: &mut Simulation, target: Vec3) -> Result<(), SimError> {
    let arm = mechanics
        .serial_arm_mut(ACTIVE_ARM_ID)
        .ok_or_else(|| SimError::Mechanics("M1e active arm 1 missing".to_owned()))?;
    let solution = arm
        .arm
        .solve_tool_position(target, arm.motion.positions())
        .map_err(|error| SimError::Mechanics(format!("M1e initial tool IK: {error:?}")))?;
    arm.arm
        .set_positions(solution.positions)
        .map_err(|error| SimError::Mechanics(format!("M1e initial tool FK: {error:?}")))?;
    arm.kinematics = arm.arm.forward_kinematics();
    arm.motion = ManipulatorMotionState::from_positions(solution.positions);
    Ok(())
}

fn solved_tool_pose(mechanics: &Simulation, target: Vec3) -> Result<Pose, SimError> {
    let instance = mechanics
        .serial_arm(ACTIVE_ARM_ID)
        .ok_or_else(|| SimError::Mechanics("M1e active arm 1 missing".to_owned()))?;
    let solution = instance
        .arm
        .solve_tool_position(target, instance.motion.positions())
        .map_err(|error| SimError::Mechanics(format!("M1e target IK: {error:?}")))?;
    let mut candidate = instance.arm.clone();
    candidate
        .set_positions(solution.positions)
        .map_err(|error| SimError::Mechanics(format!("M1e target FK: {error:?}")))?;
    Ok(candidate.forward_kinematics().tool_pose)
}

fn add_socket_coupon(
    mechanics: &mut Simulation,
    scenario: &ObservedManipulationScenario,
    socket_pose: Pose,
) -> Result<(), SimError> {
    let inner_half_width_m =
        0.5 * scenario.coupon.peg_diameter_m + scenario.coupon.socket_radial_clearance_m;
    let wall_thickness_m = scenario.coupon.socket_wall_thickness_m;
    let outer_half_width_m = inner_half_width_m + wall_thickness_m;
    let wall_center_m = inner_half_width_m + 0.5 * wall_thickness_m;
    let definitions = [
        (
            SOCKET_BODY_IDS[0],
            Vec3::new(-wall_center_m, 0.0, 0.0),
            Vec3::new(
                0.5 * wall_thickness_m,
                outer_half_width_m,
                0.5 * scenario.coupon.socket_depth_m,
            ),
        ),
        (
            SOCKET_BODY_IDS[1],
            Vec3::new(wall_center_m, 0.0, 0.0),
            Vec3::new(
                0.5 * wall_thickness_m,
                outer_half_width_m,
                0.5 * scenario.coupon.socket_depth_m,
            ),
        ),
        (
            SOCKET_BODY_IDS[2],
            Vec3::new(0.0, -wall_center_m, 0.0),
            Vec3::new(
                inner_half_width_m,
                0.5 * wall_thickness_m,
                0.5 * scenario.coupon.socket_depth_m,
            ),
        ),
        (
            SOCKET_BODY_IDS[3],
            Vec3::new(0.0, wall_center_m, 0.0),
            Vec3::new(
                inner_half_width_m,
                0.5 * wall_thickness_m,
                0.5 * scenario.coupon.socket_depth_m,
            ),
        ),
    ];
    for (body_id, local_center, half_extents_m) in definitions {
        let mut wall = RigidBody::new(
            body_id,
            Shape::Box { half_extents_m },
            socket_pose * Pose::from_translation(local_center),
            MotionType::Static,
        );
        wall.collision_filter = CollisionFilter {
            group: SOCKET_COLLISION_GROUP,
            mask: 0,
        };
        mechanics
            .add_body(wall)
            .map_err(|error| SimError::Mechanics(format!("M1e socket wall: {error:?}")))?;
    }
    Ok(())
}

fn add_calibrated_planning_obstacles(
    mechanics: &mut Simulation,
    scenario: &ObservedManipulationScenario,
) -> Result<(), SimError> {
    let reserved_ids = [
        BodyId(PEG_OBJECT_ID),
        CARRIED_COLLISION_BODY_ID,
        SOCKET_BODY_IDS[0],
        SOCKET_BODY_IDS[1],
        SOCKET_BODY_IDS[2],
        SOCKET_BODY_IDS[3],
    ];
    let mut ordered = scenario.safety.planning_obstacles.clone();
    ordered.sort_by_key(|obstacle| obstacle.id);
    for obstacle in ordered {
        let body_id = BodyId(obstacle.id);
        if reserved_ids.contains(&body_id) {
            return Err(SimError::Mechanics(format!(
                "M1e planning obstacle ID {} collides with a reserved plant body",
                obstacle.id
            )));
        }
        let mut body = RigidBody::new(
            body_id,
            Shape::Sphere {
                radius_m: obstacle.conservative_radius_m,
            },
            Pose::from_translation(array_vec3(obstacle.center_world_m)),
            MotionType::Static,
        );
        body.collision_filter = CollisionFilter {
            group: FAULT_OBSTACLE_COLLISION_GROUP,
            mask: u32::MAX,
        };
        mechanics.add_body(body).map_err(|error| {
            SimError::Mechanics(format!("M1e calibrated planning obstacle: {error:?}"))
        })?;
    }
    Ok(())
}

fn add_carried_collision_obstacle(
    mechanics: &mut Simulation,
    scenario: &ObservedManipulationScenario,
) -> Result<(), SimError> {
    let pick = array_vec3(scenario.coupon.pick_peg_center_nominal_world_m);
    let socket = array_vec3(scenario.coupon.socket_center_nominal_world_m);
    let center = pick.lerp(socket, 0.5);
    let mut obstacle = RigidBody::new(
        CARRIED_COLLISION_BODY_ID,
        Shape::Sphere {
            radius_m: scenario.safety.carried_peg_envelope_radius_m + 0.10e-3,
        },
        Pose::from_translation(center),
        MotionType::Static,
    );
    obstacle.collision_filter = CollisionFilter {
        group: FAULT_OBSTACLE_COLLISION_GROUP,
        mask: 0,
    };
    mechanics
        .add_body(obstacle)
        .map_err(|error| SimError::Mechanics(format!("M1e fault obstacle: {error:?}")))
}

fn is_intended_socket_pair(a: BodyId, b: BodyId) -> bool {
    (a == BodyId(PEG_OBJECT_ID) && SOCKET_BODY_IDS.contains(&b))
        || (b == BodyId(PEG_OBJECT_ID) && SOCKET_BODY_IDS.contains(&a))
}

fn distinct_frame_ticks(start: u64, end: u64, count: u64) -> Vec<u64> {
    if count <= 1 || start == end {
        return vec![start];
    }
    let span = end - start;
    let count = count.min(span + 1);
    (0..count)
        .map(|index| start + (index * span + (count - 1) / 2) / (count - 1))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

fn ticks_ceil(duration_s: f64, dt_s: f64) -> u64 {
    (duration_s / dt_s - 1.0e-12).ceil().max(0.0) as u64
}

fn angular_chain_acceleration_bound(speeds_rad_s: &[f64], accelerations_rad_s2: &[f64]) -> f64 {
    let speed_sum = speeds_rad_s.iter().copied().sum::<f64>();
    let speed_square_sum = speeds_rad_s.iter().map(|speed| speed * speed).sum::<f64>();
    let rotating_axis_cross_terms = 0.5 * (speed_sum * speed_sum - speed_square_sum).max(0.0);
    accelerations_rad_s2.iter().copied().sum::<f64>()
        + rotating_axis_cross_terms
        + speed_sum * speed_sum
}

fn open_gripper_corner_radius_m(config: pipe_sim_core::GripperConfig) -> f64 {
    let farthest_x_m = 0.5 * config.max_opening_m + 2.0 * config.jaw_half_extents_m.x;
    farthest_x_m
        .hypot(config.jaw_half_extents_m.y)
        .hypot(config.jaw_half_extents_m.z)
}

fn closed_jaw_transverse_radius_m(opening_m: f64, jaw_half_extents_m: Vec3) -> f64 {
    let farthest_x_m = 0.5 * opening_m + 2.0 * jaw_half_extents_m.x;
    farthest_x_m.hypot(jaw_half_extents_m.y)
}

fn side_target_forward_extent_m(tool: machine_config::CalibratedToolGeometry) -> f64 {
    tool.side_target_center_offset_tool_m[2] + tool.side_target_axial_half_extent_m
}

/// Roll-invariant transverse radius of the conservative rectangular collision
/// proxy around the cylindrical side target.  Taking the farthest of all four
/// local XY corners keeps the observed-axis guard at least as conservative as
/// the independently queried private plant proxy.
fn side_target_transverse_radius_m(tool: machine_config::CalibratedToolGeometry) -> f64 {
    let [center_x_m, center_y_m, _] = tool.side_target_center_offset_tool_m;
    [-1.0, 1.0]
        .into_iter()
        .flat_map(|x_sign| {
            [-1.0, 1.0].into_iter().map(move |y_sign| {
                (center_x_m + x_sign * tool.side_target_radius_m)
                    .hypot(center_y_m + y_sign * tool.side_target_radius_m)
            })
        })
        .fold(0.0_f64, f64::max)
}

fn side_target_bounding_radius_m(tool: machine_config::CalibratedToolGeometry) -> f64 {
    let center_z_m = tool.side_target_center_offset_tool_m[2];
    let maximum_axial_extent_m = (center_z_m - tool.side_target_axial_half_extent_m)
        .abs()
        .max((center_z_m + tool.side_target_axial_half_extent_m).abs());
    side_target_transverse_radius_m(tool).hypot(maximum_axial_extent_m)
}

/// Axial gap from the forward-most non-peg tool proxy to the socket entrance
/// when the peg tip is at the far seat. Positive is clearance; negative is
/// penetration.
fn nominal_seated_tool_socket_clearance_m(
    socket_depth_m: f64,
    peg_tip_from_center_m: f64,
    tool_to_peg_axial_offset_m: f64,
    tool_forward_extent_m: f64,
) -> Option<f64> {
    if ![
        socket_depth_m,
        peg_tip_from_center_m,
        tool_to_peg_axial_offset_m,
        tool_forward_extent_m,
    ]
    .iter()
    .all(|value| value.is_finite() && *value >= 0.0)
    {
        return None;
    }
    Some(
        peg_tip_from_center_m + tool_to_peg_axial_offset_m - tool_forward_extent_m - socket_depth_m,
    )
}

fn conservative_covariance_variance_bound(matrix_m2: [[f64; 3]; 3]) -> Option<f64> {
    if !matrix_m2.iter().flatten().all(|value| value.is_finite()) {
        return None;
    }
    let maximum_absolute_row_sum_m2 = matrix_m2
        .iter()
        .map(|row| row.iter().map(|value| value.abs()).sum::<f64>())
        .fold(0.0_f64, f64::max);
    Some(maximum_absolute_row_sum_m2.max(1.0e-24))
}

fn sign_i8(value: f64) -> i8 {
    if value > 1.0e-15 {
        1
    } else if value < -1.0e-15 {
        -1
    } else {
        0
    }
}

fn point_segment_distance_m(point: Vec3, start: Vec3, end: Vec3) -> f64 {
    let segment = end - start;
    let length_squared = segment.dot(segment);
    if length_squared <= f64::EPSILON {
        return (point - start).length();
    }
    let fraction = ((point - start).dot(segment) / length_squared).clamp(0.0, 1.0);
    (point - start.lerp(end, fraction)).length()
}

fn two_axis_tilt(tilt_rad: [f64; 2]) -> Quat {
    (Quat::from_axis_angle(Vec3::X, tilt_rad[0]) * Quat::from_axis_angle(Vec3::Y, tilt_rad[1]))
        .normalized()
}

fn optical_look_at_with_long_axis(
    origin: OpticalVec3,
    target: OpticalVec3,
    desired_long_axis_world: OpticalVec3,
) -> RigidTransform {
    let forward = (target - origin).normalized().unwrap_or(OpticalVec3::Z);
    // Roll the rectangular M1d macro head so its 2.5 mm/1280 px image axis
    // follows the calibrated insertion-axis projection in the image plane.
    let projected_long = desired_long_axis_world - forward * desired_long_axis_world.dot(forward);
    let right = projected_long.normalized().unwrap_or_else(|| {
        let fallback = if forward.z.abs() > 0.92 {
            OpticalVec3::Y
        } else {
            OpticalVec3::Z
        };
        forward
            .cross(fallback)
            .normalized()
            .unwrap_or(OpticalVec3::X)
    });
    let down = forward.cross(right).normalized().unwrap_or(OpticalVec3::Y);
    RigidTransform::new(
        OpticalMat3::new([
            [right.x, down.x, forward.x],
            [right.y, down.y, forward.y],
            [right.z, down.z, forward.z],
        ]),
        origin,
    )
}

fn optical_material(signal_scale: f64, retroreflective_gain: f64) -> OpticalMaterial {
    OpticalMaterial {
        diffuse_reflectance: (0.72 * signal_scale).clamp(0.0, 1.0),
        retroreflective_gain,
        roughness: 0.35,
    }
}

fn push_optical_capsule(
    scene: &mut OpticalScene,
    pose: Pose,
    radius_m: f64,
    half_segment_m: f64,
    material: OpticalMaterial,
    tag: u32,
) {
    let start = optical_vec(pose.transform_point(Vec3::new(0.0, 0.0, -half_segment_m)));
    let end = optical_vec(pose.transform_point(Vec3::new(0.0, 0.0, half_segment_m)));
    scene.push(OpticalPrimitive::new(
        OpticalGeometry::Cylinder(pipe_optics::Cylinder {
            start,
            end,
            radius_m,
            capped: true,
        }),
        material,
        tag,
    ));
    for center in [start, end] {
        scene.push(OpticalPrimitive::new(
            OpticalGeometry::Sphere(OpticalSphere { center, radius_m }),
            material,
            tag,
        ));
    }
}

struct TerminalToolOpticalInput {
    terminal_link_pose: Pose,
    physical_tool_pose: Pose,
    collision_envelope_radius_m: f64,
    half_segment_m: f64,
    fiducial_boss_radius_m: f64,
    tool_geometry: machine_config::CalibratedToolGeometry,
    link_material: OpticalMaterial,
    fiducial_material: OpticalMaterial,
}

fn push_optical_terminal_tool(scene: &mut OpticalScene, input: TerminalToolOpticalInput) {
    let TerminalToolOpticalInput {
        terminal_link_pose,
        physical_tool_pose,
        collision_envelope_radius_m,
        half_segment_m,
        fiducial_boss_radius_m,
        tool_geometry,
        link_material,
        fiducial_material,
    } = input;
    // A tapered final 1.50 mm ends at the calibrated recessed palm plane,
    // leaving the central grasp channel open ahead of that plane. The four
    // actual coded stations sit on the narrow side target below, not on this
    // central boss. Mechanics still sweeps the full conservative collision
    // radius over the complete terminal-link segment away from task contact.
    let boss_length_m = (6.0 * fiducial_boss_radius_m).min(2.0 * half_segment_m);
    let start =
        optical_vec(terminal_link_pose.transform_point(Vec3::new(0.0, 0.0, -half_segment_m)));
    let boss_start = optical_vec(
        physical_tool_pose
            .transform_point(Vec3::Z * (tool_geometry.palm_forward_plane_tool_z_m - boss_length_m)),
    );
    let palm_plane = optical_vec(
        physical_tool_pose.transform_point(Vec3::Z * tool_geometry.palm_forward_plane_tool_z_m),
    );
    scene.push(OpticalPrimitive::new(
        OpticalGeometry::Cylinder(pipe_optics::Cylinder {
            start,
            end: boss_start,
            radius_m: collision_envelope_radius_m,
            capped: true,
        }),
        link_material,
        OPTICAL_TAG_TOOL_LINK,
    ));
    scene.push(OpticalPrimitive::new(
        OpticalGeometry::Cylinder(pipe_optics::Cylinder {
            start: boss_start,
            end: palm_plane,
            radius_m: fiducial_boss_radius_m,
            capped: true,
        }),
        link_material,
        OPTICAL_TAG_TOOL_LINK,
    ));
    let target_radius_m = tool_geometry.side_target_radius_m;
    let target_half_length_m = tool_geometry.side_target_axial_half_extent_m;
    let target_center_tool_m = array_vec3(tool_geometry.side_target_center_offset_tool_m);
    for (sign, tag) in [
        (1.0, OPTICAL_TAG_TOOL_FIDUCIAL_POSITIVE),
        (-1.0, OPTICAL_TAG_TOOL_FIDUCIAL_NEGATIVE),
    ] {
        let center = target_center_tool_m * sign;
        let target_start = optical_vec(
            physical_tool_pose.transform_point(center - Vec3::Z * target_half_length_m),
        );
        let target_end = optical_vec(
            physical_tool_pose.transform_point(center + Vec3::Z * target_half_length_m),
        );
        scene.push(OpticalPrimitive::new(
            OpticalGeometry::Cylinder(pipe_optics::Cylinder {
                start: target_start,
                end: target_end,
                radius_m: target_radius_m,
                capped: true,
            }),
            fiducial_material,
            tag,
        ));
    }
}

fn push_optical_box(
    scene: &mut OpticalScene,
    pose: Pose,
    half: Vec3,
    material: OpticalMaterial,
    tag: u32,
) {
    let local = [
        Vec3::new(-half.x, -half.y, -half.z),
        Vec3::new(half.x, -half.y, -half.z),
        Vec3::new(half.x, half.y, -half.z),
        Vec3::new(-half.x, half.y, -half.z),
        Vec3::new(-half.x, -half.y, half.z),
        Vec3::new(half.x, -half.y, half.z),
        Vec3::new(half.x, half.y, half.z),
        Vec3::new(-half.x, half.y, half.z),
    ];
    let vertices = local.map(|point| optical_vec(pose.transform_point(point)));
    let triangles = [
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    let _ = scene.extend_triangle_mesh(&vertices, &triangles, material, tag, false);
}

fn missing_feature_reason(reason: MissingFeaturePoint) -> &'static str {
    match reason {
        MissingFeaturePoint::InvalidFeatureGeometry => "invalid_feature_geometry",
        MissingFeaturePoint::InvalidCalibration => "invalid_calibration",
        MissingFeaturePoint::InvalidConfiguration => "invalid_configuration",
        MissingFeaturePoint::CameraOutOfView => "camera_out_of_view",
        MissingFeaturePoint::ProjectorOutOfView => "projector_out_of_view",
        MissingFeaturePoint::CameraOccluded => "camera_occluded",
        MissingFeaturePoint::ProjectorOccluded => "projector_occluded",
        MissingFeaturePoint::LowSignal => "low_signal",
        MissingFeaturePoint::StochasticDropout => "stochastic_dropout",
        MissingFeaturePoint::DegenerateGeometry => "degenerate_geometry",
        MissingFeaturePoint::BehindSensor => "behind_sensor",
        MissingFeaturePoint::ExcessiveRayResidual => "excessive_ray_residual",
        MissingFeaturePoint::InsufficientQuality => "insufficient_quality",
    }
}

fn array_vec3(value: [f64; 3]) -> Vec3 {
    Vec3::new(value[0], value[1], value[2])
}

fn vec3(value: Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

fn optical_vec(value: Vec3) -> OpticalVec3 {
    OpticalVec3::new(value.x, value.y, value.z)
}

fn array_optical_vec3(value: [f64; 3]) -> OpticalVec3 {
    OpticalVec3::new(value[0], value[1], value[2])
}

/// Exact plant-side geometry query that deliberately ignores the simulation
/// collision masks used to keep task contact out of the authoritative core
/// preflight. The result never crosses the optics/contact DTO boundary.
fn private_penetration_depth_m(a: &RigidBody, b: &RigidBody) -> f64 {
    let mut a = a.clone();
    let mut b = b.clone();
    a.collision_filter = CollisionFilter::ALL;
    b.collision_filter = CollisionFilter::ALL;
    query_pair(&a, &b)
        .map(|proximity| proximity.penetration_depth_m())
        .unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::super::controller::{preflight_swept_envelope, SweptEnvelope};
    use super::*;

    fn settled_plant(fault: M1eFault) -> ObservedPlant {
        let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        let mut plant = ObservedPlant::new(&scenario, fault).unwrap();
        plant
            .advance_ticks(plant.motion_capabilities().settling_ticks)
            .unwrap();
        plant
    }

    fn observed_tail_grasp_target(plant: &ObservedPlant) -> Vec3 {
        let peg = plant.peg_body();
        let peg_axis = peg.pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
        peg.pose.translation - peg_axis * plant.scenario.grasp.tool_to_peg_axial_offset_m
    }

    fn calibrated_pick_capture_target(plant: &ObservedPlant) -> Vec3 {
        let peg = array_vec3(plant.scenario.coupon.pick_peg_center_nominal_world_m);
        let axis = array_vec3(plant.calibrated_socket_axis_world());
        peg - axis * plant.scenario.motion.pick_capture_axial_standoff_m
    }

    fn capture_tool_peg_penetrations_m(plant: &ObservedPlant) -> (f64, f64, f64) {
        let arm = plant.active_arm();
        let tool_pose = plant.physical_tool_pose();
        let peg = plant.peg_body();

        let maximum_jaw_penetration_m = arm
            .gripper
            .jaw_poses(tool_pose, arm.gripper_config)
            .into_iter()
            .zip(pipe_sim_core::GripperState::jaw_shapes(arm.gripper_config))
            .map(|(pose, shape)| {
                let jaw = RigidBody::new(BodyId(10_190), shape, pose, MotionType::Kinematic);
                private_penetration_depth_m(&jaw, peg)
            })
            .fold(0.0_f64, f64::max);

        let target_offset = array_vec3(
            plant
                .calibrated_tool_geometry
                .side_target_center_offset_tool_m,
        );
        let maximum_side_target_penetration_m = [1.0, -1.0]
            .into_iter()
            .map(|sign| {
                let target = RigidBody::new(
                    BodyId(10_191),
                    Shape::Box {
                        half_extents_m: Vec3::new(
                            plant.calibrated_tool_geometry.side_target_radius_m,
                            plant.calibrated_tool_geometry.side_target_radius_m,
                            plant
                                .calibrated_tool_geometry
                                .side_target_axial_half_extent_m,
                        ),
                    },
                    tool_pose * Pose::from_translation(target_offset * sign),
                    MotionType::Kinematic,
                );
                private_penetration_depth_m(&target, peg)
            })
            .fold(0.0_f64, f64::max);

        (
            maximum_jaw_penetration_m,
            plant.private_palm_peg_penetration_m(),
            maximum_side_target_penetration_m,
        )
    }

    fn geometry_only_rig(plant: &ObservedPlant, roi: OpticalVec3) -> StructuredLightRig {
        let mut rig = plant.build_macro_rig(roi, 0);
        rig.config.base_dropout_probability = 0.0;
        rig.config.grazing_dropout_probability = 0.0;
        rig.config.camera_pixel_sigma_floor_px = 0.0;
        rig.config.projector_pixel_sigma_floor_px = 0.0;
        rig.config.photon_centroid_coefficient_px = 0.0;
        rig.config.speckle_axial_sigma_m = 0.0;
        rig.config.depth_quantization_m = 0.0;
        rig.config.minimum_feature_confidence = 0.0;
        rig
    }

    fn feature_envelope_roi(
        plant: &ObservedPlant,
        axis: Vec3,
        tool_center: Vec3,
        peg_center: Vec3,
        include_socket: bool,
    ) -> OpticalVec3 {
        let mut points = Vec::new();
        for feature in plant.feature_model(TOOL_OBJECT_ID) {
            points.push(tool_center + axis * feature.axial_coordinate_m);
        }
        for feature in plant.feature_model(PEG_OBJECT_ID) {
            points.push(peg_center + axis * feature.axial_coordinate_m);
        }
        if include_socket {
            let socket = array_vec3(plant.scenario.coupon.socket_center_nominal_world_m);
            for feature in plant.feature_model(SOCKET_OBJECT_ID) {
                points.push(socket + axis * feature.axial_coordinate_m);
            }
        }
        let minimum = points
            .iter()
            .min_by(|a, b| a.dot(axis).total_cmp(&b.dot(axis)))
            .copied()
            .unwrap();
        let maximum = points
            .iter()
            .max_by(|a, b| a.dot(axis).total_cmp(&b.dot(axis)))
            .copied()
            .unwrap();
        optical_vec((minimum + maximum) * 0.5)
    }

    #[test]
    fn macro_head_matches_m1d_geometry_and_timing_is_integer_tick_based() {
        let mut plant = settled_plant(M1eFault::None);
        let roi = plant.scenario.coupon.pick_peg_center_nominal_world_m;
        let rig = plant.build_macro_rig(array_optical_vec3(roi), 0);
        assert_eq!(rig.cameras[0].nominal.image_size, ImageSize::new(1280, 800));
        let baseline =
            (rig.cameras[0].nominal.center_world() - rig.projector.nominal.center_world()).norm();
        assert!((baseline - 0.012).abs() < 1.0e-15);
        assert_eq!(plant.motion_capabilities().settling_ticks, 12);
        let burst = plant
            .acquire_observation_burst(&[PEG_OBJECT_ID], roi)
            .unwrap();
        assert_eq!(burst.capture_end_tick - burst.capture_start_tick, 33);
        assert_eq!(burst.available_tick - burst.capture_end_tick, 20);
        assert_eq!(burst.measurements.len() + burst.missing.len(), 12);
    }

    #[test]
    fn calibrated_pick_capture_observes_all_four_fitted_ring_stations() {
        let mut plant = settled_plant(M1eFault::None);
        let roi = plant.scenario.coupon.pick_peg_center_nominal_world_m;
        // This is the same truth-independent datum/axis construction used by
        // the executive for blind capture entry, not a target derived from the
        // private peg body pose.
        let target = calibrated_pick_capture_target(&plant);
        plant
            .command_tool_position(vec3(target), MotionClass::Transit)
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let roi_optical = array_optical_vec3(roi);
        let mut rig = plant.build_macro_rig(roi_optical, 0);
        rig.config.base_dropout_probability = 0.0;
        rig.config.grazing_dropout_probability = 0.0;
        // This assertion isolates deterministic scene/FOV/occlusion geometry;
        // noisy burst behavior is covered by the acquisition checks below.
        rig.config.camera_pixel_sigma_floor_px = 0.0;
        rig.config.projector_pixel_sigma_floor_px = 0.0;
        rig.config.photon_centroid_coefficient_px = 0.0;
        rig.config.speckle_axial_sigma_m = 0.0;
        rig.config.depth_quantization_m = 0.0;
        rig.config.minimum_feature_confidence = 0.0;
        let nominal_scene = plant.build_optical_scene();
        for object_id in [PEG_OBJECT_ID, TOOL_OBJECT_ID] {
            let geometric_observations = plant
                .observe_fitted_ring_centers(
                    &rig,
                    &nominal_scene,
                    object_id,
                    roi_optical,
                    0xA11F_0001,
                )
                .unwrap();
            assert_eq!(geometric_observations.len(), FEATURE_IDS.len());
            assert!(
                geometric_observations
                    .iter()
                    .all(|observation| matches!(observation, FeaturePointObservation::Observed(_))),
                "object={object_id} observations={geometric_observations:#?}"
            );
        }

        plant
            .advance_ticks(plant.motion_capabilities().settling_ticks)
            .unwrap();
        let burst = plant
            .acquire_observation_burst(&[PEG_OBJECT_ID, TOOL_OBJECT_ID], roi)
            .unwrap();
        for object_id in [PEG_OBJECT_ID, TOOL_OBJECT_ID] {
            let observed_ids = burst
                .measurements
                .iter()
                .filter(|measurement| measurement.object_id == object_id)
                .map(|measurement| measurement.feature_id)
                .collect::<BTreeSet<_>>();
            assert_eq!(
                observed_ids,
                FEATURE_IDS.into_iter().collect(),
                "object={object_id} missing={:#?}",
                burst.missing
            );
        }
    }

    #[test]
    fn blind_capture_has_no_hidden_tool_peg_penetration_at_any_fixed_step() {
        let mut plant = settled_plant(M1eFault::None);
        plant
            .command_gripper(plant.maximum_gripper_opening_m())
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();

        let target = calibrated_pick_capture_target(&plant);
        plant
            .command_tool_position(vec3(target), MotionClass::Transit)
            .unwrap();

        let mut maximum_penetrations_m = [0.0_f64; 3];
        let mut sample_count = 0_u32;
        loop {
            let (jaw_m, palm_m, side_target_m) = capture_tool_peg_penetrations_m(&plant);
            for (maximum_m, sample_m) in
                maximum_penetrations_m
                    .iter_mut()
                    .zip([jaw_m, palm_m, side_target_m])
            {
                *maximum_m = maximum_m.max(sample_m);
            }
            assert_eq!(jaw_m, 0.0, "jaw/peg penetration at sample {sample_count}");
            assert_eq!(palm_m, 0.0, "palm/peg penetration at sample {sample_count}");
            assert_eq!(
                side_target_m, 0.0,
                "side-target/peg penetration at sample {sample_count}"
            );
            if plant.motion_status() != MotionStatus::Active {
                break;
            }
            plant.advance_one().unwrap();
            sample_count += 1;
            assert!(sample_count <= plant.scenario.motion.maximum_steps_per_motion);
        }

        assert!(sample_count > 1);
        assert_eq!(maximum_penetrations_m, [0.0; 3]);
    }

    #[test]
    fn socket_midpoint_view_observes_tool_side_target_and_socket() {
        let mut plant = settled_plant(M1eFault::None);
        let socket = array_vec3(plant.scenario.coupon.socket_center_nominal_world_m);
        let axis = array_vec3(plant.calibrated_socket_axis_world());
        let peg_tip_from_center_m =
            plant.scenario.coupon.peg_half_segment_m + 0.5 * plant.scenario.coupon.peg_diameter_m;
        let peg_center = socket
            + axis
                * (0.5 * plant.scenario.coupon.socket_depth_m
                    - peg_tip_from_center_m
                    - plant.scenario.motion.insert_approach_distance_m);
        let approach = peg_center - axis * plant.scenario.grasp.tool_to_peg_axial_offset_m;
        plant
            .command_tool_position(vec3(approach), MotionClass::Transit)
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let roi = feature_envelope_roi(&plant, axis, approach, peg_center, true);
        let mut rig = plant.build_macro_rig(roi, 0);
        rig.config.base_dropout_probability = 0.0;
        rig.config.grazing_dropout_probability = 0.0;
        rig.config.camera_pixel_sigma_floor_px = 0.0;
        rig.config.projector_pixel_sigma_floor_px = 0.0;
        rig.config.photon_centroid_coefficient_px = 0.0;
        rig.config.speckle_axial_sigma_m = 0.0;
        rig.config.depth_quantization_m = 0.0;
        rig.config.minimum_feature_confidence = 0.0;
        let scene = plant.build_optical_scene();
        for object_id in [TOOL_OBJECT_ID, SOCKET_OBJECT_ID] {
            let observations = plant
                .observe_fitted_ring_centers(&rig, &scene, object_id, roi, 0x50C0_0001)
                .unwrap();
            assert!(
                observations
                    .iter()
                    .all(|observation| matches!(observation, FeaturePointObservation::Observed(_))),
                "object={object_id} observations={observations:#?}"
            );
        }
    }

    #[test]
    fn peg_rings_remain_visible_through_tip_to_seat_and_post_release() {
        let mut plant = settled_plant(M1eFault::None);
        let grasp_target = observed_tail_grasp_target(&plant);
        plant
            .command_tool_position(vec3(grasp_target), MotionClass::Transit)
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let closed_opening_m =
            plant.scenario.coupon.peg_diameter_m - plant.scenario.grasp.commanded_pad_compression_m;
        plant.command_gripper(closed_opening_m).unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let grasp_candidate = plant
            .active_arm()
            .gripper
            .evaluate_partial_axial_overlap_candidate(
                plant.physical_tool_pose(),
                plant.peg_body(),
                plant.active_arm().gripper_config,
                plant.scenario.grasp.minimum_axial_grasp_overlap_m,
            );
        assert!(grasp_candidate.within_finger_depth);
        assert!(
            grasp_candidate.axial_overlap_m >= plant.scenario.grasp.minimum_axial_grasp_overlap_m
        );
        assert_eq!(plant.private_palm_peg_penetration_m(), 0.0);
        let calibrated_palm_plane_m = plant.calibrated_tool_geometry.palm_forward_plane_tool_z_m;
        // Move a hypothetical unrecessed palm through the observed tail-grasp
        // station to prove that the independent geometry query is sensitive,
        // rather than merely returning a constant clear result.
        plant.calibrated_tool_geometry.palm_forward_plane_tool_z_m =
            plant.scenario.grasp.tool_to_peg_axial_offset_m;
        assert!(plant.private_palm_peg_penetration_m() > 0.0);
        plant.calibrated_tool_geometry.palm_forward_plane_tool_z_m = calibrated_palm_plane_m;
        plant.commit_grasp().unwrap();

        let socket_axis = plant
            .socket_pose
            .transform_vector(Vec3::Z)
            .normalized_or(Vec3::Z);
        let socket_seat = plant
            .socket_pose
            .transform_point(Vec3::Z * (0.5 * plant.scenario.coupon.socket_depth_m));
        let peg_tip_from_center_m =
            plant.scenario.coupon.peg_half_segment_m + 0.5 * plant.scenario.coupon.peg_diameter_m;
        let states = [
            ("lead_in", plant.scenario.contact.lead_in_start_m),
            ("mid_insert", 0.5 * plant.scenario.contact.lead_in_start_m),
            (
                "seat_tolerance",
                plant.scenario.contact.seat_axial_tolerance_m,
            ),
            ("seated_held", 0.0),
        ];
        let mut seated_tool_center = Vec3::ZERO;
        for (label, tip_to_seat_m) in states {
            let peg_center = socket_seat - socket_axis * (peg_tip_from_center_m + tip_to_seat_m);
            let tool_center =
                peg_center - socket_axis * plant.scenario.grasp.tool_to_peg_axial_offset_m;
            // This is an optical-visibility fixture sweep, not an executive
            // motion test. Place the already-held assembly directly at each
            // declared station so the force interlock correctly remains
            // mandatory for every real transit exercised elsewhere.
            set_initial_tool_pose(&mut plant.mechanics, tool_center).unwrap();
            let held_local_pose = plant
                .active_arm()
                .held_body_local_pose
                .expect("geometry fixture retains the held PEG transform");
            let held_world_pose = plant.active_arm().tool_pose() * held_local_pose;
            plant
                .mechanics
                .body_mut(BodyId(PEG_OBJECT_ID))
                .unwrap()
                .pose = held_world_pose;
            plant.commanded_tool_position_world_m = vec3(tool_center);
            let roi = feature_envelope_roi(&plant, socket_axis, tool_center, peg_center, true);
            let rig = geometry_only_rig(&plant, roi);
            let scene = plant.build_optical_scene();
            let observations = plant
                .observe_fitted_ring_centers(&rig, &scene, PEG_OBJECT_ID, roi, 0x5EA7_0001)
                .unwrap();
            assert!(
                observations
                    .iter()
                    .all(|observation| matches!(observation, FeaturePointObservation::Observed(_))),
                "state={label} observations={observations:#?}"
            );
            seated_tool_center = tool_center;
        }

        let packet = plant.contact_packet();
        assert!(packet.contact_detected);
        assert!(packet.insertion_force_proxy_n <= plant.scenario.contact.maximum_force_proxy_n);
        plant.commit_release().unwrap();
        plant
            .command_gripper(plant.maximum_gripper_opening_m())
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let seated_peg_center = plant.peg_body().pose.translation;
        let roi = feature_envelope_roi(
            &plant,
            socket_axis,
            seated_tool_center,
            seated_peg_center,
            true,
        );
        let rig = geometry_only_rig(&plant, roi);
        let scene = plant.build_optical_scene();
        let observations = plant
            .observe_fitted_ring_centers(&rig, &scene, PEG_OBJECT_ID, roi, 0x5EA7_0001)
            .unwrap();
        assert!(
            observations
                .iter()
                .all(|observation| matches!(observation, FeaturePointObservation::Observed(_))),
            "post-release observations={observations:#?}"
        );
        let (jaw_penetration_m, side_target_penetration_m) =
            plant.private_tool_socket_component_penetrations_m();
        assert_eq!(jaw_penetration_m, 0.0);
        assert_eq!(side_target_penetration_m, 0.0);
        assert_eq!(plant.private_tool_socket_max_penetration_m(), 0.0);
    }

    #[test]
    fn occluded_mating_feature_fault_is_explicit_ray_blocker_geometry() {
        let mut plant = settled_plant(M1eFault::OccludedMatingFeature);
        let roi_array = plant.scenario.coupon.socket_center_nominal_world_m;
        let roi = array_optical_vec3(roi_array);
        let rig = plant.build_macro_rig(roi, 0);
        let mut scene = plant.build_optical_scene();
        assert!(scene
            .primitives
            .iter()
            .all(|primitive| primitive.tag != OPTICAL_TAG_FAULT_OCCLUDER));
        plant.inject_fault_optical_geometry(&rig, &mut scene);
        assert!(scene
            .primitives
            .iter()
            .any(|primitive| primitive.tag == OPTICAL_TAG_FAULT_OCCLUDER));
        let observations = plant
            .observe_fitted_ring_centers(&rig, &scene, SOCKET_OBJECT_ID, roi, 0x0CC1_5100)
            .unwrap();
        assert_eq!(observations.len(), FEATURE_IDS.len());
        assert!(observations.iter().all(|observation| matches!(
            observation,
            FeaturePointObservation::Missing {
                reason: MissingFeaturePoint::CameraOccluded,
                ..
            }
        )));

        let burst = plant
            .acquire_observation_burst(&[SOCKET_OBJECT_ID], roi_array)
            .unwrap();
        assert!(burst.measurements.is_empty());
        assert_eq!(burst.missing.len(), 12);
        assert!(burst
            .missing
            .iter()
            .all(|measurement| measurement.reason == "camera_occluded"));
        assert!(burst.calibration_reference_valid);
    }

    #[test]
    fn distinct_local_actor_can_occlude_a_fitted_ring_feature() {
        let plant = settled_plant(M1eFault::None);
        let roi_array = plant.scenario.coupon.pick_peg_center_nominal_world_m;
        let roi = array_optical_vec3(roi_array);
        let rig = plant.build_macro_rig(roi, 0);
        let scene = plant.build_optical_scene();
        let frame_index = 0x0CC1_0001;
        let visible = plant
            .observe_fitted_ring_centers(&rig, &scene, PEG_OBJECT_ID, roi, frame_index)
            .unwrap();
        assert!(
            matches!(visible[0], FeaturePointObservation::Observed(_)),
            "{visible:#?}"
        );

        let first_center =
            plant.truth_ring_center_points(PEG_OBJECT_ID, roi).unwrap()[0].point_world;
        let camera_center = rig.cameras[0].actual().center_world();
        let blocker_center = camera_center + (first_center - camera_center) * 0.80;
        let mut blocked_scene = scene.clone();
        blocked_scene.push(OpticalPrimitive::new(
            OpticalGeometry::Sphere(OpticalSphere {
                center: blocker_center,
                radius_m: 0.45e-3,
            }),
            optical_material(1.0, 1.0),
            OPTICAL_TAG_GRIPPER,
        ));
        let blocked = plant
            .observe_fitted_ring_centers(&rig, &blocked_scene, PEG_OBJECT_ID, roi, frame_index)
            .unwrap();
        assert!(matches!(
            blocked[0],
            FeaturePointObservation::Missing {
                reason: MissingFeaturePoint::CameraOccluded,
                ..
            }
        ));
    }

    #[test]
    fn burst_is_deterministic_and_contains_no_truth_fields() {
        let mut first = settled_plant(M1eFault::None);
        let roi = first.scenario.coupon.pick_peg_center_nominal_world_m;
        let a = first
            .acquire_observation_burst(&[PEG_OBJECT_ID, TOOL_OBJECT_ID], roi)
            .unwrap();
        let mut second = settled_plant(M1eFault::None);
        let b = second
            .acquire_observation_burst(&[TOOL_OBJECT_ID, PEG_OBJECT_ID], roi)
            .unwrap();
        assert_eq!(a, b);
        assert!(a.measurements.iter().all(|measurement| {
            measurement.calibrated_ray_count
                == if measurement.object_id == TOOL_OBJECT_ID {
                    4
                } else {
                    2
                }
        }));
    }

    #[test]
    fn injected_sensor_faults_are_boundary_effects() {
        let mut dropout = settled_plant(M1eFault::OpticalDropout);
        let roi = dropout.scenario.coupon.pick_peg_center_nominal_world_m;
        let burst = dropout
            .acquire_observation_burst(&[PEG_OBJECT_ID], roi)
            .unwrap();
        assert!(burst.measurements.is_empty());
        assert_eq!(burst.missing.len(), 12);

        let mut stale = settled_plant(M1eFault::StaleObservation);
        let burst = stale
            .acquire_observation_burst(&[PEG_OBJECT_ID], roi)
            .unwrap();
        let maximum_age_ticks = ticks_ceil(
            stale.scenario.optics.maximum_measurement_age_s,
            stale.fixed_dt_s(),
        );
        assert!(burst
            .measurements
            .iter()
            .all(
                |measurement| burst.available_tick - measurement.capture_tick > maximum_age_ticks
            ));

        let mut bias = settled_plant(M1eFault::ExcessiveCalibrationBias);
        let burst = bias
            .acquire_observation_burst(&[PEG_OBJECT_ID], roi)
            .unwrap();
        assert!(!burst.calibration_reference_valid);
    }

    #[test]
    fn correction_floor_fault_is_a_calibrated_capability_not_truth_pose() {
        let plant = settled_plant(M1eFault::CorrectionFloorTooLarge);
        assert!(
            plant
                .motion_capabilities()
                .minimum_reproducible_correction_m
                > 2.0 * plant.scenario.motion.correction_convergence_m
        );
    }

    #[test]
    fn undersized_tool_envelope_is_rejected_against_open_jaw_geometry() {
        let plant = settled_plant(M1eFault::None);
        let required_m = open_gripper_corner_radius_m(plant.active_arm().gripper_config);
        assert!(required_m.is_finite() && required_m > 1.0e-3);
        assert!(plant.scenario.safety.tool_envelope_radius_m >= required_m);

        let (mut undersized, _) = ObservedManipulationScenario::baseline().unwrap();
        undersized.safety.tool_envelope_radius_m = required_m - 1.0e-6;
        assert!(ObservedPlant::new(&undersized, M1eFault::None).is_err());

        let carried_required_m =
            plant.scenario.coupon.peg_half_segment_m + 0.5 * plant.scenario.coupon.peg_diameter_m;
        assert!((carried_required_m - 0.900e-3).abs() < 1.0e-15);
        assert!(plant.scenario.safety.carried_peg_envelope_radius_m >= carried_required_m);
        let (mut undersized, _) = ObservedManipulationScenario::baseline().unwrap();
        undersized.safety.carried_peg_envelope_radius_m = carried_required_m - 1.0e-6;
        assert!(ObservedPlant::new(&undersized, M1eFault::None).is_err());
    }

    #[test]
    fn machine_config_provenance_is_preserved_at_controller_boundary() {
        let plant = settled_plant(M1eFault::None);
        assert_eq!(plant.machine_config_id(), "pipe_machine_m1e_coupon_v1");
        assert_eq!(plant.machine_config_source_sha256().len(), 64);
        assert!(plant
            .machine_config_source_sha256()
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit()));
    }

    #[test]
    fn calibrated_fixture_keep_outs_make_transfer_preflight_non_vacuous() {
        let nominal = settled_plant(M1eFault::None);
        let obstacles = nominal.calibrated_planning_obstacles();
        assert_eq!(
            obstacles.len(),
            nominal.scenario.safety.planning_obstacles.len()
        );
        assert!(obstacles.windows(2).all(|pair| pair[0].id < pair[1].id));
        let transfer = SweptEnvelope {
            center_start_world_m: nominal.scenario.coupon.pick_peg_center_nominal_world_m,
            center_end_world_m: nominal.scenario.coupon.socket_center_nominal_world_m,
            radius_m: nominal
                .scenario
                .safety
                .tool_envelope_radius_m
                .max(nominal.scenario.safety.carried_peg_envelope_radius_m),
            position_sigma_m: nominal.scenario.optics.correlated_calibration_sigma_m,
            hard_position_bound_m: 0.0,
            path_deviation_bound_m: 0.0,
        };
        assert!(preflight_swept_envelope(
            transfer,
            &obstacles,
            nominal.scenario.safety.minimum_obstacle_clearance_m,
        )
        .is_ok());

        let injected = settled_plant(M1eFault::CarriedPartCollision);
        let obstacles = injected.calibrated_planning_obstacles();
        assert_eq!(
            obstacles.len(),
            injected.scenario.safety.planning_obstacles.len() + 1
        );
        assert!(preflight_swept_envelope(
            transfer,
            &obstacles,
            injected.scenario.safety.minimum_obstacle_clearance_m,
        )
        .is_err());
    }

    #[test]
    fn commanded_fk_path_deviation_flips_an_off_chord_clearance_to_rejection() {
        let plant = settled_plant(M1eFault::None);
        let start = array_vec3(plant.commanded_tool_position_world_m());
        let target = array_vec3(plant.scenario.coupon.socket_center_nominal_world_m)
            - array_vec3(plant.calibrated_socket_axis_world()) * 2.0e-3;
        let path_deviation_bound_m = plant
            .preview_tool_path_deviation_bound_m(vec3(target), MotionClass::Transit)
            .unwrap();
        assert!(path_deviation_bound_m.is_finite());
        assert!(path_deviation_bound_m > 0.0);

        let chord = target - start;
        let chord_direction = chord.normalized_or(Vec3::Z);
        let reference = if chord_direction.dot(Vec3::X).abs() < 0.9 {
            Vec3::X
        } else {
            Vec3::Y
        };
        let normal = chord_direction.cross(reference).normalized_or(Vec3::Y);
        let required_clearance_m = 0.100e-3;
        let obstacle = PlanningObstacle {
            id: 77,
            center_world_m: vec3(
                start.lerp(target, 0.5)
                    + normal * (required_clearance_m + 0.5 * path_deviation_bound_m),
            ),
            conservative_radius_m: 0.0,
            position_sigma_m: 0.0,
        };
        let chord_only = SweptEnvelope {
            center_start_world_m: vec3(start),
            center_end_world_m: vec3(target),
            radius_m: 0.0,
            position_sigma_m: 0.0,
            hard_position_bound_m: 0.0,
            path_deviation_bound_m: 0.0,
        };
        assert!(preflight_swept_envelope(chord_only, &[obstacle], required_clearance_m).is_ok());

        let bounded_fk_path = SweptEnvelope {
            path_deviation_bound_m,
            ..chord_only
        };
        assert!(matches!(
            preflight_swept_envelope(bounded_fk_path, &[obstacle], required_clearance_m),
            Err(super::super::controller::SweptPreflightFailure::Clearance(check))
                if check.obstacle_id == obstacle.id
        ));
    }

    #[test]
    fn commanded_axis_change_preview_bounds_every_joint_path_sample() {
        let plant = settled_plant(M1eFault::None);
        let target = array_vec3(plant.commanded_tool_position_world_m())
            + array_vec3(plant.calibrated_socket_axis_world()) * 0.100e-3;
        let plan = plant
            .preview_tool_motion_plan(vec3(target), MotionClass::Insertion)
            .unwrap();
        let declared_bound_rad = plant
            .preview_tool_axis_change_bound_rad(vec3(target), MotionClass::Insertion)
            .unwrap();
        let mut arm = plant.active_arm().arm.clone();
        arm.set_positions(plan.start).unwrap();
        let start_axis = (arm.forward_kinematics().tool_pose.rotation
            * plant.physical_tool_axis_tilt)
            .rotate_vec3(Vec3::Z)
            .normalized_or(Vec3::Z);
        for sample_index in 0..=100 {
            arm.set_positions(plan.sample(f64::from(sample_index) / 100.0))
                .unwrap();
            let sample_axis = (arm.forward_kinematics().tool_pose.rotation
                * plant.physical_tool_axis_tilt)
                .rotate_vec3(Vec3::Z)
                .normalized_or(Vec3::Z);
            let excursion_rad = start_axis
                .cross(sample_axis)
                .length()
                .atan2(start_axis.dot(sample_axis).clamp(-1.0, 1.0));
            assert!(excursion_rad <= declared_bound_rad + 1.0e-12);
        }
    }

    #[test]
    fn correction_trace_enforces_cartesian_velocity_and_acceleration_limits() {
        let mut plant = settled_plant(M1eFault::None);
        let start = plant.active_arm().tool_pose().translation;
        let target = array_vec3(plant.commanded_tool_position_world_m())
            + Vec3::Y * plant.scenario.motion.maximum_correction_m;
        let preview_duration_s = plant
            .preview_tool_motion_duration_s(vec3(target), MotionClass::Correction)
            .unwrap();
        plant
            .command_tool_position(vec3(target), MotionClass::Correction)
            .unwrap();
        let dt_s = plant.fixed_dt_s();
        let mut previous_position = start;
        let mut previous_velocity = Vec3::ZERO;
        let mut peak_velocity_m_s = 0.0_f64;
        let mut peak_acceleration_m_s2 = 0.0_f64;
        let mut steps = 0_u32;
        while plant.motion_status() == MotionStatus::Active {
            plant.advance_one().unwrap();
            let position = plant.active_arm().tool_pose().translation;
            let velocity = (position - previous_position) / dt_s;
            let acceleration = (velocity - previous_velocity) / dt_s;
            peak_velocity_m_s = peak_velocity_m_s.max(velocity.length());
            peak_acceleration_m_s2 = peak_acceleration_m_s2.max(acceleration.length());
            previous_position = position;
            previous_velocity = velocity;
            steps += 1;
            assert!(steps <= plant.scenario.motion.maximum_steps_per_motion);
        }
        // Include the deterministic transition from the last sampled motion
        // velocity to the stopped state on the next fixed tick.
        peak_acceleration_m_s2 = peak_acceleration_m_s2.max(previous_velocity.length() / dt_s);
        eprintln!(
            "M1e bounded correction trace: peak_velocity={peak_velocity_m_s:.12e} m/s, peak_acceleration={peak_acceleration_m_s2:.12e} m/s^2, preview={preview_duration_s:.9e} s, steps={steps}"
        );
        assert_eq!(
            u64::from(steps),
            ticks_ceil(preview_duration_s, plant.fixed_dt_s())
        );
        assert!(
            peak_velocity_m_s <= plant.scenario.motion.maximum_correction_velocity_m_s + 1.0e-12,
            "peak velocity {peak_velocity_m_s:.12e} m/s"
        );
        assert!(
            peak_acceleration_m_s2
                <= plant.scenario.motion.maximum_correction_acceleration_m_s2 + 1.0e-9,
            "peak acceleration {peak_acceleration_m_s2:.12e} m/s^2"
        );
    }

    #[test]
    fn jaw_contact_persists_after_detach_until_jaws_open() {
        let mut plant = settled_plant(M1eFault::None);
        let grasp_target = observed_tail_grasp_target(&plant);
        plant
            .command_tool_position(vec3(grasp_target), MotionClass::Transit)
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let closed_opening_m =
            plant.scenario.coupon.peg_diameter_m - plant.scenario.grasp.commanded_pad_compression_m;
        plant.command_gripper(closed_opening_m).unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let pre_grasp = plant.contact_packet();
        assert!(pre_grasp.left_pad_contact && pre_grasp.right_pad_contact);
        plant.commit_grasp().unwrap();

        plant.mechanics.release_body_serial(ACTIVE_ARM_ID).unwrap();
        let before_open = plant.contact_packet();
        assert!(before_open.left_pad_contact && before_open.right_pad_contact);
        assert!(before_open.left_pad_deflection_m > 0.0);
        assert!(before_open.right_pad_deflection_m > 0.0);
        assert!(before_open.grip_force_proxy_n > 0.0);
        assert!(!before_open.contact_detected);
        assert_eq!(before_open.insertion_force_proxy_n, 0.0);

        plant
            .command_gripper(plant.maximum_gripper_opening_m())
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let after_open = plant.contact_packet();
        assert!(!after_open.left_pad_contact && !after_open.right_pad_contact);
        assert_eq!(after_open.left_pad_deflection_m, 0.0);
        assert_eq!(after_open.right_pad_deflection_m, 0.0);
        assert_eq!(after_open.grip_force_proxy_n, 0.0);
    }

    #[test]
    fn grasp_outside_capture_fault_moves_the_free_peg_before_contact() {
        let mut plant = settled_plant(M1eFault::GraspOutsideCapture);
        let grasp_target = observed_tail_grasp_target(&plant);
        plant
            .command_tool_position(vec3(grasp_target), MotionClass::Transit)
            .unwrap();
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let before = plant.peg_body().pose.translation;
        let closed_opening_m =
            plant.scenario.coupon.peg_diameter_m - plant.scenario.grasp.commanded_pad_compression_m;
        plant.command_gripper(closed_opening_m).unwrap();
        let after = plant.peg_body().pose.translation;
        assert!((after - before).length() > 2.0 * plant.scenario.grasp.maximum_center_offset_m);
        plant
            .advance_until_idle(plant.scenario.motion.maximum_steps_per_motion)
            .unwrap();
        let packet = plant.contact_packet();
        assert!(!(packet.left_pad_contact && packet.right_pad_contact));
        assert_eq!(
            plant.commit_grasp(),
            Err(PlantFailure::GraspOutsideCaptureRegion)
        );
    }

    #[test]
    fn diagonal_measurement_covariance_dominates_full_correlated_covariance() {
        let covariance_m2 = [
            [4.0e-12, 3.0e-12, 0.0],
            [3.0e-12, 4.0e-12, 0.0],
            [0.0, 0.0, 1.0e-12],
        ];
        let bound_m2 = conservative_covariance_variance_bound(covariance_m2).unwrap();
        assert_eq!(bound_m2, 7.0e-12);
        assert!(bound_m2 > covariance_m2[0][0]);
        for direction in [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [
                core::f64::consts::FRAC_1_SQRT_2,
                core::f64::consts::FRAC_1_SQRT_2,
                0.0,
            ],
        ] {
            let projected_variance_m2 = (0..3)
                .map(|row| {
                    (0..3)
                        .map(|column| {
                            direction[row] * covariance_m2[row][column] * direction[column]
                        })
                        .sum::<f64>()
                })
                .sum::<f64>();
            let squared_norm = direction.iter().map(|value| value * value).sum::<f64>();
            assert!(projected_variance_m2 <= bound_m2 * squared_norm + 1.0e-24);
        }
    }
}
