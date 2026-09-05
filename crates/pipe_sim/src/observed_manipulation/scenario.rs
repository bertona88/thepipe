//! Versioned M1e input contract. All persistent distances are metres and all
//! persistent times are seconds.

use core::fmt;

use serde::{Deserialize, Serialize};

use crate::sha256_hex;

pub const M1E_SCENARIO_SCHEMA_VERSION: u32 = 1;
const M1E_PEG_FEATURE_AVAILABLE_SPAN_M: f64 = 0.400e-3;
/// The injected jam must be geometrically beyond the classifier boundary,
/// while preserving an explicit margin below the independent force trip.
pub(super) const M1E_JAM_CLASSIFICATION_MARGIN_M: f64 = 6.0e-6;
const M1E_JAM_FORCE_HEADROOM_FRACTION: f64 = 0.05;
pub const BASELINE_M1E_SCENARIO_JSON: &str =
    include_str!("../../../../scenarios/observed_manipulation_m1e_v1.json");

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ObservedManipulationScenario {
    pub schema_version: u32,
    pub id: String,
    pub machine_config_id: String,
    pub status: String,
    pub seed: u64,
    pub pose_state: String,
    pub claim_boundary: String,
    pub coupon: CouponConfig,
    pub optics: OpticsConfig,
    pub estimator: EstimatorScenarioConfig,
    pub motion: MotionConfig,
    pub grasp: GraspConfig,
    pub contact: ContactConfig,
    pub safety: SafetyScenarioConfig,
    pub fault_profiles: Vec<FaultProfile>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct CouponConfig {
    pub peg_diameter_m: f64,
    pub peg_half_segment_m: f64,
    pub socket_radial_clearance_m: f64,
    /// Full axial distance from the entrance plane to the far seat datum.
    pub socket_depth_m: f64,
    /// Minimum end-to-end span of every object's four centerline features.
    pub minimum_feature_axial_span_m: f64,
    pub socket_wall_thickness_m: f64,
    /// Magnitude of the symmetric +/- local-X coded-rail center offset.
    pub socket_fiducial_lateral_offset_m: f64,
    pub socket_fiducial_radius_m: f64,
    pub socket_fiducial_axial_half_extent_m: f64,
    pub pick_peg_center_nominal_world_m: [f64; 3],
    pub socket_center_nominal_world_m: [f64; 3],
    pub initial_peg_error_m: [f64; 3],
    pub initial_socket_error_m: [f64; 3],
    pub initial_tool_command_error_m: [f64; 3],
    pub initial_peg_axis_tilt_rad: [f64; 2],
    pub initial_socket_axis_tilt_rad: [f64; 2],
    pub initial_tool_axis_tilt_rad: [f64; 2],
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OpticsConfig {
    pub head_id: u32,
    pub camera_id: u32,
    pub projector_id: u32,
    pub image_size_px: [u32; 2],
    pub field_size_at_target_m: [f64; 2],
    pub working_distance_m: f64,
    pub camera_projector_baseline_m: f64,
    pub pattern_count: u32,
    pub pattern_rate_hz: f64,
    pub processing_latency_s: f64,
    pub settling_interval_s: f64,
    pub burst_frame_count: u32,
    pub camera_localization_sigma_px: f64,
    pub projector_localization_sigma_px: f64,
    pub correlated_calibration_sigma_m: f64,
    pub calibration_bias_m: [f64; 3],
    pub drift_per_burst_m: [f64; 3],
    pub base_dropout_probability: f64,
    pub grazing_dropout_probability: f64,
    pub clear_wall_signal_scale: f64,
    pub minimum_confidence: f64,
    pub minimum_distinct_features: u32,
    pub minimum_calibrated_rays_per_point: u32,
    pub minimum_triangulation_heads: u32,
    pub minimum_calibration_reference_samples: u32,
    pub maximum_calibration_reference_residual_m: f64,
    pub maximum_measurement_age_s: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct EstimatorScenarioConfig {
    pub maximum_feature_residual_m: f64,
    pub maximum_innovation_sigma: f64,
    pub minimum_accepted_feature_span_m: f64,
    pub macro_position_sigma_limit_m: f64,
    pub grasp_position_sigma_limit_m: f64,
    pub transfer_position_sigma_limit_m: f64,
    pub insertion_position_sigma_limit_m: f64,
    pub axis_sigma_limit_rad: f64,
    pub transfer_axis_sigma_limit_rad: f64,
    pub free_prediction_sigma_m_per_m: f64,
    pub loaded_hold_process_sigma_m_per_sqrt_s: f64,
    pub held_transform_sigma_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MotionConfig {
    /// Initial blind-entry tool standoff along the calibrated peg-tail axis.
    /// It must exceed `pick_capture_axial_standoff_m`, making entry a monotonic
    /// approach on one surveyed corridor rather than a lateral chord.
    pub pick_capture_start_axial_standoff_m: f64,
    /// Blind-entry distance from the nominal peg center to the commanded tool
    /// center, measured toward the peg tail along the calibrated coupon axis.
    pub pick_capture_axial_standoff_m: f64,
    /// Calibration/fixture bound on the residual vector between the actual
    /// peg/tool geometry and the commanded nominal tail-axis corridor over the
    /// complete blind sweep, including commanded-path tracking error.
    pub pick_capture_relative_position_bound_m: f64,
    /// Calibration/fixture bound that covers both peg-to-calibrated-axis and
    /// peg-to-tool-axis error during blind entry.
    pub pick_capture_relative_axis_bound_rad: f64,
    pub insert_approach_distance_m: f64,
    pub maximum_correction_m: f64,
    pub minimum_reproducible_correction_m: f64,
    pub differential_backlash_m: f64,
    pub loaded_hold_error_world_m: [f64; 3],
    pub correction_gain: f64,
    pub correction_convergence_m: f64,
    pub maximum_correction_iterations: u32,
    pub maximum_steps_per_motion: u32,
    pub insertion_increment_m: f64,
    pub maximum_insertion_increments: u32,
    pub near_contact_distance_m: f64,
    pub maximum_correction_velocity_m_s: f64,
    pub maximum_correction_acceleration_m_s2: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GraspConfig {
    pub commanded_pad_compression_m: f64,
    /// `B_center - T` along the observed common axis at the grasp.
    pub tool_to_peg_axial_offset_m: f64,
    pub minimum_axial_grasp_overlap_m: f64,
    pub maximum_center_offset_m: f64,
    pub maximum_axis_error_rad: f64,
    pub minimum_bilateral_pad_deflection_m: f64,
    pub maximum_bilateral_pad_deflection_m: f64,
    pub minimum_grip_force_n: f64,
    pub maximum_grip_force_n: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ContactConfig {
    pub lead_in_start_m: f64,
    pub recoverable_lateral_error_m: f64,
    pub maximum_lateral_error_m: f64,
    pub seat_axial_tolerance_m: f64,
    pub seat_lateral_tolerance_m: f64,
    pub seat_axis_tolerance_rad: f64,
    pub lateral_stiffness_proxy_n_per_m: f64,
    pub axial_stiffness_proxy_n_per_m: f64,
    pub maximum_force_proxy_n: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SafetyScenarioConfig {
    pub tool_envelope_radius_m: f64,
    pub carried_peg_envelope_radius_m: f64,
    pub minimum_obstacle_clearance_m: f64,
    pub retreat_distance_m: f64,
    pub maximum_phase_retries: u32,
    pub planning_obstacles: Vec<PlanningObstacleConfig>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PlanningObstacleConfig {
    pub id: u32,
    pub center_world_m: [f64; 3],
    pub conservative_radius_m: f64,
    pub position_sigma_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct FaultProfile {
    pub id: String,
    pub expected_terminal_reason: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum M1eFault {
    #[default]
    None,
    OpticalDropout,
    ExcessiveCalibrationBias,
    StaleObservation,
    CorrectionFloorTooLarge,
    GraspOutsideCapture,
    InsertionJam,
    OccludedMatingFeature,
    CarriedPartCollision,
    InconsistentObservation,
    NonConvergence,
}

impl M1eFault {
    pub const fn id(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::OpticalDropout => "optical_dropout",
            Self::ExcessiveCalibrationBias => "excessive_calibration_bias",
            Self::StaleObservation => "stale_observation",
            Self::CorrectionFloorTooLarge => "correction_floor_too_large",
            Self::GraspOutsideCapture => "grasp_outside_capture",
            Self::InsertionJam => "insertion_jam",
            Self::OccludedMatingFeature => "occluded_mating_feature",
            Self::CarriedPartCollision => "carried_part_collision",
            Self::InconsistentObservation => "inconsistent_observation",
            Self::NonConvergence => "non_convergence",
        }
    }

    pub const fn available() -> &'static [&'static str] {
        &[
            "none",
            "optical_dropout",
            "excessive_calibration_bias",
            "stale_observation",
            "correction_floor_too_large",
            "grasp_outside_capture",
            "insertion_jam",
            "occluded_mating_feature",
            "carried_part_collision",
            "inconsistent_observation",
            "non_convergence",
        ]
    }

    /// Controller-owned terminal reason for a fault-injection acceptance run.
    /// Scenario files repeat this value for report readability, but may not
    /// redefine the success oracle.
    pub const fn expected_terminal_reason(self) -> Option<&'static str> {
        match self {
            Self::None => None,
            Self::OpticalDropout => Some("optical_dropout"),
            Self::ExcessiveCalibrationBias => Some("excessive_calibration_bias"),
            Self::StaleObservation => Some("stale_measurement"),
            Self::CorrectionFloorTooLarge => Some("correction_floor_too_large"),
            Self::GraspOutsideCapture => Some("grasp_outside_capture_region"),
            Self::InsertionJam => Some("insertion_jam"),
            Self::OccludedMatingFeature => Some("required_mating_feature_occluded"),
            Self::CarriedPartCollision => Some("carried_part_collision_risk"),
            Self::InconsistentObservation => Some("observation_outlier"),
            Self::NonConvergence => Some("correction_non_convergence"),
        }
    }
}

impl core::str::FromStr for M1eFault {
    type Err = ScenarioError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value.trim().to_ascii_lowercase().replace('-', "_").as_str() {
            "none" | "nominal" => Ok(Self::None),
            "optical_dropout" => Ok(Self::OpticalDropout),
            "excessive_calibration_bias" => Ok(Self::ExcessiveCalibrationBias),
            "stale_observation" => Ok(Self::StaleObservation),
            "correction_floor_too_large" => Ok(Self::CorrectionFloorTooLarge),
            "grasp_outside_capture" => Ok(Self::GraspOutsideCapture),
            "insertion_jam" => Ok(Self::InsertionJam),
            "occluded_mating_feature" => Ok(Self::OccludedMatingFeature),
            "carried_part_collision" => Ok(Self::CarriedPartCollision),
            "inconsistent_observation" => Ok(Self::InconsistentObservation),
            "non_convergence" => Ok(Self::NonConvergence),
            other => Err(ScenarioError::UnknownFault(other.to_owned())),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ScenarioError {
    Json(String),
    Invalid(String),
    UnknownFault(String),
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Json(message) => write!(formatter, "M1e scenario JSON: {message}"),
            Self::Invalid(message) => write!(formatter, "invalid M1e scenario: {message}"),
            Self::UnknownFault(value) => write!(
                formatter,
                "unknown M1e fault '{value}'; expected {}",
                M1eFault::available().join(", ")
            ),
        }
    }
}

impl std::error::Error for ScenarioError {}

impl ObservedManipulationScenario {
    pub fn baseline() -> Result<(Self, String), ScenarioError> {
        Self::from_json(BASELINE_M1E_SCENARIO_JSON)
    }

    pub fn from_json(json: &str) -> Result<(Self, String), ScenarioError> {
        let scenario: Self =
            serde_json::from_str(json).map_err(|error| ScenarioError::Json(error.to_string()))?;
        scenario.validate()?;
        Ok((scenario, sha256_hex(json.as_bytes())))
    }

    pub fn expected_failure_reason(&self, fault: M1eFault) -> Option<&str> {
        if fault == M1eFault::None {
            return None;
        }
        self.fault_profiles
            .iter()
            .find(|profile| profile.id == fault.id())
            .map(|profile| profile.expected_terminal_reason.as_str())
    }

    /// Validate constraints that apply only when a selected injected fault is
    /// executable. Keeping this separate lets deliberately low force-limit
    /// scenarios exercise the protective interlock without also promising
    /// that the distinct below-limit jam fault is feasible.
    pub(super) fn validate_fault(&self, fault: M1eFault) -> Result<(), ScenarioError> {
        if fault == M1eFault::InsertionJam && self.insertion_jam_lateral_target_m().is_none() {
            return Err(ScenarioError::Invalid(
                "insertion_jam has no lateral interval above the jam classifier threshold and below the force limit with required headroom"
                    .to_owned(),
            ));
        }
        Ok(())
    }

    /// Smallest deterministic lateral target that is unmistakably beyond the
    /// jam threshold. Feasibility reserves one complete insertion increment's
    /// axial load plus five percent of the force limit for angular/path error.
    pub(super) fn insertion_jam_lateral_target_m(&self) -> Option<f64> {
        let contact = &self.contact;
        let target_m = contact.maximum_lateral_error_m + M1E_JAM_CLASSIFICATION_MARGIN_M;
        let worst_increment_m = self
            .motion
            .insertion_increment_m
            .min(contact.lead_in_start_m);
        let axial_deflection_m =
            worst_increment_m * contact.seat_axial_tolerance_m / contact.lead_in_start_m;
        let lateral_deflection_m = (target_m - contact.seat_lateral_tolerance_m).max(0.0);
        let required_force_n = lateral_deflection_m * contact.lateral_stiffness_proxy_n_per_m
            + axial_deflection_m * contact.axial_stiffness_proxy_n_per_m;
        let force_budget_n =
            contact.maximum_force_proxy_n * (1.0 - M1E_JAM_FORCE_HEADROOM_FRACTION);
        (required_force_n <= force_budget_n).then_some(target_m)
    }

    fn validate(&self) -> Result<(), ScenarioError> {
        let fail = |message: &str| Err(ScenarioError::Invalid(message.to_owned()));
        if self.schema_version != M1E_SCENARIO_SCHEMA_VERSION {
            return fail("unsupported schema_version");
        }
        if self.id.trim().is_empty()
            || self.machine_config_id.trim().is_empty()
            || self.status != "modeled_vertical_slice_not_hardware_qualified"
            || !self.pose_state.starts_with("axisymmetric_5d")
            || self.claim_boundary.trim().is_empty()
        {
            return fail("identity, status, pose-state, or claim boundary is invalid");
        }
        let finite = |values: &[f64]| values.iter().all(|value| value.is_finite());
        if !finite(&[
            self.coupon.peg_diameter_m,
            self.coupon.peg_half_segment_m,
            self.coupon.socket_radial_clearance_m,
            self.coupon.socket_depth_m,
            self.coupon.minimum_feature_axial_span_m,
            self.coupon.socket_wall_thickness_m,
            self.coupon.socket_fiducial_lateral_offset_m,
            self.coupon.socket_fiducial_radius_m,
            self.coupon.socket_fiducial_axial_half_extent_m,
        ]) || self.coupon.peg_diameter_m <= 0.0
            || self.coupon.peg_half_segment_m <= 0.0
            || self.coupon.socket_radial_clearance_m <= 0.0
            || self.coupon.socket_depth_m <= 0.0
            || self.coupon.minimum_feature_axial_span_m
                < self.estimator.minimum_accepted_feature_span_m
            || self.coupon.minimum_feature_axial_span_m > M1E_PEG_FEATURE_AVAILABLE_SPAN_M
            || self.coupon.socket_wall_thickness_m <= 0.0
            || self.coupon.socket_fiducial_lateral_offset_m <= 0.0
            || self.coupon.socket_fiducial_radius_m <= 0.0
            || self.coupon.socket_fiducial_axial_half_extent_m <= 0.0
            || self.coupon.socket_fiducial_lateral_offset_m
                <= 0.5 * self.coupon.peg_diameter_m
                    + self.coupon.socket_radial_clearance_m
                    + self.coupon.socket_wall_thickness_m
                    + self.coupon.socket_fiducial_radius_m
            || self.coupon.minimum_feature_axial_span_m + 2.0 * self.coupon.socket_fiducial_radius_m
                > 2.0 * self.coupon.socket_fiducial_axial_half_extent_m
            || !finite(&self.coupon.pick_peg_center_nominal_world_m)
            || !finite(&self.coupon.socket_center_nominal_world_m)
            || !finite(&self.coupon.initial_peg_error_m)
            || !finite(&self.coupon.initial_socket_error_m)
            || !finite(&self.coupon.initial_tool_command_error_m)
            || !finite(&self.coupon.initial_peg_axis_tilt_rad)
            || !finite(&self.coupon.initial_socket_axis_tilt_rad)
            || !finite(&self.coupon.initial_tool_axis_tilt_rad)
        {
            return fail("coupon geometry and initial errors must be finite and positive");
        }
        let optics = &self.optics;
        if !finite(&[
            optics.working_distance_m,
            optics.camera_projector_baseline_m,
            optics.pattern_rate_hz,
            optics.processing_latency_s,
            optics.settling_interval_s,
            optics.camera_localization_sigma_px,
            optics.projector_localization_sigma_px,
            optics.correlated_calibration_sigma_m,
            optics.base_dropout_probability,
            optics.grazing_dropout_probability,
            optics.clear_wall_signal_scale,
            optics.minimum_confidence,
            optics.maximum_calibration_reference_residual_m,
            optics.maximum_measurement_age_s,
        ]) || optics.image_size_px.contains(&0)
            || !finite(&optics.field_size_at_target_m)
            || optics
                .field_size_at_target_m
                .iter()
                .any(|value| *value <= 0.0)
            || optics.working_distance_m <= 0.0
            || optics.camera_projector_baseline_m <= 0.0
            || optics.pattern_count == 0
            || optics.pattern_rate_hz <= 0.0
            || optics.processing_latency_s < 0.0
            || optics.settling_interval_s < 0.0
            || optics.burst_frame_count == 0
            || optics.camera_localization_sigma_px <= 0.0
            || optics.projector_localization_sigma_px <= 0.0
            || optics.correlated_calibration_sigma_m <= 0.0
            || !finite(&optics.calibration_bias_m)
            || !finite(&optics.drift_per_burst_m)
            || !(0.0..=1.0).contains(&optics.base_dropout_probability)
            || !(0.0..=1.0).contains(&optics.grazing_dropout_probability)
            || !(0.0..=1.0).contains(&optics.clear_wall_signal_scale)
            || !(0.0..=1.0).contains(&optics.minimum_confidence)
            || optics.minimum_distinct_features < 4
            || optics.minimum_calibrated_rays_per_point != 2
            || optics.minimum_triangulation_heads != 1
            || optics.minimum_calibration_reference_samples == 0
            || optics.minimum_calibration_reference_samples > optics.burst_frame_count
            || optics.maximum_calibration_reference_residual_m
                < optics.correlated_calibration_sigma_m
            || optics.maximum_measurement_age_s <= 0.0
            || f64::from(optics.pattern_count) / optics.pattern_rate_hz
                + optics.processing_latency_s
                > optics.maximum_measurement_age_s
        {
            return fail("optics geometry, timing, quality, or observability gate is invalid");
        }
        let estimator = &self.estimator;
        if !finite(&[
            estimator.maximum_feature_residual_m,
            estimator.maximum_innovation_sigma,
            estimator.minimum_accepted_feature_span_m,
            estimator.macro_position_sigma_limit_m,
            estimator.grasp_position_sigma_limit_m,
            estimator.transfer_position_sigma_limit_m,
            estimator.insertion_position_sigma_limit_m,
            estimator.axis_sigma_limit_rad,
            estimator.transfer_axis_sigma_limit_rad,
            estimator.free_prediction_sigma_m_per_m,
            estimator.loaded_hold_process_sigma_m_per_sqrt_s,
            estimator.held_transform_sigma_m,
        ]) || estimator.maximum_feature_residual_m <= 0.0
            || estimator.maximum_innovation_sigma <= 0.0
            || estimator.minimum_accepted_feature_span_m <= 0.0
            || estimator.macro_position_sigma_limit_m < optics.correlated_calibration_sigma_m
            || estimator.grasp_position_sigma_limit_m < optics.correlated_calibration_sigma_m
            || estimator.transfer_position_sigma_limit_m < optics.correlated_calibration_sigma_m
            || estimator.insertion_position_sigma_limit_m < optics.correlated_calibration_sigma_m
            || estimator.axis_sigma_limit_rad <= 0.0
            || estimator.transfer_axis_sigma_limit_rad < estimator.axis_sigma_limit_rad
            || estimator.free_prediction_sigma_m_per_m < 0.0
            || estimator.loaded_hold_process_sigma_m_per_sqrt_s < 0.0
            || estimator.held_transform_sigma_m < 0.0
        {
            return fail("estimator uncertainty and innovation limits are invalid");
        }
        let motion = &self.motion;
        if !finite(&[
            motion.pick_capture_start_axial_standoff_m,
            motion.pick_capture_axial_standoff_m,
            motion.pick_capture_relative_position_bound_m,
            motion.pick_capture_relative_axis_bound_rad,
            motion.insert_approach_distance_m,
            motion.maximum_correction_m,
            motion.minimum_reproducible_correction_m,
            motion.differential_backlash_m,
            motion.correction_gain,
            motion.correction_convergence_m,
            motion.insertion_increment_m,
            motion.near_contact_distance_m,
            motion.maximum_correction_velocity_m_s,
            motion.maximum_correction_acceleration_m_s2,
        ]) || !finite(&motion.loaded_hold_error_world_m)
            || motion.pick_capture_start_axial_standoff_m <= motion.pick_capture_axial_standoff_m
            || motion.pick_capture_axial_standoff_m <= 0.0
            || motion.pick_capture_relative_position_bound_m <= 0.0
            || !(0.0..core::f64::consts::FRAC_PI_2)
                .contains(&motion.pick_capture_relative_axis_bound_rad)
            || motion.insert_approach_distance_m <= 0.0
            || motion.maximum_correction_m <= 0.0
            || motion.minimum_reproducible_correction_m <= 0.0
            || motion.minimum_reproducible_correction_m > motion.maximum_correction_m
            || motion.differential_backlash_m < 0.0
            || !(0.0..=1.0).contains(&motion.correction_gain)
            || motion.correction_gain == 0.0
            || motion.correction_convergence_m <= 0.0
            || motion.maximum_correction_iterations == 0
            || motion.maximum_steps_per_motion == 0
            || motion.insertion_increment_m <= 0.0
            || motion.insertion_increment_m > motion.near_contact_distance_m
            || motion.maximum_insertion_increments == 0
            || motion.near_contact_distance_m <= 0.0
            || motion.maximum_correction_velocity_m_s <= 0.0
            || motion.maximum_correction_acceleration_m_s2 <= 0.0
        {
            return fail("motion policy is invalid");
        }
        let grasp = &self.grasp;
        if !finite(&[
            grasp.commanded_pad_compression_m,
            grasp.tool_to_peg_axial_offset_m,
            grasp.minimum_axial_grasp_overlap_m,
            grasp.maximum_center_offset_m,
            grasp.maximum_axis_error_rad,
            grasp.minimum_bilateral_pad_deflection_m,
            grasp.maximum_bilateral_pad_deflection_m,
            grasp.minimum_grip_force_n,
            grasp.maximum_grip_force_n,
        ]) || grasp.commanded_pad_compression_m < 0.0
            || grasp.tool_to_peg_axial_offset_m <= 0.0
            || grasp.minimum_axial_grasp_overlap_m <= 0.0
            || grasp.minimum_axial_grasp_overlap_m > 2.0 * self.coupon.peg_half_segment_m
            || grasp.maximum_center_offset_m <= 0.0
            || grasp.maximum_axis_error_rad <= 0.0
            || grasp.minimum_bilateral_pad_deflection_m < 0.0
            || grasp.maximum_bilateral_pad_deflection_m < grasp.minimum_bilateral_pad_deflection_m
            || grasp.minimum_grip_force_n <= 0.0
            || grasp.maximum_grip_force_n < grasp.minimum_grip_force_n
        {
            return fail("grasp evidence limits are invalid");
        }
        if motion.pick_capture_start_axial_standoff_m <= grasp.tool_to_peg_axial_offset_m {
            return fail("pickup start standoff must permit a tail-axis post-grasp retraction");
        }
        let maximum_capture_correction_m = motion.pick_capture_relative_position_bound_m
            + (motion.pick_capture_axial_standoff_m - grasp.tool_to_peg_axial_offset_m).abs()
            + 2.0
                * grasp.tool_to_peg_axial_offset_m
                * (0.5 * motion.pick_capture_relative_axis_bound_rad).sin();
        if maximum_capture_correction_m > motion.maximum_correction_m {
            return fail(
                "declared pick capture pose bounds exceed the bounded correction authority",
            );
        }
        let required_capture_half_field_m = motion.pick_capture_axial_standoff_m
            + 0.5 * self.coupon.minimum_feature_axial_span_m
            + motion.pick_capture_relative_position_bound_m
            + (self.coupon.peg_half_segment_m + 0.5 * self.coupon.peg_diameter_m)
                * motion.pick_capture_relative_axis_bound_rad.sin();
        if required_capture_half_field_m > 0.5 * self.optics.field_size_at_target_m[0] {
            return fail("declared pickup capture feature envelope exceeds the macro field");
        }
        // The controller consumes only the declared bounds above. This
        // scenario-side check separately proves that nominal synthetic truth
        // generation stays inside those calibration/fixture admission bounds.
        let configured_pick_translation_budget_m = self
            .coupon
            .initial_peg_error_m
            .iter()
            .map(|component| component * component)
            .sum::<f64>()
            .sqrt()
            + motion.minimum_reproducible_correction_m
            + motion.differential_backlash_m;
        let configured_start_relative_error_m = self
            .coupon
            .initial_peg_error_m
            .iter()
            .zip(self.coupon.initial_tool_command_error_m.iter())
            .map(|(peg, tool)| (peg - tool) * (peg - tool))
            .sum::<f64>()
            .sqrt()
            + motion.minimum_reproducible_correction_m
            + motion.differential_backlash_m;
        let configured_pick_axis_budget_rad = self
            .coupon
            .initial_peg_axis_tilt_rad
            .iter()
            .chain(self.coupon.initial_tool_axis_tilt_rad.iter())
            .map(|component| component.abs())
            .sum::<f64>();
        if configured_pick_translation_budget_m.max(configured_start_relative_error_m)
            > motion.pick_capture_relative_position_bound_m
            || configured_pick_axis_budget_rad > motion.pick_capture_relative_axis_bound_rad
        {
            return fail("nominal synthetic pickup pose exceeds declared capture bounds");
        }
        let contact = &self.contact;
        if !finite(&[
            contact.lead_in_start_m,
            contact.recoverable_lateral_error_m,
            contact.maximum_lateral_error_m,
            contact.seat_axial_tolerance_m,
            contact.seat_lateral_tolerance_m,
            contact.seat_axis_tolerance_rad,
            contact.lateral_stiffness_proxy_n_per_m,
            contact.axial_stiffness_proxy_n_per_m,
            contact.maximum_force_proxy_n,
        ]) || contact.lead_in_start_m <= 0.0
            || contact.recoverable_lateral_error_m <= 0.0
            || contact.maximum_lateral_error_m < contact.recoverable_lateral_error_m
            || contact.seat_lateral_tolerance_m > contact.recoverable_lateral_error_m
            || contact.seat_axial_tolerance_m <= 0.0
            || contact.seat_lateral_tolerance_m <= 0.0
            || contact.seat_axis_tolerance_rad <= 0.0
            || contact.lateral_stiffness_proxy_n_per_m <= 0.0
            || contact.axial_stiffness_proxy_n_per_m <= 0.0
            || contact.maximum_force_proxy_n <= 0.0
        {
            return fail("contact proxy limits are invalid");
        }
        let safety = &self.safety;
        if !finite(&[
            safety.tool_envelope_radius_m,
            safety.carried_peg_envelope_radius_m,
            safety.minimum_obstacle_clearance_m,
            safety.retreat_distance_m,
        ]) || safety.tool_envelope_radius_m <= 0.0
            || safety.carried_peg_envelope_radius_m <= 0.0
            || safety.carried_peg_envelope_radius_m
                < self.coupon.peg_half_segment_m + 0.5 * self.coupon.peg_diameter_m
            || safety.tool_envelope_radius_m
                < grasp.tool_to_peg_axial_offset_m
                    + self.coupon.peg_half_segment_m
                    + 0.5 * self.coupon.peg_diameter_m
            || safety.minimum_obstacle_clearance_m < 0.0
            || safety.retreat_distance_m <= 0.0
            || safety.maximum_phase_retries == 0
        {
            return fail("safety policy is invalid");
        }
        if safety.planning_obstacles.is_empty()
            || safety.planning_obstacles.iter().any(|obstacle| {
                obstacle.id == 0
                    || !finite(&obstacle.center_world_m)
                    || !obstacle.conservative_radius_m.is_finite()
                    || obstacle.conservative_radius_m <= 0.0
                    || !obstacle.position_sigma_m.is_finite()
                    || obstacle.position_sigma_m < 0.0
            })
        {
            return fail("at least one finite calibrated planning obstacle is required");
        }
        let mut obstacle_ids = safety
            .planning_obstacles
            .iter()
            .map(|obstacle| obstacle.id)
            .collect::<Vec<_>>();
        obstacle_ids.sort_unstable();
        obstacle_ids.dedup();
        if obstacle_ids.len() != safety.planning_obstacles.len() {
            return fail("calibrated planning obstacle IDs must be unique");
        }
        if self.fault_profiles.len() != M1eFault::available().len() - 1 {
            return fail("fault profile set is incomplete");
        }
        for fault_id in &M1eFault::available()[1..] {
            if self
                .fault_profiles
                .iter()
                .filter(|profile| profile.id == *fault_id)
                .count()
                != 1
            {
                return fail("fault profiles must contain each non-nominal fault exactly once");
            }
        }
        for profile in &self.fault_profiles {
            let fault = profile.id.parse::<M1eFault>().map_err(|_| {
                ScenarioError::Invalid("fault profile contains an unknown ID".to_owned())
            })?;
            if fault.expected_terminal_reason() != Some(profile.expected_terminal_reason.as_str()) {
                return fail(
                    "fault profile terminal reason does not match the controller-owned policy",
                );
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::plant::{ObservedPlant, PEG_OBJECT_ID};
    use super::*;

    fn calibration_health(scenario: &ObservedManipulationScenario) -> (Option<f64>, u32, bool) {
        let mut plant = ObservedPlant::new(scenario, M1eFault::None).unwrap();
        plant
            .advance_ticks(plant.motion_capabilities().settling_ticks)
            .unwrap();
        let burst = plant
            .acquire_observation_burst(
                &[PEG_OBJECT_ID],
                scenario.coupon.pick_peg_center_nominal_world_m,
            )
            .unwrap();
        (
            burst.calibration_reference_residual_m,
            burst.calibration_reference_sample_count,
            burst.calibration_reference_valid,
        )
    }

    #[test]
    fn embedded_scenario_is_strict_valid_and_hashed() {
        let (scenario, hash) = ObservedManipulationScenario::baseline().unwrap();
        assert_eq!(scenario.schema_version, M1E_SCENARIO_SCHEMA_VERSION);
        assert_eq!(scenario.id, "observed_manipulation_m1e_v1");
        assert_eq!(hash.len(), 64);
        assert_eq!(
            scenario.expected_failure_reason(M1eFault::InsertionJam),
            Some("insertion_jam")
        );
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let broken = BASELINE_M1E_SCENARIO_JSON.replacen(
            "\"schema_version\": 1,",
            "\"schema_version\": 1, \"surprise\": true,",
            1,
        );
        assert!(matches!(
            ObservedManipulationScenario::from_json(&broken),
            Err(ScenarioError::Json(_))
        ));
    }

    #[test]
    fn every_named_fault_has_an_exact_expected_reason() {
        let (scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        for name in &M1eFault::available()[1..] {
            let fault = name.parse::<M1eFault>().unwrap();
            assert!(scenario.expected_failure_reason(fault).is_some());
        }
    }

    #[test]
    fn calibration_reference_residual_and_sample_gates_are_deterministic_at_boundaries() {
        let (baseline, _) = ObservedManipulationScenario::baseline().unwrap();
        let first = calibration_health(&baseline);
        let second = calibration_health(&baseline);
        assert_eq!(first, second, "same scenario and seed must replay exactly");

        let (Some(residual_m), sample_count, _) = first else {
            panic!("baseline must produce a calibration-reference residual");
        };
        assert!(sample_count > 0);
        let epsilon_m = (residual_m * 1.0e-9).max(1.0e-15);

        let mut residual_accepted = baseline.clone();
        residual_accepted
            .optics
            .maximum_calibration_reference_residual_m = residual_m + epsilon_m;
        assert!(calibration_health(&residual_accepted).2);

        let mut residual_rejected = baseline.clone();
        residual_rejected
            .optics
            .maximum_calibration_reference_residual_m = (residual_m - epsilon_m).max(0.0);
        assert!(!calibration_health(&residual_rejected).2);

        let mut sample_count_accepted = baseline.clone();
        sample_count_accepted
            .optics
            .minimum_calibration_reference_samples = sample_count;
        assert!(calibration_health(&sample_count_accepted).2);

        let mut sample_count_rejected = baseline.clone();
        sample_count_rejected
            .optics
            .minimum_calibration_reference_samples = sample_count.saturating_add(1);
        assert!(!calibration_health(&sample_count_rejected).2);

        let mut missing = baseline;
        missing.optics.base_dropout_probability = 1.0;
        let missing_health = calibration_health(&missing);
        assert_eq!(missing_health, (None, 0, false));
    }

    #[test]
    fn rejects_non_finite_or_internally_impossible_numeric_contracts() {
        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.motion.maximum_correction_m = f64::INFINITY;
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(_))
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.optics.processing_latency_s = scenario.optics.maximum_measurement_age_s;
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(_))
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.fault_profiles[0].expected_terminal_reason.clear();
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(_))
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.fault_profiles[0].expected_terminal_reason =
            "grasp_outside_capture_region".to_owned();
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(message))
                if message.contains("controller-owned policy")
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.motion.pick_capture_relative_position_bound_m =
            scenario.motion.maximum_correction_m;
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(message))
                if message.contains("capture pose bounds exceed")
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.motion.pick_capture_start_axial_standoff_m =
            scenario.motion.pick_capture_axial_standoff_m;
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(message)) if message.contains("motion policy")
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.optics.field_size_at_target_m[0] = 2.0e-3;
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(message)) if message.contains("capture feature envelope")
        ));

        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        scenario.coupon.initial_tool_command_error_m = [0.5e-3, 0.0, 0.0];
        assert!(matches!(
            scenario.validate(),
            Err(ScenarioError::Invalid(message)) if message.contains("pickup pose exceeds")
        ));
    }

    #[test]
    fn selected_jam_fault_requires_a_below_force_limit_lateral_interval() {
        let (mut scenario, _) = ObservedManipulationScenario::baseline().unwrap();
        assert!(scenario.validate_fault(M1eFault::InsertionJam).is_ok());

        scenario.contact.maximum_force_proxy_n = 0.002;
        assert!(scenario.validate_fault(M1eFault::None).is_ok());
        assert!(matches!(
            scenario.validate_fault(M1eFault::InsertionJam),
            Err(ScenarioError::Invalid(message)) if message.contains("no lateral interval")
        ));
    }
}
