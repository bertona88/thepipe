use serde::Serialize;

use super::controller::{ClassifiedContactEvidence, ContactPacket, ControlPhase, EstimateView};
use super::estimator::PoseEstimate;

pub const OBSERVED_MANIPULATION_REPORT_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TruthFirewallReport {
    pub controller_truth_access_count: u64,
    pub controller_accepts_raw_depth_samples: bool,
    pub controller_accepts_scene_truth: bool,
    pub controller_input_contract: &'static str,
    pub truth_use_contract: &'static str,
}

impl Default for TruthFirewallReport {
    fn default() -> Self {
        Self {
            controller_truth_access_count: 0,
            controller_accepts_raw_depth_samples: false,
            controller_accepts_scene_truth: false,
            controller_input_contract:
                "commanded_state+timestamped_feature_measurements+estimates+calibration+raw_contact_packets",
            truth_use_contract: "sensor_generation_and_post_run_evaluation_only",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TimingReport {
    pub fixed_step_s: f64,
    pub settling_interval_s: f64,
    pub coded_pattern_count: u32,
    pub pattern_rate_hz: f64,
    pub processing_latency_s: f64,
    pub maximum_measurement_age_s: f64,
    pub stale_near_contact_command_count: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ObservationBurstRecord {
    pub sequence: u32,
    pub phase: ControlPhase,
    pub roi: &'static str,
    pub capture_start_tick: u64,
    pub capture_end_tick: u64,
    pub available_tick: u64,
    pub requested_object_ids: Vec<u32>,
    pub observed_feature_count: u32,
    pub missing_feature_count: u32,
    pub accepted_object_ids: Vec<u32>,
    pub rejection_reasons: Vec<String>,
    pub triangulation_head_count: u32,
    pub calibrated_rays_per_observed_point: u32,
    pub calibration_reference_residual_m: Option<f64>,
    pub calibration_reference_sample_count: u32,
    pub required_calibration_reference_sample_count: u32,
    pub maximum_calibration_reference_residual_m: f64,
    pub calibration_reference_valid: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct EstimatorUpdateRecord {
    pub sequence: u32,
    pub phase: ControlPhase,
    /// `optical_burst` for a measurement update or `command_prediction` for
    /// propagation between stopped observations.
    pub update_kind: &'static str,
    /// Present only when the update consumed a timestamped optical burst.
    pub burst_sequence: Option<u32>,
    pub applied_position_sigma_limit_m: f64,
    pub applied_axis_sigma_limit_rad: f64,
    pub accepted_by_controller: bool,
    pub estimate: PoseEstimate,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct UncertaintyGuardRecord {
    pub sequence: u32,
    pub tick: u64,
    pub phase: ControlPhase,
    pub kind: &'static str,
    pub object_ids: Vec<u32>,
    pub position_sigma_m: Option<f64>,
    pub axis_sigma_rad: Option<f64>,
    pub position_limit_m: f64,
    pub axis_limit_rad: f64,
    pub passed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ForceInterlockRecord {
    pub tick: u64,
    pub channel: &'static str,
    pub measured_force_proxy_n: f64,
    pub limit_force_proxy_n: f64,
    pub motion_was_active: bool,
    pub stop_command_sequence: Option<u64>,
    pub packet: ContactPacket,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CorrectionIterationRecord {
    pub phase: ControlPhase,
    pub iteration: u32,
    pub decision_tick: u64,
    pub measurement_age_s: f64,
    pub residual_before_m: f64,
    pub requested_correction_world_m: Option<[f64; 3]>,
    pub requested_correction_m: f64,
    pub estimator_position_sigma_m: f64,
    pub estimator_axis_sigma_rad: f64,
    pub outcome: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct DecisionRecord {
    pub sequence: u32,
    pub tick: u64,
    pub time_s: f64,
    pub phase: ControlPhase,
    pub action: &'static str,
    pub reason: String,
    pub near_contact: bool,
    pub command_sequence: Option<u64>,
    pub target_world_m: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_axis_world: Option<[f64; 3]>,
    pub relevant_estimates: Vec<EstimateView>,
    pub contact_evidence: Option<ClassifiedContactEvidence>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct AcceptanceGate {
    pub id: &'static str,
    /// False for a nominal-success gate in an intentionally injected fault
    /// run. A non-applicable gate is never represented as a pass.
    pub applicable: bool,
    pub passed: bool,
    pub evidence: String,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ControllerMetrics {
    pub observation_bursts: u32,
    pub accepted_pose_estimates: u32,
    pub maximum_accepted_position_sigma_m: f64,
    pub maximum_accepted_axis_sigma_rad: f64,
    pub correction_iterations: u32,
    pub successful_corrections: u32,
    pub guarded_grasp_confirmed: bool,
    pub transfer_preflight_passed: bool,
    pub transfer_completed: bool,
    pub preflight_obstacle_checks: u32,
    pub minimum_predicted_clearance_m: Option<f64>,
    pub insertion_increment_count: u32,
    pub guarded_insertion_confirmed: bool,
    pub force_interlock_trip_count: u32,
    pub maximum_grip_force_proxy_n: f64,
    pub maximum_insertion_force_proxy_n: f64,
    pub seated_from_observation_and_contact: bool,
    pub release_confirmed: bool,
    pub retreat_confirmed: bool,
    pub recovery_count: u32,
}

impl Default for ControllerMetrics {
    fn default() -> Self {
        Self {
            observation_bursts: 0,
            accepted_pose_estimates: 0,
            maximum_accepted_position_sigma_m: 0.0,
            maximum_accepted_axis_sigma_rad: 0.0,
            correction_iterations: 0,
            successful_corrections: 0,
            guarded_grasp_confirmed: false,
            transfer_preflight_passed: false,
            transfer_completed: false,
            preflight_obstacle_checks: 0,
            minimum_predicted_clearance_m: None,
            insertion_increment_count: 0,
            guarded_insertion_confirmed: false,
            force_interlock_trip_count: 0,
            maximum_grip_force_proxy_n: 0.0,
            maximum_insertion_force_proxy_n: 0.0,
            seated_from_observation_and_contact: false,
            release_confirmed: false,
            retreat_confirmed: false,
            recovery_count: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct EvaluationOnlyTruthMetrics {
    pub label: &'static str,
    pub final_peg_tip_to_socket_seat_error_m: f64,
    pub final_peg_lateral_error_m: f64,
    pub final_peg_axial_error_m: f64,
    pub final_peg_axis_error_rad: f64,
    pub final_tool_center_to_socket_center_distance_m: f64,
    pub physical_grasp_attachment_present: bool,
    pub physical_release_verified: bool,
    pub maximum_unplanned_penetration_m: f64,
    pub peak_grip_force_proxy_n: f64,
    pub peak_insertion_force_proxy_n: f64,
    pub within_declared_seat_tolerances: bool,
    pub note: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ObservedManipulationReport {
    pub schema_version: u32,
    pub scenario_id: String,
    pub scenario_schema_version: u32,
    pub scenario_sha256: String,
    pub machine_config_id: String,
    pub machine_config_sha256: String,
    pub seed: u64,
    pub injected_fault: &'static str,
    pub expected_terminal_reason: Option<String>,
    pub status: &'static str,
    pub phase: ControlPhase,
    pub terminal_reason: Option<String>,
    pub expected_outcome_observed: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fixed_head: Option<super::scenario::FixedHeadConfig>,
    pub fidelity: &'static str,
    pub hardware_qualification_status: &'static str,
    pub pose_state: String,
    pub roll_observable: bool,
    pub optical_fidelity_boundary: &'static str,
    pub contact_fidelity_boundary: &'static str,
    pub attachment_fidelity_boundary: &'static str,
    pub truth_firewall: TruthFirewallReport,
    pub timing: TimingReport,
    pub metrics: ControllerMetrics,
    pub observation_bursts: Vec<ObservationBurstRecord>,
    pub estimator_updates: Vec<EstimatorUpdateRecord>,
    pub uncertainty_guards: Vec<UncertaintyGuardRecord>,
    pub force_interlocks: Vec<ForceInterlockRecord>,
    pub correction_iterations: Vec<CorrectionIterationRecord>,
    pub decisions: Vec<DecisionRecord>,
    pub acceptance_gates: Vec<AcceptanceGate>,
    pub controller_report_sha256: String,
    pub controller_report_hash_scope: &'static str,
    /// Available only after the runtime reaches a terminal state. Keeping
    /// this absent from ready/running reports makes the post-run-only truth
    /// boundary enforceable at the serialized API, not merely conventional.
    pub evaluation_only_truth: Option<EvaluationOnlyTruthMetrics>,
}

impl ObservedManipulationReport {
    pub fn to_json(&self, pretty: bool) -> Result<String, crate::SimError> {
        crate::serialize_json(self, pretty)
    }
}
