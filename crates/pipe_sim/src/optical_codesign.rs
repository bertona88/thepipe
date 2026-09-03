//! Executable M1d optics/robot co-design study.
//!
//! The embedded document is the reviewable input. This module validates its
//! geometry, propagates declared uncertainties, sweeps macro field/baseline
//! choices, and solves the remaining arm-control allocation at every phase.

use core::fmt;

use pipe_optics::{
    remaining_independent_rms_budget, symmetric_triangulation_angle_rad, PrecisionModelInput,
    PrecisionPrediction,
};
use serde::{Deserialize, Serialize};

use crate::sha256_hex;

pub const OPTICAL_CODESIGN_SCHEMA_VERSION: u32 = 1;
const BASELINE_OPTICAL_CODESIGN_JSON: &str =
    include_str!("../../../scenarios/optical_codesign_m1d.json");

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OpticalCodesignConfig {
    pub schema_version: u32,
    pub id: String,
    pub status: String,
    pub claim_boundary: String,
    pub global_layout: GlobalLayoutConfig,
    pub macro_layout: MacroLayoutConfig,
    pub global_profile: OpticalProfileConfig,
    pub macro_profile: OpticalProfileConfig,
    pub targets: OpticalTargets,
    pub macro_sweep: MacroSweepConfig,
    pub phases: Vec<PhaseCodesignConfig>,
    pub evidence_sources: Vec<EvidenceSource>,
    pub qualification_gates: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GlobalLayoutConfig {
    pub role: String,
    pub camera_candidate: String,
    pub mount_radius_m: f64,
    pub axial_offsets_m: [f64; 2],
    pub horizontal_field_of_view_deg: f64,
    pub azimuths_by_end_deg: [[f64; 3]; 2],
    pub target_world_m: [f64; 3],
    pub rigid_mount_required: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MacroLayoutConfig {
    pub role: String,
    pub camera_candidate: String,
    pub projector_candidate: String,
    pub mount_mode: String,
    pub entrance_pupil_baseline_m: f64,
    pub perpendicular_working_distance_m: f64,
    pub field_width_m: f64,
    pub field_height_m: f64,
    pub stationary_during_coded_burst: bool,
    pub tiled_views_required_for_full_housing: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OpticalProfileConfig {
    pub id: String,
    pub claim_class: String,
    pub image_width_px: u32,
    pub image_height_px: u32,
    pub field_width_at_target_m: f64,
    pub range_m: f64,
    pub effective_baseline_m: f64,
    pub triangulation_angle_deg: f64,
    pub camera_localization_sigma_px: f64,
    pub correspondence_localization_sigma_px: f64,
    pub correlated_calibration_sigma_m: f64,
    pub surface_axial_sigma_m: f64,
    pub depth_quantization_m: f64,
    pub pattern_count: u32,
    pub pattern_rate_hz: f64,
    pub processing_latency_s: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct OpticalTargets {
    pub global_lateral_nominal_sigma_m: f64,
    pub global_axial_nominal_sigma_m: f64,
    pub macro_sampling_nominal_m_px: f64,
    pub macro_sampling_worst_m_px: f64,
    pub macro_lateral_servo_sigma_m: f64,
    pub macro_axial_servo_sigma_m: f64,
    pub macro_depth_nominal_sigma_m: f64,
    pub macro_depth_worst_sigma_m: f64,
    pub sensor_to_estimate_worst_s: f64,
    pub coarse_tcp_sigma_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct MacroSweepConfig {
    pub field_widths_m: Vec<f64>,
    pub baselines_m: Vec<f64>,
    pub perpendicular_working_distance_m: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PhaseCodesignConfig {
    pub id: String,
    pub optical_profile: String,
    pub motion_policy: String,
    pub target_basis: String,
    pub target_lateral_sigma_m: f64,
    pub target_axial_sigma_m: f64,
    pub hold_drift_lateral_sigma_m: f64,
    pub hold_drift_axial_sigma_m: f64,
    pub latency_motion_lateral_sigma_m: f64,
    pub latency_motion_axial_sigma_m: f64,
    pub contact_process_lateral_sigma_m: f64,
    pub contact_process_axial_sigma_m: f64,
    pub minimum_independent_views: u32,
    pub observability_failure_action: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct EvidenceSource {
    pub id: String,
    pub claim_class: String,
    pub item: String,
    pub claims: Vec<String>,
    pub url: String,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct OpticalProfilePrediction {
    pub id: String,
    pub claim_class: String,
    pub image_width_px: u32,
    pub image_height_px: u32,
    pub field_width_at_target_m: f64,
    pub object_space_sampling_m_px: f64,
    pub range_m: f64,
    pub effective_baseline_m: f64,
    pub triangulation_angle_deg: f64,
    pub effective_focal_length_px: f64,
    pub lateral_random_sigma_m: f64,
    pub axial_geometric_sigma_m: f64,
    pub correlated_calibration_sigma_m: f64,
    pub surface_axial_sigma_m: f64,
    pub lateral_total_sigma_m: f64,
    pub axial_total_sigma_m: f64,
    pub capture_duration_s: f64,
    pub sensor_to_estimate_latency_s: f64,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct MacroSweepRow {
    pub field_width_m: f64,
    pub baseline_m: f64,
    pub perpendicular_working_distance_m: f64,
    pub triangulation_angle_deg: f64,
    pub object_space_sampling_m_px: f64,
    pub lateral_total_sigma_m: f64,
    pub axial_total_sigma_m: f64,
    pub meets_nominal_sampling_target: bool,
    pub meets_worst_sampling_target: bool,
    pub meets_nominal_depth_target: bool,
    pub meets_worst_depth_target: bool,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct PhaseBudgetReport {
    pub id: String,
    pub optical_profile: String,
    pub motion_policy: String,
    pub target_basis: String,
    pub target_lateral_sigma_m: f64,
    pub target_axial_sigma_m: f64,
    pub optical_lateral_sigma_m: f64,
    pub optical_axial_sigma_m: f64,
    pub hold_drift_lateral_sigma_m: f64,
    pub hold_drift_axial_sigma_m: f64,
    pub latency_motion_lateral_sigma_m: f64,
    pub latency_motion_axial_sigma_m: f64,
    pub contact_process_lateral_sigma_m: f64,
    pub contact_process_axial_sigma_m: f64,
    pub maximum_arm_control_residual_lateral_sigma_m: Option<f64>,
    pub maximum_arm_control_residual_axial_sigma_m: Option<f64>,
    pub maximum_uncompensated_lateral_speed_m_s: Option<f64>,
    pub maximum_uncompensated_axial_speed_m_s: Option<f64>,
    pub minimum_independent_views: u32,
    pub observability_failure_action: String,
    pub model_budget_status: String,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct ModelGateResult {
    pub id: String,
    pub claim_class: String,
    pub measured_or_modeled_value: f64,
    pub maximum_allowed_value: f64,
    pub units: String,
    pub status: String,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct OpticalCodesignReport {
    pub schema_version: u32,
    pub study_id: String,
    pub input_sha256: String,
    pub overall_status: String,
    pub hardware_qualification_status: String,
    pub claim_boundary: String,
    pub configuration: OpticalCodesignConfig,
    pub global_prediction: OpticalProfilePrediction,
    pub macro_prediction: OpticalProfilePrediction,
    pub model_gates: Vec<ModelGateResult>,
    pub macro_sweep: Vec<MacroSweepRow>,
    pub phase_budgets: Vec<PhaseBudgetReport>,
    pub design_decisions: Vec<String>,
}

impl OpticalCodesignReport {
    pub fn to_json(&self, pretty: bool) -> Result<String, OpticalCodesignError> {
        if pretty {
            serde_json::to_string_pretty(self)
        } else {
            serde_json::to_string(self)
        }
        .map_err(|error| OpticalCodesignError::Serialization(error.to_string()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OpticalCodesignError {
    InvalidConfiguration(String),
    Serialization(String),
}

impl fmt::Display for OpticalCodesignError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(message) => {
                write!(
                    formatter,
                    "invalid optical co-design configuration: {message}"
                )
            }
            Self::Serialization(message) => {
                write!(
                    formatter,
                    "optical co-design serialization failed: {message}"
                )
            }
        }
    }
}

impl std::error::Error for OpticalCodesignError {}

pub fn optical_codesign_report() -> Result<OpticalCodesignReport, OpticalCodesignError> {
    let configuration: OpticalCodesignConfig = serde_json::from_str(BASELINE_OPTICAL_CODESIGN_JSON)
        .map_err(|error| {
            OpticalCodesignError::InvalidConfiguration(format!("JSON parse: {error}"))
        })?;
    validate_configuration(&configuration)?;

    let global_prediction = predict_profile(&configuration.global_profile)?;
    let macro_prediction = predict_profile(&configuration.macro_profile)?;
    let macro_sweep = build_macro_sweep(&configuration)?;
    let phase_budgets = build_phase_budgets(&configuration, &global_prediction, &macro_prediction);

    let mut model_gates = vec![
        upper_bound_gate(
            "global_lateral_nominal",
            global_prediction.lateral_total_sigma_m,
            configuration.targets.global_lateral_nominal_sigma_m,
            "m_rms",
        ),
        upper_bound_gate(
            "global_axial_nominal",
            global_prediction.axial_total_sigma_m,
            configuration.targets.global_axial_nominal_sigma_m,
            "m_rms",
        ),
        upper_bound_gate(
            "macro_sampling_nominal",
            macro_prediction.object_space_sampling_m_px,
            configuration.targets.macro_sampling_nominal_m_px,
            "m_per_px",
        ),
        upper_bound_gate(
            "macro_depth_nominal",
            macro_prediction.axial_total_sigma_m,
            configuration.targets.macro_depth_nominal_sigma_m,
            "m_rms",
        ),
        upper_bound_gate(
            "macro_sensor_to_estimate_latency",
            macro_prediction.sensor_to_estimate_latency_s,
            configuration.targets.sensor_to_estimate_worst_s,
            "s",
        ),
    ];
    let infeasible_phase_count = phase_budgets
        .iter()
        .filter(|phase| phase.model_budget_status != "feasible")
        .count();
    model_gates.push(ModelGateResult {
        id: "all_phase_error_budgets_feasible".to_owned(),
        claim_class: "modeled".to_owned(),
        measured_or_modeled_value: infeasible_phase_count as f64,
        maximum_allowed_value: 0.0,
        units: "infeasible_phase_count".to_owned(),
        status: if infeasible_phase_count == 0 {
            "pass"
        } else {
            "fail"
        }
        .to_owned(),
    });
    let all_model_gates_pass = model_gates.iter().all(|gate| gate.status == "pass");

    Ok(OpticalCodesignReport {
        schema_version: OPTICAL_CODESIGN_SCHEMA_VERSION,
        study_id: configuration.id.clone(),
        input_sha256: sha256_hex(BASELINE_OPTICAL_CODESIGN_JSON.as_bytes()),
        overall_status: if all_model_gates_pass {
            "model_feasible_hardware_qualification_required"
        } else {
            "model_infeasible"
        }
        .to_owned(),
        hardware_qualification_status: "not_started".to_owned(),
        claim_boundary: configuration.claim_boundary.clone(),
        configuration,
        global_prediction,
        macro_prediction,
        model_gates,
        macro_sweep,
        phase_budgets,
        design_decisions: vec![
            "Retain six fixed global cameras for coverage and coarse closed-loop motion."
                .to_owned(),
            "Use one macro camera plus a calibrated projector on one rigid observer head; do not place a stereo pair on independent compliant arms."
                .to_owned(),
            "Use a 2.5 mm macro field for nominal sampling and tile views when the complete 6 x 4 mm housing must be inspected."
                .to_owned(),
            "Pause the manipulated part during coded macro acquisition, then move in bounded increments and remeasure."
                .to_owned(),
            "Treat the phase residuals as loaded closed-loop actuation requirements, not as raw encoder or open-loop absolute-accuracy claims."
                .to_owned(),
        ],
    })
}

fn validate_configuration(config: &OpticalCodesignConfig) -> Result<(), OpticalCodesignError> {
    if config.schema_version != OPTICAL_CODESIGN_SCHEMA_VERSION {
        return invalid(format!(
            "schema {} does not match runtime schema {}",
            config.schema_version, OPTICAL_CODESIGN_SCHEMA_VERSION
        ));
    }
    if config.status != "modeled_design_candidate_not_hardware_qualified" {
        return invalid("unqualified claim status must be preserved");
    }
    if config.phases.is_empty() || config.evidence_sources.is_empty() {
        return invalid("phase contracts and evidence sources cannot be empty");
    }
    validate_profile(&config.global_profile)?;
    validate_profile(&config.macro_profile)?;

    let targets = [
        config.targets.global_lateral_nominal_sigma_m,
        config.targets.global_axial_nominal_sigma_m,
        config.targets.macro_sampling_nominal_m_px,
        config.targets.macro_sampling_worst_m_px,
        config.targets.macro_lateral_servo_sigma_m,
        config.targets.macro_axial_servo_sigma_m,
        config.targets.macro_depth_nominal_sigma_m,
        config.targets.macro_depth_worst_sigma_m,
        config.targets.sensor_to_estimate_worst_s,
        config.targets.coarse_tcp_sigma_m,
    ];
    if targets.iter().any(|value| !value.is_finite() || *value <= 0.0) {
        return invalid("all optical and robot targets must be finite and positive");
    }

    let global = &config.global_layout;
    if !global.rigid_mount_required
        || !global.mount_radius_m.is_finite()
        || global.mount_radius_m <= 0.0
        || !global.horizontal_field_of_view_deg.is_finite()
        || global.horizontal_field_of_view_deg <= 0.0
        || global.horizontal_field_of_view_deg >= 180.0
        || global
            .axial_offsets_m
            .iter()
            .chain(global.target_world_m.iter())
            .any(|value| !value.is_finite())
        || global
            .azimuths_by_end_deg
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
    {
        return invalid("global layout must contain finite rigid-mount geometry");
    }
    if global.axial_offsets_m[0] * global.axial_offsets_m[1] >= 0.0 {
        return invalid("global camera triplets must lie on opposite axial ends");
    }
    require_close(
        "global axial offset symmetry",
        global.axial_offsets_m[0].abs(),
        global.axial_offsets_m[1].abs(),
    )?;

    let mut camera_positions = [[[0.0; 3]; 3]; 2];
    for (end_index, azimuths) in global.azimuths_by_end_deg.iter().enumerate() {
        for camera_index in 0..3 {
            let next_index = (camera_index + 1) % 3;
            let spacing_deg =
                (azimuths[next_index] - azimuths[camera_index]).rem_euclid(360.0);
            require_close(
                &format!("global end {end_index} azimuth spacing {camera_index}"),
                spacing_deg,
                120.0,
            )?;
            camera_positions[end_index][camera_index] = global_camera_position(
                global.mount_radius_m,
                global.axial_offsets_m[end_index],
                azimuths[camera_index],
            );
        }
    }
    for camera_index in 0..3 {
        let clocking_deg = (global.azimuths_by_end_deg[1][camera_index]
            - global.azimuths_by_end_deg[0][camera_index])
            .rem_euclid(360.0);
        require_close(
            &format!("global end clocking {camera_index}"),
            clocking_deg,
            60.0,
        )?;
    }

    for (end_index, positions) in camera_positions.iter().enumerate() {
        for camera_index in 0..3 {
            let range_m = point_distance(positions[camera_index], global.target_world_m);
            require_close(
                &format!("global range end {end_index} camera {camera_index}"),
                range_m,
                config.global_profile.range_m,
            )?;
            let field_width_m = 2.0
                * range_m
                * (0.5 * global.horizontal_field_of_view_deg.to_radians()).tan();
            require_close(
                &format!("global field width end {end_index} camera {camera_index}"),
                field_width_m,
                config.global_profile.field_width_at_target_m,
            )?;

            let next_index = (camera_index + 1) % 3;
            let baseline_m = point_distance(positions[camera_index], positions[next_index]);
            require_close(
                &format!("global baseline end {end_index} pair {camera_index}"),
                baseline_m,
                config.global_profile.effective_baseline_m,
            )?;
            let angle_rad = triangulation_angle_at_target_rad(
                positions[camera_index],
                positions[next_index],
                global.target_world_m,
            )
            .ok_or_else(|| {
                OpticalCodesignError::InvalidConfiguration(format!(
                    "invalid global triangulation geometry at end {end_index} pair {camera_index}"
                ))
            })?;
            require_close(
                &format!("global angle end {end_index} pair {camera_index}"),
                angle_rad.to_degrees(),
                config.global_profile.triangulation_angle_deg,
            )?;
        }
    }

    let macro_geometry = [
        config.macro_layout.entrance_pupil_baseline_m,
        config.macro_layout.perpendicular_working_distance_m,
        config.macro_layout.field_width_m,
        config.macro_layout.field_height_m,
    ];
    if macro_geometry
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return invalid("macro layout geometry must be finite and positive");
    }

    let macro_angle_rad = symmetric_triangulation_angle_rad(
        config.macro_layout.entrance_pupil_baseline_m,
        config.macro_layout.perpendicular_working_distance_m,
    )
    .ok_or_else(|| {
        OpticalCodesignError::InvalidConfiguration("invalid macro geometry".to_owned())
    })?;
    let macro_range_m = config
        .macro_layout
        .perpendicular_working_distance_m
        .hypot(0.5 * config.macro_layout.entrance_pupil_baseline_m);
    require_close(
        "macro baseline",
        config.macro_layout.entrance_pupil_baseline_m,
        config.macro_profile.effective_baseline_m,
    )?;
    require_close("macro range", macro_range_m, config.macro_profile.range_m)?;
    require_close(
        "macro angle",
        macro_angle_rad.to_degrees(),
        config.macro_profile.triangulation_angle_deg,
    )?;
    require_close(
        "macro field width",
        config.macro_layout.field_width_m,
        config.macro_profile.field_width_at_target_m,
    )?;
    let expected_height_m = config.macro_layout.field_width_m
        * f64::from(config.macro_profile.image_height_px)
        / f64::from(config.macro_profile.image_width_px);
    require_close(
        "macro field height",
        expected_height_m,
        config.macro_layout.field_height_m,
    )?;

    if !config.macro_layout.stationary_during_coded_burst
        || config
            .phases
            .iter()
            .any(|phase| phase.minimum_independent_views < 2)
    {
        return invalid(
            "stationary macro capture and at least two independent views are mandatory",
        );
    }
    for phase in &config.phases {
        if phase.optical_profile != "global" && phase.optical_profile != "macro" {
            return invalid(format!(
                "phase '{}' selects unknown optical profile '{}'",
                phase.id, phase.optical_profile
            ));
        }
        let allocations = [
            phase.target_lateral_sigma_m,
            phase.target_axial_sigma_m,
            phase.hold_drift_lateral_sigma_m,
            phase.hold_drift_axial_sigma_m,
            phase.latency_motion_lateral_sigma_m,
            phase.latency_motion_axial_sigma_m,
            phase.contact_process_lateral_sigma_m,
            phase.contact_process_axial_sigma_m,
        ];
        if allocations
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
            || phase.target_lateral_sigma_m == 0.0
            || phase.target_axial_sigma_m == 0.0
        {
            return invalid(format!(
                "phase '{}' has an invalid RMS allocation",
                phase.id
            ));
        }
    }
    if config.macro_sweep.field_widths_m.is_empty()
        || config.macro_sweep.baselines_m.is_empty()
        || config
            .macro_sweep
            .field_widths_m
            .iter()
            .chain(config.macro_sweep.baselines_m.iter())
            .chain(core::iter::once(
                &config.macro_sweep.perpendicular_working_distance_m,
            ))
            .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return invalid("macro sweep must contain finite positive geometry");
    }
    Ok(())
}

fn validate_profile(profile: &OpticalProfileConfig) -> Result<(), OpticalCodesignError> {
    precision_input(profile)
        .predict()
        .map_err(|error| OpticalCodesignError::InvalidConfiguration(error.to_string()))?;
    if !profile.effective_baseline_m.is_finite()
        || profile.effective_baseline_m <= 0.0
        || profile.pattern_count == 0
        || !profile.pattern_rate_hz.is_finite()
        || profile.pattern_rate_hz <= 0.0
        || !profile.processing_latency_s.is_finite()
        || profile.processing_latency_s < 0.0
    {
        return invalid(format!(
            "profile '{}' has invalid timing or baseline",
            profile.id
        ));
    }
    Ok(())
}

fn precision_input(profile: &OpticalProfileConfig) -> PrecisionModelInput {
    PrecisionModelInput {
        image_width_px: profile.image_width_px,
        field_width_at_target_m: profile.field_width_at_target_m,
        range_m: profile.range_m,
        triangulation_angle_rad: profile.triangulation_angle_deg.to_radians(),
        camera_localization_sigma_px: profile.camera_localization_sigma_px,
        correspondence_localization_sigma_px: profile.correspondence_localization_sigma_px,
        correlated_calibration_sigma_m: profile.correlated_calibration_sigma_m,
        surface_axial_sigma_m: profile.surface_axial_sigma_m,
        depth_quantization_m: profile.depth_quantization_m,
    }
}

fn predict_profile(
    profile: &OpticalProfileConfig,
) -> Result<OpticalProfilePrediction, OpticalCodesignError> {
    let prediction = precision_input(profile)
        .predict()
        .map_err(|error| OpticalCodesignError::InvalidConfiguration(error.to_string()))?;
    Ok(profile_prediction(profile, prediction))
}

fn profile_prediction(
    profile: &OpticalProfileConfig,
    prediction: PrecisionPrediction,
) -> OpticalProfilePrediction {
    let capture_duration_s = f64::from(profile.pattern_count) / profile.pattern_rate_hz;
    OpticalProfilePrediction {
        id: profile.id.clone(),
        claim_class: profile.claim_class.clone(),
        image_width_px: profile.image_width_px,
        image_height_px: profile.image_height_px,
        field_width_at_target_m: profile.field_width_at_target_m,
        object_space_sampling_m_px: prediction.object_space_sampling_m_px,
        range_m: profile.range_m,
        effective_baseline_m: profile.effective_baseline_m,
        triangulation_angle_deg: profile.triangulation_angle_deg,
        effective_focal_length_px: prediction.effective_focal_length_px,
        lateral_random_sigma_m: prediction.lateral_random_sigma_m,
        axial_geometric_sigma_m: prediction.axial_geometric_sigma_m,
        correlated_calibration_sigma_m: profile.correlated_calibration_sigma_m,
        surface_axial_sigma_m: profile.surface_axial_sigma_m,
        lateral_total_sigma_m: prediction.lateral_total_sigma_m,
        axial_total_sigma_m: prediction.axial_total_sigma_m,
        capture_duration_s,
        sensor_to_estimate_latency_s: capture_duration_s + profile.processing_latency_s,
    }
}

fn build_macro_sweep(
    configuration: &OpticalCodesignConfig,
) -> Result<Vec<MacroSweepRow>, OpticalCodesignError> {
    let mut rows = Vec::new();
    for field_width_m in configuration.macro_sweep.field_widths_m.iter().copied() {
        for baseline_m in configuration.macro_sweep.baselines_m.iter().copied() {
            let working_distance_m = configuration.macro_sweep.perpendicular_working_distance_m;
            let angle_rad = symmetric_triangulation_angle_rad(baseline_m, working_distance_m)
                .ok_or_else(|| {
                    OpticalCodesignError::InvalidConfiguration(
                        "macro sweep contains invalid geometry".to_owned(),
                    )
                })?;
            let range_m = working_distance_m.hypot(0.5 * baseline_m);
            let mut profile = configuration.macro_profile.clone();
            profile.field_width_at_target_m = field_width_m;
            profile.effective_baseline_m = baseline_m;
            profile.range_m = range_m;
            profile.triangulation_angle_deg = angle_rad.to_degrees();
            let prediction = precision_input(&profile)
                .predict()
                .map_err(|error| OpticalCodesignError::InvalidConfiguration(error.to_string()))?;
            rows.push(MacroSweepRow {
                field_width_m,
                baseline_m,
                perpendicular_working_distance_m: working_distance_m,
                triangulation_angle_deg: angle_rad.to_degrees(),
                object_space_sampling_m_px: prediction.object_space_sampling_m_px,
                lateral_total_sigma_m: prediction.lateral_total_sigma_m,
                axial_total_sigma_m: prediction.axial_total_sigma_m,
                meets_nominal_sampling_target: prediction.object_space_sampling_m_px
                    <= configuration.targets.macro_sampling_nominal_m_px,
                meets_worst_sampling_target: prediction.object_space_sampling_m_px
                    <= configuration.targets.macro_sampling_worst_m_px,
                meets_nominal_depth_target: prediction.axial_total_sigma_m
                    <= configuration.targets.macro_depth_nominal_sigma_m,
                meets_worst_depth_target: prediction.axial_total_sigma_m
                    <= configuration.targets.macro_depth_worst_sigma_m,
            });
        }
    }
    Ok(rows)
}

fn build_phase_budgets(
    configuration: &OpticalCodesignConfig,
    global: &OpticalProfilePrediction,
    macro_profile: &OpticalProfilePrediction,
) -> Vec<PhaseBudgetReport> {
    configuration
        .phases
        .iter()
        .map(|phase| {
            let optical = if phase.optical_profile == "global" {
                global
            } else {
                macro_profile
            };
            let lateral_remaining = remaining_independent_rms_budget(
                phase.target_lateral_sigma_m,
                &[
                    optical.lateral_total_sigma_m,
                    phase.hold_drift_lateral_sigma_m,
                    phase.latency_motion_lateral_sigma_m,
                    phase.contact_process_lateral_sigma_m,
                ],
            );
            let axial_remaining = remaining_independent_rms_budget(
                phase.target_axial_sigma_m,
                &[
                    optical.axial_total_sigma_m,
                    phase.hold_drift_axial_sigma_m,
                    phase.latency_motion_axial_sigma_m,
                    phase.contact_process_axial_sigma_m,
                ],
            );
            PhaseBudgetReport {
                id: phase.id.clone(),
                optical_profile: phase.optical_profile.clone(),
                motion_policy: phase.motion_policy.clone(),
                target_basis: phase.target_basis.clone(),
                target_lateral_sigma_m: phase.target_lateral_sigma_m,
                target_axial_sigma_m: phase.target_axial_sigma_m,
                optical_lateral_sigma_m: optical.lateral_total_sigma_m,
                optical_axial_sigma_m: optical.axial_total_sigma_m,
                hold_drift_lateral_sigma_m: phase.hold_drift_lateral_sigma_m,
                hold_drift_axial_sigma_m: phase.hold_drift_axial_sigma_m,
                latency_motion_lateral_sigma_m: phase.latency_motion_lateral_sigma_m,
                latency_motion_axial_sigma_m: phase.latency_motion_axial_sigma_m,
                contact_process_lateral_sigma_m: phase.contact_process_lateral_sigma_m,
                contact_process_axial_sigma_m: phase.contact_process_axial_sigma_m,
                maximum_arm_control_residual_lateral_sigma_m: lateral_remaining,
                maximum_arm_control_residual_axial_sigma_m: axial_remaining,
                maximum_uncompensated_lateral_speed_m_s: speed_limit(
                    phase.latency_motion_lateral_sigma_m,
                    optical.sensor_to_estimate_latency_s,
                ),
                maximum_uncompensated_axial_speed_m_s: speed_limit(
                    phase.latency_motion_axial_sigma_m,
                    optical.sensor_to_estimate_latency_s,
                ),
                minimum_independent_views: phase.minimum_independent_views,
                observability_failure_action: phase.observability_failure_action.clone(),
                model_budget_status: if lateral_remaining.is_some() && axial_remaining.is_some() {
                    "feasible"
                } else {
                    "infeasible"
                }
                .to_owned(),
            }
        })
        .collect()
}

fn speed_limit(motion_allocation_m: f64, latency_s: f64) -> Option<f64> {
    (motion_allocation_m > 0.0 && latency_s > 0.0).then_some(motion_allocation_m / latency_s)
}

fn upper_bound_gate(id: &str, value: f64, limit: f64, units: &str) -> ModelGateResult {
    ModelGateResult {
        id: id.to_owned(),
        claim_class: "modeled".to_owned(),
        measured_or_modeled_value: value,
        maximum_allowed_value: limit,
        units: units.to_owned(),
        status: if value <= limit { "pass" } else { "fail" }.to_owned(),
    }
}

fn global_camera_position(radius_m: f64, axial_m: f64, azimuth_deg: f64) -> [f64; 3] {
    let azimuth_rad = azimuth_deg.to_radians();
    [
        radius_m * azimuth_rad.cos(),
        radius_m * azimuth_rad.sin(),
        axial_m,
    ]
}

fn point_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn triangulation_angle_at_target_rad(
    camera_a: [f64; 3],
    camera_b: [f64; 3],
    target: [f64; 3],
) -> Option<f64> {
    let ray_a = [
        camera_a[0] - target[0],
        camera_a[1] - target[1],
        camera_a[2] - target[2],
    ];
    let ray_b = [
        camera_b[0] - target[0],
        camera_b[1] - target[1],
        camera_b[2] - target[2],
    ];
    let norm_a = point_distance(ray_a, [0.0; 3]);
    let norm_b = point_distance(ray_b, [0.0; 3]);
    if !norm_a.is_finite() || !norm_b.is_finite() || norm_a <= 0.0 || norm_b <= 0.0 {
        return None;
    }
    let cosine = (ray_a[0] * ray_b[0] + ray_a[1] * ray_b[1] + ray_a[2] * ray_b[2])
        / (norm_a * norm_b);
    cosine.is_finite().then(|| cosine.clamp(-1.0, 1.0).acos())
}

fn require_close(label: &str, derived: f64, declared: f64) -> Result<(), OpticalCodesignError> {
    let tolerance = 1.0e-10 * derived.abs().max(declared.abs()).max(1.0);
    if !derived.is_finite() || !declared.is_finite() || (derived - declared).abs() > tolerance {
        return invalid(format!(
            "{label} is inconsistent: derived {derived}, declared {declared}"
        ));
    }
    Ok(())
}

fn invalid<T>(message: impl Into<String>) -> Result<T, OpticalCodesignError> {
    Err(OpticalCodesignError::InvalidConfiguration(message.into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline_configuration() -> OpticalCodesignConfig {
        serde_json::from_str(BASELINE_OPTICAL_CODESIGN_JSON).unwrap()
    }

    #[test]
    fn m1d_baseline_is_model_feasible_but_not_hardware_qualified() {
        let report = optical_codesign_report().unwrap();
        assert_eq!(
            report.overall_status,
            "model_feasible_hardware_qualification_required"
        );
        assert_eq!(report.hardware_qualification_status, "not_started");
        assert!(report.model_gates.iter().all(|gate| gate.status == "pass"));
        assert!(report
            .phase_budgets
            .iter()
            .all(|phase| phase.model_budget_status == "feasible"));
    }

    #[test]
    fn reference_predictions_preserve_the_precision_claim_boundary() {
        let report = optical_codesign_report().unwrap();
        assert!((report.global_prediction.lateral_total_sigma_m - 24.452_417e-6).abs() < 1.0e-12);
        assert!((report.global_prediction.axial_total_sigma_m - 43.832_315e-6).abs() < 1.0e-12);
        assert!(
            (report.macro_prediction.object_space_sampling_m_px - 1.953_125e-6).abs() < 1.0e-15
        );
        assert!((report.macro_prediction.lateral_total_sigma_m - 3.020_529e-6).abs() < 1.0e-12);
        assert!((report.macro_prediction.axial_total_sigma_m - 3.433_738e-6).abs() < 1.0e-12);
        assert!(report.claim_boundary.contains("No value"));
    }

    #[test]
    fn sweep_rejects_four_millimetre_field_on_sampling() {
        let report = optical_codesign_report().unwrap();
        let rows: Vec<_> = report
            .macro_sweep
            .iter()
            .filter(|row| (row.field_width_m - 0.004).abs() < 1.0e-12)
            .collect();
        assert_eq!(rows.len(), 3);
        assert!(rows.iter().all(|row| !row.meets_worst_sampling_target));
    }

    #[test]
    fn insertion_phase_drives_the_tight_arm_residual() {
        let report = optical_codesign_report().unwrap();
        let insertion = report
            .phase_budgets
            .iter()
            .find(|phase| phase.id == "guarded_insertion")
            .unwrap();
        let lateral = insertion
            .maximum_arm_control_residual_lateral_sigma_m
            .unwrap();
        let axial = insertion
            .maximum_arm_control_residual_axial_sigma_m
            .unwrap();
        assert!((lateral - 7.866_156e-6).abs() < 1.0e-12);
        assert!((axial - 17.894_397e-6).abs() < 1.0e-12);
    }

    #[test]
    fn report_is_deterministic() {
        let first = optical_codesign_report().unwrap().to_json(false).unwrap();
        let second = optical_codesign_report().unwrap().to_json(false).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn validator_rejects_second_end_range_drift() {
        let mut config = baseline_configuration();
        config.global_layout.axial_offsets_m[1] += 1.0e-3;

        let error = validate_configuration(&config).unwrap_err();
        assert!(error.to_string().contains("axial offset symmetry"));
    }

    #[test]
    fn validator_rejects_camera_azimuth_drift() {
        let mut config = baseline_configuration();
        config.global_layout.azimuths_by_end_deg[1][1] += 1.0;

        let error = validate_configuration(&config).unwrap_err();
        assert!(error.to_string().contains("azimuth spacing"));
    }

    #[test]
    fn validator_uses_the_declared_world_target() {
        let mut config = baseline_configuration();
        config.global_layout.target_world_m[0] = 1.0e-3;

        let error = validate_configuration(&config).unwrap_err();
        assert!(error.to_string().contains("global range"));
    }
}
