//! Truth-free M1e decision primitives.
//!
//! This module deliberately depends only on controller-visible value types.
//! It must not import the machine plant, rigid bodies, raw optical depth
//! samples, or renderer scene frames.

use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlPhase {
    Initialize,
    EnterCapture,
    PickCorrection,
    GuardedGrasp,
    Transfer,
    SocketCorrection,
    GuardedInsertion,
    SeatVerification,
    Release,
    Retreat,
    Complete,
    FailedSafe,
}

impl ControlPhase {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Initialize => "initialize",
            Self::EnterCapture => "enter_capture",
            Self::PickCorrection => "pick_correction",
            Self::GuardedGrasp => "guarded_grasp",
            Self::Transfer => "transfer",
            Self::SocketCorrection => "socket_correction",
            Self::GuardedInsertion => "guarded_insertion",
            Self::SeatVerification => "seat_verification",
            Self::Release => "release",
            Self::Retreat => "retreat",
            Self::Complete => "complete",
            Self::FailedSafe => "failed_safe",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct EstimateView {
    pub object_id: u32,
    pub valid: bool,
    pub invalid_reason: Option<String>,
    pub position_world_m: [f64; 3],
    pub axis_world: [f64; 3],
    pub position_sigma_m: f64,
    pub axis_sigma_rad: f64,
    pub capture_tick: u64,
    pub available_tick: u64,
    pub distinct_feature_count: u32,
    pub triangulation_head_count: u32,
    pub minimum_calibrated_rays_per_point: u32,
    pub residual_rms_m: f64,
    pub provenance: EstimateProvenance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EstimateSource {
    DirectFeatureFit,
    HeldTransformFromFreshTool,
}

/// Auditable source metadata for a controller-visible pose. A held-transform
/// pose deliberately reports zero PEG feature counts in `EstimateView`; the
/// independent TOOL observation supporting the derived pose is recorded here.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct EstimateProvenance {
    pub source: EstimateSource,
    pub held_transform_anchor_capture_tick: Option<u64>,
    pub held_transform_anchor_available_tick: Option<u64>,
    pub supporting_tool_capture_tick: Option<u64>,
    pub supporting_tool_distinct_feature_count: u32,
    pub supporting_tool_triangulation_head_count: u32,
    pub supporting_tool_minimum_calibrated_rays_per_point: u32,
    /// Hard one-sided position bound caused by unobservable rotation of the
    /// grasp's lateral offset about the axisymmetric tool axis.
    pub unobservable_roll_position_bound_m: f64,
    /// Hard angular bound caused by the observed PEG/TOOL axis mismatch whose
    /// azimuth cannot be transported without observable tool roll.
    pub unobservable_roll_axis_bound_rad: f64,
}

impl EstimateProvenance {
    pub const fn direct_feature_fit() -> Self {
        Self {
            source: EstimateSource::DirectFeatureFit,
            held_transform_anchor_capture_tick: None,
            held_transform_anchor_available_tick: None,
            supporting_tool_capture_tick: None,
            supporting_tool_distinct_feature_count: 0,
            supporting_tool_triangulation_head_count: 0,
            supporting_tool_minimum_calibrated_rays_per_point: 0,
            unobservable_roll_position_bound_m: 0.0,
            unobservable_roll_axis_bound_rad: 0.0,
        }
    }
}

/// Reduced, controller-visible grasp transform for axisymmetric M1e geometry.
/// Only the offset along the observed tool axis is transported as a mean. The
/// lateral offset direction and PEG-axis mismatch azimuth depend on
/// unobservable tool roll, so their observed magnitudes become explicit hard
/// bounds and are also added to the derived covariance at one third of the
/// bound (the runtime uses three-sigma envelopes).
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct HeldTransformEstimate {
    pub captured_at_tick: u64,
    pub available_at_tick: u64,
    pub axial_offset_m: f64,
    pub lateral_offset_bound_m: f64,
    pub axis_mismatch_bound_rad: f64,
    pub position_sigma_m: f64,
    pub axis_sigma_rad: f64,
    pub residual_rms_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeldTransformFailure {
    InvalidSourceEstimate,
    NonDirectSourceEstimate,
    InvalidConfiguredUncertainty,
    CarrierTimestampRegression,
    InvalidTransform,
}

/// Estimate the reduced grasp transform exclusively from fresh optical PEG and
/// TOOL estimates plus a configured attachment uncertainty.
pub fn estimate_held_transform(
    peg: &EstimateView,
    tool: &EstimateView,
    held_transform_sigma_m: f64,
) -> Result<HeldTransformEstimate, HeldTransformFailure> {
    if !valid_pose_value(peg) || !valid_pose_value(tool) {
        return Err(HeldTransformFailure::InvalidSourceEstimate);
    }
    if peg.provenance.source != EstimateSource::DirectFeatureFit
        || tool.provenance.source != EstimateSource::DirectFeatureFit
    {
        return Err(HeldTransformFailure::NonDirectSourceEstimate);
    }
    if !held_transform_sigma_m.is_finite() || held_transform_sigma_m < 0.0 {
        return Err(HeldTransformFailure::InvalidConfiguredUncertainty);
    }
    let offset = sub(peg.position_world_m, tool.position_world_m);
    let axial_offset_m = dot(offset, tool.axis_world);
    let lateral_offset = sub(offset, scale(tool.axis_world, axial_offset_m));
    let lateral_offset_bound_m = norm(lateral_offset);
    let axis_mismatch_bound_rad = axis_angle(peg.axis_world, tool.axis_world);
    let transform = HeldTransformEstimate {
        captured_at_tick: peg.capture_tick.min(tool.capture_tick),
        available_at_tick: peg.available_tick.max(tool.available_tick),
        axial_offset_m,
        lateral_offset_bound_m,
        axis_mismatch_bound_rad,
        position_sigma_m: peg.position_sigma_m + tool.position_sigma_m + held_transform_sigma_m,
        axis_sigma_rad: peg.axis_sigma_rad + tool.axis_sigma_rad,
        residual_rms_m: peg.residual_rms_m.max(tool.residual_rms_m),
    };
    if !valid_held_transform(transform) {
        return Err(HeldTransformFailure::InvalidTransform);
    }
    Ok(transform)
}

/// Derive an axisymmetric PEG pose from a valid grasp transform and a fresh,
/// directly observed TOOL pose. PEG feature counts remain zero: TOOL support is
/// reported separately in the provenance and is what the estimate guard uses.
pub fn derive_held_peg_from_tool(
    peg_object_id: u32,
    transform: HeldTransformEstimate,
    tool: &EstimateView,
    tick_period_s: f64,
    loaded_process_sigma_m_per_sqrt_s: f64,
    axis_lever_arm_m: f64,
) -> Result<EstimateView, HeldTransformFailure> {
    if !valid_pose_value(tool) {
        return Err(HeldTransformFailure::InvalidSourceEstimate);
    }
    if tool.provenance.source != EstimateSource::DirectFeatureFit {
        return Err(HeldTransformFailure::NonDirectSourceEstimate);
    }
    if !valid_held_transform(transform) {
        return Err(HeldTransformFailure::InvalidTransform);
    }
    if !tick_period_s.is_finite()
        || tick_period_s <= 0.0
        || !loaded_process_sigma_m_per_sqrt_s.is_finite()
        || loaded_process_sigma_m_per_sqrt_s < 0.0
        || !axis_lever_arm_m.is_finite()
        || axis_lever_arm_m <= 0.0
    {
        return Err(HeldTransformFailure::InvalidConfiguredUncertainty);
    }
    if tool.capture_tick < transform.captured_at_tick {
        return Err(HeldTransformFailure::CarrierTimestampRegression);
    }
    let elapsed_s =
        tool.capture_tick.saturating_sub(transform.captured_at_tick) as f64 * tick_period_s;
    let relative_process_sigma_m = loaded_process_sigma_m_per_sqrt_s * elapsed_s.sqrt();
    let relative_position_sigma_m = transform.position_sigma_m.hypot(relative_process_sigma_m);
    let relative_axis_sigma_rad = transform
        .axis_sigma_rad
        .hypot(relative_process_sigma_m / axis_lever_arm_m);
    Ok(EstimateView {
        object_id: peg_object_id,
        valid: true,
        invalid_reason: None,
        position_world_m: add(
            tool.position_world_m,
            scale(tool.axis_world, transform.axial_offset_m),
        ),
        // Roll-free axis transport has no observable azimuth for the captured
        // PEG/TOOL mismatch. Use the fresh tool axis as the mean and carry that
        // entire mismatch as a declared bound plus covariance contribution.
        axis_world: tool.axis_world,
        position_sigma_m: tool.position_sigma_m
            + relative_position_sigma_m
            + transform.lateral_offset_bound_m / 3.0,
        axis_sigma_rad: tool.axis_sigma_rad
            + relative_axis_sigma_rad
            + transform.axis_mismatch_bound_rad / 3.0,
        capture_tick: tool.capture_tick,
        available_tick: tool.available_tick,
        distinct_feature_count: 0,
        triangulation_head_count: 0,
        minimum_calibrated_rays_per_point: 0,
        residual_rms_m: tool.residual_rms_m.max(transform.residual_rms_m),
        provenance: EstimateProvenance {
            source: EstimateSource::HeldTransformFromFreshTool,
            held_transform_anchor_capture_tick: Some(transform.captured_at_tick),
            held_transform_anchor_available_tick: Some(transform.available_at_tick),
            supporting_tool_capture_tick: Some(tool.capture_tick),
            supporting_tool_distinct_feature_count: tool.distinct_feature_count,
            supporting_tool_triangulation_head_count: tool.triangulation_head_count,
            supporting_tool_minimum_calibrated_rays_per_point: tool
                .minimum_calibrated_rays_per_point,
            unobservable_roll_position_bound_m: transform.lateral_offset_bound_m,
            unobservable_roll_axis_bound_rad: transform.axis_mismatch_bound_rad,
        },
    })
}

fn valid_pose_value(estimate: &EstimateView) -> bool {
    estimate.valid
        && all_finite(&estimate.position_world_m)
        && all_finite(&estimate.axis_world)
        && (norm(estimate.axis_world) - 1.0).abs() <= 1.0e-6
        && estimate.position_sigma_m.is_finite()
        && estimate.position_sigma_m >= 0.0
        && estimate.axis_sigma_rad.is_finite()
        && estimate.axis_sigma_rad >= 0.0
        && estimate.residual_rms_m.is_finite()
        && estimate.residual_rms_m >= 0.0
        && estimate.capture_tick <= estimate.available_tick
}

fn valid_held_transform(transform: HeldTransformEstimate) -> bool {
    transform.captured_at_tick <= transform.available_at_tick
        && transform.axial_offset_m.is_finite()
        && transform.lateral_offset_bound_m.is_finite()
        && transform.lateral_offset_bound_m >= 0.0
        && transform.axis_mismatch_bound_rad.is_finite()
        && transform.axis_mismatch_bound_rad >= 0.0
        && transform.position_sigma_m.is_finite()
        && transform.position_sigma_m >= 0.0
        && transform.axis_sigma_rad.is_finite()
        && transform.axis_sigma_rad >= 0.0
        && transform.residual_rms_m.is_finite()
        && transform.residual_rms_m >= 0.0
}

fn axis_angle(a: [f64; 3], b: [f64; 3]) -> f64 {
    let cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    norm(cross).atan2(dot(a, b).clamp(-1.0, 1.0))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimateGate {
    pub maximum_age_ticks: u64,
    pub maximum_position_sigma_m: f64,
    pub maximum_axis_sigma_rad: f64,
    pub minimum_distinct_features: u32,
    pub minimum_triangulation_heads: u32,
    pub minimum_calibrated_rays_per_point: u32,
    pub maximum_residual_m: f64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EstimateGuardFailure {
    InvalidEstimate,
    MeasurementFromFuture,
    StaleMeasurement,
    ExcessivePositionUncertainty,
    ExcessiveAxisUncertainty,
    MissingRequiredFeature,
    InsufficientTriangulationHeads,
    InsufficientCalibratedRays,
    ExcessiveObservationResidual,
}

pub fn guard_estimate(
    now_tick: u64,
    estimate: &EstimateView,
    gate: EstimateGate,
) -> Result<(), EstimateGuardFailure> {
    if !estimate.valid {
        return Err(EstimateGuardFailure::InvalidEstimate);
    }
    let axis_norm = norm(estimate.axis_world);
    if !all_finite(&estimate.position_world_m)
        || !all_finite(&estimate.axis_world)
        || !estimate.position_sigma_m.is_finite()
        || estimate.position_sigma_m < 0.0
        || !estimate.axis_sigma_rad.is_finite()
        || estimate.axis_sigma_rad < 0.0
        || !estimate.residual_rms_m.is_finite()
        || estimate.residual_rms_m < 0.0
        || !axis_norm.is_finite()
        || (axis_norm - 1.0).abs() > 1.0e-6
        || !gate.maximum_position_sigma_m.is_finite()
        || gate.maximum_position_sigma_m < 0.0
        || !gate.maximum_axis_sigma_rad.is_finite()
        || gate.maximum_axis_sigma_rad < 0.0
        || !gate.maximum_residual_m.is_finite()
        || gate.maximum_residual_m < 0.0
        || gate.minimum_distinct_features == 0
        || gate.minimum_triangulation_heads == 0
        || gate.minimum_calibrated_rays_per_point == 0
    {
        return Err(EstimateGuardFailure::InvalidEstimate);
    }
    if estimate.available_tick > now_tick || estimate.capture_tick > estimate.available_tick {
        return Err(EstimateGuardFailure::MeasurementFromFuture);
    }
    if now_tick.saturating_sub(estimate.capture_tick) > gate.maximum_age_ticks {
        return Err(EstimateGuardFailure::StaleMeasurement);
    }
    if estimate.position_sigma_m > gate.maximum_position_sigma_m {
        return Err(EstimateGuardFailure::ExcessivePositionUncertainty);
    }
    if estimate.axis_sigma_rad > gate.maximum_axis_sigma_rad {
        return Err(EstimateGuardFailure::ExcessiveAxisUncertainty);
    }
    let (feature_count, head_count, rays_per_point) = match estimate.provenance.source {
        EstimateSource::DirectFeatureFit => (
            estimate.distinct_feature_count,
            estimate.triangulation_head_count,
            estimate.minimum_calibrated_rays_per_point,
        ),
        EstimateSource::HeldTransformFromFreshTool => (
            estimate.provenance.supporting_tool_distinct_feature_count,
            estimate.provenance.supporting_tool_triangulation_head_count,
            estimate
                .provenance
                .supporting_tool_minimum_calibrated_rays_per_point,
        ),
    };
    if feature_count < gate.minimum_distinct_features {
        return Err(EstimateGuardFailure::MissingRequiredFeature);
    }
    if head_count < gate.minimum_triangulation_heads {
        return Err(EstimateGuardFailure::InsufficientTriangulationHeads);
    }
    if rays_per_point < gate.minimum_calibrated_rays_per_point {
        return Err(EstimateGuardFailure::InsufficientCalibratedRays);
    }
    if estimate.residual_rms_m > gate.maximum_residual_m {
        return Err(EstimateGuardFailure::ExcessiveObservationResidual);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct RelativeEstimateUncertainty {
    /// Worst-case scalar-marginal sum. Same-head common-mode calibration error
    /// may cancel in reality, but M1e carries no cross-object covariance and
    /// therefore must not assume either independence or cancellation.
    pub position_sigma_m: f64,
    pub axis_sigma_rad: f64,
}

pub fn guard_relative_estimates(
    first: &EstimateView,
    second: &EstimateView,
    maximum_position_sigma_m: f64,
    maximum_axis_sigma_rad: f64,
) -> Result<RelativeEstimateUncertainty, EstimateGuardFailure> {
    if !first.valid || !second.valid {
        return Err(EstimateGuardFailure::InvalidEstimate);
    }
    let relative = RelativeEstimateUncertainty {
        position_sigma_m: first.position_sigma_m + second.position_sigma_m,
        axis_sigma_rad: first.axis_sigma_rad + second.axis_sigma_rad,
    };
    if !relative.position_sigma_m.is_finite()
        || !maximum_position_sigma_m.is_finite()
        || maximum_position_sigma_m < 0.0
        || relative.position_sigma_m > maximum_position_sigma_m
    {
        return Err(EstimateGuardFailure::ExcessivePositionUncertainty);
    }
    if !relative.axis_sigma_rad.is_finite()
        || !maximum_axis_sigma_rad.is_finite()
        || maximum_axis_sigma_rad < 0.0
        || relative.axis_sigma_rad > maximum_axis_sigma_rad
    {
        return Err(EstimateGuardFailure::ExcessiveAxisUncertainty);
    }
    Ok(relative)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxialGraspOverlapPolicy {
    pub jaw_axial_half_length_m: f64,
    pub peg_cylindrical_half_length_m: f64,
    pub minimum_overlap_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct AxialGraspOverlapEvidence {
    pub mean_overlap_m: f64,
    pub conservative_overlap_m: f64,
    pub relative_position_sigma_m: f64,
    pub relative_axis_sigma_rad: f64,
}

/// Require a finite length of full-radius cylindrical shaft to overlap the
/// calibrated jaw land. Position and axis uncertainty are applied as 3-sigma
/// bounds before the controller authorizes closure.
pub fn guard_axial_grasp_overlap(
    tool: &EstimateView,
    peg: &EstimateView,
    policy: AxialGraspOverlapPolicy,
) -> Result<AxialGraspOverlapEvidence, &'static str> {
    if !valid_pose_value(tool)
        || !valid_pose_value(peg)
        || !policy.jaw_axial_half_length_m.is_finite()
        || policy.jaw_axial_half_length_m <= 0.0
        || !policy.peg_cylindrical_half_length_m.is_finite()
        || policy.peg_cylindrical_half_length_m <= 0.0
        || !policy.minimum_overlap_m.is_finite()
        || policy.minimum_overlap_m <= 0.0
    {
        return Err("invalid_axial_grasp_geometry");
    }
    let relative_position_sigma_m = tool.position_sigma_m + peg.position_sigma_m;
    let relative_axis_sigma_rad = tool.axis_sigma_rad + peg.axis_sigma_rad;
    let mean_axis_error_rad = axis_angle(tool.axis_world, peg.axis_world);
    let worst_axis_error_rad =
        (mean_axis_error_rad + 3.0 * relative_axis_sigma_rad).min(core::f64::consts::FRAC_PI_2);
    let mean_half_projection_m =
        policy.peg_cylindrical_half_length_m * mean_axis_error_rad.cos().max(0.0);
    let conservative_half_projection_m =
        policy.peg_cylindrical_half_length_m * worst_axis_error_rad.cos().max(0.0);
    let center_m = dot(
        sub(peg.position_world_m, tool.position_world_m),
        tool.axis_world,
    );
    let overlap = |center_m: f64, half_projection_m: f64| {
        (policy
            .jaw_axial_half_length_m
            .min(center_m + half_projection_m)
            - (-policy.jaw_axial_half_length_m).max(center_m - half_projection_m))
        .max(0.0)
    };
    let mean_overlap_m = overlap(center_m, mean_half_projection_m);
    let center_bound_m = 3.0 * relative_position_sigma_m;
    let conservative_overlap_m = overlap(center_m - center_bound_m, conservative_half_projection_m)
        .min(overlap(
            center_m + center_bound_m,
            conservative_half_projection_m,
        ));
    let evidence = AxialGraspOverlapEvidence {
        mean_overlap_m,
        conservative_overlap_m,
        relative_position_sigma_m,
        relative_axis_sigma_rad,
    };
    if conservative_overlap_m + f64::EPSILON < policy.minimum_overlap_m {
        return Err("insufficient_axial_grasp_overlap");
    }
    Ok(evidence)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PalmPegAxialClearancePolicy {
    /// Forward-most central palm plane in tool-local Z. A recessed palm is
    /// negative; the open jaw channel may extend forward of it.
    pub palm_forward_plane_tool_z_m: f64,
    pub peg_cylindrical_half_length_m: f64,
    pub peg_radius_m: f64,
    pub minimum_clearance_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct PalmPegAxialClearanceEvidence {
    pub minimum_clearance_m: f64,
    pub observed_center_axial_offset_m: f64,
    pub peg_backward_support_m: f64,
    pub relative_position_bound_m: f64,
    pub tool_axis_projection_bound_m: f64,
}

/// Require the peg's trailing capsule surface to remain ahead of the recessed
/// central palm. The jaw/peg contact is intentional, but no other tool solid
/// inherits that exemption. Scalar marginal uncertainties are summed because
/// this safety gate does not assume independent errors.
pub fn guard_palm_peg_axial_clearance(
    tool: &EstimateView,
    peg: &EstimateView,
    policy: PalmPegAxialClearancePolicy,
) -> Result<PalmPegAxialClearanceEvidence, &'static str> {
    if !valid_pose_value(tool)
        || !valid_pose_value(peg)
        || !policy.palm_forward_plane_tool_z_m.is_finite()
        || policy.palm_forward_plane_tool_z_m >= 0.0
        || !policy.peg_cylindrical_half_length_m.is_finite()
        || policy.peg_cylindrical_half_length_m <= 0.0
        || !policy.peg_radius_m.is_finite()
        || policy.peg_radius_m <= 0.0
        || !policy.minimum_clearance_m.is_finite()
        || policy.minimum_clearance_m < 0.0
    {
        return Err("invalid_palm_peg_clearance_geometry");
    }
    let relative = sub(peg.position_world_m, tool.position_world_m);
    let observed_center_axial_offset_m = dot(relative, tool.axis_world);
    let peg_backward_support_m = policy.peg_cylindrical_half_length_m + policy.peg_radius_m;
    let relative_position_bound_m = 3.0 * (tool.position_sigma_m + peg.position_sigma_m);
    let tool_axis_cone_rad = (3.0 * tool.axis_sigma_rad).min(core::f64::consts::FRAC_PI_2);
    let tool_axis_projection_bound_m = 2.0 * norm(relative) * (0.5 * tool_axis_cone_rad).sin();
    let minimum_clearance_m = observed_center_axial_offset_m
        - peg_backward_support_m
        - policy.palm_forward_plane_tool_z_m
        - relative_position_bound_m
        - tool_axis_projection_bound_m;
    let evidence = PalmPegAxialClearanceEvidence {
        minimum_clearance_m,
        observed_center_axial_offset_m,
        peg_backward_support_m,
        relative_position_bound_m,
        tool_axis_projection_bound_m,
    };
    if minimum_clearance_m + f64::EPSILON < policy.minimum_clearance_m {
        return Err("palm_peg_clearance_risk");
    }
    Ok(evidence)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JawSocketAxialClearancePolicy {
    pub jaw_axial_half_length_m: f64,
    pub jaw_transverse_radius_m: f64,
    pub side_target_forward_extent_m: f64,
    pub side_target_transverse_radius_m: f64,
    pub socket_depth_m: f64,
    pub minimum_clearance_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JawSocketMotionPreview {
    pub commanded_translation_world_m: [f64; 3],
    pub anticipated_tool_position_sigma_m: f64,
    pub anticipated_tool_axis_sigma_rad: f64,
    pub commanded_tool_axis_change_bound_rad: f64,
    pub path_deviation_bound_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct JawSocketAxialClearanceEvidence {
    pub minimum_clearance_m: f64,
    pub projected_jaw_half_extent_m: f64,
    pub projected_side_target_forward_extent_m: f64,
    pub projected_tool_forward_extent_m: f64,
    pub relative_position_sigma_m: f64,
    pub relative_axis_sigma_rad: f64,
    pub commanded_tool_axis_change_bound_rad: f64,
    pub path_deviation_bound_m: f64,
    pub maximum_tool_socket_axis_error_rad: f64,
    pub socket_axis_projection_bound_m: f64,
}

/// Conservative swept clearance between the jaw/tool envelope and the socket
/// entrance plane. The socket bore exemption applies only to the PEG; this
/// gate prevents the jaws from inheriting that exemption.
pub fn guard_jaw_socket_axial_clearance(
    tool: &EstimateView,
    socket: &EstimateView,
    motion: JawSocketMotionPreview,
    policy: JawSocketAxialClearancePolicy,
) -> Result<JawSocketAxialClearanceEvidence, &'static str> {
    if !valid_pose_value(tool)
        || !valid_pose_value(socket)
        || !all_finite(&motion.commanded_translation_world_m)
        || !motion.anticipated_tool_position_sigma_m.is_finite()
        || motion.anticipated_tool_position_sigma_m < 0.0
        || !motion.anticipated_tool_axis_sigma_rad.is_finite()
        || motion.anticipated_tool_axis_sigma_rad < 0.0
        || !motion.commanded_tool_axis_change_bound_rad.is_finite()
        || motion.commanded_tool_axis_change_bound_rad < 0.0
        || !motion.path_deviation_bound_m.is_finite()
        || motion.path_deviation_bound_m < 0.0
        || !policy.jaw_axial_half_length_m.is_finite()
        || policy.jaw_axial_half_length_m <= 0.0
        || !policy.jaw_transverse_radius_m.is_finite()
        || policy.jaw_transverse_radius_m <= 0.0
        || !policy.side_target_forward_extent_m.is_finite()
        || policy.side_target_forward_extent_m <= 0.0
        || !policy.side_target_transverse_radius_m.is_finite()
        || policy.side_target_transverse_radius_m <= 0.0
        || !policy.socket_depth_m.is_finite()
        || policy.socket_depth_m <= 0.0
        || !policy.minimum_clearance_m.is_finite()
        || policy.minimum_clearance_m < 0.0
    {
        return Err("invalid_jaw_socket_clearance_geometry");
    }
    // This safety boundary receives scalar marginals, not a relative
    // covariance with a declared independence contract. Sum them so common or
    // unknown correlation cannot make the bound optimistic.
    let relative_position_sigma_m =
        motion.anticipated_tool_position_sigma_m + socket.position_sigma_m;
    let relative_axis_sigma_rad = motion.anticipated_tool_axis_sigma_rad + socket.axis_sigma_rad;
    let mean_axis_error_rad = axis_angle(tool.axis_world, socket.axis_world);
    let maximum_axis_error_rad = (mean_axis_error_rad
        + motion.commanded_tool_axis_change_bound_rad
        + 3.0 * relative_axis_sigma_rad)
        .min(core::f64::consts::FRAC_PI_2);
    let projected_jaw_half_extent_m = maximum_axial_support_over_axis_cone(
        policy.jaw_axial_half_length_m,
        policy.jaw_transverse_radius_m,
        maximum_axis_error_rad,
    );
    let projected_side_target_forward_extent_m = maximum_axial_support_over_axis_cone(
        policy.side_target_forward_extent_m,
        policy.side_target_transverse_radius_m,
        maximum_axis_error_rad,
    );
    let projected_tool_forward_extent_m =
        projected_jaw_half_extent_m.max(projected_side_target_forward_extent_m);
    let end_tool_world_m = add(tool.position_world_m, motion.commanded_translation_world_m);
    let start_center_to_entrance_m = dot(
        sub(socket.position_world_m, tool.position_world_m),
        socket.axis_world,
    ) - 0.5 * policy.socket_depth_m;
    let end_center_to_entrance_m = dot(
        sub(socket.position_world_m, end_tool_world_m),
        socket.axis_world,
    ) - 0.5 * policy.socket_depth_m;
    let socket_axis_cone_rad = (3.0 * socket.axis_sigma_rad).min(core::f64::consts::FRAC_PI_2);
    let maximum_socket_pivot_radius_m = norm(sub(socket.position_world_m, tool.position_world_m))
        .max(norm(sub(socket.position_world_m, end_tool_world_m)));
    // Exact chord bound for rotating a vector of the maximum pivot radius by
    // any angle inside the socket-axis uncertainty cone.
    let socket_axis_projection_bound_m =
        2.0 * maximum_socket_pivot_radius_m * (0.5 * socket_axis_cone_rad).sin();
    let minimum_clearance_m = start_center_to_entrance_m.min(end_center_to_entrance_m)
        - projected_tool_forward_extent_m
        - 3.0 * relative_position_sigma_m
        - motion.path_deviation_bound_m
        - socket_axis_projection_bound_m;
    let evidence = JawSocketAxialClearanceEvidence {
        minimum_clearance_m,
        projected_jaw_half_extent_m,
        projected_side_target_forward_extent_m,
        projected_tool_forward_extent_m,
        relative_position_sigma_m,
        relative_axis_sigma_rad,
        commanded_tool_axis_change_bound_rad: motion.commanded_tool_axis_change_bound_rad,
        path_deviation_bound_m: motion.path_deviation_bound_m,
        maximum_tool_socket_axis_error_rad: maximum_axis_error_rad,
        socket_axis_projection_bound_m,
    };
    if minimum_clearance_m + f64::EPSILON < policy.minimum_clearance_m {
        return Err("jaw_socket_clearance_risk");
    }
    Ok(evidence)
}

fn maximum_axial_support_over_axis_cone(
    axial_extent_m: f64,
    transverse_radius_m: f64,
    maximum_axis_error_rad: f64,
) -> f64 {
    let optimum_rad = transverse_radius_m.atan2(axial_extent_m);
    if maximum_axis_error_rad >= optimum_rad {
        axial_extent_m.hypot(transverse_radius_m)
    } else {
        axial_extent_m * maximum_axis_error_rad.cos()
            + transverse_radius_m * maximum_axis_error_rad.sin()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrectionPolicy {
    pub gain: f64,
    pub convergence_m: f64,
    pub maximum_magnitude_m: f64,
    pub minimum_reproducible_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum CorrectionDecision {
    Converged {
        residual_m: f64,
    },
    Command {
        residual_before_m: f64,
        correction_world_m: [f64; 3],
        target_world_m: [f64; 3],
    },
    Rejected {
        reason: &'static str,
        requested_magnitude_m: f64,
    },
}

pub fn decide_correction(
    current_world_m: [f64; 3],
    desired_world_m: [f64; 3],
    policy: CorrectionPolicy,
) -> CorrectionDecision {
    let residual = sub(desired_world_m, current_world_m);
    let residual_m = norm(residual);
    if !residual_m.is_finite()
        || !policy.gain.is_finite()
        || !policy.convergence_m.is_finite()
        || !policy.maximum_magnitude_m.is_finite()
        || !policy.minimum_reproducible_m.is_finite()
    {
        return CorrectionDecision::Rejected {
            reason: "non_finite_correction",
            requested_magnitude_m: residual_m,
        };
    }
    if residual_m <= policy.convergence_m {
        return CorrectionDecision::Converged { residual_m };
    }
    if policy.minimum_reproducible_m > policy.convergence_m * 2.0 {
        return CorrectionDecision::Rejected {
            reason: "correction_floor_too_large",
            requested_magnitude_m: residual_m,
        };
    }
    let mut correction = scale(residual, policy.gain);
    // Cartesian components below the measured correction floor cannot be
    // commanded reproducibly. Suppress them rather than letting noise-driven
    // sign flips excite a full backlash event on that axis.
    for component in &mut correction {
        if component.abs() < policy.minimum_reproducible_m {
            *component = 0.0;
        }
    }
    let correction_m = norm(correction);
    if correction_m <= f64::EPSILON {
        return CorrectionDecision::Rejected {
            reason: "correction_below_reproducible_floor",
            requested_magnitude_m: residual_m * policy.gain,
        };
    }
    if correction_m > policy.maximum_magnitude_m {
        return CorrectionDecision::Rejected {
            reason: "correction_magnitude_limit",
            requested_magnitude_m: correction_m,
        };
    }
    CorrectionDecision::Command {
        residual_before_m: residual_m,
        correction_world_m: correction,
        target_world_m: add(current_world_m, correction),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct PlanningObstacle {
    pub id: u32,
    pub center_world_m: [f64; 3],
    pub conservative_radius_m: f64,
    pub position_sigma_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct SweptEnvelope {
    pub center_start_world_m: [f64; 3],
    pub center_end_world_m: [f64; 3],
    pub radius_m: f64,
    pub position_sigma_m: f64,
    /// One-sided calibrated/command-model bound, applied directly rather than
    /// treated as a random standard deviation.
    pub hard_position_bound_m: f64,
    /// Certified maximum departure of the commanded FK path from the endpoint
    /// chord, including the bounded interval between deterministic samples.
    pub path_deviation_bound_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ClearanceCheck {
    pub obstacle_id: u32,
    pub minimum_clearance_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SweptPreflightFailure {
    InvalidGeometry,
    Clearance(ClearanceCheck),
}

/// Conservative controller-side segment/sphere preflight. Both the moving
/// envelope and obstacle are inflated by three standard deviations.
pub fn preflight_swept_envelope(
    envelope: SweptEnvelope,
    obstacles: &[PlanningObstacle],
    required_clearance_m: f64,
) -> Result<Vec<ClearanceCheck>, SweptPreflightFailure> {
    if !all_finite(&envelope.center_start_world_m)
        || !all_finite(&envelope.center_end_world_m)
        || !envelope.radius_m.is_finite()
        || envelope.radius_m < 0.0
        || !envelope.position_sigma_m.is_finite()
        || envelope.position_sigma_m < 0.0
        || !envelope.hard_position_bound_m.is_finite()
        || envelope.hard_position_bound_m < 0.0
        || !envelope.path_deviation_bound_m.is_finite()
        || envelope.path_deviation_bound_m < 0.0
        || !required_clearance_m.is_finite()
        || required_clearance_m < 0.0
        || obstacles.iter().any(|obstacle| {
            !all_finite(&obstacle.center_world_m)
                || !obstacle.conservative_radius_m.is_finite()
                || obstacle.conservative_radius_m < 0.0
                || !obstacle.position_sigma_m.is_finite()
                || obstacle.position_sigma_m < 0.0
        })
    {
        return Err(SweptPreflightFailure::InvalidGeometry);
    }
    let mut ordered = obstacles.to_vec();
    ordered.sort_by_key(|obstacle| obstacle.id);
    let mut checks = Vec::with_capacity(ordered.len());
    for obstacle in ordered {
        let centerline_distance = point_segment_distance(
            obstacle.center_world_m,
            envelope.center_start_world_m,
            envelope.center_end_world_m,
        );
        let inflated_radius = envelope.radius_m
            + obstacle.conservative_radius_m
            + 3.0 * envelope.position_sigma_m.max(0.0)
            + envelope.hard_position_bound_m
            + envelope.path_deviation_bound_m
            + 3.0 * obstacle.position_sigma_m.max(0.0);
        let check = ClearanceCheck {
            obstacle_id: obstacle.id,
            minimum_clearance_m: centerline_distance - inflated_radius,
        };
        if check.minimum_clearance_m < required_clearance_m {
            return Err(SweptPreflightFailure::Clearance(check));
        }
        checks.push(check);
    }
    Ok(checks)
}

fn all_finite(value: &[f64; 3]) -> bool {
    value.iter().all(|component| component.is_finite())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactState {
    FreeMotion,
    LeadInContact,
    RecoverableLateralContact,
    ExcessiveInterference,
    Seated,
    Jammed,
}

/// Raw contact channels that can be implemented by jaw switches/deflection
/// gauges and a compliant-axis force proxy.  The packet deliberately contains
/// no simulator contact class, attachment state, or seating verdict.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ContactPacket {
    pub captured_at_tick: u64,
    /// Non-jaw contact on the held part/tool load path.
    pub contact_detected: bool,
    pub left_pad_contact: bool,
    pub right_pad_contact: bool,
    pub left_pad_deflection_m: f64,
    pub right_pad_deflection_m: f64,
    pub grip_force_proxy_n: f64,
    pub insertion_force_proxy_n: f64,
}

/// Relative mating geometry derived from estimator output.  It is a
/// controller-visible observation, not a plant query.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct RelativeMatingPose {
    pub captured_at_tick: u64,
    pub available_at_tick: u64,
    pub axial_error_m: f64,
    pub lateral_error_m: f64,
    pub axis_error_rad: f64,
    pub position_sigma_m: f64,
    pub axis_sigma_rad: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactClassificationPolicy {
    pub maximum_packet_age_ticks: u64,
    pub maximum_pose_age_ticks: u64,
    pub lead_in_start_m: f64,
    pub recoverable_lateral_error_m: f64,
    pub maximum_lateral_error_m: f64,
    pub seat_axial_tolerance_m: f64,
    pub seat_lateral_tolerance_m: f64,
    pub seat_axis_tolerance_rad: f64,
    /// Converts observed angular error into a conservative lateral error at
    /// the peg endpoint.
    pub axis_lever_arm_m: f64,
    pub maximum_position_sigma_m: f64,
    pub maximum_axis_sigma_rad: f64,
    pub maximum_force_proxy_n: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactClassificationFailure {
    InvalidPolicy,
    InvalidPacket,
    ContactPacketFromFuture,
    StaleContactPacket,
    InvalidRelativeGeometry,
    ExcessiveRelativeUncertainty,
    RelativeGeometryFromFuture,
    StaleRelativeGeometry,
    InconsistentContactGeometry,
    ContactForceLimit,
}

/// Contact state classified exclusively from raw channels and optional
/// estimator-derived relative geometry.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ClassifiedContactEvidence {
    pub captured_at_tick: u64,
    pub classified_at_tick: u64,
    pub state: ContactState,
    pub contact_detected: bool,
    pub bilateral_jaw_contact: bool,
    pub left_pad_deflection_m: f64,
    pub right_pad_deflection_m: f64,
    pub grip_force_proxy_n: f64,
    pub insertion_force_proxy_n: f64,
    pub relative_mating_pose: Option<RelativeMatingPose>,
}

/// Apply deterministic validity, freshness, force, and observed-geometry
/// gates before assigning a reduced contact state.  `relative_mating_pose` is
/// optional for grasp/transport packets, but is required to classify any
/// asserted non-jaw contact.
pub fn classify_contact_packet(
    now_tick: u64,
    packet: ContactPacket,
    relative_mating_pose: Option<RelativeMatingPose>,
    policy: ContactClassificationPolicy,
) -> Result<ClassifiedContactEvidence, ContactClassificationFailure> {
    if !valid_contact_policy(policy) {
        return Err(ContactClassificationFailure::InvalidPolicy);
    }
    if !packet.left_pad_deflection_m.is_finite()
        || !packet.right_pad_deflection_m.is_finite()
        || !packet.grip_force_proxy_n.is_finite()
        || !packet.insertion_force_proxy_n.is_finite()
        || packet.left_pad_deflection_m < 0.0
        || packet.right_pad_deflection_m < 0.0
        || packet.grip_force_proxy_n < 0.0
        || packet.insertion_force_proxy_n < 0.0
        || (!packet.left_pad_contact && packet.left_pad_deflection_m > 0.0)
        || (!packet.right_pad_contact && packet.right_pad_deflection_m > 0.0)
        || (!packet.contact_detected && packet.insertion_force_proxy_n > 0.0)
    {
        return Err(ContactClassificationFailure::InvalidPacket);
    }
    if packet.captured_at_tick > now_tick {
        return Err(ContactClassificationFailure::ContactPacketFromFuture);
    }
    if now_tick.saturating_sub(packet.captured_at_tick) > policy.maximum_packet_age_ticks {
        return Err(ContactClassificationFailure::StaleContactPacket);
    }
    if packet.insertion_force_proxy_n > policy.maximum_force_proxy_n {
        return Err(ContactClassificationFailure::ContactForceLimit);
    }

    let state = match relative_mating_pose {
        Some(relative) => {
            if !relative.axial_error_m.is_finite()
                || !relative.lateral_error_m.is_finite()
                || !relative.axis_error_rad.is_finite()
                || !relative.position_sigma_m.is_finite()
                || !relative.axis_sigma_rad.is_finite()
                || relative.axial_error_m < 0.0
                || relative.lateral_error_m < 0.0
                || relative.axis_error_rad < 0.0
                || relative.position_sigma_m < 0.0
                || relative.axis_sigma_rad < 0.0
                || relative.captured_at_tick > relative.available_at_tick
            {
                return Err(ContactClassificationFailure::InvalidRelativeGeometry);
            }
            if relative.available_at_tick > now_tick {
                return Err(ContactClassificationFailure::RelativeGeometryFromFuture);
            }
            if now_tick.saturating_sub(relative.captured_at_tick) > policy.maximum_pose_age_ticks {
                return Err(ContactClassificationFailure::StaleRelativeGeometry);
            }
            if relative.position_sigma_m > policy.maximum_position_sigma_m
                || relative.axis_sigma_rad > policy.maximum_axis_sigma_rad
            {
                return Err(ContactClassificationFailure::ExcessiveRelativeUncertainty);
            }
            classify_relative_contact(packet, relative, policy)?
        }
        None if packet.contact_detected => {
            return Err(ContactClassificationFailure::InconsistentContactGeometry);
        }
        None => ContactState::FreeMotion,
    };

    Ok(ClassifiedContactEvidence {
        captured_at_tick: packet.captured_at_tick,
        classified_at_tick: now_tick,
        state,
        contact_detected: packet.contact_detected,
        bilateral_jaw_contact: packet.left_pad_contact && packet.right_pad_contact,
        left_pad_deflection_m: packet.left_pad_deflection_m,
        right_pad_deflection_m: packet.right_pad_deflection_m,
        grip_force_proxy_n: packet.grip_force_proxy_n,
        insertion_force_proxy_n: packet.insertion_force_proxy_n,
        relative_mating_pose,
    })
}

fn valid_contact_policy(policy: ContactClassificationPolicy) -> bool {
    [
        policy.lead_in_start_m,
        policy.recoverable_lateral_error_m,
        policy.maximum_lateral_error_m,
        policy.seat_axial_tolerance_m,
        policy.seat_lateral_tolerance_m,
        policy.seat_axis_tolerance_rad,
        policy.axis_lever_arm_m,
        policy.maximum_position_sigma_m,
        policy.maximum_axis_sigma_rad,
        policy.maximum_force_proxy_n,
    ]
    .iter()
    .all(|value| value.is_finite())
        && policy.lead_in_start_m > 0.0
        && policy.seat_axial_tolerance_m > 0.0
        && policy.seat_lateral_tolerance_m > 0.0
        && policy.seat_axis_tolerance_rad > 0.0
        && policy.axis_lever_arm_m > 0.0
        && policy.maximum_position_sigma_m > 0.0
        && policy.maximum_axis_sigma_rad > 0.0
        && policy.maximum_force_proxy_n > 0.0
        && policy.seat_lateral_tolerance_m <= policy.recoverable_lateral_error_m
        && policy.recoverable_lateral_error_m <= policy.maximum_lateral_error_m
}

fn classify_relative_contact(
    packet: ContactPacket,
    relative: RelativeMatingPose,
    policy: ContactClassificationPolicy,
) -> Result<ContactState, ContactClassificationFailure> {
    if !packet.contact_detected {
        return Ok(ContactState::FreeMotion);
    }
    if relative.axial_error_m > policy.lead_in_start_m {
        return Err(ContactClassificationFailure::InconsistentContactGeometry);
    }
    if relative.axial_error_m <= policy.seat_axial_tolerance_m
        && relative.lateral_error_m <= policy.seat_lateral_tolerance_m
        && relative.axis_error_rad <= policy.seat_axis_tolerance_rad
    {
        return Ok(ContactState::Seated);
    }
    if relative.lateral_error_m <= policy.seat_lateral_tolerance_m
        && relative.axis_error_rad <= policy.seat_axis_tolerance_rad
    {
        return Ok(ContactState::LeadInContact);
    }
    let effective_lateral_error_m =
        relative.lateral_error_m + relative.axis_error_rad.sin().abs() * policy.axis_lever_arm_m;
    if effective_lateral_error_m <= policy.recoverable_lateral_error_m {
        Ok(ContactState::RecoverableLateralContact)
    } else if effective_lateral_error_m <= policy.maximum_lateral_error_m {
        Ok(ContactState::ExcessiveInterference)
    } else {
        Ok(ContactState::Jammed)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspEvidenceGate {
    pub minimum_pad_deflection_m: f64,
    pub maximum_pad_deflection_m: f64,
    pub minimum_force_n: f64,
    pub maximum_force_n: f64,
}

pub fn guard_grasp_evidence(
    evidence: ClassifiedContactEvidence,
    gate: GraspEvidenceGate,
) -> Result<(), &'static str> {
    if !evidence.left_pad_deflection_m.is_finite()
        || !evidence.right_pad_deflection_m.is_finite()
        || !evidence.grip_force_proxy_n.is_finite()
        || evidence.left_pad_deflection_m < 0.0
        || evidence.right_pad_deflection_m < 0.0
        || evidence.grip_force_proxy_n < 0.0
        || !gate.minimum_pad_deflection_m.is_finite()
        || !gate.maximum_pad_deflection_m.is_finite()
        || !gate.minimum_force_n.is_finite()
        || !gate.maximum_force_n.is_finite()
        || gate.minimum_pad_deflection_m < 0.0
        || gate.maximum_pad_deflection_m < gate.minimum_pad_deflection_m
        || gate.minimum_force_n < 0.0
        || gate.maximum_force_n < gate.minimum_force_n
    {
        return Err("invalid_grasp_evidence");
    }
    if !evidence.bilateral_jaw_contact {
        return Err("bilateral_contact_missing");
    }
    for deflection in [
        evidence.left_pad_deflection_m,
        evidence.right_pad_deflection_m,
    ] {
        if deflection < gate.minimum_pad_deflection_m || deflection > gate.maximum_pad_deflection_m
        {
            return Err("pad_deflection_out_of_bounds");
        }
    }
    if evidence.grip_force_proxy_n < gate.minimum_force_n
        || evidence.grip_force_proxy_n > gate.maximum_force_n
    {
        return Err("grip_force_out_of_bounds");
    }
    Ok(())
}

fn point_segment_distance(point: [f64; 3], start: [f64; 3], end: [f64; 3]) -> f64 {
    let segment = sub(end, start);
    let length_squared = dot(segment, segment);
    if length_squared <= f64::EPSILON {
        return norm(sub(point, start));
    }
    let t = (dot(sub(point, start), segment) / length_squared).clamp(0.0, 1.0);
    norm(sub(point, add(start, scale(segment, t))))
}

pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn scale(value: [f64; 3], factor: f64) -> [f64; 3] {
    [value[0] * factor, value[1] * factor, value[2] * factor]
}

pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn norm(value: [f64; 3]) -> f64 {
    dot(value, value).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn estimate() -> EstimateView {
        EstimateView {
            object_id: 1,
            valid: true,
            invalid_reason: None,
            position_world_m: [0.0; 3],
            axis_world: [0.0, 0.0, 1.0],
            position_sigma_m: 4.0e-6,
            axis_sigma_rad: 0.01,
            capture_tick: 100,
            available_tick: 150,
            distinct_feature_count: 4,
            triangulation_head_count: 1,
            minimum_calibrated_rays_per_point: 2,
            residual_rms_m: 2.0e-6,
            provenance: EstimateProvenance::direct_feature_fit(),
        }
    }

    fn gate() -> EstimateGate {
        EstimateGate {
            maximum_age_ticks: 60,
            maximum_position_sigma_m: 10.0e-6,
            maximum_axis_sigma_rad: 0.02,
            minimum_distinct_features: 4,
            minimum_triangulation_heads: 1,
            minimum_calibrated_rays_per_point: 2,
            maximum_residual_m: 20.0e-6,
        }
    }

    fn contact_packet() -> ContactPacket {
        ContactPacket {
            captured_at_tick: 150,
            contact_detected: true,
            left_pad_contact: true,
            right_pad_contact: true,
            left_pad_deflection_m: 4.0e-6,
            right_pad_deflection_m: 5.0e-6,
            grip_force_proxy_n: 0.02,
            insertion_force_proxy_n: 0.01,
        }
    }

    fn relative_pose() -> RelativeMatingPose {
        RelativeMatingPose {
            captured_at_tick: 145,
            available_at_tick: 150,
            axial_error_m: 80.0e-6,
            lateral_error_m: 20.0e-6,
            axis_error_rad: 0.005,
            position_sigma_m: 5.0e-6,
            axis_sigma_rad: 0.005,
        }
    }

    fn contact_policy() -> ContactClassificationPolicy {
        ContactClassificationPolicy {
            maximum_packet_age_ticks: 10,
            maximum_pose_age_ticks: 20,
            lead_in_start_m: 300.0e-6,
            recoverable_lateral_error_m: 95.0e-6,
            maximum_lateral_error_m: 140.0e-6,
            seat_axial_tolerance_m: 18.0e-6,
            seat_lateral_tolerance_m: 30.0e-6,
            seat_axis_tolerance_rad: 0.010,
            axis_lever_arm_m: 300.0e-6,
            maximum_position_sigma_m: 10.0e-6,
            maximum_axis_sigma_rad: 0.02,
            maximum_force_proxy_n: 0.080,
        }
    }

    #[test]
    fn stale_estimate_is_rejected_before_near_contact() {
        assert_eq!(
            guard_estimate(161, &estimate(), gate()),
            Err(EstimateGuardFailure::StaleMeasurement)
        );
    }

    #[test]
    fn one_head_with_two_calibrated_rays_is_not_counted_as_two_heads() {
        let mut required_two_heads = gate();
        required_two_heads.minimum_triangulation_heads = 2;
        assert_eq!(
            guard_estimate(155, &estimate(), required_two_heads),
            Err(EstimateGuardFailure::InsufficientTriangulationHeads)
        );
    }

    #[test]
    fn held_transform_is_direct_only_and_declares_roll_ambiguity() {
        let tool = estimate();
        let mut peg = estimate();
        peg.object_id = 2;
        peg.position_world_m = [6.0e-6, -8.0e-6, 100.0e-6];
        peg.axis_world = [0.006_f64.sin(), 0.0, 0.006_f64.cos()];

        let transform = estimate_held_transform(&peg, &tool, 4.0e-6).unwrap();
        assert!((transform.axial_offset_m - 100.0e-6).abs() < 1.0e-15);
        assert!((transform.lateral_offset_bound_m - 10.0e-6).abs() < 1.0e-15);
        assert!((transform.axis_mismatch_bound_rad - 0.006).abs() < 1.0e-12);

        let mut fresh_tool = tool.clone();
        fresh_tool.position_world_m = [1.0e-3, 2.0e-3, 3.0e-3];
        fresh_tool.capture_tick = 160;
        fresh_tool.available_tick = 165;
        let derived =
            derive_held_peg_from_tool(2, transform, &fresh_tool, 0.001, 2.0e-6, 0.5e-3).unwrap();
        assert_eq!(derived.position_world_m, [1.0e-3, 2.0e-3, 3.1e-3]);
        assert_eq!(derived.distinct_feature_count, 0);
        assert_eq!(derived.triangulation_head_count, 0);
        assert_eq!(
            derived.provenance.source,
            EstimateSource::HeldTransformFromFreshTool
        );
        assert_eq!(
            derived.provenance.supporting_tool_capture_tick,
            Some(fresh_tool.capture_tick)
        );
        assert_eq!(
            derived.provenance.held_transform_anchor_capture_tick,
            Some(transform.captured_at_tick)
        );
        assert_eq!(
            derived.provenance.unobservable_roll_position_bound_m,
            10.0e-6
        );
        let mut permissive_gate = gate();
        permissive_gate.maximum_position_sigma_m = 20.0e-6;
        permissive_gate.maximum_axis_sigma_rad = 0.05;
        assert_eq!(guard_estimate(165, &derived, permissive_gate), Ok(()));

        let mut forged_source = peg;
        forged_source.provenance = derived.provenance;
        assert_eq!(
            estimate_held_transform(&forged_source, &tool, 4.0e-6),
            Err(HeldTransformFailure::NonDirectSourceEstimate)
        );
    }

    #[test]
    fn axial_grasp_overlap_is_covariance_bounded() {
        let tool = estimate();
        let mut peg = estimate();
        peg.position_world_m[2] = 0.750e-3;
        let policy = AxialGraspOverlapPolicy {
            jaw_axial_half_length_m: 0.200e-3,
            peg_cylindrical_half_length_m: 0.700e-3,
            minimum_overlap_m: 0.100e-3,
        };
        let evidence = guard_axial_grasp_overlap(&tool, &peg, policy).unwrap();
        assert!(evidence.mean_overlap_m > 0.149e-3);
        assert!(evidence.conservative_overlap_m >= policy.minimum_overlap_m);

        peg.position_world_m[2] = 0.820e-3;
        assert_eq!(
            guard_axial_grasp_overlap(&tool, &peg, policy),
            Err("insufficient_axial_grasp_overlap")
        );
    }

    #[test]
    fn recessed_palm_clears_tail_grasp_and_old_central_flange_does_not() {
        let tool = estimate();
        let mut peg = estimate();
        peg.position_world_m[2] = 0.750e-3;
        let recessed = PalmPegAxialClearancePolicy {
            palm_forward_plane_tool_z_m: -0.350e-3,
            peg_cylindrical_half_length_m: 0.700e-3,
            peg_radius_m: 0.200e-3,
            minimum_clearance_m: 0.100e-3,
        };
        let evidence = guard_palm_peg_axial_clearance(&tool, &peg, recessed).unwrap();
        assert!(evidence.minimum_clearance_m >= recessed.minimum_clearance_m);

        let old_central_flange = PalmPegAxialClearancePolicy {
            palm_forward_plane_tool_z_m: -f64::EPSILON,
            ..recessed
        };
        assert_eq!(
            guard_palm_peg_axial_clearance(&tool, &peg, old_central_flange),
            Err("palm_peg_clearance_risk")
        );
    }

    #[test]
    fn jaw_socket_gate_accepts_offset_tool_and_rejects_centered_false_pass() {
        let mut tool = estimate();
        tool.position_world_m[2] = -1.250e-3;
        let socket = estimate();
        let policy = JawSocketAxialClearancePolicy {
            jaw_axial_half_length_m: 0.200e-3,
            jaw_transverse_radius_m: 0.463e-3,
            side_target_forward_extent_m: 0.350e-3,
            side_target_transverse_radius_m: 0.709e-3,
            socket_depth_m: 0.800e-3,
            minimum_clearance_m: 0.100e-3,
        };
        let motion = JawSocketMotionPreview {
            commanded_translation_world_m: [0.0; 3],
            anticipated_tool_position_sigma_m: 6.0e-6,
            anticipated_tool_axis_sigma_rad: 0.006,
            commanded_tool_axis_change_bound_rad: 0.002,
            path_deviation_bound_m: 0.0,
        };
        let accepted = guard_jaw_socket_axial_clearance(&tool, &socket, motion, policy).unwrap();
        assert!(accepted.minimum_clearance_m >= policy.minimum_clearance_m);
        assert!(
            accepted.projected_side_target_forward_extent_m > accepted.projected_jaw_half_extent_m
        );
        assert_eq!(
            accepted.projected_tool_forward_extent_m,
            accepted.projected_side_target_forward_extent_m
        );

        tool.position_world_m[2] = 0.0;
        assert_eq!(
            guard_jaw_socket_axial_clearance(&tool, &socket, motion, policy),
            Err("jaw_socket_clearance_risk")
        );
    }

    #[test]
    fn side_target_closes_the_short_jaw_false_pass() {
        let mut tool = estimate();
        tool.position_world_m[2] = -0.750e-3;
        tool.position_sigma_m = 0.0;
        tool.axis_sigma_rad = 0.0;
        let mut socket = estimate();
        socket.position_sigma_m = 0.0;
        socket.axis_sigma_rad = 0.0;
        let target_aware = JawSocketAxialClearancePolicy {
            jaw_axial_half_length_m: 0.200e-3,
            jaw_transverse_radius_m: 0.463e-3,
            side_target_forward_extent_m: 0.350e-3,
            side_target_transverse_radius_m: 0.709e-3,
            socket_depth_m: 0.800e-3,
            minimum_clearance_m: 0.100e-3,
        };
        let stationary = JawSocketMotionPreview {
            commanded_translation_world_m: [0.0; 3],
            anticipated_tool_position_sigma_m: 0.0,
            anticipated_tool_axis_sigma_rad: 0.0,
            commanded_tool_axis_change_bound_rad: 0.0,
            path_deviation_bound_m: 0.0,
        };
        assert_eq!(
            guard_jaw_socket_axial_clearance(&tool, &socket, stationary, target_aware),
            Err("jaw_socket_clearance_risk")
        );

        // This emulates the previous jaw-only proxy. It would have reported
        // 0.150 mm clearance and incorrectly authorized the same pose.
        let jaw_only = JawSocketAxialClearancePolicy {
            side_target_forward_extent_m: target_aware.jaw_axial_half_length_m,
            side_target_transverse_radius_m: target_aware.jaw_transverse_radius_m,
            ..target_aware
        };
        let legacy_false_pass =
            guard_jaw_socket_axial_clearance(&tool, &socket, stationary, jaw_only).unwrap();
        assert!((legacy_false_pass.minimum_clearance_m - 0.150e-3).abs() < 1.0e-15);
    }

    #[test]
    fn commanded_axis_sweep_bound_can_change_clearance_to_rejection() {
        let mut tool = estimate();
        tool.position_world_m[2] = -0.900e-3;
        tool.position_sigma_m = 0.0;
        tool.axis_sigma_rad = 0.0;
        let mut socket = estimate();
        socket.position_sigma_m = 0.0;
        socket.axis_sigma_rad = 0.0;
        let policy = JawSocketAxialClearancePolicy {
            jaw_axial_half_length_m: 0.200e-3,
            jaw_transverse_radius_m: 0.463e-3,
            side_target_forward_extent_m: 0.350e-3,
            side_target_transverse_radius_m: 0.709e-3,
            socket_depth_m: 0.800e-3,
            minimum_clearance_m: 0.100e-3,
        };
        let stationary = JawSocketMotionPreview {
            commanded_translation_world_m: [0.0; 3],
            anticipated_tool_position_sigma_m: 0.0,
            anticipated_tool_axis_sigma_rad: 0.0,
            commanded_tool_axis_change_bound_rad: 0.0,
            path_deviation_bound_m: 0.0,
        };
        assert!(guard_jaw_socket_axial_clearance(&tool, &socket, stationary, policy).is_ok());
        let rotating = JawSocketMotionPreview {
            commanded_tool_axis_change_bound_rad: 0.20,
            ..stationary
        };
        assert_eq!(
            guard_jaw_socket_axial_clearance(&tool, &socket, rotating, policy),
            Err("jaw_socket_clearance_risk")
        );
    }

    #[test]
    fn forged_non_finite_estimate_and_contact_data_fail_closed() {
        let mut forged = estimate();
        forged.position_sigma_m = f64::NAN;
        assert_eq!(
            guard_estimate(155, &forged, gate()),
            Err(EstimateGuardFailure::InvalidEstimate)
        );

        let evidence = ClassifiedContactEvidence {
            captured_at_tick: 155,
            classified_at_tick: 155,
            state: ContactState::LeadInContact,
            contact_detected: false,
            bilateral_jaw_contact: true,
            left_pad_deflection_m: f64::NAN,
            right_pad_deflection_m: 4.0e-6,
            grip_force_proxy_n: 0.02,
            insertion_force_proxy_n: 0.0,
            relative_mating_pose: None,
        };
        assert_eq!(
            guard_grasp_evidence(
                evidence,
                GraspEvidenceGate {
                    minimum_pad_deflection_m: 2.0e-6,
                    maximum_pad_deflection_m: 24.0e-6,
                    minimum_force_n: 0.005,
                    maximum_force_n: 0.150,
                }
            ),
            Err("invalid_grasp_evidence")
        );
    }

    #[test]
    fn contact_classifier_assigns_states_from_raw_channels_and_observed_geometry() {
        let cases = [
            (false, 500.0e-6, 0.0, 0.0, ContactState::FreeMotion),
            (true, 80.0e-6, 20.0e-6, 0.005, ContactState::LeadInContact),
            (true, 12.0e-6, 20.0e-6, 0.005, ContactState::Seated),
            (
                true,
                80.0e-6,
                70.0e-6,
                0.005,
                ContactState::RecoverableLateralContact,
            ),
            (
                true,
                80.0e-6,
                110.0e-6,
                0.005,
                ContactState::ExcessiveInterference,
            ),
            (true, 80.0e-6, 150.0e-6, 0.005, ContactState::Jammed),
        ];
        for (detected, axial, lateral, axis, expected) in cases {
            let mut packet = contact_packet();
            packet.contact_detected = detected;
            if !detected {
                packet.insertion_force_proxy_n = 0.0;
            }
            let mut relative = relative_pose();
            relative.axial_error_m = axial;
            relative.lateral_error_m = lateral;
            relative.axis_error_rad = axis;
            let classified =
                classify_contact_packet(155, packet, Some(relative), contact_policy()).unwrap();
            assert_eq!(classified.state, expected);
            assert!(classified.bilateral_jaw_contact);
        }
    }

    #[test]
    fn contact_classifier_fails_closed_for_invalid_stale_and_impossible_inputs() {
        struct Case {
            name: &'static str,
            now_tick: u64,
            packet: ContactPacket,
            relative: Option<RelativeMatingPose>,
            policy: ContactClassificationPolicy,
            expected: ContactClassificationFailure,
        }
        let mut non_finite = contact_packet();
        non_finite.insertion_force_proxy_n = f64::NAN;
        let mut packet_from_future = contact_packet();
        packet_from_future.captured_at_tick = 156;
        let mut stale_packet = contact_packet();
        stale_packet.captured_at_tick = 140;
        let mut invalid_geometry = relative_pose();
        invalid_geometry.lateral_error_m = f64::INFINITY;
        let mut geometry_from_future = relative_pose();
        geometry_from_future.available_at_tick = 156;
        let mut stale_geometry = relative_pose();
        stale_geometry.captured_at_tick = 130;
        let mut impossible_geometry = relative_pose();
        impossible_geometry.axial_error_m = 301.0e-6;
        let mut uncertain_geometry = relative_pose();
        uncertain_geometry.position_sigma_m = 11.0e-6;
        let mut force_limit = contact_packet();
        force_limit.insertion_force_proxy_n = 0.081;
        let mut invalid_policy = contact_policy();
        invalid_policy.maximum_lateral_error_m = 50.0e-6;
        let cases = [
            Case {
                name: "invalid policy",
                now_tick: 155,
                packet: contact_packet(),
                relative: Some(relative_pose()),
                policy: invalid_policy,
                expected: ContactClassificationFailure::InvalidPolicy,
            },
            Case {
                name: "non-finite packet",
                now_tick: 155,
                packet: non_finite,
                relative: Some(relative_pose()),
                policy: contact_policy(),
                expected: ContactClassificationFailure::InvalidPacket,
            },
            Case {
                name: "future packet",
                now_tick: 155,
                packet: packet_from_future,
                relative: Some(relative_pose()),
                policy: contact_policy(),
                expected: ContactClassificationFailure::ContactPacketFromFuture,
            },
            Case {
                name: "stale packet",
                now_tick: 155,
                packet: stale_packet,
                relative: Some(relative_pose()),
                policy: contact_policy(),
                expected: ContactClassificationFailure::StaleContactPacket,
            },
            Case {
                name: "invalid observed geometry",
                now_tick: 155,
                packet: contact_packet(),
                relative: Some(invalid_geometry),
                policy: contact_policy(),
                expected: ContactClassificationFailure::InvalidRelativeGeometry,
            },
            Case {
                name: "future observed geometry",
                now_tick: 155,
                packet: contact_packet(),
                relative: Some(geometry_from_future),
                policy: contact_policy(),
                expected: ContactClassificationFailure::RelativeGeometryFromFuture,
            },
            Case {
                name: "stale observed geometry",
                now_tick: 155,
                packet: contact_packet(),
                relative: Some(stale_geometry),
                policy: contact_policy(),
                expected: ContactClassificationFailure::StaleRelativeGeometry,
            },
            Case {
                name: "contact without geometry",
                now_tick: 155,
                packet: contact_packet(),
                relative: None,
                policy: contact_policy(),
                expected: ContactClassificationFailure::InconsistentContactGeometry,
            },
            Case {
                name: "excessive relative uncertainty",
                now_tick: 155,
                packet: contact_packet(),
                relative: Some(uncertain_geometry),
                policy: contact_policy(),
                expected: ContactClassificationFailure::ExcessiveRelativeUncertainty,
            },
            Case {
                name: "contact outside lead-in",
                now_tick: 155,
                packet: contact_packet(),
                relative: Some(impossible_geometry),
                policy: contact_policy(),
                expected: ContactClassificationFailure::InconsistentContactGeometry,
            },
            Case {
                name: "force limit",
                now_tick: 155,
                packet: force_limit,
                relative: Some(relative_pose()),
                policy: contact_policy(),
                expected: ContactClassificationFailure::ContactForceLimit,
            },
        ];
        for case in cases {
            assert_eq!(
                classify_contact_packet(case.now_tick, case.packet, case.relative, case.policy),
                Err(case.expected),
                "{}",
                case.name
            );
        }
    }

    #[test]
    fn correction_is_bounded_and_deterministic() {
        let policy = CorrectionPolicy {
            gain: 0.8,
            convergence_m: 5.0e-6,
            maximum_magnitude_m: 100.0e-6,
            minimum_reproducible_m: 2.0e-6,
        };
        let first = decide_correction([0.0; 3], [50.0e-6, -25.0e-6, 0.0], policy);
        let second = decide_correction([0.0; 3], [50.0e-6, -25.0e-6, 0.0], policy);
        assert_eq!(first, second);
        assert!(matches!(first, CorrectionDecision::Command { .. }));
    }

    #[test]
    fn correction_deadbands_sub_floor_axes_without_crossing_the_target() {
        let decision = decide_correction(
            [0.0; 3],
            [10.0e-6, 1.0e-6, -2.0e-6],
            CorrectionPolicy {
                gain: 0.8,
                convergence_m: 3.0e-6,
                maximum_magnitude_m: 50.0e-6,
                minimum_reproducible_m: 5.0e-6,
            },
        );
        let CorrectionDecision::Command {
            correction_world_m, ..
        } = decision
        else {
            panic!("expected a conditioned command");
        };
        assert!((correction_world_m[0] - 8.0e-6).abs() < 1.0e-18);
        assert_eq!(correction_world_m[1..], [0.0, 0.0]);
        assert!(correction_world_m[0] <= 10.0e-6);
    }

    #[test]
    fn relative_pose_gate_does_not_assume_unmodeled_common_mode_cancellation() {
        let mut first = estimate();
        let mut second = estimate();
        first.position_sigma_m = 8.0e-6;
        second.position_sigma_m = 8.0e-6;
        assert_eq!(
            guard_relative_estimates(&first, &second, 12.0e-6, 0.03),
            Err(EstimateGuardFailure::ExcessivePositionUncertainty)
        );
        assert_eq!(
            guard_relative_estimates(&first, &second, 15.0e-6, 0.03),
            Err(EstimateGuardFailure::ExcessivePositionUncertainty)
        );
        let accepted = guard_relative_estimates(&first, &second, 16.0e-6, 0.03).unwrap();
        assert!((accepted.position_sigma_m - 16.0e-6).abs() < 1.0e-15);
    }

    #[test]
    fn controller_decision_depends_on_observation_not_latent_world() {
        // The pure API has no plant argument. Two hypothetical plants replaying
        // this identical controller-visible transcript must yield one decision.
        let transcript = estimate();
        let a = decide_correction(
            transcript.position_world_m,
            [12.0e-6, 0.0, 0.0],
            CorrectionPolicy {
                gain: 1.0,
                convergence_m: 5.0e-6,
                maximum_magnitude_m: 50.0e-6,
                minimum_reproducible_m: 2.0e-6,
            },
        );
        let b = decide_correction(
            transcript.position_world_m,
            [12.0e-6, 0.0, 0.0],
            CorrectionPolicy {
                gain: 1.0,
                convergence_m: 5.0e-6,
                maximum_magnitude_m: 50.0e-6,
                minimum_reproducible_m: 2.0e-6,
            },
        );
        assert_eq!(a, b);
    }

    #[test]
    fn gripper_and_carried_part_envelopes_are_checked_in_stable_order() {
        let obstacles = [
            PlanningObstacle {
                id: 9,
                center_world_m: [3.0e-3, 0.0, 0.0],
                conservative_radius_m: 0.2e-3,
                position_sigma_m: 2.0e-6,
            },
            PlanningObstacle {
                id: 2,
                center_world_m: [4.0e-3, 0.0, 0.0],
                conservative_radius_m: 0.2e-3,
                position_sigma_m: 2.0e-6,
            },
        ];
        let checks = preflight_swept_envelope(
            SweptEnvelope {
                center_start_world_m: [0.0, 0.0, 0.0],
                center_end_world_m: [0.0, 1.0e-3, 0.0],
                radius_m: 0.5e-3,
                position_sigma_m: 2.0e-6,
                hard_position_bound_m: 0.0,
                path_deviation_bound_m: 0.0,
            },
            &obstacles,
            0.1e-3,
        )
        .unwrap();
        assert_eq!(checks[0].obstacle_id, 2);
        assert_eq!(checks[1].obstacle_id, 9);
    }

    #[test]
    fn independent_tool_and_carried_sweeps_preserve_their_distinct_origins() {
        let obstacle = PlanningObstacle {
            id: 21,
            center_world_m: [0.0, 0.0, -1.90e-3],
            conservative_radius_m: 0.10e-3,
            position_sigma_m: 0.0,
        };
        let tool = SweptEnvelope {
            center_start_world_m: [0.0, 0.0, 0.0],
            center_end_world_m: [0.0, 1.0e-3, 0.0],
            radius_m: 1.80e-3,
            position_sigma_m: 0.0,
            hard_position_bound_m: 0.0,
            path_deviation_bound_m: 0.0,
        };
        let carried_peg = SweptEnvelope {
            center_start_world_m: [0.0, 0.0, 0.75e-3],
            center_end_world_m: [0.0, 1.0e-3, 0.75e-3],
            radius_m: 0.95e-3,
            position_sigma_m: 0.0,
            hard_position_bound_m: 0.0,
            path_deviation_bound_m: 0.0,
        };
        let required_clearance_m = 0.10e-3;

        assert!(matches!(
            preflight_swept_envelope(tool, &[obstacle], required_clearance_m),
            Err(SweptPreflightFailure::Clearance(check)) if check.obstacle_id == obstacle.id
        ));
        assert!(preflight_swept_envelope(carried_peg, &[obstacle], required_clearance_m).is_ok());

        // The pre-fix pseudo-union moved the larger tool radius to the PEG
        // center. It would incorrectly pass this obstacle behind the tool.
        let recentered_pseudo_union = SweptEnvelope {
            radius_m: tool.radius_m.max(carried_peg.radius_m),
            ..carried_peg
        };
        assert!(preflight_swept_envelope(
            recentered_pseudo_union,
            &[obstacle],
            required_clearance_m
        )
        .is_ok());
    }

    #[test]
    fn non_finite_or_negative_sweeps_fail_closed_even_without_obstacles() {
        let result = preflight_swept_envelope(
            SweptEnvelope {
                center_start_world_m: [0.0, f64::NAN, 0.0],
                center_end_world_m: [0.0; 3],
                radius_m: 0.5e-3,
                position_sigma_m: 2.0e-6,
                hard_position_bound_m: 0.0,
                path_deviation_bound_m: 0.0,
            },
            &[],
            0.1e-3,
        );
        assert_eq!(result, Err(SweptPreflightFailure::InvalidGeometry));
    }
}
