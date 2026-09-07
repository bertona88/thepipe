use super::*;
use crate::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MetrologyHealth {
    pub calibration_id: String,
    pub measured_reference_rms_m: Option<f64>,
    pub maximum_reference_error_m: Option<f64>,
    pub reference_count: usize,
    pub oldest_reference_acquisition_s: Option<f64>,
    pub available_s: f64,
    pub temperature_delta_k: f64,
    pub calibration_valid: bool,
    pub thermal_valid: bool,
    pub reference_source: Option<ObservationSource>,
}
/// Independent permanent-reference residuals in the maintained world frame.
/// No best-fit registration is applied here: that would hide frame drift.
pub fn evaluate_health(
    config: &MetrologyConfig,
    measured: &[ObservedPoint],
    now_s: f64,
) -> MetrologyHealth {
    let mut squared = 0.0;
    let mut maximum: f64 = 0.0;
    let mut count = 0;
    let mut oldest = f64::INFINITY;
    let mut keys = BTreeSet::new();
    let mut source = None;
    for p in measured {
        if p.object_id != REFERENCE_OBJECT_ID
            || p.quality.calibration_id != config.calibration.id
            || p.quality.available_s > now_s
            || !nonnegative(p.quality.available_s)
            || !nonnegative(p.quality.oldest_acquisition_s)
            || p.quality.oldest_acquisition_s > p.quality.available_s
            || source.is_some_and(|s| s != p.quality.source)
            || p.quality.source == ObservationSource::IdealSyntheticFeature
            || !p.position_world_m.is_finite()
            || invert_spd(p.covariance_m2).is_none()
            || !keys.insert(p.feature_id)
        {
            continue;
        }
        if let Some(r) = config
            .references
            .iter()
            .find(|r| r.feature_id == p.feature_id)
        {
            let expected = config
                .calibration
                .world_from_tube
                .transform_point(r.point_tube_m);
            let error = (p.position_world_m - expected).norm();
            squared += error * error;
            maximum = maximum.max(error);
            count += 1;
            source = Some(p.quality.source);
            oldest = oldest.min(p.quality.oldest_acquisition_s);
        }
    }
    let reference_k = config.calibration.reference_temperature_k;
    let delta = config
        .devices
        .iter()
        .map(|d| (d.temperature_k - reference_k).abs())
        .chain(
            [
                config.thermal.structure_temperature_k,
                config.thermal.tube_temperature_k,
                config.thermal.reference_temperature_k,
            ]
            .iter()
            .map(|t| (t - reference_k).abs()),
        )
        .fold(0.0, f64::max);
    MetrologyHealth {
        calibration_id: config.calibration.id.clone(),
        measured_reference_rms_m: (count > 0).then(|| (squared / count as f64).sqrt()),
        maximum_reference_error_m: (count > 0).then_some(maximum),
        reference_count: count,
        oldest_reference_acquisition_s: (count > 0).then_some(oldest),
        available_s: now_s,
        temperature_delta_k: delta,
        calibration_valid: config.validate().is_ok() && config.calibration.valid_at(now_s),
        thermal_valid: delta <= config.thermal.maximum_calibrated_delta_k,
        reference_source: source,
    }
}
pub const REFERENCE_OBJECT_ID: u32 = 4_000_000_000;
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrecisionContract {
    pub maximum_tool_3d_rms_m: f64,
    pub maximum_target_3d_rms_m: f64,
    pub maximum_orientation_rms_rad: f64,
    pub minimum_usable_cameras: usize,
    pub minimum_reference_count: usize,
    pub maximum_reference_rms_m: f64,
    pub maximum_reference_error_m: f64,
    pub maximum_age_s: f64,
    pub maximum_tool_target_skew_s: f64,
    /// A simulation gate can admit noisy synthetic data; a physical gate cannot.
    pub require_physical_observations: bool,
}
impl Default for PrecisionContract {
    fn default() -> Self {
        Self {
            maximum_tool_3d_rms_m: 5e-6,
            maximum_target_3d_rms_m: 5e-6,
            maximum_orientation_rms_rad: 0.003,
            minimum_usable_cameras: 2,
            minimum_reference_count: 3,
            maximum_reference_rms_m: 5e-6,
            maximum_reference_error_m: 8e-6,
            maximum_age_s: 0.10,
            maximum_tool_target_skew_s: 0.01,
            require_physical_observations: false,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PrecisionRejection {
    MissingState,
    InvalidContract,
    InvalidCovariance,
    ExcessiveUncertainty,
    InsufficientViews,
    Stale,
    Unavailable,
    InvalidCalibration,
    UnhealthyReferences,
    ThermalDrift,
    OutsidePrecisionVolume,
    WrongMode,
    InvalidProvenance,
    TemporalMismatch,
    InvalidGeometry,
}
/// Fail-closed control contract. It grants optical support only; collision,
/// contact force and mechanical command limits still apply separately.
pub fn require_precision(
    config: &MetrologyConfig,
    policy: &PrecisionContract,
    tool: &ObservedToolPose,
    target: &ObservedPoint,
    health: &MetrologyHealth,
    now_s: f64,
) -> Result<(), PrecisionRejection> {
    if !nonnegative(now_s)
        || ![
            policy.maximum_tool_3d_rms_m,
            policy.maximum_target_3d_rms_m,
            policy.maximum_orientation_rms_rad,
            policy.maximum_reference_rms_m,
            policy.maximum_reference_error_m,
            policy.maximum_age_s,
            policy.maximum_tool_target_skew_s,
        ]
        .iter()
        .all(|&v| positive(v))
        || policy.minimum_usable_cameras < 2
        || policy.minimum_reference_count < 3
    {
        return Err(PrecisionRejection::InvalidContract);
    }
    if config.validate().is_err()
        || !config.calibration.valid_at(now_s)
        || !health.calibration_valid
        || health.calibration_id != config.calibration.id
    {
        return Err(PrecisionRejection::InvalidCalibration);
    }
    if policy.require_physical_observations
        && config.calibration.provenance != CalibrationProvenance::PhysicalArtifactFit
    {
        return Err(PrecisionRejection::InvalidProvenance);
    }
    if !valid_transform(tool.world_from_tcp)
        || !valid_transform(tool.world_from_fiducial)
        || !target.position_world_m.is_finite()
    {
        return Err(PrecisionRejection::InvalidGeometry);
    }
    if invert_spd(tool.tcp_covariance).is_none() || invert_spd(target.covariance_m2).is_none() {
        return Err(PrecisionRejection::InvalidCovariance);
    }
    if !health.thermal_valid
        || !nonnegative(health.temperature_delta_k)
        || health.temperature_delta_k > config.thermal.maximum_calibrated_delta_k
    {
        return Err(PrecisionRejection::ThermalDrift);
    }
    if health.reference_source.is_none()
        || health.reference_source == Some(ObservationSource::IdealSyntheticFeature)
        || (policy.require_physical_observations
            && health.reference_source != Some(ObservationSource::PhysicalImageFeature))
    {
        return Err(PrecisionRejection::InvalidProvenance);
    }
    let temperatures = [
        config.thermal.structure_temperature_k,
        config.thermal.tube_temperature_k,
        config.thermal.reference_temperature_k,
    ];
    if temperatures
        .iter()
        .chain(config.devices.iter().map(|d| &d.temperature_k))
        .any(|t| {
            (*t - config.calibration.reference_temperature_k).abs()
                > config.thermal.maximum_calibrated_delta_k
        })
    {
        return Err(PrecisionRejection::ThermalDrift);
    }
    let (Some(rms), Some(maximum), Some(time)) = (
        health.measured_reference_rms_m,
        health.maximum_reference_error_m,
        health.oldest_reference_acquisition_s,
    ) else {
        return Err(PrecisionRejection::UnhealthyReferences);
    };
    if health.reference_count < policy.minimum_reference_count
        || !nonnegative(rms)
        || rms > policy.maximum_reference_rms_m
        || !nonnegative(maximum)
        || maximum > policy.maximum_reference_error_m
    {
        return Err(PrecisionRejection::UnhealthyReferences);
    }
    if !nonnegative(time) || time > now_s || now_s - time > policy.maximum_age_s {
        return Err(PrecisionRejection::Stale);
    }
    if !nonnegative(health.available_s) || health.available_s > now_s {
        return Err(PrecisionRejection::Unavailable);
    }
    for q in [&tool.quality, &target.quality] {
        if q.mode != MeasurementMode::Precision {
            return Err(PrecisionRejection::WrongMode);
        }
        if q.source == ObservationSource::IdealSyntheticFeature
            || (policy.require_physical_observations
                && q.source != ObservationSource::PhysicalImageFeature)
        {
            return Err(PrecisionRejection::InvalidProvenance);
        }
        if q.calibration_id != config.calibration.id {
            return Err(PrecisionRejection::InvalidCalibration);
        }
        if q.usable_camera_count < policy.minimum_usable_cameras
            || q.supporting_observations.is_empty()
        {
            return Err(PrecisionRejection::InsufficientViews);
        }
        let camera_ids = q
            .supporting_observations
            .iter()
            .filter_map(|o| config.device(o.sensor_id).ok())
            .filter(|d| d.kind == DeviceKind::Camera)
            .map(|d| d.id)
            .collect::<BTreeSet<_>>();
        if camera_ids.len() != q.usable_camera_count
            || q.best_triangulation_angle_rad < config.minimum_angle_rad
            || !positive(q.best_triangulation_angle_rad)
            || !positive(q.maximum_baseline_m)
        {
            return Err(PrecisionRejection::InsufficientViews);
        }
        if !nonnegative(q.available_s) || q.available_s > now_s {
            return Err(PrecisionRejection::Unavailable);
        }
        if !nonnegative(q.oldest_acquisition_s)
            || !nonnegative(q.newest_acquisition_s)
            || q.newest_acquisition_s < q.oldest_acquisition_s
            || q.newest_acquisition_s > q.available_s
            || now_s - q.oldest_acquisition_s > policy.maximum_age_s
        {
            return Err(PrecisionRejection::Stale);
        }
        if !nonnegative(q.normalized_residual_rms)
            || q.normalized_residual_rms > config.maximum_normalized_residual
        {
            return Err(PrecisionRejection::InvalidProvenance);
        }
    }
    if tool.quality.class != MeasurementClass::ToolPose
        || target.quality.class != MeasurementClass::CriticalFeature
    {
        return Err(PrecisionRejection::WrongMode);
    }
    if (tool.quality.newest_acquisition_s - target.quality.newest_acquisition_s).abs()
        > policy.maximum_tool_target_skew_s
    {
        return Err(PrecisionRejection::TemporalMismatch);
    }
    if tool.predicted_tcp_3d_rms_m() > policy.maximum_tool_3d_rms_m
        || target.predicted_3d_rms_m() > policy.maximum_target_3d_rms_m
        || tool.orientation_rms_rad() > policy.maximum_orientation_rms_rad
    {
        return Err(PrecisionRejection::ExcessiveUncertainty);
    }
    if !config
        .precision_volume
        .contains(tool.world_from_tcp.translation)
        || !config.precision_volume.contains(target.position_world_m)
    {
        return Err(PrecisionRejection::OutsidePrecisionVolume);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservedEntityKind {
    Arm,
    Tcp,
    Gripper,
    Tool,
    Part,
    CriticalFeature,
    Obstacle,
    CalibrationReference,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObservedEntity {
    pub kind: ObservedEntityKind,
    pub point: Option<ObservedPoint>,
    pub pose: Option<ObservedToolPose>,
}
/// Observation state is deliberately separate from plant/commanded/encoder
/// state. Updates replace previous acquisitions; correlated floors never fuse
/// repeatedly. Missing observations invalidate that entity on every refresh.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ObservedWorld {
    pub entities: BTreeMap<u32, ObservedEntity>,
    pub health: Option<MetrologyHealth>,
    pub occupancy: Option<OccupancyVolume>,
    pub observation_epoch: u64,
}
impl ObservedWorld {
    pub fn begin_acquisition(&mut self, epoch: u64) {
        self.entities.clear();
        self.health = None;
        self.occupancy = None;
        self.observation_epoch = epoch;
    }
    pub fn require_operation(
        &self,
        config: &MetrologyConfig,
        policy: &PrecisionContract,
        tool_id: u32,
        target_id: u32,
        now_s: f64,
    ) -> Result<(), PrecisionRejection> {
        let tool = self
            .entities
            .get(&tool_id)
            .and_then(|e| e.pose.as_ref())
            .ok_or(PrecisionRejection::MissingState)?;
        let target = self
            .entities
            .get(&target_id)
            .and_then(|e| e.point.as_ref())
            .ok_or(PrecisionRejection::MissingState)?;
        let health = self
            .health
            .as_ref()
            .ok_or(PrecisionRejection::UnhealthyReferences)?;
        require_precision(config, policy, tool, target, health, now_s)
    }
}

/// A local axis feature from two independently reconstructed section centres.
/// Shared calibration floors are retained, with no full-surface reconstruction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObservedAxisFeature {
    pub origin_world_m: Vec3,
    pub direction_world: Vec3,
    pub angular_covariance_rad2: [[f64; 3]; 3],
    pub supports: Vec<ObservedPoint>,
}
pub fn fit_axis_feature(
    a: ObservedPoint,
    b: ObservedPoint,
) -> Result<ObservedAxisFeature, MetrologyError> {
    if a.object_id != b.object_id
        || a.feature_id == b.feature_id
        || a.quality.calibration_id != b.quality.calibration_id
        || a.quality.mode != b.quality.mode
        || a.quality.source != b.quality.source
        || (a.quality.newest_acquisition_s - b.quality.newest_acquisition_s).abs() > 1e-6
        || invert_spd(a.covariance_m2).is_none()
        || invert_spd(b.covariance_m2).is_none()
    {
        return Err(MetrologyError::InvalidObservation);
    }
    let delta = b.position_world_m - a.position_world_m;
    let length = delta.norm();
    if length < 1e-6 {
        return Err(MetrologyError::UnobservablePose);
    }
    let axis = delta / length;
    let j = (crate::Mat3::IDENTITY - crate::Mat3::outer(axis, axis)) * (1.0 / length);
    // Conservative: no common-mode cancellation assumed for different features.
    let c = crate::Mat3::new(a.covariance_m2) + crate::Mat3::new(b.covariance_m2);
    Ok(ObservedAxisFeature {
        origin_world_m: a.position_world_m,
        direction_world: axis,
        angular_covariance_rad2: (j * c * j.transpose()).m,
        supports: vec![a, b],
    })
}
