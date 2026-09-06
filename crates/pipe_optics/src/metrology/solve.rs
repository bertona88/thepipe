use super::*;
use crate::{Mat3, PinholeCamera, RigidTransform, Vec2, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

type NormalSystem<const N: usize> = ([[f64; N]; N], [f64; N], f64);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationSource {
    SyntheticNoisyImageFeature,
    IdealSyntheticFeature,
    PhysicalImageFeature,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PixelObservation {
    pub object_id: u32,
    pub feature_id: u32,
    pub sensor_id: u32,
    pub pixel: Vec2,
    pub sigma_px: Vec2,
    pub timing: AcquisitionTiming,
    pub mode: MeasurementMode,
    pub class: MeasurementClass,
    pub illumination: Illumination,
    pub calibration_id: String,
    pub source: ObservationSource,
    pub saturated: bool,
    pub decoded_projector: Option<DecodedProjectorPixel>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObservationSupport {
    pub sensor_id: u32,
    pub feature_id: u32,
    pub sequence_id: u64,
    pub trigger_id: u64,
    pub pixel: Vec2,
    pub sigma_px: Vec2,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeasurementQuality {
    pub usable_camera_count: usize,
    pub usable_projector_count: usize,
    pub supporting_observations: Vec<ObservationSupport>,
    pub best_triangulation_angle_rad: f64,
    pub maximum_baseline_m: f64,
    pub sampling_m_per_px: Vec<[f64; 2]>,
    pub normalized_residual_rms: f64,
    pub confidence: f64,
    pub partial_visibility: bool,
    pub available_s: f64,
    pub oldest_acquisition_s: f64,
    pub newest_acquisition_s: f64,
    pub mode: MeasurementMode,
    pub class: MeasurementClass,
    pub calibration_id: String,
    pub source: ObservationSource,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObservedPoint {
    pub object_id: u32,
    pub feature_id: u32,
    pub position_world_m: Vec3,
    pub random_covariance_m2: [[f64; 3]; 3],
    pub covariance_m2: [[f64; 3]; 3],
    pub quality: MeasurementQuality,
}
impl ObservedPoint {
    /// sqrt(trace), the RMS norm of the 3-D position error, NOT per-axis RMS.
    pub fn predicted_3d_rms_m(&self) -> f64 {
        trace3(self.covariance_m2).sqrt()
    }
    pub fn sigma_xyz_m(&self) -> [f64; 3] {
        [
            self.covariance_m2[0][0].sqrt(),
            self.covariance_m2[1][1].sqrt(),
            self.covariance_m2[2][2].sqrt(),
        ]
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetrologyFeature {
    pub id: u32,
    pub point_fiducial_m: Vec3,
    pub diameter_m: f64,
    /// None represents a spherical, multi-face marker, not a two-sided flat print.
    pub normal_fiducial: Option<Vec3>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RigidFiducial {
    pub object_id: u32,
    pub arm_id: Option<u32>,
    pub tool_type: String,
    pub identity_code: String,
    pub geometry_calibration_id: String,
    pub features: Vec<MetrologyFeature>,
    /// T_fiducial_to_TCP maps fiducial coordinates INTO TCP coordinates.
    /// world_from_tcp = world_from_fiducial * inverse(tcp_from_fiducial).
    pub tcp_from_fiducial: RigidTransform,
}
impl RigidFiducial {
    pub fn validate(&self) -> Result<(), MetrologyError> {
        let mut ids = BTreeSet::new();
        if self.object_id == 0
            || self.identity_code.is_empty()
            || self.geometry_calibration_id.is_empty()
            || self.features.len() < 3
            || !valid_transform(self.tcp_from_fiducial)
            || self.features.iter().any(|f| {
                !ids.insert(f.id)
                    || !f.point_fiducial_m.is_finite()
                    || !positive(f.diameter_m)
                    || f.normal_fiducial
                        .is_some_and(|n| !n.is_finite() || (n.norm() - 1.0).abs() > 1e-6)
            })
        {
            return Err(config_error("invalid calibrated rigid fiducial"));
        }
        Ok(())
    }
    pub fn baseline(object_id: u32) -> Self {
        Self {
            object_id,
            arm_id: Some(object_id),
            tool_type: "distal_gripper".into(),
            identity_code: format!("tool-{object_id}"),
            geometry_calibration_id: "synthetic_calibration_v1".into(),
            features: [
                Vec3::new(-0.002, -0.001, 0.0),
                Vec3::new(0.002, -0.001, 0.0),
                Vec3::new(-0.001, 0.002, 0.0005),
                Vec3::new(0.0015, 0.0015, -0.0005),
                Vec3::new(0.0, -0.0005, 0.0015),
            ]
            .iter()
            .enumerate()
            .map(|(i, &p)| MetrologyFeature {
                id: i as u32 + 1,
                point_fiducial_m: p,
                diameter_m: 0.0003,
                normal_fiducial: None,
            })
            .collect(),
            tcp_from_fiducial: RigidTransform::new(Mat3::IDENTITY, Vec3::new(0.0, 0.0, -0.003)),
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ObservedToolPose {
    pub object_id: u32,
    pub identity_code: String,
    pub world_from_fiducial: RigidTransform,
    pub world_from_tcp: RigidTransform,
    /// [tx,ty,tz,rx,ry,rz], world-axis perturbations about the indicated origin.
    /// Translation m², rotation rad², cross terms m·rad.
    pub fiducial_covariance: [[f64; 6]; 6],
    pub tcp_covariance: [[f64; 6]; 6],
    pub quality: MeasurementQuality,
}
impl ObservedToolPose {
    pub fn predicted_tcp_3d_rms_m(&self) -> f64 {
        (self.tcp_covariance[0][0] + self.tcp_covariance[1][1] + self.tcp_covariance[2][2]).sqrt()
    }
    pub fn orientation_rms_rad(&self) -> f64 {
        (self.tcp_covariance[3][3] + self.tcp_covariance[4][4] + self.tcp_covariance[5][5]).sqrt()
    }
}

fn validate_observations(
    config: &MetrologyConfig,
    observations: &[PixelObservation],
) -> Result<(), MetrologyError> {
    config.validate()?;
    let first = observations
        .first()
        .ok_or(MetrologyError::InsufficientViews)?;
    let mut keys = BTreeSet::new();
    for o in observations {
        let d = config.device(o.sensor_id)?;
        o.timing.validate()?;
        if !keys.insert((
            o.sensor_id,
            o.feature_id,
            o.timing.sequence_id,
            o.timing.trigger_id,
        )) {
            return Err(MetrologyError::DuplicateObservation);
        }
        if o.object_id != first.object_id
            || !o.pixel.is_finite()
            || !positive(o.sigma_px.x)
            || !positive(o.sigma_px.y)
            || !d.model.image_size.contains(o.pixel)
            || o.source != first.source
            || o.mode != first.mode
            || o.class != first.class
            || !positive(o.illumination.wavelength_m)
        {
            return Err(MetrologyError::InvalidObservation);
        }
        if o.calibration_id != config.calibration.id
            || !config.calibration.valid_at(o.timing.available_s)
        {
            return Err(MetrologyError::InvalidCalibration);
        }
        if o.saturated {
            return Err(MetrologyError::Saturation);
        }
        if o.timing.sequence_id != first.timing.sequence_id
            || o.timing.trigger_id != first.timing.trigger_id
            || (o.timing.frame_timestamp_s - first.timing.frame_timestamp_s).abs()
                > config.acquisition.max_trigger_skew_s
            || o.timing.trigger_offset_s.abs() > config.acquisition.max_trigger_skew_s
        {
            return Err(MetrologyError::IncompatibleTiming);
        }
        if o.mode == MeasurementMode::Precision
            && (o.timing.motion_evidence == MotionEvidence::Unknown
                || o.timing.speed_bound_m_s * o.timing.exposure_duration_s
                    > config.acquisition.max_exposure_motion_m)
        {
            return Err(MetrologyError::MotionDuringSequence);
        }
        if d.kind == DeviceKind::Projector {
            let decoded = o
                .decoded_projector
                .as_ref()
                .ok_or(MetrologyError::InvalidObservation)?;
            if decoded.pixel != o.pixel
                || decoded.sigma_px != o.sigma_px
                || decoded.sequence_id != o.timing.sequence_id
                || !positive(decoded.modulation)
                || decoded.frame_count != config.patterns.patterns().len()
                || !nonnegative(decoded.sequence_start_s)
                || !positive(decoded.sequence_end_s - decoded.sequence_start_s)
                || decoded.available_s < decoded.sequence_end_s
                || !decoded.available_s.is_finite()
                || o.illumination.kind != IlluminationKind::Structured
            {
                return Err(MetrologyError::InvalidObservation);
            }
            if o.timing.motion_evidence == MotionEvidence::Unknown
                || o.timing.speed_bound_m_s * (decoded.sequence_end_s - decoded.sequence_start_s)
                    > config.acquisition.max_sequence_motion_m
            {
                return Err(MetrologyError::MotionDuringSequence);
            }
        } else if o.decoded_projector.is_some() {
            return Err(MetrologyError::InvalidObservation);
        }
    }
    Ok(())
}

/// Weighted nonlinear multi-view triangulation. The covariance is obtained
/// from the actual distorted projection Jacobian at the reconstructed point.
pub fn reconstruct_point(
    config: &MetrologyConfig,
    observations: &[PixelObservation],
) -> Result<ObservedPoint, MetrologyError> {
    validate_observations(config, observations)?;
    let first = &observations[0];
    if observations
        .iter()
        .any(|o| o.feature_id != first.feature_id)
    {
        return Err(MetrologyError::InvalidObservation);
    }
    let mut obs: Vec<_> = observations.iter().collect();
    obs.sort_by_key(|o| o.sensor_id);
    if obs
        .iter()
        .map(|o| o.sensor_id)
        .collect::<BTreeSet<_>>()
        .len()
        < 2
    {
        return Err(MetrologyError::InsufficientViews);
    }
    let mut initial = None;
    let mut best_angle = 0.0;
    for i in 0..obs.len() {
        for j in i + 1..obs.len() {
            let a = config
                .device(obs[i].sensor_id)?
                .model
                .ray(obs[i].pixel)
                .ok_or(MetrologyError::InvalidObservation)?;
            let b = config
                .device(obs[j].sensor_id)?
                .model
                .ray(obs[j].pixel)
                .ok_or(MetrologyError::InvalidObservation)?;
            if let Some(t) = crate::triangulate_rays(a, b) {
                if t.intersection_angle_rad > best_angle
                    && t.first_distance_m > 0.0
                    && t.second_distance_m > 0.0
                {
                    best_angle = t.intersection_angle_rad;
                    initial = Some(t.point);
                }
            }
        }
    }
    if best_angle < config.minimum_angle_rad {
        return Err(MetrologyError::DegenerateGeometry);
    }
    let mut point = initial.ok_or(MetrologyError::DegenerateGeometry)?;
    for _ in 0..20 {
        let (h, g, _) = point_system(config, &obs, point)?;
        let inv = invert_spd(h).ok_or(MetrologyError::DegenerateGeometry)?;
        let step = mat_vec(inv, g);
        let delta = Vec3::new(step[0], step[1], step[2]);
        if !delta.is_finite() || delta.norm() > 0.01 {
            return Err(MetrologyError::InconsistentObservations);
        }
        point += delta;
        if delta.norm() < 1e-11 {
            break;
        }
    }
    let (h, _, rss) = point_system(config, &obs, point)?;
    let random = invert_spd(h).ok_or(MetrologyError::DegenerateGeometry)?;
    let residual = (rss / (2 * obs.len() - 3) as f64).sqrt();
    if residual > config.maximum_normalized_residual {
        return Err(MetrologyError::InconsistentObservations);
    }
    let quality = quality(config, &obs, point, residual)?;
    let mut cov = random;
    let floor = config
        .errors
        .common_variance_m2(quality.usable_projector_count > 0);
    for (i, row) in cov.iter_mut().enumerate() {
        row[i] += floor;
    }
    Ok(ObservedPoint {
        object_id: first.object_id,
        feature_id: first.feature_id,
        position_world_m: point,
        random_covariance_m2: random,
        covariance_m2: cov,
        quality,
    })
}
fn point_system(
    config: &MetrologyConfig,
    obs: &[&PixelObservation],
    point: Vec3,
) -> Result<NormalSystem<3>, MetrologyError> {
    let mut h = [[0.0; 3]; 3];
    let mut g = [0.0; 3];
    let mut rss = 0.0;
    for o in obs {
        let camera = config.device(o.sensor_id)?.model;
        let prediction = camera
            .project(point)
            .ok_or(MetrologyError::OutsideField)?
            .pixel;
        let j = projection_jacobian(camera, point)?;
        for (axis, (residual, sigma)) in [
            (o.pixel.x - prediction.x, o.sigma_px.x),
            (o.pixel.y - prediction.y, o.sigma_px.y),
        ]
        .iter()
        .enumerate()
        {
            accumulate(&mut h, &mut g, j[axis], *residual, 1.0 / sigma.powi(2));
            rss += (residual / sigma).powi(2);
        }
    }
    Ok((h, g, rss))
}
pub fn projection_jacobian(
    camera: PinholeCamera,
    point: Vec3,
) -> Result<[[f64; 3]; 2], MetrologyError> {
    let mut j = [[0.0; 3]; 2];
    let epsilon_m = 1e-7;
    for (i, axis) in [Vec3::X, Vec3::Y, Vec3::Z].iter().enumerate() {
        let a = camera
            .project(point + *axis * epsilon_m)
            .ok_or(MetrologyError::OutsideField)?
            .pixel;
        let b = camera
            .project(point - *axis * epsilon_m)
            .ok_or(MetrologyError::OutsideField)?
            .pixel;
        let d = (a - b) / (2.0 * epsilon_m);
        j[0][i] = d.x;
        j[1][i] = d.y;
    }
    Ok(j)
}

/// All observed feature subsets constrain the same six-DOF rigid body. Initial
/// pose comes from at least three triangulated, non-collinear features; no
/// commanded, encoder or truth pose is accepted by this API.
pub fn estimate_tool_pose(
    config: &MetrologyConfig,
    model: &RigidFiducial,
    observations: &[PixelObservation],
) -> Result<ObservedToolPose, MetrologyError> {
    model.validate()?;
    validate_observations(config, observations)?;
    if model.geometry_calibration_id != config.calibration.id
        || observations.iter().any(|o| {
            o.object_id != model.object_id || !model.features.iter().any(|f| f.id == o.feature_id)
        })
    {
        return Err(MetrologyError::InvalidCalibration);
    }
    let mut groups: BTreeMap<u32, Vec<PixelObservation>> = BTreeMap::new();
    for o in observations {
        groups.entry(o.feature_id).or_default().push(o.clone());
    }
    let mut pairs = Vec::new();
    for (id, obs) in groups {
        if let Ok(p) = reconstruct_point(config, &obs) {
            pairs.push((
                model
                    .features
                    .iter()
                    .find(|f| f.id == id)
                    .unwrap()
                    .point_fiducial_m,
                p.position_world_m,
            ));
        }
    }
    let mut pose = None;
    let mut best_area = 0.0;
    for i in 0..pairs.len() {
        for j in i + 1..pairs.len() {
            for k in j + 1..pairs.len() {
                let local = [pairs[i].0, pairs[j].0, pairs[k].0];
                let world = [pairs[i].1, pairs[j].1, pairs[k].1];
                let area = (local[1] - local[0]).cross(local[2] - local[0]).norm();
                if area > best_area && area > 1e-10 {
                    if let (Some(a), Some(b)) = (basis(local), basis(world)) {
                        let r = b * a.transpose();
                        pose = Some(RigidTransform::new(r, world[0] - r * local[0]));
                        best_area = area;
                    }
                }
            }
        }
    }
    let mut pose = pose.ok_or(MetrologyError::UnobservablePose)?;
    let mut obs: Vec<_> = observations.iter().collect();
    obs.sort_by_key(|o| (o.sensor_id, o.feature_id));
    for _ in 0..25 {
        let (h, g, _) = pose_system(config, model, &obs, pose)?;
        let inverse = invert_spd(h).ok_or(MetrologyError::UnobservablePose)?;
        let step = mat_vec(inverse, g);
        let dt = Vec3::new(step[0], step[1], step[2]);
        let dr = Vec3::new(step[3], step[4], step[5]);
        if !dt.is_finite() || !dr.is_finite() || dt.norm() > 0.01 || dr.norm() > 0.5 {
            return Err(MetrologyError::InconsistentObservations);
        }
        pose.translation += dt;
        pose.rotation = Mat3::from_axis_angle(dr) * pose.rotation;
        if dt.norm() < 1e-11 && dr.norm() < 1e-8 {
            break;
        }
    }
    let (h, _, rss) = pose_system(config, model, &obs, pose)?;
    let mut cov = invert_spd(h).ok_or(MetrologyError::UnobservablePose)?;
    let residual = (rss / (2 * obs.len() - 6) as f64).sqrt();
    if residual > config.maximum_normalized_residual {
        return Err(MetrologyError::InconsistentObservations);
    }
    let mut q = quality(config, &obs, pose.translation, residual)?;
    q.partial_visibility = obs.len()
        < model.features.len()
            * config
                .devices
                .iter()
                .filter(|d| d.kind == DeviceKind::Camera)
                .count();
    let common = config
        .errors
        .common_variance_m2(q.usable_projector_count > 0)
        + config.errors.fiducial_manufacturing_rms_m.powi(2) / 3.0;
    let centroid = model
        .features
        .iter()
        .fold(Vec3::ZERO, |sum, f| sum + f.point_fiducial_m)
        / model.features.len() as f64;
    let span = model
        .features
        .iter()
        .map(|f| (f.point_fiducial_m - centroid).norm_squared())
        .sum::<f64>()
        / model.features.len() as f64;
    if span < 1e-12 {
        return Err(MetrologyError::UnobservablePose);
    }
    // Conservative shared spatial bias across the marker spans can also rotate
    // a body. Retain this floor rather than averaging manufacturing/calibration.
    for (i, row) in cov.iter_mut().enumerate() {
        row[i] += if i < 3 { common } else { common / span };
    }
    let fiducial_from_tcp = model.tcp_from_fiducial.inverse();
    let tcp = pose.compose(fiducial_from_tcp);
    let offset = pose.rotation * fiducial_from_tcp.translation;
    let mut propagation = [[0.0; 6]; 6];
    for (i, row) in propagation.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    let s = skew(offset);
    for (i, row) in propagation.iter_mut().enumerate().take(3) {
        for j in 0..3 {
            row[j + 3] = -s.m[i][j];
        }
    }
    let mut tcp_cov = propagate(propagation, cov);
    for (i, row) in tcp_cov.iter_mut().enumerate() {
        row[i] += if i < 3 {
            config.errors.tcp_calibration_rms_m.powi(2) / 3.0
        } else {
            config.errors.tcp_rotation_rms_rad.powi(2) / 3.0
        };
    }
    Ok(ObservedToolPose {
        object_id: model.object_id,
        identity_code: model.identity_code.clone(),
        world_from_fiducial: pose,
        world_from_tcp: tcp,
        fiducial_covariance: cov,
        tcp_covariance: tcp_cov,
        quality: q,
    })
}
fn pose_system(
    config: &MetrologyConfig,
    model: &RigidFiducial,
    obs: &[&PixelObservation],
    pose: RigidTransform,
) -> Result<NormalSystem<6>, MetrologyError> {
    let mut h = [[0.0; 6]; 6];
    let mut g = [0.0; 6];
    let mut rss = 0.0;
    for o in obs {
        let local = model
            .features
            .iter()
            .find(|f| f.id == o.feature_id)
            .unwrap()
            .point_fiducial_m;
        let rotated = pose.rotation * local;
        let point = rotated + pose.translation;
        let camera = config.device(o.sensor_id)?.model;
        let predicted = camera
            .project(point)
            .ok_or(MetrologyError::OutsideField)?
            .pixel;
        let jp = projection_jacobian(camera, point)?;
        let cross = skew(rotated);
        for (axis, (r, sigma)) in [
            (o.pixel.x - predicted.x, o.sigma_px.x),
            (o.pixel.y - predicted.y, o.sigma_px.y),
        ]
        .iter()
        .enumerate()
        {
            let mut j = [0.0; 6];
            j[..3].copy_from_slice(&jp[axis]);
            for c in 0..3 {
                for k in 0..3 {
                    j[c + 3] -= jp[axis][k] * cross.m[k][c];
                }
            }
            accumulate(&mut h, &mut g, j, *r, 1.0 / sigma.powi(2));
            rss += (r / sigma).powi(2);
        }
    }
    Ok((h, g, rss))
}
fn basis(p: [Vec3; 3]) -> Option<Mat3> {
    let x = (p[1] - p[0]).normalized()?;
    let z = x.cross(p[2] - p[0]).normalized()?;
    let y = z.cross(x);
    Some(Mat3::new([
        [x.x, y.x, z.x],
        [x.y, y.y, z.y],
        [x.z, y.z, z.z],
    ]))
}
fn quality(
    config: &MetrologyConfig,
    obs: &[&PixelObservation],
    point: Vec3,
    residual: f64,
) -> Result<MeasurementQuality, MetrologyError> {
    let mut cameras = BTreeSet::new();
    let mut projectors = BTreeSet::new();
    let mut sampling = Vec::new();
    let mut centers = Vec::new();
    for o in obs {
        let d = config.device(o.sensor_id)?;
        let inserted = if d.kind == DeviceKind::Camera {
            cameras.insert(o.sensor_id)
        } else {
            projectors.insert(o.sensor_id)
        };
        if inserted {
            centers.push(d.model.center_world());
            sampling.push(
                d.sampling_m_per_px(point)
                    .ok_or(MetrologyError::OutsideField)?,
            );
        }
    }
    let mut baseline: f64 = 0.0;
    let mut angle: f64 = 0.0;
    for i in 0..centers.len() {
        for j in i + 1..centers.len() {
            baseline = baseline.max((centers[i] - centers[j]).norm());
            if let (Some(a), Some(b)) = (
                (centers[i] - point).normalized(),
                (centers[j] - point).normalized(),
            ) {
                angle = angle.max(a.dot(b).abs().clamp(0.0, 1.0).acos());
            }
        }
    }
    let first = obs[0];
    Ok(MeasurementQuality {
        usable_camera_count: cameras.len(),
        usable_projector_count: projectors.len(),
        supporting_observations: obs
            .iter()
            .map(|o| ObservationSupport {
                sensor_id: o.sensor_id,
                feature_id: o.feature_id,
                sequence_id: o.timing.sequence_id,
                trigger_id: o.timing.trigger_id,
                pixel: o.pixel,
                sigma_px: o.sigma_px,
            })
            .collect(),
        best_triangulation_angle_rad: angle,
        maximum_baseline_m: baseline,
        sampling_m_per_px: sampling,
        normalized_residual_rms: residual,
        confidence: 1.0 / (1.0 + residual * residual),
        partial_visibility: cameras.len()
            < config
                .devices
                .iter()
                .filter(|d| d.kind == DeviceKind::Camera)
                .count(),
        available_s: obs
            .iter()
            .map(|o| {
                o.decoded_projector
                    .as_ref()
                    .map_or(o.timing.available_s, |d| {
                        d.available_s.max(o.timing.available_s)
                    })
            })
            .fold(0.0, f64::max),
        oldest_acquisition_s: obs
            .iter()
            .map(|o| {
                o.decoded_projector
                    .as_ref()
                    .map_or(o.timing.exposure_start_s, |d| d.sequence_start_s)
            })
            .fold(f64::INFINITY, f64::min),
        newest_acquisition_s: obs
            .iter()
            .map(|o| {
                o.decoded_projector.as_ref().map_or(
                    o.timing.exposure_start_s + o.timing.exposure_duration_s,
                    |d| d.sequence_end_s,
                )
            })
            .fold(0.0, f64::max),
        mode: first.mode,
        class: first.class,
        calibration_id: first.calibration_id.clone(),
        source: first.source,
    })
}
pub(crate) fn trace3(c: [[f64; 3]; 3]) -> f64 {
    c[0][0] + c[1][1] + c[2][2]
}
pub(crate) fn skew(p: Vec3) -> Mat3 {
    Mat3::new([[0.0, -p.z, p.y], [p.z, 0.0, -p.x], [-p.y, p.x, 0.0]])
}
pub(crate) fn accumulate<const N: usize>(
    h: &mut [[f64; N]; N],
    g: &mut [f64; N],
    j: [f64; N],
    r: f64,
    w: f64,
) {
    for i in 0..N {
        g[i] += j[i] * r * w;
        for k in 0..N {
            h[i][k] += j[i] * j[k] * w;
        }
    }
}
pub(crate) fn mat_vec<const N: usize>(a: [[f64; N]; N], x: [f64; N]) -> [f64; N] {
    let mut y = [0.0; N];
    for i in 0..N {
        for (j, v) in x.iter().enumerate() {
            y[i] += a[i][j] * v;
        }
    }
    y
}
pub(crate) fn propagate<const N: usize>(j: [[f64; N]; N], c: [[f64; N]; N]) -> [[f64; N]; N] {
    let mut out = [[0.0; N]; N];
    for i in 0..N {
        for k in 0..N {
            for (a, row) in c.iter().enumerate() {
                for (b, value) in row.iter().enumerate() {
                    out[i][k] += j[i][a] * value * j[k][b];
                }
            }
        }
    }
    out
}
/// Diagonally equilibrated Cholesky. Scaling makes metre/radian mixed systems
/// numerically comparable; non-positive/singular directions fail closed.
pub(crate) fn invert_spd<const N: usize>(a: [[f64; N]; N]) -> Option<[[f64; N]; N]> {
    let mut scale = [0.0; N];
    for i in 0..N {
        if !positive(a[i][i]) {
            return None;
        }
        scale[i] = a[i][i].sqrt();
    }
    let mut l = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..=i {
            if !a[i][j].is_finite() || (a[i][j] - a[j][i]).abs() > 1e-8 * scale[i] * scale[j] {
                return None;
            }
            let mut s = a[i][j] / scale[i] / scale[j];
            for k in 0..j {
                s -= l[i][k] * l[j][k];
            }
            if i == j {
                if !s.is_finite() || s <= 1e-10 {
                    return None;
                }
                l[i][j] = s.sqrt();
            } else {
                l[i][j] = s / l[j][j];
            }
        }
    }
    let mut inv = [[0.0; N]; N];
    for col in 0..N {
        let mut y = [0.0; N];
        for i in 0..N {
            let mut v = if i == col { 1.0 } else { 0.0 };
            for (k, yk) in y.iter().enumerate().take(i) {
                v -= l[i][k] * yk;
            }
            y[i] = v / l[i][i];
        }
        let mut x = [0.0; N];
        for i in (0..N).rev() {
            let mut v = y[i];
            for k in i + 1..N {
                v -= l[k][i] * x[k];
            }
            x[i] = v / l[i][i];
        }
        for i in 0..N {
            inv[i][col] = x[i] / scale[i] / scale[col];
        }
    }
    Some(inv)
}
