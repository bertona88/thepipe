//! Independent accuracy evaluation is a separate consumer of observations.
//! Calibration data and verification artifact identities must be disjoint.
use super::synthetic::*;
use super::*;
use crate::{Mat3, RigidTransform, Scene, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReferenceProvenance {
    IndependentSyntheticGeometry,
    IndependentlyCharacterizedPhysicalArtifact,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VerificationReference {
    pub artifact_id: String,
    pub feature_id: u32,
    pub known_position_world_m: Vec3,
    pub characterization_rms_m: f64,
    pub provenance: ReferenceProvenance,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VerificationSample {
    pub reference: VerificationReference,
    pub observed: ObservedPoint,
    pub error_world_m: Vec3,
    pub depth_error_m: f64,
    pub normalized_estimation_error_squared: f64,
}
pub fn verify_point(
    config: &MetrologyConfig,
    reference: VerificationReference,
    observed: ObservedPoint,
    depth_axis_world: Vec3,
) -> Result<VerificationSample, MetrologyError> {
    if !config.calibration.excludes(&reference.artifact_id) || reference.artifact_id.is_empty() {
        return Err(MetrologyError::VerificationLeakage);
    }
    if reference.provenance == ReferenceProvenance::IndependentlyCharacterizedPhysicalArtifact
        && (config.calibration.provenance != CalibrationProvenance::PhysicalArtifactFit
            || !positive(reference.characterization_rms_m))
    {
        return Err(MetrologyError::InvalidCalibration);
    }
    if !reference.known_position_world_m.is_finite()
        || !nonnegative(reference.characterization_rms_m)
        || reference.feature_id != observed.feature_id
        || observed.quality.calibration_id != config.calibration.id
        || !observed.position_world_m.is_finite()
        || (reference.provenance == ReferenceProvenance::IndependentlyCharacterizedPhysicalArtifact
            && observed.quality.source != ObservationSource::PhysicalImageFeature)
        || observed.quality.source == ObservationSource::IdealSyntheticFeature
    {
        return Err(MetrologyError::InvalidObservation);
    }
    let axis = depth_axis_world
        .normalized()
        .ok_or(MetrologyError::InvalidObservation)?;
    let mut covariance = observed.covariance_m2;
    for (i, row) in covariance.iter_mut().enumerate() {
        row[i] += reference.characterization_rms_m.powi(2) / 3.0;
    }
    let info = invert_spd(covariance).ok_or(MetrologyError::InvalidObservation)?;
    let error = observed.position_world_m - reference.known_position_world_m;
    let e = [error.x, error.y, error.z];
    let weighted = mat_vec(info, e);
    Ok(VerificationSample {
        reference,
        observed,
        error_world_m: error,
        depth_error_m: error.dot(axis),
        normalized_estimation_error_squared: e.iter().zip(weighted).map(|(a, b)| a * b).sum(),
    })
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VerificationSummary {
    pub condition: String,
    pub attempted: usize,
    pub accepted: usize,
    pub position_3d_rms_m: Option<f64>,
    pub position_axis_rms_m: Option<Vec3>,
    pub depth_rms_m: Option<f64>,
    pub predicted_3d_rms_m: Option<f64>,
    /// Within-feature RMS scatter after removing its empirical mean. Explicitly
    /// separate from error against independent known geometry.
    pub repeatability_3d_rms_m: Option<f64>,
    pub mean_nees: Option<f64>,
    pub fraction_inside_95_percent_3d_ellipsoid: Option<f64>,
    pub mean_usable_camera_count: Option<f64>,
    pub occlusion_fraction: f64,
    pub minimum_usable_view_fraction: f64,
    pub local_target_met_in_simulation: bool,
    pub samples: Vec<VerificationSample>,
    pub rejected_measurements: std::collections::BTreeMap<String, usize>,
}
impl VerificationSummary {
    pub fn from_samples(
        condition: String,
        attempted: usize,
        samples: Vec<VerificationSample>,
        occlusion_fraction: f64,
        view_fraction: f64,
    ) -> Self {
        let n = samples.len();
        let mut squares = Vec3::ZERO;
        let mut depth = 0.0;
        let mut predicted = 0.0;
        let mut nees = 0.0;
        let mut count = 0;
        let mut views = 0;
        let mut groups: std::collections::BTreeMap<(&str, u32), Vec<Vec3>> =
            std::collections::BTreeMap::new();
        for s in &samples {
            squares += s.error_world_m.component_mul(s.error_world_m);
            depth += s.depth_error_m.powi(2);
            predicted += s.observed.predicted_3d_rms_m().powi(2);
            nees += s.normalized_estimation_error_squared;
            count += usize::from(s.normalized_estimation_error_squared <= 7.814727903);
            views += s.observed.quality.usable_camera_count;
            groups
                .entry((&s.reference.artifact_id, s.reference.feature_id))
                .or_default()
                .push(s.observed.position_world_m);
        }
        let mut repeat_squared = 0.0;
        let mut repeat_dof = 0;
        for points in groups.values() {
            if points.len() > 1 {
                let mean = points.iter().fold(Vec3::ZERO, |a, &p| a + p) / points.len() as f64;
                repeat_squared += points
                    .iter()
                    .map(|&p| (p - mean).norm_squared())
                    .sum::<f64>();
                repeat_dof += points.len() - 1;
            }
        }
        let position = (n > 0).then(|| ((squares.x + squares.y + squares.z) / n as f64).sqrt());
        let repeat = (repeat_dof > 0).then(|| (repeat_squared / repeat_dof as f64).sqrt());
        Self {
            rejected_measurements: Default::default(),
            condition,
            attempted,
            accepted: n,
            position_3d_rms_m: position,
            position_axis_rms_m: (n > 0).then(|| {
                Vec3::new(
                    (squares.x / n as f64).sqrt(),
                    (squares.y / n as f64).sqrt(),
                    (squares.z / n as f64).sqrt(),
                )
            }),
            depth_rms_m: (n > 0).then(|| (depth / n as f64).sqrt()),
            predicted_3d_rms_m: (n > 0).then(|| (predicted / n as f64).sqrt()),
            repeatability_3d_rms_m: repeat,
            mean_nees: (n > 0).then(|| nees / n as f64),
            fraction_inside_95_percent_3d_ellipsoid: (n > 0).then(|| count as f64 / n as f64),
            mean_usable_camera_count: (n > 0).then(|| views as f64 / n as f64),
            occlusion_fraction,
            minimum_usable_view_fraction: view_fraction,
            local_target_met_in_simulation: attempted > 0
                && n == attempted
                && position.is_some_and(|p| p <= 5e-6)
                && repeat.is_some_and(|r| r <= 2e-6),
            samples,
        }
    }
}
pub fn verify_volume(
    config: &MetrologyConfig,
    scene: &Scene,
    repeats: u32,
    structured: bool,
) -> Result<VerificationSummary, MetrologyError> {
    config.validate()?;
    if !(2..=1000).contains(&repeats) {
        return Err(config_error(
            "verification requires 2..1000 repeated acquisitions",
        ));
    }
    let mut samples = Vec::new();
    let mut rejected = std::collections::BTreeMap::new();
    let mut attempted = 0;
    let mut occluded = 0.0;
    let mut attempts = 0;
    let mut min_views: f64 = 1.0;
    let mut id = 0;
    for x in [-0.4, 0.0, 0.4] {
        for y in [-0.4, 0.0, 0.4] {
            for z in [-0.4, 0.0, 0.4] {
                id += 1;
                let point = config.precision_volume.center_world_m
                    + Vec3::new(x, y, z).component_mul(config.precision_volume.size_m);
                let truth = TruthFeature {
                    object_id: 2000,
                    feature_id: id,
                    point_world_m: point,
                    normal_world: None,
                    velocity_world_m_s: Vec3::ZERO,
                    surface: SurfaceResponse::default(),
                };
                for repeat in 0..repeats {
                    attempted += 1;
                    let t = timing(config, repeat as u64 + 1, 1.0 + repeat as f64 * 0.1, 0.0);
                    let acquisition = if structured {
                        acquire_structured(
                            config,
                            scene,
                            &truth,
                            config
                                .devices
                                .iter()
                                .find(|d| d.kind == DeviceKind::Camera)
                                .unwrap()
                                .id,
                            config
                                .devices
                                .iter()
                                .find(|d| d.kind == DeviceKind::Projector)
                                .ok_or(MetrologyError::InvalidConfiguration(
                                    "missing projector".into(),
                                ))?
                                .id,
                            &t,
                        )
                    } else {
                        acquire_passive(
                            config,
                            scene,
                            &truth,
                            MeasurementMode::Precision,
                            MeasurementClass::CriticalFeature,
                            &t,
                        )
                    };
                    if let Ok(a) = acquisition {
                        occluded += a.occlusion_fraction();
                        attempts += 1;
                        min_views = min_views.min(a.usable_view_fraction());
                        let reconstructed = reconstruct_point(config, &a.observations);
                        if let Ok(observed) = reconstructed {
                            let reference = VerificationReference {
                                artifact_id: "independent_3d_volume_artifact".into(),
                                feature_id: id,
                                known_position_world_m: point,
                                characterization_rms_m: 0.0,
                                provenance: ReferenceProvenance::IndependentSyntheticGeometry,
                            };
                            let axis = config.devices[0].model.world_from_camera.rotation * Vec3::Z;
                            samples.push(verify_point(config, reference, observed, axis)?);
                        } else if let Err(e) = reconstructed {
                            *rejected.entry(format!("{e:?}")).or_insert(0) += 1;
                        }
                    } else {
                        min_views = 0.0;
                        if let Err(e) = acquisition {
                            *rejected.entry(format!("{e:?}")).or_insert(0) += 1;
                        }
                    }
                }
            }
        }
    }
    let mut report = VerificationSummary::from_samples(
        if structured {
            "hybrid_structured_light".into()
        } else {
            "passive_precision".into()
        },
        attempted,
        samples,
        if attempts > 0 {
            occluded / attempts as f64
        } else {
            0.0
        },
        min_views,
    );
    report.rejected_measurements = rejected;
    Ok(report)
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolVerification {
    pub attempted: usize,
    pub accepted: usize,
    pub tcp_position_3d_rms_m: Option<f64>,
    pub orientation_rms_rad: Option<f64>,
    pub predicted_tcp_3d_rms_m: Option<f64>,
    pub repeatability_3d_rms_m: Option<f64>,
    pub mean_pose_nees: Option<f64>,
}
pub fn verify_tool(
    config: &MetrologyConfig,
    scene: &Scene,
    repeats: u32,
) -> Result<ToolVerification, MetrologyError> {
    if repeats < 2 {
        return Err(config_error(
            "tool verification requires repeat acquisitions",
        ));
    }
    let mut model = RigidFiducial::baseline(1);
    model.geometry_calibration_id = config.calibration.id.clone();
    let truth = RigidTransform::new(
        Mat3::from_axis_angle(Vec3::new(0.2, -0.3, 0.4)),
        config.precision_volume.center_world_m,
    );
    let mut errors = Vec::new();
    let mut angle2 = 0.0;
    let mut predicted = 0.0;
    let mut nees = 0.0;
    for i in 0..repeats {
        let a = acquire_tool(
            config,
            scene,
            &model,
            truth,
            MeasurementMode::Precision,
            &timing(config, i as u64 + 1, 1.0 + 0.1 * i as f64, 0.0),
        )?;
        if let Ok(p) = estimate_tool_pose(config, &model, &a.observations) {
            let error = p.world_from_tcp.translation - truth.translation;
            let rotation = p.world_from_tcp.rotation * truth.rotation.transpose();
            let theta = ((rotation.trace() - 1.0) * 0.5).clamp(-1.0, 1.0).acos();
            angle2 += theta * theta;
            let factor = if theta < 1e-8 {
                0.5
            } else {
                theta / (2.0 * theta.sin())
            };
            let rv = Vec3::new(
                rotation.m[2][1] - rotation.m[1][2],
                rotation.m[0][2] - rotation.m[2][0],
                rotation.m[1][0] - rotation.m[0][1],
            ) * factor;
            let e = [error.x, error.y, error.z, rv.x, rv.y, rv.z];
            let weighted = mat_vec(
                invert_spd(p.tcp_covariance).ok_or(MetrologyError::DegenerateGeometry)?,
                e,
            );
            nees += e.iter().zip(weighted).map(|(a, b)| a * b).sum::<f64>();
            predicted += p.predicted_tcp_3d_rms_m().powi(2);
            errors.push(error);
        }
    }
    let n = errors.len();
    let mean = errors.iter().fold(Vec3::ZERO, |a, &b| a + b) / n.max(1) as f64;
    Ok(ToolVerification {
        attempted: repeats as usize,
        accepted: n,
        tcp_position_3d_rms_m: (n > 0)
            .then(|| (errors.iter().map(|e| e.norm_squared()).sum::<f64>() / n as f64).sqrt()),
        orientation_rms_rad: (n > 0).then(|| (angle2 / n as f64).sqrt()),
        predicted_tcp_3d_rms_m: (n > 0).then(|| (predicted / n as f64).sqrt()),
        repeatability_3d_rms_m: (n > 1).then(|| {
            (errors
                .iter()
                .map(|&e| (e - mean).norm_squared())
                .sum::<f64>()
                / (n - 1) as f64)
                .sqrt()
        }),
        mean_pose_nees: (n > 0).then(|| nees / n as f64),
    })
}
