//! A tracked marker is not a directly measured mating surface.
use super::*;
use crate::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RelationProvenance {
    NominalDesign,
    SyntheticIndependentCharacterization,
    PhysicalIndependentCharacterization,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartFeatureRelation {
    pub feature_id: u32,
    /// Feature centre in the rigid marker frame, metres.
    pub center_fiducial_m: Vec3,
    pub axis_fiducial: Vec3,
    pub calibration_id: String,
    pub provenance: RelationProvenance,
    /// Independent marker-to-feature dimensional characterization, 3D RMS.
    pub characterization_rms_m: f64,
    pub axis_characterization_rms_rad: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InferredPartFeature {
    pub provenance: String,
    pub relation: PartFeatureRelation,
    pub center: ObservedPoint,
    pub axis_world: Vec3,
    pub axis_rms_rad: f64,
}
/// Full pose Jacobian [I, -[R p]x], including position/orientation cross terms.
/// Independent relation uncertainty is added once, isotropically. Correlation
/// between distinct optical estimates is NOT assumed to cancel.
pub fn infer_part_feature(
    pose: &ObservedToolPose,
    relation: &PartFeatureRelation,
) -> Result<InferredPartFeature, MetrologyError> {
    if relation.calibration_id.is_empty()
        || !relation.center_fiducial_m.is_finite()
        || (relation.axis_fiducial.norm() - 1.0).abs() > 1e-9
        || !relation.axis_fiducial.is_finite()
        || !positive(relation.characterization_rms_m)
        || !positive(relation.axis_characterization_rms_rad)
        || invert_spd(pose.fiducial_covariance).is_none()
        || !valid_transform(pose.world_from_fiducial)
    {
        return Err(MetrologyError::InvalidObservation);
    }
    let p = pose
        .world_from_fiducial
        .transform_vector(relation.center_fiducial_m);
    let j = [
        [1., 0., 0., 0., p.z, -p.y],
        [0., 1., 0., -p.z, 0., p.x],
        [0., 0., 1., p.y, -p.x, 0.],
    ];
    let mut covariance = [[0.; 3]; 3];
    for (i, row) in covariance.iter_mut().enumerate() {
        for (k, value) in row.iter_mut().enumerate() {
            for a in 0..6 {
                for b in 0..6 {
                    *value += j[i][a] * pose.fiducial_covariance[a][b] * j[k][b];
                }
            }
            if i == k {
                *value += relation.characterization_rms_m.powi(2) / 3.;
            }
        }
    }
    let mut quality = pose.quality.clone();
    quality.class = MeasurementClass::CriticalFeature;
    Ok(InferredPartFeature {
        provenance:
            "inferred_from_rigid_markers_and_characterized_relation; not_direct_surface_measurement"
                .into(),
        relation: relation.clone(),
        center: ObservedPoint {
            object_id: pose.object_id,
            feature_id: relation.feature_id,
            position_world_m: pose
                .world_from_fiducial
                .transform_point(relation.center_fiducial_m),
            // Conservative: no separation of random/common errors is available
            // from the rigid-pose solver. Retain the entire covariance here.
            random_covariance_m2: covariance,
            covariance_m2: covariance,
            quality,
        },
        axis_world: pose
            .world_from_fiducial
            .transform_vector(relation.axis_fiducial),
        axis_rms_rad: (pose.orientation_rms_rad().powi(2)
            + relation.axis_characterization_rms_rad.powi(2))
        .sqrt(),
    })
}

/// An inferred datum needs its own characterization provenance in addition to
/// the camera/marker calibration. Nominal CAD is not a dimensional standard.
pub fn require_inferred_precision(
    config: &MetrologyConfig,
    policy: &PrecisionContract,
    tool: &ObservedToolPose,
    feature: &InferredPartFeature,
    health: &MetrologyHealth,
    now_s: f64,
) -> Result<(), PrecisionRejection> {
    if feature.relation.calibration_id.is_empty()
        || feature.center.feature_id != feature.relation.feature_id
        || !positive(feature.relation.characterization_rms_m)
        || !positive(feature.relation.axis_characterization_rms_rad)
        || !feature.axis_world.is_finite()
        || (feature.axis_world.norm() - 1.).abs() > 1e-9
        || !feature.axis_rms_rad.is_finite()
        || feature.axis_rms_rad < feature.relation.axis_characterization_rms_rad
        || !feature.center.predicted_3d_rms_m().is_finite()
        || feature.center.predicted_3d_rms_m() < feature.relation.characterization_rms_m
    {
        return Err(PrecisionRejection::InvalidCovariance);
    }
    if feature.relation.provenance == RelationProvenance::NominalDesign
        || (policy.require_physical_observations
            && feature.relation.provenance
                != RelationProvenance::PhysicalIndependentCharacterization)
    {
        return Err(PrecisionRejection::InvalidProvenance);
    }
    require_precision(config, policy, tool, &feature.center, health, now_s)
}
