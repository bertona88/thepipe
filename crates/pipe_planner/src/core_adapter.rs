//! Explicit SI-to-task-unit adapter for `pipe_sim_core`.

use pipe_sim_core::{BodyId, CollisionReport, Pose, Quat, Vec3};

use crate::{
    AssemblyContactDefinition, ComponentId, ContactFeature, NominalAssemblyPose, PoseError6d,
    SensorFrame,
};

const METRES_TO_MILLIMETRES: f64 = 1000.0;
const RADIANS_TO_DEGREES: f64 = 180.0 / core::f64::consts::PI;

/// Collision pairs deliberately used by the active phase (gripper/part,
/// part/bore, or gear/gear). All other reported pairs remain safety inputs.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IntendedContactPairs {
    ordered_pairs: Vec<(BodyId, BodyId)>,
}

impl IntendedContactPairs {
    fn from_feature_proxy_pairs(pairs: impl IntoIterator<Item = (BodyId, BodyId)>) -> Self {
        let mut ordered_pairs = pairs
            .into_iter()
            .filter(|(a, b)| a != b)
            .map(canonical_pair)
            .collect::<Vec<_>>();
        ordered_pairs.sort_unstable();
        ordered_pairs.dedup();
        Self { ordered_pairs }
    }

    pub fn contains(&self, a: BodyId, b: BodyId) -> bool {
        self.ordered_pairs
            .binary_search(&canonical_pair((a, b)))
            .is_ok()
    }

    pub fn pairs(&self) -> &[(BodyId, BodyId)] {
        &self.ordered_pairs
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FeatureProxyBinding {
    pub feature: ContactFeature,
    /// Dedicated collision body for this feature. A body ID may not be shared
    /// by two features, otherwise allowing a seat could hide a wall strike.
    pub proxy_body: BodyId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FeatureProxyMap {
    bindings: Vec<FeatureProxyBinding>,
}

impl FeatureProxyMap {
    pub fn new(
        bindings: impl IntoIterator<Item = FeatureProxyBinding>,
    ) -> Result<Self, CoreAdapterError> {
        let mut bindings = bindings.into_iter().collect::<Vec<_>>();
        bindings.sort_by_key(|binding| binding.feature);
        for window in bindings.windows(2) {
            if window[0].feature == window[1].feature {
                return Err(CoreAdapterError::DuplicateFeatureProxy(window[0].feature));
            }
        }
        let mut bodies = bindings
            .iter()
            .map(|binding| (binding.proxy_body, binding.feature))
            .collect::<Vec<_>>();
        bodies.sort_by_key(|(body, _)| *body);
        for window in bodies.windows(2) {
            if window[0].0 == window[1].0 {
                return Err(CoreAdapterError::AliasedFeatureProxy {
                    body: window[0].0,
                    first: window[0].1,
                    second: window[1].1,
                });
            }
        }
        Ok(Self { bindings })
    }

    pub fn proxy_body(&self, feature: ContactFeature) -> Option<BodyId> {
        self.bindings
            .binary_search_by_key(&feature, |binding| binding.feature)
            .ok()
            .map(|index| self.bindings[index].proxy_body)
    }

    pub fn intended_pairs<'a>(
        &self,
        definitions: impl IntoIterator<Item = &'a AssemblyContactDefinition>,
    ) -> Result<IntendedContactPairs, CoreAdapterError> {
        definitions
            .into_iter()
            .map(|definition| {
                let a = self
                    .proxy_body(definition.feature_a)
                    .ok_or(CoreAdapterError::MissingFeatureProxy(definition.feature_a))?;
                let b = self
                    .proxy_body(definition.feature_b)
                    .ok_or(CoreAdapterError::MissingFeatureProxy(definition.feature_b))?;
                Ok((a, b))
            })
            .collect::<Result<Vec<_>, _>>()
            .map(IntendedContactPairs::from_feature_proxy_pairs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionProjectionConfig {
    /// Conservative clearance used when the collision query reports no nearby
    /// unplanned pair. It must be finite because planner safety gates reject
    /// infinities and NaNs.
    pub no_nearby_pair_clearance_mm: f64,
    /// First-order contact estimate used until the force solver exposes pair
    /// impulses. Tune from a printed coupon, never from the nominal resin data
    /// sheet alone.
    pub contact_stiffness_n_per_m: f64,
}

impl Default for CollisionProjectionConfig {
    fn default() -> Self {
        Self {
            no_nearby_pair_clearance_mm: 1.0,
            contact_stiffness_n_per_m: 1000.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionSafetySample {
    pub min_unplanned_clearance_mm: f64,
    pub estimated_unplanned_contact_force_n: f64,
}

impl CollisionSafetySample {
    pub fn apply_to(self, frame: &mut SensorFrame) {
        frame.min_unplanned_clearance_mm = self.min_unplanned_clearance_mm;
        frame.unplanned_contact_force_n = self.estimated_unplanned_contact_force_n;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoreAdapterError {
    InvalidProjectionConfig,
    NonFiniteCollisionReport,
    DuplicateFeatureProxy(ContactFeature),
    AliasedFeatureProxy {
        body: BodyId,
        first: ContactFeature,
        second: ContactFeature,
    },
    MissingFeatureProxy(ContactFeature),
}

/// Convert core proximity/contact output into the two phase-independent safety
/// measurements consumed by the executive.
pub fn project_collision_report(
    report: &CollisionReport,
    intended: &IntendedContactPairs,
    config: CollisionProjectionConfig,
) -> Result<CollisionSafetySample, CoreAdapterError> {
    if !config.no_nearby_pair_clearance_mm.is_finite()
        || config.no_nearby_pair_clearance_mm < 0.0
        || !config.contact_stiffness_n_per_m.is_finite()
        || config.contact_stiffness_n_per_m <= 0.0
    {
        return Err(CoreAdapterError::InvalidProjectionConfig);
    }

    let mut min_clearance_mm = config.no_nearby_pair_clearance_mm;
    let mut max_force_n: f64 = 0.0;
    for clearance in &report.clearances {
        if !clearance.distance_m.is_finite() {
            return Err(CoreAdapterError::NonFiniteCollisionReport);
        }
        if !intended.contains(clearance.body_a, clearance.body_b) {
            min_clearance_mm =
                min_clearance_mm.min(clearance.distance_m.max(0.0) * METRES_TO_MILLIMETRES);
        }
    }
    for contact in &report.contacts {
        if !contact.penetration_depth_m.is_finite() || !contact.signed_distance_m.is_finite() {
            return Err(CoreAdapterError::NonFiniteCollisionReport);
        }
        if !intended.contains(contact.body_a, contact.body_b) {
            min_clearance_mm = 0.0;
            max_force_n = max_force_n
                .max(contact.penetration_depth_m.max(0.0) * config.contact_stiffness_n_per_m);
        }
    }

    Ok(CollisionSafetySample {
        min_unplanned_clearance_mm: min_clearance_mm,
        estimated_unplanned_contact_force_n: max_force_n,
    })
}

/// Target-relative pose error in the executive's millimetre/degree units.
/// Translation is expressed in world axes; rotation is the shortest scaled
/// axis carrying target orientation to observed orientation.
pub fn pose_error_from_core(target: Pose, observed: Pose) -> PoseError6d {
    let translation = (observed.translation - target.translation) * METRES_TO_MILLIMETRES;
    let rotation = shortest_scaled_axis(target.rotation.inverse() * observed.rotation);
    PoseError6d {
        translation_mm: [translation.x, translation.y, translation.z],
        rotation_deg: [
            rotation.x * RADIANS_TO_DEGREES,
            rotation.y * RADIANS_TO_DEGREES,
            rotation.z * RADIANS_TO_DEGREES,
        ],
    }
}

pub fn nominal_pose_to_core(value: NominalAssemblyPose) -> Pose {
    let axis_alignment = Quat::from_two_vectors(
        Vec3::Z,
        Vec3::new(value.part_axis[0], value.part_axis[1], value.part_axis[2]),
    );
    let phase = Quat::from_axis_angle(Vec3::Z, value.phase_deg.to_radians());
    Pose::new(
        Vec3::new(
            value.position_mm[0] / METRES_TO_MILLIMETRES,
            value.position_mm[1] / METRES_TO_MILLIMETRES,
            value.position_mm[2] / METRES_TO_MILLIMETRES,
        ),
        axis_alignment * phase,
    )
}

/// Simple deterministic mapping for simulations that reserve body IDs 0--999
/// for the cell and arms.
pub fn default_component_body_id(component: ComponentId) -> BodyId {
    BodyId(1000 + u32::from(component.0))
}

fn canonical_pair((a, b): (BodyId, BodyId)) -> (BodyId, BodyId) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn shortest_scaled_axis(rotation: Quat) -> Vec3 {
    let mut q = rotation.normalized();
    if q.w < 0.0 {
        q = Quat::new(-q.w, -q.x, -q.y, -q.z);
    }
    let vector = Vec3::new(q.x, q.y, q.z);
    let vector_norm = vector.length();
    if vector_norm <= 1.0e-12 {
        Vec3::ZERO
    } else {
        let angle = 2.0 * vector_norm.atan2(q.w.clamp(-1.0, 1.0));
        vector * (angle / vector_norm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipe_sim_core::{Clearance, Contact, ContactKind};

    #[test]
    fn pose_adapter_uses_mm_and_shortest_rotation() {
        let target = Pose::IDENTITY;
        let observed = Pose::new(
            Vec3::new(10.0e-6, -20.0e-6, 0.0),
            Quat::from_axis_angle(Vec3::Z, 2.0_f64.to_radians()),
        );
        let error = pose_error_from_core(target, observed);
        assert!((error.translation_mm[0] - 0.010).abs() < 1.0e-12);
        assert!((error.translation_mm[1] + 0.020).abs() < 1.0e-12);
        assert!((error.rotation_deg[2] - 2.0).abs() < 1.0e-12);
    }

    #[test]
    fn intended_contacts_are_removed_from_safety_projection() {
        let a = BodyId(1);
        let b = BodyId(2);
        let c = BodyId(3);
        let report = CollisionReport {
            broad_phase_pairs: vec![(a, b), (a, c)],
            contacts: vec![Contact {
                body_a: a,
                body_b: b,
                point_a_world_m: Vec3::ZERO,
                point_b_world_m: Vec3::ZERO,
                normal_a_to_b: Vec3::X,
                signed_distance_m: -10.0e-6,
                penetration_depth_m: 10.0e-6,
                kind: ContactKind::ExactAnalytic,
            }],
            clearances: vec![Clearance {
                body_a: a,
                body_b: c,
                distance_m: 40.0e-6,
                point_a_world_m: Vec3::ZERO,
                point_b_world_m: Vec3::ZERO,
                kind: ContactKind::ExactAnalytic,
            }],
        };
        let floor = ContactFeature::HousingFloor;
        let gear_face = ContactFeature::GearLowerFace {
            gear: ComponentId(4),
        };
        let unrelated = ContactFeature::GearTeeth {
            gear: ComponentId(4),
        };
        let proxies = FeatureProxyMap::new([
            FeatureProxyBinding {
                feature: floor,
                proxy_body: a,
            },
            FeatureProxyBinding {
                feature: gear_face,
                proxy_body: b,
            },
            FeatureProxyBinding {
                feature: unrelated,
                proxy_body: c,
            },
        ])
        .unwrap();
        let definition = AssemblyContactDefinition {
            activates_with: ComponentId(4),
            activates_at: crate::Phase::Insert,
            feature_a: floor,
            feature_b: gear_face,
            kind: crate::AssemblyContactKind::GearOnFloor,
        };
        let intended = proxies.intended_pairs([&definition]).unwrap();
        let sample =
            project_collision_report(&report, &intended, CollisionProjectionConfig::default())
                .unwrap();
        assert!((sample.min_unplanned_clearance_mm - 0.040).abs() < 1.0e-12);
        assert_eq!(sample.estimated_unplanned_contact_force_n, 0.0);
    }

    #[test]
    fn feature_proxy_map_rejects_whole_body_aliasing() {
        let body = BodyId(10);
        let result = FeatureProxyMap::new([
            FeatureProxyBinding {
                feature: ContactFeature::HousingFloor,
                proxy_body: body,
            },
            FeatureProxyBinding {
                feature: ContactFeature::HousingCoverRim,
                proxy_body: body,
            },
        ]);
        assert!(matches!(
            result,
            Err(CoreAdapterError::AliasedFeatureProxy { body: id, .. }) if id == body
        ));
    }
}
