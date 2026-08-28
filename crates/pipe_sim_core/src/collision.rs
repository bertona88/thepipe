//! Deterministic broad phase and analytic/declared-approximation narrow phase.

use crate::geometry::{BodyId, GearGeometry, RigidBody, Shape};
use crate::math::{Pose, Vec3, EPSILON};

// Geometry is routinely micrometre-scale. This is a squared-length degeneracy
// threshold, not the metre-valued `EPSILON` used by normalized vectors.
const SEGMENT_EPSILON_SQUARED_M2: f64 = 1.0e-30;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionSettings {
    /// Physical distance at which a pair enters the contact solver.
    pub contact_offset_m: f64,
    /// Positive gaps at or below this value are reported for metrology.
    pub clearance_threshold_m: f64,
    /// Geometric degeneracy threshold, distinct from physical allowances.
    pub numeric_epsilon_m: f64,
}

impl Default for CollisionSettings {
    fn default() -> Self {
        Self {
            contact_offset_m: 1.0e-6,
            clearance_threshold_m: 100.0e-6,
            numeric_epsilon_m: 1.0e-12,
        }
    }
}

impl CollisionSettings {
    pub fn is_valid(self) -> bool {
        self.contact_offset_m >= 0.0
            && self.clearance_threshold_m >= self.contact_offset_m
            && self.numeric_epsilon_m > 0.0
            && self.contact_offset_m.is_finite()
            && self.clearance_threshold_m.is_finite()
            && self.numeric_epsilon_m.is_finite()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContactKind {
    ExactAnalytic,
    CapsuleBoxApproximation,
    OrientedBoxSatApproximation,
    GearAnnularEnvelopeApproximation,
    GearMeshApproximation,
    GearBoxEnvelopeApproximation,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Proximity {
    /// Canonical ascending body ID pair.
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point_a_world_m: Vec3,
    pub point_b_world_m: Vec3,
    pub normal_a_to_b: Vec3,
    /// Positive gap, zero tangency, negative penetration.
    pub signed_distance_m: f64,
    pub kind: ContactKind,
}

impl Proximity {
    pub fn penetration_depth_m(self) -> f64 {
        (-self.signed_distance_m).max(0.0)
    }

    pub fn is_penetrating(self) -> bool {
        self.signed_distance_m < 0.0
    }

    fn flipped(mut self) -> Self {
        core::mem::swap(&mut self.body_a, &mut self.body_b);
        core::mem::swap(&mut self.point_a_world_m, &mut self.point_b_world_m);
        self.normal_a_to_b = -self.normal_a_to_b;
        self
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub point_a_world_m: Vec3,
    pub point_b_world_m: Vec3,
    pub normal_a_to_b: Vec3,
    pub signed_distance_m: f64,
    pub penetration_depth_m: f64,
    pub kind: ContactKind,
}

impl From<Proximity> for Contact {
    fn from(value: Proximity) -> Self {
        Self {
            body_a: value.body_a,
            body_b: value.body_b,
            point_a_world_m: value.point_a_world_m,
            point_b_world_m: value.point_b_world_m,
            normal_a_to_b: value.normal_a_to_b,
            signed_distance_m: value.signed_distance_m,
            penetration_depth_m: value.penetration_depth_m(),
            kind: value.kind,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Clearance {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub distance_m: f64,
    pub point_a_world_m: Vec3,
    pub point_b_world_m: Vec3,
    pub kind: ContactKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GearMeshClearance {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub axes_parallel: bool,
    pub modules_compatible: bool,
    pub center_distance_m: f64,
    pub ideal_center_distance_m: f64,
    /// Positive means gears are farther apart than nominal.
    pub center_offset_m: f64,
    pub minimum_root_tip_clearance_m: f64,
    pub axial_overlap_m: f64,
    /// First-order backlash from center-distance increase.
    pub estimated_backlash_m: f64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionReport {
    pub broad_phase_pairs: Vec<(BodyId, BodyId)>,
    pub contacts: Vec<Contact>,
    pub clearances: Vec<Clearance>,
}

impl CollisionReport {
    pub fn query(bodies: &[RigidBody], settings: CollisionSettings) -> Self {
        let broad_phase_pairs = broad_phase_pairs(bodies, settings.clearance_threshold_m);
        let mut ordered = bodies
            .iter()
            .filter(|body| body.enabled)
            .collect::<Vec<_>>();
        ordered.sort_by_key(|body| body.id);
        let mut contacts = Vec::new();
        let mut clearances = Vec::new();

        for (id_a, id_b) in &broad_phase_pairs {
            let a = ordered
                .binary_search_by_key(id_a, |body| body.id)
                .ok()
                .map(|index| ordered[index]);
            let b = ordered
                .binary_search_by_key(id_b, |body| body.id)
                .ok()
                .map(|index| ordered[index]);
            if let (Some(a), Some(b)) = (a, b) {
                if let Some(proximity) = query_pair(a, b) {
                    if proximity.signed_distance_m <= settings.contact_offset_m {
                        contacts.push(proximity.into());
                    } else if proximity.signed_distance_m <= settings.clearance_threshold_m {
                        clearances.push(Clearance {
                            body_a: proximity.body_a,
                            body_b: proximity.body_b,
                            distance_m: proximity.signed_distance_m,
                            point_a_world_m: proximity.point_a_world_m,
                            point_b_world_m: proximity.point_b_world_m,
                            kind: proximity.kind,
                        });
                    }
                }
            }
        }
        Self {
            broad_phase_pairs,
            contacts,
            clearances,
        }
    }
}

/// Stable O(n²) broad phase suitable for tens to low hundreds of assembly-cell
/// bodies. Output is invariant to insertion order.
pub fn broad_phase_pairs(bodies: &[RigidBody], query_margin_m: f64) -> Vec<(BodyId, BodyId)> {
    let mut ordered = bodies
        .iter()
        .filter(|body| body.enabled && body.shape.is_valid())
        .collect::<Vec<_>>();
    ordered.sort_by_key(|body| body.id);
    let mut pairs = Vec::new();
    for i in 0..ordered.len() {
        for j in (i + 1)..ordered.len() {
            let a = ordered[i];
            let b = ordered[j];
            if a.id == b.id || !a.collision_filter.allows(b.collision_filter) {
                continue;
            }
            if a.aabb().distance(b.aabb()) <= query_margin_m.max(0.0) {
                pairs.push((a.id, b.id));
            }
        }
    }
    pairs
}

/// Query two bodies. The returned orientation and IDs are canonical even if the
/// arguments are reversed.
pub fn query_pair(a: &RigidBody, b: &RigidBody) -> Option<Proximity> {
    if !a.enabled
        || !b.enabled
        || a.id == b.id
        || !a.shape.is_valid()
        || !b.shape.is_valid()
        || !a.collision_filter.allows(b.collision_filter)
    {
        return None;
    }
    if a.id < b.id {
        Some(query_shapes(a.id, a.pose, a.shape, b.id, b.pose, b.shape))
    } else {
        Some(query_shapes(b.id, b.pose, b.shape, a.id, a.pose, a.shape))
    }
}

fn query_shapes(
    id_a: BodyId,
    pose_a: Pose,
    shape_a: Shape,
    id_b: BodyId,
    pose_b: Pose,
    shape_b: Shape,
) -> Proximity {
    use Shape::*;
    let raw = match (shape_a, shape_b) {
        (Sphere { radius_m: ra }, Sphere { radius_m: rb }) => {
            sphere_sphere(id_a, pose_a.translation, ra, id_b, pose_b.translation, rb)
        }
        (
            Sphere { radius_m },
            Capsule {
                radius_m: capsule_radius,
                half_segment_m,
            },
        ) => sphere_capsule(
            id_a,
            pose_a.translation,
            radius_m,
            id_b,
            capsule_segment(pose_b, half_segment_m),
            capsule_radius,
        ),
        (Capsule { .. }, Sphere { .. }) => {
            query_shapes(id_b, pose_b, shape_b, id_a, pose_a, shape_a).flipped()
        }
        (Sphere { radius_m }, Box { half_extents_m }) => sphere_box(
            id_a,
            pose_a.translation,
            radius_m,
            id_b,
            pose_b,
            half_extents_m,
        ),
        (Box { .. }, Sphere { .. }) => {
            query_shapes(id_b, pose_b, shape_b, id_a, pose_a, shape_a).flipped()
        }
        (
            Capsule {
                radius_m: ra,
                half_segment_m: ha,
            },
            Capsule {
                radius_m: rb,
                half_segment_m: hb,
            },
        ) => capsule_capsule(
            id_a,
            capsule_segment(pose_a, ha),
            ra,
            id_b,
            capsule_segment(pose_b, hb),
            rb,
        ),
        (
            Capsule {
                radius_m,
                half_segment_m,
            },
            Box { half_extents_m },
        ) => capsule_box(
            id_a,
            capsule_segment(pose_a, half_segment_m),
            radius_m,
            id_b,
            pose_b,
            half_extents_m,
        ),
        (Box { .. }, Capsule { .. }) => {
            query_shapes(id_b, pose_b, shape_b, id_a, pose_a, shape_a).flipped()
        }
        (Box { half_extents_m: ha }, Box { half_extents_m: hb }) => box_box(
            id_a,
            pose_a,
            ha,
            id_b,
            pose_b,
            hb,
            ContactKind::OrientedBoxSatApproximation,
        ),
        (Sphere { radius_m }, Gear(gear)) => {
            sphere_gear(id_a, pose_a.translation, radius_m, id_b, pose_b, gear)
        }
        (Gear(_), Sphere { .. }) => {
            query_shapes(id_b, pose_b, shape_b, id_a, pose_a, shape_a).flipped()
        }
        (
            Capsule {
                radius_m,
                half_segment_m,
            },
            Gear(gear),
        ) => capsule_gear(
            id_a,
            capsule_segment(pose_a, half_segment_m),
            radius_m,
            id_b,
            pose_b,
            gear,
        ),
        (Gear(_), Capsule { .. }) => {
            query_shapes(id_b, pose_b, shape_b, id_a, pose_a, shape_a).flipped()
        }
        (Gear(gear_a), Gear(gear_b)) => gear_gear(id_a, pose_a, gear_a, id_b, pose_b, gear_b),
        (Gear(gear), Box { half_extents_m }) => {
            gear_box(id_a, pose_a, gear, id_b, pose_b, half_extents_m)
        }
        (Box { .. }, Gear(_)) => {
            query_shapes(id_b, pose_b, shape_b, id_a, pose_a, shape_a).flipped()
        }
    };
    // `query_shapes` reversals can temporarily invert IDs. Restore the public
    // canonical orientation once, at the outer return boundary.
    if raw.body_a <= raw.body_b {
        raw
    } else {
        raw.flipped()
    }
}

fn fallback_normal(id_a: BodyId, id_b: BodyId) -> Vec3 {
    // Stable and nonzero for coincident primitives; ID ordering is canonical.
    match (id_a.0 ^ id_b.0) % 3 {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

fn sphere_sphere(
    id_a: BodyId,
    center_a: Vec3,
    radius_a: f64,
    id_b: BodyId,
    center_b: Vec3,
    radius_b: f64,
) -> Proximity {
    let delta = center_b - center_a;
    let distance = delta.length();
    let normal = delta.normalized_or(fallback_normal(id_a, id_b));
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: center_a + normal * radius_a,
        point_b_world_m: center_b - normal * radius_b,
        normal_a_to_b: normal,
        signed_distance_m: distance - radius_a - radius_b,
        kind: ContactKind::ExactAnalytic,
    }
}

fn capsule_segment(pose: Pose, half_segment_m: f64) -> (Vec3, Vec3) {
    (
        pose.transform_point(Vec3::Z * -half_segment_m),
        pose.transform_point(Vec3::Z * half_segment_m),
    )
}

fn closest_point_on_segment(point: Vec3, segment: (Vec3, Vec3)) -> Vec3 {
    let delta = segment.1 - segment.0;
    let denominator = delta.length_squared();
    if denominator <= SEGMENT_EPSILON_SQUARED_M2 {
        segment.0
    } else {
        segment.0 + delta * ((point - segment.0).dot(delta) / denominator).clamp(0.0, 1.0)
    }
}

fn sphere_capsule(
    id_a: BodyId,
    center_a: Vec3,
    radius_a: f64,
    id_b: BodyId,
    segment_b: (Vec3, Vec3),
    radius_b: f64,
) -> Proximity {
    let axis_b = closest_point_on_segment(center_a, segment_b);
    sphere_sphere(id_a, center_a, radius_a, id_b, axis_b, radius_b)
}

fn closest_segment_points(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> (Vec3, Vec3) {
    // Christer Ericson, Real-Time Collision Detection, section 5.1.9.
    let d1 = a.1 - a.0;
    let d2 = b.1 - b.0;
    let r = a.0 - b.0;
    let aa = d1.dot(d1);
    let ee = d2.dot(d2);
    let ff = d2.dot(r);
    let (mut s, mut t);
    if aa <= SEGMENT_EPSILON_SQUARED_M2 && ee <= SEGMENT_EPSILON_SQUARED_M2 {
        return (a.0, b.0);
    } else if aa <= SEGMENT_EPSILON_SQUARED_M2 {
        s = 0.0;
        t = (ff / ee).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if ee <= SEGMENT_EPSILON_SQUARED_M2 {
            t = 0.0;
            s = (-c / aa).clamp(0.0, 1.0);
        } else {
            let bb = d1.dot(d2);
            let denominator = aa * ee - bb * bb;
            // Use a relative parallelism test: `denominator` has units m^4 and
            // must not be compared with a metre-valued absolute epsilon.
            s = if denominator > 1.0e-12 * aa * ee {
                ((bb * ff - c * ee) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (bb * s + ff) / ee;
            if t < 0.0 {
                t = 0.0;
                s = (-c / aa).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((bb - c) / aa).clamp(0.0, 1.0);
            }
        }
    }
    (a.0 + d1 * s, b.0 + d2 * t)
}

fn capsule_capsule(
    id_a: BodyId,
    segment_a: (Vec3, Vec3),
    radius_a: f64,
    id_b: BodyId,
    segment_b: (Vec3, Vec3),
    radius_b: f64,
) -> Proximity {
    let (axis_a, axis_b) = closest_segment_points(segment_a, segment_b);
    sphere_sphere(id_a, axis_a, radius_a, id_b, axis_b, radius_b)
}

/// Point versus solid OBB. Returns signed distance, direction from the point
/// toward the box for separation/resolution, and a boundary witness.
fn point_box(point_world: Vec3, box_pose: Pose, half: Vec3) -> (f64, Vec3, Vec3) {
    let point = box_pose.inverse_transform_point(point_world);
    let closest = point.clamp(-half, half);
    let outside_delta = closest - point;
    let outside_distance = outside_delta.length();
    if outside_distance > EPSILON {
        let normal_local = outside_delta / outside_distance;
        return (
            outside_distance,
            box_pose.transform_vector(normal_local),
            box_pose.transform_point(closest),
        );
    }

    let distances = [
        half.x - point.x.abs(),
        half.y - point.y.abs(),
        half.z - point.z.abs(),
    ];
    let mut axis = 0;
    if distances[1] < distances[axis] {
        axis = 1;
    }
    if distances[2] < distances[axis] {
        axis = 2;
    }
    let sign = if point.component(axis) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let outward_local = Vec3::ZERO.with_component(axis, sign);
    let boundary = point.with_component(axis, half.component(axis) * sign);
    (
        -distances[axis],
        box_pose.transform_vector(-outward_local),
        box_pose.transform_point(boundary),
    )
}

fn sphere_box(
    id_a: BodyId,
    center_a: Vec3,
    radius_a: f64,
    id_b: BodyId,
    box_pose: Pose,
    half: Vec3,
) -> Proximity {
    let (point_distance, normal, box_witness) = point_box(center_a, box_pose, half);
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: center_a + normal * radius_a,
        point_b_world_m: box_witness,
        normal_a_to_b: normal,
        signed_distance_m: point_distance - radius_a,
        kind: ContactKind::ExactAnalytic,
    }
}

fn capsule_box(
    id_a: BodyId,
    segment: (Vec3, Vec3),
    radius_m: f64,
    id_b: BodyId,
    box_pose: Pose,
    half: Vec3,
) -> Proximity {
    let delta = segment.1 - segment.0;
    let evaluate = |t: f64| point_box(segment.0 + delta * t, box_pose, half).0;
    // Signed point-box distance is not globally convex inside the box. Seed a
    // fixed grid, then refine the best interval deterministically.
    const SEEDS: usize = 17;
    let mut best_t = 0.0;
    let mut best_value = evaluate(0.0);
    for index in 1..SEEDS {
        let t = index as f64 / (SEEDS - 1) as f64;
        let value = evaluate(t);
        if value < best_value {
            best_t = t;
            best_value = value;
        }
    }
    let seed_step = 1.0 / (SEEDS - 1) as f64;
    let mut left = (best_t - seed_step).max(0.0);
    let mut right = (best_t + seed_step).min(1.0);
    for _ in 0..40 {
        let t1 = (2.0 * left + right) / 3.0;
        let t2 = (left + 2.0 * right) / 3.0;
        if evaluate(t1) <= evaluate(t2) {
            right = t2;
        } else {
            left = t1;
        }
    }
    best_t = (left + right) * 0.5;
    let axis_point = segment.0 + delta * best_t;
    let (point_distance, normal, box_witness) = point_box(axis_point, box_pose, half);
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: axis_point + normal * radius_m,
        point_b_world_m: box_witness,
        normal_a_to_b: normal,
        signed_distance_m: point_distance - radius_m,
        kind: ContactKind::CapsuleBoxApproximation,
    }
}

fn support_box(pose: Pose, half: Vec3, direction: Vec3) -> Vec3 {
    let rotation = pose.rotation.to_mat3();
    let mut point = pose.translation;
    for index in 0..3 {
        let axis = rotation.column(index);
        point += axis
            * half.component(index)
            * if axis.dot(direction) >= 0.0 {
                1.0
            } else {
                -1.0
            };
    }
    point
}

fn box_box(
    id_a: BodyId,
    pose_a: Pose,
    half_a: Vec3,
    id_b: BodyId,
    pose_b: Pose,
    half_b: Vec3,
    kind: ContactKind,
) -> Proximity {
    let rotation_a = pose_a.rotation.to_mat3();
    let rotation_b = pose_b.rotation.to_mat3();
    let mut axes = Vec::with_capacity(15);
    for index in 0..3 {
        axes.push(rotation_a.column(index));
    }
    for index in 0..3 {
        axes.push(rotation_b.column(index));
    }
    for a in 0..3 {
        for b in 0..3 {
            let cross = rotation_a.column(a).cross(rotation_b.column(b));
            if cross.length_squared() > 1.0e-20 {
                axes.push(cross.normalized());
            }
        }
    }
    let center_delta = pose_b.translation - pose_a.translation;
    let mut best_gap = f64::NEG_INFINITY;
    let mut best_normal = fallback_normal(id_a, id_b);
    for mut axis in axes {
        if center_delta.dot(axis) < 0.0 {
            axis = -axis;
        }
        let ra = (0..3)
            .map(|index| rotation_a.column(index).dot(axis).abs() * half_a.component(index))
            .sum::<f64>();
        let rb = (0..3)
            .map(|index| rotation_b.column(index).dot(axis).abs() * half_b.component(index))
            .sum::<f64>();
        let gap = center_delta.dot(axis).abs() - ra - rb;
        if gap > best_gap {
            best_gap = gap;
            best_normal = axis;
        }
    }
    let point_a = support_box(pose_a, half_a, best_normal);
    let point_b = support_box(pose_b, half_b, -best_normal);
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: point_a,
        point_b_world_m: point_b,
        normal_a_to_b: best_normal,
        signed_distance_m: best_gap,
        kind,
    }
}

fn gear_box(
    id_gear: BodyId,
    gear_pose: Pose,
    gear: GearGeometry,
    id_box: BodyId,
    box_pose: Pose,
    box_half_extents: Vec3,
) -> Proximity {
    let hub = box_box(
        id_gear,
        gear_pose,
        Vec3::new(
            gear.hub_radius_m,
            gear.hub_radius_m,
            gear.half_total_height_m,
        ),
        id_box,
        box_pose,
        box_half_extents,
        ContactKind::GearBoxEnvelopeApproximation,
    );
    let tooth_pose = gear_pose * Pose::from_translation(Vec3::Z * gear.tooth_center_offset_m);
    let teeth = box_box(
        id_gear,
        tooth_pose,
        Vec3::new(gear.tip_radius_m, gear.tip_radius_m, gear.half_thickness_m),
        id_box,
        box_pose,
        box_half_extents,
        ContactKind::GearBoxEnvelopeApproximation,
    );
    if teeth.signed_distance_m <= hub.signed_distance_m {
        teeth
    } else {
        hub
    }
}

/// Signed point proximity to one local-space annular finite cylinder.
fn point_annular_cylinder(
    local: Vec3,
    bore_radius_m: f64,
    outer_radius_m: f64,
    half_height_m: f64,
    center_z_m: f64,
) -> (f64, Vec3, Vec3) {
    let radial = local.x.hypot(local.y);
    let radial_dir = if radial > EPSILON {
        Vec3::new(local.x / radial, local.y / radial, 0.0)
    } else {
        Vec3::X
    };
    let relative_z = local.z - center_z_m;
    let abs_z = relative_z.abs();
    let z_sign = if relative_z >= 0.0 { 1.0 } else { -1.0 };
    let in_radial_band = radial >= bore_radius_m && radial <= outer_radius_m;
    let in_z_band = abs_z <= half_height_m;

    let (signed, normal_local, witness_local) = if in_radial_band && in_z_band {
        let to_inner = radial - bore_radius_m;
        let to_outer = outer_radius_m - radial;
        let to_face = half_height_m - abs_z;
        if to_inner <= to_outer && to_inner <= to_face {
            // Escape toward bore is -radial; normal is opposite escape.
            (
                -to_inner,
                radial_dir,
                radial_dir * bore_radius_m + Vec3::Z * local.z,
            )
        } else if to_outer <= to_face {
            (
                -to_outer,
                -radial_dir,
                radial_dir * outer_radius_m + Vec3::Z * local.z,
            )
        } else {
            (
                -to_face,
                Vec3::Z * -z_sign,
                Vec3::new(local.x, local.y, center_z_m + z_sign * half_height_m),
            )
        }
    } else {
        let target_radius = radial.clamp(bore_radius_m, outer_radius_m);
        let target_z = relative_z.clamp(-half_height_m, half_height_m) + center_z_m;
        let witness = radial_dir * target_radius + Vec3::Z * target_z;
        let delta = witness - local;
        let distance = delta.length();
        (distance, delta.normalized_or(Vec3::Z), witness)
    };
    (signed, normal_local, witness_local)
}

/// Point versus the union of a full-height annular hub and the offset annular
/// tooth envelope, both expressed in gear-local coordinates.
fn point_gear(point_world: Vec3, gear_pose: Pose, gear: GearGeometry) -> (f64, Vec3, Vec3) {
    let local = gear_pose.inverse_transform_point(point_world);
    let hub = point_annular_cylinder(
        local,
        gear.bore_radius_m,
        gear.hub_radius_m,
        gear.half_total_height_m,
        0.0,
    );
    let teeth = point_annular_cylinder(
        local,
        gear.bore_radius_m,
        gear.tip_radius_m,
        gear.half_thickness_m,
        gear.tooth_center_offset_m,
    );
    let (signed, normal_local, witness_local) = if teeth.0 <= hub.0 { teeth } else { hub };
    (
        signed,
        gear_pose.transform_vector(normal_local),
        gear_pose.transform_point(witness_local),
    )
}

fn sphere_gear(
    id_a: BodyId,
    center_a: Vec3,
    radius_a: f64,
    id_b: BodyId,
    gear_pose: Pose,
    gear: GearGeometry,
) -> Proximity {
    let (point_distance, normal, gear_witness) = point_gear(center_a, gear_pose, gear);
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: center_a + normal * radius_a,
        point_b_world_m: gear_witness,
        normal_a_to_b: normal,
        signed_distance_m: point_distance - radius_a,
        kind: ContactKind::GearAnnularEnvelopeApproximation,
    }
}

fn capsule_gear(
    id_a: BodyId,
    segment: (Vec3, Vec3),
    radius_a: f64,
    id_b: BodyId,
    gear_pose: Pose,
    gear: GearGeometry,
) -> Proximity {
    let delta = segment.1 - segment.0;
    const SEEDS: usize = 33;
    let mut best_t = 0.0;
    let mut best_distance = f64::INFINITY;
    for index in 0..SEEDS {
        let t = index as f64 / (SEEDS - 1) as f64;
        let distance = point_gear(segment.0 + delta * t, gear_pose, gear).0;
        if distance < best_distance {
            best_distance = distance;
            best_t = t;
        }
    }
    let step = 1.0 / (SEEDS - 1) as f64;
    let mut left = (best_t - step).max(0.0);
    let mut right = (best_t + step).min(1.0);
    for _ in 0..40 {
        let t1 = (2.0 * left + right) / 3.0;
        let t2 = (left + 2.0 * right) / 3.0;
        if point_gear(segment.0 + delta * t1, gear_pose, gear).0
            <= point_gear(segment.0 + delta * t2, gear_pose, gear).0
        {
            right = t2;
        } else {
            left = t1;
        }
    }
    best_t = (left + right) * 0.5;
    let axis_point = segment.0 + delta * best_t;
    let (point_distance, normal, gear_witness) = point_gear(axis_point, gear_pose, gear);
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: axis_point + normal * radius_a,
        point_b_world_m: gear_witness,
        normal_a_to_b: normal,
        signed_distance_m: point_distance - radius_a,
        kind: ContactKind::GearAnnularEnvelopeApproximation,
    }
}

pub fn gear_mesh_clearance(a: &RigidBody, b: &RigidBody) -> Option<GearMeshClearance> {
    let (gear_a, gear_b) = match (a.shape, b.shape) {
        (Shape::Gear(a), Shape::Gear(b)) => (a, b),
        _ => return None,
    };
    let (a, gear_a, b, gear_b) = if a.id <= b.id {
        (a, gear_a, b, gear_b)
    } else {
        (b, gear_b, a, gear_a)
    };
    let axis_a = a.pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
    let axis_b = b.pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
    let axes_parallel = axis_a.dot(axis_b).abs() >= 0.999;
    let delta = b.pose.translation - a.pose.translation;
    let tooth_center_a = a
        .pose
        .transform_point(Vec3::Z * gear_a.tooth_center_offset_m);
    let tooth_center_b = b
        .pose
        .transform_point(Vec3::Z * gear_b.tooth_center_offset_m);
    let axial_distance = (tooth_center_b - tooth_center_a).dot(axis_a).abs();
    let radial_delta = delta - axis_a * delta.dot(axis_a);
    let center_distance_m = radial_delta.length();
    let ideal_center_distance_m = gear_a.pitch_radius_m + gear_b.pitch_radius_m;
    let center_offset_m = center_distance_m - ideal_center_distance_m;
    let minimum_root_tip_clearance_m =
        (center_distance_m - gear_a.tip_radius_m - gear_b.root_radius_m)
            .min(center_distance_m - gear_a.root_radius_m - gear_b.tip_radius_m);
    let axial_overlap_m =
        (gear_a.half_thickness_m + gear_b.half_thickness_m - axial_distance).max(0.0);
    let pressure_angle = 0.5 * (gear_a.pressure_angle_rad + gear_b.pressure_angle_rad);
    Some(GearMeshClearance {
        body_a: a.id,
        body_b: b.id,
        axes_parallel,
        modules_compatible: (gear_a.module_m - gear_b.module_m).abs()
            <= 1.0e-6 * gear_a.module_m.max(gear_b.module_m),
        center_distance_m,
        ideal_center_distance_m,
        center_offset_m,
        minimum_root_tip_clearance_m,
        axial_overlap_m,
        estimated_backlash_m: 2.0 * center_offset_m * pressure_angle.tan(),
    })
}

fn gear_gear(
    id_a: BodyId,
    pose_a: Pose,
    gear_a: GearGeometry,
    id_b: BodyId,
    pose_b: Pose,
    gear_b: GearGeometry,
) -> Proximity {
    let body_a = RigidBody::new(
        id_a,
        Shape::Gear(gear_a),
        pose_a,
        crate::geometry::MotionType::Static,
    );
    let body_b = RigidBody::new(
        id_b,
        Shape::Gear(gear_b),
        pose_b,
        crate::geometry::MotionType::Static,
    );
    let clearance = gear_mesh_clearance(&body_a, &body_b).expect("gear pair");
    let axis = pose_a.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
    let center_delta = pose_b.translation - pose_a.translation;
    let axial_component = center_delta.dot(axis);
    let radial = center_delta - axis * axial_component;
    let tooth_center_a = pose_a.transform_point(Vec3::Z * gear_a.tooth_center_offset_m);
    let tooth_center_b = pose_b.transform_point(Vec3::Z * gear_b.tooth_center_offset_m);
    let tooth_axial_component = (tooth_center_b - tooth_center_a).dot(axis);

    if !clearance.axes_parallel {
        return box_box(
            id_a,
            pose_a,
            Vec3::new(
                gear_a.tip_radius_m,
                gear_a.tip_radius_m,
                gear_a.half_total_height_m,
            ),
            id_b,
            pose_b,
            Vec3::new(
                gear_b.tip_radius_m,
                gear_b.tip_radius_m,
                gear_b.half_total_height_m,
            ),
            ContactKind::GearBoxEnvelopeApproximation,
        );
    }

    if clearance.axial_overlap_m <= 0.0 {
        let normal = axis
            * if tooth_axial_component >= 0.0 {
                1.0
            } else {
                -1.0
            };
        let signed_distance =
            tooth_axial_component.abs() - gear_a.half_thickness_m - gear_b.half_thickness_m;
        return Proximity {
            body_a: id_a,
            body_b: id_b,
            point_a_world_m: tooth_center_a + normal * gear_a.half_thickness_m,
            point_b_world_m: tooth_center_b - normal * gear_b.half_thickness_m,
            normal_a_to_b: normal,
            signed_distance_m: signed_distance,
            kind: ContactKind::GearMeshApproximation,
        };
    }

    let normal = radial.normalized_or(fallback_normal(id_a, id_b));
    // At useful external-mesh spacing, pitch circles define flank contact while
    // root-tip clearance guards gross interference. At near-coincident centers,
    // the root disks dominate and must report penetration.
    let signed_distance = clearance
        .center_offset_m
        .min(clearance.minimum_root_tip_clearance_m);
    Proximity {
        body_a: id_a,
        body_b: id_b,
        point_a_world_m: tooth_center_a + normal * gear_a.pitch_radius_m,
        point_b_world_m: tooth_center_b - normal * gear_b.pitch_radius_m,
        normal_a_to_b: normal,
        signed_distance_m: signed_distance,
        kind: ContactKind::GearMeshApproximation,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{MotionType, Shape};
    use crate::math::Quat;

    fn body(id: u32, shape: Shape, translation: Vec3) -> RigidBody {
        RigidBody::new(
            BodyId(id),
            shape,
            Pose::from_translation(translation),
            MotionType::Static,
        )
    }

    #[test]
    fn sphere_signed_distance_has_micrometre_resolution() {
        let a = body(1, Shape::Sphere { radius_m: 0.25e-3 }, Vec3::ZERO);
        for (offset, expected) in [(0.0, 0.0), (2.0e-6, 2.0e-6), (-2.0e-6, -2.0e-6)] {
            let b = body(
                2,
                Shape::Sphere { radius_m: 0.25e-3 },
                Vec3::X * (0.5e-3 + offset),
            );
            assert!((query_pair(&a, &b).unwrap().signed_distance_m - expected).abs() < 1.0e-15);
        }
    }

    #[test]
    fn reversed_arguments_keep_canonical_witnesses() {
        let a = body(7, Shape::Sphere { radius_m: 1.0 }, Vec3::ZERO);
        let b = body(2, Shape::Sphere { radius_m: 1.0 }, Vec3::X * 3.0);
        assert_eq!(query_pair(&a, &b), query_pair(&b, &a));
        assert_eq!(query_pair(&a, &b).unwrap().body_a, BodyId(2));
    }

    #[test]
    fn coincident_spheres_never_generate_nan() {
        let a = body(1, Shape::Sphere { radius_m: 1.0 }, Vec3::ZERO);
        let b = body(2, Shape::Sphere { radius_m: 1.0 }, Vec3::ZERO);
        let result = query_pair(&a, &b).unwrap();
        assert!(result.normal_a_to_b.is_finite());
        assert!((result.normal_a_to_b.length() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn zero_length_capsule_matches_sphere() {
        let sphere = body(1, Shape::Sphere { radius_m: 0.5 }, Vec3::ZERO);
        let capsule = body(
            2,
            Shape::Capsule {
                radius_m: 0.25,
                half_segment_m: 0.0,
            },
            Vec3::X,
        );
        assert!((query_pair(&sphere, &capsule).unwrap().signed_distance_m - 0.25).abs() < 1e-14);
    }

    #[test]
    fn millimetre_scale_perpendicular_capsules_find_line_crossing() {
        let mut a = body(
            1,
            Shape::Capsule {
                radius_m: 5.0e-6,
                half_segment_m: 0.5e-3,
            },
            Vec3::ZERO,
        );
        a.pose.rotation = Quat::from_two_vectors(Vec3::Z, Vec3::X);
        let mut b = body(
            2,
            Shape::Capsule {
                radius_m: 5.0e-6,
                half_segment_m: 0.5e-3,
            },
            Vec3::ZERO,
        );
        b.pose.rotation = Quat::from_two_vectors(Vec3::Z, Vec3::Y);
        assert!((query_pair(&a, &b).unwrap().signed_distance_m + 10.0e-6).abs() < 1.0e-12);
    }

    #[test]
    fn sphere_inside_rotated_box_reports_penetration() {
        let sphere = body(1, Shape::Sphere { radius_m: 0.1 }, Vec3::ZERO);
        let mut box_body = body(
            2,
            Shape::Box {
                half_extents_m: Vec3::splat(1.0),
            },
            Vec3::ZERO,
        );
        box_body.pose.rotation = Quat::from_axis_angle(Vec3::Z, 0.3);
        let result = query_pair(&sphere, &box_body).unwrap();
        assert!((result.signed_distance_m + 1.1).abs() < 1e-12);
        assert!(result.normal_a_to_b.is_finite());
    }

    #[test]
    fn capsule_box_detects_crossing_segment() {
        let capsule = body(
            1,
            Shape::Capsule {
                radius_m: 0.1,
                half_segment_m: 2.0,
            },
            Vec3::ZERO,
        );
        let box_body = body(
            2,
            Shape::Box {
                half_extents_m: Vec3::splat(0.5),
            },
            Vec3::ZERO,
        );
        assert!(query_pair(&capsule, &box_body).unwrap().signed_distance_m < -0.1);
    }

    #[test]
    fn broad_phase_is_insertion_order_invariant() {
        let a = body(3, Shape::Sphere { radius_m: 1.0 }, Vec3::ZERO);
        let b = body(1, Shape::Sphere { radius_m: 1.0 }, Vec3::X * 1.5);
        let c = body(2, Shape::Sphere { radius_m: 1.0 }, Vec3::X * 10.0);
        assert_eq!(
            broad_phase_pairs(&[a.clone(), b.clone(), c.clone()], 0.0),
            broad_phase_pairs(&[c, a, b], 0.0)
        );
    }

    #[test]
    fn shaft_clearance_respects_annular_gear_bore() {
        let gear_geometry =
            GearGeometry::uniform_spur(16, 0.1e-3, 25.0_f64.to_radians(), 0.5e-3, 0.15e-3);
        let gear = body(2, Shape::Gear(gear_geometry), Vec3::ZERO);
        let narrow_shaft = body(
            1,
            Shape::Capsule {
                radius_m: 0.145e-3,
                half_segment_m: 1.0e-3,
            },
            Vec3::ZERO,
        );
        let wide_shaft = body(
            3,
            Shape::Capsule {
                radius_m: 0.155e-3,
                half_segment_m: 1.0e-3,
            },
            Vec3::ZERO,
        );
        assert!(
            (query_pair(&narrow_shaft, &gear).unwrap().signed_distance_m - 5.0e-6).abs() < 1.0e-9
        );
        assert!(
            (query_pair(&wide_shaft, &gear).unwrap().signed_distance_m + 5.0e-6).abs() < 1.0e-9
        );
    }

    #[test]
    fn stepped_gear_does_not_extend_teeth_through_full_hub_height() {
        let geometry = GearGeometry::spur_with_hub(
            12,
            0.1e-3,
            25.0_f64.to_radians(),
            0.35e-3,
            0.21e-3,
            0.275e-3,
            1.30e-3,
        );
        let gear = body(2, Shape::Gear(geometry), Vec3::ZERO);
        let near_upper_hub = body(
            1,
            Shape::Sphere { radius_m: 0.05e-3 },
            Vec3::new(0.50e-3, 0.0, 0.50e-3),
        );
        let in_tooth_band = body(
            3,
            Shape::Sphere { radius_m: 0.05e-3 },
            Vec3::new(0.50e-3, 0.0, geometry.tooth_center_offset_m),
        );
        let upper = query_pair(&near_upper_hub, &gear).unwrap();
        assert!((upper.signed_distance_m - 0.175e-3).abs() < 1.0e-9);
        assert!(query_pair(&gear, &in_tooth_band).unwrap().signed_distance_m < 0.0);
    }

    #[test]
    fn nominal_module_matched_gears_touch_at_pitch_circles() {
        let ga = GearGeometry::uniform_spur(12, 0.1e-3, 25.0_f64.to_radians(), 0.4e-3, 0.12e-3);
        let gb = GearGeometry::uniform_spur(24, 0.1e-3, 25.0_f64.to_radians(), 0.4e-3, 0.12e-3);
        let a = body(1, Shape::Gear(ga), Vec3::ZERO);
        let b = body(
            2,
            Shape::Gear(gb),
            Vec3::X * (ga.pitch_radius_m + gb.pitch_radius_m),
        );
        let mesh = gear_mesh_clearance(&a, &b).unwrap();
        assert!(mesh.modules_compatible);
        assert!(mesh.minimum_root_tip_clearance_m > 0.0);
        assert!(query_pair(&a, &b).unwrap().signed_distance_m.abs() < 1e-15);
    }

    #[test]
    fn report_separates_contacts_from_metrology_clearances() {
        let settings = CollisionSettings {
            contact_offset_m: 1.0e-6,
            clearance_threshold_m: 20.0e-6,
            numeric_epsilon_m: 1.0e-12,
        };
        let a = body(1, Shape::Sphere { radius_m: 0.5e-3 }, Vec3::ZERO);
        let b = body(2, Shape::Sphere { radius_m: 0.5e-3 }, Vec3::X * 1.01e-3);
        let report = CollisionReport::query(&[a, b], settings);
        assert!(report.contacts.is_empty());
        assert_eq!(report.clearances.len(), 1);
        assert!((report.clearances[0].distance_m - 10.0e-6).abs() < 1e-15);
    }
}
