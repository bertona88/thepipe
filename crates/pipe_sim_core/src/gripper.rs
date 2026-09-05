//! Simple parallel-jaw micro-gripper.

use crate::geometry::{BodyId, RigidBody, Shape};
use crate::math::{Pose, Vec3};

const CONTACT_EPSILON_M: f64 = 1.0e-12;

/// Smallest axial material overlap accepted by the opt-in partial-overlap
/// grasp model.  A caller may demand more overlap, but may not weaken this
/// mechanics floor.
pub const MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M: f64 = 100.0e-6;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperConfig {
    pub min_opening_m: f64,
    pub max_opening_m: f64,
    pub max_speed_m_s: f64,
    pub jaw_half_extents_m: Vec3,
    pub pad_compliance_m: f64,
    pub max_grip_force_n: f64,
}

impl Default for GripperConfig {
    fn default() -> Self {
        Self {
            min_opening_m: 80.0e-6,
            max_opening_m: 2.8e-3,
            max_speed_m_s: 4.0e-3,
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 600.0e-6),
            pad_compliance_m: 12.0e-6,
            max_grip_force_n: 0.15,
        }
    }
}

impl GripperConfig {
    pub fn is_valid(self) -> bool {
        self.min_opening_m >= 0.0
            && self.max_opening_m >= self.min_opening_m
            && self.max_speed_m_s > 0.0
            && self.jaw_half_extents_m.x > 0.0
            && self.jaw_half_extents_m.y > 0.0
            && self.jaw_half_extents_m.z > 0.0
            && self.pad_compliance_m >= 0.0
            && self.max_grip_force_n > 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspCandidate {
    pub body_id: BodyId,
    pub required_opening_m: f64,
    pub center_error_m: Vec3,
    /// Positive means free space; negative means geometric interference.
    pub jaw_clearance_m: f64,
    pub within_finger_depth: bool,
    /// Local-Z overlap of the full-cross-section material and the jaw pad span.
    /// For a capsule this is the centerline segment overlap, excluding rounded
    /// caps. This is diagnostic for legacy candidates and an enforced gate for
    /// candidates carrying `minimum_axial_overlap_m`.
    pub axial_overlap_m: f64,
    /// `Some` only for the explicit partial axial-overlap evaluation path.
    pub minimum_axial_overlap_m: Option<f64>,
}

impl GraspCandidate {
    pub fn is_reachable(self, config: GripperConfig) -> bool {
        let axial_overlap_is_sufficient = match self.minimum_axial_overlap_m {
            Some(minimum) => {
                minimum.is_finite()
                    && minimum >= MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M
                    && self.axial_overlap_m >= minimum
            }
            None => true,
        };
        self.required_opening_m <= config.max_opening_m + 2.0 * config.pad_compliance_m
            && self.required_opening_m >= config.min_opening_m - 2.0 * config.pad_compliance_m
            && self.within_finger_depth
            && axial_overlap_is_sufficient
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperState {
    pub command_opening_m: f64,
    pub opening_m: f64,
    pub opening_velocity_m_s: f64,
    pub held_body: Option<BodyId>,
    pub estimated_grip_force_n: f64,
    /// Retained only for a held body acquired through the opt-in partial
    /// axial-overlap path. Legacy grasps remain `None`.
    pub held_minimum_axial_overlap_m: Option<f64>,
}

impl GripperState {
    pub fn new(opening_m: f64, config: GripperConfig) -> Self {
        let opening_m = opening_m.clamp(config.min_opening_m, config.max_opening_m);
        Self {
            command_opening_m: opening_m,
            opening_m,
            opening_velocity_m_s: 0.0,
            held_body: None,
            estimated_grip_force_n: 0.0,
            held_minimum_axial_overlap_m: None,
        }
    }

    pub fn set_command(&mut self, opening_m: f64, config: GripperConfig) {
        self.command_opening_m = opening_m.clamp(config.min_opening_m, config.max_opening_m);
    }

    /// Hold the current jaw opening without dropping an attached body.
    pub fn stop(&mut self) {
        self.command_opening_m = self.opening_m;
        self.opening_velocity_m_s = 0.0;
    }

    pub fn step(&mut self, dt_s: f64, config: GripperConfig) {
        if dt_s <= 0.0 || !dt_s.is_finite() || !config.is_valid() {
            return;
        }
        let error = self.command_opening_m - self.opening_m;
        let delta = error.signum() * error.abs().min(config.max_speed_m_s * dt_s);
        self.opening_m += delta;
        self.opening_velocity_m_s = delta / dt_s;
        if self.held_body.is_none() {
            self.estimated_grip_force_n = 0.0;
        }
    }

    pub fn jaw_poses(self, tool_pose: Pose, config: GripperConfig) -> [Pose; 2] {
        let offset = self.opening_m * 0.5 + config.jaw_half_extents_m.x;
        [
            tool_pose * Pose::from_translation(Vec3::new(-offset, 0.0, 0.0)),
            tool_pose * Pose::from_translation(Vec3::new(offset, 0.0, 0.0)),
        ]
    }

    pub fn jaw_shapes(config: GripperConfig) -> [Shape; 2] {
        let shape = Shape::Box {
            half_extents_m: config.jaw_half_extents_m,
        };
        [shape, shape]
    }

    /// Candidate test using an AABB evaluated directly in the tool frame.
    /// Computing a world AABB first and rotating its corners back would apply
    /// the orientation envelope twice and can reject slender, correctly
    /// aligned parts.
    pub fn evaluate_candidate(
        self,
        tool_pose: Pose,
        body: &RigidBody,
        config: GripperConfig,
    ) -> GraspCandidate {
        let local_aabb = body.shape.aabb(tool_pose.inverse() * body.pose);
        let local_min = local_aabb.min;
        let local_max = local_aabb.max;
        let size = local_max - local_min;
        let center = (local_min + local_max) * 0.5;
        let depth = config.jaw_half_extents_m;
        let within_finger_depth =
            center.y.abs() + size.y * 0.5 <= depth.y && center.z.abs() + size.z * 0.5 <= depth.z;
        GraspCandidate {
            body_id: body.id,
            required_opening_m: size.x,
            center_error_m: center,
            jaw_clearance_m: self.opening_m - size.x,
            within_finger_depth,
            axial_overlap_m: full_cross_section_axial_overlap_m(
                body.shape,
                tool_pose.inverse() * body.pose,
                -depth.z,
                depth.z,
            ),
            minimum_axial_overlap_m: None,
        }
    }

    /// Evaluate a shaft-like body whose center may lie outside the local-Z jaw
    /// span while a sufficient portion of the body still overlaps both pads.
    ///
    /// This is deliberately opt-in: [`Self::evaluate_candidate`] retains its
    /// full axial-containment rule. The caller-provided threshold must be at
    /// least [`MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M`], otherwise the candidate is
    /// fail-closed. Local-Y containment, lateral centering, jaw contact, and
    /// pad-compression limits remain unchanged.
    ///
    /// Capsules are the only shape accepted by this reduced shaft model. Its
    /// overlap gate excludes the rounded caps. Required opening and lateral
    /// center error use a conservative bound over the full-radius shaft land
    /// intersecting the pad slab, including centerline drift caused by
    /// peg/tool axis tilt.
    pub fn evaluate_partial_axial_overlap_candidate(
        self,
        tool_pose: Pose,
        body: &RigidBody,
        config: GripperConfig,
        minimum_axial_overlap_m: f64,
    ) -> GraspCandidate {
        let body_in_tool = tool_pose.inverse() * body.pose;
        let local_aabb = body.shape.aabb(body_in_tool);
        let local_min = local_aabb.min;
        let local_max = local_aabb.max;
        let size = local_max - local_min;
        let center = (local_min + local_max) * 0.5;
        let depth = config.jaw_half_extents_m;
        let partial_metrics =
            capsule_partial_axial_metrics(body.shape, body_in_tool, -depth.z, depth.z);
        let axial_overlap_m = partial_metrics
            .map(|metrics| metrics.full_radius_axial_overlap_m)
            .unwrap_or(0.0);
        let valid_request = minimum_axial_overlap_m.is_finite()
            && minimum_axial_overlap_m >= MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M;
        let within_lateral_finger_depth = center.y.abs() + size.y * 0.5 <= depth.y;
        let within_finger_depth = within_lateral_finger_depth
            && partial_metrics.is_some()
            && valid_request
            && axial_overlap_m >= minimum_axial_overlap_m;
        let required_opening_m = partial_metrics
            .map(|metrics| metrics.conservative_required_opening_m)
            .unwrap_or(size.x);
        let center_error_m = Vec3::new(
            partial_metrics
                .map(|metrics| metrics.conservative_contact_center_x_m)
                .unwrap_or(center.x),
            center.y,
            center.z,
        );
        GraspCandidate {
            body_id: body.id,
            required_opening_m,
            center_error_m,
            jaw_clearance_m: self.opening_m - required_opening_m,
            within_finger_depth,
            axial_overlap_m,
            minimum_axial_overlap_m: Some(minimum_axial_overlap_m),
        }
    }

    /// Re-evaluate a held body using the acquisition mode retained by the
    /// gripper. This lets a partial-overlap grasp survive deterministic
    /// simulation refreshes without weakening the legacy candidate API.
    pub fn evaluate_held_candidate(
        self,
        tool_pose: Pose,
        body: &RigidBody,
        config: GripperConfig,
    ) -> GraspCandidate {
        if let Some(minimum_axial_overlap_m) = self.held_minimum_axial_overlap_m {
            self.evaluate_partial_axial_overlap_candidate(
                tool_pose,
                body,
                config,
                minimum_axial_overlap_m,
            )
        } else {
            self.evaluate_candidate(tool_pose, body, config)
        }
    }

    pub fn try_grasp(&mut self, candidate: GraspCandidate, config: GripperConfig) -> bool {
        if !candidate.is_reachable(config)
            // Positive clearance is still free space. Compliance bounds how
            // far the pads may compress; it must never create a grasp across
            // an air gap.
            || candidate.jaw_clearance_m > CONTACT_EPSILON_M
            || candidate.jaw_clearance_m < -config.pad_compliance_m * 2.0
            || candidate.center_error_m.x.abs() > config.pad_compliance_m
            || self.held_body.is_some()
        {
            return false;
        }
        self.held_body = Some(candidate.body_id);
        self.held_minimum_axial_overlap_m = candidate.minimum_axial_overlap_m;
        self.estimated_grip_force_n = grip_force(candidate.jaw_clearance_m, config);
        true
    }

    /// Refresh the reduced pad-compression force for an attached body.
    /// Returns `false` after the jaws have opened past geometric contact and
    /// the body has consequently been released.
    pub fn update_held_contact(
        &mut self,
        candidate: GraspCandidate,
        config: GripperConfig,
    ) -> bool {
        if self.held_body != Some(candidate.body_id) {
            return false;
        }
        if candidate.minimum_axial_overlap_m != self.held_minimum_axial_overlap_m
            || candidate.jaw_clearance_m > CONTACT_EPSILON_M
            || !candidate.within_finger_depth
        {
            self.release();
            return false;
        }
        self.estimated_grip_force_n = grip_force(candidate.jaw_clearance_m, config);
        true
    }

    pub fn release(&mut self) -> Option<BodyId> {
        self.estimated_grip_force_n = 0.0;
        self.held_minimum_axial_overlap_m = None;
        self.held_body.take()
    }
}

fn axial_overlap(body_min_m: f64, body_max_m: f64, pad_min_m: f64, pad_max_m: f64) -> f64 {
    (body_max_m.min(pad_max_m) - body_min_m.max(pad_min_m)).max(0.0)
}

fn full_cross_section_axial_overlap_m(
    shape: Shape,
    body_in_tool: Pose,
    pad_min_z_m: f64,
    pad_max_z_m: f64,
) -> f64 {
    match shape {
        Shape::Capsule {
            radius_m,
            half_segment_m,
        } => {
            let axis = body_in_tool
                .transform_vector(Vec3::Z)
                .normalized_or(Vec3::Z);
            // A tilted cylindrical cross-section projects into tool Z. Erode
            // the pad slab by that projection so every cross-section counted
            // by this overlap fits completely on the axial pad land.
            let radial_z_extent_m = radius_m * (1.0 - axis.z * axis.z).max(0.0).sqrt();
            let full_section_pad_min_z_m = pad_min_z_m + radial_z_extent_m;
            let full_section_pad_max_z_m = pad_max_z_m - radial_z_extent_m;
            if full_section_pad_min_z_m > full_section_pad_max_z_m {
                return 0.0;
            }
            let segment_offset_z_m = axis.z.abs() * half_segment_m;
            axial_overlap(
                body_in_tool.translation.z - segment_offset_z_m,
                body_in_tool.translation.z + segment_offset_z_m,
                full_section_pad_min_z_m,
                full_section_pad_max_z_m,
            )
        }
        Shape::Box { .. } | Shape::Gear(_) | Shape::Sphere { .. } => 0.0,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PartialAxialMetrics {
    full_radius_axial_overlap_m: f64,
    conservative_required_opening_m: f64,
    conservative_contact_center_x_m: f64,
}

fn capsule_partial_axial_metrics(
    shape: Shape,
    body_in_tool: Pose,
    pad_min_z_m: f64,
    pad_max_z_m: f64,
) -> Option<PartialAxialMetrics> {
    let Shape::Capsule {
        radius_m,
        half_segment_m,
    } = shape
    else {
        return None;
    };
    let axis = body_in_tool
        .transform_vector(Vec3::Z)
        .normalized_or(Vec3::Z);
    let full_radius_axial_overlap_m =
        full_cross_section_axial_overlap_m(shape, body_in_tool, pad_min_z_m, pad_max_z_m);

    // Model contact on the full-radius shaft land selected by the overlap
    // gate, not on rounded-cap material. A tilted radial cross-section can
    // enter the pad slab while its centerline is just outside it, so expand
    // the slab by the cross-section's tool-Z projection before bounding the X
    // drift. Expanding each X side by a full radius then over-bounds the exact
    // projected radius and keeps this opening estimate conservative.
    let radial_z_extent_m = radius_m * (1.0 - axis.z * axis.z).max(0.0).sqrt();
    let (parameter_min_m, parameter_max_m) = segment_parameter_range_in_z_slab(
        body_in_tool.translation.z,
        axis.z,
        half_segment_m,
        pad_min_z_m - radial_z_extent_m,
        pad_max_z_m + radial_z_extent_m,
    )?;
    let centerline_x_a_m = body_in_tool.translation.x + axis.x * parameter_min_m;
    let centerline_x_b_m = body_in_tool.translation.x + axis.x * parameter_max_m;
    let material_min_x_m = centerline_x_a_m.min(centerline_x_b_m) - radius_m;
    let material_max_x_m = centerline_x_a_m.max(centerline_x_b_m) + radius_m;
    Some(PartialAxialMetrics {
        full_radius_axial_overlap_m,
        conservative_required_opening_m: material_max_x_m - material_min_x_m,
        conservative_contact_center_x_m: 0.5 * (material_min_x_m + material_max_x_m),
    })
}

fn segment_parameter_range_in_z_slab(
    center_z_m: f64,
    axis_z: f64,
    half_segment_m: f64,
    slab_min_z_m: f64,
    slab_max_z_m: f64,
) -> Option<(f64, f64)> {
    if axis_z.abs() <= CONTACT_EPSILON_M {
        return (center_z_m >= slab_min_z_m && center_z_m <= slab_max_z_m)
            .then_some((-half_segment_m, half_segment_m));
    }
    let parameter_a_m = (slab_min_z_m - center_z_m) / axis_z;
    let parameter_b_m = (slab_max_z_m - center_z_m) / axis_z;
    let parameter_min_m = parameter_a_m.min(parameter_b_m).max(-half_segment_m);
    let parameter_max_m = parameter_a_m.max(parameter_b_m).min(half_segment_m);
    (parameter_max_m >= parameter_min_m).then_some((parameter_min_m, parameter_max_m))
}

fn grip_force(jaw_clearance_m: f64, config: GripperConfig) -> f64 {
    let compression_m = (-jaw_clearance_m).max(0.0);
    if config.pad_compliance_m > 0.0 {
        config.max_grip_force_n * (compression_m / (2.0 * config.pad_compliance_m)).clamp(0.0, 1.0)
    } else if compression_m > 0.0 {
        config.max_grip_force_n
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{MotionType, Shape};
    use crate::math::Quat;

    #[test]
    fn jaws_move_symmetrically() {
        let config = GripperConfig::default();
        let state = GripperState::new(1.0e-3, config);
        let poses = state.jaw_poses(Pose::IDENTITY, config);
        assert!((poses[0].translation.x + poses[1].translation.x).abs() < 1e-15);
    }

    #[test]
    fn stop_holds_current_opening_and_preserves_grasp_state() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(config.max_opening_m, config);
        state.set_command(config.min_opening_m, config);
        state.step(0.1, config);
        state.held_body = Some(BodyId(9));
        let opening = state.opening_m;
        state.stop();
        state.step(0.1, config);
        assert_eq!(state.opening_m, opening);
        assert_eq!(state.command_opening_m, opening);
        assert_eq!(state.opening_velocity_m_s, 0.0);
        assert_eq!(state.held_body, Some(BodyId(9)));
    }

    #[test]
    fn baseline_opening_and_force_limit_match_cad() {
        let config = GripperConfig::default();
        assert_eq!(config.max_opening_m, 2.8e-3);
        assert_eq!(config.max_grip_force_n, 0.15);
        assert!(config.max_grip_force_n <= 0.15);
    }

    #[test]
    fn centered_sphere_is_a_grasp_candidate() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.4e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(candidate.is_reachable(config));
        assert!(state.try_grasp(candidate, config));
        assert_eq!(state.held_body, Some(BodyId(7)));
        assert_eq!(state.release(), Some(BodyId(7)));
    }

    #[test]
    fn positive_jaw_clearance_cannot_grasp_across_air() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.41e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(candidate.jaw_clearance_m > 0.0);
        assert!(!state.try_grasp(candidate, config));
        assert_eq!(state.held_body, None);
    }

    #[test]
    fn off_center_body_cannot_claim_symmetric_pad_contact() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.39e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::from_translation(Vec3::X * 30.0e-6),
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(candidate.center_error_m.x.abs() > config.pad_compliance_m);
        assert!(!state.try_grasp(candidate, config));
    }

    #[test]
    fn opening_past_contact_releases_a_held_body() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.39e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Sphere { radius_m: 0.2e-3 },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(state.try_grasp(candidate, config));
        assert!(state.estimated_grip_force_n > 0.0);

        state.opening_m = 0.41e-3;
        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(!state.update_held_contact(candidate, config));
        assert_eq!(state.held_body, None);
        assert_eq!(state.estimated_grip_force_n, 0.0);
    }

    #[test]
    fn far_object_is_rejected_by_finger_depth() {
        let config = GripperConfig::default();
        let state = GripperState::new(0.4e-3, config);
        let body = RigidBody::new(
            BodyId(1),
            Shape::Sphere { radius_m: 0.1e-3 },
            Pose::from_translation(Vec3::new(0.0, 0.0, 2.0e-3)),
            MotionType::Dynamic,
        );
        assert!(
            !state
                .evaluate_candidate(Pose::IDENTITY, &body, config)
                .within_finger_depth
        );
    }

    #[test]
    fn lateral_containment_uses_physical_jaw_half_span() {
        let config = GripperConfig::default();
        let state = GripperState::new(0.2e-3, config);
        let sphere_at = |id, center_y_m| {
            RigidBody::new(
                BodyId(id),
                Shape::Sphere { radius_m: 0.1e-3 },
                Pose::from_translation(Vec3::Y * center_y_m),
                MotionType::Dynamic,
            )
        };

        let inside = sphere_at(1, 149.0e-6);
        let outside = sphere_at(2, 151.0e-6);
        assert!(
            state
                .evaluate_candidate(Pose::IDENTITY, &inside, config)
                .within_finger_depth
        );
        assert!(
            !state
                .evaluate_candidate(Pose::IDENTITY, &outside, config)
                .within_finger_depth
        );

        let partial_capsule_at = |id, center_y_m| {
            RigidBody::new(
                BodyId(id),
                Shape::Capsule {
                    radius_m: 0.2e-3,
                    half_segment_m: 0.7e-3,
                },
                Pose::from_translation(Vec3::new(0.0, center_y_m, 0.75e-3)),
                MotionType::Dynamic,
            )
        };
        let partial_inside = state.evaluate_partial_axial_overlap_candidate(
            Pose::IDENTITY,
            &partial_capsule_at(3, 49.0e-6),
            config,
            MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
        );
        let partial_outside = state.evaluate_partial_axial_overlap_candidate(
            Pose::IDENTITY,
            &partial_capsule_at(4, 51.0e-6),
            config,
            MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
        );
        assert!(partial_inside.within_finger_depth);
        assert!(!partial_outside.within_finger_depth);
    }

    #[test]
    fn explicit_partial_overlap_accepts_axially_offset_capsule() {
        let config = GripperConfig {
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 200.0e-6),
            ..GripperConfig::default()
        };
        let mut state = GripperState::new(0.39e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Capsule {
                radius_m: 0.2e-3,
                half_segment_m: 0.7e-3,
            },
            Pose::from_translation(Vec3::Z * 0.75e-3),
            MotionType::Dynamic,
        );

        let legacy = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(!legacy.within_finger_depth);
        assert!(!state.try_grasp(legacy, config));

        let partial = state.evaluate_partial_axial_overlap_candidate(
            Pose::IDENTITY,
            &body,
            config,
            MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
        );
        assert!((partial.axial_overlap_m - 0.15e-3).abs() < 1.0e-15);
        assert!((partial.required_opening_m - 0.4e-3).abs() < 1.0e-15);
        assert!(partial.within_finger_depth);
        assert!(state.try_grasp(partial, config));
        assert_eq!(
            state.held_minimum_axial_overlap_m,
            Some(MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M)
        );
    }

    #[test]
    fn partial_overlap_rejects_insufficient_axial_material() {
        let config = GripperConfig {
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 200.0e-6),
            ..GripperConfig::default()
        };
        let mut state = GripperState::new(0.31e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Capsule {
                radius_m: 0.2e-3,
                half_segment_m: 0.7e-3,
            },
            Pose::from_translation(Vec3::Z * 0.82e-3),
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_partial_axial_overlap_candidate(
            Pose::IDENTITY,
            &body,
            config,
            MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
        );

        assert!((candidate.axial_overlap_m - 80.0e-6).abs() < 1.0e-15);
        assert!(!candidate.within_finger_depth);
        assert!(!state.try_grasp(candidate, config));
    }

    #[test]
    fn cap_only_overlap_cannot_satisfy_partial_shaft_gate() {
        let config = GripperConfig {
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 200.0e-6),
            ..GripperConfig::default()
        };
        let mut state = GripperState::new(0.39e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Capsule {
                radius_m: 0.2e-3,
                half_segment_m: 0.7e-3,
            },
            Pose::from_translation(Vec3::Z * 0.99e-3),
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_partial_axial_overlap_candidate(
            Pose::IDENTITY,
            &body,
            config,
            MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
        );
        assert_eq!(candidate.axial_overlap_m, 0.0);
        assert!(!candidate.within_finger_depth);
        assert!(!candidate.is_reachable(config));
        assert!(!state.try_grasp(candidate, config));
        assert_eq!(state.held_body, None);
    }

    #[test]
    fn partial_capsule_bound_includes_maximum_axis_tilt() {
        let config = GripperConfig {
            jaw_half_extents_m: Vec3::new(100.0e-6, 250.0e-6, 200.0e-6),
            ..GripperConfig::default()
        };
        for tilt_rad in [0.0, 0.015] {
            let mut state = GripperState::new(0.38e-3, config);
            let body = RigidBody::new(
                BodyId(7),
                Shape::Capsule {
                    radius_m: 0.2e-3,
                    half_segment_m: 0.7e-3,
                },
                Pose::new(Vec3::Z * 0.75e-3, Quat::from_axis_angle(Vec3::Y, tilt_rad)),
                MotionType::Dynamic,
            );
            let candidate = state.evaluate_partial_axial_overlap_candidate(
                Pose::IDENTITY,
                &body,
                config,
                MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M,
            );

            let axis_x = tilt_rad.sin();
            let axis_z = tilt_rad.cos();
            let radial_z_extent_m = 0.2e-3 * (1.0 - axis_z * axis_z).max(0.0).sqrt();
            let expected_overlap_m = 0.2e-3 - radial_z_extent_m - (0.75e-3 - axis_z * 0.7e-3);
            let parameter_min_m = -0.7e-3;
            let parameter_max_m = (0.2e-3 + radial_z_extent_m - 0.75e-3) / axis_z;
            let expected_opening_m = 0.4e-3 + axis_x.abs() * (parameter_max_m - parameter_min_m);
            let expected_center_x_m = 0.5 * axis_x * (parameter_min_m + parameter_max_m);

            assert!((candidate.axial_overlap_m - expected_overlap_m).abs() < 1.0e-15);
            assert!((candidate.required_opening_m - expected_opening_m).abs() < 1.0e-15);
            assert!((candidate.center_error_m.x - expected_center_x_m).abs() < 1.0e-15);
            assert!(candidate.center_error_m.x.abs() <= config.pad_compliance_m);
            let compression_m = -candidate.jaw_clearance_m;
            assert!(compression_m <= 2.0 * config.pad_compliance_m);
            assert!(0.5 * compression_m > candidate.center_error_m.x.abs());
            assert!(state.try_grasp(candidate, config));
        }
    }

    #[test]
    fn partial_overlap_request_below_mechanics_floor_fails_closed() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.39e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Capsule {
                radius_m: 0.2e-3,
                half_segment_m: 0.7e-3,
            },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );
        let candidate = state.evaluate_partial_axial_overlap_candidate(
            Pose::IDENTITY,
            &body,
            config,
            MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M * 0.5,
        );

        assert!(!candidate.within_finger_depth);
        assert!(!candidate.is_reachable(config));
        assert!(!state.try_grasp(candidate, config));
    }

    #[test]
    fn legacy_m1c_capsule_still_requires_and_accepts_full_containment() {
        let config = GripperConfig::default();
        let mut state = GripperState::new(0.388e-3, config);
        let body = RigidBody::new(
            BodyId(7),
            Shape::Capsule {
                radius_m: 0.2e-3,
                // M1c's calibration peg cylindrical half-length.
                half_segment_m: 0.35e-3,
            },
            Pose::IDENTITY,
            MotionType::Dynamic,
        );

        let candidate = state.evaluate_candidate(Pose::IDENTITY, &body, config);
        assert!(candidate.within_finger_depth);
        assert!((candidate.required_opening_m - 0.4e-3).abs() < 1.0e-15);
        assert_eq!(candidate.minimum_axial_overlap_m, None);
        assert!((candidate.axial_overlap_m - 0.7e-3).abs() < 1.0e-15);
        assert!(state.try_grasp(candidate, config));
        assert_eq!(state.held_minimum_axial_overlap_m, None);
    }
}
