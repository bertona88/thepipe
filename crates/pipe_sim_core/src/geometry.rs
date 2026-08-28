//! Rigid-body and collision geometry definitions.

use crate::math::{Pose, Vec3, EPSILON};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId(pub u32);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub const fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    pub fn from_center_half_extents(center: Vec3, half_extents: Vec3) -> Self {
        let extents = half_extents.abs();
        Self::new(center - extents, center + extents)
    }

    pub fn union(self, rhs: Self) -> Self {
        Self::new(self.min.min(rhs.min), self.max.max(rhs.max))
    }

    pub fn expanded(self, amount_m: f64) -> Self {
        let expansion = Vec3::splat(amount_m.max(0.0));
        Self::new(self.min - expansion, self.max + expansion)
    }

    pub fn overlaps(self, rhs: Self) -> bool {
        self.min.x <= rhs.max.x
            && self.max.x >= rhs.min.x
            && self.min.y <= rhs.max.y
            && self.max.y >= rhs.min.y
            && self.min.z <= rhs.max.z
            && self.max.z >= rhs.min.z
    }

    pub fn contains(self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    pub fn center(self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    pub fn half_extents(self) -> Vec3 {
        (self.max - self.min) * 0.5
    }

    /// Unsigned Euclidean gap between boxes (zero when overlapping).
    pub fn distance(self, rhs: Self) -> f64 {
        let dx = (rhs.min.x - self.max.x)
            .max(self.min.x - rhs.max.x)
            .max(0.0);
        let dy = (rhs.min.y - self.max.y)
            .max(self.min.y - rhs.max.y)
            .max(0.0);
        let dz = (rhs.min.z - self.max.z)
            .max(self.min.z - rhs.max.z)
            .max(0.0);
        Vec3::new(dx, dy, dz).length()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GearGeometry {
    pub teeth: u16,
    pub module_m: f64,
    pub pressure_angle_rad: f64,
    pub pitch_radius_m: f64,
    pub root_radius_m: f64,
    pub tip_radius_m: f64,
    /// Half of the axial toothed-face width.
    pub half_thickness_m: f64,
    pub bore_radius_m: f64,
    /// Radius of the coaxial hub which spans `half_total_height_m`.
    pub hub_radius_m: f64,
    /// Half of the complete hub/part height.
    pub half_total_height_m: f64,
    /// Tooth-band center relative to the overall part center along local Z.
    pub tooth_center_offset_m: f64,
}

impl GearGeometry {
    /// Standard full-depth external spur gear with a full-height coaxial hub.
    ///
    /// Local Z=0 is the center of the total part. The toothed face is aligned
    /// with the negative-Z end, matching the build123d gearbox convention.
    #[allow(clippy::too_many_arguments)]
    pub fn spur_with_hub(
        teeth: u16,
        module_m: f64,
        pressure_angle_rad: f64,
        face_width_m: f64,
        bore_radius_m: f64,
        hub_radius_m: f64,
        total_height_m: f64,
    ) -> Self {
        let pitch_radius_m = teeth as f64 * module_m * 0.5;
        Self {
            teeth,
            module_m,
            pressure_angle_rad,
            pitch_radius_m,
            root_radius_m: (pitch_radius_m - 1.25 * module_m).max(bore_radius_m),
            tip_radius_m: pitch_radius_m + module_m,
            half_thickness_m: face_width_m * 0.5,
            bore_radius_m,
            hub_radius_m,
            half_total_height_m: total_height_m * 0.5,
            tooth_center_offset_m: -0.5 * (total_height_m - face_width_m),
        }
    }

    /// Uniform-height spur envelope with an explicit pressure angle.
    pub fn uniform_spur(
        teeth: u16,
        module_m: f64,
        pressure_angle_rad: f64,
        thickness_m: f64,
        bore_radius_m: f64,
    ) -> Self {
        let tip_radius_m = teeth as f64 * module_m * 0.5 + module_m;
        Self::spur_with_hub(
            teeth,
            module_m,
            pressure_angle_rad,
            thickness_m,
            bore_radius_m,
            tip_radius_m,
            thickness_m,
        )
    }

    /// Legacy 20-degree, uniform-height approximation. New baseline code
    /// should use [`Self::spur_with_hub`] with an explicit pressure angle.
    pub fn uniform_spur_20deg(
        teeth: u16,
        module_m: f64,
        thickness_m: f64,
        bore_radius_m: f64,
    ) -> Self {
        Self::uniform_spur(
            teeth,
            module_m,
            20.0_f64.to_radians(),
            thickness_m,
            bore_radius_m,
        )
    }

    pub fn is_valid(self) -> bool {
        let scalars = [
            self.module_m,
            self.pressure_angle_rad,
            self.pitch_radius_m,
            self.root_radius_m,
            self.tip_radius_m,
            self.half_thickness_m,
            self.bore_radius_m,
            self.hub_radius_m,
            self.half_total_height_m,
            self.tooth_center_offset_m,
        ];
        self.teeth >= 4
            && scalars.iter().all(|value| value.is_finite())
            && self.module_m > 0.0
            && self.pressure_angle_rad > 0.0
            && self.pressure_angle_rad < core::f64::consts::FRAC_PI_2
            && self.pitch_radius_m > 0.0
            && self.root_radius_m >= self.bore_radius_m
            && self.pitch_radius_m >= self.root_radius_m
            && self.tip_radius_m >= self.pitch_radius_m
            && self.half_thickness_m > 0.0
            && self.bore_radius_m >= 0.0
            && self.hub_radius_m >= self.bore_radius_m
            && self.hub_radius_m <= self.tip_radius_m
            && self.half_total_height_m >= self.half_thickness_m
            && self.tooth_center_offset_m.abs() + self.half_thickness_m
                <= self.half_total_height_m + EPSILON
    }

    pub fn circular_pitch_m(self) -> f64 {
        core::f64::consts::PI * self.module_m
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Sphere {
        radius_m: f64,
    },
    /// Local line segment runs from `-Z * half_segment_m` to
    /// `+Z * half_segment_m`, swept by the radius.
    Capsule {
        radius_m: f64,
        half_segment_m: f64,
    },
    Box {
        half_extents_m: Vec3,
    },
    /// Spur gear with local axis along +Z. Narrow phase uses a deterministic
    /// involute-envelope approximation and reports its approximation kind.
    Gear(GearGeometry),
}

impl Shape {
    pub fn is_valid(self) -> bool {
        match self {
            Self::Sphere { radius_m } => radius_m > 0.0 && radius_m.is_finite(),
            Self::Capsule {
                radius_m,
                half_segment_m,
            } => {
                radius_m > 0.0
                    && half_segment_m >= 0.0
                    && radius_m.is_finite()
                    && half_segment_m.is_finite()
            }
            Self::Box { half_extents_m } => {
                half_extents_m.is_finite()
                    && half_extents_m.x > 0.0
                    && half_extents_m.y > 0.0
                    && half_extents_m.z > 0.0
            }
            Self::Gear(gear) => gear.is_valid(),
        }
    }

    pub fn local_bounding_radius_m(self) -> f64 {
        match self {
            Self::Sphere { radius_m } => radius_m,
            Self::Capsule {
                radius_m,
                half_segment_m,
            } => radius_m + half_segment_m,
            Self::Box { half_extents_m } => half_extents_m.length(),
            Self::Gear(gear) => {
                let tooth_corner = (gear.tip_radius_m.powi(2)
                    + (gear.tooth_center_offset_m.abs() + gear.half_thickness_m).powi(2))
                .sqrt();
                let hub_corner =
                    (gear.hub_radius_m.powi(2) + gear.half_total_height_m.powi(2)).sqrt();
                tooth_corner.max(hub_corner)
            }
        }
    }

    pub fn volume_m3(self) -> f64 {
        match self {
            Self::Sphere { radius_m } => 4.0 / 3.0 * core::f64::consts::PI * radius_m.powi(3),
            Self::Capsule {
                radius_m,
                half_segment_m,
            } => {
                core::f64::consts::PI * radius_m.powi(2) * (2.0 * half_segment_m)
                    + 4.0 / 3.0 * core::f64::consts::PI * radius_m.powi(3)
            }
            Self::Box { half_extents_m } => {
                8.0 * half_extents_m.x * half_extents_m.y * half_extents_m.z
            }
            Self::Gear(gear) => {
                // Annular full-height hub plus the tooth envelope extending
                // radially beyond the hub over only the toothed face width.
                let hub_volume = core::f64::consts::PI
                    * (gear.hub_radius_m.powi(2) - gear.bore_radius_m.powi(2))
                    * (2.0 * gear.half_total_height_m);
                let tooth_extension = core::f64::consts::PI
                    * (gear.tip_radius_m.powi(2) - gear.hub_radius_m.powi(2))
                    * (2.0 * gear.half_thickness_m);
                hub_volume + tooth_extension
            }
        }
    }

    pub fn aabb(self, pose: Pose) -> Aabb {
        match self {
            Self::Sphere { radius_m } => {
                Aabb::from_center_half_extents(pose.translation, Vec3::splat(radius_m))
            }
            Self::Capsule {
                radius_m,
                half_segment_m,
            } => {
                let axis = pose.transform_vector(Vec3::Z);
                let a = pose.translation - axis * half_segment_m;
                let b = pose.translation + axis * half_segment_m;
                Aabb::new(
                    a.min(b) - Vec3::splat(radius_m),
                    a.max(b) + Vec3::splat(radius_m),
                )
            }
            Self::Box { half_extents_m } => {
                let absolute_rotation = pose.rotation.to_mat3().abs();
                Aabb::from_center_half_extents(
                    pose.translation,
                    absolute_rotation * half_extents_m + Vec3::splat(EPSILON),
                )
            }
            Self::Gear(gear) => {
                // Union the full-height hub cylinder with the offset, thin
                // toothed cylinder. This avoids falsely extending tooth tips
                // through the complete 1.30 mm baseline part height.
                let axis = pose.transform_vector(Vec3::Z).normalized_or(Vec3::Z);
                let cylinder_aabb = |radius_m: f64, half_height_m: f64, center_z_m: f64| {
                    let projected = |component: f64| {
                        radius_m * (1.0 - component * component).max(0.0).sqrt()
                            + half_height_m * component.abs()
                    };
                    Aabb::from_center_half_extents(
                        pose.transform_point(Vec3::Z * center_z_m),
                        Vec3::new(projected(axis.x), projected(axis.y), projected(axis.z)),
                    )
                };
                cylinder_aabb(gear.hub_radius_m, gear.half_total_height_m, 0.0).union(
                    cylinder_aabb(
                        gear.tip_radius_m,
                        gear.half_thickness_m,
                        gear.tooth_center_offset_m,
                    ),
                )
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub density_kg_m3: f64,
    pub friction: f64,
    pub restitution: f64,
}

impl Default for Material {
    fn default() -> Self {
        Self {
            // Generic photopolymer resin.
            density_kg_m3: 1_150.0,
            friction: 0.35,
            restitution: 0.05,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionType {
    Static,
    Kinematic,
    Dynamic,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CollisionFilter {
    pub group: u32,
    pub mask: u32,
}

impl CollisionFilter {
    pub const ALL: Self = Self {
        group: 1,
        mask: u32::MAX,
    };

    pub fn allows(self, rhs: Self) -> bool {
        (self.mask & rhs.group) != 0 && (rhs.mask & self.group) != 0
    }
}

impl Default for CollisionFilter {
    fn default() -> Self {
        Self::ALL
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct RigidBody {
    pub id: BodyId,
    pub pose: Pose,
    pub shape: Shape,
    pub motion: MotionType,
    pub material: Material,
    pub mass_kg: f64,
    pub linear_velocity_m_s: Vec3,
    pub angular_velocity_rad_s: Vec3,
    pub accumulated_force_n: Vec3,
    pub accumulated_torque_nm: Vec3,
    pub collision_filter: CollisionFilter,
    pub enabled: bool,
    /// Opaque value reserved for the host application.
    pub user_tag: u32,
}

impl RigidBody {
    pub fn new(id: BodyId, shape: Shape, pose: Pose, motion: MotionType) -> Self {
        let material = Material::default();
        let mass_kg = if motion == MotionType::Dynamic {
            (shape.volume_m3() * material.density_kg_m3).max(EPSILON)
        } else {
            f64::INFINITY
        };
        Self {
            id,
            pose,
            shape,
            motion,
            material,
            mass_kg,
            linear_velocity_m_s: Vec3::ZERO,
            angular_velocity_rad_s: Vec3::ZERO,
            accumulated_force_n: Vec3::ZERO,
            accumulated_torque_nm: Vec3::ZERO,
            collision_filter: CollisionFilter::default(),
            enabled: true,
            user_tag: 0,
        }
    }

    pub fn with_material(mut self, material: Material) -> Self {
        self.material = material;
        if self.motion == MotionType::Dynamic {
            self.mass_kg = (self.shape.volume_m3() * material.density_kg_m3).max(EPSILON);
        }
        self
    }

    pub fn with_mass_kg(mut self, mass_kg: f64) -> Self {
        if self.motion == MotionType::Dynamic && mass_kg.is_finite() && mass_kg > 0.0 {
            self.mass_kg = mass_kg;
        }
        self
    }

    pub fn inverse_mass(&self) -> f64 {
        if self.motion == MotionType::Dynamic && self.mass_kg.is_finite() && self.mass_kg > EPSILON
        {
            1.0 / self.mass_kg
        } else {
            0.0
        }
    }

    pub fn aabb(&self) -> Aabb {
        self.shape.aabb(self.pose)
    }

    pub fn apply_force(&mut self, force_n: Vec3) {
        if self.motion == MotionType::Dynamic {
            self.accumulated_force_n += force_n;
        }
    }

    pub fn apply_torque(&mut self, torque_nm: Vec3) {
        if self.motion == MotionType::Dynamic {
            self.accumulated_torque_nm += torque_nm;
        }
    }

    pub fn clear_forces(&mut self) {
        self.accumulated_force_n = Vec3::ZERO;
        self.accumulated_torque_nm = Vec3::ZERO;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quat;

    #[test]
    fn rotated_box_aabb_contains_all_corners() {
        let shape = Shape::Box {
            half_extents_m: Vec3::new(1.0, 2.0, 0.5),
        };
        let pose = Pose::new(
            Vec3::new(3.0, -2.0, 1.0),
            Quat::from_axis_angle(Vec3::Z, 0.7),
        );
        let aabb = shape.aabb(pose);
        for x in [-1.0, 1.0] {
            for y in [-2.0, 2.0] {
                for z in [-0.5, 0.5] {
                    assert!(aabb.contains(pose.transform_point(Vec3::new(x, y, z))));
                }
            }
        }
    }

    #[test]
    fn spur_gear_uses_standard_pitch_dimensions() {
        let gear = GearGeometry::spur_with_hub(
            20,
            100.0e-6,
            25.0_f64.to_radians(),
            350.0e-6,
            210.0e-6,
            275.0e-6,
            1.30e-3,
        );
        assert!((gear.pitch_radius_m - 1.0e-3).abs() < 1e-15);
        assert!((gear.tip_radius_m - 1.1e-3).abs() < 1e-15);
        assert!((gear.pressure_angle_rad.to_degrees() - 25.0).abs() < 1e-12);
        assert!((2.0 * gear.half_thickness_m - 0.35e-3).abs() < 1e-15);
        assert!((2.0 * gear.half_total_height_m - 1.30e-3).abs() < 1e-15);
        assert!((gear.tooth_center_offset_m + 0.475e-3).abs() < 1e-15);
        assert_eq!(gear.hub_radius_m, 0.275e-3);
        assert_eq!(gear.bore_radius_m, 0.210e-3);
        assert!(gear.is_valid());
    }

    #[test]
    fn stepped_gear_aabb_covers_total_hub_and_offset_tooth_face() {
        let gear = GearGeometry::spur_with_hub(
            12,
            0.1e-3,
            25.0_f64.to_radians(),
            0.35e-3,
            0.21e-3,
            0.275e-3,
            1.30e-3,
        );
        let aabb = Shape::Gear(gear).aabb(Pose::IDENTITY);
        assert!((aabb.min.z + 0.65e-3).abs() < 1e-15);
        assert!((aabb.max.z - 0.65e-3).abs() < 1e-15);
        assert!((aabb.max.x - 0.70e-3).abs() < 1e-15);
    }

    #[test]
    fn legacy_uniform_helper_is_explicitly_twenty_degrees() {
        let gear = GearGeometry::uniform_spur_20deg(20, 0.1e-3, 0.5e-3, 0.15e-3);
        assert!((gear.pressure_angle_rad.to_degrees() - 20.0).abs() < 1e-12);
        assert_eq!(gear.half_thickness_m, gear.half_total_height_m);
        assert_eq!(gear.tooth_center_offset_m, 0.0);
    }

    #[test]
    fn collision_filters_require_both_masks() {
        let a = CollisionFilter {
            group: 0b0010,
            mask: 0b0100,
        };
        let b = CollisionFilter {
            group: 0b0100,
            mask: 0b0010,
        };
        assert!(a.allows(b));
        assert!(!a.allows(CollisionFilter {
            group: 0b1000,
            mask: 0b0010
        }));
    }
}
