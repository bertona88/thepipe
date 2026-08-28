use crate::math::{Mat3, Ray, RigidTransform, Vec2, Vec3};
use crate::noise::{keyed_seed, DeterministicRng};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImageSize {
    pub width: u32,
    pub height: u32,
}

impl ImageSize {
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }

    pub fn contains(self, pixel: Vec2) -> bool {
        pixel.is_finite()
            && pixel.x >= -0.5
            && pixel.y >= -0.5
            && pixel.x < self.width as f64 - 0.5
            && pixel.y < self.height as f64 - 0.5
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraIntrinsics {
    pub fx_px: f64,
    pub fy_px: f64,
    pub cx_px: f64,
    pub cy_px: f64,
    /// Pixel-axis skew. Commodity sensors are normally close to zero.
    pub skew_px: f64,
}

impl CameraIntrinsics {
    pub const fn new(fx_px: f64, fy_px: f64, cx_px: f64, cy_px: f64) -> Self {
        Self {
            fx_px,
            fy_px,
            cx_px,
            cy_px,
            skew_px: 0.0,
        }
    }

    pub fn is_valid(self) -> bool {
        self.fx_px.is_finite()
            && self.fy_px.is_finite()
            && self.fx_px > 0.0
            && self.fy_px > 0.0
            && self.cx_px.is_finite()
            && self.cy_px.is_finite()
            && self.skew_px.is_finite()
    }
}

/// OpenCV-compatible Brown-Conrady radial/tangential lens distortion.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BrownConrady {
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub p1: f64,
    pub p2: f64,
}

impl BrownConrady {
    pub const NONE: Self = Self {
        k1: 0.0,
        k2: 0.0,
        k3: 0.0,
        p1: 0.0,
        p2: 0.0,
    };

    pub fn distort(self, undistorted: Vec2) -> Vec2 {
        let x = undistorted.x;
        let y = undistorted.y;
        let r2 = x * x + y * y;
        let radial = 1.0 + r2 * (self.k1 + r2 * (self.k2 + r2 * self.k3));
        let xy2 = 2.0 * x * y;
        Vec2::new(
            x * radial + self.p1 * xy2 + self.p2 * (r2 + 2.0 * x * x),
            y * radial + self.p1 * (r2 + 2.0 * y * y) + self.p2 * xy2,
        )
    }

    /// Iteratively invert distortion. Returns `None` for non-finite/divergent input.
    pub fn undistort(self, distorted: Vec2) -> Option<Vec2> {
        if !distorted.is_finite() {
            return None;
        }
        let mut estimate = distorted;
        for _ in 0..12 {
            let x = estimate.x;
            let y = estimate.y;
            let r2 = x * x + y * y;
            let radial = 1.0 + r2 * (self.k1 + r2 * (self.k2 + r2 * self.k3));
            if radial.abs() < 1.0e-12 || !radial.is_finite() {
                return None;
            }
            let tangential = Vec2::new(
                2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x),
                self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y,
            );
            let next = (distorted - tangential) / radial;
            if !next.is_finite() || next.norm_squared() > 1.0e8 {
                return None;
            }
            estimate = next;
        }
        Some(estimate)
    }

    fn plus(self, rhs: Self) -> Self {
        Self {
            k1: self.k1 + rhs.k1,
            k2: self.k2 + rhs.k2,
            k3: self.k3 + rhs.k3,
            p1: self.p1 + rhs.p1,
            p2: self.p2 + rhs.p2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProjectedPoint {
    pub pixel: Vec2,
    /// Positive optical-axis distance in camera coordinates, not Euclidean range.
    pub optical_depth_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinholeCamera {
    pub image_size: ImageSize,
    pub intrinsics: CameraIntrinsics,
    pub distortion: BrownConrady,
    /// Pose taking camera coordinates (+Z forward, +X right, +Y down) to world.
    pub world_from_camera: RigidTransform,
    pub near_m: f64,
    pub far_m: f64,
}

impl PinholeCamera {
    pub fn new(
        image_size: ImageSize,
        intrinsics: CameraIntrinsics,
        distortion: BrownConrady,
        world_from_camera: RigidTransform,
    ) -> Self {
        Self {
            image_size,
            intrinsics,
            distortion,
            world_from_camera,
            near_m: 0.001,
            far_m: 10.0,
        }
    }

    pub fn is_valid(self) -> bool {
        self.image_size.width > 0
            && self.image_size.height > 0
            && self.intrinsics.is_valid()
            && self.near_m >= 0.0
            && self.far_m > self.near_m
    }

    pub fn center_world(self) -> Vec3 {
        self.world_from_camera.translation
    }

    pub fn project(self, world: Vec3) -> Option<ProjectedPoint> {
        if !self.is_valid() {
            return None;
        }
        let camera = self.world_from_camera.inverse().transform_point(world);
        if camera.z <= self.near_m || camera.z > self.far_m || !camera.is_finite() {
            return None;
        }
        let distorted = self
            .distortion
            .distort(Vec2::new(camera.x / camera.z, camera.y / camera.z));
        let pixel = Vec2::new(
            self.intrinsics.fx_px * distorted.x
                + self.intrinsics.skew_px * distorted.y
                + self.intrinsics.cx_px,
            self.intrinsics.fy_px * distorted.y + self.intrinsics.cy_px,
        );
        self.image_size.contains(pixel).then_some(ProjectedPoint {
            pixel,
            optical_depth_m: camera.z,
        })
    }

    pub fn ray(self, pixel: Vec2) -> Option<Ray> {
        if !self.is_valid() || !self.image_size.contains(pixel) {
            return None;
        }
        let yd = (pixel.y - self.intrinsics.cy_px) / self.intrinsics.fy_px;
        let xd = (pixel.x - self.intrinsics.cx_px - self.intrinsics.skew_px * yd)
            / self.intrinsics.fx_px;
        let undistorted = self.distortion.undistort(Vec2::new(xd, yd))?;
        let local_direction = Vec3::new(undistorted.x, undistorted.y, 1.0);
        Ray::new(
            self.center_world(),
            self.world_from_camera.transform_vector(local_direction),
        )
    }
}

/// Difference between calibration values used by reconstruction and physical optics.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CalibrationDrift {
    /// Sensor motion in its nominal local axes.
    pub translation_m: Vec3,
    /// Axis-angle sensor rotation in nominal local axes.
    pub rotation_vector_rad: Vec3,
    pub focal_scale: Vec2,
    pub principal_shift_px: Vec2,
    pub distortion_delta: BrownConrady,
}

impl CalibrationDrift {
    pub const ZERO: Self = Self {
        translation_m: Vec3::ZERO,
        rotation_vector_rad: Vec3::ZERO,
        focal_scale: Vec2::ZERO,
        principal_shift_px: Vec2::ZERO,
        distortion_delta: BrownConrady::NONE,
    };

    pub fn apply(self, nominal: PinholeCamera) -> PinholeCamera {
        let local_delta = RigidTransform::new(
            Mat3::from_axis_angle(self.rotation_vector_rad),
            self.translation_m,
        );
        let mut actual = nominal;
        actual.world_from_camera = nominal.world_from_camera.compose(local_delta);
        actual.intrinsics.fx_px *= 1.0 + self.focal_scale.x;
        actual.intrinsics.fy_px *= 1.0 + self.focal_scale.y;
        actual.intrinsics.cx_px += self.principal_shift_px.x;
        actual.intrinsics.cy_px += self.principal_shift_px.y;
        actual.distortion = nominal.distortion.plus(self.distortion_delta);
        actual
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalibratedCamera {
    pub id: u32,
    /// Calibration used for ray reconstruction.
    pub nominal: PinholeCamera,
    /// Difference of the physical sensor from `nominal`.
    pub drift: CalibrationDrift,
}

impl CalibratedCamera {
    pub const fn new(id: u32, nominal: PinholeCamera) -> Self {
        Self {
            id,
            nominal,
            drift: CalibrationDrift::ZERO,
        }
    }

    pub fn actual(self) -> PinholeCamera {
        self.drift.apply(self.nominal)
    }
}

/// One-sigma, per-frame calibration drift random walk.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriftRandomWalk {
    pub translation_sigma_m: Vec3,
    pub rotation_sigma_rad: Vec3,
    pub focal_sigma_fraction: Vec2,
    pub principal_sigma_px: Vec2,
    pub distortion_sigma: BrownConrady,
    /// Fraction of existing drift retained each step. 1.0 is a pure random walk.
    pub retention: f64,
}

impl Default for DriftRandomWalk {
    fn default() -> Self {
        Self {
            translation_sigma_m: Vec3::splat(0.2e-6),
            rotation_sigma_rad: Vec3::splat(2.0e-6),
            focal_sigma_fraction: Vec2::new(0.2e-6, 0.2e-6),
            principal_sigma_px: Vec2::new(0.001, 0.001),
            distortion_sigma: BrownConrady {
                k1: 1.0e-7,
                k2: 1.0e-8,
                k3: 1.0e-9,
                p1: 1.0e-8,
                p2: 1.0e-8,
            },
            retention: 0.999,
        }
    }
}

impl DriftRandomWalk {
    /// Advance drift deterministically for a sensor and frame number.
    pub fn advance(
        self,
        drift: &mut CalibrationDrift,
        seed: u64,
        sensor_id: u32,
        frame_index: u64,
    ) {
        let mut rng = DeterministicRng::new(keyed_seed(
            seed,
            &[0xd71f_7a1b, sensor_id as u64, frame_index],
        ));
        let retain = self.retention.clamp(0.0, 1.0);
        drift.translation_m = drift.translation_m * retain
            + Vec3::new(
                rng.normal() * self.translation_sigma_m.x,
                rng.normal() * self.translation_sigma_m.y,
                rng.normal() * self.translation_sigma_m.z,
            );
        drift.rotation_vector_rad = drift.rotation_vector_rad * retain
            + Vec3::new(
                rng.normal() * self.rotation_sigma_rad.x,
                rng.normal() * self.rotation_sigma_rad.y,
                rng.normal() * self.rotation_sigma_rad.z,
            );
        drift.focal_scale = drift.focal_scale * retain
            + Vec2::new(
                rng.normal() * self.focal_sigma_fraction.x,
                rng.normal() * self.focal_sigma_fraction.y,
            );
        drift.principal_shift_px = drift.principal_shift_px * retain
            + Vec2::new(
                rng.normal() * self.principal_sigma_px.x,
                rng.normal() * self.principal_sigma_px.y,
            );
        drift.distortion_delta.k1 =
            retain * drift.distortion_delta.k1 + rng.normal() * self.distortion_sigma.k1;
        drift.distortion_delta.k2 =
            retain * drift.distortion_delta.k2 + rng.normal() * self.distortion_sigma.k2;
        drift.distortion_delta.k3 =
            retain * drift.distortion_delta.k3 + rng.normal() * self.distortion_sigma.k3;
        drift.distortion_delta.p1 =
            retain * drift.distortion_delta.p1 + rng.normal() * self.distortion_sigma.p1;
        drift.distortion_delta.p2 =
            retain * drift.distortion_delta.p2 + rng.normal() * self.distortion_sigma.p2;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera() -> PinholeCamera {
        PinholeCamera::new(
            ImageSize::new(640, 480),
            CameraIntrinsics::new(520.0, 515.0, 319.5, 239.5),
            BrownConrady {
                k1: -0.12,
                k2: 0.03,
                k3: -0.002,
                p1: 0.001,
                p2: -0.0007,
            },
            RigidTransform::IDENTITY,
        )
    }

    #[test]
    fn distortion_inversion_is_accurate() {
        let d = camera().distortion;
        let p = Vec2::new(0.31, -0.22);
        let recovered = d.undistort(d.distort(p)).unwrap();
        assert!((recovered - p).norm() < 1.0e-10);
    }

    #[test]
    fn project_then_unproject_points_at_target() {
        let c = camera();
        let target = Vec3::new(0.08, -0.04, 0.6);
        let projected = c.project(target).unwrap();
        let ray = c.ray(projected.pixel).unwrap();
        assert!(ray.direction.cross(target.normalized().unwrap()).norm() < 1.0e-10);
    }

    #[test]
    fn drift_walk_replays_exactly() {
        let model = DriftRandomWalk::default();
        let mut a = CalibrationDrift::ZERO;
        let mut b = CalibrationDrift::ZERO;
        model.advance(&mut a, 7, 2, 100);
        model.advance(&mut b, 7, 2, 100);
        assert_eq!(a, b);
        assert_ne!(a, CalibrationDrift::ZERO);
    }
}
