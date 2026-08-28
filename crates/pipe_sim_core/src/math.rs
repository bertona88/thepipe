//! Small deterministic 3-D math layer.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

pub const EPSILON: f64 = 1.0e-12;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);

    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn splat(value: f64) -> Self {
        Self::new(value, value, value)
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    #[inline]
    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    #[inline]
    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    #[inline]
    pub fn normalized_or(self, fallback: Self) -> Self {
        let length = self.length();
        if length > EPSILON && length.is_finite() {
            self / length
        } else {
            fallback
        }
    }

    #[inline]
    pub fn normalized(self) -> Self {
        self.normalized_or(Self::ZERO)
    }

    #[inline]
    pub fn min(self, rhs: Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y), self.z.min(rhs.z))
    }

    #[inline]
    pub fn max(self, rhs: Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y), self.z.max(rhs.z))
    }

    #[inline]
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    #[inline]
    pub fn clamp(self, min: Self, max: Self) -> Self {
        Self::new(
            self.x.clamp(min.x, max.x),
            self.y.clamp(min.y, max.y),
            self.z.clamp(min.z, max.z),
        )
    }

    #[inline]
    pub fn component(self, index: usize) -> f64 {
        match index {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!("Vec3 component index out of range"),
        }
    }

    #[inline]
    pub fn with_component(mut self, index: usize, value: f64) -> Self {
        match index {
            0 => self.x = value,
            1 => self.y = value,
            2 => self.z = value,
            _ => panic!("Vec3 component index out of range"),
        }
        self
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    #[inline]
    pub fn lerp(self, rhs: Self, t: f64) -> Self {
        self + (rhs - self) * t
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Self::Output {
        rhs * self
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    /// Row-major entries.
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    pub const IDENTITY: Self = Self {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub const fn from_rows(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    pub fn transpose(self) -> Self {
        Self::from_rows([
            [self.m[0][0], self.m[1][0], self.m[2][0]],
            [self.m[0][1], self.m[1][1], self.m[2][1]],
            [self.m[0][2], self.m[1][2], self.m[2][2]],
        ])
    }

    pub fn mul_vec3(self, value: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * value.x + self.m[0][1] * value.y + self.m[0][2] * value.z,
            self.m[1][0] * value.x + self.m[1][1] * value.y + self.m[1][2] * value.z,
            self.m[2][0] * value.x + self.m[2][1] * value.y + self.m[2][2] * value.z,
        )
    }

    pub fn column(self, index: usize) -> Vec3 {
        Vec3::new(self.m[0][index], self.m[1][index], self.m[2][index])
    }

    pub fn abs(self) -> Self {
        let mut result = self;
        for row in &mut result.m {
            for value in row {
                *value = value.abs();
            }
        }
        result
    }
}

impl Default for Mat3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Self::Output {
        self.mul_vec3(rhs)
    }
}

impl Mul for Mat3 {
    type Output = Mat3;
    fn mul(self, rhs: Self) -> Self::Output {
        let mut result = [[0.0; 3]; 3];
        for (row, values) in result.iter_mut().enumerate() {
            for (column, value) in values.iter_mut().enumerate() {
                *value = (0..3).map(|k| self.m[row][k] * rhs.m[k][column]).sum();
            }
        }
        Mat3::from_rows(result)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub const IDENTITY: Self = Self::new(1.0, 0.0, 0.0, 0.0);

    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn from_axis_angle(axis: Vec3, angle_rad: f64) -> Self {
        let axis = axis.normalized_or(Vec3::Z);
        let half = angle_rad * 0.5;
        let sine = half.sin();
        Self::new(half.cos(), axis.x * sine, axis.y * sine, axis.z * sine).normalized()
    }

    pub fn from_scaled_axis(scaled_axis: Vec3) -> Self {
        let angle = scaled_axis.length();
        if angle <= EPSILON {
            Self::IDENTITY
        } else {
            Self::from_axis_angle(scaled_axis / angle, angle)
        }
    }

    /// Shortest rotation carrying `from` onto `to`.
    pub fn from_two_vectors(from: Vec3, to: Vec3) -> Self {
        let a = from.normalized_or(Vec3::Z);
        let b = to.normalized_or(Vec3::Z);
        let dot = a.dot(b).clamp(-1.0, 1.0);
        if dot > 1.0 - 1.0e-12 {
            Self::IDENTITY
        } else if dot < -1.0 + 1.0e-12 {
            let helper = if a.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
            Self::from_axis_angle(
                a.cross(helper).normalized_or(Vec3::Z),
                core::f64::consts::PI,
            )
        } else {
            let cross = a.cross(b);
            Self::new(1.0 + dot, cross.x, cross.y, cross.z).normalized()
        }
    }

    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    pub fn length_squared(self) -> f64 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn normalized(self) -> Self {
        let length = self.length_squared().sqrt();
        if length <= EPSILON || !length.is_finite() {
            Self::IDENTITY
        } else {
            Self::new(
                self.w / length,
                self.x / length,
                self.y / length,
                self.z / length,
            )
        }
    }

    pub fn inverse(self) -> Self {
        let norm = self.length_squared();
        if norm <= EPSILON {
            Self::IDENTITY
        } else {
            let conjugate = self.conjugate();
            Self::new(
                conjugate.w / norm,
                conjugate.x / norm,
                conjugate.y / norm,
                conjugate.z / norm,
            )
        }
    }

    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn rotate_vec3(self, value: Vec3) -> Vec3 {
        // Equivalent to q * (0, value) * conjugate(q), with fewer operations.
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(value) * 2.0;
        value + t * self.w + qv.cross(t)
    }

    pub fn to_mat3(self) -> Mat3 {
        let q = self.normalized();
        let (xx, yy, zz) = (q.x * q.x, q.y * q.y, q.z * q.z);
        let (xy, xz, yz) = (q.x * q.y, q.x * q.z, q.y * q.z);
        let (wx, wy, wz) = (q.w * q.x, q.w * q.y, q.w * q.z);
        Mat3::from_rows([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ])
    }
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul for Quat {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Position in metres.
    pub translation: Vec3,
    pub rotation: Quat,
}

impl Pose {
    pub const IDENTITY: Self = Self::new(Vec3::ZERO, Quat::IDENTITY);

    pub const fn new(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub const fn from_translation(translation: Vec3) -> Self {
        Self::new(translation, Quat::IDENTITY)
    }

    pub fn transform_point(self, local: Vec3) -> Vec3 {
        self.translation + self.rotation.rotate_vec3(local)
    }

    pub fn transform_vector(self, local: Vec3) -> Vec3 {
        self.rotation.rotate_vec3(local)
    }

    pub fn inverse_transform_point(self, world: Vec3) -> Vec3 {
        self.rotation
            .inverse()
            .rotate_vec3(world - self.translation)
    }

    pub fn inverse_transform_vector(self, world: Vec3) -> Vec3 {
        self.rotation.inverse().rotate_vec3(world)
    }

    pub fn inverse(self) -> Self {
        let inverse_rotation = self.rotation.inverse();
        Self::new(
            inverse_rotation.rotate_vec3(-self.translation),
            inverse_rotation,
        )
    }

    /// Compose poses so `self * rhs` applies `rhs` in `self`'s frame.
    pub fn compose(self, rhs: Self) -> Self {
        Self::new(
            self.transform_point(rhs.translation),
            (self.rotation * rhs.rotation).normalized(),
        )
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul<Pose> for Pose {
    type Output = Pose;
    fn mul(self, rhs: Pose) -> Self::Output {
        self.compose(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) {
        assert!((a - b).length() < 1.0e-10, "{a:?} != {b:?}");
    }

    #[test]
    fn cross_product_is_right_handed() {
        assert_eq!(Vec3::X.cross(Vec3::Y), Vec3::Z);
    }

    #[test]
    fn quaternion_rotates_and_inverts() {
        let q = Quat::from_axis_angle(Vec3::Z, core::f64::consts::FRAC_PI_2);
        close(q.rotate_vec3(Vec3::X), Vec3::Y);
        close(q.inverse().rotate_vec3(Vec3::Y), Vec3::X);
    }

    #[test]
    fn pose_composition_matches_nested_transforms() {
        let a = Pose::new(
            Vec3::new(1.0, 2.0, 3.0),
            Quat::from_axis_angle(Vec3::Y, 0.4),
        );
        let b = Pose::new(
            Vec3::new(-0.5, 0.2, 0.7),
            Quat::from_axis_angle(Vec3::X, -0.2),
        );
        let point = Vec3::new(0.1, 0.2, 0.3);
        close(
            (a * b).transform_point(point),
            a.transform_point(b.transform_point(point)),
        );
        close(
            (a * b)
                .inverse()
                .transform_point((a * b).transform_point(point)),
            point,
        );
    }
}
