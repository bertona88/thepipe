use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Div<f64> for Vec2 {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self::new(self.x / rhs, self.y / rhs)
    }
}

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

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    pub fn normalized(self) -> Option<Self> {
        let n = self.norm();
        (n > 1.0e-15 && n.is_finite()).then_some(self / n)
    }

    pub fn component_mul(self, rhs: Self) -> Self {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }

    pub fn min(self, rhs: Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y), self.z.min(rhs.z))
    }

    pub fn max(self, rhs: Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y), self.z.max(rhs.z))
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
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
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Vec3 {
        rhs * self
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Mat3 {
    /// Row-major elements.
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    pub const ZERO: Self = Self { m: [[0.0; 3]; 3] };
    pub const IDENTITY: Self = Self {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub const fn new(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    pub fn diagonal(v: Vec3) -> Self {
        Self::new([[v.x, 0.0, 0.0], [0.0, v.y, 0.0], [0.0, 0.0, v.z]])
    }

    pub fn outer(a: Vec3, b: Vec3) -> Self {
        Self::new([
            [a.x * b.x, a.x * b.y, a.x * b.z],
            [a.y * b.x, a.y * b.y, a.y * b.z],
            [a.z * b.x, a.z * b.y, a.z * b.z],
        ])
    }

    pub fn transpose(self) -> Self {
        Self::new([
            [self.m[0][0], self.m[1][0], self.m[2][0]],
            [self.m[0][1], self.m[1][1], self.m[2][1]],
            [self.m[0][2], self.m[1][2], self.m[2][2]],
        ])
    }

    pub fn determinant(self) -> f64 {
        let m = self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    pub fn inverse(self) -> Option<Self> {
        let m = self.m;
        let det = self.determinant();
        let scale = m
            .iter()
            .flatten()
            .fold(0.0_f64, |largest, value| largest.max(value.abs()));
        // Singularity is relative to matrix scale. An absolute determinant gate
        // incorrectly rejects perfectly conditioned covariance matrices in m^2.
        let determinant_scale = scale * scale * scale;
        if !det.is_finite()
            || !scale.is_finite()
            || scale == 0.0
            || det.abs() <= 32.0 * f64::EPSILON * determinant_scale
        {
            return None;
        }
        let inv = Self::new([
            [
                m[1][1] * m[2][2] - m[1][2] * m[2][1],
                m[0][2] * m[2][1] - m[0][1] * m[2][2],
                m[0][1] * m[1][2] - m[0][2] * m[1][1],
            ],
            [
                m[1][2] * m[2][0] - m[1][0] * m[2][2],
                m[0][0] * m[2][2] - m[0][2] * m[2][0],
                m[0][2] * m[1][0] - m[0][0] * m[1][2],
            ],
            [
                m[1][0] * m[2][1] - m[1][1] * m[2][0],
                m[0][1] * m[2][0] - m[0][0] * m[2][1],
                m[0][0] * m[1][1] - m[0][1] * m[1][0],
            ],
        ]);
        Some(inv * (1.0 / det))
    }

    pub fn from_axis_angle(rotation_vector_rad: Vec3) -> Self {
        let angle = rotation_vector_rad.norm();
        if angle < 1.0e-14 {
            // First order is more useful than identity for tiny calibration drift.
            let v = rotation_vector_rad;
            return Self::new([[1.0, -v.z, v.y], [v.z, 1.0, -v.x], [-v.y, v.x, 1.0]]);
        }
        let a = rotation_vector_rad / angle;
        let (s, c) = angle.sin_cos();
        let k = 1.0 - c;
        Self::new([
            [
                c + a.x * a.x * k,
                a.x * a.y * k - a.z * s,
                a.x * a.z * k + a.y * s,
            ],
            [
                a.y * a.x * k + a.z * s,
                c + a.y * a.y * k,
                a.y * a.z * k - a.x * s,
            ],
            [
                a.z * a.x * k - a.y * s,
                a.z * a.y * k + a.x * s,
                c + a.z * a.z * k,
            ],
        ])
    }

    pub fn trace(self) -> f64 {
        self.m[0][0] + self.m[1][1] + self.m[2][2]
    }
}

impl Add for Mat3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        let mut out = Self::ZERO;
        for r in 0..3 {
            for c in 0..3 {
                out.m[r][c] = self.m[r][c] + rhs.m[r][c];
            }
        }
        out
    }
}

impl AddAssign for Mat3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl Sub for Mat3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        let mut out = Self::ZERO;
        for r in 0..3 {
            for c in 0..3 {
                out.m[r][c] = self.m[r][c] - rhs.m[r][c];
            }
        }
        out
    }
}

impl Mul<f64> for Mat3 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        let mut out = self;
        for row in &mut out.m {
            for value in row {
                *value *= rhs;
            }
        }
        out
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Vec3 {
        Vec3::new(
            self.m[0][0] * rhs.x + self.m[0][1] * rhs.y + self.m[0][2] * rhs.z,
            self.m[1][0] * rhs.x + self.m[1][1] * rhs.y + self.m[1][2] * rhs.z,
            self.m[2][0] * rhs.x + self.m[2][1] * rhs.y + self.m[2][2] * rhs.z,
        )
    }
}

impl Mul<Mat3> for Mat3 {
    type Output = Mat3;
    fn mul(self, rhs: Mat3) -> Mat3 {
        let mut out = Mat3::ZERO;
        for r in 0..3 {
            for c in 0..3 {
                for k in 0..3 {
                    out.m[r][c] += self.m[r][k] * rhs.m[k][c];
                }
            }
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidTransform {
    /// Rotation from local coordinates into world/parent coordinates.
    pub rotation: Mat3,
    /// Local origin expressed in world/parent coordinates.
    pub translation: Vec3,
}

impl Default for RigidTransform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl RigidTransform {
    pub const IDENTITY: Self = Self {
        rotation: Mat3::IDENTITY,
        translation: Vec3::ZERO,
    };

    pub const fn new(rotation: Mat3, translation: Vec3) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn transform_point(self, p: Vec3) -> Vec3 {
        self.rotation * p + self.translation
    }

    pub fn transform_vector(self, v: Vec3) -> Vec3 {
        self.rotation * v
    }

    pub fn inverse(self) -> Self {
        // Camera/fixture rotations are orthonormal by construction.
        let r = self.rotation.transpose();
        Self::new(r, r * -self.translation)
    }

    /// Composition `self * local`: apply `local`, then `self`.
    pub fn compose(self, local: Self) -> Self {
        Self::new(
            self.rotation * local.rotation,
            self.transform_point(local.translation),
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    pub origin: Vec3,
    /// Unit direction.
    pub direction: Vec3,
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3) -> Option<Self> {
        direction
            .normalized()
            .map(|direction| Self { origin, direction })
    }

    pub fn at(self, distance_m: f64) -> Vec3 {
        self.origin + self.direction * distance_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_inverse_round_trip() {
        let m = Mat3::new([[2.0, 1.0, 0.1], [0.2, 3.0, 0.0], [0.0, 0.5, 4.0]]);
        let product = m * m.inverse().unwrap();
        for r in 0..3 {
            for c in 0..3 {
                let expected = if r == c { 1.0 } else { 0.0 };
                assert!((product.m[r][c] - expected).abs() < 1.0e-12);
            }
        }
    }

    #[test]
    fn inverse_is_scale_invariant_for_micron_covariance() {
        let covariance = Mat3::diagonal(Vec3::splat(1.0e-12));
        let information = covariance.inverse().unwrap();
        assert!((information.m[0][0] - 1.0e12).abs() < 1.0);
        let product = covariance * information;
        assert!((product.m[0][0] - 1.0).abs() < 1.0e-12);
    }
}
