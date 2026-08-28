use crate::math::{Mat3, Ray, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Triangulation {
    /// Midpoint of the shortest segment between the two rays.
    pub point: Vec3,
    pub point_on_first_ray: Vec3,
    pub point_on_second_ray: Vec3,
    pub first_distance_m: f64,
    pub second_distance_m: f64,
    /// Shortest distance between the rays; a useful correspondence residual.
    pub ray_separation_m: f64,
    /// Acute angle between viewing lines.
    pub intersection_angle_rad: f64,
    /// Approximate line-intersection condition number. 1 is orthogonal; infinity parallel.
    pub condition_number: f64,
}

/// Least-squares closest point between two infinite viewing rays.
///
/// Negative ray distances are preserved so callers can explicitly reject points
/// reconstructed behind a camera/projector.
pub fn triangulate_rays(first: Ray, second: Ray) -> Option<Triangulation> {
    let b = first.direction.dot(second.direction).clamp(-1.0, 1.0);
    let denominator = 1.0 - b * b;
    if denominator < 1.0e-14 {
        return None;
    }
    let offset = first.origin - second.origin;
    let d = first.direction.dot(offset);
    let e = second.direction.dot(offset);
    let first_distance_m = (b * e - d) / denominator;
    let second_distance_m = (e - b * d) / denominator;
    let point_on_first_ray = first.at(first_distance_m);
    let point_on_second_ray = second.at(second_distance_m);
    let point = (point_on_first_ray + point_on_second_ray) * 0.5;
    let abs_cos = b.abs();
    let intersection_angle_rad = abs_cos.acos();
    Some(Triangulation {
        point,
        point_on_first_ray,
        point_on_second_ray,
        first_distance_m,
        second_distance_m,
        ray_separation_m: (point_on_first_ray - point_on_second_ray).norm(),
        intersection_angle_rad,
        condition_number: (1.0 + abs_cos) / (1.0 - abs_cos).max(1.0e-15),
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Covariance3 {
    /// Symmetric position covariance in square metres.
    pub matrix_m2: Mat3,
}

impl Covariance3 {
    pub fn isotropic(sigma_m: f64) -> Self {
        Self {
            matrix_m2: Mat3::IDENTITY * sigma_m.max(0.0).powi(2),
        }
    }

    /// Axial/lateral model around a unit viewing direction.
    pub fn viewing_ray(direction: Vec3, lateral_sigma_m: f64, axial_sigma_m: f64) -> Self {
        let axis = direction.normalized().unwrap_or(Vec3::Z);
        let lateral_variance = lateral_sigma_m.max(0.0).powi(2);
        let axial_variance = axial_sigma_m.max(0.0).powi(2);
        Self {
            matrix_m2: Mat3::IDENTITY * lateral_variance
                + Mat3::outer(axis, axis) * (axial_variance - lateral_variance),
        }
    }

    pub fn information(self) -> Option<Mat3> {
        self.matrix_m2.inverse()
    }

    pub fn rms_sigma_m(self) -> f64 {
        (self.matrix_m2.trace().max(0.0) / 3.0).sqrt()
    }

    pub fn axial_variance_m2(self, direction: Vec3) -> Option<f64> {
        let axis = direction.normalized()?;
        Some(axis.dot(self.matrix_m2 * axis).max(0.0))
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct QualityMetrics {
    pub signal_to_noise: f64,
    pub triangulation_angle_rad: f64,
    pub ray_separation_m: f64,
    pub reprojection_error_px: f64,
    pub estimated_axial_sigma_m: f64,
    pub condition_number: f64,
    /// Compact 0..1 score for visualization/planning, not a substitute for covariance.
    pub confidence: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointEstimate {
    pub point: Vec3,
    pub covariance: Covariance3,
    pub quality: QualityMetrics,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FusedPoint {
    pub point: Vec3,
    pub covariance: Covariance3,
    /// Sum of covariance-weighted squared residuals; useful for detecting bad matches.
    pub chi_squared: f64,
    pub contributors: usize,
}

/// Information-form fusion of independent 3D point estimates.
pub fn fuse_points(estimates: &[PointEstimate]) -> Option<FusedPoint> {
    if estimates.is_empty() {
        return None;
    }
    let mut total_information = Mat3::ZERO;
    let mut information_point = Vec3::ZERO;
    let mut valid = 0;
    for estimate in estimates {
        if !estimate.point.is_finite() {
            continue;
        }
        let Some(information) = estimate.covariance.information() else {
            continue;
        };
        total_information += information;
        information_point += information * estimate.point;
        valid += 1;
    }
    let fused_covariance = total_information.inverse()?;
    let point = fused_covariance * information_point;
    let mut chi_squared = 0.0;
    for estimate in estimates {
        if let Some(information) = estimate.covariance.information() {
            let residual = estimate.point - point;
            chi_squared += residual.dot(information * residual);
        }
    }
    Some(FusedPoint {
        point,
        covariance: Covariance3 {
            matrix_m2: fused_covariance,
        },
        chi_squared,
        contributors: valid,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intersecting_rays_reconstruct_known_point() {
        let point = Vec3::new(0.02, -0.01, 0.4);
        let a = Ray::new(Vec3::ZERO, point).unwrap();
        let b = Ray::new(Vec3::new(0.1, 0.0, 0.0), point - Vec3::new(0.1, 0.0, 0.0)).unwrap();
        let result = triangulate_rays(a, b).unwrap();
        assert!((result.point - point).norm() < 1.0e-12);
        assert!(result.ray_separation_m < 1.0e-12);
        assert!(result.first_distance_m > 0.0 && result.second_distance_m > 0.0);
    }

    #[test]
    fn information_fusion_weights_precise_point_more() {
        let estimates = [
            PointEstimate {
                point: Vec3::new(0.0, 0.0, 1.0),
                covariance: Covariance3::isotropic(0.001),
                quality: QualityMetrics::default(),
            },
            PointEstimate {
                point: Vec3::new(0.1, 0.0, 1.0),
                covariance: Covariance3::isotropic(0.1),
                quality: QualityMetrics::default(),
            },
        ];
        let fused = fuse_points(&estimates).unwrap();
        assert!(fused.point.x < 0.001);
        assert!(fused.covariance.rms_sigma_m() < 0.001);
        assert_eq!(fused.contributors, 2);
    }
}
