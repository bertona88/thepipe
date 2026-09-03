//! First-order optical precision calculations used for architecture trades.
//!
//! These calculations are deliberately analytic. They turn declared geometry
//! and uncertainty allocations into predictions; they do not qualify hardware.

use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RandomTriangulationPrecision {
    pub lateral_sigma_m: f64,
    pub axial_sigma_m: f64,
}

/// Propagate independent image-coordinate uncertainty through a pinhole
/// triangulation at the centre of the declared field.
pub fn random_triangulation_precision(
    range_m: f64,
    focal_length_px: f64,
    triangulation_angle_rad: f64,
    camera_localization_sigma_px: f64,
    correspondence_localization_sigma_px: f64,
) -> Option<RandomTriangulationPrecision> {
    let values = [
        range_m,
        focal_length_px,
        triangulation_angle_rad,
        camera_localization_sigma_px,
        correspondence_localization_sigma_px,
    ];
    if values.iter().any(|value| !value.is_finite())
        || range_m <= 0.0
        || focal_length_px <= 0.0
        || !(0.0..core::f64::consts::PI).contains(&triangulation_angle_rad)
        || camera_localization_sigma_px < 0.0
        || correspondence_localization_sigma_px < 0.0
    {
        return None;
    }
    let sine = triangulation_angle_rad.sin();
    if sine <= 1.0e-12 {
        return None;
    }
    let lateral_sigma_m = range_m * camera_localization_sigma_px / focal_length_px;
    let disparity_sigma_px = camera_localization_sigma_px
        .hypot(correspondence_localization_sigma_px);
    let axial_sigma_m = range_m * disparity_sigma_px / (focal_length_px * sine);
    Some(RandomTriangulationPrecision {
        lateral_sigma_m,
        axial_sigma_m,
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrecisionModelInput {
    pub image_width_px: u32,
    /// Width of the object-plane field represented by `image_width_px`.
    pub field_width_at_target_m: f64,
    /// Slant range from the camera entrance pupil to the target datum.
    pub range_m: f64,
    /// Included angle between the two rays at the target datum.
    pub triangulation_angle_rad: f64,
    pub camera_localization_sigma_px: f64,
    pub correspondence_localization_sigma_px: f64,
    /// Correlated residual after intrinsic/extrinsic calibration. Applied to
    /// both lateral and axial estimates and not reduced by frame averaging.
    pub correlated_calibration_sigma_m: f64,
    /// Reflectance, speckle, and imperfect-surface residual in depth.
    pub surface_axial_sigma_m: f64,
    /// Quantizer step, treated as a uniform distribution.
    pub depth_quantization_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrecisionPrediction {
    pub object_space_sampling_m_px: f64,
    pub effective_focal_length_px: f64,
    pub lateral_random_sigma_m: f64,
    pub axial_geometric_sigma_m: f64,
    pub lateral_total_sigma_m: f64,
    pub axial_total_sigma_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrecisionModelError {
    InvalidImageWidth,
    InvalidGeometry,
    InvalidUncertainty,
}

impl fmt::Display for PrecisionModelError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidImageWidth => formatter.write_str("image width must be positive"),
            Self::InvalidGeometry => formatter.write_str(
                "field width, range, and triangulation angle must describe finite positive geometry",
            ),
            Self::InvalidUncertainty => formatter.write_str(
                "localization, calibration, surface, and quantization uncertainties must be finite and non-negative",
            ),
        }
    }
}

impl PrecisionModelInput {
    pub fn predict(self) -> Result<PrecisionPrediction, PrecisionModelError> {
        if self.image_width_px == 0 {
            return Err(PrecisionModelError::InvalidImageWidth);
        }
        if !self.field_width_at_target_m.is_finite()
            || !self.range_m.is_finite()
            || !self.triangulation_angle_rad.is_finite()
            || self.field_width_at_target_m <= 0.0
            || self.range_m <= 0.0
            || !(0.0..core::f64::consts::PI).contains(&self.triangulation_angle_rad)
        {
            return Err(PrecisionModelError::InvalidGeometry);
        }
        let uncertainties = [
            self.camera_localization_sigma_px,
            self.correspondence_localization_sigma_px,
            self.correlated_calibration_sigma_m,
            self.surface_axial_sigma_m,
            self.depth_quantization_m,
        ];
        if uncertainties
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(PrecisionModelError::InvalidUncertainty);
        }

        let object_space_sampling_m_px =
            self.field_width_at_target_m / f64::from(self.image_width_px);
        let effective_focal_length_px = self.range_m / object_space_sampling_m_px;
        let random = random_triangulation_precision(
            self.range_m,
            effective_focal_length_px,
            self.triangulation_angle_rad,
            self.camera_localization_sigma_px,
            self.correspondence_localization_sigma_px,
        )
        .ok_or(PrecisionModelError::InvalidGeometry)?;
        let quantization_sigma_m = self.depth_quantization_m / 12.0_f64.sqrt();
        Ok(PrecisionPrediction {
            object_space_sampling_m_px,
            effective_focal_length_px,
            lateral_random_sigma_m: random.lateral_sigma_m,
            axial_geometric_sigma_m: random.axial_sigma_m,
            lateral_total_sigma_m: random
                .lateral_sigma_m
                .hypot(self.correlated_calibration_sigma_m),
            axial_total_sigma_m: random
                .axial_sigma_m
                .hypot(self.surface_axial_sigma_m)
                .hypot(quantization_sigma_m)
                .hypot(self.correlated_calibration_sigma_m),
        })
    }
}

/// Remaining independent RMS allocation after known contributors are combined
/// by root-sum-square. `None` means the declared contributors already consume
/// the target or an input is invalid.
pub fn remaining_independent_rms_budget(
    target_sigma_m: f64,
    contributors_sigma_m: &[f64],
) -> Option<f64> {
    if !target_sigma_m.is_finite() || target_sigma_m <= 0.0 {
        return None;
    }
    let mut used_variance_m2 = 0.0;
    for contributor in contributors_sigma_m {
        if !contributor.is_finite() || *contributor < 0.0 {
            return None;
        }
        used_variance_m2 += contributor * contributor;
    }
    let remaining_variance_m2 = target_sigma_m * target_sigma_m - used_variance_m2;
    (remaining_variance_m2 > 0.0).then_some(remaining_variance_m2.sqrt())
}

/// Included angle for two entrance pupils separated by `baseline_m`, symmetric
/// about a datum at perpendicular working distance `working_distance_m`.
pub fn symmetric_triangulation_angle_rad(
    baseline_m: f64,
    working_distance_m: f64,
) -> Option<f64> {
    if !baseline_m.is_finite()
        || !working_distance_m.is_finite()
        || baseline_m <= 0.0
        || working_distance_m <= 0.0
    {
        return None;
    }
    Some(2.0 * (0.5 * baseline_m / working_distance_m).atan())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn macro_reference_prediction_matches_hand_calculation() {
        let angle = symmetric_triangulation_angle_rad(0.012, 0.015).unwrap();
        let prediction = PrecisionModelInput {
            image_width_px: 1280,
            field_width_at_target_m: 0.0025,
            range_m: 0.015_f64.hypot(0.006),
            triangulation_angle_rad: angle,
            camera_localization_sigma_px: 0.18,
            correspondence_localization_sigma_px: 0.18,
            correlated_calibration_sigma_m: 3.0e-6,
            surface_axial_sigma_m: 1.5e-6,
            depth_quantization_m: 0.5e-6,
        }
        .predict()
        .unwrap();
        assert!((prediction.object_space_sampling_m_px - 1.953_125e-6).abs() < 1.0e-15);
        assert!((prediction.lateral_total_sigma_m - 3.020_529e-6).abs() < 1.0e-12);
        assert!((prediction.axial_total_sigma_m - 3.433_738e-6).abs() < 1.0e-12);
    }

    #[test]
    fn infeasible_budget_fails_closed() {
        assert_eq!(
            remaining_independent_rms_budget(10.0e-6, &[8.0e-6, 8.0e-6]),
            None
        );
        assert_eq!(remaining_independent_rms_budget(0.0, &[]), None);
    }
}
