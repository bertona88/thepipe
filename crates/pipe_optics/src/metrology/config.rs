use super::*;
use crate::{BrownConrady, CameraIntrinsics, ImageSize, Mat3, PinholeCamera, RigidTransform, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasurementClass {
    Occupancy,
    ToolPose,
    CriticalFeature,
    DenseGeometry,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasurementMode {
    Tracking,
    Precision,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IlluminationKind {
    Diffuse,
    Backlight,
    Structured,
    Grazing,
    Dark,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Polarization {
    None,
    ParallelLinear,
    CrossedLinear,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Illumination {
    pub kind: IlluminationKind,
    pub wavelength_m: f64,
    pub polarization: Polarization,
    pub source_id: u32,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LensModel {
    Perspective,
    TelecentricCandidateUnsupported,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceKind {
    Camera,
    Projector,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OpticalDevice {
    pub id: u32,
    pub kind: DeviceKind,
    pub model: PinholeCamera,
    pub lens_model: LensModel,
    pub pixel_pitch_m: [f64; 2],
    /// Effective focal distance at the sensor. At macro magnification this is
    /// NOT the thin lens's infinity focal length.
    pub effective_focal_length_m: f64,
    pub nominal_lens_focal_length_m: f64,
    pub focus_working_distance_m: f64,
    pub usable_depth_of_field_m: f64,
    pub aperture_diameter_m: f64,
    pub housing_radius_m: f64,
    pub bit_depth: u8,
    pub monochrome: bool,
    pub global_shutter: bool,
    pub raw_access: bool,
    pub hardware_trigger: bool,
    pub deterministic_exposure: bool,
    pub focus_locked: bool,
    pub autofocus: bool,
    pub electronic_stabilization: bool,
    pub temperature_k: f64,
    pub focal_fraction_per_k: f64,
}
impl OpticalDevice {
    pub fn sampling_m_per_px(&self, world: Vec3) -> Option<[f64; 2]> {
        let z = self.model.project(world)?.optical_depth_m;
        Some([
            z / self.model.intrinsics.fx_px,
            z / self.model.intrinsics.fy_px,
        ])
    }
    pub fn field_at_focus_m(&self) -> [f64; 2] {
        [
            self.focus_working_distance_m * self.model.image_size.width as f64
                / self.model.intrinsics.fx_px,
            self.focus_working_distance_m * self.model.image_size.height as f64
                / self.model.intrinsics.fy_px,
        ]
    }
    pub fn validate(&self) -> Result<(), MetrologyError> {
        let d = self.model.distortion;
        if self.id == 0
            || self.model.image_size.width > 65535
            || self.model.image_size.height > 65535
            || !self.model.is_valid()
            || !valid_transform(self.model.world_from_camera)
            || ![d.k1, d.k2, d.k3, d.p1, d.p2, self.focal_fraction_per_k]
                .iter()
                .all(|v| v.is_finite())
            || ![
                self.pixel_pitch_m[0],
                self.pixel_pitch_m[1],
                self.effective_focal_length_m,
                self.nominal_lens_focal_length_m,
                self.focus_working_distance_m,
                self.usable_depth_of_field_m,
                self.aperture_diameter_m,
                self.housing_radius_m,
                self.temperature_k,
            ]
            .iter()
            .all(|&v| positive(v))
            || !(8..=16).contains(&self.bit_depth)
            || !self.focus_locked
            || self.autofocus
            || self.electronic_stabilization
            || !self.hardware_trigger
            || !self.deterministic_exposure
            || (self.kind == DeviceKind::Camera && (!self.global_shutter || !self.raw_access))
        {
            return Err(config_error(
                "invalid sensor, pose, focus or acquisition capability",
            ));
        }
        if self.lens_model != LensModel::Perspective {
            return Err(config_error(
                "telecentric candidate is representable but not simulated",
            ));
        }
        for (f, p) in [
            (self.model.intrinsics.fx_px, self.pixel_pitch_m[0]),
            (self.model.intrinsics.fy_px, self.pixel_pitch_m[1]),
        ] {
            if (f * p / self.effective_focal_length_m - 1.0).abs() > 1e-6 {
                return Err(config_error("focal length and pixel intrinsics disagree"));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrecisionVolume {
    pub center_world_m: Vec3,
    pub size_m: Vec3,
}
impl PrecisionVolume {
    pub fn contains(&self, p: Vec3) -> bool {
        let d = p - self.center_world_m;
        p.is_finite()
            && d.x.abs() <= self.size_m.x * 0.5
            && d.y.abs() <= self.size_m.y * 0.5
            && d.z.abs() <= self.size_m.z * 0.5
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationLayer {
    Sensor,
    CameraIntrinsics,
    CameraDistortion,
    CameraExtrinsics,
    ProjectorIntrinsics,
    ProjectorDistortion,
    ProjectorExtrinsics,
    WorldFrame,
    TubeReferences,
    FiducialGeometry,
    FiducialToTcp,
    PartFeatures,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationRecord {
    pub layer: CalibrationLayer,
    pub revision: String,
    pub parent_frame: String,
    pub child_frame: String,
    pub valid_until_s: f64,
    pub valid: bool,
    /// Identity of the physical artifact, not just an image or feature number.
    pub fit_artifact_ids: Vec<String>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    pub id: String,
    pub provenance: CalibrationProvenance,
    pub records: Vec<CalibrationRecord>,
    pub reference_temperature_k: f64,
    pub world_from_tube: RigidTransform,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationProvenance {
    SyntheticNominal,
    PhysicalArtifactFit,
}
impl Calibration {
    pub fn valid_at(&self, t: f64) -> bool {
        positive(self.reference_temperature_k)
            && !self.id.is_empty()
            && nonnegative(t)
            && valid_transform(self.world_from_tube)
            && self.records.len() == 12
            && self
                .records
                .iter()
                .map(|r| r.layer)
                .collect::<BTreeSet<_>>()
                .len()
                == 12
            && self.records.iter().all(|r| {
                r.valid
                    && !r.revision.is_empty()
                    && !r.parent_frame.is_empty()
                    && !r.child_frame.is_empty()
                    && r.valid_until_s.is_finite()
                    && r.valid_until_s >= t
            })
    }
    pub fn excludes(&self, artifact: &str) -> bool {
        !artifact.is_empty()
            && !self
                .records
                .iter()
                .any(|r| r.fit_artifact_ids.iter().any(|id| id == artifact))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalState {
    pub structure_temperature_k: f64,
    pub tube_temperature_k: f64,
    pub reference_temperature_k: f64,
    pub structure_expansion_per_k: f64,
    pub reference_expansion_per_k: f64,
    pub anchor_world_m: Vec3,
    pub maximum_calibrated_delta_k: f64,
}
impl ThermalState {
    pub fn expanded(&self, p: Vec3, calibration_k: f64) -> Vec3 {
        self.anchor_world_m
            + (p - self.anchor_world_m)
                * (1.0
                    + self.structure_expansion_per_k
                        * (self.structure_temperature_k - calibration_k))
    }
}
/// Each non-pixel entry is a TOTAL 3-D RMS target in metres. These are explicit
/// provisional priors, NOT fitted accuracy results. Common terms are added once
/// after the observation solve; no reduction with feature/view/burst count.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ErrorBudget {
    pub localization_sigma_px: f64,
    pub tracking_localization_sigma_px: f64,
    pub photon_localization_coefficient_px: f64,
    pub camera_calibration_rms_m: f64,
    pub projector_calibration_rms_m: f64,
    pub distortion_residual_rms_m: f64,
    pub structural_drift_rms_m: f64,
    pub residual_thermal_rms_m: f64,
    pub surface_interaction_rms_m: f64,
    pub reconstruction_rms_m: f64,
    pub fiducial_manufacturing_rms_m: f64,
    pub tcp_calibration_rms_m: f64,
    pub tcp_rotation_rms_rad: f64,
    pub correspondence_sigma_px: f64,
    pub phase_intensity_sigma: f64,
}
impl ErrorBudget {
    pub fn common_variance_m2(&self, structured: bool) -> f64 {
        (self.camera_calibration_rms_m.powi(2)
            + if structured {
                self.projector_calibration_rms_m.powi(2)
            } else {
                0.0
            }
            + self.distortion_residual_rms_m.powi(2)
            + self.structural_drift_rms_m.powi(2)
            + self.residual_thermal_rms_m.powi(2)
            + self.surface_interaction_rms_m.powi(2)
            + self.reconstruction_rms_m.powi(2))
            / 3.0
    }
    pub fn validate(&self) -> bool {
        positive(self.localization_sigma_px)
            && positive(self.tracking_localization_sigma_px)
            && [
                self.photon_localization_coefficient_px,
                self.camera_calibration_rms_m,
                self.projector_calibration_rms_m,
                self.distortion_residual_rms_m,
                self.structural_drift_rms_m,
                self.residual_thermal_rms_m,
                self.surface_interaction_rms_m,
                self.reconstruction_rms_m,
                self.fiducial_manufacturing_rms_m,
                self.tcp_calibration_rms_m,
                self.tcp_rotation_rms_rad,
                self.correspondence_sigma_px,
                self.phase_intensity_sigma,
            ]
            .iter()
            .all(|&v| nonnegative(v))
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcquisitionConfig {
    pub tracking_rate_hz: f64,
    pub exposure_s: f64,
    pub trigger_sigma_s: f64,
    pub max_trigger_skew_s: f64,
    pub processing_latency_s: f64,
    pub max_exposure_motion_m: f64,
    pub max_sequence_motion_m: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceFeature {
    pub artifact_id: String,
    pub feature_id: u32,
    pub point_tube_m: Vec3,
    pub characterized_rms_m: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetrologyConfig {
    pub schema_version: u32,
    pub id: String,
    pub tube_id_m: f64,
    pub tube_working_length_m: f64,
    pub precision_volume: PrecisionVolume,
    pub devices: Vec<OpticalDevice>,
    pub calibration: Calibration,
    pub references: Vec<ReferenceFeature>,
    pub thermal: ThermalState,
    pub errors: ErrorBudget,
    pub acquisition: AcquisitionConfig,
    pub patterns: PatternConfig,
    pub illumination: Illumination,
    pub minimum_angle_rad: f64,
    pub maximum_normalized_residual: f64,
    pub seed: u64,
}
impl MetrologyConfig {
    pub fn validate(&self) -> Result<(), MetrologyError> {
        if self.schema_version != 1
            || self.id.is_empty()
            || !positive(self.tube_id_m)
            || !positive(self.tube_working_length_m)
            || !self.precision_volume.center_world_m.is_finite()
            || ![
                self.precision_volume.size_m.x,
                self.precision_volume.size_m.y,
                self.precision_volume.size_m.z,
            ]
            .iter()
            .all(|&v| positive(v))
            || !self.errors.validate()
            || !self.calibration.valid_at(0.0)
        {
            return Err(config_error(
                "invalid volume, errors or calibration hierarchy",
            ));
        }
        let z = self
            .calibration
            .world_from_tube
            .inverse()
            .transform_point(self.precision_volume.center_world_m);
        // Conservative enclosing sphere keeps a rotated zone inside the tube.
        let r = self.precision_volume.size_m.norm() * 0.5;
        if z.x.hypot(z.y) + r > self.tube_id_m * 0.5
            || z.z.abs() + r > self.tube_working_length_m * 0.5
        {
            return Err(config_error("precision volume outside tube"));
        }
        let mut ids = BTreeSet::new();
        for d in &self.devices {
            d.validate()?;
            if !ids.insert(d.id) {
                return Err(config_error("duplicate sensor id"));
            }
        }
        if self
            .devices
            .iter()
            .filter(|d| d.kind == DeviceKind::Camera)
            .count()
            < 2
        {
            return Err(config_error("at least two cameras required"));
        }
        let a = &self.acquisition;
        let t = &self.thermal;
        if ![
            a.tracking_rate_hz,
            a.exposure_s,
            a.max_exposure_motion_m,
            a.max_sequence_motion_m,
            t.structure_temperature_k,
            t.tube_temperature_k,
            t.reference_temperature_k,
            t.maximum_calibrated_delta_k,
            self.illumination.wavelength_m,
            self.maximum_normalized_residual,
        ]
        .iter()
        .all(|&v| positive(v))
            || ![
                a.trigger_sigma_s,
                a.max_trigger_skew_s,
                a.processing_latency_s,
                t.structure_expansion_per_k,
                t.reference_expansion_per_k,
            ]
            .iter()
            .all(|&v| nonnegative(v))
            || a.exposure_s > 1.0 / a.tracking_rate_hz
            || !t.anchor_world_m.is_finite()
            || !positive(self.minimum_angle_rad)
            || self.minimum_angle_rad >= std::f64::consts::FRAC_PI_2
        {
            return Err(config_error("invalid timing, thermal or geometric limits"));
        }
        let mut reference_ids = BTreeSet::new();
        if self.references.len() < 3
            || self.references.iter().any(|r| {
                !r.point_tube_m.is_finite()
                    || !nonnegative(r.characterized_rms_m)
                    || !self.calibration.excludes(&r.artifact_id)
                    || !reference_ids.insert(r.feature_id)
            })
        {
            return Err(config_error(
                "online references must be distinct and excluded from calibration fit",
            ));
        }
        self.patterns.validate()?;
        if a.exposure_s > 1.0 / self.patterns.frame_rate_hz {
            return Err(config_error("exposure exceeds pattern interval"));
        }
        Ok(())
    }
    pub fn device(&self, id: u32) -> Result<&OpticalDevice, MetrologyError> {
        self.devices
            .iter()
            .find(|d| d.id == id)
            .ok_or(MetrologyError::InvalidObservation)
    }
    pub fn baseline() -> Self {
        let center = Vec3::ZERO;
        let mut devices = Vec::new();
        for i in 0..6 {
            let angle = (i as f64 * 60.0 + 15.0).to_radians();
            let eye = Vec3::new(
                0.05 * angle.cos(),
                0.05 * angle.sin(),
                if i % 2 == 0 { 0.010 } else { -0.010 },
            );
            devices.push(baseline_device(i + 1, DeviceKind::Camera, eye, center));
        }
        devices.push(baseline_device(
            101,
            DeviceKind::Projector,
            Vec3::new(0.035, -0.035, 0.0),
            center,
        ));
        let layers = [
            CalibrationLayer::Sensor,
            CalibrationLayer::CameraIntrinsics,
            CalibrationLayer::CameraDistortion,
            CalibrationLayer::CameraExtrinsics,
            CalibrationLayer::ProjectorIntrinsics,
            CalibrationLayer::ProjectorDistortion,
            CalibrationLayer::ProjectorExtrinsics,
            CalibrationLayer::WorldFrame,
            CalibrationLayer::TubeReferences,
            CalibrationLayer::FiducialGeometry,
            CalibrationLayer::FiducialToTcp,
            CalibrationLayer::PartFeatures,
        ];
        Self {
            schema_version: 1,
            id: "cylindrical_metrology_candidate_v1".into(),
            tube_id_m: 0.100,
            tube_working_length_m: 0.160,
            precision_volume: PrecisionVolume {
                center_world_m: center,
                size_m: Vec3::splat(0.025),
            },
            devices,
            calibration: Calibration {
                id: "synthetic_calibration_v1".into(),
                provenance: CalibrationProvenance::SyntheticNominal,
                reference_temperature_k: 293.15,
                world_from_tube: RigidTransform::IDENTITY,
                records: layers
                    .iter()
                    .map(|&layer| CalibrationRecord {
                        layer,
                        revision: "synthetic_nominal_not_hardware_calibrated".into(),
                        parent_frame: "world".into(),
                        child_frame: format!("{layer:?}"),
                        valid_until_s: 86400.0,
                        valid: true,
                        fit_artifact_ids: vec!["calibration_grid".into()],
                    })
                    .collect(),
            },
            references: [
                Vec3::new(-0.008, -0.008, -0.008),
                Vec3::new(0.008, -0.006, 0.008),
                Vec3::new(0.001, 0.008, -0.007),
                Vec3::new(-0.008, 0.006, 0.006),
            ]
            .iter()
            .enumerate()
            .map(|(i, &p)| ReferenceFeature {
                artifact_id: "online_reference_cage".into(),
                feature_id: i as u32 + 1,
                point_tube_m: p,
                characterized_rms_m: 0.5e-6,
            })
            .collect(),
            thermal: ThermalState {
                structure_temperature_k: 293.15,
                tube_temperature_k: 293.15,
                reference_temperature_k: 293.15,
                structure_expansion_per_k: 23e-6,
                reference_expansion_per_k: 0.5e-6,
                anchor_world_m: Vec3::ZERO,
                maximum_calibrated_delta_k: 2.0,
            },
            errors: ErrorBudget {
                localization_sigma_px: 0.075,
                tracking_localization_sigma_px: 1.8,
                photon_localization_coefficient_px: 2.0,
                camera_calibration_rms_m: 1.0e-6,
                projector_calibration_rms_m: 1.0e-6,
                distortion_residual_rms_m: 0.5e-6,
                structural_drift_rms_m: 0.7e-6,
                residual_thermal_rms_m: 1e-6,
                surface_interaction_rms_m: 1e-6,
                reconstruction_rms_m: 0.8e-6,
                fiducial_manufacturing_rms_m: 0.5e-6,
                tcp_calibration_rms_m: 1e-6,
                tcp_rotation_rms_rad: 50e-6,
                correspondence_sigma_px: 0.03,
                phase_intensity_sigma: 0.003,
            },
            acquisition: AcquisitionConfig {
                tracking_rate_hz: 60.0,
                exposure_s: 0.0002,
                trigger_sigma_s: 1e-6,
                max_trigger_skew_s: 5e-6,
                processing_latency_s: 0.005,
                max_exposure_motion_m: 1e-6,
                max_sequence_motion_m: 0.5e-6,
            },
            patterns: PatternConfig::default(),
            illumination: Illumination {
                kind: IlluminationKind::Diffuse,
                wavelength_m: 450e-9,
                polarization: Polarization::None,
                source_id: 101,
            },
            minimum_angle_rad: 10f64.to_radians(),
            maximum_normalized_residual: 6.0,
            seed: 41,
        }
    }
}
pub fn look_at(eye: Vec3, target: Vec3) -> RigidTransform {
    let forward = (target - eye).normalized().unwrap_or(Vec3::Z);
    let up = if forward.cross(Vec3::Z).norm() > 0.1 {
        Vec3::Z
    } else {
        Vec3::Y
    };
    let right = forward.cross(up).normalized().unwrap();
    let down = forward.cross(right);
    RigidTransform::new(
        Mat3::new([
            [right.x, down.x, forward.x],
            [right.y, down.y, forward.y],
            [right.z, down.z, forward.z],
        ]),
        eye,
    )
}
fn baseline_device(id: u32, kind: DeviceKind, eye: Vec3, target: Vec3) -> OpticalDevice {
    let distance = (eye - target).norm();
    let pitch = 3e-6;
    let size = if kind == DeviceKind::Camera {
        ImageSize::new(4096, 3000)
    } else {
        ImageSize::new(1920, 1200)
    };
    let sampling = if kind == DeviceKind::Camera {
        9e-6
    } else {
        25e-6
    };
    let focal = distance / sampling * pitch;
    OpticalDevice {
        id,
        kind,
        model: PinholeCamera::new(
            size,
            CameraIntrinsics::new(
                focal / pitch,
                focal / pitch,
                (size.width as f64 - 1.0) * 0.5,
                (size.height as f64 - 1.0) * 0.5,
            ),
            BrownConrady {
                k1: -0.025,
                k2: 0.003,
                k3: 0.0,
                p1: 0.0002,
                p2: -0.0001,
            },
            look_at(eye, target),
        ),
        lens_model: LensModel::Perspective,
        pixel_pitch_m: [pitch; 2],
        effective_focal_length_m: focal,
        nominal_lens_focal_length_m: 1.0 / (1.0 / focal + 1.0 / distance),
        focus_working_distance_m: distance,
        usable_depth_of_field_m: 0.030,
        aperture_diameter_m: 0.002,
        housing_radius_m: 0.008,
        bit_depth: 12,
        monochrome: true,
        global_shutter: true,
        raw_access: true,
        hardware_trigger: true,
        deterministic_exposure: true,
        focus_locked: true,
        autofocus: false,
        electronic_stabilization: false,
        temperature_k: 293.15,
        focal_fraction_per_k: 2e-6,
    }
}
pub(crate) fn valid_transform(t: RigidTransform) -> bool {
    if !t.translation.is_finite() || !t.rotation.m.iter().flatten().all(|x| x.is_finite()) {
        return false;
    }
    let q = t.rotation.transpose() * t.rotation - Mat3::IDENTITY;
    q.m.iter().flatten().all(|x| x.abs() < 1e-7) && (t.rotation.determinant() - 1.0).abs() < 1e-7
}
