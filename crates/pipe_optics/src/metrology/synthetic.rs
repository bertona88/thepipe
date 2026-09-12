//! Reduced synthetic optical frontend. Ray visibility uses supplied machine
//! geometry. It simulates extracted image features and encoded intensities, not
//! images, diffraction PSFs, lens MTF, a BRDF renderer or hardware electronics.
//! Truth, actual calibration and surface geometry never cross into `solve`.
use super::*;
use crate::noise::{keyed_seed, DeterministicRng};
use crate::{Mat3, PinholeCamera, RigidTransform, Scene, Vec2, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SurfaceResponse {
    /// Diffuse reflectance at 450, 650, 850 nm. Linear interpolation is a
    /// reduced spectral model, not an assumption that blue suits all materials.
    pub diffuse_at_450_650_850nm: [f64; 3],
    pub specular_fraction: f64,
    pub translucent_fraction: f64,
    pub interreflection_fraction: f64,
    pub texture_contrast: f64,
    /// Characteristic precision feature width in object space.
    pub feature_width_m: f64,
    pub reference_photoelectrons: f64,
    pub ambient_photoelectrons: f64,
    pub read_noise_electrons: f64,
    pub full_well_electrons: f64,
}
impl Default for SurfaceResponse {
    fn default() -> Self {
        Self {
            diffuse_at_450_650_850nm: [0.7, 0.65, 0.55],
            specular_fraction: 0.02,
            translucent_fraction: 0.0,
            interreflection_fraction: 0.0,
            texture_contrast: 1.0,
            feature_width_m: 0.0003,
            reference_photoelectrons: 20000.0,
            ambient_photoelectrons: 100.0,
            read_noise_electrons: 4.0,
            full_well_electrons: 30000.0,
        }
    }
}
impl SurfaceResponse {
    fn signal(
        &self,
        illumination: &Illumination,
        exposure_s: f64,
        incidence: f64,
    ) -> Result<(f64, f64), MetrologyError> {
        if !self
            .diffuse_at_450_650_850nm
            .iter()
            .chain([
                &self.specular_fraction,
                &self.translucent_fraction,
                &self.interreflection_fraction,
                &self.texture_contrast,
            ])
            .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
            || !positive(self.reference_photoelectrons)
            || !positive(self.full_well_electrons)
            || !nonnegative(self.ambient_photoelectrons)
            || !nonnegative(self.read_noise_electrons)
            || !positive(illumination.wavelength_m)
        {
            return Err(MetrologyError::InvalidObservation);
        }
        if !positive(self.feature_width_m) {
            return Err(MetrologyError::InvalidObservation);
        }
        if self.translucent_fraction > 0.05 || self.interreflection_fraction > 0.1 {
            return Err(MetrologyError::UnsupportedSurface);
        }
        if illumination.kind == IlluminationKind::Dark {
            return Err(MetrologyError::LowSignal);
        }
        if self.texture_contrast < 0.05
            && illumination.kind != IlluminationKind::Structured
            && illumination.kind != IlluminationKind::Backlight
        {
            return Err(MetrologyError::LowSignal);
        }
        let x = ((illumination.wavelength_m * 1e9 - 450.0) / 200.0).clamp(0.0, 2.0);
        let i = (x.floor() as usize).min(1);
        let f = x - i as f64;
        let diffuse =
            self.diffuse_at_450_650_850nm[i] * (1.0 - f) + self.diffuse_at_450_650_850nm[i + 1] * f;
        let (transmission, spec_scale) = match illumination.polarization {
            Polarization::None => (1.0, 1.0),
            Polarization::ParallelLinear => (0.45, 0.8),
            Polarization::CrossedLinear => (0.35, 0.15),
        };
        let specular = self.specular_fraction * spec_scale;
        if specular > 0.25 {
            return Err(MetrologyError::UnsupportedSurface);
        }
        let signal = self.reference_photoelectrons
            * (exposure_s / 0.0002)
            * transmission
            * diffuse
            * incidence.max(0.0);
        let total = signal + self.ambient_photoelectrons + self.reference_photoelectrons * specular;
        if total >= self.full_well_electrons {
            return Err(MetrologyError::Saturation);
        }
        let snr = signal / (total + self.read_noise_electrons.powi(2)).sqrt();
        if snr < 10.0 || incidence < 0.1 {
            return Err(MetrologyError::LowSignal);
        }
        Ok((signal, snr))
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct TruthFeature {
    pub object_id: u32,
    pub feature_id: u32,
    pub point_world_m: Vec3,
    pub normal_world: Option<Vec3>,
    pub velocity_world_m_s: Vec3,
    pub surface: SurfaceResponse,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VisibilityAttempt {
    pub object_id: u32,
    pub feature_id: u32,
    pub sensor_id: u32,
    pub rejection: Option<MetrologyError>,
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SyntheticAcquisition {
    pub observations: Vec<PixelObservation>,
    pub visibility: Vec<VisibilityAttempt>,
    pub patterns: Vec<PatternExposure>,
}
impl SyntheticAcquisition {
    pub fn occlusion_fraction(&self) -> f64 {
        if self.visibility.is_empty() {
            return 0.0;
        }
        self.visibility
            .iter()
            .filter(|v| v.rejection == Some(MetrologyError::Occluded))
            .count() as f64
            / self.visibility.len() as f64
    }
    pub fn usable_view_fraction(&self) -> f64 {
        if self.visibility.is_empty() {
            return 0.0;
        }
        self.visibility
            .iter()
            .filter(|v| v.rejection.is_none())
            .count() as f64
            / self.visibility.len() as f64
    }
}
fn normal3(seed: u64, keys: &[u64], rms: f64) -> Vec3 {
    let mut rng = DeterministicRng::new(keyed_seed(seed, keys));
    let s = rms / 3f64.sqrt();
    Vec3::new(rng.normal() * s, rng.normal() * s, rng.normal() * s)
}
/// Actual device calibration belongs only to the simulation sensor boundary.
/// Extrinsic, intrinsic/distortion, focal/thermal and shared structural terms
/// perturb projections before reconstruction uses the nominal calibration.
pub fn actual_device(config: &MetrologyConfig, device: &OpticalDevice) -> PinholeCamera {
    let mut actual = device.model;
    let e = &config.errors;
    let calibration_rms = if device.kind == DeviceKind::Camera {
        e.camera_calibration_rms_m
    } else {
        e.projector_calibration_rms_m
    };
    let sensor_shift = normal3(config.seed, &[11, device.id as u64], calibration_rms);
    let structural = normal3(config.seed, &[12], e.structural_drift_rms_m);
    let thermal = normal3(config.seed, &[13], e.residual_thermal_rms_m);
    actual.world_from_camera.translation = config.thermal.expanded(
        actual.center_world(),
        config.calibration.reference_temperature_k,
    ) + sensor_shift
        + structural
        + thermal;
    let mut rng = DeterministicRng::new(keyed_seed(config.seed, &[14, device.id as u64]));
    // Residual distortion coefficient normalized to a 0.2 normalized-image
    // radius at the working distance; effect is spatially varying, not a shift.
    actual.distortion.k1 += rng.normal() * e.distortion_residual_rms_m
        / (device.focus_working_distance_m * 0.2f64.powi(3));
    let scale = 1.0
        + device.focal_fraction_per_k
            * (device.temperature_k - config.calibration.reference_temperature_k);
    actual.intrinsics.fx_px *= scale;
    actual.intrinsics.fy_px *= scale;
    actual
}
fn biased_feature(config: &MetrologyConfig, feature: &TruthFeature, sequence: u64) -> Vec3 {
    feature.point_world_m
        + normal3(
            config.seed,
            &[15, feature.object_id as u64, sequence],
            config.errors.surface_interaction_rms_m,
        )
        + normal3(
            config.seed,
            &[16, feature.object_id as u64],
            config.errors.reconstruction_rms_m,
        )
}
/// Thin-lens defocus disk second moment plus Gaussian approximation to the
/// Airy central lobe. Returned in sensor pixels. This is a reduced optical
/// transfer model, not a measured MTF or a hard depth-of-field claim.
pub fn optical_blur_sigma_px(
    device: &OpticalDevice,
    depth_m: f64,
    wavelength_m: f64,
) -> Result<f64, MetrologyError> {
    let f = device.nominal_lens_focal_length_m;
    if depth_m <= f || !positive(wavelength_m) {
        return Err(MetrologyError::OutsideField);
    }
    let image_distance = f * depth_m / (depth_m - f);
    let defocus_diameter = device.aperture_diameter_m
        * (image_distance - device.effective_focal_length_m).abs()
        / image_distance;
    let airy_fwhm =
        1.028 * wavelength_m * device.effective_focal_length_m / device.aperture_diameter_m;
    Ok(
        ((defocus_diameter / 4.0).powi(2) + (airy_fwhm / 2.355).powi(2)).sqrt()
            / device.pixel_pitch_m[0],
    )
}
fn check_view(
    config: &MetrologyConfig,
    scene: &Scene,
    feature: &TruthFeature,
    device: &OpticalDevice,
    points: (Vec3, Vec3),
    illumination: &Illumination,
    exposure_s: f64,
) -> Result<(Vec2, f64, f64), MetrologyError> {
    let (point, physical_point) = points;
    let actual = actual_device(config, device);
    let projection = actual.project(point).ok_or(MetrologyError::OutsideField)?;
    if (projection.optical_depth_m - device.focus_working_distance_m).abs()
        > device.usable_depth_of_field_m * 0.5
    {
        return Err(MetrologyError::OutsideField);
    }
    // No self-tag exemption: target endpoints use a submicron ray tolerance;
    // the rest of the same tool remains a real occluder.
    if scene.occluded(actual.center_world(), physical_point, 0.1e-6) {
        return Err(MetrologyError::Occluded);
    }
    let incidence = if let Some(normal) = feature.normal_world {
        if !normal.is_finite() || (normal.norm() - 1.0).abs() > 1e-6 {
            return Err(MetrologyError::InvalidObservation);
        }
        normal
            .dot(
                (actual.center_world() - point)
                    .normalized()
                    .ok_or(MetrologyError::InvalidObservation)?,
            )
            .max(0.0)
    } else {
        1.0
    };
    let (signal, _) = feature
        .surface
        .signal(illumination, exposure_s, incidence)?;
    let blur = optical_blur_sigma_px(
        device,
        projection.optical_depth_m,
        illumination.wavelength_m,
    )?;
    let sampling = projection.optical_depth_m / device.model.intrinsics.fx_px;
    // Contrast loss when the blur disk becomes large relative to the feature.
    let contrast_scale =
        1.0 / (1.0 + (4.0 * blur * sampling / feature.surface.feature_width_m).powi(2));
    if contrast_scale < 0.02 {
        return Err(MetrologyError::LowSignal);
    }
    Ok((projection.pixel, signal * contrast_scale, blur))
}
pub fn acquire_passive(
    config: &MetrologyConfig,
    scene: &Scene,
    feature: &TruthFeature,
    mode: MeasurementMode,
    class: MeasurementClass,
    timing: &AcquisitionTiming,
) -> Result<SyntheticAcquisition, MetrologyError> {
    config.validate()?;
    timing.validate()?;
    if !feature.point_world_m.is_finite() || !feature.velocity_world_m_s.is_finite() {
        return Err(MetrologyError::InvalidObservation);
    }
    if feature.velocity_world_m_s.norm() > timing.speed_bound_m_s + 1e-12
        && timing.motion_evidence != MotionEvidence::Unknown
    {
        return Err(MetrologyError::MotionDuringSequence);
    }
    let mut out = SyntheticAcquisition::default();
    for device in config
        .devices
        .iter()
        .filter(|d| d.kind == DeviceKind::Camera)
    {
        let observed = (|| {
            let mut rng = DeterministicRng::new(keyed_seed(
                config.seed,
                &[
                    17,
                    device.id as u64,
                    feature.object_id as u64,
                    feature.feature_id as u64,
                    timing.sequence_id,
                ],
            ));
            let offset = rng.normal() * config.acquisition.trigger_sigma_s;
            let point = biased_feature(config, feature, timing.sequence_id)
                + feature.velocity_world_m_s * (timing.exposure_duration_s * 0.5 + offset);
            let (pixel, signal, optical_blur) = check_view(
                config,
                scene,
                feature,
                device,
                (
                    point,
                    feature.point_world_m
                        + feature.velocity_world_m_s * (timing.exposure_duration_s * 0.5 + offset),
                ),
                &config.illumination,
                timing.exposure_duration_s,
            )?;
            let base = if mode == MeasurementMode::Precision {
                config.errors.localization_sigma_px
            } else {
                config.errors.tracking_localization_sigma_px
            };
            let sampling = device
                .sampling_m_per_px(point)
                .ok_or(MetrologyError::OutsideField)?;
            let temporal_sigma = feature.velocity_world_m_s.norm()
                * config.acquisition.trigger_sigma_s
                / sampling[0];
            let blur_sigma = feature.velocity_world_m_s.norm() * timing.exposure_duration_s
                / (12f64.sqrt() * sampling[0]);
            let sigma = (base.powi(2)
                + config.errors.photon_localization_coefficient_px.powi(2) / signal
                + optical_blur.powi(2) / signal
                + temporal_sigma.powi(2)
                + blur_sigma.powi(2))
            .sqrt();
            let noisy = pixel + Vec2::new(rng.normal() * sigma, rng.normal() * sigma);
            let mut observed_timing = timing.clone();
            observed_timing.trigger_offset_s = offset;
            observed_timing.exposure_start_s += offset;
            observed_timing.frame_timestamp_s += offset;
            observed_timing.available_s = observed_timing
                .available_s
                .max(observed_timing.exposure_start_s + observed_timing.exposure_duration_s);
            if observed_timing.exposure_start_s < 0.0 {
                return Err(MetrologyError::IncompatibleTiming);
            }
            Ok(PixelObservation {
                object_id: feature.object_id,
                feature_id: feature.feature_id,
                sensor_id: device.id,
                pixel: noisy,
                sigma_px: Vec2::new(sigma, sigma),
                timing: observed_timing,
                mode,
                class,
                illumination: config.illumination.clone(),
                calibration_id: config.calibration.id.clone(),
                source: ObservationSource::SyntheticNoisyImageFeature,
                saturated: false,
                decoded_projector: None,
            })
        })();
        out.visibility.push(VisibilityAttempt {
            object_id: feature.object_id,
            feature_id: feature.feature_id,
            sensor_id: device.id,
            rejection: observed.as_ref().err().cloned(),
        });
        if let Ok(o) = observed {
            out.observations.push(o);
        }
    }
    Ok(out)
}

/// A single camera/projector head acquires a real multi-frame encoded sequence.
/// The returned projector coordinate is decoded from intensities, never copied
/// from a projected truth point. Camera pixels are localized at scan midpoint.
pub fn acquire_structured(
    config: &MetrologyConfig,
    scene: &Scene,
    feature: &TruthFeature,
    camera_id: u32,
    projector_id: u32,
    timing: &AcquisitionTiming,
) -> Result<SyntheticAcquisition, MetrologyError> {
    config.validate()?;
    timing.validate()?;
    let camera = config.device(camera_id)?;
    let projector = config.device(projector_id)?;
    if camera.kind != DeviceKind::Camera
        || projector.kind != DeviceKind::Projector
        || projector.model.image_size != config.patterns.projector_size
    {
        return Err(MetrologyError::InvalidConfiguration(
            "structured head or pattern dimensions invalid".into(),
        ));
    }
    let mut sl = config.clone();
    sl.illumination.kind = IlluminationKind::Structured;
    sl.illumination.source_id = projector_id;
    let duration = config.patterns.duration_s(timing.exposure_duration_s);
    let start = timing.exposure_start_s;
    let mut rng = DeterministicRng::new(keyed_seed(
        config.seed,
        &[
            21,
            camera_id as u64,
            projector_id as u64,
            feature.object_id as u64,
            feature.feature_id as u64,
            timing.sequence_id,
        ],
    ));
    let mut frames = Vec::new();
    for (i, pattern) in config.patterns.patterns().into_iter().enumerate() {
        let dt = i as f64 / config.patterns.frame_rate_hz;
        let p = biased_feature(config, feature, timing.sequence_id)
            + feature.velocity_world_m_s * (dt + timing.exposure_duration_s * 0.5);
        let (_, _, camera_blur) = check_view(
            &sl,
            scene,
            feature,
            camera,
            (
                p,
                feature.point_world_m
                    + feature.velocity_world_m_s * (dt + timing.exposure_duration_s * 0.5),
            ),
            &sl.illumination,
            timing.exposure_duration_s,
        )?;
        let (projector_pixel, signal, projector_blur) = check_view(
            &sl,
            scene,
            feature,
            projector,
            (
                p,
                feature.point_world_m
                    + feature.velocity_world_m_s * (dt + timing.exposure_duration_s * 0.5),
            ),
            &sl.illumination,
            timing.exposure_duration_s,
        )?;
        let response = (signal / 20000.0).clamp(0.0, 1.0);
        let camera_sampling = camera
            .sampling_m_per_px(p)
            .ok_or(MetrologyError::OutsideField)?[0];
        let projector_sampling = projector
            .sampling_m_per_px(p)
            .ok_or(MetrologyError::OutsideField)?[0];
        let combined_blur = (projector_blur.powi(2)
            + (camera_blur * camera_sampling / projector_sampling).powi(2))
        .sqrt();
        let mtf = (-0.5
            * (std::f64::consts::TAU * combined_blur / config.patterns.phase_period_px).powi(2))
        .exp();
        // Phase modulation is attenuated by both optical paths. Gray boundaries
        // are still idealized sharp classification; low contrast fails decode.
        let pattern_intensity = config.patterns.intensity(pattern, projector_pixel);
        let pattern_intensity = if matches!(pattern, Pattern::Phase { .. }) {
            0.5 + (pattern_intensity - 0.5) * mtf
        } else {
            pattern_intensity
        };
        let intensity = 0.05
            + 0.8 * response * pattern_intensity
            + rng.normal() * config.errors.phase_intensity_sigma;
        let mut t = timing.clone();
        t.trigger_id += i as u64;
        t.exposure_start_s = start + dt;
        t.frame_timestamp_s = t.exposure_start_s + t.exposure_duration_s * 0.5;
        t.available_s = start + duration + config.acquisition.processing_latency_s;
        frames.push(PatternExposure {
            pattern,
            timing: t,
            illumination: sl.illumination.clone(),
            intensity: intensity.max(0.0),
        });
    }
    let decoded = decode_hybrid(&sl, &frames)?;
    let mut mid = timing.clone();
    mid.exposure_start_s = start + (duration - timing.exposure_duration_s) * 0.5;
    mid.frame_timestamp_s = mid.exposure_start_s + mid.exposure_duration_s * 0.5;
    mid.available_s = decoded.available_s;
    let mut moved = feature.clone();
    moved.point_world_m +=
        feature.velocity_world_m_s * (duration - timing.exposure_duration_s) * 0.5;
    let passive = acquire_passive(
        &sl,
        scene,
        &moved,
        MeasurementMode::Precision,
        MeasurementClass::CriticalFeature,
        &mid,
    )?;
    let mut camera_obs = passive
        .observations
        .into_iter()
        .find(|o| o.sensor_id == camera_id)
        .ok_or(MetrologyError::Occluded)?;
    // Preserve the camera timestamp for the decoded ray's observation bundle;
    // full scan start/end remain attached and drive freshness and motion gates.
    mid = camera_obs.timing.clone();
    camera_obs.timing.available_s = decoded.available_s;
    let projector_obs = PixelObservation {
        object_id: feature.object_id,
        feature_id: feature.feature_id,
        sensor_id: projector_id,
        pixel: decoded.pixel,
        sigma_px: decoded.sigma_px,
        timing: mid,
        mode: MeasurementMode::Precision,
        class: MeasurementClass::CriticalFeature,
        illumination: sl.illumination,
        calibration_id: config.calibration.id.clone(),
        source: ObservationSource::SyntheticNoisyImageFeature,
        saturated: false,
        decoded_projector: Some(decoded),
    };
    Ok(SyntheticAcquisition {
        observations: vec![camera_obs, projector_obs],
        visibility: vec![
            VisibilityAttempt {
                object_id: feature.object_id,
                feature_id: feature.feature_id,
                sensor_id: camera_id,
                rejection: None,
            },
            VisibilityAttempt {
                object_id: feature.object_id,
                feature_id: feature.feature_id,
                sensor_id: projector_id,
                rejection: None,
            },
        ],
        patterns: frames,
    })
}
pub fn acquire_tool(
    config: &MetrologyConfig,
    scene: &Scene,
    model: &RigidFiducial,
    truth_world_from_tcp: RigidTransform,
    mode: MeasurementMode,
    timing: &AcquisitionTiming,
) -> Result<SyntheticAcquisition, MetrologyError> {
    acquire_fiducial(
        config,
        scene,
        model,
        truth_world_from_tcp,
        mode,
        timing,
        false,
    )
}

/// Direct printed marks stay on their declared substrate tangent planes.
/// Mount/manufacturing variation is tangential; normal substrate form error is
/// not simulated by this reduced frontend. Calibration covariance and independent
/// marker-to-feature characterization remain required. This does not exempt any
/// ray from occlusion by the marked body or change the endpoint tolerance.
pub fn acquire_surface_markers(
    config: &MetrologyConfig,
    scene: &Scene,
    model: &RigidFiducial,
    truth_world_from_tcp: RigidTransform,
    mode: MeasurementMode,
    timing: &AcquisitionTiming,
) -> Result<SyntheticAcquisition, MetrologyError> {
    if model.features.iter().any(|f| f.normal_fiducial.is_none()) {
        return Err(MetrologyError::InvalidConfiguration(
            "surface marks require substrate normals".into(),
        ));
    }
    acquire_fiducial(
        config,
        scene,
        model,
        truth_world_from_tcp,
        mode,
        timing,
        true,
    )
}

fn acquire_fiducial(
    config: &MetrologyConfig,
    scene: &Scene,
    model: &RigidFiducial,
    truth_world_from_tcp: RigidTransform,
    mode: MeasurementMode,
    timing: &AcquisitionTiming,
    surface_attached: bool,
) -> Result<SyntheticAcquisition, MetrologyError> {
    model.validate()?;
    if !valid_transform(truth_world_from_tcp) {
        return Err(MetrologyError::InvalidObservation);
    }
    let tcp_error = RigidTransform::new(
        Mat3::from_axis_angle(normal3(
            config.seed,
            &[31, model.object_id as u64],
            config.errors.tcp_rotation_rms_rad,
        )),
        normal3(
            config.seed,
            &[32, model.object_id as u64],
            config.errors.tcp_calibration_rms_m,
        ),
    );
    let actual_fiducial = truth_world_from_tcp
        .compose(model.tcp_from_fiducial)
        .compose(tcp_error);
    let mut out = SyntheticAcquisition::default();
    for feature in &model.features {
        let local = feature.point_fiducial_m
            + normal3(
                config.seed,
                &[33, model.object_id as u64, feature.id as u64],
                config.errors.fiducial_manufacturing_rms_m,
            );
        let nominal_fiducial = truth_world_from_tcp.compose(model.tcp_from_fiducial);
        let displaced = actual_fiducial.transform_point(local);
        let normal = if surface_attached {
            feature
                .normal_fiducial
                .map(|n| nominal_fiducial.transform_vector(n))
        } else {
            feature
                .normal_fiducial
                .map(|n| actual_fiducial.transform_vector(n))
        };
        let point_world_m = if surface_attached {
            let nominal = nominal_fiducial.transform_point(feature.point_fiducial_m);
            let n = normal.ok_or(MetrologyError::InvalidObservation)?;
            nominal + surface_tangent_displacement(displaced - nominal, n)
        } else {
            displaced
        };
        let truth = TruthFeature {
            object_id: model.object_id,
            feature_id: feature.id,
            point_world_m,
            normal_world: normal,
            velocity_world_m_s: Vec3::ZERO,
            surface: SurfaceResponse {
                feature_width_m: feature.diameter_m,
                ..SurfaceResponse::default()
            },
        };
        let a = acquire_passive(
            config,
            scene,
            &truth,
            mode,
            MeasurementClass::ToolPose,
            timing,
        )?;
        out.observations.extend(a.observations);
        out.visibility.extend(a.visibility);
    }
    Ok(out)
}
fn surface_tangent_displacement(displacement: Vec3, unit_normal: Vec3) -> Vec3 {
    displacement - unit_normal * displacement.dot(unit_normal)
}
pub fn timing(
    config: &MetrologyConfig,
    sequence_id: u64,
    start_s: f64,
    speed_bound_m_s: f64,
) -> AcquisitionTiming {
    AcquisitionTiming {
        sequence_id,
        trigger_id: sequence_id * 100,
        exposure_start_s: start_s,
        exposure_duration_s: config.acquisition.exposure_s,
        frame_timestamp_s: start_s + config.acquisition.exposure_s * 0.5,
        available_s: start_s
            + config.acquisition.exposure_s
            + config.acquisition.processing_latency_s,
        trigger_offset_s: 0.0,
        motion_evidence: if speed_bound_m_s == 0.0 {
            MotionEvidence::StoppedInterlock
        } else {
            MotionEvidence::IndependentOpticalBound
        },
        speed_bound_m_s,
    }
}
pub fn acquire_references(
    config: &MetrologyConfig,
    scene: &Scene,
    timing: &AcquisitionTiming,
) -> Result<Vec<ObservedPoint>, MetrologyError> {
    let mut points = Vec::new();
    for reference in &config.references {
        let expansion = 1.0
            + config.thermal.reference_expansion_per_k
                * (config.thermal.reference_temperature_k
                    - config.calibration.reference_temperature_k);
        let p = config
            .calibration
            .world_from_tube
            .transform_point(reference.point_tube_m * expansion)
            + normal3(
                config.seed,
                &[34, reference.feature_id as u64],
                reference.characterized_rms_m,
            );
        let feature = TruthFeature {
            object_id: REFERENCE_OBJECT_ID,
            feature_id: reference.feature_id,
            point_world_m: p,
            normal_world: None,
            velocity_world_m_s: Vec3::ZERO,
            surface: SurfaceResponse::default(),
        };
        let a = acquire_passive(
            config,
            scene,
            &feature,
            MeasurementMode::Precision,
            MeasurementClass::CriticalFeature,
            timing,
        )?;
        if let Ok(point) = reconstruct_point(config, &a.observations) {
            points.push(point);
        }
    }
    Ok(points)
}

#[cfg(test)]
mod surface_mark_tests {
    use super::*;
    use crate::{Geometry, Material, Primitive, Sphere, Triangle};

    #[test]
    fn surface_marks_remain_visible_only_from_the_unblocked_substrate_side() {
        let config = MetrologyConfig::baseline();
        let mut model = RigidFiducial::baseline(700);
        model.arm_id = None;
        model.tcp_from_fiducial = RigidTransform::new(Mat3::IDENTITY, Vec3::ZERO);
        for (index, feature) in model.features.iter_mut().enumerate() {
            feature.point_fiducial_m = Vec3::new(
                ((index % 3) as f64 - 1.) * 0.001,
                ((index / 3) as f64 - 0.5) * 0.001,
                0.,
            );
            feature.normal_fiducial = Some(Vec3::Z);
        }
        let mut scene = Scene::default();
        for (a, b, c) in [
            (
                Vec3::new(-0.01, -0.01, 0.),
                Vec3::new(0.01, -0.01, 0.),
                Vec3::new(0.01, 0.01, 0.),
            ),
            (
                Vec3::new(-0.01, -0.01, 0.),
                Vec3::new(0.01, 0.01, 0.),
                Vec3::new(-0.01, 0.01, 0.),
            ),
        ] {
            scene.push(Primitive::new(
                Geometry::Triangle(Triangle {
                    a,
                    b,
                    c,
                    double_sided: true,
                }),
                Material::default(),
                700,
            ));
        }
        let t = timing(&config, 1, 0.01, 0.);
        let visible = acquire_surface_markers(
            &config,
            &scene,
            &model,
            RigidTransform::new(Mat3::IDENTITY, Vec3::ZERO),
            MeasurementMode::Precision,
            &t,
        )
        .unwrap();
        assert!(visible.observations.len() >= 6);
        for attempt in visible.visibility.iter().filter(|v| v.rejection.is_none()) {
            assert!(
                config
                    .device(attempt.sensor_id)
                    .unwrap()
                    .model
                    .center_world()
                    .z
                    > 0.
            );
        }
        scene.push(Primitive::new(
            Geometry::Sphere(Sphere {
                center: Vec3::ZERO,
                radius_m: 0.005,
            }),
            Material::default(),
            701,
        ));
        let blocked = acquire_surface_markers(
            &config,
            &scene,
            &model,
            RigidTransform::new(Mat3::IDENTITY, Vec3::ZERO),
            MeasurementMode::Precision,
            &t,
        )
        .unwrap();
        assert!(
            blocked.observations.is_empty(),
            "surface mode must not exempt occluders"
        );
        model.features[0].normal_fiducial = None;
        assert!(acquire_surface_markers(
            &config,
            &scene,
            &model,
            RigidTransform::new(Mat3::IDENTITY, Vec3::ZERO),
            MeasurementMode::Precision,
            &t
        )
        .is_err());
    }
}
