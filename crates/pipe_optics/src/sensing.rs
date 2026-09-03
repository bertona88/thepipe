use crate::camera::{CalibratedCamera, DriftRandomWalk};
use crate::math::{Vec2, Vec3};
use crate::noise::{keyed_seed, DeterministicRng};
use crate::precision::random_triangulation_precision;
use crate::reconstruction::{triangulate_rays, Covariance3, QualityMetrics};
use crate::scene::{Hit, Scene};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScanConfig {
    /// Grid decimation for full-frame scans. 1 traces every detector pixel.
    pub pixel_stride: u32,
    /// Electronics/centroid floor before photon-dependent noise.
    pub camera_pixel_sigma_floor_px: f64,
    pub projector_pixel_sigma_floor_px: f64,
    /// Coefficient divided by sqrt(photoelectrons) and added in quadrature.
    pub photon_centroid_coefficient_px: f64,
    /// Detector/pattern coordinate quantization. Zero disables it.
    pub camera_pixel_quantization_px: f64,
    pub projector_pixel_quantization_px: f64,
    /// Output range quantization. Zero disables it.
    pub depth_quantization_m: f64,
    /// Residual speckle/surface-model axial floor after triangulation.
    pub speckle_axial_sigma_m: f64,
    /// Photoelectrons from a white Lambertian target, normal to the projector,
    /// at `reference_range_m`.
    pub reference_photoelectrons: f64,
    pub reference_range_m: f64,
    pub ambient_photoelectrons: f64,
    pub read_noise_electrons: f64,
    pub minimum_signal_to_noise: f64,
    pub base_dropout_probability: f64,
    pub grazing_dropout_probability: f64,
    pub minimum_triangulation_angle_rad: f64,
    pub maximum_ray_separation_m: f64,
    pub occlusion_epsilon_m: f64,
    pub fiducial_corner_sigma_floor_px: f64,
    pub minimum_fiducial_corners: u8,
}

impl Default for ScanConfig {
    fn default() -> Self {
        Self {
            pixel_stride: 4,
            camera_pixel_sigma_floor_px: 0.06,
            projector_pixel_sigma_floor_px: 0.10,
            photon_centroid_coefficient_px: 4.0,
            camera_pixel_quantization_px: 1.0 / 16.0,
            projector_pixel_quantization_px: 1.0 / 8.0,
            depth_quantization_m: 0.5e-6,
            // Speckle and imperfect polymer/metal reflectance dominate optimistic
            // pinhole math at the 1-10 mm working volume.
            speckle_axial_sigma_m: 1.5e-6,
            reference_photoelectrons: 4_000.0,
            reference_range_m: 0.030,
            ambient_photoelectrons: 250.0,
            read_noise_electrons: 6.0,
            minimum_signal_to_noise: 3.0,
            base_dropout_probability: 0.01,
            grazing_dropout_probability: 0.25,
            minimum_triangulation_angle_rad: 1.0_f64.to_radians(),
            maximum_ray_separation_m: 30.0e-6,
            occlusion_epsilon_m: 0.5e-6,
            fiducial_corner_sigma_floor_px: 0.08,
            minimum_fiducial_corners: 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissingReturn {
    OutsideImage,
    InvalidCalibration,
    NoSurface,
    ProjectorOutOfView,
    ProjectorOccluded,
    LowSignal,
    StochasticDropout,
    DegenerateGeometry,
    BehindSensor,
    ExcessiveRayResidual,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthSample {
    pub camera_id: u32,
    /// Detector coordinate requested before centroid noise.
    pub detector_pixel: Vec2,
    /// Quantized/noisy coordinate used by reconstruction.
    pub observed_camera_pixel: Vec2,
    pub observed_projector_pixel: Vec2,
    pub true_point: Vec3,
    pub measured_point: Vec3,
    pub true_range_m: f64,
    pub measured_range_m: f64,
    pub range_error_m: f64,
    pub primitive_index: usize,
    pub primitive_tag: u32,
    pub signal_photoelectrons: f64,
    pub covariance: Covariance3,
    pub quality: QualityMetrics,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum DepthReturn {
    Sample(DepthSample),
    Missing {
        camera_id: u32,
        detector_pixel: Vec2,
        reason: MissingReturn,
    },
}

impl DepthReturn {
    pub fn missing_reason(self) -> Option<MissingReturn> {
        match self {
            Self::Sample(_) => None,
            Self::Missing { reason, .. } => Some(reason),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ScanStats {
    pub attempted: u64,
    pub valid: u64,
    pub no_surface: u64,
    pub projector_coverage_or_occlusion: u64,
    pub low_signal_or_dropout: u64,
    pub geometric_rejection: u64,
    pub invalid: u64,
}

impl ScanStats {
    fn record(&mut self, result: &DepthReturn) {
        self.attempted += 1;
        match result {
            DepthReturn::Sample(_) => self.valid += 1,
            DepthReturn::Missing { reason, .. } => match reason {
                MissingReturn::NoSurface => self.no_surface += 1,
                MissingReturn::ProjectorOutOfView | MissingReturn::ProjectorOccluded => {
                    self.projector_coverage_or_occlusion += 1
                }
                MissingReturn::LowSignal | MissingReturn::StochasticDropout => {
                    self.low_signal_or_dropout += 1
                }
                MissingReturn::DegenerateGeometry
                | MissingReturn::BehindSensor
                | MissingReturn::ExcessiveRayResidual => self.geometric_rejection += 1,
                MissingReturn::OutsideImage | MissingReturn::InvalidCalibration => {
                    self.invalid += 1
                }
            },
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScanFrame {
    pub frame_index: u64,
    /// Includes missing pixels, making failure patterns directly inspectable.
    pub returns: Vec<DepthReturn>,
    pub stats: ScanStats,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fiducial {
    pub id: u32,
    /// Clockwise or counter-clockwise corners in world coordinates.
    pub corners_world: [Vec3; 4],
    /// Front direction; markers are one-sided.
    pub normal_world: Vec3,
    /// Relative printed contrast/retroreflective response, nominally 0..1.
    pub contrast: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiducialObservation {
    pub camera_id: u32,
    pub fiducial_id: u32,
    pub corners_px: [Option<Vec2>; 4],
    pub visible_corner_count: u8,
    pub detected: bool,
    /// Isotropic per-corner covariance in square pixels.
    pub corner_variance_px2: f64,
    pub mean_signal_to_noise: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StructuredLightRig {
    pub cameras: Vec<CalibratedCamera>,
    pub projector: CalibratedCamera,
    pub config: ScanConfig,
    pub seed: u64,
}

impl StructuredLightRig {
    pub fn new(
        cameras: Vec<CalibratedCamera>,
        projector: CalibratedCamera,
        config: ScanConfig,
        seed: u64,
    ) -> Self {
        Self {
            cameras,
            projector,
            config,
            seed,
        }
    }

    /// Advance physical-vs-calibrated drift for all optics.
    pub fn advance_calibration_drift(&mut self, model: DriftRandomWalk, frame_index: u64) {
        for camera in &mut self.cameras {
            model.advance(&mut camera.drift, self.seed, camera.id, frame_index);
        }
        model.advance(
            &mut self.projector.drift,
            self.seed,
            self.projector.id,
            frame_index,
        );
    }

    pub fn scan(&self, scene: &Scene, frame_index: u64) -> ScanFrame {
        let mut returns = Vec::new();
        let mut stats = ScanStats::default();
        let stride = self.config.pixel_stride.max(1) as usize;
        for camera_index in 0..self.cameras.len() {
            let image = self.cameras[camera_index].nominal.image_size;
            for y in (0..image.height).step_by(stride) {
                for x in (0..image.width).step_by(stride) {
                    let result = self.sample_pixel(scene, frame_index, camera_index, x, y);
                    stats.record(&result);
                    returns.push(result);
                }
            }
        }
        ScanFrame {
            frame_index,
            returns,
            stats,
        }
    }

    /// Trace and reconstruct one detector pixel, including all visibility failures.
    pub fn sample_pixel(
        &self,
        scene: &Scene,
        frame_index: u64,
        camera_index: usize,
        x: u32,
        y: u32,
    ) -> DepthReturn {
        let Some(camera) = self.cameras.get(camera_index).copied() else {
            return missing(
                0,
                Vec2::new(x as f64, y as f64),
                MissingReturn::InvalidCalibration,
            );
        };
        let detector_pixel = Vec2::new(x as f64, y as f64);
        let actual_camera = camera.actual();
        let actual_projector = self.projector.actual();
        if !actual_camera.image_size.contains(detector_pixel) {
            return missing(camera.id, detector_pixel, MissingReturn::OutsideImage);
        }
        let Some(physical_camera_ray) = actual_camera.ray(detector_pixel) else {
            return missing(camera.id, detector_pixel, MissingReturn::InvalidCalibration);
        };
        let Some(hit) = scene.intersect(
            physical_camera_ray,
            actual_camera.near_m,
            actual_camera.far_m,
        ) else {
            return missing(camera.id, detector_pixel, MissingReturn::NoSurface);
        };
        let Some(projector_projection) = actual_projector.project(hit.position) else {
            return missing(camera.id, detector_pixel, MissingReturn::ProjectorOutOfView);
        };
        if scene.occluded(
            actual_projector.center_world(),
            hit.position,
            self.config.occlusion_epsilon_m.max(0.0),
        ) {
            return missing(camera.id, detector_pixel, MissingReturn::ProjectorOccluded);
        }

        let (signal, snr, incidence) = self.surface_signal(
            hit,
            actual_camera.center_world(),
            actual_projector.center_world(),
        );
        if !snr.is_finite() || snr < self.config.minimum_signal_to_noise.max(0.0) {
            return missing(camera.id, detector_pixel, MissingReturn::LowSignal);
        }

        let mut rng = DeterministicRng::new(keyed_seed(
            self.seed,
            &[
                0x5ca1_0001,
                frame_index,
                camera.id as u64,
                x as u64,
                y as u64,
            ],
        ));
        let dropout_probability = (self.config.base_dropout_probability
            + self.config.grazing_dropout_probability * (1.0 - incidence))
            .clamp(0.0, 1.0);
        if rng.uniform() < dropout_probability {
            return missing(camera.id, detector_pixel, MissingReturn::StochasticDropout);
        }

        let photon_sigma =
            self.config.photon_centroid_coefficient_px.max(0.0) / signal.max(1.0).sqrt();
        let camera_sigma_px = photon_sigma.hypot(self.config.camera_pixel_sigma_floor_px.max(0.0));
        let projector_sigma_px =
            photon_sigma.hypot(self.config.projector_pixel_sigma_floor_px.max(0.0));
        let observed_camera_pixel = Vec2::new(
            quantize(
                detector_pixel.x + rng.normal() * camera_sigma_px,
                self.config.camera_pixel_quantization_px,
            ),
            quantize(
                detector_pixel.y + rng.normal() * camera_sigma_px,
                self.config.camera_pixel_quantization_px,
            ),
        );
        let observed_projector_pixel = Vec2::new(
            quantize(
                projector_projection.pixel.x + rng.normal() * projector_sigma_px,
                self.config.projector_pixel_quantization_px,
            ),
            quantize(
                projector_projection.pixel.y + rng.normal() * projector_sigma_px,
                self.config.projector_pixel_quantization_px,
            ),
        );
        let (Some(camera_ray), Some(projector_ray)) = (
            camera.nominal.ray(observed_camera_pixel),
            self.projector.nominal.ray(observed_projector_pixel),
        ) else {
            return missing(camera.id, detector_pixel, MissingReturn::InvalidCalibration);
        };
        let Some(triangulation) = triangulate_rays(camera_ray, projector_ray) else {
            return missing(camera.id, detector_pixel, MissingReturn::DegenerateGeometry);
        };
        if triangulation.intersection_angle_rad < self.config.minimum_triangulation_angle_rad {
            return missing(camera.id, detector_pixel, MissingReturn::DegenerateGeometry);
        }
        if triangulation.first_distance_m <= 0.0 || triangulation.second_distance_m <= 0.0 {
            return missing(camera.id, detector_pixel, MissingReturn::BehindSensor);
        }
        if triangulation.ray_separation_m > self.config.maximum_ray_separation_m.max(0.0) {
            return missing(
                camera.id,
                detector_pixel,
                MissingReturn::ExcessiveRayResidual,
            );
        }

        let speckle_sigma = self.config.speckle_axial_sigma_m.max(0.0) * (1.0 + 1.0 / snr.max(1.0));
        let measured_range_m = quantize(
            triangulation.first_distance_m + rng.normal() * speckle_sigma,
            self.config.depth_quantization_m,
        );
        if measured_range_m <= 0.0 {
            return missing(camera.id, detector_pixel, MissingReturn::BehindSensor);
        }
        let measured_point = camera_ray.at(measured_range_m);
        let focal_px = (camera.nominal.intrinsics.fx_px * camera.nominal.intrinsics.fy_px)
            .abs()
            .sqrt()
            .max(1.0);
        let Some(random_precision) = random_triangulation_precision(
            measured_range_m,
            focal_px,
            triangulation.intersection_angle_rad,
            camera_sigma_px,
            projector_sigma_px,
        ) else {
            return missing(camera.id, detector_pixel, MissingReturn::DegenerateGeometry);
        };
        let lateral_sigma_m = random_precision.lateral_sigma_m;
        let geometric_axial_sigma_m = random_precision.axial_sigma_m;
        let quantization_sigma_m = self.config.depth_quantization_m.max(0.0) / 12.0_f64.sqrt();
        let axial_sigma_m = geometric_axial_sigma_m
            .hypot(speckle_sigma)
            .hypot(quantization_sigma_m);
        let covariance =
            Covariance3::viewing_ray(camera_ray.direction, lateral_sigma_m, axial_sigma_m);
        let reprojection_error_px = reprojection_error(
            measured_point,
            camera.nominal,
            self.projector.nominal,
            observed_camera_pixel,
            observed_projector_pixel,
        );
        let geometry_score = (triangulation.intersection_angle_rad.sin() / 0.25).clamp(0.0, 1.0);
        let signal_score = (snr / (snr + 10.0)).clamp(0.0, 1.0);
        let residual_scale = (camera_sigma_px + projector_sigma_px).max(0.05);
        let residual_score = 1.0 / (1.0 + reprojection_error_px / residual_scale);
        let confidence = (geometry_score * signal_score * residual_score).clamp(0.0, 1.0);
        let quality = QualityMetrics {
            signal_to_noise: snr,
            triangulation_angle_rad: triangulation.intersection_angle_rad,
            ray_separation_m: triangulation.ray_separation_m,
            reprojection_error_px,
            estimated_axial_sigma_m: axial_sigma_m,
            condition_number: triangulation.condition_number,
            confidence,
        };
        let true_range_m = hit.distance_m;
        DepthReturn::Sample(DepthSample {
            camera_id: camera.id,
            detector_pixel,
            observed_camera_pixel,
            observed_projector_pixel,
            true_point: hit.position,
            measured_point,
            true_range_m,
            measured_range_m,
            range_error_m: measured_range_m - true_range_m,
            primitive_index: hit.primitive_index,
            primitive_tag: hit.tag,
            signal_photoelectrons: signal,
            covariance,
            quality,
        })
    }

    /// Observe one-sided four-corner markers from every camera. Occluded/missed
    /// corners remain `None`, so partial fiducials are represented honestly.
    pub fn observe_fiducials(
        &self,
        scene: &Scene,
        frame_index: u64,
        fiducials: &[Fiducial],
    ) -> Vec<FiducialObservation> {
        let mut observations = Vec::with_capacity(self.cameras.len() * fiducials.len());
        for camera in self.cameras.iter().copied() {
            let actual = camera.actual();
            for fiducial in fiducials.iter().copied() {
                let center = fiducial
                    .corners_world
                    .iter()
                    .copied()
                    .fold(Vec3::ZERO, |sum, p| sum + p)
                    / 4.0;
                let view = (actual.center_world() - center)
                    .normalized()
                    .unwrap_or(Vec3::ZERO);
                let facing = fiducial
                    .normal_world
                    .normalized()
                    .unwrap_or(Vec3::ZERO)
                    .dot(view)
                    .max(0.0);
                let contrast = fiducial.contrast.clamp(0.0, 1.0);
                let range = (center - actual.center_world()).norm().max(1.0e-9);
                let signal = self.config.reference_photoelectrons.max(0.0)
                    * contrast
                    * facing
                    * (self.config.reference_range_m.max(1.0e-9) / range).powi(2);
                let snr = photoelectron_snr(
                    signal,
                    self.config.ambient_photoelectrons,
                    self.config.read_noise_electrons,
                );
                let corner_sigma = self.config.fiducial_corner_sigma_floor_px.max(0.0).hypot(
                    self.config.photon_centroid_coefficient_px.max(0.0) / signal.max(1.0).sqrt(),
                );
                let mut corners_px = [None; 4];
                let mut visible_corner_count = 0_u8;
                if facing > 0.0 && snr >= self.config.minimum_signal_to_noise.max(0.0) {
                    for (corner_index, corner) in fiducial.corners_world.iter().copied().enumerate()
                    {
                        let Some(projected) = actual.project(corner) else {
                            continue;
                        };
                        if scene.occluded(
                            actual.center_world(),
                            corner,
                            self.config.occlusion_epsilon_m.max(0.0),
                        ) {
                            continue;
                        }
                        let mut rng = DeterministicRng::new(keyed_seed(
                            self.seed,
                            &[
                                0xf1d0_c1a1,
                                frame_index,
                                camera.id as u64,
                                fiducial.id as u64,
                                corner_index as u64,
                            ],
                        ));
                        let dropout = (self.config.base_dropout_probability
                            + self.config.grazing_dropout_probability * (1.0 - facing))
                            .clamp(0.0, 1.0);
                        if rng.uniform() < dropout {
                            continue;
                        }
                        corners_px[corner_index] = Some(Vec2::new(
                            quantize(
                                projected.pixel.x + rng.normal() * corner_sigma,
                                self.config.camera_pixel_quantization_px,
                            ),
                            quantize(
                                projected.pixel.y + rng.normal() * corner_sigma,
                                self.config.camera_pixel_quantization_px,
                            ),
                        ));
                        visible_corner_count += 1;
                    }
                }
                observations.push(FiducialObservation {
                    camera_id: camera.id,
                    fiducial_id: fiducial.id,
                    corners_px,
                    visible_corner_count,
                    detected: visible_corner_count >= self.config.minimum_fiducial_corners,
                    corner_variance_px2: corner_sigma * corner_sigma,
                    mean_signal_to_noise: snr,
                });
            }
        }
        observations
    }

    fn surface_signal(
        &self,
        hit: Hit,
        camera_center: Vec3,
        projector_center: Vec3,
    ) -> (f64, f64, f64) {
        let to_camera = (camera_center - hit.position)
            .normalized()
            .unwrap_or(Vec3::ZERO);
        let to_projector = (projector_center - hit.position)
            .normalized()
            .unwrap_or(Vec3::ZERO);
        let view_cosine = hit.normal.dot(to_camera).max(0.0);
        let illumination_cosine = hit.normal.dot(to_projector).max(0.0);
        let incidence = view_cosine.min(illumination_cosine);
        let range = (projector_center - hit.position).norm().max(1.0e-9);
        let baseline_angle = to_camera.dot(to_projector).clamp(-1.0, 1.0).acos();
        let retro_lobe = (-(baseline_angle / 0.08).powi(2)).exp();
        let retro_gain = 1.0 + (hit.material.retroreflective_gain.max(1.0) - 1.0) * retro_lobe;
        let signal = self.config.reference_photoelectrons.max(0.0)
            * hit.material.diffuse_reflectance.clamp(0.0, 1.0)
            * illumination_cosine
            * view_cosine.sqrt()
            * retro_gain
            * (self.config.reference_range_m.max(1.0e-9) / range).powi(2);
        let snr = photoelectron_snr(
            signal,
            self.config.ambient_photoelectrons,
            self.config.read_noise_electrons,
        );
        (signal, snr, incidence)
    }
}

fn photoelectron_snr(signal: f64, ambient: f64, read_noise: f64) -> f64 {
    let variance = signal.max(0.0) + ambient.max(0.0) + read_noise.max(0.0).powi(2);
    if variance <= 0.0 {
        0.0
    } else {
        signal.max(0.0) / variance.sqrt()
    }
}

fn quantize(value: f64, step: f64) -> f64 {
    if step > 0.0 && step.is_finite() {
        (value / step).round() * step
    } else {
        value
    }
}

fn missing(camera_id: u32, detector_pixel: Vec2, reason: MissingReturn) -> DepthReturn {
    DepthReturn::Missing {
        camera_id,
        detector_pixel,
        reason,
    }
}

fn reprojection_error(
    point: Vec3,
    camera: crate::camera::PinholeCamera,
    projector: crate::camera::PinholeCamera,
    camera_pixel: Vec2,
    projector_pixel: Vec2,
) -> f64 {
    let camera_error = camera
        .project(point)
        .map(|p| (p.pixel - camera_pixel).norm())
        .unwrap_or(f64::INFINITY);
    let projector_error = projector
        .project(point)
        .map(|p| (p.pixel - projector_pixel).norm())
        .unwrap_or(f64::INFINITY);
    (camera_error * camera_error + projector_error * projector_error).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{BrownConrady, CameraIntrinsics, ImageSize, PinholeCamera};
    use crate::math::RigidTransform;
    use crate::scene::{Aabb, Geometry, Material, Primitive};

    fn optical_device(id: u32, x_m: f64) -> CalibratedCamera {
        let mut camera = PinholeCamera::new(
            ImageSize::new(160, 120),
            CameraIntrinsics::new(300.0, 300.0, 79.5, 59.5),
            BrownConrady::NONE,
            RigidTransform::new(crate::Mat3::IDENTITY, Vec3::new(x_m, 0.0, 0.0)),
        );
        camera.near_m = 0.001;
        camera.far_m = 0.2;
        CalibratedCamera::new(id, camera)
    }

    fn target_scene() -> Scene {
        Scene::new(vec![Primitive::new(
            Geometry::Aabb(Aabb {
                min: Vec3::new(-0.012, -0.010, 0.050),
                max: Vec3::new(0.012, 0.010, 0.051),
            }),
            Material {
                diffuse_reflectance: 0.7,
                ..Material::default()
            },
            99,
        )])
    }

    fn rig(seed: u64) -> StructuredLightRig {
        let config = ScanConfig {
            base_dropout_probability: 0.0,
            grazing_dropout_probability: 0.0,
            maximum_ray_separation_m: 200.0e-6,
            ..ScanConfig::default()
        };
        StructuredLightRig::new(
            vec![optical_device(1, 0.0)],
            optical_device(10, 0.010),
            config,
            seed,
        )
    }

    fn macro_device(id: u32, x_m: f64) -> CalibratedCamera {
        let mut camera = PinholeCamera::new(
            ImageSize::new(1280, 800),
            // About a 12 mm horizontal field at 20 mm working distance.
            CameraIntrinsics::new(2_133.0, 2_133.0, 639.5, 399.5),
            BrownConrady::NONE,
            RigidTransform::new(crate::Mat3::IDENTITY, Vec3::new(x_m, 0.0, 0.0)),
        );
        camera.near_m = 0.005;
        camera.far_m = 0.040;
        CalibratedCamera::new(id, camera)
    }

    #[test]
    fn structured_light_reports_when_coarse_optics_are_not_micron_metrology() {
        let result = rig(42).sample_pixel(&target_scene(), 7, 0, 80, 60);
        let DepthReturn::Sample(sample) = result else {
            panic!("unexpected missing return: {result:?}");
        };
        assert_eq!(sample.primitive_tag, 99);
        assert!(sample.measured_point.z > 0.049 && sample.measured_point.z < 0.051);
        assert!(sample.quality.triangulation_angle_rad > 0.1);
        assert!(sample.quality.estimated_axial_sigma_m > 0.0);
        // This deliberately low-resolution 160x120 rig cannot honestly verify
        // single-digit-micron fits: its reported depth sigma is tens/hundreds of um.
        assert!(sample.quality.estimated_axial_sigma_m > 20.0e-6);
        assert!(sample.quality.estimated_axial_sigma_m < 500.0e-6);
    }

    #[test]
    fn same_seed_and_frame_are_bit_repeatable() {
        let a = rig(123).sample_pixel(&target_scene(), 88, 0, 80, 60);
        let b = rig(123).sample_pixel(&target_scene(), 88, 0, 80, 60);
        assert_eq!(a, b);
    }

    #[test]
    fn gearbox_macro_geometry_can_reach_fifteen_micron_depth_sigma() {
        let scene = Scene::new(vec![Primitive::new(
            Geometry::Aabb(Aabb {
                min: Vec3::new(-0.006, -0.004, 0.020),
                max: Vec3::new(0.006, 0.004, 0.0205),
            }),
            Material::default(),
            8,
        )]);
        let config = ScanConfig {
            base_dropout_probability: 0.0,
            grazing_dropout_probability: 0.0,
            ..ScanConfig::default()
        };
        let rig = StructuredLightRig::new(
            vec![macro_device(1, 0.0)],
            macro_device(2, 0.005),
            config,
            55,
        );
        let result = rig.sample_pixel(&scene, 0, 0, 640, 400);
        let DepthReturn::Sample(sample) = result else {
            panic!("unexpected missing return: {result:?}");
        };
        assert!(sample.quality.estimated_axial_sigma_m < 15.0e-6);
        assert!(sample.quality.triangulation_angle_rad > 0.20);
    }

    #[test]
    fn co_located_projector_reports_unobservable_depth() {
        let mut rig = rig(0);
        rig.projector = rig.cameras[0];
        let result = rig.sample_pixel(&target_scene(), 0, 0, 80, 60);
        assert_eq!(
            result.missing_reason(),
            Some(MissingReturn::DegenerateGeometry)
        );
    }

    #[test]
    fn projector_shadow_is_not_mistaken_for_depth() {
        let mut scene = target_scene();
        scene.push(Primitive::new(
            Geometry::Sphere(crate::scene::Sphere {
                center: Vec3::new(0.005, 0.0, 0.025),
                radius_m: 0.002,
            }),
            Material::default(),
            4,
        ));
        let result = rig(0).sample_pixel(&scene, 0, 0, 80, 60);
        assert_eq!(
            result.missing_reason(),
            Some(MissingReturn::ProjectorOccluded)
        );
    }

    #[test]
    fn fiducial_visibility_and_noise_are_reported() {
        let marker = Fiducial {
            id: 5,
            corners_world: [
                Vec3::new(-0.002, -0.002, 0.0499),
                Vec3::new(0.002, -0.002, 0.0499),
                Vec3::new(0.002, 0.002, 0.0499),
                Vec3::new(-0.002, 0.002, 0.0499),
            ],
            normal_world: -Vec3::Z,
            contrast: 0.9,
        };
        let observations = rig(90).observe_fiducials(&target_scene(), 1, &[marker]);
        assert_eq!(observations.len(), 1);
        assert!(observations[0].detected);
        assert_eq!(observations[0].visible_corner_count, 4);
        assert!(observations[0].corner_variance_px2 > 0.0);
    }
}
