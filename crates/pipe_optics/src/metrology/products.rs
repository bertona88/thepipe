//! Secondary surface/occupancy products. Sparse surface scans share one real
//! pattern sequence; voxel visual hulls consume silhouettes, not plant geometry.
use super::synthetic::*;
use super::*;
use crate::{Scene, Vec2, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SurfaceScan {
    pub sequence_id: u64,
    pub sequence_duration_s: f64,
    pub pixel_stride: u32,
    pub attempted: usize,
    pub observed_points: Vec<ObservedPoint>,
    pub rejections: Vec<MetrologyError>,
}
/// Synthetic full-surface acquisition at a configurable detector stride.
/// Each sampled detector location runs intensity decode plus calibrated rays;
/// truth intersections only generate optical input and never become estimates.
pub fn acquire_surface_scan(
    config: &MetrologyConfig,
    scene: &Scene,
    camera_id: u32,
    projector_id: u32,
    pixel_stride: u32,
    acquisition: &AcquisitionTiming,
    surface: &SurfaceResponse,
) -> Result<SurfaceScan, MetrologyError> {
    config.validate()?;
    if pixel_stride == 0 {
        return Err(MetrologyError::InvalidConfiguration(
            "zero surface scan stride".into(),
        ));
    }
    let camera = config.device(camera_id)?;
    let size = camera.model.image_size;
    if (size.width.div_ceil(pixel_stride) as u64) * (size.height.div_ceil(pixel_stride) as u64)
        > 100_000
    {
        return Err(MetrologyError::InvalidConfiguration(
            "surface scan exceeds 100000 sampled pixels; increase stride".into(),
        ));
    }
    let actual = actual_device(config, camera);
    let mut scan = SurfaceScan {
        sequence_id: acquisition.sequence_id,
        sequence_duration_s: config.patterns.duration_s(acquisition.exposure_duration_s),
        pixel_stride,
        attempted: 0,
        observed_points: Vec::new(),
        rejections: Vec::new(),
    };
    for y in (0..size.height).step_by(pixel_stride as usize) {
        for x in (0..size.width).step_by(pixel_stride as usize) {
            scan.attempted += 1;
            let ray = actual
                .ray(Vec2::new(x as f64, y as f64))
                .ok_or(MetrologyError::InvalidObservation)?;
            let Some(hit) = scene.intersect(ray, actual.near_m, actual.far_m) else {
                scan.rejections.push(MetrologyError::LowSignal);
                continue;
            };
            let truth = TruthFeature {
                object_id: 3_000_000_000,
                feature_id: y * size.width + x,
                point_world_m: hit.position,
                normal_world: Some(hit.normal),
                velocity_world_m_s: Vec3::ZERO,
                surface: surface.clone(),
            };
            let estimate =
                acquire_structured(config, scene, &truth, camera_id, projector_id, acquisition)
                    .and_then(|mut a| {
                        for o in &mut a.observations {
                            o.class = MeasurementClass::DenseGeometry;
                        }
                        reconstruct_point(config, &a.observations)
                    });
            match estimate {
                Ok(point) => scan.observed_points.push(point),
                Err(e) => scan.rejections.push(e),
            }
        }
    }
    Ok(scan)
}

/// Binary silhouettes are an observation-only frontend boundary. Foreground
/// pixels use native detector coordinates, and must be segmented from opposing
/// backlight images. Unknown regions must not be encoded as background.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SilhouetteObservation {
    pub camera_id: u32,
    pub foreground_pixels: BTreeSet<(u32, u32)>,
    pub unknown_pixels: BTreeSet<(u32, u32)>,
    pub edge_uncertainty_px: f64,
    pub timing: AcquisitionTiming,
    pub illumination: Illumination,
    pub calibration_id: String,
    pub source: ObservationSource,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum OccupancyState {
    OccupiedOrOccluded,
    EmptyOutsideDilatedSilhouettes,
    Unknown,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OccupancyVoxel {
    pub center_world_m: Vec3,
    pub state: OccupancyState,
    pub usable_views: usize,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OccupancyVolume {
    pub class: MeasurementClass,
    pub region: PrecisionVolume,
    pub voxel_pitch_m: f64,
    pub voxel_half_diagonal_m: f64,
    pub supporting_silhouettes: Vec<SilhouetteSupport>,
    pub voxels: Vec<OccupancyVoxel>,
    pub acquisition_start_s: f64,
    pub available_s: f64,
    pub calibration_id: String,
    pub source: ObservationSource,
    pub fidelity: String,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SilhouetteSupport {
    pub camera_id: u32,
    pub edge_uncertainty_px: f64,
    pub timing: AcquisitionTiming,
}

/// Conservative visual hull of observed, dilated silhouettes. Hidden concavity
/// stays occupied. Grid spacing is NOT a validated occupancy accuracy claim.
/// This intentionally has no Scene/Simulation argument and cannot see truth.
pub fn reconstruct_occupancy(
    config: &MetrologyConfig,
    masks: &[SilhouetteObservation],
    region: PrecisionVolume,
    voxel_pitch_m: f64,
) -> Result<OccupancyVolume, MetrologyError> {
    config.validate()?;
    if !positive(voxel_pitch_m)
        || !region.center_world_m.is_finite()
        || ![region.size_m.x, region.size_m.y, region.size_m.z]
            .iter()
            .all(|&v| positive(v))
    {
        return Err(config_error("invalid occupancy grid"));
    }
    let counts = [region.size_m.x, region.size_m.y, region.size_m.z]
        .map(|v| (v / voxel_pitch_m).ceil() as usize);
    if counts
        .iter()
        .try_fold(1usize, |n, &c| n.checked_mul(c))
        .map_or(true, |n| n > 2_000_000)
    {
        return Err(config_error("occupancy request exceeds 2000000 voxels"));
    }
    let first = masks.first().ok_or(MetrologyError::InsufficientViews)?;
    let mut ids = BTreeSet::new();
    for mask in masks {
        mask.timing.validate()?;
        let device = config.device(mask.camera_id)?;
        if device.kind != DeviceKind::Camera
            || !ids.insert(mask.camera_id)
            || mask.calibration_id != config.calibration.id
            || !config.calibration.valid_at(mask.timing.available_s)
            || mask.source != first.source
            || !nonnegative(mask.edge_uncertainty_px)
            || mask.illumination.kind != IlluminationKind::Backlight
            || mask.timing.sequence_id != first.timing.sequence_id
            || (mask.timing.frame_timestamp_s - first.timing.frame_timestamp_s).abs()
                > config.acquisition.max_trigger_skew_s
            || mask
                .foreground_pixels
                .iter()
                .chain(mask.unknown_pixels.iter())
                .any(|&(x, y)| {
                    x >= device.model.image_size.width || y >= device.model.image_size.height
                })
        {
            return Err(MetrologyError::InvalidObservation);
        }
    }
    if masks.len() < 2 {
        return Err(MetrologyError::InsufficientViews);
    }
    let mut voxels = Vec::new();
    let minimum = region.center_world_m - region.size_m * 0.5;
    for z in 0..counts[2] {
        for y in 0..counts[1] {
            for x in 0..counts[0] {
                let p = minimum
                    + Vec3::new(x as f64 + 0.5, y as f64 + 0.5, z as f64 + 0.5) * voxel_pitch_m;
                let mut views = 0;
                let mut empty = false;
                for mask in masks {
                    let device = config.device(mask.camera_id)?;
                    let Some(pixel) = device.model.project(p) else {
                        continue;
                    };
                    let sampling = device
                        .sampling_m_per_px(p)
                        .ok_or(MetrologyError::OutsideField)?;
                    // Cover voxel half diagonal plus a three-sigma measured mask edge.
                    let radius = (0.5 * 3f64.sqrt() * voxel_pitch_m / sampling[0]
                        + 3.0 * mask.edge_uncertainty_px)
                        .ceil() as i64;
                    if radius > 128 {
                        return Err(config_error(
                            "occupancy dilation exceeds 128 pixels; use coarser image processing",
                        ));
                    }
                    let cx = pixel.pixel.x.round() as i64;
                    let cy = pixel.pixel.y.round() as i64;
                    let mut foreground = false;
                    let mut unknown = false;
                    for iy in cy - radius..=cy + radius {
                        for ix in cx - radius..=cx + radius {
                            if ix < 0
                                || iy < 0
                                || ix >= device.model.image_size.width as i64
                                || iy >= device.model.image_size.height as i64
                            {
                                unknown = true;
                                continue;
                            }
                            let key = (ix as u32, iy as u32);
                            foreground |= mask.foreground_pixels.contains(&key);
                            unknown |= mask.unknown_pixels.contains(&key);
                        }
                    }
                    if !unknown {
                        views += 1;
                        empty |= !foreground;
                    }
                }
                voxels.push(OccupancyVoxel {
                    center_world_m: p,
                    usable_views: views,
                    state: if views < 2 {
                        OccupancyState::Unknown
                    } else if empty {
                        OccupancyState::EmptyOutsideDilatedSilhouettes
                    } else {
                        OccupancyState::OccupiedOrOccluded
                    },
                });
            }
        }
    }
    Ok(OccupancyVolume {class:MeasurementClass::Occupancy,region,voxel_pitch_m,voxel_half_diagonal_m:3f64.sqrt()*0.5*voxel_pitch_m,
        supporting_silhouettes:masks.iter().map(|m|SilhouetteSupport {camera_id:m.camera_id,edge_uncertainty_px:m.edge_uncertainty_px,timing:m.timing.clone()}).collect(),voxels,acquisition_start_s:masks.iter().map(|m|m.timing.exposure_start_s).fold(f64::INFINITY,f64::min),
        available_s:masks.iter().map(|m|m.timing.available_s).fold(0.0,f64::max),calibration_id:config.calibration.id.clone(),source:first.source,
        fidelity:"Visual hull with measured edge uncertainty and voxel dilation; occluded concavities remain occupied. Segmentation errors and calibration drift require independent validation and planning safety margins.".into()})
}
