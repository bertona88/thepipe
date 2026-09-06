use pipe_optics::metrology::{synthetic::*, verification::*, *};
use pipe_optics::{BrownConrady, Geometry, Mat3, Primitive, RigidTransform, Scene, Sphere, Vec3};

fn feature(point: Vec3) -> TruthFeature {
    TruthFeature {
        object_id: 7,
        feature_id: 1,
        point_world_m: point,
        normal_world: None,
        velocity_world_m_s: Vec3::ZERO,
        surface: SurfaceResponse::default(),
    }
}
fn observe(c: &MetrologyConfig, p: Vec3, sequence: u64) -> SyntheticAcquisition {
    acquire_passive(
        c,
        &Scene::default(),
        &feature(p),
        MeasurementMode::Precision,
        MeasurementClass::CriticalFeature,
        &timing(c, sequence, 1.0, 0.0),
    )
    .unwrap()
}
fn random_only() -> MetrologyConfig {
    let mut c = MetrologyConfig::baseline();
    c.errors = ErrorBudget {
        localization_sigma_px: 0.075,
        tracking_localization_sigma_px: 1.0,
        photon_localization_coefficient_px: 0.0,
        camera_calibration_rms_m: 0.0,
        projector_calibration_rms_m: 0.0,
        distortion_residual_rms_m: 0.0,
        structural_drift_rms_m: 0.0,
        residual_thermal_rms_m: 0.0,
        surface_interaction_rms_m: 0.0,
        reconstruction_rms_m: 0.0,
        fiducial_manufacturing_rms_m: 0.0,
        tcp_calibration_rms_m: 0.0,
        tcp_rotation_rms_rad: 0.0,
        correspondence_sigma_px: 0.0,
        phase_intensity_sigma: 0.0,
    };
    c.acquisition.trigger_sigma_s = 0.0;
    c
}

#[test]
fn repeatable_noise_and_observation_order_do_not_change_solution() {
    let c = MetrologyConfig::baseline();
    let a = observe(&c, Vec3::new(0.002, -0.001, 0.003), 20);
    assert_eq!(a, observe(&c, Vec3::new(0.002, -0.001, 0.003), 20));
    let p = reconstruct_point(&c, &a.observations).unwrap();
    let mut reversed = a.observations.clone();
    reversed.reverse();
    assert_eq!(p, reconstruct_point(&c, &reversed).unwrap());
    assert_ne!(a, observe(&c, Vec3::new(0.002, -0.001, 0.003), 21));
    assert!(p.predicted_3d_rms_m() > 1e-6);
}

#[test]
fn depth_covariance_obeys_focal_baseline_scaling_and_is_consistent() {
    let mut c = random_only();
    c.devices.retain(|d| d.kind == DeviceKind::Camera);
    c.devices.truncate(2);
    for (i, d) in c.devices.iter_mut().enumerate() {
        d.model.world_from_camera = RigidTransform::new(
            Mat3::IDENTITY,
            Vec3::new(if i == 0 { -0.010 } else { 0.010 }, 0.0, -0.080),
        );
        d.focus_working_distance_m = 0.080;
        d.nominal_lens_focal_length_m = 1.0 / (1.0 / d.effective_focal_length_m + 1.0 / 0.080);
        d.usable_depth_of_field_m = 0.05;
        d.model.distortion = BrownConrady::NONE;
    }
    let observations = observe(&c, Vec3::ZERO, 1).observations;
    let p = reconstruct_point(&c, &observations).unwrap();
    let expected = 0.080f64.powi(2) / (c.devices[0].model.intrinsics.fx_px * 0.020)
        * 2f64.sqrt()
        * observations[0].sigma_px.x;
    assert!((p.covariance_m2[2][2].sqrt() / expected - 1.0).abs() < 0.01);
    let mut wider = c.clone();
    for d in &mut wider.devices {
        d.model.world_from_camera.translation.x *= 1.5;
    }
    let q = reconstruct_point(&wider, &observe(&wider, Vec3::ZERO, 1).observations).unwrap();
    assert!((p.covariance_m2[2][2] / q.covariance_m2[2][2] - 2.25).abs() < 0.05);
    let r = verify_volume(&random_only(), &Scene::default(), 24, false).unwrap();
    assert!(r.accepted > 500);
    assert!((r.mean_nees.unwrap() - 3.0).abs() < 0.7);
    assert!(r.fraction_inside_95_percent_3d_ellipsoid.unwrap() > 0.91);
}

#[test]
fn covariance_and_accuracy_can_fail_independently() {
    let c = MetrologyConfig::baseline();
    let baseline = verify_volume(&c, &Scene::default(), 4, false).unwrap();
    assert!(baseline.accepted > 50);
    let mut bad = c;
    bad.errors.localization_sigma_px = 4.0;
    let degraded = verify_volume(&bad, &Scene::default(), 4, false).unwrap();
    assert!(!degraded.local_target_met_in_simulation);
    assert!(degraded.position_3d_rms_m.unwrap() > 5e-6);
    assert!(degraded.position_3d_rms_m.unwrap() > baseline.position_3d_rms_m.unwrap() * 2.0);
}

#[test]
fn partial_feature_subsets_recover_six_dof_and_tcp_lever_arm() {
    let c = random_only();
    let mut model = RigidFiducial::baseline(2);
    let truth = RigidTransform::new(
        Mat3::from_axis_angle(Vec3::new(0.3, -0.2, 0.4)),
        Vec3::new(0.001, 0.002, 0.0),
    );
    let mut a = acquire_tool(
        &c,
        &Scene::default(),
        &model,
        truth,
        MeasurementMode::Precision,
        &timing(&c, 4, 1.0, 0.0),
    )
    .unwrap();
    // Every camera loses a different feature, and one feature has only one view.
    a.observations
        .retain(|o| o.feature_id != o.sensor_id % 5 + 1 && (o.feature_id != 5 || o.sensor_id == 1));
    let pose = estimate_tool_pose(&c, &model, &a.observations).unwrap();
    assert!((pose.world_from_tcp.translation - truth.translation).norm() < 3e-6);
    let angle = ((pose.world_from_tcp.rotation * truth.rotation.transpose()).trace() * 0.5 - 0.5)
        .clamp(-1.0, 1.0)
        .acos();
    assert!(angle < 0.002);
    assert!(pose.quality.partial_visibility);
    let original = pose.predicted_tcp_3d_rms_m();
    model.tcp_from_fiducial.translation.z = -0.010;
    let longer = estimate_tool_pose(&c, &model, &a.observations).unwrap();
    assert!(longer.predicted_tcp_3d_rms_m() > original * 2.0);
}

#[test]
fn collinear_markers_do_not_invent_roll_and_duplicate_views_are_rejected() {
    let c = random_only();
    let mut m = RigidFiducial::baseline(2);
    for (i, f) in m.features.iter_mut().enumerate() {
        f.point_fiducial_m = Vec3::Z * (i as f64 * 0.001);
    }
    let a = acquire_tool(
        &c,
        &Scene::default(),
        &m,
        RigidTransform::IDENTITY,
        MeasurementMode::Precision,
        &timing(&c, 1, 1.0, 0.0),
    )
    .unwrap();
    assert_eq!(
        estimate_tool_pose(&c, &m, &a.observations),
        Err(MetrologyError::UnobservablePose)
    );
    let mut a = observe(&c, Vec3::ZERO, 1);
    a.observations.push(a.observations[0].clone());
    assert_eq!(
        reconstruct_point(&c, &a.observations),
        Err(MetrologyError::DuplicateObservation)
    );
}

#[test]
fn hybrid_decodes_fractional_projector_pixels_and_rejects_motion_or_missing_frames() {
    let mut c = random_only();
    c.errors.phase_intensity_sigma = 0.001;
    c.errors.correspondence_sigma_px = 0.02;
    let f = feature(Vec3::new(0.0013, -0.0007, 0.002));
    let t = timing(&c, 3, 1.0, 0.0);
    let a = acquire_structured(&c, &Scene::default(), &f, 1, 101, &t).unwrap();
    assert_eq!(a.patterns.len(), 54);
    let projected = c
        .device(101)
        .unwrap()
        .model
        .project(f.point_world_m)
        .unwrap()
        .pixel;
    let d = decode_hybrid(&c, &a.patterns).unwrap();
    assert!((d.pixel - projected).norm() < 0.15);
    let p = reconstruct_point(&c, &a.observations).unwrap();
    assert!((p.position_world_m - f.point_world_m).norm() < 5e-6);
    assert!(p.quality.newest_acquisition_s - p.quality.oldest_acquisition_s > 0.02);
    let mut bad = a.patterns.clone();
    bad.pop();
    assert_eq!(
        decode_hybrid(&c, &bad),
        Err(MetrologyError::IncompatibleTiming)
    );
    let mut bad = a.patterns.clone();
    for f in &mut bad {
        f.timing.motion_evidence = MotionEvidence::IndependentOpticalBound;
        f.timing.speed_bound_m_s = 0.001;
    }
    assert_eq!(
        decode_hybrid(&c, &bad),
        Err(MetrologyError::MotionDuringSequence)
    );
    let mut bad = a.patterns;
    bad[10].timing.sequence_id += 1;
    assert_eq!(
        decode_hybrid(&c, &bad),
        Err(MetrologyError::IncompatibleTiming)
    );
}

#[test]
fn gray_phase_disagreement_saturation_and_surface_fail_closed() {
    let c = MetrologyConfig::baseline();
    let t = timing(&c, 1, 1.0, 0.0);
    let f = feature(Vec3::ZERO);
    let a = acquire_structured(&c, &Scene::default(), &f, 1, 101, &t).unwrap();
    let mut saturated = a.patterns.clone();
    saturated[4].intensity = 1.0;
    assert_eq!(
        decode_hybrid(&c, &saturated),
        Err(MetrologyError::Saturation)
    );
    let mut corrupt = a.patterns;
    for frame in &mut corrupt {
        if matches!(
            frame.pattern,
            Pattern::Gray {
                axis: PatternAxis::U,
                bit: 4,
                ..
            }
        ) {
            frame.intensity = 0.6 - frame.intensity;
        }
    }
    assert!(decode_hybrid(&c, &corrupt).is_err());
    for (dark, translucent, metal) in [
        (true, false, false),
        (false, true, false),
        (false, false, true),
    ] {
        let mut f = f.clone();
        if dark {
            f.surface.diffuse_at_450_650_850nm = [0.0001; 3];
        }
        if translucent {
            f.surface.translucent_fraction = 0.8;
        }
        if metal {
            f.surface.specular_fraction = 0.9;
        }
        assert!(acquire_structured(&c, &Scene::default(), &f, 1, 101, &t).is_err());
    }
    let mut c = c;
    c.illumination.polarization = Polarization::CrossedLinear;
    let mut f = f;
    f.surface.specular_fraction = 0.9;
    // Crossed polarization improves this simplified case; translucency still fails.
    assert!(acquire_structured(&c, &Scene::default(), &f, 1, 101, &t).is_ok());
    f.surface.translucent_fraction = 0.4;
    assert_eq!(
        acquire_structured(&c, &Scene::default(), &f, 1, 101, &t),
        Err(MetrologyError::UnsupportedSurface)
    );
}

#[test]
fn actual_geometry_occludes_camera_and_projector_rays_without_tag_exemptions() {
    let c = MetrologyConfig::baseline();
    let f = feature(Vec3::ZERO);
    let t = timing(&c, 1, 1.0, 0.0);
    let blocked = Scene::new(vec![Primitive::new(
        Geometry::Sphere(Sphere {
            center: c.devices[0].model.center_world() * 0.5,
            radius_m: 0.003,
        }),
        Default::default(),
        f.object_id,
    )]);
    let a = acquire_passive(
        &c,
        &blocked,
        &f,
        MeasurementMode::Precision,
        MeasurementClass::CriticalFeature,
        &t,
    )
    .unwrap();
    assert!(a
        .visibility
        .iter()
        .any(|v| v.sensor_id == 1 && v.rejection == Some(MetrologyError::Occluded)));
    assert!(!a.observations.iter().any(|o| o.sensor_id == 1));
    let shadow = Scene::new(vec![Primitive::new(
        Geometry::Sphere(Sphere {
            center: c.device(101).unwrap().model.center_world() * 0.5,
            radius_m: 0.003,
        }),
        Default::default(),
        9,
    )]);
    assert_eq!(
        acquire_structured(&c, &shadow, &f, 1, 101, &t),
        Err(MetrologyError::Occluded)
    );
}

#[test]
fn health_detects_thermal_and_common_frame_shift_not_visible_in_reprojection() {
    let c = MetrologyConfig::baseline();
    let t = timing(&c, 1, 1.0, 0.0);
    let baseline = acquire_references(&c, &Scene::default(), &t).unwrap();
    let h = evaluate_health(&c, &baseline, 1.02);
    assert!(h.reference_count >= 3);
    assert!(h.measured_reference_rms_m.unwrap() < 5e-6);
    let mut thermal = c.clone();
    thermal.thermal.structure_temperature_k += 5.0;
    let shifted = acquire_references(&thermal, &Scene::default(), &t).unwrap();
    let hot = evaluate_health(&thermal, &shifted, 1.02);
    assert!(!hot.thermal_valid);
    let p = Vec3::new(0.1, 0.0, 0.0);
    assert!((thermal.thermal.expanded(p, 293.15).x - p.x - 11.5e-6).abs() < 1e-12);
    let mut moved = baseline;
    for p in &mut moved {
        p.position_world_m.x += 20e-6;
    }
    let shifted = evaluate_health(&c, &moved, 1.02);
    assert!(shifted.measured_reference_rms_m.unwrap() > 15e-6);
}

#[test]
fn operation_rejects_nan_stale_unavailable_unknown_calibration_and_ideal_truth() {
    let c = random_only();
    let model = RigidFiducial::baseline(2);
    let t = timing(&c, 1, 1.0, 0.0);
    let a = acquire_tool(
        &c,
        &Scene::default(),
        &model,
        RigidTransform::IDENTITY,
        MeasurementMode::Precision,
        &t,
    )
    .unwrap();
    let tool = estimate_tool_pose(&c, &model, &a.observations).unwrap();
    let point =
        reconstruct_point(&c, &observe(&c, Vec3::new(0.003, 0.0, 0.0), 1).observations).unwrap();
    let refs = acquire_references(&c, &Scene::default(), &t).unwrap();
    let h = evaluate_health(&c, &refs, 1.02);
    let policy = PrecisionContract::default();
    assert_eq!(
        require_precision(&c, &policy, &tool, &point, &h, 1.02),
        Ok(())
    );
    let mut bad = tool.clone();
    bad.tcp_covariance[0][0] = f64::NAN;
    assert_eq!(
        require_precision(&c, &policy, &bad, &point, &h, 1.02),
        Err(PrecisionRejection::InvalidCovariance)
    );
    assert_eq!(
        require_precision(&c, &policy, &tool, &point, &h, 1.5),
        Err(PrecisionRejection::Stale)
    );
    let mut bad = tool.clone();
    bad.quality.available_s = 1.1;
    assert_eq!(
        require_precision(&c, &policy, &bad, &point, &h, 1.02),
        Err(PrecisionRejection::Unavailable)
    );
    let mut bad = tool.clone();
    bad.quality.source = ObservationSource::IdealSyntheticFeature;
    assert_eq!(
        require_precision(&c, &policy, &bad, &point, &h, 1.02),
        Err(PrecisionRejection::InvalidProvenance)
    );
    let mut bad = c.clone();
    bad.calibration.records.pop();
    assert_eq!(
        require_precision(&bad, &policy, &tool, &point, &h, 1.02),
        Err(PrecisionRejection::InvalidCalibration)
    );
    let mut physical = policy;
    physical.require_physical_observations = true;
    assert_eq!(
        require_precision(&c, &physical, &tool, &point, &h, 1.02),
        Err(PrecisionRejection::InvalidProvenance)
    );
}

#[test]
fn calibration_artifacts_cannot_verify_their_own_accuracy() {
    let c = MetrologyConfig::baseline();
    let point = reconstruct_point(&c, &observe(&c, Vec3::ZERO, 1).observations).unwrap();
    let reference = VerificationReference {
        artifact_id: "calibration_grid".into(),
        feature_id: 1,
        known_position_world_m: Vec3::ZERO,
        characterization_rms_m: 1e-6,
        provenance: ReferenceProvenance::IndependentSyntheticGeometry,
    };
    assert_eq!(
        verify_point(&c, reference, point, Vec3::Z),
        Err(MetrologyError::VerificationLeakage)
    );
}

#[test]
fn configurations_reject_bad_units_poses_and_unsupported_optics() {
    let c = MetrologyConfig::baseline();
    assert!(c.validate().is_ok());
    for f in [
        |c: &mut MetrologyConfig| c.devices[0].pixel_pitch_m[0] = 3.0,
        |c: &mut MetrologyConfig| c.devices[0].model.world_from_camera.rotation = Mat3::ZERO,
        |c: &mut MetrologyConfig| {
            c.devices[0].lens_model = LensModel::TelecentricCandidateUnsupported
        },
        |c: &mut MetrologyConfig| c.acquisition.exposure_s = f64::NAN,
        |c: &mut MetrologyConfig| c.devices[0].autofocus = true,
    ] {
        let mut bad = c.clone();
        f(&mut bad);
        assert!(bad.validate().is_err());
    }
}

#[test]
fn focus_aperture_wavelength_and_feature_size_change_sampling_uncertainty() {
    let c = MetrologyConfig::baseline();
    let device = &c.devices[0];
    let focused = optical_blur_sigma_px(device, device.focus_working_distance_m, 450e-9).unwrap();
    let defocused =
        optical_blur_sigma_px(device, device.focus_working_distance_m + 0.012, 450e-9).unwrap();
    assert!(defocused > focused * 5.0);
    let infrared = optical_blur_sigma_px(device, device.focus_working_distance_m, 850e-9).unwrap();
    assert!(infrared > focused * 1.8);
    let mut stopped = timing(&c, 1, 1.0, 0.0);
    let mut moving = feature(Vec3::ZERO);
    moving.velocity_world_m_s = Vec3::X * 0.001;
    assert_eq!(
        acquire_passive(
            &c,
            &Scene::default(),
            &moving,
            MeasurementMode::Precision,
            MeasurementClass::CriticalFeature,
            &stopped
        ),
        Err(MetrologyError::MotionDuringSequence)
    );
    stopped.motion_evidence = MotionEvidence::IndependentOpticalBound;
    stopped.speed_bound_m_s = 0.001;
    assert_eq!(
        acquire_structured(&c, &Scene::default(), &moving, 1, 101, &stopped),
        Err(MetrologyError::MotionDuringSequence)
    );
}

#[test]
fn silhouette_product_keeps_unknown_views_and_dense_scan_is_separate() {
    use std::collections::BTreeSet;
    let mut c = random_only();
    c.errors.phase_intensity_sigma = 0.001;
    c.errors.correspondence_sigma_px = 0.02;
    let t = timing(&c, 1, 1.0, 0.0);
    let region = PrecisionVolume {
        center_world_m: Vec3::ZERO,
        size_m: Vec3::splat(100e-6),
    };
    let mut illumination = c.illumination.clone();
    illumination.kind = IlluminationKind::Backlight;
    let masks = c
        .devices
        .iter()
        .filter(|d| d.kind == DeviceKind::Camera)
        .take(2)
        .map(|d| {
            let p = d.model.project(Vec3::ZERO).unwrap().pixel;
            let pixel = (p.x.round() as u32, p.y.round() as u32);
            SilhouetteObservation {
                camera_id: d.id,
                foreground_pixels: BTreeSet::from([pixel]),
                unknown_pixels: BTreeSet::new(),
                edge_uncertainty_px: 0.5,
                timing: t.clone(),
                illumination: illumination.clone(),
                calibration_id: c.calibration.id.clone(),
                source: ObservationSource::SyntheticNoisyImageFeature,
            }
        })
        .collect::<Vec<_>>();
    let hull = reconstruct_occupancy(&c, &masks, region.clone(), 100e-6).unwrap();
    assert_eq!(hull.voxels[0].state, OccupancyState::OccupiedOrOccluded);
    assert_eq!(hull.class, MeasurementClass::Occupancy);
    let mut missing = masks.clone();
    missing[0].unknown_pixels = missing[0].foreground_pixels.clone();
    assert_eq!(
        reconstruct_occupancy(&c, &missing, region.clone(), 100e-6)
            .unwrap()
            .voxels[0]
            .state,
        OccupancyState::Unknown
    );
    let mut empty = masks;
    empty[0].foreground_pixels.clear();
    assert_eq!(
        reconstruct_occupancy(&c, &empty, region, 100e-6)
            .unwrap()
            .voxels[0]
            .state,
        OccupancyState::EmptyOutsideDilatedSilhouettes
    );
    let surface = Scene::new(vec![Primitive::new(
        Geometry::Sphere(Sphere {
            center: Vec3::ZERO,
            radius_m: 0.003,
        }),
        Default::default(),
        8,
    )]);
    let scan =
        acquire_surface_scan(&c, &surface, 1, 101, 512, &t, &SurfaceResponse::default()).unwrap();
    assert_eq!(scan.attempted, 48);
    assert!(scan.sequence_duration_s > 0.02);
    assert_eq!(
        scan.observed_points.len() + scan.rejections.len(),
        scan.attempted
    );
    assert!(
        !scan.observed_points.is_empty(),
        "the scan must reconstruct actual surface points: {:?}",
        scan.rejections
    );
    assert!(scan
        .observed_points
        .iter()
        .all(|p| p.quality.class == MeasurementClass::DenseGeometry));
}
