use pipe_optics::{
    metrology::{synthetic::*, *},
    *,
};
#[test]
fn relation_error_and_pose_lever_arm_survive_inference() {
    let c = MetrologyConfig::baseline();
    let f = RigidFiducial::baseline(700);
    let t = timing(&c, 1, 1., 0.);
    let a = acquire_tool(
        &c,
        &Scene::default(),
        &f,
        RigidTransform::IDENTITY,
        MeasurementMode::Precision,
        &t,
    )
    .unwrap();
    let p = estimate_tool_pose(&c, &f, &a.observations).unwrap();
    let mut r = PartFeatureRelation {
        feature_id: 30,
        center_fiducial_m: Vec3::ZERO,
        axis_fiducial: Vec3::Z,
        calibration_id: "held_out_relation".into(),
        provenance: RelationProvenance::SyntheticIndependentCharacterization,
        characterization_rms_m: 1e-6,
        axis_characterization_rms_rad: 1e-4,
    };
    let near = infer_part_feature(&p, &r).unwrap();
    r.center_fiducial_m = Vec3::X * 0.050;
    let far = infer_part_feature(&p, &r).unwrap();
    assert!(far.center.predicted_3d_rms_m() > near.center.predicted_3d_rms_m());
    r.characterization_rms_m = 8e-6;
    let uncertain = infer_part_feature(&p, &r).unwrap();
    assert!(
        (uncertain.center.predicted_3d_rms_m().powi(2)
            - far.center.predicted_3d_rms_m().powi(2)
            - 63e-12)
            .abs()
            < 1e-20
    );
    r.axis_fiducial = Vec3::ZERO;
    assert!(infer_part_feature(&p, &r).is_err());
}

#[test]
fn nominal_cad_relation_cannot_admit_precision_contact() {
    let c = MetrologyConfig::baseline();
    let f = RigidFiducial::baseline(700);
    let t = timing(&c, 1, 1., 0.);
    let a = acquire_tool(
        &c,
        &Scene::default(),
        &f,
        RigidTransform::IDENTITY,
        MeasurementMode::Precision,
        &t,
    )
    .unwrap();
    let p = estimate_tool_pose(&c, &f, &a.observations).unwrap();
    let relation = PartFeatureRelation {
        feature_id: 30,
        center_fiducial_m: Vec3::ZERO,
        axis_fiducial: Vec3::Z,
        calibration_id: "cad".into(),
        provenance: RelationProvenance::NominalDesign,
        characterization_rms_m: 1e-6,
        axis_characterization_rms_rad: 1e-4,
    };
    let feature = infer_part_feature(&p, &relation).unwrap();
    let health = evaluate_health(&c, &[], t.available_s);
    assert_eq!(
        require_inferred_precision(
            &c,
            &PrecisionContract::default(),
            &p,
            &feature,
            &health,
            t.available_s
        ),
        Err(PrecisionRejection::InvalidProvenance)
    );
    let mut bad = p;
    bad.fiducial_covariance[0][0] = -1.;
    assert!(infer_part_feature(&bad, &relation).is_err());
}
