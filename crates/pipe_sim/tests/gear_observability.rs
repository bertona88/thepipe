use pipe_sim::gear_observability::*;
#[test]
fn study_is_deterministic_and_transparency_does_not_become_a_valid_surface() {
    let c = GearStudyConfig {
        repeats: 1,
        projector_angle_candidates_rad: vec![-std::f64::consts::FRAC_PI_4],
        marker_offset_candidates_m: vec![0.006],
        camera_axial_candidates_m: vec![0.025],
        relation_rms_candidates_m: vec![8e-6],
        ..Default::default()
    };
    let a = run_gear_study(c.clone(), &"a".repeat(40)).unwrap();
    let b = run_gear_study(c, &"a".repeat(40)).unwrap();
    assert_eq!(
        serde_json::to_string(&a).unwrap(),
        serde_json::to_string(&b).unwrap()
    );
    let candidate = &a.candidates[0];
    assert_eq!(
        candidate.samples.len() + candidate.rejected_design_states.len(),
        5
    );
    assert!(!candidate.samples.is_empty());
    assert!(!a.execution_probe.assembly_completed);
    assert!(a
        .execution_probe
        .stopped_reason
        .contains("ToolPathCollision"));
    assert!(candidate.static_states_with_contact > 0);
    let mut characterized = a.configuration.clone();
    characterized.relation_rms_candidates_m = vec![1e-6];
    let tighter = run_gear_study(characterized, &"a".repeat(40)).unwrap();
    for (loose, tight) in candidate.samples.iter().zip(&tighter.candidates[0].samples) {
        assert_eq!(
            loose.marker_pose, tight.marker_pose,
            "independent geometry must not leak into optical fit"
        );
        assert_eq!(
            loose.truth_world_from_shaft, candidate.samples[0].truth_world_from_shaft,
            "fixture must not follow the arm"
        );
    }
    for s in &candidate.samples {
        assert!(s.occluder_count > 400);
        assert!(s.direct_transparent_passive.estimate.is_none());
        assert!(s.direct_transparent_structured.estimate.is_none());
        assert!(
            s.needs_stop_or_reposition,
            "8 um relation error cannot meet 5 um contract"
        );
        if let Some(b) = &s.inferred_bore {
            assert!(b.center.predicted_3d_rms_m() >= 8e-6);
            assert!(b.provenance.contains("not_direct"));
        }
    }
    assert!(!candidate.all_states_optically_supported);
    assert!(candidate.accepted_alignment_3d_rms_m.is_none());
}
#[test]
fn invalid_studies_are_rejected() {
    let mut c = GearStudyConfig::default();
    c.shaft_radius_m = c.bore_radius_m;
    assert!(run_gear_study(c, &"a".repeat(40)).is_err());
    assert!(run_gear_study(GearStudyConfig::default(), "main").is_err());
}
