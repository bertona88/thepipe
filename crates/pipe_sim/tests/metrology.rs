use pipe_optics::metrology::*;
use pipe_optics::{Geometry, Primitive, Scene, Sphere, Vec3};
use pipe_sim::point_motion::PointMotionRuntime;
use pipe_sim_core::ManipulatorId;

#[test]
fn machine_acquisition_uses_live_geometry_and_advances_time() {
    let mut machine = PointMotionRuntime::new().unwrap();
    let mut metrology = machine.metrology_candidate().unwrap();
    assert_eq!(
        metrology.config.tube_id_m, 0.160,
        "legacy runtime scale is explicit"
    );
    let first = machine.acquire_metrology(&mut metrology).unwrap();
    assert!(first.occluder_count > 30);
    assert!(first.time_s > 0.0);
    assert!(first.machine_tick > 0);
    assert_eq!(first.config_sha256.len(), 64);
    assert!(!first.visibility.is_empty());
    assert!(
        first.visibility.iter().any(|v| v.rejection.is_some()),
        "all live links and jaws must cast real shadows"
    );
    assert!(first.world.health.is_some());
    // Missing data replaces, rather than silently retaining, the old state.
    metrology.extra_occluders = Scene::new(vec![Primitive::new(
        Geometry::Sphere(Sphere {
            center: Vec3::ZERO,
            radius_m: 0.075,
        }),
        Default::default(),
        9,
    )]);
    let blocked = machine.acquire_metrology(&mut metrology).unwrap();
    assert!(blocked.world.entities.is_empty());
    assert!(blocked.world.health.unwrap().reference_count < 3);
    assert!(blocked.time_s > first.time_s);
}

#[test]
fn precision_gate_blocks_commands_without_observed_tool_target_support() {
    let mut machine = PointMotionRuntime::new().unwrap();
    let mut metrology = machine.metrology_candidate().unwrap();
    let before = machine.scene_frame_json(false).unwrap();
    assert!(machine
        .submit_precision_target(
            &mut metrology,
            &PrecisionContract::default(),
            1,
            999,
            ManipulatorId(1),
            pipe_sim_core::Vec3::ZERO
        )
        .is_err());
    assert_eq!(before, machine.scene_frame_json(false).unwrap());
}

#[test]
fn calibration_zone_cannot_silently_use_another_machine_scale() {
    let mut machine = PointMotionRuntime::new().unwrap();
    let mut metrology = machine.metrology_candidate().unwrap();
    metrology.config = MetrologyConfig::baseline();
    assert!(machine.acquire_metrology(&mut metrology).is_err());
    assert!(metrology.world.entities.is_empty());
}

#[test]
fn moving_machine_cannot_produce_a_stopped_precision_exposure() {
    let mut machine = PointMotionRuntime::new().unwrap();
    let mut metrology = machine.metrology_candidate().unwrap();
    machine.submit_calibration_target().unwrap();
    assert!(machine.acquire_metrology(&mut metrology).is_err());
    assert!(metrology.world.entities.is_empty());
}
