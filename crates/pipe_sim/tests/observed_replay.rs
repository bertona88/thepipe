use pipe_sim::observed_manipulation::{M1eFault, ObservedManipulationRuntime};

const REVISION: &str = "2540de4ac91e23e1f08fd5040461b439639f0b9a";

#[test]
fn recording_does_not_change_control_and_cannot_publish_before_terminal() {
    let mut plain = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    let expected = plain.run_cycle().unwrap();
    let mut recorded = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    recorded.enable_replay(500, 20_000).unwrap();
    assert!(recorded.replay(REVISION, "test").is_err());
    assert_eq!(recorded.run_cycle().unwrap(), expected);
    let replay = recorded.replay(REVISION, "test").unwrap();
    assert_eq!(replay.report, expected);
    assert_eq!(replay.frames.first().unwrap().scene.tick, 0);
    assert_eq!(replay.frames.last().unwrap().scene.tick, expected.decisions.last().unwrap().tick);
    assert!(replay.frames.windows(2).all(|pair| pair[0].scene.tick < pair[1].scene.tick));
    for frame in &replay.frames {
        assert!(frame.scene.estimate.is_none(), "never manufacture full-state estimates");
        let truth = frame.scene.truth.as_ref().unwrap();
        for body in &truth.rigid_bodies {
            let geometry = frame.bodies.iter().find(|entry| entry.body_id == body.id).unwrap();
            assert_eq!(geometry.pose, body.pose);
        }
        assert_eq!(frame.contact_packet.captured_at_tick, frame.scene.tick);
    }
    assert_eq!(replay, recorded.replay(REVISION, "test").unwrap());
}

#[test]
fn recording_overflow_is_an_export_error_not_a_control_input() {
    let mut runtime = ObservedManipulationRuntime::new(M1eFault::None).unwrap();
    assert!(runtime.enable_replay(0, 2).is_err());
    runtime.enable_replay(1, 2).unwrap();
    let report = runtime.run_cycle().unwrap();
    assert_eq!(report.status, "complete");
    assert!(runtime.replay(REVISION, "test").is_err());
}

#[test]
fn failed_cycle_keeps_failure_and_stop_in_the_replay() {
    let mut runtime = ObservedManipulationRuntime::new(M1eFault::OpticalDropout).unwrap();
    runtime.enable_replay(100, 20_000).unwrap();
    let report = runtime.run_cycle().unwrap();
    assert!(!runtime.is_completed());
    let replay = runtime.replay(REVISION, "test").unwrap();
    assert_eq!(replay.report.terminal_reason, report.terminal_reason);
    assert!(replay.frames.last().unwrap().scene.commanded.manipulators.iter().all(|arm| arm.stopped));
}
