//! Evaluation-only recording. These samples never enter the controller.

use serde::Serialize;

use super::controller::ContactPacket;
use super::report::ObservedManipulationReport;
use crate::scene::{ColliderSnapshot, PoseSnapshot, SceneFrame};

pub const OBSERVED_REPLAY_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ObservedReplayFrame {
    pub scene: SceneFrame,
    /// Shapes and poses are copied from the actual reduced mechanics bodies.
    pub bodies: Vec<ColliderSnapshot>,
    /// Includes the plant's distal mounting tilt; scene.tool_pose is the FK flange.
    pub physical_tool_pose: PoseSnapshot,
    pub socket_pose: PoseSnapshot,
    pub physical_jaws: Vec<ColliderSnapshot>,
    /// Timestamped reduced load channels, not calibrated force measurements.
    pub contact_packet: ContactPacket,
    pub commanded_tool_position_world_m: [f64; 3],
    pub commanded_tool_axis_world: Option<[f64; 3]>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ObservedReplay {
    pub schema_version: u32,
    pub source_revision: String,
    pub generation_command: String,
    pub coordinate_frame: &'static str,
    pub length_unit: &'static str,
    pub angle_unit: &'static str,
    pub time_unit: &'static str,
    pub sample_every_ticks: u64,
    pub sampling: &'static str,
    pub geometry_fidelity: &'static str,
    pub estimate_mapping: &'static str,
    pub frames: Vec<ObservedReplayFrame>,
    pub report: ObservedManipulationReport,
}

pub(super) struct ReplayRecorder {
    pub every_ticks: u64,
    pub maximum_frames: usize,
    pub overflowed: bool,
    pub frames: Vec<ObservedReplayFrame>,
}

impl ReplayRecorder {
    pub fn push(&mut self, frame: ObservedReplayFrame) {
        // At a decision tick keep the final authoritative state at that tick.
        // Every individual decision remains in the report's ordered event log.
        if self.frames.last().is_some_and(|last| last.scene.tick == frame.scene.tick) {
            *self.frames.last_mut().expect("checked last frame") = frame;
        } else if self.frames.len() < self.maximum_frames {
            self.frames.push(frame);
        } else {
            self.overflowed = true;
        }
    }
}
