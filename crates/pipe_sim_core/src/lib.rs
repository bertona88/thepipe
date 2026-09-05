//! Deterministic mechanics primitives for the Pipe microrobotic assembly cell.
//!
//! The crate intentionally has no third-party dependencies.  Its public structs
//! are plain Rust data so a thin `wasm-bindgen`, C, Python, or serde adapter can
//! be maintained without coupling the engineering model to a presentation
//! layer.  All physical scalar fields use SI units; suffixes such as `_m` and
//! `_s` make that convention explicit at FFI boundaries.

#![forbid(unsafe_code)]

pub mod actuator;
pub mod arm;
pub mod collision;
pub mod geometry;
pub mod gripper;
pub mod machine;
pub mod math;
pub mod serial_arm;
pub mod simulation;
pub mod units;

pub use actuator::{ActuatorConfig, ActuatorState};
pub use arm::{
    ArmConfig, ArmError, ArmKinematics, ArmSegmentConfig, ArmSegmentState, ContinuumArm,
    SegmentFrame,
};
pub use collision::{
    broad_phase_pairs, gear_mesh_clearance, query_pair, Clearance, CollisionReport,
    CollisionSettings, Contact, ContactKind, GearMeshClearance, Proximity,
};
pub use geometry::{
    Aabb, BodyId, CollisionFilter, GearGeometry, Material, MotionType, RigidBody, Shape,
};
pub use gripper::{GraspCandidate, GripperConfig, GripperState, MIN_PARTIAL_GRASP_AXIAL_OVERLAP_M};
pub use machine::{
    wrap_angle_pi, CarriageConfig, CarriageState, CarriageTarget, MachineBackend, MachineCommand,
    MachineCommandError, MachineCommandEvent, ManipulatorId, ManipulatorMotionConfig,
    ManipulatorMotionState, PipeCellConfig, QualificationTargets, RailTopology, SafetyConfig,
    ToolMotionPlan, ToolMotionStatus, TubeGeometry, MACHINE_CONFIG_SCHEMA_VERSION,
};
pub use math::{Mat3, Pose, Quat, Vec3};
pub use serial_arm::{
    SerialArm, SerialArmConfig, SerialArmError, SerialArmKinematics, SerialJointPositions,
    TendonJointConfig, TendonJointTelemetry, ToolAxisSolution, ToolPositionIkError,
    ToolPositionSolution, TENDON_JOINT_COUNT,
};
pub use simulation::{serial_arm_link_body_id, SERIAL_ARM_COLLISION_BODY_ID_BASE};
pub use simulation::{
    ArmId, ArmInstance, SerialArmInstance, Simulation, SimulationConfig, SimulationError,
    StepReport, ToolMotionTraceSample,
};
pub use units::{Angle, Force, Length, Mass, Time, Torque};

/// Format version for snapshots produced by downstream adapters.
///
/// The core itself does not prescribe a serializer, but exposing a version lets
/// JS/WASM and native wrappers reject incompatible snapshots deterministically.
pub const STATE_SCHEMA_VERSION: u32 = 1;
