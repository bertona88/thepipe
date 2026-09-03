//! Deterministic end-to-end reference simulation for the Pipe assembly cell.
//!
//! Detailed 2PP process variation is intentionally out of scope. The parts are
//! idealized envelopes; the simulator concentrates on the inexpensive tendon
//! mechanics, observed-volume metrology, collision/force gates, and assembly
//! executive that are the hard system-level questions.

#![forbid(unsafe_code)]

mod machine_config;
pub mod point_motion;
pub mod scene;
pub mod simple_manipulation;

use std::collections::BTreeSet;
use std::fmt;

use pipe_optics::{
    BrownConrady, CalibratedCamera, CameraIntrinsics, DepthReturn, Geometry, ImageSize,
    Mat3 as OpticalMat3, Material as OpticalMaterial, MissingReturn, PinholeCamera, Primitive,
    RigidTransform, ScanConfig, Scene, Sphere as OpticalSphere, StructuredLightRig,
    Vec3 as OpticalVec3,
};
use pipe_planner::{
    nominal_pose_to_core, AlignmentObservation, AssemblyContactKind, AssemblyScenario, Command,
    ComponentGeometry, ComponentId, ComponentKind, ComponentPlan, Decision, EventKind,
    ExecutiveStatus, FailureReason, GearboxTaskExecutive, GripperObservation, HandoffObservation,
    InsertionObservation, MeshObservation, MicroGearboxScenario, Phase, PoseError6d, SensorFrame,
    TaskMetrics, VerificationObservation, VisionObservation,
};
use pipe_sim_core::{
    ArmId, BodyId, CollisionFilter, CollisionReport, GearGeometry, MachineCommand, ManipulatorId,
    MotionType, PipeCellConfig, Pose, Quat, RigidBody, SerialArm, SerialArmInstance,
    SerialJointPositions, Shape, Simulation, SimulationConfig, StepReport, Vec3,
    SERIAL_ARM_COLLISION_BODY_ID_BASE,
};
use serde::Serialize;
use sha2::{Digest, Sha256};

pub use point_motion::{
    CalibrationCycleReport, PointMotionReport, PointMotionRuntime, PointMotionTraceRecord,
    CALIBRATION_APPROACH_WORLD_M, CALIBRATION_TARGET_WORLD_M, POINT_MOTION_REPORT_SCHEMA_VERSION,
};
pub use scene::{SceneDescription, SceneFrame, SCENE_SCHEMA_VERSION};
pub use simple_manipulation::{
    ManipulationTraceRecord, SimpleManipulationReport, SimpleManipulationRuntime,
    CALIBRATION_INSERT_APPROACH_WORLD_M, CALIBRATION_INSERT_WORLD_M, CALIBRATION_PEG_BODY_ID,
    CALIBRATION_PICK_APPROACH_WORLD_M, CALIBRATION_PICK_WORLD_M,
    SIMPLE_MANIPULATION_REPORT_SCHEMA_VERSION,
};

pub const REPORT_SCHEMA_VERSION: u32 = 1;
const CONTROL_PERIOD_MS: u64 = 20;
const MIN_INDEPENDENT_CAMERA_VIEWS: u32 = 2;
const OPTICAL_CORRELATED_FLOOR_MM: f64 = 0.003;
const OBSTACLE_BODY_ID: BodyId = BodyId(9_000);
const HOUSING_FLOOR_BODY_ID: BodyId = BodyId(8_000);
const HOUSING_LEFT_WALL_BODY_ID: BodyId = BodyId(8_001);
const HOUSING_RIGHT_WALL_BODY_ID: BodyId = BodyId(8_002);
const HOUSING_FRONT_WALL_BODY_ID: BodyId = BodyId(8_003);
const HOUSING_BACK_WALL_BODY_ID: BodyId = BodyId(8_004);
const PART_GROUP: u32 = 0b0001;
const OBSTACLE_GROUP: u32 = 0b0010;
const FIXTURE_GROUP: u32 = 0b0100;
const COMPILED_CAD_PARAMETER_SHA256: &str =
    "23e1fbcbdb795ad262ae09f9da75272a0955566c41d4ed8d16d399cbdfeab40c";
const COMPILED_CAD_GEOMETRY_FACTS_SHA256: &str =
    "44c1a7054ab49b4421c69557aab21dded9aa0543d378b13ec8a7ff79cf18632d";

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultPlan {
    pub collision: bool,
    pub occlusion: bool,
    pub insertion_force: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScenarioSpec {
    pub name: String,
    pub seed: u64,
    /// SHA-256 over this complete run-spec provenance tuple, including fault
    /// selection. This is not an alias for any one source document hash.
    pub configuration_sha256: String,
    /// SHA-256 of canonical scenario JSON when loaded from a file. Compiled
    /// aliases report `None` rather than pretending to have source bytes.
    pub scenario_sha256: Option<String>,
    pub cad_parameter_sha256: Option<String>,
    pub cad_geometry_facts_sha256: Option<String>,
    pub faults: FaultPlan,
}

impl ScenarioSpec {
    pub fn named(name: &str) -> Result<Self, SimError> {
        let normalized = name.trim().to_ascii_lowercase();
        let faults = match normalized.as_str() {
            "nominal" | "gearbox_baseline_v1" | "ideal-2pp" | "ideal_2pp_m01_v1" => {
                FaultPlan::default()
            }
            "collision" => FaultPlan {
                collision: true,
                ..FaultPlan::default()
            },
            "occlusion" => FaultPlan {
                occlusion: true,
                ..FaultPlan::default()
            },
            "insertion-force" | "insertion_force" => FaultPlan {
                insertion_force: true,
                ..FaultPlan::default()
            },
            "fault-suite" | "fault_suite" => FaultPlan {
                collision: true,
                occlusion: true,
                insertion_force: true,
            },
            _ => return Err(SimError::UnknownScenario(name.to_owned())),
        };
        let canonical = match normalized.as_str() {
            "nominal" | "gearbox_baseline_v1" | "ideal-2pp" | "ideal_2pp_m01_v1" => {
                "gearbox_baseline_v1"
            }
            "insertion_force" => "insertion-force",
            "fault_suite" => "fault-suite",
            _ => normalized.as_str(),
        };
        let mut spec = Self {
            name: canonical.to_owned(),
            seed: 0x5049_5045_5F47_4258,
            configuration_sha256: String::new(),
            scenario_sha256: None,
            cad_parameter_sha256: Some(COMPILED_CAD_PARAMETER_SHA256.to_owned()),
            cad_geometry_facts_sha256: Some(COMPILED_CAD_GEOMETRY_FACTS_SHA256.to_owned()),
            faults,
        };
        spec.refresh_configuration_sha256();
        Ok(spec)
    }

    /// Recompute the stable run identifier after a trusted adapter attaches
    /// source-document provenance.
    pub fn refresh_configuration_sha256(&mut self) {
        let fault_bits = u8::from(self.faults.collision)
            | (u8::from(self.faults.occlusion) << 1)
            | (u8::from(self.faults.insertion_force) << 2);
        let provenance = format!(
            "pipe-run-config/v1\nname={}\nseed={:016x}\nfaults={fault_bits}\nscenario={}\ncad_parameters={}\ncad_geometry={}\n",
            self.name,
            self.seed,
            self.scenario_sha256.as_deref().unwrap_or("none"),
            self.cad_parameter_sha256.as_deref().unwrap_or("none"),
            self.cad_geometry_facts_sha256
                .as_deref()
                .unwrap_or("none"),
        );
        self.configuration_sha256 = sha256_hex(provenance.as_bytes());
    }

    pub const fn available() -> &'static [&'static str] {
        &[
            "gearbox_baseline_v1",
            "nominal",
            "collision",
            "occlusion",
            "insertion-force",
            "fault-suite",
        ]
    }
}

impl Default for ScenarioSpec {
    fn default() -> Self {
        Self::named("nominal").expect("built-in scenario")
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum SimError {
    UnknownScenario(String),
    InvalidScenario(String),
    Mechanics(String),
    CycleLimit(u32),
    Json(String),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownScenario(name) => write!(
                f,
                "unknown scenario '{name}'; expected {}",
                ScenarioSpec::available().join(", ")
            ),
            Self::InvalidScenario(reason) => write!(f, "invalid scenario: {reason}"),
            Self::Mechanics(reason) => write!(f, "mechanics error: {reason}"),
            Self::CycleLimit(limit) => write!(f, "simulation exceeded {limit} control cycles"),
            Self::Json(reason) => write!(f, "JSON serialization failed: {reason}"),
        }
    }
}

impl std::error::Error for SimError {}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
pub struct PoseState {
    pub translation_mm: [f64; 3],
    pub rotation_deg: [f64; 3],
}

impl PoseState {
    fn norm_mm(self) -> f64 {
        self.translation_mm[0]
            .hypot(self.translation_mm[1])
            .hypot(self.translation_mm[2])
    }

    fn corrected(&mut self, translation: [f64; 3], rotation: [f64; 3], gain: f64) {
        for axis in 0..3 {
            self.translation_mm[axis] += gain * translation[axis];
            self.rotation_deg[axis] += gain * rotation[axis];
        }
    }

    fn observed(self, bias_mm: [f64; 3]) -> PoseError6d {
        PoseError6d {
            translation_mm: std::array::from_fn(|axis| self.translation_mm[axis] + bias_mm[axis]),
            rotation_deg: self.rotation_deg,
        }
    }
}

#[derive(Clone, Debug)]
struct ComponentState {
    id: ComponentId,
    name: String,
    body_id: BodyId,
    plan: ComponentPlan,
    pose_error: PoseState,
    grip_force_n: f64,
    retained: bool,
    receiver_force_n: f64,
    receiver_has_part: bool,
    donor_released: bool,
    at_insertion_approach: bool,
    depth_mm: f64,
    axial_force_n: f64,
    lateral_force_n: f64,
    seated: bool,
    closure_features_confirmed: u8,
    mesh_sweep_deg: f64,
    mesh_torque_mn_mm: f64,
    backlash_mm: f64,
    teeth_engaged: bool,
    verification_rotation_deg: f64,
    completed: bool,
}

impl ComponentState {
    fn new(index: usize, plan: &ComponentPlan) -> Self {
        Self {
            id: plan.id,
            name: plan.name.clone(),
            body_id: BodyId(u32::from(plan.id.0)),
            plan: plan.clone(),
            pose_error: seeded_pose(index, 1.0),
            grip_force_n: 0.0,
            retained: false,
            receiver_force_n: 0.0,
            receiver_has_part: false,
            donor_released: false,
            at_insertion_approach: false,
            depth_mm: 0.0,
            axial_force_n: 0.0,
            lateral_force_n: 0.0,
            seated: false,
            closure_features_confirmed: 0,
            mesh_sweep_deg: 0.0,
            mesh_torque_mn_mm: 0.0,
            backlash_mm: 0.020,
            teeth_engaged: false,
            verification_rotation_deg: 0.0,
            completed: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct FaultState {
    collision_used: bool,
    occlusion_used: bool,
    insertion_force_used: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InjectedFault {
    Collision,
    Occlusion,
    InsertionForce,
}

impl InjectedFault {
    fn name(self) -> &'static str {
        match self {
            Self::Collision => "collision",
            Self::Occlusion => "occlusion",
            Self::InsertionForce => "insertion-force",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct OpticalTelemetry {
    pub attempted_rays: u32,
    pub valid_target_returns: u32,
    pub global_attempted_rays: u32,
    pub global_valid_target_returns: u32,
    pub macro_attempted_rays: u32,
    pub macro_valid_target_returns: u32,
    pub valid_camera_views: u32,
    pub global_valid_camera_views: u32,
    pub macro_valid_camera_views: u32,
    pub expected_visible_components: u32,
    pub visible_components: u32,
    pub occluded_returns: u32,
    pub no_surface_returns: u32,
    pub projector_coverage_failures: u32,
    pub signal_or_dropout_failures: u32,
    pub geometric_rejections: u32,
    pub invalid_returns: u32,
    pub confidence: f64,
    pub position_sigma_mm: f64,
    pub orientation_sigma_deg: f64,
    pub mean_range_error_um: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct MechanicsTelemetry {
    pub physics_step_index: u64,
    pub physics_time_s: f64,
    pub contact_count: usize,
    pub near_clearance_count: usize,
    pub intended_contact_count: usize,
    pub unplanned_contact_count: usize,
    pub unplanned_clearance_count: usize,
    pub diagnostic_arm_contact_count: usize,
    pub diagnostic_arm_clearance_count: usize,
    pub min_unplanned_clearance_mm: f64,
    pub maximum_penetration_um: f64,
    pub actuator_rms_tracking_error_um: f64,
    pub active_insertion_depth_mm: f64,
    pub active_axial_force_n: f64,
    pub active_lateral_force_n: f64,
}

/// Compact control-cycle record retained in acceptance reports.
///
/// The renderer-facing [`SceneFrame`] is deliberately excluded. Scene frames
/// are available on live [`StepSnapshot`] values and through
/// [`ReferenceSimulator::scene_frame`], but duplicating them across thousands
/// of report records makes the machine-readable acceptance artifact needlessly
/// large and changes the version-1 report schema.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ReportSnapshot {
    pub cycle: u64,
    pub control_time_ms: u64,
    pub component_id: Option<u16>,
    pub component_name: Option<String>,
    pub phase: Option<String>,
    pub status: String,
    pub command: String,
    pub injected_fault: Option<String>,
    pub optical: OpticalTelemetry,
    pub mechanics: MechanicsTelemetry,
    pub pose_error: Option<PoseState>,
    pub components_completed: u64,
    pub retries: u64,
    pub events: Vec<String>,
}

/// One live executive step, including the current renderer-neutral machine
/// scene. The compact report fields remain flattened for the WASM JSON API.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepSnapshot {
    #[serde(flatten)]
    pub report: ReportSnapshot,
    pub scene: SceneFrame,
}

impl StepSnapshot {
    pub fn to_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(self, pretty)
    }
}

impl std::ops::Deref for StepSnapshot {
    type Target = ReportSnapshot;

    fn deref(&self) -> &Self::Target {
        &self.report
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ComponentReport {
    pub id: u16,
    pub name: String,
    pub completed: bool,
    pub seated: bool,
    pub insertion_depth_mm: f64,
    pub final_pose_error_mm: f64,
    pub mesh_sweep_deg: f64,
    pub peak_mesh_torque_mn_mm: f64,
    pub backlash_mm: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TurnAcceptance {
    pub input_turns: f64,
    pub output_turns: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GearTrainAcceptance {
    /// This is an analytic ideal-gear constraint, not tooth-resolved contact.
    pub method: &'static str,
    pub measurement_source: &'static str,
    pub executed_in_task_loop: bool,
    pub tooth_resolved_contact: bool,
    pub input_teeth: u16,
    pub output_teeth: u16,
    pub expected_reduction: f64,
    pub observed_reduction: f64,
    pub same_direction: bool,
    pub forward: TurnAcceptance,
    pub reverse: TurnAcceptance,
    pub mesh_backlash_mm: [f64; 2],
    pub cover_latch_signatures: [bool; 2],
    pub accepted: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct BacklashMeshConstraint {
    driver_pitch_radius_mm: f64,
    driven_pitch_radius_mm: f64,
    backlash_mm: f64,
    flank_direction: i8,
    deadband_remaining_mm: f64,
}

impl BacklashMeshConstraint {
    fn new(driver_teeth: u16, driven_teeth: u16, module_mm: f64, backlash_mm: f64) -> Self {
        Self {
            driver_pitch_radius_mm: module_mm * f64::from(driver_teeth) * 0.5,
            driven_pitch_radius_mm: module_mm * f64::from(driven_teeth) * 0.5,
            backlash_mm,
            flank_direction: 0,
            deadband_remaining_mm: backlash_mm,
        }
    }

    /// Advance an external involute constraint. Direction reversals first
    /// consume the configured pitch-circle backlash before torque crosses to
    /// the opposite flank.
    fn drive(&mut self, driver_delta_turns: f64) -> f64 {
        if driver_delta_turns == 0.0 {
            return 0.0;
        }
        let direction = if driver_delta_turns.is_sign_positive() {
            1
        } else {
            -1
        };
        if direction != self.flank_direction {
            self.flank_direction = direction;
            self.deadband_remaining_mm = self.backlash_mm;
        }
        let driver_travel_mm =
            driver_delta_turns.abs() * std::f64::consts::TAU * self.driver_pitch_radius_mm;
        let deadband_mm = driver_travel_mm.min(self.deadband_remaining_mm);
        self.deadband_remaining_mm -= deadband_mm;
        let transmitted_mm = driver_travel_mm - deadband_mm;
        -f64::from(direction) * transmitted_mm
            / (std::f64::consts::TAU * self.driven_pitch_radius_mm)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ReducedGearTrainState {
    input_turns: f64,
    idler_turns: f64,
    output_turns: f64,
    input_to_idler: BacklashMeshConstraint,
    idler_to_output: BacklashMeshConstraint,
}

impl ReducedGearTrainState {
    fn new(
        input_teeth: u16,
        idler_teeth: u16,
        output_teeth: u16,
        module_mm: f64,
        backlash_mm: [f64; 2],
    ) -> Self {
        Self {
            input_turns: 0.0,
            idler_turns: 0.0,
            output_turns: 0.0,
            input_to_idler: BacklashMeshConstraint::new(
                input_teeth,
                idler_teeth,
                module_mm,
                backlash_mm[0],
            ),
            idler_to_output: BacklashMeshConstraint::new(
                idler_teeth,
                output_teeth,
                module_mm,
                backlash_mm[1],
            ),
        }
    }

    fn drive_input(&mut self, delta_turns: f64) {
        self.input_turns += delta_turns;
        let idler_delta = self.input_to_idler.drive(delta_turns);
        self.idler_turns += idler_delta;
        let output_delta = self.idler_to_output.drive(idler_delta);
        self.output_turns += output_delta;
    }

    /// Zero the angle encoders without resetting which tooth flanks carry
    /// load. This mirrors a real forward/reverse acceptance measurement after
    /// backlash take-up.
    fn zero_encoders(&mut self) {
        self.input_turns = 0.0;
        self.idler_turns = 0.0;
        self.output_turns = 0.0;
    }
}

#[derive(Clone, Debug, PartialEq)]
struct CollisionAudit {
    report: CollisionReport,
    intended_contact_count: usize,
    unplanned_contact_count: usize,
    unplanned_clearance_count: usize,
    diagnostic_arm_contact_count: usize,
    diagnostic_arm_clearance_count: usize,
    min_unplanned_clearance_mm: f64,
    maximum_unplanned_penetration_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct FidelityReport {
    pub tier: &'static str,
    pub mechanics_model: &'static str,
    pub collision_model: &'static str,
    pub optics_model: &'static str,
    pub controller_model: &'static str,
    pub part_model: &'static str,
    pub excluded_physics: Vec<&'static str>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct TaskMetricReport {
    pub commands_issued: u64,
    pub transitions: u64,
    pub visual_servo_corrections: u64,
    pub guarded_insert_commands: u64,
    pub mesh_dither_commands: u64,
    pub retainer_closure_commands: u64,
    pub retries: u64,
    pub safety_stops: u64,
    pub components_completed: u64,
    pub min_unplanned_clearance_mm: Option<f64>,
    pub max_unplanned_contact_force_n: f64,
    pub max_insertion_axial_force_n: f64,
    pub max_insertion_lateral_force_n: f64,
    pub max_mesh_torque_mn_mm: f64,
}

impl From<&TaskMetrics> for TaskMetricReport {
    fn from(value: &TaskMetrics) -> Self {
        Self {
            commands_issued: value.commands_issued,
            transitions: value.transitions,
            visual_servo_corrections: value.visual_servo_corrections,
            guarded_insert_commands: value.guarded_insert_commands,
            mesh_dither_commands: value.mesh_dither_commands,
            retainer_closure_commands: value.retainer_closure_commands,
            retries: value.retries,
            safety_stops: value.safety_stops,
            components_completed: value.components_completed,
            min_unplanned_clearance_mm: value.min_unplanned_clearance_mm,
            max_unplanned_contact_force_n: value.max_unplanned_contact_force_n,
            max_insertion_axial_force_n: value.max_insertion_axial_force_n,
            max_insertion_lateral_force_n: value.max_insertion_lateral_force_n,
            max_mesh_torque_mn_mm: value.max_mesh_torque_mn_mm,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SimulationReport {
    pub schema_version: u32,
    pub scenario: String,
    pub recipe: String,
    pub seed: u64,
    pub configuration_sha256: String,
    pub scenario_sha256: Option<String>,
    pub cad_parameter_sha256: Option<String>,
    pub cad_geometry_facts_sha256: Option<String>,
    pub status: String,
    pub completed: bool,
    pub control_cycles: u64,
    pub control_time_ms: u64,
    pub physics_time_s: f64,
    pub metrics: TaskMetricReport,
    pub failures: Vec<String>,
    pub components: Vec<ComponentReport>,
    pub gear_train_acceptance: GearTrainAcceptance,
    pub fidelity: FidelityReport,
    pub snapshots: Vec<ReportSnapshot>,
}

impl SimulationReport {
    pub fn to_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(self, pretty)
    }
}

fn serialize_json<T: Serialize>(value: &T, pretty: bool) -> Result<String, SimError> {
    if pretty {
        serde_json::to_string_pretty(value)
    } else {
        serde_json::to_string(value)
    }
    .map_err(|error| SimError::Json(error.to_string()))
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// Observed-volume cell that feeds physical/sensing state into the real task
/// executive. No phase result is pre-recorded: every gate value comes from the
/// current component, tendon, collision, optical, or force state.
pub struct ReferenceSimulator {
    spec: ScenarioSpec,
    scenario: AssemblyScenario,
    executive: GearboxTaskExecutive,
    machine_config_id: String,
    machine_config_sha256: String,
    cell_config: PipeCellConfig,
    mechanics: Simulation,
    optics: StructuredLightRig,
    components: Vec<ComponentState>,
    faults: FaultState,
    cycle: u64,
    control_time_ms: u64,
    last_snapshot: Option<StepSnapshot>,
    snapshots: Vec<ReportSnapshot>,
    failures: Vec<String>,
}

impl ReferenceSimulator {
    pub fn new(mut spec: ScenarioSpec) -> Result<Self, SimError> {
        // ScenarioSpec remains an ergonomic public transport type for native
        // and WASM callers. Recompute its derived identity at the trust
        // boundary so callers cannot mutate seed, faults, or source hashes and
        // accidentally emit a stale run identifier.
        spec.refresh_configuration_sha256();
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        scenario
            .validate()
            .map_err(|error| SimError::InvalidScenario(error.to_string()))?;
        let executive = GearboxTaskExecutive::new(scenario.clone())
            .map_err(|error| SimError::InvalidScenario(error.to_string()))?;
        let machine_config = machine_config::load_baseline_machine_config()?;
        if machine_config.cell.manipulator_count != scenario.cell.mobile_arm_count {
            return Err(SimError::InvalidScenario(format!(
                "machine config has {} manipulators but scenario requests {}",
                machine_config.cell.manipulator_count, scenario.cell.mobile_arm_count
            )));
        }
        if (machine_config.cell.tube.working_length_m * 1_000.0
            - scenario.cell.usable_tube_length_mm)
            .abs()
            > 1.0e-12
        {
            return Err(SimError::InvalidScenario(
                "machine config and task scenario disagree on usable tube length".to_owned(),
            ));
        }
        let components = scenario
            .recipe
            .components
            .iter()
            .enumerate()
            .map(|(index, part)| ComponentState::new(index, part))
            .collect::<Vec<_>>();
        let mut mechanics = build_mechanics(&scenario, machine_config.cell)?;
        let first = components.first().map(|part| part.id);
        sync_bodies(&mut mechanics, &components, first);
        Ok(Self {
            optics: build_optics(spec.seed),
            spec,
            scenario,
            executive,
            machine_config_id: machine_config.id,
            machine_config_sha256: machine_config.source_sha256,
            cell_config: machine_config.cell,
            mechanics,
            components,
            faults: FaultState::default(),
            cycle: 0,
            control_time_ms: 0,
            last_snapshot: None,
            snapshots: Vec::new(),
            failures: Vec::new(),
        })
    }

    pub fn from_scenario_name(name: &str) -> Result<Self, SimError> {
        Self::new(ScenarioSpec::named(name)?)
    }

    pub fn scenario_spec(&self) -> &ScenarioSpec {
        &self.spec
    }

    pub fn is_terminal(&self) -> bool {
        !matches!(self.executive.status(), ExecutiveStatus::Running)
    }

    pub fn is_completed(&self) -> bool {
        matches!(self.executive.status(), ExecutiveStatus::Completed)
    }

    pub fn last_snapshot(&self) -> Option<&StepSnapshot> {
        self.last_snapshot.as_ref()
    }

    pub fn snapshots(&self) -> &[ReportSnapshot] {
        &self.snapshots
    }

    pub fn scene_description(&self) -> SceneDescription {
        let mut body_names = self
            .components
            .iter()
            .map(|part| (part.body_id.0, part.name.clone()))
            .collect::<Vec<_>>();
        body_names.extend([
            (HOUSING_FLOOR_BODY_ID.0, "housing-floor".to_owned()),
            (HOUSING_LEFT_WALL_BODY_ID.0, "housing-left-wall".to_owned()),
            (
                HOUSING_RIGHT_WALL_BODY_ID.0,
                "housing-right-wall".to_owned(),
            ),
            (
                HOUSING_FRONT_WALL_BODY_ID.0,
                "housing-front-wall".to_owned(),
            ),
            (HOUSING_BACK_WALL_BODY_ID.0, "housing-back-wall".to_owned()),
            (OBSTACLE_BODY_ID.0, "injected-obstacle".to_owned()),
        ]);
        scene::build_scene_description(
            &self.machine_config_id,
            &self.machine_config_sha256,
            self.cell_config,
            &self.mechanics,
            &self.optics,
            &body_names,
        )
    }

    pub fn scene_frame(&self) -> SceneFrame {
        if let Some(snapshot) = &self.last_snapshot {
            snapshot.scene.clone()
        } else {
            let collision = self
                .mechanics
                .query_collisions_with_arms(self.mechanics.config.collision);
            scene::build_scene_frame(&self.mechanics, &collision.contacts)
        }
    }

    pub fn scene_description_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(&self.scene_description(), pretty)
    }

    pub fn scene_frame_json(&self, pretty: bool) -> Result<String, SimError> {
        serialize_json(&self.scene_frame(), pretty)
    }

    pub fn step(&mut self) -> Result<&StepSnapshot, SimError> {
        if self.is_terminal() {
            return self.last_snapshot.as_ref().ok_or_else(|| {
                SimError::InvalidScenario("terminal state has no snapshot".to_owned())
            });
        }
        self.cycle += 1;
        self.control_time_ms += CONTROL_PERIOD_MS;
        let active = self
            .executive
            .active_component()
            .map(|component| component.id);
        let phase = self.executive.phase();
        let fault = self.select_fault(active, phase);
        self.place_collision_obstacle(active, fault);
        let physics = self.advance_physics()?;
        let collision = self.audit_collisions(active, phase);
        let optical = self.measure_optics(active, fault);
        let (frame, mechanics) =
            self.make_sensor_frame(active, phase, fault, &optical, &physics, &collision);
        let decision = self.executive.tick(&frame);
        self.record_failures(&decision, fault);
        self.apply_command(&decision.command, fault)?;
        self.consume_fault(fault);
        self.mark_completed();
        let next_active = self
            .executive
            .active_component()
            .map(|component| component.id);
        sync_bodies(&mut self.mechanics, &self.components, next_active);

        let metrics = self.executive.metrics();
        let component_name = active.and_then(|id| self.component(id).map(|part| part.name.clone()));
        let pose_error = active.and_then(|id| self.component(id).map(|part| part.pose_error));
        let scene_collision = self
            .mechanics
            .query_collisions_with_arms(self.mechanics.config.collision);
        let scene = scene::build_scene_frame(&self.mechanics, &scene_collision.contacts);
        let report_snapshot = ReportSnapshot {
            cycle: self.cycle,
            control_time_ms: self.control_time_ms,
            component_id: active.map(|id| id.0),
            component_name,
            phase: phase.map(phase_name).map(str::to_owned),
            status: status_name(&decision.status).to_owned(),
            command: command_name(&decision.command).to_owned(),
            injected_fault: fault.map(InjectedFault::name).map(str::to_owned),
            optical,
            mechanics,
            pose_error,
            components_completed: metrics.components_completed,
            retries: metrics.retries,
            events: decision.events.iter().map(event_summary).collect(),
        };
        let snapshot = StepSnapshot {
            report: report_snapshot.clone(),
            scene,
        };
        self.snapshots.push(report_snapshot);
        self.last_snapshot = Some(snapshot);
        Ok(self.last_snapshot.as_ref().expect("snapshot just set"))
    }

    pub fn run_to_completion(&mut self, max_cycles: u32) -> Result<SimulationReport, SimError> {
        while !self.is_terminal() && self.cycle < u64::from(max_cycles) {
            self.step()?;
        }
        if !self.is_terminal() {
            return Err(SimError::CycleLimit(max_cycles));
        }
        Ok(self.report())
    }

    pub fn report(&self) -> SimulationReport {
        SimulationReport {
            schema_version: REPORT_SCHEMA_VERSION,
            scenario: self.spec.name.clone(),
            recipe: self.scenario.recipe.name.clone(),
            seed: self.spec.seed,
            configuration_sha256: self.spec.configuration_sha256.clone(),
            scenario_sha256: self.spec.scenario_sha256.clone(),
            cad_parameter_sha256: self.spec.cad_parameter_sha256.clone(),
            cad_geometry_facts_sha256: self.spec.cad_geometry_facts_sha256.clone(),
            status: status_name(self.executive.status()).to_owned(),
            completed: self.is_completed(),
            control_cycles: self.cycle,
            control_time_ms: self.control_time_ms,
            physics_time_s: self.mechanics.time_s,
            metrics: TaskMetricReport::from(self.executive.metrics()),
            failures: self.failures.clone(),
            components: self
                .components
                .iter()
                .map(|part| ComponentReport {
                    id: part.id.0,
                    name: part.name.clone(),
                    completed: part.completed,
                    seated: part.seated,
                    insertion_depth_mm: part.depth_mm,
                    final_pose_error_mm: part.pose_error.norm_mm(),
                    mesh_sweep_deg: part.mesh_sweep_deg,
                    peak_mesh_torque_mn_mm: part.mesh_torque_mn_mm,
                    backlash_mm: part.backlash_mm,
                })
                .collect(),
            gear_train_acceptance: self.gear_train_acceptance(),
            fidelity: FidelityReport {
                tier: "F1-reduced",
                mechanics_model: "reduced observed-pose/force assembly plant plus fixed-step tendon compliance, backlash, gripper and rigid-body state; not the normative F1 hardware-feasibility gate",
                collision_model: "feature-aware analytic signed-distance checks for moving/placed parts, the registered five-piece housing shell, and obstacle; uncoupled arm proxies are diagnostic-only and excluded from the assembly safety gate",
                optics_model: "ray-gated synthetic active-feature pose sensing over multi-part sphere proxies, with distinct-view observability, projector occlusion, quantization, covariance and seeded photon noise; no full CAD-image pose estimator",
                controller_model: "guarded task executive driven only by simulated sensor packets",
                part_model: "idealized 2PP nominal envelopes; exact print defects intentionally excluded",
                excluded_physics: vec![
                    "2PP voxel/process variation",
                    "resin cure shrinkage and surface adhesion",
                    "full finite-element tendon/backbone deformation",
                    "fluid and electrostatic effects",
                    "tooth-resolved gear contact (the reduced tier uses analytic pitch constraints)",
                    "multi-arm trajectory validation (uncoupled diagnostic arm proxies only)",
                    "CAD-mesh optical rendering and image-derived 6D pose estimation",
                ],
            },
            snapshots: self.snapshots.clone(),
        }
    }

    pub fn report_json(&self, pretty: bool) -> Result<String, SimError> {
        self.report().to_json(pretty)
    }

    fn gear_train_acceptance(&self) -> GearTrainAcceptance {
        let input_plan = self
            .scenario
            .recipe
            .components
            .iter()
            .find(|plan| {
                plan.kind == ComponentKind::Gear && plan.name.to_ascii_lowercase().contains("input")
            })
            .expect("validated recipe has an input gear");
        let idler_plan = self
            .scenario
            .recipe
            .components
            .iter()
            .find(|plan| {
                plan.kind == ComponentKind::Gear && plan.name.to_ascii_lowercase().contains("idler")
            })
            .expect("validated recipe has an idler gear");
        let output_plan = self
            .scenario
            .recipe
            .components
            .iter()
            .find(|plan| {
                plan.kind == ComponentKind::Gear
                    && plan.name.to_ascii_lowercase().contains("output")
            })
            .expect("validated recipe has an output gear");
        let input_teeth = gear_teeth(input_plan.geometry).expect("input is a gear");
        let idler_teeth = gear_teeth(idler_plan.geometry).expect("idler is a gear");
        let output_teeth = gear_teeth(output_plan.geometry).expect("output is a gear");
        let module_mm = match input_plan.geometry {
            ComponentGeometry::SpurGear { module_mm, .. } => module_mm,
            _ => unreachable!("validated input plan is a gear"),
        };
        let expected_reduction = f64::from(output_teeth) / f64::from(input_teeth);
        let input_state = self
            .component(input_plan.id)
            .expect("runtime state mirrors the validated recipe");
        let idler_state = self
            .component(idler_plan.id)
            .expect("runtime state mirrors the validated recipe");
        let mesh_backlash_mm = [input_state.backlash_mm, idler_state.backlash_mm];
        let mesh_gates_pass = [input_state, idler_state]
            .iter()
            .all(|part| part.mesh_sweep_deg > 0.0 && part.teeth_engaged)
            && mesh_backlash_mm
                .iter()
                .all(|backlash| (*backlash - 0.020).abs() <= 1.0e-12);
        let cover = self.components.iter().find(|part| {
            self.scenario
                .recipe
                .components
                .iter()
                .find(|plan| plan.id == part.id)
                .is_some_and(|plan| plan.kind == ComponentKind::Retainer)
        });
        let cover_latch_signatures = [
            cover.is_some_and(|part| part.closure_features_confirmed >= 1),
            cover.is_some_and(|part| part.closure_features_confirmed >= 2),
        ];
        let all_insertions_accepted = self.components.iter().all(|part| {
            let plan = self
                .scenario
                .recipe
                .components
                .iter()
                .find(|plan| plan.id == part.id)
                .expect("runtime component comes from recipe");
            part.completed
                && part.seated
                && (plan.insertion_travel_mm - part.depth_mm).abs()
                    <= plan.insertion_depth_tolerance_mm
        });
        let measurement_ready = all_insertions_accepted && mesh_gates_pass;
        let mut train = ReducedGearTrainState::new(
            input_teeth,
            idler_teeth,
            output_teeth,
            module_mm,
            mesh_backlash_mm,
        );
        let (forward, reverse) = if measurement_ready {
            // Preload the forward flank, then zero the virtual optical angle
            // encoders exactly as the physical test would before its ratio run.
            train.drive_input(0.10);
            train.zero_encoders();
            train.drive_input(10.0);
            let forward = TurnAcceptance {
                input_turns: train.input_turns,
                output_turns: train.output_turns,
            };

            // Reverse from the loaded forward flank. The measured output is
            // slightly under one turn because both meshes consume backlash.
            train.zero_encoders();
            train.drive_input(-2.0);
            let reverse = TurnAcceptance {
                input_turns: train.input_turns,
                output_turns: train.output_turns,
            };
            (forward, reverse)
        } else {
            (
                TurnAcceptance {
                    input_turns: 0.0,
                    output_turns: 0.0,
                },
                TurnAcceptance {
                    input_turns: 0.0,
                    output_turns: 0.0,
                },
            )
        };
        let observed_reduction = if forward.output_turns.abs() > f64::EPSILON {
            forward.input_turns.abs() / forward.output_turns.abs()
        } else {
            0.0
        };
        let same_direction = measurement_ready
            && forward.input_turns * forward.output_turns > 0.0
            && reverse.input_turns * reverse.output_turns > 0.0;
        let reverse_expected_turns = reverse.input_turns / expected_reduction;
        let ratio_pass = (observed_reduction - expected_reduction).abs() <= 1.0e-9
            && (reverse.output_turns - reverse_expected_turns).abs() <= 0.02;
        let accepted = self.is_completed()
            && measurement_ready
            && cover_latch_signatures == [true, true]
            && same_direction
            && ratio_pass;
        GearTrainAcceptance {
            method: "stateful_analytic_backlash_constraint_reduced",
            measurement_source: "post-run analytic check gated by completed insertion, mesh, and latch observations; not a modeled rotary-tool measurement",
            executed_in_task_loop: false,
            tooth_resolved_contact: false,
            input_teeth,
            output_teeth,
            expected_reduction,
            observed_reduction,
            same_direction,
            forward,
            reverse,
            mesh_backlash_mm,
            cover_latch_signatures,
            accepted,
        }
    }

    fn component(&self, id: ComponentId) -> Option<&ComponentState> {
        self.components.iter().find(|part| part.id == id)
    }

    fn component_mut(&mut self, id: ComponentId) -> Option<&mut ComponentState> {
        self.components.iter_mut().find(|part| part.id == id)
    }

    fn component_index(&self, id: ComponentId) -> usize {
        self.components
            .iter()
            .position(|part| part.id == id)
            .unwrap_or(0)
    }

    fn select_fault(
        &self,
        component: Option<ComponentId>,
        phase: Option<Phase>,
    ) -> Option<InjectedFault> {
        let (id, phase) = (component?.0, phase?);
        if self.spec.faults.occlusion
            && !self.faults.occlusion_used
            && id == 4
            && phase == Phase::Locate
        {
            Some(InjectedFault::Occlusion)
        } else if self.spec.faults.collision
            && !self.faults.collision_used
            && id == 4
            && phase == Phase::Align
        {
            Some(InjectedFault::Collision)
        } else if self.spec.faults.insertion_force
            && !self.faults.insertion_force_used
            && id == 3
            && phase == Phase::Insert
        {
            Some(InjectedFault::InsertionForce)
        } else {
            None
        }
    }

    fn consume_fault(&mut self, fault: Option<InjectedFault>) {
        match fault {
            Some(InjectedFault::Collision) => self.faults.collision_used = true,
            Some(InjectedFault::Occlusion) => self.faults.occlusion_used = true,
            Some(InjectedFault::InsertionForce) => self.faults.insertion_force_used = true,
            None => {}
        }
    }

    fn mark_completed(&mut self) {
        let count = self.executive.metrics().components_completed as usize;
        for (index, part) in self.components.iter_mut().enumerate() {
            part.completed = index < count;
        }
    }

    fn place_collision_obstacle(
        &mut self,
        component: Option<ComponentId>,
        fault: Option<InjectedFault>,
    ) {
        let position = component
            .and_then(|id| self.component(id))
            .map(world_position);
        if let Some(obstacle) = self.mechanics.body_mut(OBSTACLE_BODY_ID) {
            obstacle.enabled = fault == Some(InjectedFault::Collision);
            if let Some(position) = position {
                // The fault is injected against G3's 0.275 mm full-height hub:
                // 0.275 + 0.150 - 0.400 = 0.025 mm penetration. This keeps the
                // event independent of contact-offset rounding without an
                // unrealistically deep overlap.
                obstacle.pose.translation = position + Vec3::new(0.400e-3, 0.0, 0.0);
            }
        }
    }

    fn advance_physics(&mut self) -> Result<StepReport, SimError> {
        let mut report = None;
        for _ in 0..20 {
            report = Some(
                self.mechanics
                    .step()
                    .map_err(|error| SimError::Mechanics(format!("{error:?}")))?,
            );
        }
        Ok(report.expect("positive step count"))
    }

    fn audit_collisions(
        &self,
        active: Option<ComponentId>,
        phase: Option<Phase>,
    ) -> CollisionAudit {
        let report = self
            .mechanics
            .query_collisions_with_arms(self.mechanics.config.collision);
        let intended_pairs =
            intended_collision_pairs(&self.scenario, &self.components, active, phase);
        let mut intended_contact_count = 0;
        let mut unplanned_contact_count = 0;
        let mut unplanned_clearance_count = 0;
        let mut diagnostic_arm_contact_count = 0;
        let mut diagnostic_arm_clearance_count = 0;
        // With no returned near pair the analytic query establishes only this
        // lower bound, not an invented 1 mm measured clearance.
        let mut min_unplanned_clearance_mm: f64 =
            self.mechanics.config.collision.clearance_threshold_m * 1_000.0;
        let mut maximum_unplanned_penetration_m: f64 = 0.0;

        for contact in &report.contacts {
            if is_arm_body(contact.body_a) || is_arm_body(contact.body_b) {
                diagnostic_arm_contact_count += 1;
            } else if intended_pairs.contains(&canonical_body_pair(contact.body_a, contact.body_b))
            {
                intended_contact_count += 1;
            } else {
                unplanned_contact_count += 1;
                min_unplanned_clearance_mm = 0.0;
                maximum_unplanned_penetration_m =
                    maximum_unplanned_penetration_m.max(contact.penetration_depth_m);
            }
        }
        for clearance in &report.clearances {
            if is_arm_body(clearance.body_a) || is_arm_body(clearance.body_b) {
                diagnostic_arm_clearance_count += 1;
            } else if !intended_pairs
                .contains(&canonical_body_pair(clearance.body_a, clearance.body_b))
            {
                unplanned_clearance_count += 1;
                min_unplanned_clearance_mm =
                    min_unplanned_clearance_mm.min(clearance.distance_m.max(0.0) * 1_000.0);
            }
        }

        CollisionAudit {
            report,
            intended_contact_count,
            unplanned_contact_count,
            unplanned_clearance_count,
            diagnostic_arm_contact_count,
            diagnostic_arm_clearance_count,
            min_unplanned_clearance_mm,
            maximum_unplanned_penetration_m,
        }
    }

    fn measure_optics(
        &self,
        active: Option<ComponentId>,
        fault: Option<InjectedFault>,
    ) -> OpticalTelemetry {
        let Some(id) = active else {
            return OpticalTelemetry::default();
        };
        let Some(part) = self.component(id) else {
            return OpticalTelemetry::default();
        };
        let visible_candidates = self
            .components
            .iter()
            .filter(|candidate| candidate.completed || candidate.id == id)
            .collect::<Vec<_>>();
        let candidate_tags = visible_candidates
            .iter()
            .map(|candidate| u32::from(candidate.id.0))
            .collect::<BTreeSet<_>>();
        // The observer head is represented in its commanded local measurement
        // pose around the active component. Subtract only the nominal active
        // pose; the physical pose error remains in the optical measurement.
        let optical_frame_origin = optical_vec(nominal_world_position(part));
        let scene = optical_scene(
            &visible_candidates,
            optical_frame_origin,
            (fault == Some(InjectedFault::Occlusion)).then_some(id),
        );
        let mut result = OpticalTelemetry {
            expected_visible_components: visible_candidates.len() as u32,
            ..OpticalTelemetry::default()
        };
        let mut camera_confidence_sums = vec![0.0; self.optics.cameras.len()];
        let mut camera_sigma_sums_mm = vec![0.0; self.optics.cameras.len()];
        let mut camera_return_counts = vec![0_u32; self.optics.cameras.len()];
        let mut range_error_sum_um = 0.0;
        let mut visible_tags = BTreeSet::new();
        let mut target_views = BTreeSet::new();
        for candidate in &visible_candidates {
            let candidate_tag = u32::from(candidate.id.0);
            let center = optical_vec(world_position(candidate)) - optical_frame_origin;
            for (camera_index, camera) in self.optics.cameras.iter().enumerate() {
                let Some(projected) = camera.nominal.project(center) else {
                    continue;
                };
                // Five rays support the active-feature uncertainty gate. A
                // single centre ray is sufficient for coarse visibility of
                // each already-placed context part and avoids turning every
                // 20 ms control observation into a dense scene scan.
                let offsets: &[(f64, f64)] = if candidate.id == id {
                    &[(0.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)]
                } else {
                    &[(0.0, 0.0)]
                };
                for &(dx, dy) in offsets {
                    let x = (projected.pixel.x + dx).round();
                    let y = (projected.pixel.y + dy).round();
                    if x < 0.0 || y < 0.0 {
                        continue;
                    }
                    result.attempted_rays += 1;
                    if camera_index < 6 {
                        result.global_attempted_rays += 1;
                    } else {
                        result.macro_attempted_rays += 1;
                    }
                    match self.optics.sample_pixel(
                        &scene,
                        self.cycle,
                        camera_index,
                        x as u32,
                        y as u32,
                    ) {
                        DepthReturn::Sample(sample) => {
                            if candidate_tags.contains(&sample.primitive_tag) {
                                visible_tags.insert(sample.primitive_tag);
                            }
                            if candidate_tag == u32::from(id.0)
                                && sample.primitive_tag == candidate_tag
                            {
                                result.valid_target_returns += 1;
                                target_views.insert(camera_index);
                                if camera_index < 6 {
                                    result.global_valid_target_returns += 1;
                                } else {
                                    result.macro_valid_target_returns += 1;
                                }
                                camera_confidence_sums[camera_index] +=
                                    sample.quality.confidence.clamp(0.0, 1.0);
                                camera_sigma_sums_mm[camera_index] +=
                                    sample.covariance.rms_sigma_m() * 1_000.0;
                                camera_return_counts[camera_index] += 1;
                                range_error_sum_um += sample.range_error_m * 1.0e6;
                            } else if candidate_tag == u32::from(id.0)
                                && fault == Some(InjectedFault::Occlusion)
                            {
                                result.occluded_returns += 1;
                            }
                        }
                        DepthReturn::Missing { reason, .. } => match reason {
                            MissingReturn::NoSurface => result.no_surface_returns += 1,
                            MissingReturn::ProjectorOccluded => {
                                result.projector_coverage_failures += 1;
                                if candidate_tag == u32::from(id.0) {
                                    result.occluded_returns += 1;
                                }
                            }
                            MissingReturn::ProjectorOutOfView => {
                                result.projector_coverage_failures += 1;
                            }
                            MissingReturn::LowSignal | MissingReturn::StochasticDropout => {
                                result.signal_or_dropout_failures += 1;
                            }
                            MissingReturn::DegenerateGeometry
                            | MissingReturn::BehindSensor
                            | MissingReturn::ExcessiveRayResidual => {
                                result.geometric_rejections += 1;
                            }
                            MissingReturn::OutsideImage | MissingReturn::InvalidCalibration => {
                                result.invalid_returns += 1;
                            }
                        },
                    }
                }
            }
        }
        result.visible_components = visible_tags.len() as u32;
        result.valid_camera_views = target_views.len() as u32;
        result.global_valid_camera_views =
            target_views.iter().filter(|&&index| index < 6).count() as u32;
        result.macro_valid_camera_views =
            target_views.iter().filter(|&&index| index >= 6).count() as u32;
        if result.valid_camera_views >= MIN_INDEPENDENT_CAMERA_VIEWS {
            // Neighboring samples share a camera, calibration, projector and
            // speckle realization. Average confidence inside each view, then
            // combine only the independent camera observations. Sampling the
            // same image more densely must not manufacture confidence.
            result.confidence =
                fuse_camera_confidences(&camera_confidence_sums, &camera_return_counts);
            // Neighboring rays from one camera share calibration, projector,
            // and speckle error. Average them only within that view, fuse the
            // distinct view estimates by information, then retain a correlated
            // calibration floor. More pixels from one camera cannot manufacture
            // independent pose certainty.
            result.position_sigma_mm = fuse_camera_sigmas_mm(
                &camera_sigma_sums_mm,
                &camera_return_counts,
                OPTICAL_CORRELATED_FLOOR_MM,
            );
            result.mean_range_error_um =
                range_error_sum_um / f64::from(result.valid_target_returns);
            result.orientation_sigma_deg = (result.position_sigma_mm
                / orientation_feature_baseline_mm(part))
            .atan()
            .to_degrees()
            .max(0.04);
        } else {
            result.confidence = 0.05 * f64::from(result.valid_camera_views);
            result.position_sigma_mm = 0.050;
            result.orientation_sigma_deg = 5.0;
        }
        result
    }

    fn make_sensor_frame(
        &mut self,
        active: Option<ComponentId>,
        phase: Option<Phase>,
        fault: Option<InjectedFault>,
        optical: &OpticalTelemetry,
        physics: &StepReport,
        collision: &CollisionAudit,
    ) -> (SensorFrame, MechanicsTelemetry) {
        let current = active.and_then(|id| self.component(id)).cloned();
        let mechanics = MechanicsTelemetry {
            physics_step_index: physics.step_index,
            physics_time_s: physics.time_s,
            contact_count: collision.report.contacts.len(),
            near_clearance_count: collision.report.clearances.len(),
            intended_contact_count: collision.intended_contact_count,
            unplanned_contact_count: collision.unplanned_contact_count,
            unplanned_clearance_count: collision.unplanned_clearance_count,
            diagnostic_arm_contact_count: collision.diagnostic_arm_contact_count,
            diagnostic_arm_clearance_count: collision.diagnostic_arm_clearance_count,
            min_unplanned_clearance_mm: collision.min_unplanned_clearance_mm,
            maximum_penetration_um: collision.maximum_unplanned_penetration_m * 1.0e6,
            actuator_rms_tracking_error_um: actuator_rms_error_m(&self.mechanics) * 1.0e6,
            active_insertion_depth_mm: current.as_ref().map_or(0.0, |part| part.depth_mm),
            active_axial_force_n: current.as_ref().map_or(0.0, |part| part.axial_force_n),
            active_lateral_force_n: current.as_ref().map_or(0.0, |part| part.lateral_force_n),
        };
        let mut frame = SensorFrame::empty(self.control_time_ms);
        frame.capture_time_ms = self.control_time_ms.saturating_sub(12);
        frame.min_unplanned_clearance_mm = collision.min_unplanned_clearance_mm;
        frame.unplanned_contact_force_n =
            (collision.maximum_unplanned_penetration_m * 1_000.0).min(0.10);
        let (Some(id), Some(part)) = (active, current) else {
            return (frame, mechanics);
        };
        let pose_error = part
            .pose_error
            .observed(optical_bias(self.cycle, optical.mean_range_error_um));
        let vision = VisionObservation {
            component: id,
            confidence: optical.confidence,
            position_sigma_mm: optical.position_sigma_mm,
            orientation_sigma_deg: optical.orientation_sigma_deg,
            target_error: pose_error,
        };
        match phase {
            Some(Phase::Locate) | Some(Phase::Pick) => frame.vision = Some(vision),
            Some(Phase::Handoff) => {
                frame.handoff = Some(HandoffObservation {
                    component: id,
                    target_error: pose_error,
                    receiver_force_n: part.receiver_force_n,
                    receiver_has_part: part.receiver_has_part,
                    donor_released: part.donor_released,
                })
            }
            Some(Phase::Align) => {
                frame.alignment = Some(AlignmentObservation {
                    component: id,
                    bore_error: pose_error,
                })
            }
            Some(Phase::Insert) => {
                let mut axial_force_n = insertion_force(&part, &self.scenario);
                if fault == Some(InjectedFault::InsertionForce) {
                    let active_limit_n = self
                        .scenario
                        .recipe
                        .components
                        .iter()
                        .find(|plan| plan.id == id)
                        .expect("active component comes from recipe")
                        .max_insertion_axial_force_n;
                    axial_force_n = active_limit_n + 0.010;
                }
                let lateral_stiffness_n_per_mm = match part.plan.kind {
                    // The 70 um diametral running clearance leaves a gear
                    // considerably less laterally constrained than a shaft or
                    // the cover rim during guarded descent.
                    ComponentKind::Gear => 0.20,
                    _ => 0.38,
                };
                let lateral_force_n = 0.0008
                    + lateral_stiffness_n_per_mm
                        * part.pose_error.translation_mm[0]
                            .hypot(part.pose_error.translation_mm[1]);
                if let Some(runtime) = self.component_mut(id) {
                    runtime.axial_force_n = axial_force_n;
                    runtime.lateral_force_n = lateral_force_n;
                }
                frame.insertion = Some(InsertionObservation {
                    component: id,
                    depth_mm: part.depth_mm,
                    axial_force_n,
                    lateral_force_n,
                    seated: part.seated,
                    closure_features_confirmed: part.closure_features_confirmed,
                });
            }
            Some(Phase::Mesh) => {
                frame.mesh = Some(MeshObservation {
                    component: id,
                    sweep_deg: part.mesh_sweep_deg,
                    peak_torque_mn_mm: part.mesh_torque_mn_mm,
                    backlash_mm: part.backlash_mm,
                    teeth_engaged: part.teeth_engaged,
                })
            }
            Some(Phase::Verify) => {
                frame.verification = Some(VerificationObservation {
                    component: id,
                    target_error: pose_error,
                    vision_confidence: optical.confidence,
                    rotation_test_deg: part.verification_rotation_deg,
                    peak_torque_mn_mm: verification_torque(&part),
                    torque_ripple_fraction: 0.09,
                    backlash_mm: part.backlash_mm,
                    required_features_visible: optical.valid_camera_views
                        >= MIN_INDEPENDENT_CAMERA_VIEWS,
                })
            }
            None => {}
        }
        if phase == Some(Phase::Pick) {
            frame.gripper = Some(GripperObservation {
                component: id,
                force_n: part.grip_force_n,
                slip_mm: if part.retained { 0.0005 } else { 0.0 },
                part_retained: part.retained,
            });
        }
        (frame, mechanics)
    }

    fn apply_command(
        &mut self,
        command: &Command,
        fault: Option<InjectedFault>,
    ) -> Result<(), SimError> {
        command_arm_motion(&mut self.mechanics, command)?;
        let gain = plant_gain(&self.mechanics);
        match command {
            Command::SearchVolume { .. } | Command::HoldPosition | Command::AssemblyComplete => {}
            Command::MoveToPregrasp { component, .. } => {
                let index = self.component_index(*component);
                if let Some(part) = self.component_mut(*component) {
                    part.pose_error = seeded_pose(index, 1.0);
                }
            }
            Command::VisualServo {
                component,
                correction,
                ..
            } => {
                if let Some(part) = self.component_mut(*component) {
                    part.pose_error.corrected(
                        correction.translation_mm,
                        correction.rotation_deg,
                        gain,
                    );
                }
            }
            Command::CloseTendonGripper {
                component,
                target_force_n,
            } => {
                if let Some(part) = self.component_mut(*component) {
                    part.grip_force_n = target_force_n * (0.96 + 0.02 * gain);
                    part.retained = true;
                }
            }
            Command::PresentForHandoff { component } => {
                let index = self.component_index(*component);
                if let Some(part) = self.component_mut(*component) {
                    part.pose_error = seeded_pose(index, 0.8);
                }
            }
            Command::CloseReceiverGripper {
                component,
                target_force_n,
            } => {
                if let Some(part) = self.component_mut(*component) {
                    part.receiver_force_n = target_force_n * (0.97 + 0.01 * gain);
                    part.receiver_has_part = true;
                }
            }
            Command::ReleaseDonorGripper { component } => {
                if let Some(part) = self.component_mut(*component) {
                    part.donor_released = true;
                }
            }
            Command::MoveToBore { component, .. } => {
                let index = self.component_index(*component);
                if let Some(part) = self.component_mut(*component) {
                    // The reduced collision plant represents a part only after
                    // the arm has delivered it to the insertion approach. Keep
                    // the deterministic initial error inside the 10 um fixture
                    // safety margin while leaving work for visual servoing.
                    part.pose_error = seeded_pose(index, 0.20);
                    part.at_insertion_approach = true;
                }
            }
            Command::GuardedInsert {
                component,
                target_depth_mm,
                speed_mm_s,
                ..
            } => {
                let seating_tolerance_mm = self
                    .scenario
                    .recipe
                    .components
                    .iter()
                    .find(|plan| plan.id == *component)
                    .map(|plan| plan.insertion_depth_tolerance_mm)
                    .unwrap_or(0.0);
                if let Some(part) = self.component_mut(*component) {
                    if fault != Some(InjectedFault::InsertionForce) {
                        let travel = speed_mm_s * CONTROL_PERIOD_MS as f64 / 1_000.0 * gain;
                        part.depth_mm = (part.depth_mm + travel).min(*target_depth_mm);
                        part.seated =
                            (*target_depth_mm - part.depth_mm).abs() <= seating_tolerance_mm;
                        if part.seated {
                            part.pose_error.translation_mm[2] = 0.002;
                        }
                    }
                }
            }
            Command::CloseRetainerFeature {
                component, feature, ..
            } => {
                if let Some(part) = self.component_mut(*component) {
                    part.closure_features_confirmed = part.closure_features_confirmed.max(*feature);
                    part.axial_force_n = 0.010 + 0.001 * f64::from(*feature);
                }
            }
            Command::MeshDither {
                component,
                delta_deg,
                ..
            } => {
                if let Some(part) = self.component_mut(*component) {
                    part.mesh_sweep_deg += delta_deg.abs() * (0.98 + 0.01 * gain);
                    part.mesh_torque_mn_mm =
                        0.052 + 0.004 * (part.mesh_sweep_deg / 12.0).sin().abs();
                    part.teeth_engaged = part.mesh_sweep_deg >= 12.0;
                }
            }
            Command::RunVerification {
                component,
                rotation_deg,
                ..
            } => {
                if let Some(part) = self.component_mut(*component) {
                    part.verification_rotation_deg =
                        (part.verification_rotation_deg + 32.0 * gain).min(*rotation_deg);
                    part.pose_error = PoseState {
                        translation_mm: [0.0020, -0.0015, 0.0020],
                        rotation_deg: [0.08, -0.05, 0.06],
                    };
                }
            }
            Command::RetractAndReacquire {
                component,
                distance_mm,
            } => {
                let index = self.component_index(*component);
                if let Some(part) = self.component_mut(*component) {
                    part.depth_mm = (part.depth_mm - distance_mm).max(0.0);
                    part.seated = false;
                    // Recovery returns to a pose inside the declared capture
                    // gate so the correct seat may remain intended while the
                    // visual servo removes the residual error. Grossly wrong
                    // poses stay outside the allowlist and fail closed.
                    part.pose_error = seeded_pose(index, 0.35);
                }
            }
        }
        Ok(())
    }

    fn record_failures(&mut self, decision: &Decision, fault: Option<InjectedFault>) {
        if let Some(fault) = fault {
            self.failures.push(format!(
                "cycle {}: injected {} perturbation",
                self.cycle,
                fault.name()
            ));
        }
        for event in &decision.events {
            match &event.kind {
                EventKind::GateRejected { reason } | EventKind::Aborted { reason } => {
                    self.failures.push(format!(
                        "cycle {} / component {:?} / phase {:?}: {}",
                        self.cycle,
                        event.component.map(|id| id.0),
                        event.phase,
                        failure_name(reason)
                    ));
                }
                _ => {}
            }
        }
    }
}

fn canonical_body_pair(a: BodyId, b: BodyId) -> (BodyId, BodyId) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

fn is_arm_body(id: BodyId) -> bool {
    id.0 >= SERIAL_ARM_COLLISION_BODY_ID_BASE
}

fn intended_collision_pairs(
    scenario: &AssemblyScenario,
    components: &[ComponentState],
    active: Option<ComponentId>,
    phase: Option<Phase>,
) -> Vec<(BodyId, BodyId)> {
    let (Some(active), Some(phase)) = (active, phase) else {
        return Vec::new();
    };
    let mut definitions = scenario.recipe.intended_contacts_for(active, phase);
    let active_state = components.iter().find(|part| part.id == active);

    // Whole-body reduced proxies may suppress only a declared feature contact
    // after the part has reached the insertion approach and its measured pose
    // is inside the capture gate. This applies equally to shaft seats, gear
    // bores/teeth/floors, and the cover rim: striking the correct mating body
    // from a grossly wrong pose is still an unplanned collision.
    let aligned_for_capture = active_state.is_some_and(|part| {
        let lateral_mm = part.pose_error.translation_mm[0].hypot(part.pose_error.translation_mm[1]);
        let rotation_deg = part.pose_error.rotation_deg[0]
            .hypot(part.pose_error.rotation_deg[1])
            .hypot(part.pose_error.rotation_deg[2]);
        part.at_insertion_approach
            && lateral_mm <= scenario.acceptance.alignment.max_lateral_error_mm
            && part.pose_error.translation_mm[2].abs()
                <= scenario.acceptance.alignment.max_axial_error_mm
            && rotation_deg <= scenario.acceptance.alignment.max_rotation_error_deg
    });
    if !aligned_for_capture {
        definitions.retain(|definition| definition.activates_with != active);
    }

    // The single reduced floor proxy cannot contain three literal shaft-seat
    // holes. Admit only the active shaft/floor seat relation while the shaft
    // is staged at the bore; side-wall contacts remain unplanned.
    let active_is_shaft = active_state.is_some_and(|part| part.plan.kind == ComponentKind::Shaft);
    if aligned_for_capture && matches!(phase, Phase::Align | Phase::Insert) && active_is_shaft {
        definitions.extend(
            scenario
                .recipe
                .intended_contacts
                .iter()
                .filter(|definition| {
                    definition.activates_with == active
                        && definition.kind == AssemblyContactKind::ShaftSeat
                }),
        );
    }

    // The reduced plant stages a located gear coaxially over its assigned
    // shaft before visual-servo phases begin. Keep that declared bore/shaft
    // capture relation active for the current gear; no other part pair is
    // suppressed, so a wrong-shaft or wall approach still fails closed.
    let active_is_gear = active_state.is_some_and(|part| part.plan.kind == ComponentKind::Gear);
    if aligned_for_capture && matches!(phase, Phase::Align | Phase::Insert) && active_is_gear {
        definitions.extend(
            scenario
                .recipe
                .intended_contacts
                .iter()
                .filter(|definition| {
                    definition.activates_with == active
                        && definition.kind == AssemblyContactKind::BoreOnShaft
                }),
        );
    }

    // Meshing envelopes begin to overlap at the final bore-alignment approach
    // and continue through guarded descent, before the final seated sample
    // advances the executive to Mesh. Enable only the declared current gear
    // pair; unrelated gear or wall encounters remain unplanned.
    if aligned_for_capture && matches!(phase, Phase::Align | Phase::Insert) && active_is_gear {
        definitions.extend(
            scenario
                .recipe
                .intended_contacts
                .iter()
                .filter(|definition| {
                    definition.activates_with == active
                        && definition.kind == AssemblyContactKind::GearMesh
                }),
        );
    }

    let mut pairs = Vec::new();
    for definition in definitions {
        match definition.kind {
            AssemblyContactKind::ShaftSeat => pairs.push(canonical_body_pair(
                HOUSING_FLOOR_BODY_ID,
                BodyId(u32::from(definition.activates_with.0)),
            )),
            AssemblyContactKind::BoreOnShaft | AssemblyContactKind::GearMesh => {
                if let (Some(a), Some(b)) = (
                    definition.feature_a.component(),
                    definition.feature_b.component(),
                ) {
                    pairs.push(canonical_body_pair(
                        BodyId(u32::from(a.0)),
                        BodyId(u32::from(b.0)),
                    ));
                }
            }
            AssemblyContactKind::GearOnFloor => {
                if let Some(gear) = definition
                    .feature_a
                    .component()
                    .or_else(|| definition.feature_b.component())
                {
                    pairs.push(canonical_body_pair(
                        HOUSING_FLOOR_BODY_ID,
                        BodyId(u32::from(gear.0)),
                    ));
                }
            }
            AssemblyContactKind::CoverRim | AssemblyContactKind::CoverLatch => {
                let cover = definition
                    .feature_a
                    .component()
                    .or_else(|| definition.feature_b.component())
                    .unwrap_or(definition.activates_with);
                for wall in [
                    HOUSING_LEFT_WALL_BODY_ID,
                    HOUSING_RIGHT_WALL_BODY_ID,
                    HOUSING_FRONT_WALL_BODY_ID,
                    HOUSING_BACK_WALL_BODY_ID,
                ] {
                    pairs.push(canonical_body_pair(wall, BodyId(u32::from(cover.0))));
                }
            }
        }
    }
    pairs.sort_unstable();
    pairs.dedup();
    pairs
}

fn build_mechanics(
    scenario: &AssemblyScenario,
    cell_config: PipeCellConfig,
) -> Result<Simulation, SimError> {
    let config = SimulationConfig {
        fixed_dt_s: 0.001,
        gravity_m_s2: Vec3::new(0.0, 0.0, -9.80665),
        collision: pipe_sim_core::CollisionSettings {
            clearance_threshold_m: 0.100e-3,
            ..pipe_sim_core::CollisionSettings::default()
        },
        ..SimulationConfig::default()
    };
    let mut simulation =
        Simulation::new(config).map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
    for part in &scenario.recipe.components {
        let mut body = RigidBody::new(
            BodyId(u32::from(part.id.0)),
            component_shape(part.geometry),
            Pose::IDENTITY,
            MotionType::Kinematic,
        );
        body.user_tag = u32::from(part.id.0);
        body.collision_filter = CollisionFilter {
            group: PART_GROUP,
            mask: PART_GROUP | OBSTACLE_GROUP | FIXTURE_GROUP,
        };
        body.enabled = false;
        simulation
            .add_body(body)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
    }

    add_housing_shell(&mut simulation, &scenario.recipe.registered_housing)?;

    let mut obstacle = RigidBody::new(
        OBSTACLE_BODY_ID,
        Shape::Sphere { radius_m: 0.150e-3 },
        Pose::IDENTITY,
        MotionType::Static,
    );
    obstacle.enabled = false;
    obstacle.collision_filter = CollisionFilter {
        group: OBSTACLE_GROUP,
        mask: PART_GROUP,
    };
    simulation
        .add_body(obstacle)
        .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;

    for arm_index in 0..cell_config.manipulator_count {
        let theta =
            f64::from(arm_index) * std::f64::consts::TAU / f64::from(cell_config.manipulator_count);
        let mut arm = SerialArm::new(cell_config.arm)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        arm.set_positions(SerialJointPositions {
            base_z_m: 0.0,
            base_theta_rad: theta,
            shoulder_yaw_rad: 0.0,
            // Park longitudinally along the tube wall. The acceptance plant
            // below does not claim these are executed assembly trajectories.
            shoulder_pitch_rad: std::f64::consts::FRAC_PI_2,
            elbow_pitch_rad: 0.0,
            wrist_roll_rad: 0.0,
        })
        .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        let mut instance =
            SerialArmInstance::new(ArmId(u32::from(arm_index) + 1), arm, cell_config.gripper)
                .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        instance.carriage_config = cell_config.carriage;
        instance.motion_config = cell_config.motion;
        instance.tool_motion_speed_scale = cell_config.safety.commissioning_speed_scale;
        simulation
            .add_serial_arm(instance)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
    }
    Ok(simulation)
}

fn add_housing_shell(
    simulation: &mut Simulation,
    housing: &pipe_planner::HousingPlan,
) -> Result<(), SimError> {
    let ComponentGeometry::Housing {
        outer_size_mm,
        wall_mm,
    } = housing.geometry
    else {
        return Err(SimError::InvalidScenario(
            "registered housing must use Housing geometry".to_owned(),
        ));
    };
    let origin_mm: [f64; 3] = std::array::from_fn(|axis| {
        housing.target_pose.position_mm[axis] - outer_size_mm[axis] * 0.5
    });
    let floor_mm = housing.shaft_seat_depth_mm;
    let wall_height_mm = outer_size_mm[2] - floor_mm;
    let wall_center_z_mm = origin_mm[2] + floor_mm + wall_height_mm * 0.5;
    let mut add_box = |id: BodyId, center_mm: [f64; 3], half_mm: [f64; 3]| {
        let mut body = RigidBody::new(
            id,
            Shape::Box {
                half_extents_m: Vec3::new(
                    half_mm[0] * 1.0e-3,
                    half_mm[1] * 1.0e-3,
                    half_mm[2] * 1.0e-3,
                ),
            },
            Pose::from_translation(Vec3::new(
                center_mm[0] * 1.0e-3,
                center_mm[1] * 1.0e-3,
                center_mm[2] * 1.0e-3,
            )),
            MotionType::Static,
        );
        body.user_tag = id.0;
        body.collision_filter = CollisionFilter {
            group: FIXTURE_GROUP,
            mask: PART_GROUP,
        };
        simulation
            .add_body(body)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))
    };

    add_box(
        HOUSING_FLOOR_BODY_ID,
        [
            housing.target_pose.position_mm[0],
            housing.target_pose.position_mm[1],
            origin_mm[2] + floor_mm * 0.5,
        ],
        [
            outer_size_mm[0] * 0.5,
            outer_size_mm[1] * 0.5,
            floor_mm * 0.5,
        ],
    )?;
    add_box(
        HOUSING_LEFT_WALL_BODY_ID,
        [
            origin_mm[0] + wall_mm * 0.5,
            housing.target_pose.position_mm[1],
            wall_center_z_mm,
        ],
        [wall_mm * 0.5, outer_size_mm[1] * 0.5, wall_height_mm * 0.5],
    )?;
    add_box(
        HOUSING_RIGHT_WALL_BODY_ID,
        [
            origin_mm[0] + outer_size_mm[0] - wall_mm * 0.5,
            housing.target_pose.position_mm[1],
            wall_center_z_mm,
        ],
        [wall_mm * 0.5, outer_size_mm[1] * 0.5, wall_height_mm * 0.5],
    )?;
    let inner_x_half_mm = (outer_size_mm[0] - 2.0 * wall_mm) * 0.5;
    add_box(
        HOUSING_FRONT_WALL_BODY_ID,
        [
            housing.target_pose.position_mm[0],
            origin_mm[1] + wall_mm * 0.5,
            wall_center_z_mm,
        ],
        [inner_x_half_mm, wall_mm * 0.5, wall_height_mm * 0.5],
    )?;
    add_box(
        HOUSING_BACK_WALL_BODY_ID,
        [
            housing.target_pose.position_mm[0],
            origin_mm[1] + outer_size_mm[1] - wall_mm * 0.5,
            wall_center_z_mm,
        ],
        [inner_x_half_mm, wall_mm * 0.5, wall_height_mm * 0.5],
    )?;
    Ok(())
}

fn component_shape(geometry: ComponentGeometry) -> Shape {
    match geometry {
        ComponentGeometry::Housing { outer_size_mm, .. }
        | ComponentGeometry::Retainer { outer_size_mm, .. } => Shape::Box {
            half_extents_m: Vec3::new(
                outer_size_mm[0] * 0.5e-3,
                outer_size_mm[1] * 0.5e-3,
                outer_size_mm[2] * 0.5e-3,
            ),
        },
        ComponentGeometry::Shaft {
            diameter_mm,
            length_mm,
        } => Shape::Capsule {
            radius_m: diameter_mm * 0.5e-3,
            half_segment_m: (length_mm * 0.5 - diameter_mm * 0.5).max(0.0) * 1.0e-3,
        },
        ComponentGeometry::SpurGear {
            module_mm,
            teeth,
            pressure_angle_deg,
            thickness_mm,
            bore_diameter_mm,
            hub_diameter_mm,
            total_height_mm,
        } => Shape::Gear(GearGeometry::spur_with_hub(
            teeth,
            module_mm * 1.0e-3,
            pressure_angle_deg.to_radians(),
            thickness_mm * 1.0e-3,
            bore_diameter_mm * 0.5e-3,
            hub_diameter_mm * 0.5e-3,
            total_height_mm * 1.0e-3,
        )),
    }
}

fn gear_teeth(geometry: ComponentGeometry) -> Option<u16> {
    match geometry {
        ComponentGeometry::SpurGear { teeth, .. } => Some(teeth),
        _ => None,
    }
}

fn sync_bodies(
    mechanics: &mut Simulation,
    components: &[ComponentState],
    active: Option<ComponentId>,
) {
    for part in components {
        if let Some(body) = mechanics.body_mut(part.body_id) {
            body.enabled =
                part.completed || (active == Some(part.id) && part.at_insertion_approach);
            body.pose.translation = world_position(part);
            let nominal = if part.seated {
                part.plan.target_pose
            } else {
                part.plan
                    .collision_pose_at_travel_mm(
                        part.depth_mm.clamp(0.0, part.plan.insertion_travel_mm),
                    )
                    .unwrap_or(part.plan.target_pose)
            };
            body.pose.rotation = nominal_pose_to_core(nominal).rotation
                * Quat::from_scaled_axis(Vec3::new(
                    part.pose_error.rotation_deg[0].to_radians(),
                    part.pose_error.rotation_deg[1].to_radians(),
                    part.pose_error.rotation_deg[2].to_radians(),
                ));
        }
    }
}

fn world_position(part: &ComponentState) -> Vec3 {
    nominal_world_position(part)
        + Vec3::new(
            part.pose_error.translation_mm[0] * 1.0e-3,
            part.pose_error.translation_mm[1] * 1.0e-3,
            part.pose_error.translation_mm[2] * 1.0e-3,
        )
}

fn nominal_world_position(part: &ComponentState) -> Vec3 {
    let travel_mm = if part.seated {
        part.plan.insertion_travel_mm
    } else {
        part.depth_mm.clamp(0.0, part.plan.insertion_travel_mm)
    };
    let nominal = part
        .plan
        .collision_pose_at_travel_mm(travel_mm)
        .unwrap_or(part.plan.target_pose);
    Vec3::new(
        nominal.position_mm[0] * 1.0e-3,
        nominal.position_mm[1] * 1.0e-3,
        nominal.position_mm[2] * 1.0e-3,
    )
}

fn build_optics(seed: u64) -> StructuredLightRig {
    // Three inexpensive global views sit at each end of the qualified volume.
    // Both triplets use a 60 mm radius and 120-degree spacing (103.923 mm
    // same-end chord); the upper group is clocked by 60 degrees so rail/arm
    // occlusions do not line up through both groups.
    let global_image = ImageSize::new(1_280, 800);
    let global_focal_px = 640.0 / 34.0_f64.to_radians().tan();
    let global_intrinsics = CameraIntrinsics::new(global_focal_px, global_focal_px, 639.5, 399.5);
    let global_poses = [
        (0.0_f64, -0.106),
        (120.0_f64, -0.106),
        (240.0_f64, -0.106),
        (60.0_f64, 0.106),
        (180.0_f64, 0.106),
        (300.0_f64, 0.106),
    ];
    let mut cameras = global_poses
        .into_iter()
        .enumerate()
        .map(|(index, (azimuth_deg, z_m))| {
            let theta = azimuth_deg.to_radians();
            let origin = OpticalVec3::new(0.060 * theta.cos(), 0.060 * theta.sin(), z_m);
            CalibratedCamera::new(
                index as u32 + 1,
                PinholeCamera::new(
                    global_image,
                    global_intrinsics,
                    BrownConrady::NONE,
                    look_at(origin, OpticalVec3::ZERO),
                ),
            )
        })
        .collect::<Vec<_>>();

    // Two 2048×1536 macro cameras work at ~15 mm range. Their 4×3 mm nominal
    // field gives about 1.95 um object sampling; the camera pair is 12 mm
    // apart. Both use the cell's coded projector, whose optical datum is
    // independently fixed at the scenario/CAD +Y wall position.
    let macro_image = ImageSize::new(2_048, 1_536);
    let macro_working_range_m = 0.014_696_938_456_699_07;
    let macro_focal_px = macro_working_range_m * 2_048.0 / 0.004;
    let macro_intrinsics = CameraIntrinsics::new(macro_focal_px, macro_focal_px, 1_023.5, 767.5);
    for (index, origin) in [
        OpticalVec3::new(-0.006, 0.006, 0.012),
        OpticalVec3::new(0.006, 0.006, 0.012),
    ]
    .into_iter()
    .enumerate()
    {
        cameras.push(CalibratedCamera::new(
            101 + index as u32,
            PinholeCamera::new(
                macro_image,
                macro_intrinsics,
                BrownConrady::NONE,
                look_at(origin, OpticalVec3::ZERO),
            ),
        ));
    }
    let projector_origin = OpticalVec3::new(0.0, 0.060, 0.0);
    let projector = CalibratedCamera::new(
        200,
        PinholeCamera::new(
            global_image,
            global_intrinsics,
            BrownConrady::NONE,
            look_at(projector_origin, OpticalVec3::ZERO),
        ),
    );
    let config = ScanConfig {
        pixel_stride: 16,
        camera_pixel_sigma_floor_px: 0.18,
        projector_pixel_sigma_floor_px: 0.18,
        photon_centroid_coefficient_px: 2.0,
        depth_quantization_m: 0.25e-6,
        speckle_axial_sigma_m: 0.75e-6,
        reference_photoelectrons: 18_000.0,
        reference_range_m: 0.025,
        ambient_photoelectrons: 120.0,
        read_noise_electrons: 4.0,
        base_dropout_probability: 0.002,
        grazing_dropout_probability: 0.04,
        maximum_ray_separation_m: 120.0e-6,
        ..ScanConfig::default()
    };
    StructuredLightRig::new(cameras, projector, config, seed)
}

fn look_at(origin: OpticalVec3, target: OpticalVec3) -> RigidTransform {
    let forward = (target - origin).normalized().unwrap_or(OpticalVec3::Z);
    let up = if forward.z.abs() > 0.92 {
        OpticalVec3::Y
    } else {
        OpticalVec3::Z
    };
    let right = forward.cross(up).normalized().unwrap_or(OpticalVec3::X);
    let down = forward.cross(right).normalized().unwrap_or(OpticalVec3::Y);
    RigidTransform::new(
        OpticalMat3::new([
            [right.x, down.x, forward.x],
            [right.y, down.y, forward.y],
            [right.z, down.z, forward.z],
        ]),
        origin,
    )
}

fn optical_scene(
    parts: &[&ComponentState],
    frame_origin: OpticalVec3,
    occluded: Option<ComponentId>,
) -> Scene {
    let mut scene = Scene::default();
    for part in parts {
        scene.push(Primitive::new(
            Geometry::Sphere(OpticalSphere {
                center: optical_vec(world_position(part)) - frame_origin,
                radius_m: (characteristic_radius_mm(part) * 1.0e-3).max(0.08e-3),
            }),
            OpticalMaterial {
                diffuse_reflectance: 0.78,
                retroreflective_gain: 1.0,
                roughness: 0.30,
            },
            u32::from(part.id.0),
        ));
    }
    if let Some(id) = occluded {
        if let Some(part) = parts.iter().find(|part| part.id == id) {
            scene.push(Primitive::new(
                Geometry::Sphere(OpticalSphere {
                    center: optical_vec(world_position(part)) - frame_origin
                        + OpticalVec3::new(0.0, 0.0, 3.0e-3),
                    radius_m: 4.5e-3,
                }),
                OpticalMaterial {
                    diffuse_reflectance: 0.30,
                    retroreflective_gain: 1.0,
                    roughness: 0.80,
                },
                8_000,
            ));
        }
    }
    scene
}

fn characteristic_radius_mm(part: &ComponentState) -> f64 {
    if part.name.to_ascii_lowercase().contains("shaft") {
        0.175
    } else if part.name.contains("12") {
        0.70
    } else if part.name.contains("18") {
        1.00
    } else if part.name.contains("24") {
        1.30
    } else {
        1.0
    }
}

/// Lever arm of the visible nominal feature set used by the reduced pose
/// estimator. This is deliberately separate from the spherical ray proxy:
/// shafts expose their axis over their length, gears expose tooth tips, and
/// the retainer exposes its outer corners. A calibrated image-to-CAD feature
/// estimator is an explicit F2 upgrade.
fn orientation_feature_baseline_mm(part: &ComponentState) -> f64 {
    match part.plan.geometry {
        ComponentGeometry::Shaft { length_mm, .. } => length_mm * 0.5,
        ComponentGeometry::SpurGear {
            module_mm, teeth, ..
        } => module_mm * (f64::from(teeth) + 2.0) * 0.5,
        ComponentGeometry::Retainer { outer_size_mm, .. }
        | ComponentGeometry::Housing { outer_size_mm, .. } => {
            outer_size_mm[0].hypot(outer_size_mm[1]) * 0.5
        }
    }
    .max(0.50)
}

fn fuse_camera_sigmas_mm(sums_mm: &[f64], counts: &[u32], correlated_floor_mm: f64) -> f64 {
    debug_assert_eq!(sums_mm.len(), counts.len());
    let information = sums_mm
        .iter()
        .zip(counts)
        .filter(|(_, count)| **count > 0)
        .map(|(sum_mm, count)| {
            let per_view_sigma_mm = (sum_mm / f64::from(*count)).max(1.0e-9);
            1.0 / per_view_sigma_mm.powi(2)
        })
        .sum::<f64>();
    if information <= 0.0 || !information.is_finite() {
        return 0.050;
    }
    information
        .sqrt()
        .recip()
        .hypot(correlated_floor_mm.max(0.0))
}

fn fuse_camera_confidences(sums: &[f64], counts: &[u32]) -> f64 {
    debug_assert_eq!(sums.len(), counts.len());
    let miss_probability = sums
        .iter()
        .zip(counts)
        .filter(|(_, count)| **count > 0)
        .map(|(sum, count)| {
            let per_view_confidence = (sum / f64::from(*count)).clamp(0.0, 1.0);
            1.0 - per_view_confidence
        })
        .product::<f64>();
    (1.0 - miss_probability).clamp(0.0, 0.995)
}

fn seeded_pose(index: usize, scale: f64) -> PoseState {
    let sign = if index % 2 == 0 { 1.0 } else { -1.0 };
    PoseState {
        translation_mm: [
            sign * (0.020 + 0.001 * (index % 3) as f64) * scale,
            -sign * (0.012 + 0.001 * (index % 2) as f64) * scale,
            (0.008 + 0.001 * (index % 4) as f64) * scale,
        ],
        rotation_deg: [sign * 0.72 * scale, -sign * 0.42 * scale, 0.25 * scale],
    }
}

fn insertion_force(part: &ComponentState, scenario: &AssemblyScenario) -> f64 {
    let plan = scenario
        .recipe
        .components
        .iter()
        .find(|plan| plan.id == part.id)
        .expect("runtime component comes from recipe");
    let progress = (part.depth_mm / plan.insertion_travel_mm).clamp(0.0, 1.0);
    let lateral_mm = part.pose_error.translation_mm[0].hypot(part.pose_error.translation_mm[1]);
    let radial_clearance_mm = (plan.nominal_fit_clearance_mm * 0.5).max(0.0);
    let interference_mm = (lateral_mm - radial_clearance_mm).max(0.0);
    match plan.kind {
        // Shafts are intentionally the higher-force seated operation. The
        // cubic rise represents the lower-housing press/retention feature and
        // remains comfortably below its 50 mN guarded limit when aligned.
        ComponentKind::Shaft => 0.0035 + 0.028 * progress.powi(3) + 1.10 * interference_mm,
        // The idealized gear bores are 0.42 mm on 0.35 mm shafts: this is a
        // free-running fit, not a press fit. Sliding friction therefore stays
        // below 3.5 mN through the complete descent and below the 5 mN gate.
        ComponentKind::Gear => 0.0012 + 0.0021 * progress.powi(2) + 0.20 * interference_mm,
        // Cover descent is lightly loaded; the two latch events are modeled
        // separately by CloseRetainerFeature and its axial-force limit.
        ComponentKind::Retainer => 0.0018 + 0.0040 * progress.powi(2) + 0.45 * interference_mm,
        ComponentKind::Housing => 0.0020 + 0.0030 * progress.powi(2),
    }
}

fn verification_torque(part: &ComponentState) -> f64 {
    if part.mesh_sweep_deg > 0.0 {
        part.mesh_torque_mn_mm.max(0.060)
    } else {
        0.035
    }
}

fn command_arm_motion(mechanics: &mut Simulation, command: &Command) -> Result<(), SimError> {
    if mechanics.serial_arms.is_empty() {
        return Ok(());
    }
    if matches!(command, Command::HoldPosition | Command::AssemblyComplete) {
        mechanics
            .submit_machine_command(MachineCommand::Stop { manipulator: None })
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
        return Ok(());
    }
    let component = command_component(command).map_or(0, |id| id.0);
    let amplitude_rad = match command {
        Command::VisualServo { .. } => 0.22,
        Command::GuardedInsert { .. } => 0.12,
        Command::CloseTendonGripper { .. } | Command::CloseReceiverGripper { .. } => 0.08,
        Command::MeshDither { .. } | Command::RunVerification { .. } => 0.10,
        _ => 0.16,
    };
    let arm_count = mechanics.serial_arms.len();
    let arm_index = usize::from(component) % arm_count;
    let arm_id = mechanics.serial_arms[arm_index].id;
    let manipulator = ManipulatorId(arm_id.0);
    let sign = if component % 2 == 0 { 1.0 } else { -1.0 };
    let desired_angles = [
        sign * amplitude_rad,
        -sign * amplitude_rad * 0.75,
        sign * amplitude_rad * 0.55,
        sign * amplitude_rad * 0.25,
    ];
    let base_theta_rad =
        f64::from(arm_index as u32) * std::f64::consts::TAU / f64::from(arm_count as u32);
    let base_z_m = (f64::from(component) - 4.0) * 8.0e-3;
    for machine_command in [
        MachineCommand::MoveCarriageTheta {
            manipulator,
            target_theta_rad: base_theta_rad,
        },
        MachineCommand::MoveCarriageZ {
            manipulator,
            target_z_m: base_z_m,
        },
        MachineCommand::SetJointTargets {
            manipulator,
            target_rad: desired_angles,
        },
    ] {
        mechanics
            .submit_machine_command(machine_command)
            .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
    }
    let gripper_target = match command {
        Command::CloseTendonGripper { .. } | Command::CloseReceiverGripper { .. } => {
            mechanics
                .serial_arm(arm_id)
                .expect("selected arm remains present")
                .gripper_config
                .min_opening_m
        }
        Command::ReleaseDonorGripper { .. } => {
            mechanics
                .serial_arm(arm_id)
                .expect("selected arm remains present")
                .gripper_config
                .max_opening_m
        }
        _ => return Ok(()),
    };
    mechanics
        .submit_machine_command(MachineCommand::SetGripperOpening {
            manipulator,
            target_opening_m: gripper_target,
        })
        .map_err(|error| SimError::Mechanics(format!("{error:?}")))?;
    Ok(())
}

fn actuator_rms_error_m(mechanics: &Simulation) -> f64 {
    let mut squared = 0.0;
    let mut count = 0_u32;
    for arm in &mechanics.serial_arms {
        for (telemetry, joint) in arm
            .arm
            .tendon_telemetry
            .iter()
            .zip(arm.arm.config.tendon_joints.iter())
        {
            let deflection_m = telemetry.elastic_deflection_rad * joint.routing_radius_m;
            squared += deflection_m * deflection_m;
            count += 1;
        }
    }
    if count == 0 {
        0.0
    } else {
        (squared / f64::from(count)).sqrt()
    }
}

fn plant_gain(mechanics: &Simulation) -> f64 {
    (0.91 - 0.006 * actuator_rms_error_m(mechanics) * 1.0e6).clamp(0.72, 0.91)
}

fn optical_bias(cycle: u64, mean_range_error_um: f64) -> [f64; 3] {
    let wave = (cycle as f64 * 0.754_877_666).sin();
    [
        0.00025 * wave,
        -0.00018 * wave,
        mean_range_error_um * 1.0e-3,
    ]
}

fn optical_vec(value: Vec3) -> OpticalVec3 {
    OpticalVec3::new(value.x, value.y, value.z)
}

fn command_component(command: &Command) -> Option<ComponentId> {
    match command {
        Command::SearchVolume { component }
        | Command::MoveToPregrasp { component, .. }
        | Command::VisualServo { component, .. }
        | Command::CloseTendonGripper { component, .. }
        | Command::PresentForHandoff { component }
        | Command::CloseReceiverGripper { component, .. }
        | Command::ReleaseDonorGripper { component }
        | Command::MoveToBore { component, .. }
        | Command::GuardedInsert { component, .. }
        | Command::CloseRetainerFeature { component, .. }
        | Command::MeshDither { component, .. }
        | Command::RunVerification { component, .. }
        | Command::RetractAndReacquire { component, .. } => Some(*component),
        Command::HoldPosition | Command::AssemblyComplete => None,
    }
}

fn command_name(command: &Command) -> &'static str {
    match command {
        Command::SearchVolume { .. } => "search-volume",
        Command::MoveToPregrasp { .. } => "move-to-pregrasp",
        Command::VisualServo { .. } => "visual-servo",
        Command::CloseTendonGripper { .. } => "close-tendon-gripper",
        Command::PresentForHandoff { .. } => "present-for-handoff",
        Command::CloseReceiverGripper { .. } => "close-receiver-gripper",
        Command::ReleaseDonorGripper { .. } => "release-donor-gripper",
        Command::MoveToBore { .. } => "move-to-bore",
        Command::GuardedInsert { .. } => "guarded-insert",
        Command::CloseRetainerFeature { .. } => "close-retainer-feature",
        Command::MeshDither { .. } => "mesh-dither",
        Command::RunVerification { .. } => "run-verification",
        Command::RetractAndReacquire { .. } => "retract-and-reacquire",
        Command::HoldPosition => "hold-position",
        Command::AssemblyComplete => "assembly-complete",
    }
}

fn phase_name(phase: Phase) -> &'static str {
    match phase {
        Phase::Locate => "locate",
        Phase::Pick => "pick",
        Phase::Handoff => "handoff",
        Phase::Align => "align",
        Phase::Insert => "insert",
        Phase::Mesh => "mesh",
        Phase::Verify => "verify",
    }
}

fn status_name(status: &ExecutiveStatus) -> &'static str {
    match status {
        ExecutiveStatus::Running => "running",
        ExecutiveStatus::Completed => "completed",
        ExecutiveStatus::Aborted(_) => "aborted",
    }
}

fn failure_name(reason: &FailureReason) -> &'static str {
    match reason {
        FailureReason::UnplannedClearanceTooSmall { .. } => "unplanned-clearance-too-small",
        FailureReason::UnplannedContactForceHigh { .. } => "unplanned-contact-force-high",
        FailureReason::VisionConfidenceLow { .. } => "vision-confidence-low",
        FailureReason::PositionUncertaintyHigh { .. } => "position-uncertainty-high",
        FailureReason::OrientationUncertaintyHigh { .. } => "orientation-uncertainty-high",
        FailureReason::InsertionAxialForceHigh { .. } => "insertion-axial-force-high",
        FailureReason::InsertionLateralForceHigh { .. } => "insertion-lateral-force-high",
        FailureReason::MeshTorqueHigh { .. } => "mesh-torque-high",
        FailureReason::MeshNotEngaged => "mesh-not-engaged",
        FailureReason::PhaseTimeout { .. } => "phase-timeout",
        FailureReason::RetryBudgetExhausted { .. } => "retry-budget-exhausted",
        _ => "planner-gate-rejected",
    }
}

fn event_summary(event: &pipe_planner::TaskEvent) -> String {
    match &event.kind {
        EventKind::CommandIssued(command) => format!("command:{}", command_name(command)),
        EventKind::Transition { from, to } => {
            format!("transition:{}->{}", phase_name(*from), phase_name(*to))
        }
        EventKind::GateRejected { reason } => format!("rejected:{}", failure_name(reason)),
        EventKind::RetryScheduled {
            phase,
            retry,
            limit,
        } => {
            format!("retry:{}:{retry}/{limit}", phase_name(*phase))
        }
        EventKind::ComponentCompleted => "component-completed".to_owned(),
        EventKind::AssemblyCompleted => "assembly-completed".to_owned(),
        EventKind::Aborted { reason } => format!("aborted:{}", failure_name(reason)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_recipe_completes_from_physical_observations() {
        let mut simulator = ReferenceSimulator::from_scenario_name("nominal").unwrap();
        let report = simulator.run_to_completion(12_000).unwrap();
        assert!(report.completed, "failures: {:?}", report.failures);
        assert_eq!(report.metrics.components_completed, 7);
        assert!(report.metrics.visual_servo_corrections > 0);
        assert!(report.metrics.guarded_insert_commands > 0);
        assert!(report.metrics.mesh_dither_commands > 0);
        assert!(report.components.iter().all(|component| component.seated));
        let train = &report.gear_train_acceptance;
        assert_eq!(
            train.method,
            "stateful_analytic_backlash_constraint_reduced"
        );
        assert!(!train.executed_in_task_loop);
        assert!(!train.tooth_resolved_contact);
        assert_eq!((train.input_teeth, train.output_teeth), (12, 24));
        assert_eq!(train.expected_reduction, 2.0);
        assert!((train.observed_reduction - 2.0).abs() < 1.0e-12);
        assert!(train.same_direction);
        assert_eq!(train.forward.input_turns, 10.0);
        assert!((train.forward.output_turns - 5.0).abs() < 1.0e-12);
        assert_eq!(train.reverse.input_turns, -2.0);
        assert!(train.reverse.output_turns < -0.98);
        assert!(train.reverse.output_turns > -1.0);
        assert_eq!(train.mesh_backlash_mm, [0.020, 0.020]);
        assert_eq!(train.cover_latch_signatures, [true, true]);
        assert!(train.accepted);
    }

    #[test]
    fn fault_suite_is_deterministic_and_fails_closed_on_collision() {
        let mut first = ReferenceSimulator::from_scenario_name("fault-suite").unwrap();
        let first_report = first.run_to_completion(12_000).unwrap();
        let mut second = ReferenceSimulator::from_scenario_name("fault-suite").unwrap();
        let second_report = second.run_to_completion(12_000).unwrap();
        assert!(!first_report.completed);
        assert_eq!(first_report.status, "aborted");
        assert_eq!(
            first_report.to_json(false).unwrap(),
            second_report.to_json(false).unwrap()
        );
        for fault in ["occlusion", "collision", "insertion-force"] {
            assert!(first_report
                .snapshots
                .iter()
                .any(|snapshot| snapshot.injected_fault.as_deref() == Some(fault)));
        }
        assert!(first_report.metrics.retries >= 2);
        assert!(first_report.metrics.safety_stops >= 1);
    }

    #[test]
    fn bounded_occlusion_and_insertion_force_faults_recover() {
        for scenario in ["occlusion", "insertion-force"] {
            let mut simulator = ReferenceSimulator::from_scenario_name(scenario).unwrap();
            let report = simulator.run_to_completion(12_000).unwrap();
            assert!(report.completed, "{scenario}: {:?}", report.failures);
            assert!(report.metrics.retries >= 1);
        }
    }

    #[test]
    fn report_schema_serializes_for_native_and_wasm_adapters() {
        let simulator = ReferenceSimulator::from_scenario_name("nominal").unwrap();
        let compact = simulator.report_json(false).unwrap();
        let pretty = simulator.report_json(true).unwrap();
        let report = simulator.report();
        assert!(compact.contains("\"schema_version\":1"));
        assert!(compact.contains("\"gear_train_acceptance\""));
        assert!(pretty.contains('\n'));
        assert_eq!(report.fidelity.tier, "F1-reduced");
        assert!(!report.gear_train_acceptance.executed_in_task_loop);
        assert!(!report.gear_train_acceptance.accepted);
        assert_eq!(report.scenario_sha256, None);
        assert_eq!(
            report.cad_parameter_sha256.as_deref(),
            Some(COMPILED_CAD_PARAMETER_SHA256)
        );
        assert_eq!(
            report.cad_geometry_facts_sha256.as_deref(),
            Some(COMPILED_CAD_GEOMETRY_FACTS_SHA256)
        );
        assert_eq!(report.configuration_sha256.len(), 64);
        assert_ne!(
            report.configuration_sha256,
            ScenarioSpec::named("collision")
                .unwrap()
                .configuration_sha256
        );
    }

    #[test]
    fn live_step_has_scene_but_acceptance_report_keeps_compact_trace() {
        let mut simulator = ReferenceSimulator::from_scenario_name("nominal").unwrap();
        let live_json = simulator.step().unwrap().to_json(false).unwrap();
        assert!(live_json.contains("\"scene\":"));
        assert!(live_json.contains("\"schema_version\":1"));

        let report = simulator.report();
        assert_eq!(report.snapshots.len(), 1);
        let report_json = report.to_json(false).unwrap();
        assert!(!report_json.contains("\"scene\":"));
        assert!(report_json.contains("\"snapshots\":[{\"cycle\":1"));
    }

    #[test]
    fn cover_wall_contacts_are_intended_only_inside_alignment_gate() {
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        let cover_id = ComponentId(7);
        let mut components = scenario
            .recipe
            .components
            .iter()
            .enumerate()
            .map(|(index, plan)| ComponentState::new(index, plan))
            .collect::<Vec<_>>();
        let cover = components
            .iter_mut()
            .find(|part| part.id == cover_id)
            .expect("baseline recipe includes the cover");
        cover.at_insertion_approach = true;
        cover.pose_error.translation_mm[0] =
            scenario.acceptance.alignment.max_lateral_error_mm + 0.001;

        let walls = [
            HOUSING_LEFT_WALL_BODY_ID,
            HOUSING_RIGHT_WALL_BODY_ID,
            HOUSING_FRONT_WALL_BODY_ID,
            HOUSING_BACK_WALL_BODY_ID,
        ];
        let misaligned =
            intended_collision_pairs(&scenario, &components, Some(cover_id), Some(Phase::Insert));
        assert!(walls
            .iter()
            .all(|wall| !misaligned
                .contains(&canonical_body_pair(*wall, BodyId(u32::from(cover_id.0))))));

        components
            .iter_mut()
            .find(|part| part.id == cover_id)
            .unwrap()
            .pose_error = PoseState::default();
        let aligned =
            intended_collision_pairs(&scenario, &components, Some(cover_id), Some(Phase::Insert));
        assert!(walls.iter().all(
            |wall| aligned.contains(&canonical_body_pair(*wall, BodyId(u32::from(cover_id.0))))
        ));
    }

    #[test]
    fn shaft_and_gear_contacts_require_pose_capture_and_insertion_phase() {
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        let mut components = scenario
            .recipe
            .components
            .iter()
            .enumerate()
            .map(|(index, plan)| ComponentState::new(index, plan))
            .collect::<Vec<_>>();
        let excess_lateral_mm = scenario.acceptance.alignment.max_lateral_error_mm + 0.001;

        let shaft_id = ComponentId(1);
        let shaft_pair = canonical_body_pair(HOUSING_FLOOR_BODY_ID, BodyId(1));
        let shaft = components
            .iter_mut()
            .find(|part| part.id == shaft_id)
            .unwrap();
        shaft.at_insertion_approach = true;
        shaft.pose_error.translation_mm[0] = excess_lateral_mm;
        let misaligned_shaft =
            intended_collision_pairs(&scenario, &components, Some(shaft_id), Some(Phase::Insert));
        assert!(!misaligned_shaft.contains(&shaft_pair));
        let shaft = components
            .iter_mut()
            .find(|part| part.id == shaft_id)
            .unwrap();
        shaft.pose_error = PoseState::default();
        shaft.pose_error.translation_mm[2] =
            scenario.acceptance.alignment.max_axial_error_mm + 0.001;
        let axially_misaligned_shaft =
            intended_collision_pairs(&scenario, &components, Some(shaft_id), Some(Phase::Insert));
        assert!(!axially_misaligned_shaft.contains(&shaft_pair));
        components
            .iter_mut()
            .find(|part| part.id == shaft_id)
            .unwrap()
            .pose_error = PoseState::default();
        let early_shaft =
            intended_collision_pairs(&scenario, &components, Some(shaft_id), Some(Phase::Locate));
        assert!(!early_shaft.contains(&shaft_pair));
        let aligned_shaft =
            intended_collision_pairs(&scenario, &components, Some(shaft_id), Some(Phase::Align));
        assert!(aligned_shaft.contains(&shaft_pair));

        let gear_id = ComponentId(5);
        let gear = components
            .iter_mut()
            .find(|part| part.id == gear_id)
            .unwrap();
        gear.at_insertion_approach = true;
        gear.pose_error.translation_mm[0] = excess_lateral_mm;
        let bore_pair = canonical_body_pair(BodyId(5), BodyId(2));
        let mesh_pair = canonical_body_pair(BodyId(5), BodyId(4));
        let floor_pair = canonical_body_pair(HOUSING_FLOOR_BODY_ID, BodyId(5));
        let misaligned_gear =
            intended_collision_pairs(&scenario, &components, Some(gear_id), Some(Phase::Insert));
        for pair in [bore_pair, mesh_pair, floor_pair] {
            assert!(!misaligned_gear.contains(&pair));
        }
        components
            .iter_mut()
            .find(|part| part.id == gear_id)
            .unwrap()
            .pose_error = PoseState::default();
        let aligned_gear =
            intended_collision_pairs(&scenario, &components, Some(gear_id), Some(Phase::Insert));
        for pair in [bore_pair, mesh_pair, floor_pair] {
            assert!(aligned_gear.contains(&pair));
        }
    }

    #[test]
    fn aligned_seated_force_surrogate_respects_every_component_gate() {
        let scenario = MicroGearboxScenario::gearbox_baseline_v1();
        for (index, plan) in scenario.recipe.components.iter().enumerate() {
            let mut part = ComponentState::new(index, plan);
            part.pose_error = PoseState::default();
            part.depth_mm = plan.insertion_travel_mm;
            let force_n = insertion_force(&part, &scenario);
            assert!(
                force_n < plan.max_insertion_axial_force_n,
                "{} {:?}: {force_n} N must remain below {} N",
                plan.name,
                plan.kind,
                plan.max_insertion_axial_force_n,
            );
            if plan.kind == ComponentKind::Gear {
                assert!(
                    force_n < 0.004,
                    "free-running gear force should stay below 4 mN"
                );
            }
        }
    }

    #[test]
    fn sensing_rig_has_six_global_two_macro_views_and_meets_nominal_gate() {
        let rig = build_optics(0x5049_5045_5F47_4258);
        assert_eq!(rig.cameras.len(), 8);
        for camera in &rig.cameras[..6] {
            assert_eq!(camera.nominal.image_size, ImageSize::new(1_280, 800));
            let range_m = camera.nominal.center_world().norm();
            assert!((0.121..=0.123).contains(&range_m));
            let hfov_deg = (640.0 / camera.nominal.intrinsics.fx_px)
                .atan()
                .to_degrees()
                * 2.0;
            assert!((hfov_deg - 68.0).abs() < 1.0e-12);
        }
        for camera in &rig.cameras[6..] {
            assert_eq!(camera.nominal.image_size, ImageSize::new(2_048, 1_536));
            let range_m = camera.nominal.center_world().norm();
            assert!((0.010..=0.016).contains(&range_m));
            let object_sampling_mm = range_m * 1_000.0 / camera.nominal.intrinsics.fx_px;
            assert!(object_sampling_mm <= 0.0021);
        }
        let same_end_baseline_m =
            (rig.cameras[0].nominal.center_world() - rig.cameras[1].nominal.center_world()).norm();
        assert!((same_end_baseline_m - 0.103_923_048_454_132_63).abs() < 1.0e-12);
        for camera in &rig.cameras[..3] {
            assert!((camera.nominal.center_world().z + 0.106).abs() < 1.0e-12);
        }
        for camera in &rig.cameras[3..6] {
            assert!((camera.nominal.center_world().z - 0.106).abs() < 1.0e-12);
        }
        let macro_baseline_m =
            (rig.cameras[6].nominal.center_world() - rig.cameras[7].nominal.center_world()).norm();
        assert!((macro_baseline_m - 0.012).abs() < 1.0e-9);
        assert!(rig.cameras[6..]
            .iter()
            .all(|camera| (camera.nominal.center_world().y - 0.006).abs() < 1.0e-12));
        assert_eq!(
            rig.projector.nominal.center_world(),
            OpticalVec3::new(0.0, 0.060, 0.0)
        );

        let limits = MicroGearboxScenario::gearbox_baseline_v1()
            .acceptance
            .vision;
        let mut simulator = ReferenceSimulator::from_scenario_name("nominal").unwrap();
        let snapshot = simulator.step().unwrap();
        assert!(snapshot.optical.global_attempted_rays > 0);
        assert!(snapshot.optical.macro_attempted_rays > 0);
        assert!(
            snapshot.optical.valid_camera_views >= MIN_INDEPENDENT_CAMERA_VIEWS,
            "optical telemetry: {:?}",
            snapshot.optical
        );
        assert_eq!(snapshot.optical.macro_valid_camera_views, 2);
        assert!(
            snapshot.optical.confidence >= limits.min_confidence,
            "optical telemetry: {:?}",
            snapshot.optical
        );
        assert!(
            snapshot.optical.position_sigma_mm <= limits.max_position_sigma_mm,
            "optical telemetry: {:?}",
            snapshot.optical
        );
        assert!(
            snapshot.optical.orientation_sigma_deg <= limits.max_orientation_sigma_deg,
            "optical telemetry: {:?}",
            snapshot.optical
        );
    }

    #[test]
    fn repeated_rays_from_one_camera_do_not_satisfy_two_view_gate() {
        let limits = MicroGearboxScenario::gearbox_baseline_v1()
            .acceptance
            .vision;
        let mut simulator = ReferenceSimulator::from_scenario_name("nominal").unwrap();
        let one_macro_camera = simulator.optics.cameras[6];
        simulator.optics.cameras.clear();
        simulator.optics.cameras.push(one_macro_camera);
        let snapshot = simulator.step().unwrap();
        assert!(snapshot.optical.valid_target_returns > 1);
        assert!(snapshot.optical.valid_camera_views <= 1);
        assert!(snapshot.optical.confidence < limits.min_confidence);
    }

    #[test]
    fn optical_sigma_is_invariant_to_duplicate_samples_within_one_view() {
        let single = fuse_camera_sigmas_mm(&[0.010], &[1], OPTICAL_CORRELATED_FLOOR_MM);
        let duplicated = fuse_camera_sigmas_mm(&[0.050], &[5], OPTICAL_CORRELATED_FLOOR_MM);
        assert!((single - duplicated).abs() < 1.0e-15);
        let two_views =
            fuse_camera_sigmas_mm(&[0.010, 0.010], &[1, 1], OPTICAL_CORRELATED_FLOOR_MM);
        assert!(two_views < single);
        assert!(two_views >= OPTICAL_CORRELATED_FLOOR_MM);
    }

    #[test]
    fn optical_confidence_is_invariant_to_duplicate_samples_within_one_view() {
        let single = fuse_camera_confidences(&[0.80, 0.60], &[1, 1]);
        let duplicated = fuse_camera_confidences(&[4.0, 3.0], &[5, 5]);
        assert!((single - duplicated).abs() < 1.0e-15);
        assert!((single - 0.92).abs() < 1.0e-15);
    }

    #[test]
    fn simulator_recomputes_configuration_hash_at_its_trust_boundary() {
        let mut spec = ScenarioSpec::named("nominal").unwrap();
        let stale_hash = spec.configuration_sha256.clone();
        spec.seed ^= 0x55;
        let mut expected = spec.clone();
        expected.refresh_configuration_sha256();
        assert_ne!(stale_hash, expected.configuration_sha256);

        let simulator = ReferenceSimulator::new(spec).unwrap();
        assert_eq!(
            simulator.report().configuration_sha256,
            expected.configuration_sha256
        );
    }

    #[test]
    fn invalid_scenario_is_rejected() {
        assert!(matches!(
            ScenarioSpec::named("magic-resin"),
            Err(SimError::UnknownScenario(_))
        ));
    }

    #[test]
    fn stateful_gear_constraint_measures_reversal_backlash() {
        let mut train = ReducedGearTrainState::new(12, 18, 24, 0.10, [0.020, 0.020]);
        train.drive_input(0.10);
        train.zero_encoders();
        train.drive_input(10.0);
        assert!((train.output_turns - 5.0).abs() < 1.0e-12);

        train.zero_encoders();
        train.drive_input(-2.0);
        assert!(train.output_turns < -0.98);
        assert!(train.output_turns > -1.0);
    }
}
