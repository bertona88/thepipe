use std::fmt;

/// Stable recipe-local identifier. Simulation body IDs can be mapped to this
/// at the sensor/task boundary without coupling the executive to one physics
/// implementation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ComponentId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComponentKind {
    Housing,
    Shaft,
    Gear,
    Retainer,
}

/// Nominal CAD envelope used by the reference task. Detailed involutes,
/// chamfers, bores, and print compensation remain the responsibility of the
/// build123d/physics layer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ComponentGeometry {
    Housing {
        outer_size_mm: [f64; 3],
        wall_mm: f64,
    },
    Shaft {
        diameter_mm: f64,
        length_mm: f64,
    },
    SpurGear {
        module_mm: f64,
        teeth: u16,
        pressure_angle_deg: f64,
        thickness_mm: f64,
        bore_diameter_mm: f64,
        hub_diameter_mm: f64,
        total_height_mm: f64,
    },
    Retainer {
        outer_size_mm: [f64; 3],
        thickness_mm: f64,
    },
}

impl ComponentGeometry {
    pub fn pitch_diameter_mm(self) -> Option<f64> {
        match self {
            Self::SpurGear {
                module_mm, teeth, ..
            } => Some(module_mm * f64::from(teeth)),
            _ => None,
        }
    }

    pub fn outside_diameter_mm(self) -> Option<f64> {
        match self {
            Self::SpurGear {
                module_mm, teeth, ..
            } => Some(module_mm * (f64::from(teeth) + 2.0)),
            _ => None,
        }
    }

    fn is_valid_for(self, kind: ComponentKind) -> bool {
        match (self, kind) {
            (
                Self::Housing {
                    outer_size_mm,
                    wall_mm,
                },
                ComponentKind::Housing,
            ) => {
                outer_size_mm.into_iter().all(positive)
                    && positive(wall_mm)
                    && outer_size_mm.into_iter().all(|size| wall_mm * 2.0 < size)
            }
            (
                Self::Shaft {
                    diameter_mm,
                    length_mm,
                },
                ComponentKind::Shaft,
            ) => positive(diameter_mm) && positive(length_mm),
            (
                Self::SpurGear {
                    module_mm,
                    teeth,
                    pressure_angle_deg,
                    thickness_mm,
                    bore_diameter_mm,
                    hub_diameter_mm,
                    total_height_mm,
                },
                ComponentKind::Gear,
            ) => {
                positive(module_mm)
                    && teeth >= 6
                    && pressure_angle_deg.is_finite()
                    && (0.0..45.0).contains(&pressure_angle_deg)
                    && positive(thickness_mm)
                    && positive(bore_diameter_mm)
                    && positive(hub_diameter_mm)
                    && hub_diameter_mm > bore_diameter_mm
                    && positive(total_height_mm)
                    && total_height_mm >= thickness_mm
                    && bore_diameter_mm < module_mm * (f64::from(teeth) - 2.5)
            }
            (
                Self::Retainer {
                    outer_size_mm,
                    thickness_mm,
                },
                ComponentKind::Retainer,
            ) => {
                outer_size_mm.into_iter().all(positive)
                    && positive(thickness_mm)
                    && thickness_mm <= outer_size_mm[2]
            }
            _ => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NominalAssemblyPose {
    /// Position in the lower-housing CAD frame, in millimetres.
    pub position_mm: [f64; 3],
    /// Unit vector for the component's local +Z axis in the housing frame.
    pub part_axis: [f64; 3],
    /// Rotation about `part_axis`; this preserves the deterministic tooth
    /// phase that an axis-only pose would lose.
    pub phase_deg: f64,
}

impl NominalAssemblyPose {
    fn is_valid(self) -> bool {
        let values_finite = self
            .position_mm
            .into_iter()
            .chain(self.part_axis)
            .all(f64::is_finite);
        let axis_norm = norm(self.part_axis);
        values_finite && self.phase_deg.is_finite() && (axis_norm - 1.0).abs() <= 1.0e-6
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComponentPlan {
    pub id: ComponentId,
    pub name: String,
    pub kind: ComponentKind,
    pub geometry: ComponentGeometry,
    /// Exact transform applied to the build123d part at its exported CAD
    /// origin in the fully seated assembly.
    pub cad_seated_origin_pose: NominalAssemblyPose,
    /// Exact fully seated body-centre pose used by estimation, optics, and the
    /// reduced collision plant. It is the centre of the complete CAD envelope,
    /// unlike the MIN_Z/MIN3 CAD origin recorded separately above.
    pub target_pose: NominalAssemblyPose,
    /// Unit world-space direction of positive insertion travel. The reference
    /// article is lowered along -Z.
    pub insertion_direction: [f64; 3],
    /// Whether a second arm must take the part before insertion.
    pub requires_handoff: bool,
    /// Whether the part must pass the driven mesh check.
    pub requires_mesh: bool,
    /// Scalar travel from the approach pose to `target_pose`. This
    /// is progress along `insertion_direction`, never an absolute world Z.
    pub insertion_travel_mm: f64,
    pub insertion_depth_tolerance_mm: f64,
    pub max_insertion_axial_force_n: f64,
    pub max_insertion_lateral_force_n: f64,
    /// Number of sequential closure signatures needed after seating (two for
    /// the reference cover latches, zero for shafts and gears).
    pub required_closure_features: u8,
    /// Signed nominal diametral clearance: positive is a running gap and
    /// negative is an intentional interference. This is intentionally
    /// distinct from sensed unplanned robot/environment clearance.
    pub nominal_fit_clearance_mm: f64,
}

impl ComponentPlan {
    /// Collision-proxy pose before insertion begins.
    pub fn insertion_start_collision_pose(&self) -> NominalAssemblyPose {
        self.collision_pose_unchecked(0.0)
    }

    /// Collision-proxy pose at a measured insertion travel. Returns `None`
    /// outside the closed [0, insertion_travel_mm] interval.
    pub fn collision_pose_at_travel_mm(&self, travel_mm: f64) -> Option<NominalAssemblyPose> {
        (travel_mm.is_finite() && (0.0..=self.insertion_travel_mm).contains(&travel_mm))
            .then(|| self.collision_pose_unchecked(travel_mm))
    }

    fn collision_pose_unchecked(&self, travel_mm: f64) -> NominalAssemblyPose {
        let remaining = self.insertion_travel_mm - travel_mm;
        let mut pose = self.target_pose;
        for (position, direction) in pose.position_mm.iter_mut().zip(self.insertion_direction) {
            *position -= direction * remaining;
        }
        pose
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HousingPlan {
    pub geometry: ComponentGeometry,
    /// Exact transform applied to the MIN3 build123d housing solid.
    pub cad_registered_origin_pose: NominalAssemblyPose,
    /// Registered body-centre target used by the reduced collision plant.
    pub target_pose: NominalAssemblyPose,
    pub shaft_seat_diameter_mm: f64,
    pub shaft_seat_depth_mm: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GearboxRecipe {
    pub name: String,
    /// The housing begins registered in the nest; it is localized and
    /// inspected but is not picked as one of the assembly components.
    pub registered_housing: HousingPlan,
    pub components: Vec<ComponentPlan>,
    pub gear_meshes: Vec<GearMeshPlan>,
    /// Feature-scoped intended contacts. There is deliberately no whole-body
    /// housing/part allowance: each feature must have its own collision proxy.
    pub intended_contacts: Vec<AssemblyContactDefinition>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GearMeshPlan {
    pub driver: ComponentId,
    pub driven: ComponentId,
    pub nominal_center_distance_mm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ContactFeature {
    HousingShaftSeat { shaft: ComponentId },
    HousingFloor,
    HousingCoverRim,
    HousingLatchCapture { feature: u8 },
    ShaftExterior { shaft: ComponentId },
    GearBore { gear: ComponentId },
    GearLowerFace { gear: ComponentId },
    GearTeeth { gear: ComponentId },
    CoverLip { cover: ComponentId },
    CoverLatch { cover: ComponentId, feature: u8 },
}

impl ContactFeature {
    pub fn component(self) -> Option<ComponentId> {
        match self {
            Self::HousingShaftSeat { shaft } | Self::ShaftExterior { shaft } => Some(shaft),
            Self::GearBore { gear } | Self::GearLowerFace { gear } | Self::GearTeeth { gear } => {
                Some(gear)
            }
            Self::CoverLip { cover } | Self::CoverLatch { cover, .. } => Some(cover),
            Self::HousingFloor | Self::HousingCoverRim | Self::HousingLatchCapture { .. } => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AssemblyContactKind {
    ShaftSeat,
    BoreOnShaft,
    GearOnFloor,
    GearMesh,
    CoverRim,
    CoverLatch,
}

/// An intended physical relation that becomes active at one exact assembly
/// step and phase. Once its activating component is complete it remains active
/// while later components are assembled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AssemblyContactDefinition {
    pub activates_with: ComponentId,
    pub activates_at: Phase,
    pub feature_a: ContactFeature,
    pub feature_b: ContactFeature,
    pub kind: AssemblyContactKind,
}

impl GearboxRecipe {
    pub fn component(&self, id: ComponentId) -> Option<&ComponentPlan> {
        self.components.iter().find(|part| part.id == id)
    }

    /// Feature relations valid for the current task state. Contacts established
    /// by earlier components remain valid; current-component contacts begin
    /// only at their declared phase.
    pub fn intended_contacts_for(
        &self,
        active: ComponentId,
        phase: Phase,
    ) -> Vec<&AssemblyContactDefinition> {
        let Some(active_index) = self.components.iter().position(|part| part.id == active) else {
            return Vec::new();
        };
        self.intended_contacts
            .iter()
            .filter(|definition| {
                self.components
                    .iter()
                    .position(|part| part.id == definition.activates_with)
                    .is_some_and(|activation_index| {
                        activation_index < active_index
                            || (activation_index == active_index
                                && phase.index() >= definition.activates_at.index())
                    })
            })
            .collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CellBaseline {
    pub mobile_arm_count: u8,
    pub each_base_has_z_motion: bool,
    pub each_base_has_theta_motion: bool,
    pub usable_tube_length_mm: f64,
    pub global_camera_count: u8,
    pub simultaneous_macro_views: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SafetyLimits {
    pub max_sensor_age_ms: u64,
    pub min_unplanned_clearance_mm: f64,
    pub max_unplanned_contact_force_n: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VisionLimits {
    pub min_confidence: f64,
    pub max_position_sigma_mm: f64,
    pub max_orientation_sigma_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspLimits {
    pub min_force_n: f64,
    pub max_force_n: f64,
    pub max_slip_mm: f64,
    pub pregrasp_translation_tolerance_mm: f64,
    pub pregrasp_rotation_tolerance_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandoffLimits {
    pub min_receiver_force_n: f64,
    pub max_receiver_force_n: f64,
    pub max_translation_error_mm: f64,
    pub max_rotation_error_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AlignmentLimits {
    pub max_lateral_error_mm: f64,
    pub max_axial_error_mm: f64,
    pub max_rotation_error_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InsertionLimits {
    pub max_axial_force_n: f64,
    pub max_lateral_force_n: f64,
    pub depth_tolerance_mm: f64,
    pub max_overshoot_mm: f64,
    pub guarded_speed_mm_s: f64,
    pub retract_distance_mm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshLimits {
    pub min_sweep_deg: f64,
    pub max_peak_torque_mn_mm: f64,
    pub min_backlash_mm: f64,
    pub max_backlash_mm: f64,
    pub dither_step_deg: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerificationLimits {
    pub min_vision_confidence: f64,
    pub max_translation_error_mm: f64,
    pub max_rotation_error_deg: f64,
    pub min_rotation_test_deg: f64,
    pub max_peak_torque_mn_mm: f64,
    pub max_torque_ripple_fraction: f64,
    pub min_backlash_mm: f64,
    pub max_backlash_mm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VisualServoLimits {
    pub translation_gain: f64,
    pub rotation_gain: f64,
    pub max_translation_step_mm: f64,
    pub max_rotation_step_deg: f64,
    pub speed_mm_s: f64,
    pub max_steps_per_attempt: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetryLimits {
    pub locate: u8,
    pub pick: u8,
    pub handoff: u8,
    pub align: u8,
    pub insert: u8,
    pub mesh: u8,
    pub verify: u8,
    pub max_cycles_per_attempt: u16,
}

impl RetryLimits {
    pub fn for_phase(self, phase: Phase) -> u8 {
        match phase {
            Phase::Locate => self.locate,
            Phase::Pick => self.pick,
            Phase::Handoff => self.handoff,
            Phase::Align => self.align,
            Phase::Insert => self.insert,
            Phase::Mesh => self.mesh,
            Phase::Verify => self.verify,
        }
    }
}

/// All pass/fail numbers required to reproduce an assembly result. Values are
/// in millimetres, degrees, newtons, milliseconds, and milli-newton-millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AcceptanceParameters {
    pub safety: SafetyLimits,
    pub vision: VisionLimits,
    pub grasp: GraspLimits,
    pub handoff: HandoffLimits,
    pub alignment: AlignmentLimits,
    pub insertion: InsertionLimits,
    pub mesh: MeshLimits,
    pub verification: VerificationLimits,
    pub servo: VisualServoLimits,
    pub retries: RetryLimits,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyScenario {
    pub name: String,
    pub deterministic_seed: u64,
    pub physics_step_ms: u64,
    pub cell: CellBaseline,
    pub recipe: GearboxRecipe,
    pub acceptance: AcceptanceParameters,
}

impl AssemblyScenario {
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.name.trim().is_empty() || self.physics_step_ms == 0 {
            return Err(ConfigError::InvalidScenarioMetadata);
        }
        if self.recipe.components.is_empty() {
            return Err(ConfigError::EmptyRecipe);
        }
        let housing = self.recipe.registered_housing;
        if !housing.geometry.is_valid_for(ComponentKind::Housing)
            || !housing.cad_registered_origin_pose.is_valid()
            || !housing.target_pose.is_valid()
            || !same_orientation(housing.cad_registered_origin_pose, housing.target_pose)
            || !positive(housing.shaft_seat_diameter_mm)
            || !positive(housing.shaft_seat_depth_mm)
        {
            return Err(ConfigError::InvalidHousing);
        }
        if self.cell.mobile_arm_count == 0
            || !self.cell.each_base_has_z_motion
            || !self.cell.each_base_has_theta_motion
            || !positive(self.cell.usable_tube_length_mm)
            || self.cell.global_camera_count == 0
            || self.cell.simultaneous_macro_views < 2
            || self.cell.simultaneous_macro_views > self.cell.global_camera_count
        {
            return Err(ConfigError::InvalidCellBaseline);
        }
        for (index, part) in self.recipe.components.iter().enumerate() {
            if part.name.trim().is_empty() {
                return Err(ConfigError::UnnamedComponent { index });
            }
            if !positive(part.insertion_travel_mm) {
                return Err(ConfigError::InvalidValue("component.insertion_travel_mm"));
            }
            if !positive(part.insertion_depth_tolerance_mm)
                || !positive(part.max_insertion_axial_force_n)
                || !positive(part.max_insertion_lateral_force_n)
            {
                return Err(ConfigError::InvalidValue("component.insertion_limits"));
            }
            if !part.nominal_fit_clearance_mm.is_finite() {
                return Err(ConfigError::InvalidValue(
                    "component.nominal_fit_clearance_mm",
                ));
            }
            if !part.geometry.is_valid_for(part.kind) {
                return Err(ConfigError::InvalidComponentGeometry { index });
            }
            if !part.cad_seated_origin_pose.is_valid()
                || !part.target_pose.is_valid()
                || !same_orientation(part.cad_seated_origin_pose, part.target_pose)
                || !unit_vector(part.insertion_direction)
            {
                return Err(ConfigError::InvalidComponentPose { index });
            }
            if self.recipe.components[..index]
                .iter()
                .any(|other| other.id == part.id)
            {
                return Err(ConfigError::DuplicateComponentId(part.id));
            }
        }
        for (index, mesh) in self.recipe.gear_meshes.iter().enumerate() {
            if mesh.driver == mesh.driven || !positive(mesh.nominal_center_distance_mm) {
                return Err(ConfigError::InvalidGearMesh { index });
            }
            let driver = self
                .recipe
                .components
                .iter()
                .find(|part| part.id == mesh.driver)
                .ok_or(ConfigError::InvalidGearMesh { index })?;
            let driven = self
                .recipe
                .components
                .iter()
                .find(|part| part.id == mesh.driven)
                .ok_or(ConfigError::InvalidGearMesh { index })?;
            let expected_center_distance = match (
                driver.geometry.pitch_diameter_mm(),
                driven.geometry.pitch_diameter_mm(),
            ) {
                (Some(driver_pitch), Some(driven_pitch)) => (driver_pitch + driven_pitch) * 0.5,
                _ => return Err(ConfigError::InvalidGearMesh { index }),
            };
            if (mesh.nominal_center_distance_mm - expected_center_distance).abs() > 1.0e-9
                || (point_distance_mm(
                    driver.target_pose.position_mm,
                    driven.target_pose.position_mm,
                ) - mesh.nominal_center_distance_mm)
                    .abs()
                    > 1.0e-9
                || (!driver.requires_mesh && !driven.requires_mesh)
            {
                return Err(ConfigError::InvalidGearMesh { index });
            }
        }
        for (index, contact) in self.recipe.intended_contacts.iter().enumerate() {
            if self.recipe.component(contact.activates_with).is_none()
                || contact.feature_a == contact.feature_b
                || !contact_feature_is_valid(&self.recipe, contact.feature_a)
                || !contact_feature_is_valid(&self.recipe, contact.feature_b)
                || !contact_kind_matches(*contact)
                || !contact_activation_matches(*contact)
                || !contact_geometry_matches(&self.recipe, *contact)
            {
                return Err(ConfigError::InvalidIntendedContact { index });
            }
        }
        for (index, mesh) in self.recipe.gear_meshes.iter().enumerate() {
            let has_feature_scoped_mesh = self.recipe.intended_contacts.iter().any(|contact| {
                if contact.kind != AssemblyContactKind::GearMesh {
                    return false;
                }
                matches!(
                    (contact.feature_a, contact.feature_b),
                    (
                        ContactFeature::GearTeeth { gear: a },
                        ContactFeature::GearTeeth { gear: b }
                    ) if (a == mesh.driver && b == mesh.driven)
                        || (a == mesh.driven && b == mesh.driver)
                )
            });
            if !has_feature_scoped_mesh {
                return Err(ConfigError::InvalidIntendedContact { index });
            }
        }

        let p = self.acceptance;
        let positive_values = [
            (
                "safety.min_unplanned_clearance_mm",
                p.safety.min_unplanned_clearance_mm,
            ),
            (
                "safety.max_unplanned_contact_force_n",
                p.safety.max_unplanned_contact_force_n,
            ),
            (
                "vision.max_position_sigma_mm",
                p.vision.max_position_sigma_mm,
            ),
            (
                "vision.max_orientation_sigma_deg",
                p.vision.max_orientation_sigma_deg,
            ),
            ("grasp.min_force_n", p.grasp.min_force_n),
            ("grasp.max_force_n", p.grasp.max_force_n),
            ("grasp.max_slip_mm", p.grasp.max_slip_mm),
            (
                "grasp.pregrasp_translation_tolerance_mm",
                p.grasp.pregrasp_translation_tolerance_mm,
            ),
            (
                "grasp.pregrasp_rotation_tolerance_deg",
                p.grasp.pregrasp_rotation_tolerance_deg,
            ),
            (
                "handoff.min_receiver_force_n",
                p.handoff.min_receiver_force_n,
            ),
            (
                "handoff.max_receiver_force_n",
                p.handoff.max_receiver_force_n,
            ),
            (
                "handoff.max_translation_error_mm",
                p.handoff.max_translation_error_mm,
            ),
            (
                "handoff.max_rotation_error_deg",
                p.handoff.max_rotation_error_deg,
            ),
            (
                "alignment.max_lateral_error_mm",
                p.alignment.max_lateral_error_mm,
            ),
            (
                "alignment.max_axial_error_mm",
                p.alignment.max_axial_error_mm,
            ),
            (
                "alignment.max_rotation_error_deg",
                p.alignment.max_rotation_error_deg,
            ),
            ("insertion.max_axial_force_n", p.insertion.max_axial_force_n),
            (
                "insertion.max_lateral_force_n",
                p.insertion.max_lateral_force_n,
            ),
            (
                "insertion.depth_tolerance_mm",
                p.insertion.depth_tolerance_mm,
            ),
            ("insertion.max_overshoot_mm", p.insertion.max_overshoot_mm),
            (
                "insertion.guarded_speed_mm_s",
                p.insertion.guarded_speed_mm_s,
            ),
            (
                "insertion.retract_distance_mm",
                p.insertion.retract_distance_mm,
            ),
            ("mesh.min_sweep_deg", p.mesh.min_sweep_deg),
            ("mesh.max_peak_torque_mn_mm", p.mesh.max_peak_torque_mn_mm),
            ("mesh.dither_step_deg", p.mesh.dither_step_deg),
            (
                "verification.max_translation_error_mm",
                p.verification.max_translation_error_mm,
            ),
            (
                "verification.max_rotation_error_deg",
                p.verification.max_rotation_error_deg,
            ),
            (
                "verification.min_rotation_test_deg",
                p.verification.min_rotation_test_deg,
            ),
            (
                "verification.max_peak_torque_mn_mm",
                p.verification.max_peak_torque_mn_mm,
            ),
            ("servo.translation_gain", p.servo.translation_gain),
            ("servo.rotation_gain", p.servo.rotation_gain),
            (
                "servo.max_translation_step_mm",
                p.servo.max_translation_step_mm,
            ),
            ("servo.max_rotation_step_deg", p.servo.max_rotation_step_deg),
            ("servo.speed_mm_s", p.servo.speed_mm_s),
        ];
        for (name, value) in positive_values {
            if !positive(value) {
                return Err(ConfigError::InvalidValue(name));
            }
        }
        let fractions = [
            ("vision.min_confidence", p.vision.min_confidence),
            (
                "verification.min_vision_confidence",
                p.verification.min_vision_confidence,
            ),
            (
                "verification.max_torque_ripple_fraction",
                p.verification.max_torque_ripple_fraction,
            ),
        ];
        for (name, value) in fractions {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                return Err(ConfigError::InvalidValue(name));
            }
        }
        if p.grasp.max_force_n <= p.grasp.min_force_n {
            return Err(ConfigError::InvalidRange("grasp.force"));
        }
        for part in &self.recipe.components {
            if part.max_insertion_axial_force_n > p.insertion.max_axial_force_n
                || part.max_insertion_lateral_force_n > p.insertion.max_lateral_force_n
            {
                return Err(ConfigError::InvalidRange("component.insertion_force"));
            }
        }
        if p.handoff.max_receiver_force_n <= p.handoff.min_receiver_force_n {
            return Err(ConfigError::InvalidRange("handoff.force"));
        }
        if !ordered_nonnegative(p.mesh.min_backlash_mm, p.mesh.max_backlash_mm) {
            return Err(ConfigError::InvalidRange("mesh.backlash"));
        }
        if !ordered_nonnegative(
            p.verification.min_backlash_mm,
            p.verification.max_backlash_mm,
        ) {
            return Err(ConfigError::InvalidRange("verification.backlash"));
        }
        if p.safety.max_sensor_age_ms == 0
            || p.servo.max_steps_per_attempt == 0
            || p.retries.max_cycles_per_attempt == 0
        {
            return Err(ConfigError::InvalidValue("integer limit"));
        }
        Ok(())
    }
}

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn unit_vector(value: [f64; 3]) -> bool {
    value.into_iter().all(f64::is_finite) && (norm(value) - 1.0).abs() <= 1.0e-6
}

fn same_orientation(a: NominalAssemblyPose, b: NominalAssemblyPose) -> bool {
    a.part_axis
        .into_iter()
        .zip(b.part_axis)
        .all(|(left, right)| (left - right).abs() <= 1.0e-9)
        && (a.phase_deg - b.phase_deg).abs() <= 1.0e-9
}

fn point_distance_mm(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm([a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

fn contact_feature_is_valid(recipe: &GearboxRecipe, feature: ContactFeature) -> bool {
    match feature {
        ContactFeature::HousingShaftSeat { shaft } | ContactFeature::ShaftExterior { shaft } => {
            recipe
                .component(shaft)
                .is_some_and(|part| part.kind == ComponentKind::Shaft)
        }
        ContactFeature::GearBore { gear }
        | ContactFeature::GearLowerFace { gear }
        | ContactFeature::GearTeeth { gear } => recipe
            .component(gear)
            .is_some_and(|part| part.kind == ComponentKind::Gear),
        ContactFeature::CoverLip { cover } => recipe
            .component(cover)
            .is_some_and(|part| part.kind == ComponentKind::Retainer),
        ContactFeature::CoverLatch { cover, feature } => {
            recipe.component(cover).is_some_and(|part| {
                part.kind == ComponentKind::Retainer
                    && feature > 0
                    && feature <= part.required_closure_features
            })
        }
        ContactFeature::HousingLatchCapture { feature } => feature > 0,
        ContactFeature::HousingFloor | ContactFeature::HousingCoverRim => true,
    }
}

fn contact_kind_matches(contact: AssemblyContactDefinition) -> bool {
    use AssemblyContactKind as Kind;
    use ContactFeature as Feature;
    matches!(
        (contact.kind, contact.feature_a, contact.feature_b),
        (
            Kind::ShaftSeat,
            Feature::HousingShaftSeat { .. },
            Feature::ShaftExterior { .. }
        ) | (
            Kind::ShaftSeat,
            Feature::ShaftExterior { .. },
            Feature::HousingShaftSeat { .. }
        ) | (
            Kind::BoreOnShaft,
            Feature::GearBore { .. },
            Feature::ShaftExterior { .. }
        ) | (
            Kind::BoreOnShaft,
            Feature::ShaftExterior { .. },
            Feature::GearBore { .. }
        ) | (
            Kind::GearOnFloor,
            Feature::GearLowerFace { .. },
            Feature::HousingFloor
        ) | (
            Kind::GearOnFloor,
            Feature::HousingFloor,
            Feature::GearLowerFace { .. }
        ) | (
            Kind::GearMesh,
            Feature::GearTeeth { .. },
            Feature::GearTeeth { .. }
        ) | (
            Kind::CoverRim,
            Feature::CoverLip { .. },
            Feature::HousingCoverRim
        ) | (
            Kind::CoverRim,
            Feature::HousingCoverRim,
            Feature::CoverLip { .. }
        ) | (
            Kind::CoverLatch,
            Feature::CoverLatch { .. },
            Feature::HousingLatchCapture { .. }
        ) | (
            Kind::CoverLatch,
            Feature::HousingLatchCapture { .. },
            Feature::CoverLatch { .. }
        )
    ) && contact_feature_pair_matches(contact)
}

fn contact_feature_pair_matches(contact: AssemblyContactDefinition) -> bool {
    use ContactFeature as Feature;
    match (contact.feature_a, contact.feature_b) {
        (Feature::HousingShaftSeat { shaft: a }, Feature::ShaftExterior { shaft: b })
        | (Feature::ShaftExterior { shaft: b }, Feature::HousingShaftSeat { shaft: a }) => a == b,
        (
            Feature::CoverLatch {
                cover: _,
                feature: a,
            },
            Feature::HousingLatchCapture { feature: b },
        )
        | (
            Feature::HousingLatchCapture { feature: b },
            Feature::CoverLatch {
                cover: _,
                feature: a,
            },
        ) => a == b,
        (Feature::GearTeeth { gear: a }, Feature::GearTeeth { gear: b }) => a != b,
        _ => true,
    }
}

fn contact_activation_matches(contact: AssemblyContactDefinition) -> bool {
    use AssemblyContactKind as Kind;
    use ContactFeature as Feature;
    match (contact.kind, contact.feature_a, contact.feature_b) {
        (Kind::ShaftSeat, Feature::HousingShaftSeat { shaft }, _)
        | (Kind::ShaftSeat, _, Feature::HousingShaftSeat { shaft }) => {
            contact.activates_with == shaft && contact.activates_at == Phase::Insert
        }
        (Kind::BoreOnShaft, Feature::GearBore { gear }, _)
        | (Kind::BoreOnShaft, _, Feature::GearBore { gear })
        | (Kind::GearOnFloor, Feature::GearLowerFace { gear }, _)
        | (Kind::GearOnFloor, _, Feature::GearLowerFace { gear }) => {
            contact.activates_with == gear && contact.activates_at == Phase::Insert
        }
        (Kind::GearMesh, Feature::GearTeeth { gear: a }, Feature::GearTeeth { gear: b }) => {
            (contact.activates_with == a || contact.activates_with == b)
                && contact.activates_at == Phase::Mesh
        }
        (Kind::CoverRim, Feature::CoverLip { cover }, _)
        | (Kind::CoverRim, _, Feature::CoverLip { cover })
        | (Kind::CoverLatch, Feature::CoverLatch { cover, .. }, _)
        | (Kind::CoverLatch, _, Feature::CoverLatch { cover, .. }) => {
            contact.activates_with == cover && contact.activates_at == Phase::Insert
        }
        _ => false,
    }
}

fn contact_geometry_matches(recipe: &GearboxRecipe, contact: AssemblyContactDefinition) -> bool {
    use AssemblyContactKind as Kind;
    use ContactFeature as Feature;
    match (contact.kind, contact.feature_a, contact.feature_b) {
        (Kind::BoreOnShaft, Feature::GearBore { gear }, Feature::ShaftExterior { shaft })
        | (Kind::BoreOnShaft, Feature::ShaftExterior { shaft }, Feature::GearBore { gear }) => {
            let (Some(gear), Some(shaft)) = (recipe.component(gear), recipe.component(shaft))
            else {
                return false;
            };
            let delta_x = gear.target_pose.position_mm[0] - shaft.target_pose.position_mm[0];
            let delta_y = gear.target_pose.position_mm[1] - shaft.target_pose.position_mm[1];
            delta_x.hypot(delta_y) <= 1.0e-9
        }
        (Kind::GearMesh, Feature::GearTeeth { gear: a }, Feature::GearTeeth { gear: b }) => {
            recipe.gear_meshes.iter().any(|mesh| {
                (mesh.driver == a && mesh.driven == b) || (mesh.driver == b && mesh.driven == a)
            })
        }
        _ => true,
    }
}

fn nonnegative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

fn ordered_nonnegative(min: f64, max: f64) -> bool {
    nonnegative(min) && max.is_finite() && max >= min
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    InvalidScenarioMetadata,
    EmptyRecipe,
    InvalidHousing,
    InvalidCellBaseline,
    UnnamedComponent { index: usize },
    DuplicateComponentId(ComponentId),
    InvalidComponentGeometry { index: usize },
    InvalidComponentPose { index: usize },
    InvalidGearMesh { index: usize },
    InvalidIntendedContact { index: usize },
    InvalidValue(&'static str),
    InvalidRange(&'static str),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidScenarioMetadata => write!(f, "invalid scenario metadata"),
            Self::EmptyRecipe => write!(f, "assembly recipe has no components"),
            Self::InvalidHousing => write!(f, "invalid registered housing geometry"),
            Self::InvalidCellBaseline => write!(f, "invalid assembly-cell baseline"),
            Self::UnnamedComponent { index } => {
                write!(f, "component at index {index} has no name")
            }
            Self::DuplicateComponentId(id) => write!(f, "duplicate component id {}", id.0),
            Self::InvalidComponentGeometry { index } => {
                write!(f, "component at index {index} has invalid geometry")
            }
            Self::InvalidComponentPose { index } => {
                write!(f, "component at index {index} has invalid target pose")
            }
            Self::InvalidGearMesh { index } => {
                write!(f, "gear mesh at index {index} is invalid")
            }
            Self::InvalidIntendedContact { index } => {
                write!(f, "intended contact at index {index} is invalid")
            }
            Self::InvalidValue(name) => write!(f, "invalid acceptance value: {name}"),
            Self::InvalidRange(name) => write!(f, "invalid acceptance range: {name}"),
        }
    }
}

impl std::error::Error for ConfigError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Phase {
    Locate,
    Pick,
    Handoff,
    Align,
    Insert,
    Mesh,
    Verify,
}

impl Phase {
    pub const ALL: [Self; 7] = [
        Self::Locate,
        Self::Pick,
        Self::Handoff,
        Self::Align,
        Self::Insert,
        Self::Mesh,
        Self::Verify,
    ];

    pub const fn index(self) -> usize {
        match self {
            Self::Locate => 0,
            Self::Pick => 1,
            Self::Handoff => 2,
            Self::Align => 3,
            Self::Insert => 4,
            Self::Mesh => 5,
            Self::Verify => 6,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PoseError6d {
    pub translation_mm: [f64; 3],
    pub rotation_deg: [f64; 3],
}

impl PoseError6d {
    pub fn translation_norm_mm(self) -> f64 {
        norm(self.translation_mm)
    }

    pub fn lateral_norm_mm(self) -> f64 {
        self.translation_mm[0].hypot(self.translation_mm[1])
    }

    pub fn axial_abs_mm(self) -> f64 {
        self.translation_mm[2].abs()
    }

    pub fn rotation_norm_deg(self) -> f64 {
        norm(self.rotation_deg)
    }

    pub fn is_finite(self) -> bool {
        self.translation_mm
            .into_iter()
            .chain(self.rotation_deg)
            .all(f64::is_finite)
    }
}

fn norm(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VisionObservation {
    pub component: ComponentId,
    pub confidence: f64,
    pub position_sigma_mm: f64,
    pub orientation_sigma_deg: f64,
    /// Error from the target pose computed by the perception/kinematics layer.
    pub target_error: PoseError6d,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperObservation {
    pub component: ComponentId,
    pub force_n: f64,
    pub slip_mm: f64,
    pub part_retained: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HandoffObservation {
    pub component: ComponentId,
    pub target_error: PoseError6d,
    pub receiver_force_n: f64,
    pub receiver_has_part: bool,
    pub donor_released: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AlignmentObservation {
    pub component: ComponentId,
    pub bore_error: PoseError6d,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InsertionObservation {
    pub component: ComponentId,
    /// Measured scalar travel from the component's approach pose along its
    /// declared insertion direction; not an absolute coordinate.
    pub depth_mm: f64,
    pub axial_force_n: f64,
    pub lateral_force_n: f64,
    pub seated: bool,
    pub closure_features_confirmed: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshObservation {
    pub component: ComponentId,
    pub sweep_deg: f64,
    pub peak_torque_mn_mm: f64,
    pub backlash_mm: f64,
    pub teeth_engaged: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerificationObservation {
    pub component: ComponentId,
    pub target_error: PoseError6d,
    pub vision_confidence: f64,
    pub rotation_test_deg: f64,
    pub peak_torque_mn_mm: f64,
    pub torque_ripple_fraction: f64,
    pub backlash_mm: f64,
    /// True when the active component's required verification features remain
    /// observable from the configured minimum number of independent views.
    pub required_features_visible: bool,
}

/// One synchronized observation packet. `min_unplanned_clearance_mm` and
/// `unplanned_contact_force_n` must exclude the intended gripper/part,
/// part/bore, and gear/tooth contact pairs for the active phase.
#[derive(Clone, Debug, PartialEq)]
pub struct SensorFrame {
    pub control_time_ms: u64,
    pub capture_time_ms: u64,
    pub min_unplanned_clearance_mm: f64,
    pub unplanned_contact_force_n: f64,
    pub vision: Option<VisionObservation>,
    pub gripper: Option<GripperObservation>,
    pub handoff: Option<HandoffObservation>,
    pub alignment: Option<AlignmentObservation>,
    pub insertion: Option<InsertionObservation>,
    pub mesh: Option<MeshObservation>,
    pub verification: Option<VerificationObservation>,
}

impl SensorFrame {
    pub fn empty(control_time_ms: u64) -> Self {
        Self {
            control_time_ms,
            capture_time_ms: control_time_ms,
            min_unplanned_clearance_mm: 1000.0,
            unplanned_contact_force_n: 0.0,
            vision: None,
            gripper: None,
            handoff: None,
            alignment: None,
            insertion: None,
            mesh: None,
            verification: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServoPurpose {
    Pregrasp,
    Handoff,
    BoreAlignment,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseCorrection {
    pub translation_mm: [f64; 3],
    pub rotation_deg: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    SearchVolume {
        component: ComponentId,
    },
    MoveToPregrasp {
        component: ComponentId,
        speed_mm_s: f64,
    },
    VisualServo {
        component: ComponentId,
        purpose: ServoPurpose,
        correction: PoseCorrection,
        speed_mm_s: f64,
    },
    CloseTendonGripper {
        component: ComponentId,
        target_force_n: f64,
    },
    PresentForHandoff {
        component: ComponentId,
    },
    CloseReceiverGripper {
        component: ComponentId,
        target_force_n: f64,
    },
    ReleaseDonorGripper {
        component: ComponentId,
    },
    MoveToBore {
        component: ComponentId,
        speed_mm_s: f64,
    },
    GuardedInsert {
        component: ComponentId,
        /// Absolute insertion-travel target measured from the approach pose,
        /// making repeated control packets idempotent. It is not world Z.
        target_depth_mm: f64,
        speed_mm_s: f64,
        axial_force_limit_n: f64,
        lateral_force_limit_n: f64,
    },
    CloseRetainerFeature {
        component: ComponentId,
        /// One-based closure feature index, deterministic and sequential.
        feature: u8,
        axial_force_limit_n: f64,
    },
    MeshDither {
        component: ComponentId,
        delta_deg: f64,
        torque_limit_mn_mm: f64,
    },
    RunVerification {
        component: ComponentId,
        rotation_deg: f64,
        torque_limit_mn_mm: f64,
    },
    RetractAndReacquire {
        component: ComponentId,
        distance_mm: f64,
    },
    HoldPosition,
    AssemblyComplete,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeasurementKind {
    Vision,
    Gripper,
    Handoff,
    Alignment,
    Insertion,
    Mesh,
    Verification,
}

#[derive(Clone, Debug, PartialEq)]
pub enum FailureReason {
    NonMonotonicTime {
        previous_ms: u64,
        received_ms: u64,
    },
    FutureDatedSensorFrame {
        capture_ms: u64,
        control_ms: u64,
    },
    StaleSensorFrame {
        age_ms: u64,
        limit_ms: u64,
    },
    InvalidMeasurement {
        field: &'static str,
    },
    WrongComponent {
        expected: ComponentId,
        observed: ComponentId,
    },
    MissingMeasurement(MeasurementKind),
    UnplannedClearanceTooSmall {
        measured_mm: f64,
        limit_mm: f64,
    },
    UnplannedContactForceHigh {
        measured_n: f64,
        limit_n: f64,
    },
    VisionConfidenceLow {
        measured: f64,
        limit: f64,
    },
    PositionUncertaintyHigh {
        measured_mm: f64,
        limit_mm: f64,
    },
    OrientationUncertaintyHigh {
        measured_deg: f64,
        limit_deg: f64,
    },
    VisualServoDidNotConverge {
        steps: u16,
    },
    GripForceHigh {
        measured_n: f64,
        limit_n: f64,
    },
    GripSlipHigh {
        measured_mm: f64,
        limit_mm: f64,
    },
    HandoffForceHigh {
        measured_n: f64,
        limit_n: f64,
    },
    InsertionAxialForceHigh {
        measured_n: f64,
        limit_n: f64,
    },
    InsertionLateralForceHigh {
        measured_n: f64,
        limit_n: f64,
    },
    InsertionDepthOvershoot {
        measured_mm: f64,
        target_mm: f64,
        limit_mm: f64,
    },
    PrematureSeat {
        measured_mm: f64,
        target_mm: f64,
    },
    MeshTorqueHigh {
        measured_mn_mm: f64,
        limit_mn_mm: f64,
    },
    MeshBacklashOutOfRange {
        measured_mm: f64,
        min_mm: f64,
        max_mm: f64,
    },
    MeshNotEngaged,
    VerificationPoseOutOfTolerance,
    VerificationVisionLow {
        measured: f64,
        limit: f64,
    },
    VerificationTorqueHigh {
        measured_mn_mm: f64,
        limit_mn_mm: f64,
    },
    VerificationTorqueRippleHigh {
        measured: f64,
        limit: f64,
    },
    VerificationBacklashOutOfRange {
        measured_mm: f64,
        min_mm: f64,
        max_mm: f64,
    },
    ComponentOccludedAtVerification,
    PhaseTimeout {
        cycles: u16,
    },
    RetryBudgetExhausted {
        phase: Phase,
        last_failure: Box<FailureReason>,
    },
}

impl FailureReason {
    /// Damage-risk and time-consistency faults are terminal immediately.
    /// Ordinary sensing and fit faults enter bounded recovery.
    pub fn severity(&self) -> FailureSeverity {
        match self {
            Self::NonMonotonicTime { .. }
            | Self::FutureDatedSensorFrame { .. }
            | Self::InvalidMeasurement { .. }
            | Self::UnplannedContactForceHigh { .. }
            | Self::GripForceHigh { .. }
            | Self::HandoffForceHigh { .. }
            | Self::InsertionDepthOvershoot { .. }
            | Self::RetryBudgetExhausted { .. } => FailureSeverity::Terminal,
            _ => FailureSeverity::Recoverable,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FailureSeverity {
    Recoverable,
    Terminal,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ExecutiveStatus {
    Running,
    Completed,
    Aborted(FailureReason),
}

#[derive(Clone, Debug, PartialEq)]
pub enum EventKind {
    CommandIssued(Command),
    Transition { from: Phase, to: Phase },
    GateRejected { reason: FailureReason },
    RetryScheduled { phase: Phase, retry: u8, limit: u8 },
    ComponentCompleted,
    AssemblyCompleted,
    Aborted { reason: FailureReason },
}

#[derive(Clone, Debug, PartialEq)]
pub struct TaskEvent {
    pub sequence: u64,
    pub control_time_ms: u64,
    pub component: Option<ComponentId>,
    pub phase: Option<Phase>,
    pub kind: EventKind,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct TaskMetrics {
    pub control_cycles: u64,
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

#[derive(Clone, Debug, PartialEq)]
pub struct Decision {
    pub status: ExecutiveStatus,
    pub component: Option<ComponentId>,
    pub phase: Option<Phase>,
    pub command: Command,
    /// Events produced by this call only. The full trace is available from the
    /// executive and uses the same monotonically increasing sequence numbers.
    pub events: Vec<TaskEvent>,
}
