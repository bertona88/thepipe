//! Observation-only stationary handoff protocol. No physical state access.
use serde::Serialize;

use crate::observed_manipulation::PoseEstimate;

pub const DONOR_OBJECT: u32 = 20_001;
pub const RECEIVER_OBJECT: u32 = 20_002;
pub const PEG_OBJECT: u32 = 20_003;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Action {
    AcquireDonor,
    CloseReceiver,
    TransferOwnership,
    OpenDonor,
    VerifyReceiver,
    StopBoth,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    AcquireDonor,
    CloseReceiver,
    Transfer,
    OpenDonor,
    VerifyReceiver,
    Complete,
    Stopped,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct PadEvidence {
    pub manipulator_id: u32,
    pub capture_tick: u64,
    pub left_contact: bool,
    pub right_contact: bool,
    pub force_proxy_n: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct HandoffInput {
    pub sequence: u64,
    pub tick: u64,
    pub estimates: Vec<PoseEstimate>,
    pub donor_contact: PadEvidence,
    pub receiver_contact: PadEvidence,
}

#[derive(Clone, Copy, Debug)]
pub struct Policy {
    pub maximum_age_ticks: u64,
    pub maximum_position_sigma_m: f64,
    pub maximum_axis_sigma_rad: f64,
    pub maximum_capture_error_m: f64,
    pub maximum_axis_error_rad: f64,
    pub tool_to_peg_center_m: f64,
    pub minimum_force_n: f64,
    pub maximum_force_n: f64,
}

pub struct Controller {
    pub phase: Phase,
    pub owner: Option<u32>,
    pub terminal_reason: Option<String>,
    policy: Policy,
    pending: Option<(Action, u64)>,
    last_sequence: Option<u64>,
    after_tick: u64,
}

impl Controller {
    pub fn new(policy: Policy) -> Result<Self, String> {
        if policy.maximum_age_ticks == 0
            || [
                policy.maximum_position_sigma_m,
                policy.maximum_axis_sigma_rad,
                policy.maximum_capture_error_m,
                policy.maximum_axis_error_rad,
                policy.tool_to_peg_center_m,
                policy.minimum_force_n,
                policy.maximum_force_n,
            ]
            .iter()
            .any(|x| !x.is_finite() || *x <= 0.0)
            || policy.minimum_force_n >= policy.maximum_force_n
        {
            return Err("invalid_handoff_policy".into());
        }
        Ok(Self {
            phase: Phase::AcquireDonor,
            owner: None,
            terminal_reason: None,
            policy,
            pending: None,
            last_sequence: None,
            after_tick: 0,
        })
    }

    pub fn stop(&mut self, reason: impl Into<String>) -> Action {
        if self.phase != Phase::Stopped {
            self.terminal_reason = Some(reason.into());
        }
        self.phase = Phase::Stopped;
        self.pending = None;
        Action::StopBoth
    }

    pub fn authorize(&mut self, input: &HandoffInput) -> Action {
        if self.phase == Phase::Stopped {
            return Action::StopBoth;
        }
        match self.check(input) {
            Ok(action) => {
                self.last_sequence = Some(input.sequence);
                self.pending = Some((action, input.tick));
                action
            }
            Err(reason) => self.stop(reason),
        }
    }

    fn check(&self, input: &HandoffInput) -> Result<Action, &'static str> {
        if self.pending.is_some() || self.phase == Phase::Complete {
            return Err("invalid_protocol_order");
        }
        if input.tick < self.after_tick
            || self.last_sequence.is_some_and(|seq| input.sequence <= seq)
        {
            return Err("replayed_observation");
        }
        let mut poses = Vec::new();
        for id in [DONOR_OBJECT, RECEIVER_OBJECT, PEG_OBJECT] {
            let matches: Vec<_> = input
                .estimates
                .iter()
                .filter(|e| e.object_id == id)
                .collect();
            if matches.len() != 1 {
                return Err("observation_unavailable");
            }
            let e = matches[0];
            let p = e.usable_pose().ok_or("observation_unavailable")?;
            let u = e.uncertainty.as_ref().ok_or("observation_unavailable")?;
            let capture = e.oldest_capture_tick.ok_or("observation_unavailable")?;
            let available = e.newest_available_tick.ok_or("observation_unavailable")?;
            if capture < self.after_tick
                || capture > available
                || available > input.tick
                || input.tick - capture > self.policy.maximum_age_ticks
                || e.controller_tick != input.tick
                || e.state_tick > input.tick
            {
                return Err("stale_observation");
            }
            if p.roll_observable
                || p.center_world_m
                    .iter()
                    .chain(p.axis_world_unit.iter())
                    .any(|v| !v.is_finite())
                || (norm(p.axis_world_unit) - 1.0).abs() > 1.0e-8
                || u.center_sigma_m.iter().any(|v| {
                    !v.is_finite() || *v < 0.0 || *v > self.policy.maximum_position_sigma_m
                })
                || u.axis_tangent_sigma_rad
                    .iter()
                    .any(|v| !v.is_finite() || *v < 0.0 || *v > self.policy.maximum_axis_sigma_rad)
            {
                return Err("observation_uncertainty");
            }
            poses.push((p, u));
        }
        let (peg, peg_sigma) = poses[2];
        for (tool, sigma) in &poses[..2] {
            let error = std::array::from_fn(|i| {
                tool.center_world_m[i] + tool.axis_world_unit[i] * self.policy.tool_to_peg_center_m
                    - peg.center_world_m[i]
            });
            let uncertainty = 3.0
                * (sigma.center_sigma_m.iter().copied().fold(0.0, f64::max)
                    + peg_sigma.center_sigma_m.iter().copied().fold(0.0, f64::max)
                    + self.policy.tool_to_peg_center_m
                        * sigma
                            .axis_tangent_sigma_rad
                            .iter()
                            .copied()
                            .fold(0.0, f64::max));
            let cosine = tool
                .axis_world_unit
                .iter()
                .zip(peg.axis_world_unit)
                .map(|(a, b)| a * b)
                .sum::<f64>()
                .abs();
            if norm(error) + uncertainty > self.policy.maximum_capture_error_m
                || cosine.clamp(0.0, 1.0).acos()
                    + 3.0
                        * (sigma
                            .axis_tangent_sigma_rad
                            .iter()
                            .copied()
                            .fold(0.0, f64::max)
                            + peg_sigma
                                .axis_tangent_sigma_rad
                                .iter()
                                .copied()
                                .fold(0.0, f64::max))
                    > self.policy.maximum_axis_error_rad
            {
                return Err("relative_pose_outside_capture");
            }
        }
        for (packet, id) in [(input.donor_contact, 1), (input.receiver_contact, 2)] {
            if packet.manipulator_id != id
                || packet.capture_tick < self.after_tick
                || packet.capture_tick > input.tick
                || input.tick - packet.capture_tick > self.policy.maximum_age_ticks
                || !packet.force_proxy_n.is_finite()
                || packet.force_proxy_n < 0.0
            {
                return Err("invalid_contact_packet");
            }
            if packet.force_proxy_n > self.policy.maximum_force_n {
                return Err("force_limit");
            }
        }
        let holding = |p: PadEvidence| {
            p.left_contact && p.right_contact && p.force_proxy_n >= self.policy.minimum_force_n
        };
        match self.phase {
            Phase::AcquireDonor if holding(input.donor_contact) => Ok(Action::AcquireDonor),
            Phase::CloseReceiver if holding(input.donor_contact) => Ok(Action::CloseReceiver),
            Phase::Transfer if holding(input.donor_contact) && holding(input.receiver_contact) => {
                Ok(Action::TransferOwnership)
            }
            Phase::OpenDonor if holding(input.receiver_contact) => Ok(Action::OpenDonor),
            Phase::VerifyReceiver
                if holding(input.receiver_contact)
                    && !input.donor_contact.left_contact
                    && !input.donor_contact.right_contact =>
            {
                Ok(Action::VerifyReceiver)
            }
            _ => Err("required_contact_missing"),
        }
    }

    /// Ownership changes only after the plant acknowledges its atomic operation.
    pub fn acknowledge(&mut self, action: Action, tick: u64, accepted: bool) {
        if !self
            .pending
            .is_some_and(|(expected, authorized_at)| expected == action && tick >= authorized_at)
            || tick < self.after_tick
            || !accepted
        {
            self.stop("plant_transaction_rejected");
            return;
        }
        self.pending = None;
        self.after_tick = tick;
        match action {
            Action::AcquireDonor => {
                self.owner = Some(1);
                self.phase = Phase::CloseReceiver;
            }
            Action::CloseReceiver => self.phase = Phase::Transfer,
            Action::TransferOwnership => {
                self.owner = Some(2);
                self.phase = Phase::OpenDonor;
            }
            Action::OpenDonor => self.phase = Phase::VerifyReceiver,
            Action::VerifyReceiver => self.phase = Phase::Complete,
            Action::StopBoth => {
                self.stop("invalid_protocol_order");
            }
        }
    }
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}
