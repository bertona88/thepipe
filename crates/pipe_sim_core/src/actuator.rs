//! Low-cost tendon actuator model with gearbox backlash and cable compliance.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorConfig {
    pub min_position_m: f64,
    pub max_position_m: f64,
    pub max_motor_speed_m_s: f64,
    pub max_output_speed_m_s: f64,
    /// Motor capstan radius. The joint tendon moment arm is a separate arm
    /// geometry parameter and must not be inferred from this value.
    pub capstan_radius_m: f64,
    /// Total lost motion measured at the tendon spool.
    pub backlash_m: f64,
    /// Effective series stiffness of tendon, printed structure, and spool.
    pub stiffness_n_m: f64,
    /// Kelvin-Voigt damping coefficient.
    pub damping_n_s_m: f64,
    /// Load magnitude used before the cheap actuator is treated as stalled.
    pub max_load_n: f64,
}

impl Default for ActuatorConfig {
    fn default() -> Self {
        Self {
            min_position_m: -6.0e-3,
            max_position_m: 6.0e-3,
            max_motor_speed_m_s: 12.0e-3,
            max_output_speed_m_s: 20.0e-3,
            capstan_radius_m: 3.0e-3,
            backlash_m: 18.0e-6,
            stiffness_n_m: 7_500.0,
            damping_n_s_m: 0.02,
            max_load_n: 4.0,
        }
    }
}

impl ActuatorConfig {
    pub fn is_valid(self) -> bool {
        self.min_position_m.is_finite()
            && self.max_position_m.is_finite()
            && self.min_position_m <= self.max_position_m
            && self.max_motor_speed_m_s > 0.0
            && self.max_motor_speed_m_s.is_finite()
            && self.max_output_speed_m_s > 0.0
            && self.max_output_speed_m_s.is_finite()
            && self.capstan_radius_m > 0.0
            && self.capstan_radius_m.is_finite()
            && self.backlash_m >= 0.0
            && self.backlash_m.is_finite()
            && self.stiffness_n_m > 0.0
            && self.stiffness_n_m.is_finite()
            && self.damping_n_s_m > 0.0
            && self.damping_n_s_m.is_finite()
            && self.max_load_n > 0.0
            && self.max_load_n.is_finite()
    }

    pub fn tendon_travel_for_angle_m(self, angle_rad: f64) -> f64 {
        self.capstan_radius_m * angle_rad
    }

    pub fn capstan_angle_for_travel_rad(self, travel_m: f64) -> f64 {
        travel_m / self.capstan_radius_m
    }

    pub fn maximum_capstan_torque_nm(self) -> f64 {
        self.max_load_n * self.capstan_radius_m
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorState {
    pub command_position_m: f64,
    pub motor_position_m: f64,
    /// Position after backlash but before elastic tendon deflection.
    pub transmission_position_m: f64,
    /// Actual tendon output position.
    pub output_position_m: f64,
    pub output_velocity_m_s: f64,
    pub applied_load_n: f64,
    pub stalled: bool,
}

impl ActuatorState {
    pub fn new(initial_position_m: f64, config: ActuatorConfig) -> Self {
        let initial = initial_position_m.clamp(config.min_position_m, config.max_position_m);
        Self {
            command_position_m: initial,
            motor_position_m: initial,
            transmission_position_m: initial,
            output_position_m: initial,
            output_velocity_m_s: 0.0,
            applied_load_n: 0.0,
            stalled: false,
        }
    }

    pub fn set_command(&mut self, command_position_m: f64, config: ActuatorConfig) {
        self.command_position_m =
            command_position_m.clamp(config.min_position_m, config.max_position_m);
    }

    /// Advance one deterministic fixed step.
    ///
    /// Positive `external_load_n` stretches the tendon (increases output
    /// position). Loads beyond `max_load_n` are clamped and set `stalled`.
    pub fn step(&mut self, dt_s: f64, external_load_n: f64, config: ActuatorConfig) {
        if dt_s <= 0.0 || !dt_s.is_finite() || !config.is_valid() {
            return;
        }

        let motor_error = self.command_position_m - self.motor_position_m;
        let motor_step = (config.max_motor_speed_m_s * dt_s).min(motor_error.abs());
        self.motor_position_m += motor_error.signum() * motor_step;
        self.motor_position_m = self
            .motor_position_m
            .clamp(config.min_position_m, config.max_position_m);

        // A play operator is deterministic, rate-independent, and captures the
        // direction reversal dead zone of a low-cost printed gearbox.
        let half_backlash = config.backlash_m * 0.5;
        let play_error = self.motor_position_m - self.transmission_position_m;
        if play_error > half_backlash {
            self.transmission_position_m = self.motor_position_m - half_backlash;
        } else if play_error < -half_backlash {
            self.transmission_position_m = self.motor_position_m + half_backlash;
        }

        self.stalled = external_load_n.abs() > config.max_load_n;
        self.applied_load_n = external_load_n.clamp(-config.max_load_n, config.max_load_n);

        // Exact integration of c*x' + k*(x-transmission) = load over the step,
        // assuming transmission and load are constant during this small step.
        let equilibrium = self.transmission_position_m + self.applied_load_n / config.stiffness_n_m;
        let alpha = 1.0 - (-config.stiffness_n_m * dt_s / config.damping_n_s_m).exp();
        let unconstrained_delta = (equilibrium - self.output_position_m) * alpha;
        let max_delta = config.max_output_speed_m_s * dt_s;
        let delta = unconstrained_delta.clamp(-max_delta, max_delta);
        self.output_velocity_m_s = delta / dt_s;
        self.output_position_m =
            (self.output_position_m + delta).clamp(config.min_position_m, config.max_position_m);
    }

    pub fn elastic_deflection_m(self) -> f64 {
        self.output_position_m - self.transmission_position_m
    }

    pub fn tendon_tension_n(self, config: ActuatorConfig) -> f64 {
        config.stiffness_n_m * self.elastic_deflection_m()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reversal_consumes_backlash_before_transmission_moves() {
        let config = ActuatorConfig {
            min_position_m: -1.0,
            max_position_m: 1.0,
            max_motor_speed_m_s: 10.0,
            max_output_speed_m_s: 10.0,
            capstan_radius_m: 0.01,
            backlash_m: 0.1,
            stiffness_n_m: 100.0,
            damping_n_s_m: 1.0,
            max_load_n: 10.0,
        };
        let mut state = ActuatorState::new(0.0, config);
        state.set_command(0.2, config);
        state.step(1.0, 0.0, config);
        assert!((state.transmission_position_m - 0.15).abs() < 1e-12);

        state.set_command(0.12, config);
        state.step(1.0, 0.0, config);
        assert!((state.transmission_position_m - 0.15).abs() < 1e-12);
        state.set_command(0.0, config);
        state.step(1.0, 0.0, config);
        assert!((state.transmission_position_m - 0.05).abs() < 1e-12);
    }

    #[test]
    fn static_load_converges_to_hookes_law_deflection() {
        let config = ActuatorConfig::default();
        let mut state = ActuatorState::new(0.0, config);
        for _ in 0..10_000 {
            state.step(1.0e-4, 0.09, config);
        }
        let expected = 0.09 / config.stiffness_n_m;
        assert!((state.elastic_deflection_m() - expected).abs() < 1.0e-9);
        assert!((state.tendon_tension_n(config) - 0.09).abs() < 1.0e-6);
    }

    #[test]
    fn actuator_step_is_partition_stable_at_equilibrium() {
        let config = ActuatorConfig::default();
        let mut a = ActuatorState::new(0.0, config);
        let mut b = a;
        for _ in 0..100 {
            a.step(1.0e-4, 0.01, config);
        }
        for _ in 0..10 {
            b.step(1.0e-3, 0.01, config);
        }
        assert!((a.output_position_m - b.output_position_m).abs() < 1.0e-8);
    }

    #[test]
    fn overload_sets_stall_and_clamps_force() {
        let config = ActuatorConfig::default();
        let mut state = ActuatorState::new(0.0, config);
        state.step(1.0e-3, 10.0, config);
        assert!(state.stalled);
        assert_eq!(state.applied_load_n, config.max_load_n);
    }

    #[test]
    fn default_spool_matches_locked_tendon_hardware() {
        let config = ActuatorConfig::default();
        assert_eq!(config.capstan_radius_m, 3.0e-3);
        assert_eq!(config.backlash_m, 18.0e-6);
        assert_eq!(config.stiffness_n_m, 7_500.0);
        assert_eq!(config.max_load_n, 4.0);
        assert_eq!(config.maximum_capstan_torque_nm(), 12.0e-3);
        let angle = 0.7;
        assert!(
            (config.capstan_angle_for_travel_rad(config.tendon_travel_for_angle_m(angle)) - angle)
                .abs()
                < 1.0e-15
        );
    }
}
