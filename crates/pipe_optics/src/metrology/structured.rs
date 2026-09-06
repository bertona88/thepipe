use super::*;
use crate::{ImageSize, Vec2};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PatternConfig {
    pub projector_size: ImageSize,
    pub phase_period_px: f64,
    pub phase_steps: u32,
    pub frame_rate_hz: f64,
    pub minimum_modulation: f64,
    pub saturation_level: f64,
    pub maximum_gray_phase_disagreement_px: f64,
}
impl Default for PatternConfig {
    fn default() -> Self {
        Self {
            projector_size: ImageSize::new(1920, 1200),
            phase_period_px: 16.0,
            phase_steps: 4,
            frame_rate_hz: 2000.0,
            minimum_modulation: 0.05,
            saturation_level: 0.995,
            maximum_gray_phase_disagreement_px: 0.75,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PatternAxis {
    U,
    V,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Pattern {
    Dark,
    White,
    Gray {
        axis: PatternAxis,
        bit: u32,
        inverted: bool,
    },
    Phase {
        axis: PatternAxis,
        step: u32,
    },
}
impl PatternConfig {
    pub fn validate(&self) -> Result<(), MetrologyError> {
        if self.projector_size.width < 2
            || self.projector_size.height < 2
            || self.projector_size.width > 65536
            || self.projector_size.height > 65536
            || !positive(self.phase_period_px)
            || self.phase_period_px < 4.0
            || !(4..=16).contains(&self.phase_steps)
            || !positive(self.frame_rate_hz)
            || !positive(self.minimum_modulation)
            || self.minimum_modulation >= 0.5
            || !positive(self.saturation_level)
            || self.saturation_level > 1.0
            || !positive(self.maximum_gray_phase_disagreement_px)
            || self.maximum_gray_phase_disagreement_px >= self.phase_period_px * 0.5
        {
            return Err(config_error("invalid hybrid pattern parameters"));
        }
        Ok(())
    }
    pub fn patterns(&self) -> Vec<Pattern> {
        let mut out = vec![Pattern::Dark, Pattern::White];
        for (axis, size) in [
            (PatternAxis::U, self.projector_size.width),
            (PatternAxis::V, self.projector_size.height),
        ] {
            for bit in (0..bits(size)).rev() {
                for inverted in [false, true] {
                    out.push(Pattern::Gray {
                        axis,
                        bit,
                        inverted,
                    });
                }
            }
            for step in 0..self.phase_steps {
                out.push(Pattern::Phase { axis, step });
            }
        }
        out
    }
    pub fn duration_s(&self, exposure_s: f64) -> f64 {
        (self.patterns().len() - 1) as f64 / self.frame_rate_hz + exposure_s
    }
    /// Ideal normalized projected intensity. Synthetic boundary only; decode
    /// never receives the projector coordinate used here.
    pub fn intensity(&self, p: Pattern, pixel: Vec2) -> f64 {
        let coord = |axis| {
            if axis == PatternAxis::U {
                pixel.x
            } else {
                pixel.y
            }
        };
        match p {
            Pattern::Dark => 0.0,
            Pattern::White => 1.0,
            Pattern::Gray {
                axis,
                bit,
                inverted,
            } => {
                let n = (coord(axis) + 0.5).floor().max(0.0) as u32;
                let gray = n ^ (n >> 1);
                f64::from((((gray >> bit) & 1) != 0) ^ inverted)
            }
            Pattern::Phase { axis, step } => {
                0.5 + 0.5
                    * (TAU
                        * (coord(axis) / self.phase_period_px
                            + step as f64 / self.phase_steps as f64))
                        .cos()
            }
        }
    }
}
fn bits(n: u32) -> u32 {
    32 - (n - 1).leading_zeros()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionEvidence {
    StoppedInterlock,
    IndependentOpticalBound,
    Unknown,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AcquisitionTiming {
    pub sequence_id: u64,
    pub trigger_id: u64,
    pub exposure_start_s: f64,
    pub exposure_duration_s: f64,
    pub frame_timestamp_s: f64,
    pub available_s: f64,
    pub trigger_offset_s: f64,
    pub motion_evidence: MotionEvidence,
    /// Bound on ANY measured surface point, including rotation about a TCP.
    pub speed_bound_m_s: f64,
}
impl AcquisitionTiming {
    pub fn validate(&self) -> Result<(), MetrologyError> {
        if ![
            self.exposure_start_s,
            self.available_s,
            self.speed_bound_m_s,
        ]
        .iter()
        .all(|&v| nonnegative(v))
            || !positive(self.exposure_duration_s)
            || !self.frame_timestamp_s.is_finite()
            || !self.trigger_offset_s.is_finite()
            || (self.frame_timestamp_s - self.exposure_start_s - self.exposure_duration_s * 0.5)
                .abs()
                > 1e-9
            || self.available_s < self.exposure_start_s + self.exposure_duration_s
            || (self.motion_evidence == MotionEvidence::StoppedInterlock
                && self.speed_bound_m_s != 0.0)
        {
            return Err(MetrologyError::IncompatibleTiming);
        }
        Ok(())
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PatternExposure {
    pub pattern: Pattern,
    pub timing: AcquisitionTiming,
    pub illumination: Illumination,
    pub intensity: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DecodedProjectorPixel {
    pub pixel: Vec2,
    pub sigma_px: Vec2,
    pub modulation: f64,
    pub sequence_start_s: f64,
    pub sequence_end_s: f64,
    pub available_s: f64,
    pub sequence_id: u64,
    pub frame_count: usize,
}
/// Decode both projector axes from dark/white, complementary Gray code and
/// phase intensities. Reject motion, missing/reordered frames and mismatched
/// illumination. No truth coordinate or geometry enters this function.
pub fn decode_hybrid(
    config: &MetrologyConfig,
    frames: &[PatternExposure],
) -> Result<DecodedProjectorPixel, MetrologyError> {
    config.validate()?;
    let p = &config.patterns;
    let patterns = p.patterns();
    if frames.len() != patterns.len() {
        return Err(MetrologyError::IncompatibleTiming);
    }
    let first = &frames[0];
    let start = first.timing.exposure_start_s;
    let end = frames.last().unwrap().timing.exposure_start_s
        + frames.last().unwrap().timing.exposure_duration_s;
    for (i, (f, pattern)) in frames.iter().zip(patterns).enumerate() {
        f.timing.validate()?;
        if f.pattern != pattern
            || f.timing.sequence_id != first.timing.sequence_id
            || f.timing.trigger_id != first.timing.trigger_id + i as u64
            || (f.timing.exposure_start_s - start - i as f64 / p.frame_rate_hz).abs()
                > config.acquisition.max_trigger_skew_s
            || f.illumination.kind != IlluminationKind::Structured
            || f.illumination != first.illumination
            || f.timing.trigger_offset_s.abs() > config.acquisition.max_trigger_skew_s
        {
            return Err(MetrologyError::IncompatibleTiming);
        }
        if f.timing.motion_evidence == MotionEvidence::Unknown
            || f.timing.speed_bound_m_s * (end - start) > config.acquisition.max_sequence_motion_m
            || f.timing.speed_bound_m_s * f.timing.exposure_duration_s
                > config.acquisition.max_exposure_motion_m
        {
            return Err(MetrologyError::MotionDuringSequence);
        }
        if !f.intensity.is_finite() || f.intensity < 0.0 {
            return Err(MetrologyError::InvalidObservation);
        }
        if f.intensity >= p.saturation_level {
            return Err(MetrologyError::Saturation);
        }
    }
    let contrast = frames[1].intensity - frames[0].intensity;
    if contrast < 2.0 * p.minimum_modulation {
        return Err(MetrologyError::LowSignal);
    }
    let mut coord = [0.0; 2];
    let mut sigma = [0.0; 2];
    let mut min_mod = f64::INFINITY;
    for (index, axis) in [PatternAxis::U, PatternAxis::V].iter().enumerate() {
        let size = if index == 0 {
            p.projector_size.width
        } else {
            p.projector_size.height
        };
        let mut gray = 0u32;
        for bit in 0..bits(size) {
            let normal = frames
                .iter()
                .find(|f| {
                    f.pattern
                        == Pattern::Gray {
                            axis: *axis,
                            bit,
                            inverted: false,
                        }
                })
                .unwrap()
                .intensity;
            let inverse = frames
                .iter()
                .find(|f| {
                    f.pattern
                        == Pattern::Gray {
                            axis: *axis,
                            bit,
                            inverted: true,
                        }
                })
                .unwrap()
                .intensity;
            if (normal - inverse).abs() < p.minimum_modulation {
                return Err(MetrologyError::LowSignal);
            }
            if normal > inverse {
                gray |= 1 << bit;
            }
        }
        let mut binary = gray;
        let mut shifted = gray >> 1;
        while shifted != 0 {
            binary ^= shifted;
            shifted >>= 1;
        }
        if binary >= size {
            return Err(MetrologyError::InconsistentObservations);
        }
        let mut c = 0.0;
        let mut s = 0.0;
        for step in 0..p.phase_steps {
            let intensity = frames
                .iter()
                .find(|f| f.pattern == Pattern::Phase { axis: *axis, step })
                .unwrap()
                .intensity;
            let angle = TAU * step as f64 / p.phase_steps as f64;
            c += intensity * angle.cos();
            s -= intensity * angle.sin();
        }
        let modulation = 2.0 * c.hypot(s) / p.phase_steps as f64;
        if modulation < p.minimum_modulation {
            return Err(MetrologyError::LowSignal);
        }
        min_mod = min_mod.min(modulation);
        let wrapped = s.atan2(c).rem_euclid(TAU) / TAU * p.phase_period_px;
        coord[index] =
            wrapped + ((binary as f64 - wrapped) / p.phase_period_px).round() * p.phase_period_px;
        if (coord[index] - binary as f64).abs() > p.maximum_gray_phase_disagreement_px {
            return Err(MetrologyError::InconsistentObservations);
        }
        let phase_sigma =
            (2.0 / p.phase_steps as f64).sqrt() * config.errors.phase_intensity_sigma / modulation;
        sigma[index] = ((phase_sigma * p.phase_period_px / TAU).powi(2)
            + config.errors.correspondence_sigma_px.powi(2))
        .sqrt()
        .max(1e-9);
    }
    let pixel = Vec2::new(coord[0], coord[1]);
    if !p.projector_size.contains(pixel) {
        return Err(MetrologyError::OutsideField);
    }
    Ok(DecodedProjectorPixel {
        pixel,
        sigma_px: Vec2::new(sigma[0], sigma[1]),
        modulation: min_mod,
        sequence_start_s: start,
        sequence_end_s: end,
        available_s: frames
            .iter()
            .map(|f| f.timing.available_s)
            .fold(end, f64::max),
        sequence_id: first.timing.sequence_id,
        frame_count: frames.len(),
    })
}
