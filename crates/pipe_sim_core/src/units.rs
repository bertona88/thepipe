//! Lightweight strongly typed SI scalar quantities.
//!
//! Simulation structs generally expose suffixed `f64` fields for convenient
//! FFI.  These newtypes are useful at construction boundaries where mixing
//! millimetres, radians, and seconds would otherwise be easy.

use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

macro_rules! scalar_quantity {
    ($name:ident, $ctor:ident, $getter:ident) => {
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
        pub struct $name(pub f64);

        impl $name {
            pub const ZERO: Self = Self(0.0);

            #[inline]
            pub const fn $ctor(value: f64) -> Self {
                Self(value)
            }

            #[inline]
            pub const fn $getter(self) -> f64 {
                self.0
            }

            #[inline]
            pub fn abs(self) -> Self {
                Self(self.0.abs())
            }

            #[inline]
            pub fn is_finite(self) -> bool {
                self.0.is_finite()
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self::Output {
                Self(self.0 + rhs.0)
            }
        }

        impl AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self::Output {
                Self(self.0 - rhs.0)
            }
        }

        impl SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }

        impl Mul<f64> for $name {
            type Output = Self;
            fn mul(self, rhs: f64) -> Self::Output {
                Self(self.0 * rhs)
            }
        }

        impl Div<f64> for $name {
            type Output = Self;
            fn div(self, rhs: f64) -> Self::Output {
                Self(self.0 / rhs)
            }
        }

        impl Neg for $name {
            type Output = Self;
            fn neg(self) -> Self::Output {
                Self(-self.0)
            }
        }
    };
}

scalar_quantity!(Length, from_metres, metres);
scalar_quantity!(Angle, from_radians, radians);
scalar_quantity!(Force, from_newtons, newtons);
scalar_quantity!(Torque, from_newton_metres, newton_metres);
scalar_quantity!(Mass, from_kilograms, kilograms);
scalar_quantity!(Time, from_seconds, seconds);

impl Length {
    #[inline]
    pub const fn from_millimetres(value: f64) -> Self {
        Self(value * 1.0e-3)
    }

    #[inline]
    pub const fn from_micrometres(value: f64) -> Self {
        Self(value * 1.0e-6)
    }

    #[inline]
    pub const fn millimetres(self) -> f64 {
        self.0 * 1.0e3
    }

    #[inline]
    pub const fn micrometres(self) -> f64 {
        self.0 * 1.0e6
    }
}

impl Angle {
    #[inline]
    pub fn from_degrees(value: f64) -> Self {
        Self(value.to_radians())
    }

    #[inline]
    pub fn degrees(self) -> f64 {
        self.0.to_degrees()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn micro_and_millimetre_conversions_are_exact_enough() {
        assert!((Length::from_millimetres(2.5).micrometres() - 2_500.0).abs() < 1e-12);
        assert!((Length::from_micrometres(125.0).millimetres() - 0.125).abs() < 1e-12);
    }

    #[test]
    fn angle_conversion_round_trips() {
        assert!((Angle::from_degrees(90.0).radians() - core::f64::consts::FRAC_PI_2).abs() < 1e-14);
        assert!((Angle::from_radians(core::f64::consts::PI).degrees() - 180.0).abs() < 1e-12);
    }
}
