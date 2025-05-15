use rand::Rng;

pub use crate::spaces::ControlSpace;
pub use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct GeoControl {
    pub theta: f64,
}

impl ControlSpace for GeoControl {
    fn min() -> Self {
        GeoControl { theta: -PI }
    }
    fn max() -> Self {
        GeoControl { theta: PI }
    }
    fn sample() -> Self {
        let mut rng = rand::rng();
        GeoControl {
            theta: rng.random_range(Self::min().theta..=Self::max().theta),
        }
    }
    fn clamp(&mut self) -> Self {
        self.theta = self.theta.clamp(Self::min().theta, Self::max().theta);
        *self
    }
}

#[derive(Debug, Clone, Copy)]
pub struct VelControl {
    pub omega: f64,
    pub v: f64,
}

impl ControlSpace for VelControl {
    fn min() -> Self {
        VelControl {
            omega: -1.0,
            v: 0.0,
        }
    }
    fn max() -> Self {
        VelControl { omega: 1.0, v: 1.0 }
    }
    fn sample() -> Self {
        let mut rng = rand::rng();
        VelControl {
            omega: rng.random_range(Self::min().omega..=Self::max().omega),
            v: rng.random_range(Self::min().v..=Self::max().v),
        }
    }
    fn clamp(&mut self) -> Self {
        self.omega = self.omega.clamp(Self::min().omega, Self::max().omega);
        self.v = self.v.clamp(Self::min().v, Self::max().v);
        *self
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AccControl {
    pub omega: f64,
    pub a: f64,
}

impl ControlSpace for AccControl {
    fn min() -> Self {
        AccControl {
            omega: -1.0,
            a: -0.5,
        }
    }
    fn max() -> Self {
        AccControl { omega: 1.0, a: 0.5 }
    }
    fn sample() -> Self {
        let mut rng = rand::rng();
        AccControl {
            omega: rng.random_range(Self::min().omega..=Self::max().omega),
            a: rng.random_range(Self::min().a..=Self::max().a),
        }
    }
    fn clamp(&mut self) -> Self {
        self.omega = self.omega.clamp(Self::min().omega, Self::max().omega);
        self.a = self.a.clamp(Self::min().a, Self::max().a);
        *self
    }
}
