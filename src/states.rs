use rand::Rng;

pub use crate::spaces::StateSpace;
use crate::utility::distance::{angular, euclidean};
pub use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct GeoState {
    pub x: f64,
    pub y: f64,
}

impl StateSpace for GeoState {
    fn min() -> Self {
        GeoState { x: 0.0, y: 0.0 }
    }
    fn max() -> Self {
        GeoState { x: 10.0, y: 10.0 }
    }
    fn sample() -> Self {
        let mut rng = rand::rng();
        GeoState {
            x: rng.random_range(Self::min().x..=Self::max().x),
            y: rng.random_range(Self::min().y..=Self::max().y),
        }
    }
    fn contains(other: &Self) -> bool {
        {
            return other.x >= Self::min().x
                && other.x <= Self::max().x
                && other.y >= Self::min().y
                && other.y <= Self::max().y;
        }
    }
    fn clamp(&mut self) -> Self {
        self.x = self.x.clamp(Self::min().x, Self::max().x);
        self.y = self.y.clamp(Self::min().y, Self::max().y);
        *self
    }
    fn distance_heuristic(a: &Self, b: &Self) -> f64 {
        let s = Self::span();
        euclidean(&(a.x - b.x), &(a.y - b.y)) / euclidean(&s.x, &s.y)
    }
    fn span() -> Self {
        GeoState {
            x: Self::max().x - Self::min().x,
            y: Self::max().y - Self::min().y,
        }
    }
    fn position(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct VelState {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl StateSpace for VelState {
    fn min() -> Self {
        VelState {
            x: 0.0,
            y: 0.0,
            theta: -PI,
        }
    }
    fn max() -> Self {
        VelState {
            x: 10.0,
            y: 10.0,
            theta: PI,
        }
    }
    fn sample() -> Self {
        let mut rng = rand::rng();
        VelState {
            x: rng.random_range(Self::min().x..=Self::max().x),
            y: rng.random_range(Self::min().y..=Self::max().y),
            theta: rng.random_range(Self::min().theta..=Self::max().theta),
        }
    }
    fn contains(other: &Self) -> bool {
        {
            return other.x >= Self::min().x
                && other.x <= Self::max().x
                && other.y >= Self::min().y
                && other.y <= Self::max().y
                && other.theta >= Self::min().theta
                && other.theta <= Self::max().theta;
        }
    }
    fn clamp(&mut self) -> Self {
        self.x = self.x.clamp(Self::min().x, Self::max().x);
        self.y = self.y.clamp(Self::min().y, Self::max().y);
        self.theta = self.theta.clamp(Self::min().theta, Self::max().theta);
        *self
    }
    fn distance_heuristic(a: &Self, b: &Self) -> f64 {
        let s = Self::span();
        let w_d = euclidean(&(a.x - b.x), &(a.y - b.y)) / euclidean(&s.x, &s.y);
        let w_a = angular(&a.theta, &b.theta) / (s.theta / 2.0);
        w_d + w_a
    }
    fn span() -> Self {
        VelState {
            x: Self::max().x - Self::min().x,
            y: Self::max().y - Self::min().y,
            theta: Self::max().theta - Self::min().theta,
        }
    }
    fn position(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AccState {
    pub x: f64,
    pub y: f64,
    pub v: f64,
    pub theta: f64,
}

impl StateSpace for AccState {
    fn min() -> Self {
        AccState {
            x: 0.0,
            y: 0.0,
            v: 0.0,
            theta: -PI,
        }
    }
    fn max() -> Self {
        AccState {
            x: 10.0,
            y: 10.0,
            v: 2.0,
            theta: PI,
        }
    }
    fn sample() -> Self {
        let mut rng = rand::rng();
        AccState {
            x: rng.random_range(Self::min().x..=Self::max().x),
            y: rng.random_range(Self::min().y..=Self::max().y),
            v: rng.random_range(Self::min().v..=Self::max().v),
            theta: rng.random_range(Self::min().theta..=Self::max().theta),
        }
    }
    fn contains(other: &Self) -> bool {
        {
            return other.x >= Self::min().x
                && other.x <= Self::max().x
                && other.y >= Self::min().y
                && other.y <= Self::max().y
                && other.v >= Self::min().v
                && other.v <= Self::max().v
                && other.theta >= Self::min().theta
                && other.theta <= Self::max().theta;
        }
    }
    fn clamp(&mut self) -> Self {
        self.x = self.x.clamp(Self::min().x, Self::max().x);
        self.y = self.y.clamp(Self::min().y, Self::max().y);
        self.v = self.v.clamp(Self::min().v, Self::max().v);
        self.theta = self.theta.clamp(Self::min().theta, Self::max().theta);
        *self
    }
    fn distance_heuristic(a: &Self, b: &Self) -> f64 {
        let s = Self::span();
        let w_d = euclidean(&(a.x - b.x), &(a.y - b.y)) / euclidean(&s.x, &s.y);
        let w_v = (a.v - b.v).abs() / s.v;
        let w_a = angular(&a.theta, &b.theta) / (s.theta / 2.0);
        w_d + w_v + w_a
    }
    fn span() -> Self {
        AccState {
            x: Self::max().x - Self::min().x,
            y: Self::max().y - Self::min().y,
            v: Self::max().v - Self::min().v,
            theta: Self::max().theta - Self::min().theta,
        }
    }
    fn position(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}
