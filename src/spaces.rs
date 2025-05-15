pub use rand::Rng;
use std::fmt::Debug;

pub trait StateSpace: Clone + Copy + Debug {
    fn min() -> Self;
    fn max() -> Self;
    fn clamp(&mut self) -> Self;
    fn sample() -> Self;
    fn contains(other: &Self) -> bool;
    fn distance_heuristic(a: &Self, b: &Self) -> f64;
    fn span() -> Self;
    fn position(&self) -> (f64, f64);
}
pub trait ControlSpace: Clone + Copy + Debug {
    fn min() -> Self;
    fn max() -> Self;
    fn clamp(&mut self) -> Self;
    fn sample() -> Self;
    fn from_states<T: StateSpace>(&self, _a: &T, _b: &T) -> Self {
        return Self::sample();
    }
}
