use crate::controls::*;
use crate::spaces::*;
use crate::states::*;
use crate::utility::angle;
use crate::utility::distance::euclidean;
use crate::utility::integration;

pub trait Propagator<U: ControlSpace> {
    fn propagate(&self, u: &U, t: f64) -> Self;
}
pub trait SteeringFunction<U: ControlSpace> {
    fn steer(&self, _desired: &Self) -> U {
        return U::sample();
    }
}

impl Propagator<GeoControl> for GeoState {
    fn propagate(&self, u: &GeoControl, t: f64) -> Self {
        Self {
            x: self.x + u.theta.cos() * t,
            y: self.y + u.theta.sin() * t,
        }
    }
}

impl Propagator<VelControl> for VelState {
    fn propagate(&self, u: &VelControl, t: f64) -> Self {
        let n = 10;
        let theta = |t: f64| self.theta + integration::trapezoidal(|_t: f64| u.omega, 0.0, t, n);
        let x =
            |t: f64| self.x + integration::trapezoidal(|t: f64| u.v * theta(t).cos(), 0.0, t, n);
        let y =
            |t: f64| self.y + integration::trapezoidal(|t: f64| u.v * theta(t).sin(), 0.0, t, n);

        Self {
            x: x(t),
            y: y(t),
            theta: angle::wrap(theta(t)),
        }
    }
}

impl Propagator<AccControl> for AccState {
    fn propagate(&self, u: &AccControl, t: f64) -> Self {
        let n = 10;
        let theta = |t: f64| self.theta + integration::trapezoidal(|_t: f64| u.omega, 0.0, t, n);
        let speed = |t: f64| self.v + integration::trapezoidal(|_t: f64| u.a, 0.0, t, n);
        let x = |t: f64| {
            self.x + integration::trapezoidal(|t: f64| speed(t) * theta(t).cos(), 0.0, t, n)
        };
        let y = |t: f64| {
            self.y + integration::trapezoidal(|t: f64| speed(t) * theta(t).sin(), 0.0, t, n)
        };

        Self {
            x: x(t),
            y: y(t),
            v: speed(t),
            theta: angle::wrap(theta(t)),
        }
    }
}

impl SteeringFunction<GeoControl> for GeoState {
    fn steer(&self, desired: &Self) -> GeoControl {
        let dx = desired.x - self.x;
        let dy = desired.y - self.y;

        GeoControl {
            theta: dy.atan2(dx),
        }
        .clamp()
    }
}

impl SteeringFunction<VelControl> for VelState {
    fn steer(&self, desired: &VelState) -> VelControl {
        let dx = desired.x - self.x;
        let dy = desired.y - self.y;

        VelControl {
            omega: angle::signed_angle_diff(&self.theta, &dy.atan2(dx)),
            v: (dx.powi(2) + dy.powi(2)).sqrt(),
        }
        .clamp()
    }
}

impl SteeringFunction<AccControl> for AccState {
    fn steer(&self, desired: &AccState) -> AccControl {
        let dx = desired.x - self.x;
        let dy = desired.y - self.y;
        AccControl {
            omega: angle::signed_angle_diff(&self.theta, &dy.atan2(dx)),
            a: (dx.powi(2) + dy.powi(2)).sqrt() - self.v,
        }
        .clamp()
    }
}

pub trait Environment<S: StateSpace + Propagator<U> + SteeringFunction<U>, U: ControlSpace>:
    Clone
{
    fn initial_state(&self) -> S;
    fn is_goal(&self, state: &S) -> bool;
    fn sample_goal(&self) -> S;
    fn is_valid(&self, state: &S) -> bool {
        S::contains(state)
    }
    fn sample_state(&self) -> S {
        S::sample()
    }
}

const FOREST_OBSTACLES: [[f64; 2]; 10] = [
    [2.0, 2.0],
    [5.0, 2.0],
    [8.0, 2.0],
    [0.5, 5.0],
    [3.5, 5.0],
    [6.5, 5.0],
    [9.5, 5.0],
    [2.0, 8.0],
    [5.0, 8.0],
    [8.0, 8.0],
];

#[derive(Clone)]
pub struct SimpleGeoEnv {
    initial_state: GeoState,
    goal_state: GeoState,
    goal_radius: f64,
}

impl SimpleGeoEnv {
    pub fn new(initial_state: GeoState, goal_state: GeoState, goal_radius: f64) -> Self {
        Self {
            initial_state,
            goal_state,
            goal_radius,
        }
    }
}

impl Environment<GeoState, GeoControl> for SimpleGeoEnv {
    fn initial_state(&self) -> GeoState {
        self.initial_state
    }
    fn is_goal(&self, state: &GeoState) -> bool {
        euclidean(
            &(self.goal_state.x - state.x),
            &(self.goal_state.y - state.y),
        ) < self.goal_radius
    }
    fn sample_goal(&self) -> GeoState {
        self.goal_state
    }
}

#[derive(Clone)]
pub struct SimpleVelEnv {
    initial_state: VelState,
    goal_state: VelState,
    goal_radius: f64,
}

impl SimpleVelEnv {
    pub fn new(initial_state: VelState, goal_state: VelState, goal_radius: f64) -> Self {
        Self {
            initial_state,
            goal_state,
            goal_radius,
        }
    }
}

impl Environment<VelState, VelControl> for SimpleVelEnv {
    fn initial_state(&self) -> VelState {
        self.initial_state
    }
    fn is_goal(&self, state: &VelState) -> bool {
        euclidean(
            &(self.goal_state.x - state.x),
            &(self.goal_state.y - state.y),
        ) < self.goal_radius
    }
    fn sample_goal(&self) -> VelState {
        self.goal_state
    }
}

#[derive(Clone)]
pub struct SimpleAccEnv {
    initial_state: AccState,
    goal_state: AccState,
    goal_radius: f64,
}

impl SimpleAccEnv {
    pub fn new(initial_state: AccState, goal_state: AccState, goal_radius: f64) -> Self {
        Self {
            initial_state,
            goal_state,
            goal_radius,
        }
    }
}

impl Environment<AccState, AccControl> for SimpleAccEnv {
    fn initial_state(&self) -> AccState {
        self.initial_state
    }
    fn is_goal(&self, state: &AccState) -> bool {
        euclidean(
            &(self.goal_state.x - state.x),
            &(self.goal_state.y - state.y),
        ) < self.goal_radius
    }
    fn sample_goal(&self) -> AccState {
        self.goal_state
    }
}

#[derive(Clone)]
pub struct ForestGeoEnv {
    initial_state: GeoState,
    goal_state: GeoState,
    goal_radius: f64,
}

impl ForestGeoEnv {
    pub fn new(initial_state: GeoState, goal_state: GeoState, goal_radius: f64) -> Self {
        Self {
            initial_state,
            goal_state,
            goal_radius,
        }
    }
}

impl Environment<GeoState, GeoControl> for ForestGeoEnv {
    fn initial_state(&self) -> GeoState {
        self.initial_state
    }
    fn is_goal(&self, state: &GeoState) -> bool {
        euclidean(
            &(self.goal_state.x - state.x),
            &(self.goal_state.y - state.y),
        ) < self.goal_radius
    }
    fn sample_goal(&self) -> GeoState {
        self.goal_state
    }
    fn is_valid(&self, state: &GeoState) -> bool {
        let valid_state = GeoState::contains(state);
        let obstacles = FOREST_OBSTACLES;
        valid_state
            && obstacles.into_iter().all(|obstacle| {
                euclidean(&(state.x - obstacle[0]), &(state.y - obstacle[1])) >= 1.0
            })
    }
}

#[derive(Clone)]
pub struct ForestVelEnv {
    initial_state: VelState,
    goal_state: VelState,
    goal_radius: f64,
}

impl ForestVelEnv {
    pub fn new(initial_state: VelState, goal_state: VelState, goal_radius: f64) -> Self {
        Self {
            initial_state,
            goal_state,
            goal_radius,
        }
    }
}

impl Environment<VelState, VelControl> for ForestVelEnv {
    fn initial_state(&self) -> VelState {
        self.initial_state
    }
    fn is_goal(&self, state: &VelState) -> bool {
        euclidean(
            &(self.goal_state.x - state.x),
            &(self.goal_state.y - state.y),
        ) < self.goal_radius
    }
    fn sample_goal(&self) -> VelState {
        self.goal_state
    }
    fn is_valid(&self, state: &VelState) -> bool {
        let valid_state = VelState::contains(state);
        let obstacles = FOREST_OBSTACLES;
        valid_state
            && obstacles.into_iter().all(|obstacle| {
                euclidean(&(state.x - obstacle[0]), &(state.y - obstacle[1])) >= 1.0
            })
    }
}

#[derive(Clone)]
pub struct ForestAccEnv {
    initial_state: AccState,
    goal_state: AccState,
    goal_radius: f64,
}

impl ForestAccEnv {
    pub fn new(initial_state: AccState, goal_state: AccState, goal_radius: f64) -> Self {
        Self {
            initial_state,
            goal_state,
            goal_radius,
        }
    }
}

impl Environment<AccState, AccControl> for ForestAccEnv {
    fn initial_state(&self) -> AccState {
        self.initial_state
    }
    fn is_goal(&self, state: &AccState) -> bool {
        euclidean(
            &(self.goal_state.x - state.x),
            &(self.goal_state.y - state.y),
        ) < self.goal_radius
    }
    fn sample_goal(&self) -> AccState {
        self.goal_state
    }
    fn is_valid(&self, state: &AccState) -> bool {
        let valid_state = AccState::contains(state);
        let obstacles = FOREST_OBSTACLES;
        valid_state
            && obstacles.into_iter().all(|obstacle| {
                euclidean(&(state.x - obstacle[0]), &(state.y - obstacle[1])) >= 1.0
            })
    }
}
