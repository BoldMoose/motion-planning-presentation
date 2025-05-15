use super::*;

use std::marker::PhantomData;
use std::time::{Duration, Instant};

pub struct RRT<E, S, U>
where
    E: Environment<S, U>,
    S: StateSpace + Propagator<U> + SteeringFunction<U>,
    U: ControlSpace,
{
    nodes: Vec<(S, Option<usize>)>, // Store states and their parent indices
    step_size: f64,
    max_dist: f64,
    bias: f64,
    environment: E,
    _marker: PhantomData<U>,
}

impl<E, S, U> RRT<E, S, U>
where
    E: Environment<S, U>,
    S: StateSpace + Propagator<U> + SteeringFunction<U>,
    U: ControlSpace,
{
    pub fn new(environment: E, step_size: f64, max_dist: f64, bias: f64) -> Self {
        let mut nodes = Vec::new();
        nodes.push((environment.initial_state(), None)); // Add the initial state with no parent
        Self {
            nodes,
            step_size,
            max_dist,
            bias,
            environment,
            _marker: PhantomData,
        }
    }

    fn nearest(&self, target: &S) -> usize {
        let mut nearest_index = 0;
        let mut nearest_distance = f64::MAX;

        for (index, (state, _)) in self.nodes.iter().enumerate() {
            let distance = S::distance_heuristic(state, target);
            if distance < nearest_distance {
                nearest_index = index;
                nearest_distance = distance;
            }
        }

        nearest_index
    }

    fn path(&self) -> Vec<(f64, f64)> {
        let mut path = Vec::new();
        let mut current = Some(self.nodes.len() - 1);
        while let Some(index) = current {
            path.push(self.nodes[index].0.position());
            current = self.nodes[index].1;
        }
        path.reverse();
        path
    }

    pub fn export(&self, filename: &str) -> std::io::Result<()> {
        let mut file = std::fs::File::create(filename)?;
        for (state, _) in &self.nodes {
            writeln!(file, "{},{}", state.position().0, state.position().1,)?;
        }
        Ok(())
    }
}

impl<E, S, U> Planner<S> for RRT<E, S, U>
where
    E: Environment<S, U>,
    S: StateSpace + Propagator<U> + SteeringFunction<U>,
    U: ControlSpace,
{
    fn solve(&mut self, t: Duration) -> PlannerSolution {
        let mut solution = PlannerSolution::new();
        let start_time = Instant::now();
        while start_time.elapsed() < t {
            // Sample a random state or the goal state
            let s_rand = if rand::random::<f64>() < self.bias {
                self.environment.sample_goal()
            } else {
                self.environment.sample_state()
            };

            // Find the nearest state in the tree
            let mut i_near = self.nearest(&s_rand);
            let mut s_near = self.nodes[i_near].0;

            // Construct control from the nearest state to the random sample
            let u = s_near.steer(&s_rand);

            // Generate a new state by propagating from the nearest state, in the direction of the random sample
            let mut s_new = s_near.propagate(&u, self.step_size);
            let mut total_distance = self.step_size;

            while self.environment.is_valid(&s_new) && total_distance < self.max_dist {
                // Add the new state to the tree
                self.nodes.push((s_new.clone(), Some(i_near)));

                // Check if the new state is close to the goal
                if self.environment.is_goal(&s_new) {
                    solution.success = true;
                    solution.time = start_time.elapsed();
                    solution.path = self.path();
                    solution.tree_size = self.nodes.len();

                    return solution;
                }

                s_near = s_new;
                i_near = self.nodes.len() - 1;
                s_new = s_near.propagate(&u, self.step_size);
                total_distance = total_distance + self.step_size;
            }
        }

        solution.time = start_time.elapsed();
        solution.tree_size = self.nodes.len();
        return solution;
    }
}
