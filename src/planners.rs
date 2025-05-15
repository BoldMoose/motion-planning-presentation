pub mod rrt;

use crate::environment::*;
use crate::spaces::*;

use std::fs::File;
use std::io;
use std::io::Write;
use std::time::Duration;

pub struct PlannerSolution {
    pub success: bool,
    pub time: Duration,
    pub path: Vec<(f64, f64)>,
    pub tree_size: usize,
}

impl PlannerSolution {
    fn new() -> Self {
        Self {
            success: false,
            time: Duration::from_secs(0),
            path: vec![],
            tree_size: 0,
        }
    }
    pub fn export(&self, filename: &str) -> io::Result<()> {
        let mut file = File::create(filename)?;

        for (x, y) in self.path.iter() {
            writeln!(file, "{},{}", x, y)?;
        }

        Ok(())
    }
    pub fn path_length(&self) -> f64 {
        self.path
            .windows(2)
            .map(|w| {
                let (x0, y0) = w[0];
                let (x1, y1) = w[1];
                let dx = x1 - x0;
                let dy = y1 - y0;
                (dx * dx + dy * dy).sqrt()
            })
            .sum()
    }
}

pub trait Planner<S: StateSpace> {
    fn solve(&mut self, t: Duration) -> PlannerSolution;
}
