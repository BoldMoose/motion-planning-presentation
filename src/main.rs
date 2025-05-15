pub mod controls;
pub mod environment;
pub mod planners;
pub mod spaces;
pub mod states;
pub mod utility;

use environment::{
    ForestAccEnv, ForestGeoEnv, ForestVelEnv, SimpleAccEnv, SimpleGeoEnv, SimpleVelEnv,
};
use planners::rrt::RRT;
use planners::{Planner, PlannerSolution};
use states::*;
use std::time::Duration;

fn metrics(results: &Vec<PlannerSolution>) {
    let n_total = results.len() as f64;
    let successful_runs: Vec<&PlannerSolution> = results.iter().filter(|r| r.success).collect();
    let n_success = successful_runs.len() as f64;
    let success_rate = successful_runs.len() as f64 / n_total;
    let avg_duration = results.iter().map(|r| r.time).sum::<Duration>() / n_total as u32;
    let avg_path_length = successful_runs.iter().map(|r| r.path_length()).sum::<f64>() / n_success;
    let avg_tree_size = results.iter().map(|r| r.tree_size).sum::<usize>() as f64 / n_total;
    println!(
        "{:?}, {:?}, {:?}, {:?}",
        success_rate, avg_duration, avg_path_length, avg_tree_size
    );
}

fn run_scenario<E, S, U>(env: E, scenario: &str)
where
    E: environment::Environment<S, U>,
    S: states::StateSpace + environment::Propagator<U> + environment::SteeringFunction<U>,
    U: controls::ControlSpace,
{
    let mut planner = RRT::new(env, 0.15, 1.0, 0.05);
    let solution = planner.solve(Duration::from_secs(1));
    solution
        .export(format!("results/{}_path.txt", scenario).as_str())
        .unwrap();
    planner
        .export(format!("results/{}_data.txt", scenario).as_str())
        .unwrap();
}

fn run_benchmark<E, S, U>(env: E, n: usize)
where
    E: environment::Environment<S, U>,
    S: states::StateSpace + environment::Propagator<U> + environment::SteeringFunction<U>,
    U: controls::ControlSpace,
{
    let mut results = vec![];
    for _ in 0..n {
        let mut planner = RRT::new(env.clone(), 0.15, 1.0, 0.05);
        results.push(planner.solve(Duration::from_secs(1)));
    }
    metrics(&results);
}

fn main() {
    let si_geo = GeoState { x: 0.0, y: 0.0 };
    let sg_geo = GeoState { x: 9.5, y: 9.5 };
    let si_vel = VelState {
        x: 0.0,
        y: 0.0,
        theta: PI / 4.0,
    };
    let sg_vel = VelState {
        x: 9.5,
        y: 9.5,
        theta: 0.0,
    };
    let si_acc = AccState {
        x: 0.0,
        y: 0.0,
        v: 0.0,
        theta: PI / 4.0,
    };
    let sg_acc = AccState {
        x: 9.5,
        y: 9.5,
        v: 0.0,
        theta: 0.0,
    };

    run_scenario(SimpleGeoEnv::new(si_geo, sg_geo, 0.5), "simplegeo");
    run_scenario(ForestGeoEnv::new(si_geo, sg_geo, 0.5), "forestgeo");
    run_scenario(SimpleVelEnv::new(si_vel, sg_vel, 0.5), "simplevel");
    run_scenario(ForestVelEnv::new(si_vel, sg_vel, 0.5), "forestvel");
    run_scenario(SimpleAccEnv::new(si_acc, sg_acc, 0.5), "simpleacc");
    run_scenario(ForestAccEnv::new(si_acc, sg_acc, 0.5), "forestacc");

    run_benchmark(SimpleGeoEnv::new(si_geo, sg_geo, 0.5), 100);
    run_benchmark(ForestGeoEnv::new(si_geo, sg_geo, 0.5), 100);
    run_benchmark(SimpleVelEnv::new(si_vel, sg_vel, 0.5), 100);
    run_benchmark(ForestVelEnv::new(si_vel, sg_vel, 0.5), 100);
    run_benchmark(SimpleAccEnv::new(si_acc, sg_acc, 0.5), 100);
    run_benchmark(ForestAccEnv::new(si_acc, sg_acc, 0.5), 100);
}
