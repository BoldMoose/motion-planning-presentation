use std::f64::consts::PI;

pub fn wrap(a: f64) -> f64 {
    // Normalize angles to [-PI, PI]
    let mut out = a;
    while out > PI {
        out -= 2.0 * PI;
    }
    while out < -PI {
        out += 2.0 * PI;
    }
    out
}

pub fn signed_angle_diff(actual: &f64, desired: &f64) -> f64 {
    // Calculate the minimum angle to another angle
    let options = [wrap(desired - actual), wrap(desired - actual + PI)];
    if options[0].abs() < options[1].abs() {
        options[0]
    } else {
        options[1]
    }
}
