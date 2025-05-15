use crate::utility::angle::signed_angle_diff;

pub fn euclidean(a: &f64, b: &f64) -> f64 {
    ((a).powi(2) + (b).powi(2)).sqrt()
}

pub fn angular(a: &f64, b: &f64) -> f64 {
    signed_angle_diff(a, b).abs()
}
