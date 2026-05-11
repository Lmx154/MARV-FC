use crate::localization::estimation::core::Quaternion;

pub(crate) fn assert_scalar_near(actual: f32, expected: f32, tolerance: f32) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual} expected={expected} tolerance={tolerance}"
    );
}

pub(crate) fn assert_scalar_near_f64(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual} expected={expected} tolerance={tolerance}"
    );
}

pub(crate) fn assert_vec3_near(actual: [f32; 3], expected: [f32; 3], tolerance: f32) {
    for axis in 0..3 {
        assert_scalar_near(actual[axis], expected[axis], tolerance);
    }
}

pub(crate) fn assert_vec3_near_f64(actual: [f64; 3], expected: [f64; 3], tolerance: f64) {
    for axis in 0..3 {
        assert_scalar_near_f64(actual[axis], expected[axis], tolerance);
    }
}

pub(crate) fn assert_quaternion_near(actual: [f32; 4], expected: [f32; 4], tolerance: f32) {
    for component in 0..4 {
        assert_scalar_near(actual[component], expected[component], tolerance);
    }
}

pub(crate) fn assert_quaternion_near_f64(
    actual: Quaternion<f64>,
    expected: Quaternion<f64>,
    tolerance: f64,
) {
    for component in 0..4 {
        assert_scalar_near_f64(actual[component], expected[component], tolerance);
    }
}

pub(crate) fn assert_quaternion_normalized_f64(quaternion: Quaternion<f64>, tolerance: f64) {
    assert_scalar_near_f64(quaternion.norm(), 1.0, tolerance);
}
