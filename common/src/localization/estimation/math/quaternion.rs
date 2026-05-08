//! Quaternion helpers for estimator states and process models.

use crate::localization::estimation::core::{EstimatorScalar, Mat3, Quaternion, Vec3, scalar};

pub fn identity_quaternion<T: EstimatorScalar>() -> Quaternion<T> {
    Quaternion::<T>::new(T::one(), T::zero(), T::zero(), T::zero())
}

pub fn normalize_quaternion<T: EstimatorScalar>(quaternion: &Quaternion<T>) -> Quaternion<T> {
    let norm = quaternion.norm();
    if norm <= T::default_epsilon() {
        return identity_quaternion();
    }
    quaternion / norm
}

pub fn quaternion_multiply<T: EstimatorScalar>(
    lhs: &Quaternion<T>,
    rhs: &Quaternion<T>,
) -> Quaternion<T> {
    let (w1, x1, y1, z1) = (lhs[0], lhs[1], lhs[2], lhs[3]);
    let (w2, x2, y2, z2) = (rhs[0], rhs[1], rhs[2], rhs[3]);
    Quaternion::<T>::new(
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )
}

pub fn quaternion_inverse<T: EstimatorScalar>(quaternion: &Quaternion<T>) -> Quaternion<T> {
    let norm_sq = quaternion.dot(quaternion);
    if norm_sq <= T::default_epsilon() {
        return identity_quaternion();
    }
    Quaternion::<T>::new(
        quaternion[0] / norm_sq,
        -quaternion[1] / norm_sq,
        -quaternion[2] / norm_sq,
        -quaternion[3] / norm_sq,
    )
}

pub fn rotation_vector_to_quaternion<T: EstimatorScalar>(
    rotation_vector: &Vec3<T>,
) -> Quaternion<T> {
    let angle = rotation_vector.norm();
    if angle <= T::default_epsilon() {
        let half = scalar::<T>(0.5);
        return normalize_quaternion(&Quaternion::<T>::new(
            T::one(),
            rotation_vector[0] * half,
            rotation_vector[1] * half,
            rotation_vector[2] * half,
        ));
    }

    let axis = rotation_vector / angle;
    let half_angle = angle / (T::one() + T::one());
    Quaternion::<T>::new(
        half_angle.cos(),
        axis[0] * half_angle.sin(),
        axis[1] * half_angle.sin(),
        axis[2] * half_angle.sin(),
    )
}

pub fn quaternion_to_rotation_matrix<T: EstimatorScalar>(quaternion: &Quaternion<T>) -> Mat3<T> {
    let q = normalize_quaternion(quaternion);
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let two = scalar::<T>(2.0);

    Mat3::<T>::new(
        T::one() - two * (y * y + z * z),
        two * (x * y - w * z),
        two * (x * z + w * y),
        two * (x * y + w * z),
        T::one() - two * (x * x + z * z),
        two * (y * z - w * x),
        two * (x * z - w * y),
        two * (y * z + w * x),
        T::one() - two * (x * x + y * y),
    )
}
