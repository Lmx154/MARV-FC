//! Rotation-space helpers shared by process and measurement models.

use crate::localization::estimation::core::{EstimatorScalar, Mat3, Vec3};

pub fn skew_symmetric<T: EstimatorScalar>(vector: &Vec3<T>) -> Mat3<T> {
    let (x, y, z) = (vector[0], vector[1], vector[2]);
    Mat3::<T>::new(T::zero(), -z, y, z, T::zero(), -x, -y, x, T::zero())
}
