//! Fixed-size estimator math aliases.

use core::fmt::Debug;

use nalgebra::{SMatrix, SVector};
use num_traits::FromPrimitive;

pub trait EstimatorScalar: nalgebra::RealField + Copy + Debug + FromPrimitive {}

impl<T> EstimatorScalar for T where T: nalgebra::RealField + Copy + Debug + FromPrimitive {}

pub type VecN<T, const N: usize> = SVector<T, N>;
pub type MatN<T, const N: usize> = SMatrix<T, N, N>;
pub type MatMN<T, const R: usize, const C: usize> = SMatrix<T, R, C>;
pub type MatNW<T, const N: usize, const W: usize> = SMatrix<T, N, W>;

pub type Vec1<T> = VecN<T, 1>;
pub type Vec3<T> = VecN<T, 3>;
pub type Vec4<T> = VecN<T, 4>;
pub type Mat3<T> = MatN<T, 3>;
pub type Quaternion<T> = Vec4<T>;

#[inline]
pub fn scalar<T: EstimatorScalar>(value: f64) -> T {
    T::from_f64(value).expect("estimator scalar conversion failed")
}
