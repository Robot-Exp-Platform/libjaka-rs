#![feature(adt_const_params)]
#![allow(incomplete_features)]

pub mod assets;
#[cfg(feature = "ffi")]
pub mod ffi;
mod jaka;
pub(crate) mod network;
mod params;
mod robot;
pub mod robot_impl;
pub mod types;

pub use jaka::*;
pub use params::*;
pub use robot::*;

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod libjaka {
    #[pymodule_export]
    use super::ffi::to_py::PyJakaMini2;
    #[pymodule_export]
    use roplat_behavior_ffi::{
        PyArmState, PyJointSample, PyJointState, PyLoadState, PyMotionType, PyPose,
        PySpatialSample, PySpatialState,
    };
}
