#![feature(adt_const_params)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

#[cfg(feature = "ffi")]
pub mod ffi;
pub(crate) mod network;
mod params;
mod robot;
pub mod types;

pub use params::*;
pub use robot::*;

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod libjaka {
    use pyo3::types::PyAnyMethods;
    use robot_behavior::ArmState;
    use std::time::Duration;

    #[pymodule_export]
    use super::ffi::to_py::PyJakaRobot;
    #[pymodule_export]
    use robot_behavior::{LoadState, PyArmState, PyControlType, PyMotionType, PyPose};

    #[pyo3::pyfunction]
    fn test_closure(closure: pyo3::Py<pyo3::PyAny>) {
        let state = PyArmState::from(ArmState::<6>::default());
        let duration = Duration::from_secs_f64(0.125).as_secs_f64();
        let Ok((joint, is_finished)) = pyo3::Python::with_gil(|py| {
            closure
                .call1(py, (state, duration))?
                .bind(py)
                .extract::<(PyMotionType, bool)>()
        }) else {
            println!("Failed to call closure");
            todo!()
        };
        println!("Joint: {joint:?}, Is Finished: {is_finished}");
    }
}
