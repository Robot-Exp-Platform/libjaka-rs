use std::time::Duration;

use pyo3::{
    pyclass, pymethods,
    types::{PyAnyMethods, PyModule, PyModuleMethods},
    wrap_pyfunction,
};
use robot_behavior::{
    ArmState, LoadState, PyArmState, PyControlType, PyMotionType, PyPose, behavior::*,
    py_arm_behavior, py_arm_param, py_arm_preplanned_motion, py_arm_preplanned_motion_ext,
    py_arm_preplanned_motion_impl, py_arm_real_time_control, py_arm_real_time_control_ext,
    py_robot_behavior,
};

use crate::JakaRobot;

#[pyclass(name = "JakaRobot")]
pub struct PyJakaRobot(JakaRobot);

#[pymethods]
impl PyJakaRobot {
    #[new]
    pub fn new(ip: &str) -> Self {
        PyJakaRobot(JakaRobot::new(ip))
    }
}

py_robot_behavior!(PyJakaRobot(JakaRobot));
py_arm_behavior!(PyJakaRobot<{6}>(JakaRobot));
py_arm_param!(PyJakaRobot<{6}>(JakaRobot));
py_arm_preplanned_motion!(PyJakaRobot<{6}>(JakaRobot));
py_arm_preplanned_motion_impl!(PyJakaRobot<{6}>(JakaRobot));
py_arm_preplanned_motion_ext!(PyJakaRobot<{6}>(JakaRobot));
py_arm_real_time_control!(PyJakaRobot<{6}>(JakaRobot));
py_arm_real_time_control_ext!(PyJakaRobot<{6}>(JakaRobot));

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

#[pyo3::pymodule]
fn libjaka(m: &pyo3::Bound<'_, PyModule>) -> pyo3::PyResult<()> {
    m.add_class::<PyJakaRobot>()?;
    m.add_class::<PyPose>()?;
    m.add_class::<LoadState>()?;
    m.add_class::<PyArmState>()?;
    m.add_class::<PyMotionType>()?;
    m.add_class::<PyControlType>()?;
    m.add_function(wrap_pyfunction!(test_closure, m)?)?;
    Ok(())
}
