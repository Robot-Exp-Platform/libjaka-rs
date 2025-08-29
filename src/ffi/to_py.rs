use pyo3::{pyclass, pymethods, types::PyAnyMethods};
use robot_behavior::{
    behavior::*, py_arm_behavior, py_arm_param, py_arm_preplanned_motion,
    py_arm_preplanned_motion_ext, py_arm_preplanned_motion_impl, py_arm_real_time_control,
    py_arm_real_time_control_ext, py_robot_behavior,
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
