use pyo3::{pyclass, pymethods};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, RobotBehavior, py_arm_behavior,
    py_arm_preplanned_motion, py_arm_preplanned_motion_ext, py_robot_behavior,
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
py_arm_preplanned_motion!(PyJakaRobot<{6}>(JakaRobot));
py_arm_preplanned_motion_ext!(PyJakaRobot<{6}>(JakaRobot));
py_arm_real_time_control!(PyTestRobot<{0}>(TestRobot));
py_arm_real_time_control_ext!(PyTestRobot<{0}>(TestRobot));

#[pyo3::pymodule]
fn ex_robot(m: &pyo3::Bound<'_, PyModule>) -> pyo3::PyResult<()> {
    m.add_class::<PyJakaRobot>()?;
    m.add_class::<PyPose>()?;
    m.add_class::<PyArmState>()?;
    m.add_class::<LoadState>()?;
    Ok(())
}
