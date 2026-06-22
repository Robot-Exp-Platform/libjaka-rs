use pyo3::{pyclass, pymethods};
use robot_behavior::{
    py_arm, py_flange_move, py_joint_motion, py_joint_position_control, py_robot,
};

use crate::JakaMini2;

#[pyclass(name = "JakaMini2")]
pub struct PyJakaMini2(JakaMini2);

#[pymethods]
impl PyJakaMini2 {
    #[new]
    pub fn new(ip: &str) -> Self {
        PyJakaMini2(JakaMini2::new(ip))
    }
}

py_robot!(PyJakaMini2(JakaMini2));
py_arm!(PyJakaMini2<{6}>(JakaMini2));
py_joint_motion!(PyJakaMini2<{6}>(JakaMini2));
py_flange_move!(PyJakaMini2(JakaMini2));
py_joint_position_control!(PyJakaMini2<{6}>(JakaMini2));
