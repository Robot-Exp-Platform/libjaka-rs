use pyo3::{pyclass, pymethods, types::PyAnyMethods};
use robot_behavior::{
    behavior::*, py_arm_behavior, py_arm_param, py_arm_preplanned_motion,
    py_arm_preplanned_motion_ext, py_arm_preplanned_motion_impl, py_arm_real_time_control,
    py_arm_real_time_control_ext, py_robot_behavior,
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

py_robot_behavior!(PyJakaMini2(JakaMini2));
py_arm_behavior!(PyJakaMini2<{6}>(JakaMini2));
py_arm_param!(PyJakaMini2<{6}>(JakaMini2));
py_arm_preplanned_motion!(PyJakaMini2<{6}>(JakaMini2));
py_arm_preplanned_motion_impl!(PyJakaMini2<{6}>(JakaMini2));
py_arm_preplanned_motion_ext!(PyJakaMini2<{6}>(JakaMini2));
py_arm_real_time_control!(PyJakaMini2<{6}>(JakaMini2));
py_arm_real_time_control_ext!(PyJakaMini2<{6}>(JakaMini2));
