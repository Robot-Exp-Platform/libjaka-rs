use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{ArmForwardKinematics, DhParam, EndPoint, Joints, to_radians_array};

use crate::JakaRobot;

pub struct _JakaA12L;

pub type JakaA12L = JakaRobot<_JakaA12L, 6>;

// URDF 未在 www.jaka.com 提供，不实现 RobotDescription，使用默认 URDF = None。
impl Joints<6> for JakaA12L {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([90., 180., -180., 180., 90., 90.]);
    const JOINT_MIN: [f64; 6] = [-PI * 2.; 6];
    const JOINT_MAX: [f64; 6] = [PI * 2.; 6];
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([150., 150., 210., 210., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaA12L {
    const CARTESIAN_VEL_BOUND: f64 = 4.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaA12L {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14165, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.770, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.03750, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.5515, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1338, r: 0., alpha: 0. },
    ];
}
