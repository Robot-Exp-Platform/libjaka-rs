use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{ArmDOF, ArmForwardKinematics, ArmParam, DhParam, to_radians_array};

use crate::JakaType;

pub struct _JakaA12L;

impl JakaType for _JakaA12L {
    const N: usize = 6;
}

pub type JakaA12L = crate::JakaRobot<_JakaA12L, { _JakaA12L::N }>;

impl ArmParam<{ _JakaA12L::N }> for JakaA12L {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; Self::N] = to_radians_array([90., 180., -180., 180., 90., 90.]);
    const JOINT_MIN: [f64; Self::N] = [-PI * 2.; Self::N];
    const JOINT_MAX: [f64; Self::N] = [PI * 2.; Self::N];
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([150., 150., 210., 210., 265., 265.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 4.0;
}

impl ArmForwardKinematics<{ _JakaA12L::N }> for JakaA12L {
    const DH: [robot_behavior::DhParam; _JakaA12L::N] = [
        DhParam::DH { theta: 0., d: 0.14165, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.770, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.03750, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.5515, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1338, r: 0., alpha: 0. },
    ];
}

// -- not provided on www.jaka.com --
// impl RobotFile for _JakaA12 {
//     const URDF: &'static str = "jaka/jaka_a12l.urdf";
// }
