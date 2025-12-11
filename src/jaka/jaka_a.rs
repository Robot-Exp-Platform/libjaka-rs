use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmDOF, ArmForwardKinematics, ArmParam, DhParam, RobotFile, to_radians_array,
};

use crate::{JakaRobot, JakaType};

pub struct _JakaA5;
pub struct _JakaA12;
pub struct _JakaA20;

impl JakaType for _JakaA5 {
    const N: usize = 6;
}
impl JakaType for _JakaA12 {
    const N: usize = 6;
}
impl JakaType for _JakaA20 {
    const N: usize = 6;
}

pub type JakaA5 = JakaRobot<_JakaA5, { _JakaA5::N }>;
pub type JakaA12 = JakaRobot<_JakaA12, { _JakaA12::N }>;
pub type JakaA20 = JakaRobot<_JakaA20, { _JakaA20::N }>;

impl ArmParam<{ _JakaA5::N }> for JakaA5 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaA5::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = [-PI * 2.; Self::N];
    const JOINT_MAX: [f64; Self::N] = [PI * 2.; Self::N];
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([210., 210., 210., 265., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
}

impl ArmParam<{ _JakaA12::N }> for JakaA12 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaA5::N] = to_radians_array([0., 90., 180., -180., 180., 90.]);
    const JOINT_MIN: [f64; Self::N] = [-PI * 2.; Self::N];
    const JOINT_MAX: [f64; Self::N] = [PI * 2.; Self::N];
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([150., 150., 210., 210., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];
    const CARTESIAN_VEL_BOUND: f64 = 4.0;
}

impl ArmParam<{ _JakaA20::N }> for JakaA20 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaA5::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 120., 265., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];
    const CARTESIAN_VEL_BOUND: f64 = 5.;
}

impl ArmForwardKinematics<{ _JakaA5::N }> for JakaA5 {
    const DH: [robot_behavior::DhParam; _JakaA5::N] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaA12::N }> for JakaA12 {
    const DH: [robot_behavior::DhParam; _JakaA12::N] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.770, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.03750, r: 0., alpha: -FRAC_PI_2},
        DhParam::DH { theta: 0., d: 0.5515, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaA20::N }> for JakaA20 {
    const DH: [robot_behavior::DhParam; _JakaA20::N] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.897, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.7445, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.18835, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1385, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1205, r: 0., alpha: 0. },
    ];
}

impl RobotFile for _JakaA5 {
    const URDF: &'static str = "jaka/jaka_a5.urdf";
}

impl RobotFile for _JakaA12 {
    const URDF: &'static str = "jaka/jaka_a12.urdf";
}

impl RobotFile for _JakaA20 {
    const URDF: &'static str = "jaka/jaka_a20.urdf";
}
