use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmDOF, ArmForwardKinematics, ArmParam, DhParam, RobotFile, to_radians_array,
};

use crate::{JakaRobot, JakaType};

pub struct _JakaS5;
pub struct _JakaS7;
pub struct _JakaS12;

impl JakaType for _JakaS5 {
    const N: usize = 6;
}

impl JakaType for _JakaS7 {
    const N: usize = 6;
}

impl JakaType for _JakaS12 {
    const N: usize = 6;
}

pub type JakaS5 = JakaRobot<_JakaS5, { _JakaS5::N }>;
pub type JakaS7 = JakaRobot<_JakaS7, { _JakaS7::N }>;
pub type JakaS12 = JakaRobot<_JakaS12, { _JakaS12::N }>;

impl ArmParam<{ _JakaS5::N }> for JakaS5 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaS5::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = [PI; Self::N];
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
}

impl ArmParam<{ _JakaS7::N }> for JakaS7 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaS7::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = [
        -PI * 2.,
        -85. * PI / 180.,
        -175. * PI / 180.,
        -85. * PI / 180.,
        -PI * 2.,
        -PI * 2.,
    ];
    const JOINT_MAX: [f64; Self::N] = [
        PI * 2.,
        265. * PI / 180.,
        175. * PI / 180.,
        265. * PI / 180.,
        PI * 2.,
        PI * 2.,
    ];
    const JOINT_VEL_BOUND: [f64; Self::N] = [PI; Self::N];
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 2.5;
}

impl ArmParam<{ _JakaS12::N }> for JakaS12 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaS12::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = [
        -PI * 2.,
        -85. * PI / 180.,
        -175. * PI / 180.,
        -85. * PI / 180.,
        -PI * 2.,
        -PI * 2.,
    ];
    const JOINT_MAX: [f64; Self::N] = [
        PI * 2.,
        265. * PI / 180.,
        175. * PI / 180.,
        265. * PI / 180.,
        PI * 2.,
        PI * 2.,
    ];
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 120., 180., 180., 180.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
}

impl ArmForwardKinematics<{ _JakaS5::N }> for JakaS5 {
    const DH: [robot_behavior::DhParam; _JakaS5::N] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1175, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaS7::N }> for JakaS7 {
    const DH: [robot_behavior::DhParam; _JakaS7::N] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.360, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3035, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.11501, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1175, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaS12::N }> for JakaS12 {
    const DH: [robot_behavior::DhParam; _JakaS12::N] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.595, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.5715, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.1315, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.112, r: 0., alpha: 0. },
    ];
}

impl RobotFile for JakaS5 {
    const URDF: &'static str = "jaka/jaka_s5.urdf";
}

impl RobotFile for JakaS7 {
    const URDF: &'static str = "jaka/jaka_s7.urdf";
}

impl RobotFile for JakaS12 {
    const URDF: &'static str = "jaka/jaka_s12.urdf";
}
