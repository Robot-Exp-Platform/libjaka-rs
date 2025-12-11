use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmDOF, ArmForwardKinematics, ArmParam, DhParam, RobotFile, to_radians_array,
};

use crate::{JakaRobot, JakaType};

pub struct _JakaZu3;
pub struct _JakaZu5;
pub struct _JakaZu7;
pub struct _JakaZu12;
pub struct _JakaZu18;
pub struct _JakaZu20;
pub struct _JakaZu30;

impl JakaType for _JakaZu3 {
    const N: usize = 6;
}

impl JakaType for _JakaZu5 {
    const N: usize = 6;
}

impl JakaType for _JakaZu7 {
    const N: usize = 6;
}

impl JakaType for _JakaZu12 {
    const N: usize = 6;
}

impl JakaType for _JakaZu18 {
    const N: usize = 6;
}

impl JakaType for _JakaZu20 {
    const N: usize = 6;
}

impl JakaType for _JakaZu30 {
    const N: usize = 6;
}

pub type JakaZu3 = JakaRobot<_JakaZu3, { _JakaZu3::N }>;
pub type JakaZu5 = JakaRobot<_JakaZu5, { _JakaZu5::N }>;
pub type JakaZu7 = JakaRobot<_JakaZu7, { _JakaZu7::N }>;
pub type JakaZu12 = JakaRobot<_JakaZu12, { _JakaZu12::N }>;
pub type JakaZu18 = JakaRobot<_JakaZu18, { _JakaZu18::N }>;
pub type JakaZu20 = JakaRobot<_JakaZu20, { _JakaZu20::N }>;
pub type JakaZu30 = JakaRobot<_JakaZu30, { _JakaZu30::N }>;

impl ArmParam<{ _JakaZu3::N }> for JakaZu3 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu3::N] = to_radians_array([-90., 0., 148., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([180., 180., 180., 220., 220., 220.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 1.5;
}

impl ArmParam<{ _JakaZu5::N }> for JakaZu5 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu5::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = [PI; Self::N];
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
}

impl ArmParam<{ _JakaZu7::N }> for JakaZu7 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu7::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = [PI; Self::N];
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 2.5;
}

impl ArmParam<{ _JakaZu12::N }> for JakaZu12 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu12::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 120., 180., 180., 180.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
}

impl ArmParam<{ _JakaZu18::N }> for JakaZu18 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu18::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 180., 180., 180., 180.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
}

impl ArmParam<{ _JakaZu20::N }> for JakaZu20 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu20::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 120., 220., 220., 220.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 5.0;
}

impl ArmParam<{ _JakaZu30::N }> for JakaZu30 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaZu30::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; Self::N] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; Self::N] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 120., 220., 220., 220.]);
    // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
    const CARTESIAN_VEL_BOUND: f64 = 4.2;
}

impl ArmForwardKinematics<{ _JakaZu3::N }> for JakaZu3 {
    const DH: [robot_behavior::DhParam; _JakaZu3::N] = [
        DhParam::DH { theta: 0., d: 0.15055, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.246, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.228, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.113, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1175, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.105, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaZu5::N }> for JakaZu5 {
    const DH: [robot_behavior::DhParam; _JakaZu5::N] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaZu7::N }> for JakaZu7 {
    const DH: [robot_behavior::DhParam; _JakaZu7::N] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.360, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3035, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.11501, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaZu12::N }> for JakaZu12 {
    const DH: [robot_behavior::DhParam; _JakaZu12::N] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.595, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.5715, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.1315, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaZu18::N }> for JakaZu18 {
    const DH: [robot_behavior::DhParam; _JakaZu18::N] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.510, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.400, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.154, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaZu20::N }> for JakaZu20 {
    const DH: [robot_behavior::DhParam; _JakaZu20::N] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.897, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.7445, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.18835, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1385, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1205, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaZu30::N }> for JakaZu30 {
    const DH: [robot_behavior::DhParam; _JakaZu30::N] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.6625, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.549, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.18835, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1385, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1205, r: 0., alpha: 0. },
    ];
}

impl RobotFile for JakaZu3 {
    const URDF: &'static str = "jaka/jaka_zu3.urdf";
}

impl RobotFile for JakaZu5 {
    const URDF: &'static str = "jaka/jaka_zu5.urdf";
}

impl RobotFile for JakaZu7 {
    const URDF: &'static str = "jaka/jaka_zu7.urdf";
}

impl RobotFile for JakaZu12 {
    const URDF: &'static str = "jaka/jaka_zu12.urdf";
}

impl RobotFile for JakaZu18 {
    const URDF: &'static str = "jaka/jaka_zu18.urdf";
}

impl RobotFile for JakaZu20 {
    const URDF: &'static str = "jaka/jaka_zu20.urdf";
}

impl RobotFile for JakaZu30 {
    const URDF: &'static str = "jaka/jaka_zu30.urdf";
}
