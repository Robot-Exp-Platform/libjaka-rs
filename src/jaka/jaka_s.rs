use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmForwardKinematics, DhParam, EndPoint, Joints, RobotDescription, to_radians_array,
};

use crate::JakaRobot;

pub struct _JakaS5;
pub struct _JakaS7;
pub struct _JakaS12;

pub type JakaS5 = JakaRobot<_JakaS5, 6>;
pub type JakaS7 = JakaRobot<_JakaS7, 6>;
pub type JakaS12 = JakaRobot<_JakaS12, 6>;

impl RobotDescription for JakaS5 {
    const URDF: Option<&'static str> = Some("jaka/jaka_s5.urdf");
}

impl Joints<6> for JakaS5 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaS5 {
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaS5 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1175, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaS7 {
    const URDF: Option<&'static str> = Some("jaka/jaka_s7.urdf");
}

impl Joints<6> for JakaS7 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaS7 {
    const CARTESIAN_VEL_BOUND: f64 = 2.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaS7 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.360, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3035, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.11501, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1175, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaS12 {
    const URDF: Option<&'static str> = Some("jaka/jaka_s12.urdf");
}

impl Joints<6> for JakaS12 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 180., 180., 180.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaS12 {
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaS12 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.595, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.5715, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.1315, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.112, r: 0., alpha: 0. },
    ];
}
