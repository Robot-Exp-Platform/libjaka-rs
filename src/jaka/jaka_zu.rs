use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmForwardKinematics, DhParam, EndPoint, Joints, RobotDescription, to_radians_array,
};

use crate::JakaRobot;

pub struct _JakaZu3;
pub struct _JakaZu5;
pub struct _JakaZu7;
pub struct _JakaZu12;
pub struct _JakaZu18;
pub struct _JakaZu20;
pub struct _JakaZu30;

pub type JakaZu3 = JakaRobot<_JakaZu3, 6>;
pub type JakaZu5 = JakaRobot<_JakaZu5, 6>;
pub type JakaZu7 = JakaRobot<_JakaZu7, 6>;
pub type JakaZu12 = JakaRobot<_JakaZu12, 6>;
pub type JakaZu18 = JakaRobot<_JakaZu18, 6>;
pub type JakaZu20 = JakaRobot<_JakaZu20, 6>;
pub type JakaZu30 = JakaRobot<_JakaZu30, 6>;

impl RobotDescription for JakaZu3 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu3.urdf");
}

impl Joints<6> for JakaZu3 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 148., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([180., 180., 180., 220., 220., 220.]);
    const JOINT_ACC_BOUND: [f64; 6] = [f64::MAX; 6]; // not provided
}

impl EndPoint for JakaZu3 {
    const CARTESIAN_VEL_BOUND: f64 = 1.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu3 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.15055, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.246, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.228, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.113, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1175, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.105, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaZu5 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu5.urdf");
}

impl Joints<6> for JakaZu5 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaZu5 {
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu5 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaZu7 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu7.urdf");
}

impl Joints<6> for JakaZu7 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaZu7 {
    const CARTESIAN_VEL_BOUND: f64 = 2.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu7 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.360, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3035, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.11501, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaZu12 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu12.urdf");
}

impl Joints<6> for JakaZu12 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 180., 180., 180.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaZu12 {
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu12 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.595, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.5715, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.1315, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaZu18 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu18.urdf");
}

impl Joints<6> for JakaZu18 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 180., 180., 180., 180.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaZu18 {
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu18 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.510, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.400, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.154, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaZu20 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu20.urdf");
}

impl Joints<6> for JakaZu20 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 220., 220., 220.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaZu20 {
    const CARTESIAN_VEL_BOUND: f64 = 5.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu20 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.897, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.7445, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.18835, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1385, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1205, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaZu30 {
    const URDF: Option<&'static str> = Some("jaka/jaka_zu30.urdf");
}

impl Joints<6> for JakaZu30 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 220., 220., 220.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaZu30 {
    const CARTESIAN_VEL_BOUND: f64 = 4.2;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaZu30 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.6625, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.549, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.18835, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1385, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1205, r: 0., alpha: 0. },
    ];
}
