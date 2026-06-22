use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmForwardKinematics, DhParam, EndPoint, Joints, RobotDescription, to_radians_array,
};

use crate::JakaRobot;

pub struct _JakaA5;
pub struct _JakaA12;
pub struct _JakaA20;

pub type JakaA5 = JakaRobot<_JakaA5, 6>;
pub type JakaA12 = JakaRobot<_JakaA12, 6>;
pub type JakaA20 = JakaRobot<_JakaA20, 6>;

impl RobotDescription for JakaA5 {
    const URDF: Option<&'static str> = Some("jaka/jaka_a5.urdf");
}

impl Joints<6> for JakaA5 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = [-PI * 2.; 6];
    const JOINT_MAX: [f64; 6] = [PI * 2.; 6];
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([210., 210., 210., 265., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaA5 {
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaA5 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaA12 {
    const URDF: Option<&'static str> = Some("jaka/jaka_a12.urdf");
}

impl Joints<6> for JakaA12 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([0., 90., 180., -180., 180., 90.]);
    const JOINT_MIN: [f64; 6] = [-PI * 2.; 6];
    const JOINT_MAX: [f64; 6] = [PI * 2.; 6];
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([150., 150., 210., 210., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaA12 {
    const CARTESIAN_VEL_BOUND: f64 = 4.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaA12 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.770, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.03750, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.5515, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: -FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaA20 {
    const URDF: Option<&'static str> = Some("jaka/jaka_a20.urdf");
}

impl Joints<6> for JakaA20 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 265., 265., 265.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaA20 {
    const CARTESIAN_VEL_BOUND: f64 = 5.;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaA20 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.897, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.7445, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.18835, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1385, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1205, r: 0., alpha: 0. },
    ];
}
