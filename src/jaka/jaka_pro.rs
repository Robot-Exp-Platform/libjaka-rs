use std::f64::consts::{FRAC_PI_2, PI};

use robot_behavior::{
    ArmForwardKinematics, DhParam, EndPoint, Joints, RobotDescription, to_radians_array,
};

use crate::JakaRobot;

pub struct _JakaPro5;
pub struct _JakaPro7; // Pro7 & Pro18 are also public whose 2D/3D files are however not provided
pub struct _JakaPro12;
pub struct _JakaPro16;
pub struct _JakaPro18;

pub type JakaPro5 = JakaRobot<_JakaPro5, 6>;
pub type JakaPro7 = JakaRobot<_JakaPro7, 6>;
pub type JakaPro12 = JakaRobot<_JakaPro12, 6>;
pub type JakaPro16 = JakaRobot<_JakaPro16, 6>;
pub type JakaPro18 = JakaRobot<_JakaPro18, 6>;

impl RobotDescription for JakaPro5 {
    const URDF: Option<&'static str> = Some("jaka/jaka_pro5.urdf");
}

impl Joints<6> for JakaPro5 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaPro5 {
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaPro5 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaPro7 {
    const URDF: Option<&'static str> = Some("jaka/jaka_pro7.urdf");
}

impl Joints<6> for JakaPro7 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaPro7 {
    const CARTESIAN_VEL_BOUND: f64 = 2.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaPro7 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.360, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3035, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.11501, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaPro12 {
    const URDF: Option<&'static str> = Some("jaka/jaka_pro12.urdf");
}

impl Joints<6> for JakaPro12 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 180., 180., 180.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaPro12 {
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaPro12 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.595, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.5715, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.1315, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaPro16 {
    const URDF: Option<&'static str> = Some("jaka/jaka_pro16.urdf");
}

impl Joints<6> for JakaPro16 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 120., 180., 180., 180.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaPro16 {
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaPro16 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.819, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.687, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.158, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1441, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.12665, r: 0., alpha: 0. },
    ];
}

impl RobotDescription for JakaPro18 {
    const URDF: Option<&'static str> = Some("jaka/jaka_pro18.urdf");
}

impl Joints<6> for JakaPro18 {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
    const JOINT_MIN: [f64; 6] = to_radians_array([-360., -85., -175., -85., -360., -360.]);
    const JOINT_MAX: [f64; 6] = to_radians_array([360., 265., 175., 265., 360., 360.]);
    const JOINT_VEL_BOUND: [f64; 6] = to_radians_array([120., 120., 180., 180., 180., 180.]);
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaPro18 {
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaPro18 {
    const DH: [DhParam; 6] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.510, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.400, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.154, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}
