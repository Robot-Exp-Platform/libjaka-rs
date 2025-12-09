use crate::{JakaRobot, JakaType};

pub struct _JakaPro5;
// pub struct _JakaPro7;  // Pro7 & Pro18 are also public whose 2D/3D files are however not provided
pub struct _JakaPro12;
pub struct _JakaPro16;
// pub struct _JakaPro18;

impl JakaType for _JakaPro5 {
    const N: usize = 6;
}

// impl JakaType for _JakaPro7 {
//     const N: usize = 6;
// }

impl JakaType for _JakaPro12 {
    const N: usize = 6;
}

impl JakaType for _JakaPro16 {
    const N: usize = 6;
}

// impl JakaType for _JakaPro18 {
//     const N: usize = 6;
// }

pub type JakaPro5 = JakaRobot<_JakaPro5, { _JakaPro5::N }>;
// pub type JakaPro7 = JakaRobot<_JakaPro7, { _JakaPro7::N }>;
pub type JakaPro12 = JakaRobot<_JakaPro12, { _JakaPro12::N }>;
pub type JakaPro16 = JakaRobot<_JakaPro16, { _JakaPro16::N }>;
// pub type JakaPro18 = JakaRobot<_JakaPro18, { _JakaPro18::N }>;

impl ArmParam<{ _JakaPro5::N }> for JakaPro5 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaPro5::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
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
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
}

// impl ArmParam<{ _JakaPro7::N }> for JakaPro7 {
//     const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
//     const JOINT_PACKED: [f64; _JakaPro7::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
//     const JOINT_MIN: [f64; Self::N] = [
//         -PI * 2.,
//         -85. * PI / 180.,
//         -175. * PI / 180.,
//         -85. * PI / 180.,
//         -PI * 2.,
//         -PI * 2.,
//     ];
//     const JOINT_MAX: [f64; Self::N] = [
//         PI * 2.,
//         265. * PI / 180.,
//         175. * PI / 180.,
//         265. * PI / 180.,
//         PI * 2.,
//         PI * 2.,
//     ];
//     const JOINT_VEL_BOUND: [f64; Self::N] = [PI; Self::N];
//     // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
//     const CARTESIAN_VEL_BOUND: f64 = 2.5;
// }

impl ArmParam<{ _JakaPro12::N }> for JakaPro12 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaPro12::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
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

impl ArmParam<{ _JakaPro16::N }> for JakaPro16 {
    const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; _JakaPro16::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
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
    const CARTESIAN_VEL_BOUND: f64 = 3.5;
}

// impl ArmParam<{ _JakaPro18::N }> for JakaPro18 {
//     const JOINT_DEFAULT: [f64; Self::N] = [0., 0., 0., 0., 0., 0.];
//     const JOINT_PACKED: [f64; _JakaPro18::N] = to_radians_array([-90., 0., 152., 120., 0., 0.]);
//     const JOINT_MIN: [f64; Self::N] = [
//         -PI * 2.,
//         -85. * PI / 180.,
//         -175. * PI / 180.,
//         -85. * PI / 180.,
//         -PI * 2.,
//         -PI * 2.,
//     ];
//     const JOINT_MAX: [f64; Self::N] = [
//         PI * 2.,
//         265. * PI / 180.,
//         175. * PI / 180.,
//         265. * PI / 180.,
//         PI * 2.,
//         PI * 2.,
//     ];
//     const JOINT_VEL_BOUND: [f64; Self::N] = to_radians_array([120., 120., 180., 180., 180., 180.]);
//     // const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];  // not provided
//     const CARTESIAN_VEL_BOUND: f64 = 3.5;
// }

impl ArmForwardKinematics<{ _JakaPro5::N }> for JakaPro5 {
    const DH: [robot_behavior::DhParam; _JakaPro5::N] = [
        DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.430, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.3685, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.114, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
    ];
}

// impl ArmForwardKinematics<{ _JakaPro7::N }> for JakaPro7 {
//     const DH: [robot_behavior::DhParam; _JakaPro7::N] = [
//         DhParam::DH { theta: 0., d: 0.12015, r: 0., alpha: FRAC_PI_2 },
//         DhParam::DH { theta: 0., d: 0., r: 0.360, alpha: 0. },
//         DhParam::DH { theta: 0., d: 0., r: 0.3035, alpha: 0. },
//         DhParam::DH { theta: 0., d: 0.11501, r: 0., alpha: FRAC_PI_2 },
//         DhParam::DH { theta: 0., d: 0.1135, r: 0., alpha: FRAC_PI_2 },
//         DhParam::DH { theta: 0., d: 0.107, r: 0., alpha: 0. },
//     ];
// }

impl ArmForwardKinematics<{ _JakaPro12::N }> for JakaPro12 {
    const DH: [robot_behavior::DhParam; _JakaPro12::N] = [
        DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.595, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.5715, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.1315, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
    ];
}

impl ArmForwardKinematics<{ _JakaPro16::N }> for JakaPro16 {
    const DH: [robot_behavior::DhParam; _JakaPro16::N] = [
        DhParam::DH { theta: 0., d: 0.1965, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0., r: 0.819, alpha: 0. },
        DhParam::DH { theta: 0., d: 0., r: 0.687, alpha: 0. },
        DhParam::DH { theta: 0., d: 0.158, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.1441, r: 0., alpha: FRAC_PI_2 },
        DhParam::DH { theta: 0., d: 0.12665, r: 0., alpha: 0. },
    ];
}

// impl ArmForwardKinematics<{ _JakaPro18::N }> for JakaPro18 {
//     const DH: [robot_behavior::DhParam; _JakaPro18::N] = [
//         DhParam::DH { theta: 0., d: 0.14265, r: 0., alpha: FRAC_PI_2 },
//         DhParam::DH { theta: 0., d: 0., r: 0.510, alpha: 0. },
//         DhParam::DH { theta: 0., d: 0., r: 0.400, alpha: 0. },
//         DhParam::DH { theta: 0., d: 0.154, r: 0., alpha: FRAC_PI_2 },
//         DhParam::DH { theta: 0., d: 0.115, r: 0., alpha: FRAC_PI_2 },
//         DhParam::DH { theta: 0., d: 0.1035, r: 0., alpha: 0. },
//     ];
// }

impl RobotFile for _JakaPro5 {
    const URDF: &'static str = "jaka/jaka_pro5.urdf";
}

impl RobotFile for _JakaPro12 {
    const URDF: &'static str = "jaka/jaka_pro12.urdf";
}

impl RobotFile for _JakaPro16 {
    const URDF: &'static str = "jaka/jaka_pro16.urdf";
}
