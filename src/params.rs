use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

pub const PORT_STATE: u16 = 10000;
pub const PORT_CMD: u16 = 10001;

pub const JAKA_MINI_DOF: usize = 6;
pub const JAKA_VERSION: &str = "0.1.0";

pub const JAKA_FREQUENCY: f64 = 125.0; // Hz

pub const JAKA_ROBOT_DEFAULT_JOINT: [f64; JAKA_MINI_DOF] =
    [0., FRAC_PI_3 * 2., -FRAC_PI_3 * 2., 0., -FRAC_PI_2, 0.];
pub const JAKA_ROBOT_MIN_JOINT: [f64; JAKA_MINI_DOF] = [
    -PI * 2.,
    -FRAC_PI_3 * 2.,
    -FRAC_PI_3 * 2.,
    -PI * 2.,
    -FRAC_PI_3 * 2.,
    -PI * 2.,
];
pub const JAKA_ROBOT_MAX_JOINT: [f64; JAKA_MINI_DOF] = [
    PI * 2.,
    FRAC_PI_3 * 2.,
    FRAC_PI_3 * 2.,
    PI * 2.,
    FRAC_PI_3 * 2.,
    PI * 2.,
];
pub const JAKA_ROBOT_MAX_JOINT_VEL: [f64; JAKA_MINI_DOF] = [PI; JAKA_MINI_DOF];
pub const JAKA_ROBOT_MAX_JOINT_ACC: [f64; JAKA_MINI_DOF] = [PI * 4.; JAKA_MINI_DOF];
pub const JAKA_ROBOT_MAX_CARTESIAN_VEL: f64 = 1000.;
pub const JAKA_ROBOT_MAX_CARTESIAN_ACC: f64 = 4000.;
pub const JAKA_ROBOT_MAX_ROTATION_VEL: f64 = 180.;
pub const JAKA_ROBOT_MAX_ROTATION_ACC: f64 = 400.;
