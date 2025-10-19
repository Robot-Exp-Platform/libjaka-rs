use robot_behavior::{ArmParam, Coord, OverrideOnce, behavior::*};
use std::{
    f64::consts::{FRAC_PI_2, FRAC_PI_3, PI},
    marker::PhantomData,
    thread,
};

use crate::{JakaRobot, JakaType, network::NetWork};

pub struct _JakaMini2;
pub type JakaMini2 = JakaRobot<_JakaMini2, { _JakaMini2::N }>;

impl JakaType for _JakaMini2 {
    const N: usize = 6;
}

impl JakaMini2 {
    /// Create a new JakaRobot instance with the given IP address.
    ///
    /// # Arguments
    /// * `ip` - A string slice that holds the IP address of the robot.
    pub fn new(ip: &str) -> Self {
        let network = NetWork::new(ip);
        let robot_state = NetWork::state_connect(ip);
        let mut robot = Self {
            marker: PhantomData,
            network,
            robot_state,
            streaming_handle: thread::spawn(|| {}),
            is_moving: false,
            coord: OverrideOnce::new(Coord::OCS),
            max_vel: OverrideOnce::new(Self::JOINT_VEL_BOUND),
            max_acc: OverrideOnce::new(Self::JOINT_ACC_BOUND),
            max_cartesian_vel: OverrideOnce::new(Self::CARTESIAN_VEL_BOUND),
            max_cartesian_acc: OverrideOnce::new(Self::CARTESIAN_ACC_BOUND),
            max_rotation_vel: OverrideOnce::new(Self::ROTATION_VEL_BOUND),
            max_rotation_acc: OverrideOnce::new(Self::ROTATION_ACC_BOUND),
        };
        let _ = robot.set_speed(0.05);
        robot
    }
}

impl ArmParam<{ _JakaMini2::N }> for _JakaMini2 {
    const DH: [[f64; 4]; Self::N] = [[0.; 4]; Self::N];
    const JOINT_DEFAULT: [f64; Self::N] = [0., FRAC_PI_3 * 2., -FRAC_PI_3 * 2., 0., -FRAC_PI_2, 0.];
    const JOINT_MIN: [f64; Self::N] = [
        -PI * 2.,
        -FRAC_PI_3 * 2.,
        -FRAC_PI_3 * 2.,
        -PI * 2.,
        -FRAC_PI_3 * 2.,
        -PI * 2.,
    ];
    const JOINT_MAX: [f64; Self::N] = [
        PI * 2.,
        FRAC_PI_3 * 2.,
        FRAC_PI_3 * 2.,
        PI * 2.,
        FRAC_PI_3 * 2.,
        PI * 2.,
    ];
    const JOINT_VEL_BOUND: [f64; Self::N] = [PI; Self::N];
    const JOINT_ACC_BOUND: [f64; Self::N] = [PI * 4.; Self::N];
    const JOINT_JERK_BOUND: [f64; Self::N] = [f64::MAX; Self::N];
    const CARTESIAN_VEL_BOUND: f64 = 1000.;
    const CARTESIAN_ACC_BOUND: f64 = 4000.;
    const ROTATION_VEL_BOUND: f64 = 180.;
    const ROTATION_ACC_BOUND: f64 = 400.;
    const TORQUE_BOUND: [f64; Self::N] = [f64::MAX; Self::N];
    const TORQUE_DOT_BOUND: [f64; Self::N] = [f64::MAX; Self::N];
}

impl RobotFile for JakaMini2 {
    const URDF: &'static str = "jaka/jaka_minicobo.urdf";
}
