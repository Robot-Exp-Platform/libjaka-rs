use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

use robot_behavior::{ArmForwardKinematics, DhParam, EndPoint, Joints, RobotDescription};

use crate::JakaRobot;

pub struct _JakaMini2;
pub type JakaMini2 = JakaRobot<_JakaMini2, 6>;

impl RobotDescription for JakaMini2 {
    const URDF: Option<&'static str> = Some("jaka/jaka_minicobo.urdf");
}

impl Joints<6> for JakaMini2 {
    const JOINT_DEFAULT: [f64; 6] = [0., FRAC_PI_3 * 2., -FRAC_PI_3 * 2., 0., -FRAC_PI_2, 0.];
    const JOINT_PACKED: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_MIN: [f64; 6] = [
        -PI * 2.,
        -FRAC_PI_3 * 2.,
        -FRAC_PI_3 * 2.,
        -PI * 2.,
        -FRAC_PI_3 * 2.,
        -PI * 2.,
    ];
    const JOINT_MAX: [f64; 6] = [
        PI * 2.,
        FRAC_PI_3 * 2.,
        FRAC_PI_3 * 2.,
        PI * 2.,
        FRAC_PI_3 * 2.,
        PI * 2.,
    ];
    const JOINT_VEL_BOUND: [f64; 6] = [PI; 6];
    const JOINT_ACC_BOUND: [f64; 6] = [PI * 4.; 6];
}

impl EndPoint for JakaMini2 {
    const CARTESIAN_VEL_BOUND: f64 = 1.;
    const CARTESIAN_ACC_BOUND: f64 = 8.0;
    const ROTATION_VEL_BOUND: f64 = PI;
    const ROTATION_ACC_BOUND: f64 = PI * 4.;
}

impl ArmForwardKinematics<6> for JakaMini2 {
    const DH: [DhParam; 6] = [
        DhParam::Iso3RPY { pos: [0., 0., 0.187], rpy: [0.; 3] },
        DhParam::Iso3RPY { pos: [0., -0.006, 0.], rpy: [FRAC_PI_2, -FRAC_PI_2, 0.] },
        DhParam::Iso3RPY { pos: [0.21, 0., 0.], rpy: [0., 0., -FRAC_PI_2] },
        DhParam::Iso3RPY { pos: [0., 0.2105, 0.], rpy: [-FRAC_PI_2, 0., 0.] },
        DhParam::Iso3RPY { pos: [0., 0., 0.], rpy: [FRAC_PI_2, 0., 0.] },
        DhParam::Iso3RPY { pos: [0., 0.1593, 0.], rpy: [-FRAC_PI_2, 0., 0.] },
    ];
}

#[cfg(test)]
mod tests {
    use robot_behavior::ArmForwardKinematics;

    use super::*;

    #[test]
    fn test_kine() {
        let cache = JakaMini2::kine_cache(&[0.; 6], &[0.; 6]);

        let pose = cache.end_effector_pose();

        println!("pose: {:?}", pose);
    }
}
