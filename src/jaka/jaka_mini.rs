use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

use roplat_behavior::{
    CartesianLimits, EndPoint, EndPointPose, JointLimits, Joints, Pose, RobotDescription,
    RobotResult,
};
use roplat_kinematics::{ArmForwardKinematics, DhParam};

use crate::JakaRobot;

pub struct _JakaMini2;
pub type JakaMini2 = JakaRobot<_JakaMini2, 6>;

impl RobotDescription for JakaMini2 {
    const URDF: Option<&'static str> = Some("jaka/jaka_minicobo.urdf");
}

impl Joints<6> for JakaMini2 {
    const JOINT_DEFAULT: [f64; 6] = [0., FRAC_PI_3 * 2., -FRAC_PI_3 * 2., 0., -FRAC_PI_2, 0.];
    const JOINT_PACKED: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_LIMITS: JointLimits<6> = JointLimits {
        dq_max: [PI; 6],
        ddq_max: [PI * 4.; 6],
        ..JointLimits::positions(
            [
                -PI * 2.,
                -FRAC_PI_3 * 2.,
                -FRAC_PI_3 * 2.,
                -PI * 2.,
                -FRAC_PI_3 * 2.,
                -PI * 2.,
            ],
            [
                PI * 2.,
                FRAC_PI_3 * 2.,
                FRAC_PI_3 * 2.,
                PI * 2.,
                FRAC_PI_3 * 2.,
                PI * 2.,
            ],
        )
    };
}

impl EndPoint for JakaMini2 {
    const CARTESIAN_LIMITS: CartesianLimits = CartesianLimits {
        linear_velocity_max: 1.,
        linear_acceleration_max: 8.0,
        angular_velocity_max: PI,
        angular_acceleration_max: PI * 4.,
        ..CartesianLimits::UNBOUNDED
    };
}

impl EndPointPose for JakaMini2 {
    fn get_endpoint(&self) -> Pose {
        self.flange_pose()
    }
    fn read_endpoint(&mut self) -> RobotResult<Pose> {
        self.read_flange_pose()
    }
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
roplat_kinematics::dh_kinematics!(JakaMini2, 6);

#[cfg(test)]
mod tests {
    use roplat_kinematics::ArmForwardKinematics;

    use super::*;

    #[test]
    fn test_kine() {
        let cache = JakaMini2::kine_cache(&[0.; 6], &[0.; 6]);

        let pose = cache.end_effector_pose();

        println!("pose: {:?}", pose);
    }
}
