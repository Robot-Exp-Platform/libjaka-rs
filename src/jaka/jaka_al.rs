use std::f64::consts::{FRAC_PI_2, PI};

use roplat_behavior::{
    CartesianLimits, EndPoint, EndPointPose, JointLimits, Joints, Pose, RobotResult,
    to_radians_array,
};
use roplat_kinematics::{ArmForwardKinematics, DhParam};

use crate::JakaRobot;

pub struct _JakaA12L;

pub type JakaA12L = JakaRobot<_JakaA12L, 6>;

// No URDF is published on www.jaka.com, so RobotDescription is not implemented and URDF stays None.
impl Joints<6> for JakaA12L {
    const JOINT_DEFAULT: [f64; 6] = [0., 0., 0., 0., 0., 0.];
    const JOINT_PACKED: [f64; 6] = to_radians_array([90., 180., -180., 180., 90., 90.]);
    const JOINT_LIMITS: JointLimits<6> = JointLimits {
        dq_max: to_radians_array([150., 150., 210., 210., 265., 265.]),
        ddq_max: [PI * 4.; 6],
        ..JointLimits::positions([-PI * 2.; 6], [PI * 2.; 6])
    };
}

impl EndPoint for JakaA12L {
    const CARTESIAN_LIMITS: CartesianLimits = CartesianLimits {
        linear_velocity_max: 4.0,
        linear_acceleration_max: 8.0,
        angular_velocity_max: PI,
        angular_acceleration_max: PI * 4.,
        ..CartesianLimits::UNBOUNDED
    };
}

impl EndPointPose for JakaA12L {
    fn get_endpoint(&self) -> Pose {
        self.flange_pose()
    }
    fn read_endpoint(&mut self) -> RobotResult<Pose> {
        self.read_flange_pose()
    }
}

impl ArmForwardKinematics<6> for JakaA12L {
    const DH: [DhParam; 6] = [
        DhParam::DH {
            theta: 0.,
            d: 0.14165,
            r: 0.,
            alpha: FRAC_PI_2,
        },
        DhParam::DH {
            theta: 0.,
            d: 0.,
            r: 0.770,
            alpha: 0.,
        },
        DhParam::DH {
            theta: 0.,
            d: 0.03750,
            r: 0.,
            alpha: -FRAC_PI_2,
        },
        DhParam::DH {
            theta: 0.,
            d: 0.5515,
            r: 0.,
            alpha: FRAC_PI_2,
        },
        DhParam::DH {
            theta: 0.,
            d: 0.115,
            r: 0.,
            alpha: -FRAC_PI_2,
        },
        DhParam::DH {
            theta: 0.,
            d: 0.1338,
            r: 0.,
            alpha: 0.,
        },
    ];
}
roplat_kinematics::dh_kinematics!(JakaA12L, 6);
