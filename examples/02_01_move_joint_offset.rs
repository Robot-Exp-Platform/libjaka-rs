use libjaka::JakaMini2;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_motion_limits(robot.motion_limits().scaled(0.05))?;
    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)?;

    robot.move_to::<Relative<JointSpace<6>>>([0.0, 0.04, -0.04, 0.0, 0.0, 0.0])?;

    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)
}
