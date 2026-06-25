use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_scale(0.05);
    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)?;

    robot.set_coord(Coord::Relative);
    robot.move_to::<JointSpace<6>>([0.0, 0.04, -0.04, 0.0, 0.0, 0.0])?;

    robot.set_coord(Coord::OCS);
    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)
}
