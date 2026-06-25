use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_scale(0.05);
    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)?;

    let mut mid = JakaMini2::JOINT_DEFAULT;
    mid[1] += 0.05;
    mid[2] -= 0.05;

    robot.move_waypoints::<JointSpace<6>>(vec![
        JakaMini2::JOINT_DEFAULT,
        mid,
        JakaMini2::JOINT_DEFAULT,
    ])
}
