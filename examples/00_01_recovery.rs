use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    if robot.is_moving()? {
        robot.stop()?;
    }
    robot.clear_emergency_stop()
}
