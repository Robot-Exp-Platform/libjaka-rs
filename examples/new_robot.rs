use libjaka_rs::JakaRobot;
use robot_behavior::{RobotBehavior, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = JakaRobot::new("10.5.5.100");
    robot.enable()?;
    robot.disable()?;
    Ok(())
}
