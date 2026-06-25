use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    println!("{}", JakaMini2::version());
    robot.init()?;
    robot.enable()?;
    robot.disable()?;
    robot.shutdown()
}
