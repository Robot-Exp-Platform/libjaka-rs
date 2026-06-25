use std::f64::consts::PI;

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_scale(0.05);
    robot.move_to::<FlangeSpace>(Pose::Euler([0.3, 0.0, 0.03], [PI, 0.0, PI]))
}
