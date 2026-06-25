use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_scale(0.05);
    robot.set_coord(Coord::Relative);
    robot.move_to::<FlangeSpace>(Pose::from([0.02, 0.0, 0.0]))?;
    robot.move_to::<FlangeSpace>(Pose::from([-0.02, 0.0, 0.0]))?;
    robot.set_coord(Coord::OCS);
    Ok(())
}
