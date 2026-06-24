use std::f64::consts::PI;

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100").with_cartesian_vel(1.0);
    robot.enable()?;
    robot.stop()?;

    // 以受限的笛卡尔速度移动到法兰目标位姿（平移单位：米，姿态单位：弧度）。
    robot.move_to::<FlangeSpace>(Pose::Euler([0.3, 0.0, 0.03], [PI, 0.0, PI]))?;

    Ok(())
}
