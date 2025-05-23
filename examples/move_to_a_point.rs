// use std::{thread::sleep, time::Duration};

use libjaka_rs::JakaRobot;
use robot_behavior::{ArmPreplannedMotionExt, Pose, RobotBehavior, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = JakaRobot::new("10.5.5.100");
    robot.enable()?;
    robot._stop_program()?;
    robot.move_cartesian_async(
        &Pose::Euler([-300.0, 0.0, 60.0], [180.0, 0.0, 180.0]),
        100.0,
    )?; // 接触纸面
    // robot.move_cartesian_async(&Pose::Euler([-320.0, 0.0, 60.0], [180.0, 0.0, 0.0]), 100.0)?; // 离开纸面
    // robot.move_cartesian_async(&Pose::Euler([-350.0, 0.0, 50.0], [180.0, 0.0, 0.0]), 100.0)?;

    Ok(())
}
