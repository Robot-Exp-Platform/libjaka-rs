// use std::{thread::sleep, time::Duration};

use libjaka_rs::JakaRobot;
use robot_behavior::{ArmPreplannedMotionExt, RobotBehavior, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = JakaRobot::new("10.5.5.100");
    robot.enable()?;
    robot._stop_program()?;
    // robot.move_joint_async(&[0.0; 6], 100.0)?;
    robot.move_joint_async(&[-90., -30., -90., 0., -30., 0.], 100.0)?;
    // robot.move_cartesian(&Pose::Euler([200.0, 300.0, 400.0], [0.0, 180.0, 0.0]), 50.0)?;
    // let move_l_data = MoveLData {
    //     cart_position: [400.0, 0.0, 300.0, 0.0, 0.0, 0.0],
    //     accel: 100.0,
    //     speed: 20.0,
    //     relflag: 0,
    // };
    // robot._move_l(move_l_data)?;
    // sleep(Duration::from_secs(5));
    // robot.disable()?;
    Ok(())
}
