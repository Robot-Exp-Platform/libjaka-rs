use std::{thread::sleep, time::Duration};

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    for _ in 0..50 {
        let state = robot.read_state()?;
        println!(
            "joint={:?}, tcp={:?}, enabled={:?}, emergency_stop={:?}",
            &state.joint_actual_position[..6],
            &state.actual_position[..6],
            state.enabled,
            state.emergency_stop
        );
        sleep(Duration::from_millis(20));
    }
    Ok(())
}
