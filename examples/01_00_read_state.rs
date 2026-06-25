use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");
    let state = robot.read_state()?;

    println!("joint: {:?}", &state.joint_actual_position[..6]);
    println!("tcp: {:?}", &state.actual_position[..6]);
    println!("task_state: {:?}", state.task_state);
    println!("task_mode: {:?}", state.task_mode);
    println!("interp_mode: {:?}", state.interp_mode);
    println!("enabled: {:?}", state.enabled);
    println!("protective_stop: {:?}", state.protective_stop);
    println!("emergency_stop: {:?}", state.emergency_stop);
    Ok(())
}
