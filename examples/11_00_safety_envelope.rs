use libjaka::{JakaMini2, types::SetClsnSensitivityData};
use robot_behavior::{LoadState, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_scale(0.05);
    robot.set_load(LoadState { m: 0.0, x: [0.0; 3], i: [0.0; 9] })?;
    let result: RobotResult<()> = robot
        .robot_impl
        ._set_clsn_sensitivity(SetClsnSensitivityData { sensitivity_level: 3 })?
        .into();
    result?;

    println!("scale configured to 0.05");
    println!("collision sensitivity configured to level 3");
    Ok(())
}
