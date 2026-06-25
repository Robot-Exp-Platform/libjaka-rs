use libjaka::{
    JakaMini2,
    types::{TioVout, TioVoutMode},
};
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    println!("initial TIO VOUT: {:?}", robot.get_tio_vout()?);
    robot.set_tio_vout(TioVout::Enable(TioVoutMode::V24V))?;
    println!("enabled TIO VOUT: {:?}", robot.get_tio_vout()?);
    robot.set_tio_vout(TioVout::Disable)?;
    println!("disabled TIO VOUT: {:?}", robot.get_tio_vout()?);
    Ok(())
}
