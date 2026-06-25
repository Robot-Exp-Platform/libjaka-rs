use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/examples/safe_joint_trajectory.json"
    );

    robot.enable()?;
    robot.set_scale(0.05);
    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)?;
    robot.move_traj_from_file::<JointSpace<6>>(path)
}
