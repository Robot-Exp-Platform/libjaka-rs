use std::f64::consts::PI;

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    robot.set_scale(0.05);
    robot.move_to::<JointSpace<6>>(JakaMini2::JOINT_DEFAULT)?;

    let start = JakaMini2::JOINT_DEFAULT;
    let traj = (0..800)
        .map(|i| {
            let s = i as f64 / 799.0;
            let mut q = start;
            q[1] += 0.04 * f64::sin(2.0 * PI * s);
            q[2] -= 0.04 * f64::sin(2.0 * PI * s);
            q
        })
        .collect();

    robot.move_traj::<JointSpace<6>>(traj)
}
