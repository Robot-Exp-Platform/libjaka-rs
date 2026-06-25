use std::{f64::consts::PI, time::Duration};

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");

    robot.enable()?;
    let mut elapsed = Duration::ZERO;
    let mut initial = None;
    robot.control_with::<CartesianPoseControl<6>, _>(move |state, dt| {
        let initial = *initial.get_or_insert_with(|| state.flange.meas.pose.unwrap_or_default());
        elapsed += dt;

        let offset = 0.02 * f64::sin(2.0 * PI * elapsed.as_secs_f64() / 4.0);
        let mut pose = initial.homo();
        pose[12] += offset;

        (Pose::Homo(pose), elapsed >= Duration::from_secs(4))
    })
}
