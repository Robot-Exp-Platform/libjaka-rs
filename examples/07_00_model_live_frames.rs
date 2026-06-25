#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use libjaka::JakaMini2;
use robot_behavior::{Link, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");
    let state = robot.state()?;
    let q = state.joint.meas.q.unwrap_or(JakaMini2::JOINT_DEFAULT);
    let cache = <JakaMini2 as ArmForwardKinematics<6>>::kine_cache(&q, &[0.0; 6]);

    for link in 0..=6 {
        println!("link {link}: {:?}", cache.link_pose(Link(link)));
    }
    println!("end effector: {:?}", cache.end_effector_pose());
    Ok(())
}
