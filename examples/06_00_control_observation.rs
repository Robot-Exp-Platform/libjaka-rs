use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");
    let samples = Arc::new(Mutex::new((0usize, 0usize)));

    let before_samples = samples.clone();
    robot.before(move |state, _| {
        let mut samples = before_samples.lock().unwrap();
        samples.0 += 1;
        if state.emergency_stop[0] {
            println!("emergency stop is active");
        }
    });

    let after_samples = samples.clone();
    robot.after(move |_, _| {
        after_samples.lock().unwrap().1 += 1;
    });

    robot.enable()?;
    let mut elapsed = Duration::ZERO;
    let mut target = None;
    robot.control_with::<JointPositionControl<6>, _>(move |state, dt| {
        let target = *target.get_or_insert_with(|| state.meas.q.unwrap_or([0.0; 6]));
        elapsed += dt;
        (target, elapsed >= Duration::from_secs(2))
    })?;

    let (before, after) = *samples.lock().unwrap();
    println!("before samples: {before}");
    println!("after samples: {after}");
    Ok(())
}
