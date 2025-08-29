use crate::JakaRobot;
use nalgebra as na;
use robot_behavior::{
    ArmState, Coord, LoadState, MotionType, Pose, RobotResult, behavior::*, c_arm_behavior,
    c_arm_param, c_arm_preplanned_motion, c_arm_preplanned_motion_ext,
    c_arm_preplanned_motion_impl, c_robot_behavior, impl_self,
};

struct CJakaRobot(JakaRobot);

#[cxx::bridge]
mod ffi {
    extern "Rust" {
        type CJakaRobot;

        fn create(ip: &str) -> Box<CJakaRobot>;

        fn version(&self) -> String;
        fn init(&mut self) -> Result<()>;
        fn shutdown(&mut self) -> Result<()>;
        fn enable(&mut self) -> Result<()>;
        fn disable(&mut self) -> Result<()>;
        fn reset(&mut self) -> Result<()>;
        fn is_moving(&mut self) -> bool;
        fn stop(&mut self) -> Result<()>;
        fn pause(&mut self) -> Result<()>;
        fn resume(&mut self) -> Result<()>;
        fn emergency_stop(&mut self) -> Result<()>;
        fn clear_emergency_stop(&mut self) -> Result<()>;

        fn move_joint(&mut self, target: &[f64; 6]) -> Result<()>;
        fn move_joint_async(&mut self, target: &[f64; 6]) -> Result<()>;

        // fn move_cartesian(&mut self, target: &Pose) -> Result<()>;
        // fn move_cartesian_async(&mut self, target: &Pose) -> Result<()>;
    }
}

fn create(ip: &str) -> Box<CJakaRobot> {
    Box::new(CJakaRobot(JakaRobot::new(ip)))
}

impl CJakaRobot {
    c_robot_behavior!(CJakaRobot(JakaRobot));
    c_arm_behavior!(CJakaRobot<{6}>(JakaRobot));
    c_arm_param!(CJakaRobot<{6}>(JakaRobot));
    c_arm_preplanned_motion_impl!(CJakaRobot<{6}>(JakaRobot));
    c_arm_preplanned_motion!(CJakaRobot<{6}>(JakaRobot));
    c_arm_preplanned_motion_ext!(CJakaRobot<{6}>(JakaRobot));
}

// impl CJakaRobot {
//     pub fn version(&self) -> String {
//         self.0.version()
//     }
//     pub fn init(&mut self) -> RobotResult<()> {
//         self.0.init()
//     }
//     pub fn shutdown(&mut self) -> RobotResult<()> {
//         self.0.shutdown()
//     }
//     pub fn enable(&mut self) -> RobotResult<()> {
//         self.0.enable()
//     }
//     pub fn disable(&mut self) -> RobotResult<()> {
//         self.0.disable()
//     }
//     pub fn reset(&mut self) -> RobotResult<()> {
//         self.0.reset()
//     }
//     pub fn is_moving(&mut self) -> bool {
//         self.0.is_moving()
//     }
//     pub fn stop(&mut self) -> RobotResult<()> {
//         self.0.stop()
//     }
//     pub fn pause(&mut self) -> RobotResult<()> {
//         self.0.pause()
//     }
//     pub fn resume(&mut self) -> RobotResult<()> {
//         self.0.resume()
//     }
//     pub fn emergency_stop(&mut self) -> RobotResult<()> {
//         self.0.emergency_stop()
//     }
//     pub fn clear_emergency_stop(&mut self) -> RobotResult<()> {
//         self.0.clear_emergency_stop()
//     }
// }
