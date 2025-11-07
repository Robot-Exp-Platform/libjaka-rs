use robot_behavior::{
    ControlType, CxxArmState, LoadState, MotionType, behavior::*, cxx_arm_behavior, cxx_arm_param,
    cxx_arm_preplanned_motion, cxx_arm_preplanned_motion_ext, cxx_arm_preplanned_motion_impl,
    cxx_robot_behavior,
};

struct JakaMini2(crate::JakaMini2);

#[cxx::bridge]
mod libjaka {
    pub struct CxxMotionType {
        pub mode: CxxMotionTypeMode,
        pub values: Vec<f64>,
    }
    pub enum CxxMotionTypeMode {
        Joint,
        JointVel,
        Cartesian,
        CartesianVel,
        Position,
        PositionVel,
        Stop,
    }
    pub struct CxxControlType {
        pub mode: CxxControlTypeMode,
        pub values: Vec<f64>,
    }
    pub enum CxxControlTypeMode {
        Zero,
        Torque,
    }
    pub struct CxxLoadState {
        pub m: f64,
        pub x: [f64; 3],
        pub i: [f64; 9],
    }
    extern "Rust" {
        type JakaMini2;

        #[Self = "JakaMini2"]
        fn attach(ip: &str) -> Box<JakaMini2>;

        #[Self = "JakaMini2"]
        fn version() -> String;
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

pub use libjaka::*;

impl JakaMini2 {
    pub fn attach(ip: &str) -> Box<Self> {
        Box::new(JakaMini2(crate::JakaMini2::new(ip)))
    }
}

impl JakaMini2 {
    cxx_robot_behavior!(JakaMini2(crate::JakaMini2));
    cxx_arm_behavior!(JakaMini2<{6}>(crate::JakaMini2));
    cxx_arm_param!(JakaMini2<{6}>(crate::JakaMini2));
    cxx_arm_preplanned_motion_impl!(JakaMini2<{6}>(crate::JakaMini2));
    cxx_arm_preplanned_motion!(JakaMini2<{6}>(crate::JakaMini2));
    cxx_arm_preplanned_motion_ext!(JakaMini2<{6}>(crate::JakaMini2));
}

impl<const N: usize> From<CxxMotionType> for MotionType<N> {
    fn from(cxx: CxxMotionType) -> Self {
        match cxx.mode {
            CxxMotionTypeMode::Joint => MotionType::Joint(cxx.values.try_into().unwrap()),
            CxxMotionTypeMode::JointVel => MotionType::JointVel(cxx.values.try_into().unwrap()),
            CxxMotionTypeMode::Cartesian => MotionType::Cartesian(cxx.values.into()),
            CxxMotionTypeMode::CartesianVel => {
                MotionType::CartesianVel(cxx.values.try_into().unwrap())
            }
            CxxMotionTypeMode::Position => MotionType::Position(cxx.values.try_into().unwrap()),
            CxxMotionTypeMode::PositionVel => {
                MotionType::PositionVel(cxx.values.try_into().unwrap())
            }
            CxxMotionTypeMode::Stop => MotionType::Stop,
            _ => panic!("Invalid mode for MotionType"),
        }
    }
}

impl<const N: usize> From<MotionType<N>> for CxxMotionType {
    fn from(motion: MotionType<N>) -> Self {
        match motion {
            MotionType::Joint(v) => {
                CxxMotionType { mode: CxxMotionTypeMode::Joint, values: v.to_vec() }
            }
            MotionType::JointVel(v) => {
                CxxMotionType { mode: CxxMotionTypeMode::JointVel, values: v.to_vec() }
            }
            MotionType::Cartesian(v) => {
                CxxMotionType { mode: CxxMotionTypeMode::Cartesian, values: v.into() }
            }
            MotionType::CartesianVel(v) => {
                CxxMotionType { mode: CxxMotionTypeMode::CartesianVel, values: v.to_vec() }
            }
            MotionType::Position(v) => {
                CxxMotionType { mode: CxxMotionTypeMode::Position, values: v.to_vec() }
            }
            MotionType::PositionVel(v) => {
                CxxMotionType { mode: CxxMotionTypeMode::PositionVel, values: v.to_vec() }
            }
            MotionType::Stop => CxxMotionType {
                mode: CxxMotionTypeMode::Position, // Use Position as a placeholder for Stop
                values: vec![],
            },
        }
    }
}

impl<const N: usize> From<CxxControlType> for ControlType<N> {
    fn from(cxx: CxxControlType) -> Self {
        match cxx.mode {
            CxxControlTypeMode::Zero => ControlType::Zero,
            CxxControlTypeMode::Torque => ControlType::Torque(cxx.values.try_into().unwrap()),
            _ => panic!("Invalid mode for ControlType"),
        }
    }
}

impl<const N: usize> From<ControlType<N>> for CxxControlType {
    fn from(control: ControlType<N>) -> Self {
        match control {
            ControlType::Zero => CxxControlType { mode: CxxControlTypeMode::Zero, values: vec![] },
            ControlType::Torque(v) => {
                CxxControlType { mode: CxxControlTypeMode::Torque, values: v.to_vec() }
            }
        }
    }
}

impl From<CxxLoadState> for LoadState {
    fn from(cxx: CxxLoadState) -> Self {
        LoadState { m: cxx.m, x: cxx.x, i: cxx.i }
    }
}
