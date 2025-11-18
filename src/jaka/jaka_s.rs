use crate::{JakaRobot, JakaType};

pub struct _JakaS5;
pub struct _JakaS12;

impl JakaType for _JakaS5 {
    const N: usize = 6;
}

impl JakaType for _JakaS12 {
    const N: usize = 6;
}

pub type JakaS5 = JakaRobot<_JakaS5, { _JakaS5::N }>;
pub type JakaS12 = JakaRobot<_JakaS12, { _JakaS12::N }>;
