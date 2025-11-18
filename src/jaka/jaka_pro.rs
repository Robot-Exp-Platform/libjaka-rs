use crate::{JakaRobot, JakaType};

pub struct _JakaPro5;
pub struct _JakaPro12;
pub struct _JakaPro16;

impl JakaType for _JakaPro5 {
    const N: usize = 6;
}

impl JakaType for _JakaPro12 {
    const N: usize = 6;
}

impl JakaType for _JakaPro16 {
    const N: usize = 6;
}

pub type JakaPro5 = JakaRobot<_JakaPro5, { _JakaPro5::N }>;
pub type JakaPro12 = JakaRobot<_JakaPro12, { _JakaPro12::N }>;
pub type JakaPro16 = JakaRobot<_JakaPro16, { _JakaPro16::N }>;
