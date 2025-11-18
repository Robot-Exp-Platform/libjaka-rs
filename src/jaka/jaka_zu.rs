use crate::{JakaRobot, JakaType};

pub struct _JakaZu3;
pub struct _JakaZu5;
pub struct _JakaZu7;
pub struct _JakaZu12;
pub struct _JakaZu18;
pub struct _JakaZu20;
pub struct _JakaZu30;

impl JakaType for _JakaZu3 {
    const N: usize = 6;
}

impl JakaType for _JakaZu5 {
    const N: usize = 6;
}

impl JakaType for _JakaZu7 {
    const N: usize = 6;
}

impl JakaType for _JakaZu12 {
    const N: usize = 6;
}

impl JakaType for _JakaZu18 {
    const N: usize = 6;
}

impl JakaType for _JakaZu20 {
    const N: usize = 6;
}

impl JakaType for _JakaZu30 {
    const N: usize = 6;
}

pub type JakaZu3 = JakaRobot<_JakaZu3, { _JakaZu3::N }>;
pub type JakaZu5 = JakaRobot<_JakaZu5, { _JakaZu5::N }>;
pub type JakaZu7 = JakaRobot<_JakaZu7, { _JakaZu7::N }>;
pub type JakaZu12 = JakaRobot<_JakaZu12, { _JakaZu12::N }>;
pub type JakaZu18 = JakaRobot<_JakaZu18, { _JakaZu18::N }>;
pub type JakaZu20 = JakaRobot<_JakaZu20, { _JakaZu20::N }>;
pub type JakaZu30 = JakaRobot<_JakaZu30, { _JakaZu30::N }>;
