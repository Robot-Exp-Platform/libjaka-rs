use crate::JakaType;

pub struct _JakaA12L;

impl JakaType for _JakaA12L {
    const N: usize = 6;
}

pub type JakaA12L = crate::JakaRobot<_JakaA12L, { _JakaA12L::N }>;
