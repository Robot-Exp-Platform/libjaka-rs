#![feature(adt_const_params)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

#[cfg(feature = "ffi")]
pub mod ffi;
pub(crate) mod network;
mod params;
mod robot;
pub mod types;

pub use params::*;
pub use robot::JakaRobot;
