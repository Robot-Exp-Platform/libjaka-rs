use std::marker::ConstParamTy;

use serde::{Deserialize, Serialize};

#[derive(ConstParamTy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Command {
    PowerOn,
    PowerOff,
    EnableRobot,
    JointMove,
    // 其他指令
}

struct Request<const C: Command, D> {
    data: D,
}

struct Response<const C: Command, S> {
    state: S,
}

enum ErrorCode {
    Success,
    Exception,
    Error,
}

struct DefaultState {
    error_code: String,
    error_msg: String,
}

// power on

type PowerOnRequest = Request<{ Command::PowerOn }, ()>;
type PowerOnResponse = Response<{ Command::PowerOn }, PowerOnState>;
type PowerOnState = DefaultState;

// power off

// enable robot

// joint move
type JointMoveRequest = Request<{ Command::JointMove }, JointMoveData>;
type JointMoveResponse = Response<{ Command::JointMove }, JointMoveState>;
type JointMoveState = DefaultState;
struct JointMoveData {
    joint_angles: [f64; 6],
    speed: f64,
    accel: f64,
    relflag: u8,
}
