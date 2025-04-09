use std::{marker::ConstParamTy, net::SocketAddrV4};

use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(ConstParamTy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Command {
    PowerOn,
    PowerOff,
    EnableRobot,
    JointMove,
    EndMove,
    Shutdown,
    Quit,
    GetRobotState,
    DisableRobot,
    TorqueControlEnable,
    TorqueFeedforward,
    ServoMove,
    ServoJ,
    ServoP,
    GetData,
    RapidRate,
    LoadProgram,
    GetLoadedProgram,
    PlayProgram,
    PauseProgram,
    ResumeProgram,
    StopProgram,
    GetProgramState,
    SetDigitalOutput,
    GetDigitalInputStatus,
    SetAnalogOutput,
    SetToolOffsets,
    SetToolId,
    SetUserOffsets,
    SetUserId,
    GetExtioStatus,
    GetFuncdiStatus,
    DragStatus,
    QueryUserDefinedVariable,
    ModifyUserDefinedVariable,
    ProtectiveStopStatus,
    Jog,
    MoveL,
    WaitComplete, //DEPRECATED
    SetPayload,
    GetPayload,
    SetClsnSensitivity,
    GetClsnSensitivity,
    KineForward,
    KineInverse,
    ClearError,
    GetJointPos,
    GetTcpPos,
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
type PowerOffRequest = Request<{ Command::PowerOff }, ()>;
type PowerOffResponse = Response<{ Command::PowerOff }, PowerOffState>;
type PowerOffState = DefaultState;

// enable robot
type EnableRobotRequest = Request<{ Command::EnableRobot }, ()>;
type EnableRobotResponse = Response<{ Command::EnableRobot }, EnableRobotState>;
type EnableRobotState = DefaultState;

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

// end move
type EndMoveRequest = Request<{ Command::EndMove }, EndMoveData>;
type EndMoveResponse = Response<{ Command::EndMove }, EndMoveState>;
type EndMoveState = DefaultState;
struct EndMoveData {
    end_position: [f64; 6],
    speed: f64,
    accel: f64,
}

// shutdown
type ShutdownRequest = Request<{ Command::Shutdown }, ()>;
type ShutdownResponse = Response<{ Command::Shutdown }, ShutdownState>;
type ShutdownState = DefaultState;

// quit
type QuitRequest = Request<{ Command::Quit }, ()>;
type QuitResponse = Response<{ Command::Quit }, QuitState>;
type QuitState = DefaultState;

// get robot state
type GetRobotStateRequest = Request<{ Command::GetRobotState }, ()>;
type GetRobotStateResponse = Response<{ Command::GetRobotState }, GetRobotStateState>;
struct GetRobotStateState {
    error_code: String,
    error_msg: String,
    enable: String,
    power: String,
}

// disable robot
type DisableRobotRequest = Request<{ Command::DisableRobot }, ()>;
type DisableRobotResponse = Response<{ Command::DisableRobot }, DisableRobotState>;
type DisableRobotState = DefaultState;

// torque control enable
type TorqueControlEnableRequest =
    Request<{ Command::TorqueControlEnable }, TorqueControlEnableData>;
type TorqueControlEnableResponse =
    Response<{ Command::TorqueControlEnable }, TorqueControlEnableState>;
type TorqueControlEnableState = DefaultState;
struct TorqueControlEnableData {
    enable_flag: u8,
}

// torque feedforward
type TorqueFeedforwardRequest = Request<{ Command::TorqueFeedforward }, TorqueFeedforwardData>;
type TorqueFeedforwardResponse = Response<{ Command::TorqueFeedforward }, TorqueFeedforwardState>;
type TorqueFeedforwardState = DefaultState;
struct TorqueFeedforwardData {
    grv_current: [f64; 6],
    includegrvflag: u8,
}

// servo move
type ServoMoveRequest = Request<{ Command::ServoMove }, ServoMoveData>;
type ServoMoveResponse = Response<{ Command::ServoMove }, ServoMoveState>;
type ServoMoveState = DefaultState;
struct ServoMoveData {
    relflag: u8,
}

// servo j
type ServoJRequest = Request<{ Command::ServoJ }, ServoJData>;
type ServoJResponse = Response<{ Command::ServoJ }, ServoJState>;
type ServoJState = DefaultState;
struct ServoJData {
    joint_angles: [f64; 6],
    relflag: u8,
}

// servo p
type ServoPRequest = Request<{ Command::ServoP }, ServoPData>;
type ServoPResponse = Response<{ Command::ServoP }, ServoPState>;
type ServoPState = DefaultState;
struct ServoPData {
    cart_position: [f64; 6], // 文档中变量名有拼写错误
    relflag: u8,
}

// get data
type GetDataRequest = Request<{ Command::GetData }, ()>;
type GetDataResponse = Response<{ Command::GetData }, GetDataState>;
struct GetDataState {
    error_code: String,
    error_msg: String,
    tio_dout: [u8; 8],
    tool_position: [f64; 9],
    tio_ain: [u8; 2],
    paused: u8,
    cmd_name: String,
    estop: u8,
    current_tool_id: u8,
    actual_position: [f64; 9],
    joint_actual_position: [f64; 9],
    rapidrate: f64,
    enabled: u8,
}

// rapid rate
type RapidRateRequest = Request<{ Command::RapidRate }, RapidRateData>;
type RapidRateResponse = Response<{ Command::RapidRate }, RapidRateState>;
type RapidRateState = DefaultState;
struct RapidRateData {
    rate_value: f64,
}

// load program
type LoadProgramRequest = Request<{ Command::LoadProgram }, LoadProgramData>;
type LoadProgramResponse = Response<{ Command::LoadProgram }, LoadProgramState>;
type LoadProgramState = DefaultState;
struct LoadProgramData {
    program_name: String,
}

// get loaded program
type GetLoadedProgramRequest = Request<{ Command::GetLoadedProgram }, ()>;
type GetLoadedProgramResponse = Response<{ Command::GetLoadedProgram }, GetLoadedProgramState>;
struct GetLoadedProgramState {
    error_code: String,
    error_msg: String,
    program_name: String,
}

// play program
type PlayProgramRequest = Request<{ Command::PlayProgram }, ()>;
type PlayProgramResponse = Response<{ Command::PlayProgram }, PlayProgramState>;
type PlayProgramState = DefaultState;

// pause program
type PauseProgramRequest = Request<{ Command::PauseProgram }, ()>;
type PauseProgramResponse = Response<{ Command::PauseProgram }, PauseProgramState>;
type PauseProgramState = DefaultState;

// resume program
type ResumeProgramRequest = Request<{ Command::ResumeProgram }, ()>;
type ResumeProgramResponse = Response<{ Command::ResumeProgram }, ResumeProgramState>;
type ResumeProgramState = DefaultState;

// stop program
type StopRequest = Request<{ Command::StopProgram }, ()>;
type StopResponse = Response<{ Command::StopProgram }, StopState>;
type StopState = DefaultState;

// get program state
type GetProgramStateRequest = Request<{ Command::GetProgramState }, ()>;
type GetProgramStateResponse = Response<{ Command::GetProgramState }, GetProgramStateState>;
struct GetProgramStateState {
    error_code: String,
    error_msg: String,
    program_state: String,
}

// set digital output
type SetDigitalOutputRequest = Request<{ Command::SetDigitalOutput }, SetDigitalOutputData>;
type SetDigitalOutputResponse = Response<{ Command::SetDigitalOutput }, SetDigitalOutputState>;
type SetDigitalOutputState = DefaultState;
struct SetDigitalOutputData {
    type_number: u8,
    index: u8,
    value: u8,
}

// get digital input status
type GetDigitalInputStatusRequest = Request<{ Command::GetDigitalInputStatus }, ()>;
type GetDigitalInputStatusResponse =
    Response<{ Command::GetDigitalInputStatus }, GetDigitalInputStatusState>;
struct GetDigitalInputStatusState {
    error_code: String,
    error_msg: String,
    din_status: [u8; 64],
}

// set analog output
type SetAnalogOutputRequest = Request<{ Command::SetAnalogOutput }, SetAnalogOutputData>;
type SetAnalogOutputResponse = Response<{ Command::SetAnalogOutput }, SetAnalogOutputState>;
type SetAnalogOutputState = DefaultState;
struct SetAnalogOutputData {
    type_number: u8,
    index: u8,
    value: f64,
}

// set tool offsets
type SetToolOffsetsRequest = Request<{ Command::SetToolOffsets }, SetToolOffsetsData>;
type SetToolOffsetsResponse = Response<{ Command::SetToolOffsets }, SetToolOffsetsState>;
type SetToolOffsetsState = DefaultState;
struct SetToolOffsetsData {
    tooloffset: [f64; 6],
    id: u8,
    name: String,
}

// set tool id
type SetToolIdRequest = Request<{ Command::SetToolId }, SetToolIdData>;
type SetToolIdResponse = Response<{ Command::SetToolId }, SetToolIdState>;
type SetToolIdState = DefaultState;
struct SetToolIdData {
    tool_id: u8,
}

// set user offsets
type SetUserOffsetsRequest = Request<{ Command::SetUserOffsets }, SetUserOffsetsData>;
type SetUserOffsetsResponse = Response<{ Command::SetUserOffsets }, SetUserOffsetsState>;
type SetUserOffsetsState = DefaultState;
struct SetUserOffsetsData {
    useroffset: [f64; 6], //文档中的变量名有拼写错误
    id: u8,
    name: String,
}

// set user id
type SetUserIdRequest = Request<{ Command::SetUserId }, SetUserIdData>;
type SetUserIdResponse = Response<{ Command::SetUserId }, SetUserIdState>;
type SetUserIdState = DefaultState;
struct SetUserIdData {
    user_frame_id: u8, //文档中的变量名说明有错误
}

// get extio status
type GetExtioStatusRequest = Request<{ Command::GetExtioStatus }, ()>;
type GetExtioStatusResponse = Response<{ Command::GetExtioStatus }, GetExtioStatusState>;
struct GetExtioStatusState {
    error_code: String,
    error_msg: String,
    extio_status: ExtioStatus,
}
struct ExtioStatus {
    version: u8,
    setups: Vec<Value>,
}
// struct ExtioSetup {
//     setup_id: u8,
//     mobus_io_name: String,
//     address: SocketAddrV4,
//     channel_counts: ChannelCounts,
//     pinmap: HashMap<String, u8>,
// }
// struct ChannelCounts {
//     ai: (u8, u8),
//     do_: (u8, u8),
//     ao: (u8, u8),
//     di: (u8, u8),
// }
// struct ExtioCurrentState {
//     state: u8,
//     di: (u8, u8, u8, u8, u8, u8, u8, u8),
// }

// get funcdi status
type GetFuncdiStatusRequest = Request<{ Command::GetFuncdiStatus }, ()>;
type GetFuncdiStatusResponse = Response<{ Command::GetFuncdiStatus }, GetFuncdiStatusState>;
struct GetFuncdiStatusState {
    error_code: String,
    error_msg: String,
    funcdi_status: [[i8; 2]; 12],
}

// drag status
type DragStatusRequest = Request<{ Command::DragStatus }, ()>;
type DragStatusResponse = Response<{ Command::DragStatus }, DragStatusState>;
struct DragStatusState {
    error_code: String,
    error_msg: String,
    drag_status: bool,
}

// query user defined variable
type QueryUserDefinedVariableRequest = Request<{ Command::QueryUserDefinedVariable }, ()>;
type QueryUserDefinedVariableResponse =
    Response<{ Command::QueryUserDefinedVariable }, QueryUserDefinedVariableState>;
struct QueryUserDefinedVariableState {
    error_code: String,
    error_msg: String,
    var_list: Vec<Variable>,
}
struct Variable {
    alias: String,
    id: u32,
    value: f64,
}

// modify user defined variable
type ModifyUserDefinedVariableRequest =
    Request<{ Command::ModifyUserDefinedVariable }, ModifyUserDefinedVariableData>;
type ModifyUserDefinedVariableResponse =
    Response<{ Command::ModifyUserDefinedVariable }, ModifyUserDefinedVariableState>;
type ModifyUserDefinedVariableState = DefaultState;
struct ModifyUserDefinedVariableData {
    id_new: u32,
    alias_new: String,
    value_new: f64,
}

// protective stop status
type ProtectiveStopStatusRequest = Request<{ Command::ProtectiveStopStatus }, ()>;
type ProtectiveStopStatusResponse =
    Response<{ Command::ProtectiveStopStatus }, ProtectiveStopStatusState>;
struct ProtectiveStopStatusState {
    error_code: String,
    error_msg: String,
    protective_stop: u8,
}

// jog
type JogRequest = Request<{ Command::Jog }, JogData>;
type JogResponse = Response<{ Command::Jog }, JogState>;
type JogState = DefaultState;
enum JogData {
    Mode0 {
        coord_map: u8,
        jnum: u8,
    },
    Mode1 {
        coord_map: u8,
        jnum: u8,
        jogvel: f64,
    },
    Mode2 {
        coord_map: u8,
        jnum: u8,
        jogvel: f64,
        poscmd: f64,
    },
}

impl JogData {
    fn jog_mode(&self) -> u8 {
        match self {
            JogData::Mode0 { .. } => 0,
            JogData::Mode1 { .. } => 1,
            JogData::Mode2 { .. } => 2,
        }
    }
}

// move l
type MoveLRequest = Request<{ Command::MoveL }, MoveLData>;
type MoveLResponse = Response<{ Command::MoveL }, MoveLState>;
type MoveLState = DefaultState;
struct MoveLData {
    cart_position: [f64; 6],
    speed: f64,
    accel: f64,
    relflag: u8,
}

// wait complete DEPRECATED
type WaitCompleteRequest = Request<{ Command::WaitComplete }, ()>;
type WaitCompleteResponse = Response<{ Command::WaitComplete }, WaitCompleteState>;
type WaitCompleteState = DefaultState;

// set payload
type SetPayloadRequest = Request<{ Command::SetPayload }, SetPayloadData>;
type SetPayloadResponse = Response<{ Command::SetPayload }, SetPayloadState>;
type SetPayloadState = DefaultState;
struct SetPayloadData {
    mass: f64,
    centroid: [f64; 3],
}

// get payload
type GetPayloadRequest = Request<{ Command::GetPayload }, ()>;
type GetPayloadResponse = Response<{ Command::GetPayload }, GetPayloadState>;
struct GetPayloadState {
    error_code: String,
    error_msg: String,
    mass: f64,
    centroid: [f64; 3],
}

// set clsn sensitivity
type SetClsnSensitivityRequest = Request<{ Command::SetClsnSensitivity }, SetClsnSensitivityData>;
type SetClsnSensitivityResponse =
    Response<{ Command::SetClsnSensitivity }, SetClsnSensitivityState>;
type SetClsnSensitivityState = DefaultState;
struct SetClsnSensitivityData {
    sensitivity_level: u8, // 更改了文档中的变量名以与下一命令一致
}

// get clsn sensitivity
type GetClsnSensitivityRequest = Request<{ Command::GetClsnSensitivity }, ()>;
type GetClsnSensitivityResponse =
    Response<{ Command::GetClsnSensitivity }, GetClsnSensitivityState>;
struct GetClsnSensitivityState {
    error_code: String,
    error_msg: String,
    sensitivity_level: u8,
}

// kine forward
type KineForwardRequest = Request<{ Command::KineForward }, ()>;
type KineForwardResponse = Response<{ Command::KineForward }, KineForwardState>;
struct KineForwardState {
    error_code: String,
    error_msg: String,
    cart_position: [f64; 6],
}
struct KineForwardData {
    joint_angles: [f64; 6],
}

// kine inverse
type KineInverseRequest = Request<{ Command::KineInverse }, KineInverseData>;
type KineInverseResponse = Response<{ Command::KineInverse }, KineInverseState>;
struct KineInverseState {
    error_code: String,
    error_msg: String,
    joint_angles: [f64; 6],
}
struct KineInverseData {
    joint_angles: [f64; 6],
    cart_position: [f64; 6],
}

// clear error
type ClearErrorRequest = Request<{ Command::ClearError }, ()>;
type ClearErrorResponse = Response<{ Command::ClearError }, ClearErrorState>;
type ClearErrorState = DefaultState;

// get joint pos
type GetJointPosRequest = Request<{ Command::GetJointPos }, ()>;
type GetJointPosResponse = Response<{ Command::GetJointPos }, GetJointPosState>;
struct GetJointPosState {
    error_code: String,
    error_msg: String,
    joint_angles: [f64; 6],
}

// get tcp pos
type GetTcpPosRequest = Request<{ Command::GetTcpPos }, ()>;
type GetTcpPosResponse = Response<{ Command::GetTcpPos }, GetTcpPosState>;
struct GetTcpPosState {
    error_code: String,
    error_msg: String,
    tcp_pos: [f64; 6],
}
