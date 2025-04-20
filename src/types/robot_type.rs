use std::marker::ConstParamTy;

use robot_behavior::{RobotException, RobotResult};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use serde_json::Value;
use serde_with::serde_as;

pub trait CommandSerde: Sized {
    fn serialize(&self) -> String;
    fn deserialize(data: &str) -> RobotResult<Self>;
}

#[derive(ConstParamTy, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub enum Command {
    #[serde(rename = "power_on")]
    PowerOn,
    #[serde(rename = "power_off")]
    PowerOff,
    #[serde(rename = "enable_robot")]
    EnableRobot,
    #[serde(rename = "joint_move")]
    JointMove,
    #[serde(rename = "end_move")]
    EndMove,
    #[serde(rename = "shutdown")]
    Shutdown,
    #[serde(rename = "quit")]
    Quit,
    #[serde(rename = "get_robot_state")]
    GetRobotState,
    #[serde(rename = "disable_robot")]
    DisableRobot,
    #[serde(rename = "torque_control_enable")]
    TorqueControlEnable,
    #[serde(rename = "torque_feedforward")]
    TorqueFeedforward,
    #[serde(rename = "servo_move")]
    ServoMove,
    #[serde(rename = "servo_j")]
    ServoJ,
    #[serde(rename = "servo_p")]
    ServoP,
    #[serde(rename = "get_data")]
    GetData,
    #[serde(rename = "rapid_rate")]
    RapidRate,
    #[serde(rename = "load_program")]
    LoadProgram,
    #[serde(rename = "get_loaded_program")]
    GetLoadedProgram,
    #[serde(rename = "play_program")]
    PlayProgram,
    #[serde(rename = "pause_program")]
    PauseProgram,
    #[serde(rename = "resume_program")]
    ResumeProgram,
    #[serde(rename = "stop_program")]
    StopProgram,
    #[serde(rename = "get_program_state")]
    GetProgramState,
    #[serde(rename = "set_digital_output")]
    SetDigitalOutput,
    #[serde(rename = "get_digital_input_status")]
    GetDigitalInputStatus,
    #[serde(rename = "set_analog_output")]
    SetAnalogOutput,
    #[serde(rename = "set_tool_offsets")]
    SetToolOffsets,
    #[serde(rename = "set_tool_id")]
    SetToolId,
    #[serde(rename = "set_user_offsets")]
    SetUserOffsets,
    #[serde(rename = "set_user_id")]
    SetUserId,
    #[serde(rename = "get_extio_status")]
    GetExtioStatus,
    #[serde(rename = "get_funcdi_status")]
    GetFuncdiStatus,
    #[serde(rename = "drag_status")]
    DragStatus,
    #[serde(rename = "query_user_defined_variable")]
    QueryUserDefinedVariable,
    #[serde(rename = "modify_user_defined_variable")]
    ModifyUserDefinedVariable,
    #[serde(rename = "protective_stop_status")]
    ProtectiveStopStatus,
    #[serde(rename = "jog")]
    Jog,
    #[serde(rename = "move_l")]
    MoveL,
    #[serde(rename = "wait_complete")] //DEPRECATED
    WaitComplete, //DEPRECATED
    #[serde(rename = "set_payload")]
    SetPayload,
    #[serde(rename = "get_payload")]
    GetPayload,
    #[serde(rename = "set_clsn_sensitivity")]
    SetClsnSensitivity,
    #[serde(rename = "get_clsn_sensitivity")]
    GetClsnSensitivity,
    #[serde(rename = "kine_forward")]
    KineForward,
    #[serde(rename = "kine_inverse")]
    KineInverse,
    #[serde(rename = "clear_error")]
    ClearError,
    #[serde(rename = "get_joint_pos")]
    GetJointPos,
    #[serde(rename = "get_tcp_pos")]
    GetTcpPos,
}
pub struct Request<const C: Command, D> {
    pub data: D,
}

pub struct Response<const C: Command, S> {
    pub state: S,
}

pub enum ErrorCode {
    Success,
    Exception,
    Error,
}

#[derive(Serialize, Deserialize)]
pub struct DefaultState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
}

// power on
pub type PowerOnRequest = Request<{ Command::PowerOn }, ()>;
pub type PowerOnResponse = Response<{ Command::PowerOn }, PowerOnState>;
pub type PowerOnState = DefaultState;

// power off
pub type PowerOffRequest = Request<{ Command::PowerOff }, ()>;
pub type PowerOffResponse = Response<{ Command::PowerOff }, PowerOffState>;
pub type PowerOffState = DefaultState;

// enable robot
pub type EnableRobotRequest = Request<{ Command::EnableRobot }, ()>;
pub type EnableRobotResponse = Response<{ Command::EnableRobot }, EnableRobotState>;
pub type EnableRobotState = DefaultState;

// joint move
pub type JointMoveRequest = Request<{ Command::JointMove }, JointMoveData>;
pub type JointMoveResponse = Response<{ Command::JointMove }, JointMoveState>;
pub type JointMoveState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct JointMoveData {
    #[serde(rename = "jointPosition")]
    pub joint_position: [f64; 6],
    pub speed: f64,
    pub accel: f64,
    #[serde(rename = "relFlag")]
    pub relflag: u8,
}

// end move
pub type EndMoveRequest = Request<{ Command::EndMove }, EndMoveData>;
pub type EndMoveResponse = Response<{ Command::EndMove }, EndMoveState>;
pub type EndMoveState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct EndMoveData {
    #[serde(rename = "endPosition")]
    pub end_position: [f64; 6],
    pub speed: f64,
    pub accel: f64,
}

// shutdown
pub type ShutdownRequest = Request<{ Command::Shutdown }, ()>;
pub type ShutdownResponse = Response<{ Command::Shutdown }, ShutdownState>;
pub type ShutdownState = DefaultState;

// quit
pub type QuitRequest = Request<{ Command::Quit }, ()>;
pub type QuitResponse = Response<{ Command::Quit }, QuitState>;
pub type QuitState = DefaultState;

// get robot state
pub type GetRobotStateRequest = Request<{ Command::GetRobotState }, ()>;
pub type GetRobotStateResponse = Response<{ Command::GetRobotState }, GetRobotStateState>;
#[derive(Serialize, Deserialize)]
pub struct GetRobotStateState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub enable: String,
    pub power: String,
}

// disable robot
pub type DisableRobotRequest = Request<{ Command::DisableRobot }, ()>;
pub type DisableRobotResponse = Response<{ Command::DisableRobot }, DisableRobotState>;
pub type DisableRobotState = DefaultState;

// torque control enable
pub type TorqueControlEnableRequest =
    Request<{ Command::TorqueControlEnable }, TorqueControlEnableData>;
pub type TorqueControlEnableResponse =
    Response<{ Command::TorqueControlEnable }, TorqueControlEnableState>;
pub type TorqueControlEnableState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct TorqueControlEnableData {
    pub enable_flag: u8,
}

// torque feedforward
pub type TorqueFeedforwardRequest = Request<{ Command::TorqueFeedforward }, TorqueFeedforwardData>;
pub type TorqueFeedforwardResponse =
    Response<{ Command::TorqueFeedforward }, TorqueFeedforwardState>;
pub type TorqueFeedforwardState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct TorqueFeedforwardData {
    #[serde(rename = "grvCurrent")]
    pub grv_current: [f64; 6],
    #[serde(rename = "includeGrvFlag")]
    pub includegrvflag: u8,
}

// servo move
pub type ServoMoveRequest = Request<{ Command::ServoMove }, ServoMoveData>;
pub type ServoMoveResponse = Response<{ Command::ServoMove }, ServoMoveState>;
pub type ServoMoveState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct ServoMoveData {
    #[serde(rename = "relFlag")]
    pub relflag: u8,
}

// servo j
pub type ServoJRequest = Request<{ Command::ServoJ }, ServoJData>;
pub type ServoJResponse = Response<{ Command::ServoJ }, ServoJState>;
pub type ServoJState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct ServoJData {
    #[serde(rename = "jointAngles")]
    pub joint_angles: [f64; 6],
    #[serde(rename = "relFlag")]
    pub relflag: u8,
}

// servo p
pub type ServoPRequest = Request<{ Command::ServoP }, ServoPData>;
pub type ServoPResponse = Response<{ Command::ServoP }, ServoPState>;
pub type ServoPState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct ServoPData {
    #[serde(rename = "catPosition")]
    pub cat_position: [f64; 6],
    #[serde(rename = "relFlag")]
    pub relflag: u8,
}

// get data
pub type GetDataRequest = Request<{ Command::GetData }, ()>;
pub type GetDataResponse = Response<{ Command::GetData }, GetDataState>;
#[derive(Serialize, Deserialize)]
pub struct GetDataState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub tio_dout: [u8; 8],
    pub tool_position: [f64; 9],
    pub paused: u8,
    pub cmd_name: String,
    pub estop: u8,
    pub current_tool_id: u8,
    pub tio_ain: [u8; 2],
    pub actual_position: [f64; 9],
    pub joint_actual_position: [f64; 9],
    pub rapidrate: f64,
    pub enabled: u8,
}

// rapid rate
pub type RapidRateRequest = Request<{ Command::RapidRate }, RapidRateData>;
pub type RapidRateResponse = Response<{ Command::RapidRate }, RapidRateState>;
pub type RapidRateState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct RapidRateData {
    pub rate_value: f64,
}

// load program
pub type LoadProgramRequest = Request<{ Command::LoadProgram }, LoadProgramData>;
pub type LoadProgramResponse = Response<{ Command::LoadProgram }, LoadProgramState>;
pub type LoadProgramState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct LoadProgramData {
    #[serde(rename = "programName")]
    pub program_name: String,
}

// get loaded program
pub type GetLoadedProgramRequest = Request<{ Command::GetLoadedProgram }, ()>;
pub type GetLoadedProgramResponse = Response<{ Command::GetLoadedProgram }, GetLoadedProgramState>;
#[derive(Serialize, Deserialize)]
pub struct GetLoadedProgramState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    #[serde(rename = "programName")]
    pub program_name: String,
}

// play program
pub type PlayProgramRequest = Request<{ Command::PlayProgram }, ()>;
pub type PlayProgramResponse = Response<{ Command::PlayProgram }, PlayProgramState>;
pub type PlayProgramState = DefaultState;

// pause program
pub type PauseProgramRequest = Request<{ Command::PauseProgram }, ()>;
pub type PauseProgramResponse = Response<{ Command::PauseProgram }, PauseProgramState>;
pub type PauseProgramState = DefaultState;

// resume program
pub type ResumeProgramRequest = Request<{ Command::ResumeProgram }, ()>;
pub type ResumeProgramResponse = Response<{ Command::ResumeProgram }, ResumeProgramState>;
pub type ResumeProgramState = DefaultState;

// stop program
pub type StopProgramRequest = Request<{ Command::StopProgram }, ()>;
pub type StopProgramResponse = Response<{ Command::StopProgram }, StopProgramState>;
pub type StopProgramState = DefaultState;

// get program state
pub type GetProgramStateRequest = Request<{ Command::GetProgramState }, ()>;
pub type GetProgramStateResponse = Response<{ Command::GetProgramState }, GetProgramStateState>;
#[derive(Serialize, Deserialize)]
pub struct GetProgramStateState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    #[serde(rename = "programName")]
    pub program_state: String,
}

// set digital output
pub type SetDigitalOutputRequest = Request<{ Command::SetDigitalOutput }, SetDigitalOutputData>;
pub type SetDigitalOutputResponse = Response<{ Command::SetDigitalOutput }, SetDigitalOutputState>;
pub type SetDigitalOutputState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetDigitalOutputData {
    #[serde(rename = "type")]
    pub type_number: u8,
    pub index: u8,
    pub value: u8,
}

// get digital input status
pub type GetDigitalInputStatusRequest = Request<{ Command::GetDigitalInputStatus }, ()>;
pub type GetDigitalInputStatusResponse =
    Response<{ Command::GetDigitalInputStatus }, GetDigitalInputStatusState>;
#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct GetDigitalInputStatusState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    #[serde_as(as = "[_; 64]")]
    pub din_status: [u8; 64],
}

// set analog output
pub type SetAnalogOutputRequest = Request<{ Command::SetAnalogOutput }, SetAnalogOutputData>;
pub type SetAnalogOutputResponse = Response<{ Command::SetAnalogOutput }, SetAnalogOutputState>;
pub type SetAnalogOutputState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetAnalogOutputData {
    #[serde(rename = "type")]
    pub type_number: u8,
    pub index: u8,
    pub value: f64,
}

// set tool offsets
pub type SetToolOffsetsRequest = Request<{ Command::SetToolOffsets }, SetToolOffsetsData>;
pub type SetToolOffsetsResponse = Response<{ Command::SetToolOffsets }, SetToolOffsetsState>;
pub type SetToolOffsetsState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetToolOffsetsData {
    pub tooloffset: [f64; 6],
    pub id: u8,
    pub name: String,
}

// set tool id
pub type SetToolIdRequest = Request<{ Command::SetToolId }, SetToolIdData>;
pub type SetToolIdResponse = Response<{ Command::SetToolId }, SetToolIdState>;
pub type SetToolIdState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetToolIdData {
    pub tool_id: u8,
}

// pub set user offsets
pub type SetUserOffsetsRequest = Request<{ Command::SetUserOffsets }, SetUserOffsetsData>;
pub type SetUserOffsetsResponse = Response<{ Command::SetUserOffsets }, SetUserOffsetsState>;
pub type SetUserOffsetsState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetUserOffsetsData {
    pub useroffset: [f64; 6], //文档中的变量名有拼写错误
    pub id: u8,
    pub name: String,
}

// set user id
pub type SetUserIdRequest = Request<{ Command::SetUserId }, SetUserIdData>;
pub type SetUserIdResponse = Response<{ Command::SetUserId }, SetUserIdState>;
pub type SetUserIdState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetUserIdData {
    pub user_frame_id: u8, //文档中的变量名说明有错误
}

// get extio status
pub type GetExtioStatusRequest = Request<{ Command::GetExtioStatus }, ()>;
pub type GetExtioStatusResponse = Response<{ Command::GetExtioStatus }, GetExtioStatusState>;
#[derive(Serialize, Deserialize)]
pub struct GetExtioStatusState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub extio_status: ExtioStatus,
}
#[derive(Serialize, Deserialize)]
pub struct ExtioStatus {
    pub version: u8,
    pub setups: Vec<Value>,
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
pub type GetFuncdiStatusRequest = Request<{ Command::GetFuncdiStatus }, ()>;
pub type GetFuncdiStatusResponse = Response<{ Command::GetFuncdiStatus }, GetFuncdiStatusState>;
#[derive(Serialize, Deserialize)]
pub struct GetFuncdiStatusState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub funcdi_status: [[i8; 2]; 12],
}

// pub drag status
pub type DragStatusRequest = Request<{ Command::DragStatus }, ()>;
pub type DragStatusResponse = Response<{ Command::DragStatus }, DragStatusState>;
#[derive(Serialize, Deserialize)]
pub struct DragStatusState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub drag_status: bool,
}

// query user defined variable
pub type QueryUserDefinedVariableRequest = Request<{ Command::QueryUserDefinedVariable }, ()>;
pub type QueryUserDefinedVariableResponse =
    Response<{ Command::QueryUserDefinedVariable }, QueryUserDefinedVariableState>;
#[derive(Serialize, Deserialize)]
pub struct QueryUserDefinedVariableState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub var_list: Vec<Value>,
}

// modify user defined variable
pub type ModifyUserDefinedVariableRequest =
    Request<{ Command::ModifyUserDefinedVariable }, ModifyUserDefinedVariableData>;
pub type ModifyUserDefinedVariableResponse =
    Response<{ Command::ModifyUserDefinedVariable }, ModifyUserDefinedVariableState>;
pub type ModifyUserDefinedVariableState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct ModifyUserDefinedVariableData {
    pub id_new: u32,
    pub alias_new: String,
    pub value_new: Value,
}

// protective stop status
pub type ProtectiveStopStatusRequest = Request<{ Command::ProtectiveStopStatus }, ()>;
pub type ProtectiveStopStatusResponse =
    Response<{ Command::ProtectiveStopStatus }, ProtectiveStopStatusState>;
#[derive(Serialize, Deserialize)]
pub struct ProtectiveStopStatusState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub protective_stop: u8,
}

// jog
pub type JogRequest = Request<{ Command::Jog }, JogData>;
pub type JogResponse = Response<{ Command::Jog }, JogState>;
pub type JogState = DefaultState;
#[derive(Serialize, Deserialize)]
pub enum JogData {
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
    pub fn jog_mode(&self) -> u8 {
        match self {
            JogData::Mode0 { .. } => 0,
            JogData::Mode1 { .. } => 1,
            JogData::Mode2 { .. } => 2,
        }
    }
}

// move l
pub type MoveLRequest = Request<{ Command::MoveL }, MoveLData>;
pub type MoveLResponse = Response<{ Command::MoveL }, MoveLState>;
pub type MoveLState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct MoveLData {
    #[serde(rename = "cartPosition")]
    pub cart_position: [f64; 6],
    pub speed: f64,
    pub accel: f64,
    #[serde(rename = "relFlag")]
    pub relflag: u8,
}

// wait complete DEPRECATED
pub type WaitCompleteRequest = Request<{ Command::WaitComplete }, ()>;
pub type WaitCompleteResponse = Response<{ Command::WaitComplete }, WaitCompleteState>;
pub type WaitCompleteState = DefaultState;

// set payload
pub type SetPayloadRequest = Request<{ Command::SetPayload }, SetPayloadData>;
pub type SetPayloadResponse = Response<{ Command::SetPayload }, SetPayloadState>;
pub type SetPayloadState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetPayloadData {
    pub mass: f64,
    pub centroid: [f64; 3],
}

// get payload
pub type GetPayloadRequest = Request<{ Command::GetPayload }, ()>;
pub type GetPayloadResponse = Response<{ Command::GetPayload }, GetPayloadState>;
#[derive(Serialize, Deserialize)]
pub struct GetPayloadState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub mass: f64,
    pub centroid: [f64; 3],
}

// set clsn sensitivity
pub type SetClsnSensitivityRequest =
    Request<{ Command::SetClsnSensitivity }, SetClsnSensitivityData>;
pub type SetClsnSensitivityResponse =
    Response<{ Command::SetClsnSensitivity }, SetClsnSensitivityState>;
pub type SetClsnSensitivityState = DefaultState;
#[derive(Serialize, Deserialize)]
pub struct SetClsnSensitivityData {
    #[serde(rename = "sensitivityVal")]
    pub sensitivity_level: u8, // 更改了文档中的变量名以与下一命令一致
}

// get clsn sensitivity
pub type GetClsnSensitivityRequest = Request<{ Command::GetClsnSensitivity }, ()>;
pub type GetClsnSensitivityResponse =
    Response<{ Command::GetClsnSensitivity }, GetClsnSensitivityState>;
#[derive(Serialize, Deserialize)]
pub struct GetClsnSensitivityState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    #[serde(rename = "sensitivityLevel")]
    pub sensitivity_level: u8,
}

// kine forward
pub type KineForwardRequest = Request<{ Command::KineForward }, ()>;
pub type KineForwardResponse = Response<{ Command::KineForward }, KineForwardState>;
#[derive(Serialize, Deserialize)]
pub struct KineForwardState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    #[serde(rename = "jointPosition")]
    pub joint_angles: [f64; 6],
}
#[derive(Serialize, Deserialize)]
pub struct KineForwardData {
    #[serde(rename = "cartPosition")]
    pub cart_position: [f64; 6],
}

// kine inverse
pub type KineInverseRequest = Request<{ Command::KineInverse }, KineInverseData>;
pub type KineInverseResponse = Response<{ Command::KineInverse }, KineInverseState>;
#[derive(Serialize, Deserialize)]
pub struct KineInverseState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    #[serde(rename = "cartPosition")]
    pub cart_position: [f64; 6],
}
#[derive(Serialize, Deserialize)]
pub struct KineInverseData {
    #[serde(rename = "jointPosition")]
    pub joint_angles: [f64; 6],
}

// clear error
pub type ClearErrorRequest = Request<{ Command::ClearError }, ()>;
pub type ClearErrorResponse = Response<{ Command::ClearError }, ClearErrorState>;
pub type ClearErrorState = DefaultState;

// get joint pos
pub type GetJointPosRequest = Request<{ Command::GetJointPos }, ()>;
pub type GetJointPosResponse = Response<{ Command::GetJointPos }, GetJointPosState>;
#[derive(Serialize, Deserialize)]
pub struct GetJointPosState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub joint_pos: [f64; 6],
}

// get tcp pos
pub type GetTcpPosRequest = Request<{ Command::GetTcpPos }, ()>;
pub type GetTcpPosResponse = Response<{ Command::GetTcpPos }, GetTcpPosState>;
#[derive(Serialize, Deserialize)]
pub struct GetTcpPosState {
    #[serde(rename = "errorCode")]
    pub error_code: String,
    #[serde(rename = "errorMsg")]
    pub error_msg: String,
    pub tcp_pos: [f64; 6],
}

impl<const C: Command, D> From<D> for Request<C, D> {
    fn from(data: D) -> Self {
        Self { data }
    }
}

impl<const C: Command, S> From<S> for Response<C, S> {
    fn from(state: S) -> Self {
        Self { state }
    }
}

impl<const C: Command, S> Response<C, S> {
    pub fn into_state(self) -> S {
        self.state
    }
}

impl<const C: Command, D> CommandSerde for Request<C, D>
where
    D: Serialize + DeserializeOwned,
{
    fn serialize(&self) -> String {
        let mut value = serde_json::to_value(&self.data).unwrap();
        let command = serde_json::to_value(&C).unwrap();
        match &mut value {
            Value::Object(obj) => {
                obj.insert("cmdName".to_string(), command);
            }
            Value::Null => {
                let mut obj = serde_json::Map::new();
                obj.insert("cmdName".to_string(), serde_json::to_value(C).unwrap());
                value = Value::Object(obj);
            }
            _ => {}
        }
        value.to_string()
    }
    fn deserialize(data: &str) -> RobotResult<Self> {
        let mut value: Value = serde_json::from_str(data).unwrap();
        if let Value::Object(obj) = &mut value {
            obj.remove("cmdName");
        }
        serde_json::from_value::<D>(value)
            .map(|data| Request { data })
            .map_err(|e| RobotException::DeserializeError(e.to_string()))
    }
}

impl<const C: Command, S> CommandSerde for Response<C, S>
where
    S: Serialize + DeserializeOwned,
{
    fn serialize(&self) -> String {
        let mut value = serde_json::to_value(&self.state).unwrap();
        match &mut value {
            Value::Object(obj) => {
                obj.insert("cmdName".to_string(), serde_json::to_value(C).unwrap());
            }
            Value::Null => {
                let mut obj = serde_json::Map::new();
                obj.insert("cmdName".to_string(), serde_json::to_value(C).unwrap());
                value = Value::Object(obj);
            }
            _ => {}
        }
        value.to_string()
    }
    fn deserialize(data: &str) -> RobotResult<Self> {
        let mut value: Value = serde_json::from_str(data).unwrap();
        if let Value::Object(obj) = &mut value {
            obj.remove("cmdName");
        }
        serde_json::from_value::<S>(value)
            .map(|state| Response { state })
            .map_err(|e| RobotException::DeserializeError(e.to_string()))
    }
}

#[cfg(test)]
mod tests {}
