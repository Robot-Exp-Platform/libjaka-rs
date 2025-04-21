use crate::{JAKA_DOF, JAKA_VERSION, network::NetWork, types::*};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, ArmState, ControlType, LoadState,
    MotionType, Pose, RobotBehavior, RobotResult,
};
use std::sync::{Arc, RwLock};

/// # Jaja Robot (节卡机器人)
pub struct JakaRobot {
    network: NetWork,
    robot_state: Arc<RwLock<RobotState>>,
    is_moving: bool,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty; $ret_type:ty) => {
        fn $fn_name(&mut self, arg: $arg_type) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> =
                self.network
                    .send_and_recv(Request::<$command, $arg_type>::from(arg))?;
            Ok(response.state)
        }
    };
    ($fn_name:ident, $command:expr;; $ret_type:ty) => {
        fn $fn_name(&mut self) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> =
                self.network
                    .send_and_recv(Request::<$command, ()>::from(()))?;
            Ok(response.state)
        }
    };
}

impl JakaRobot {
    /// Create a new JakaRobot instance with the given IP address.
    ///
    /// # Arguments
    /// * `ip` - A string slice that holds the IP address of the robot.
    pub fn new(ip: &str) -> Self {
        let network = NetWork::new(ip);
        let robot_state = NetWork::state_connect(ip);
        Self {
            network,
            robot_state,
            is_moving: false,
        }
    }

    cmd_fn!(_power_on, {Command::PowerOn};; PowerOnState);
    cmd_fn!(_power_off, {Command::PowerOff};; PowerOffState);
    cmd_fn!(_enable, {Command::EnableRobot};; EnableRobotState);
    cmd_fn!(_joint_move, {Command::JointMove}; data: JointMoveData; JointMoveState);
    cmd_fn!(_end_move, {Command::EndMove};data: EndMoveData; EndMoveState);
    cmd_fn!(_shutdown, {Command::Shutdown};; ShutdownState);
    cmd_fn!(_quit, {Command::Quit};; QuitState);
    cmd_fn!(_get_robot_state, {Command::GetRobotState};; GetRobotStateState);
    cmd_fn!(_disable, {Command::DisableRobot};; DisableRobotState);
    cmd_fn!(_torque_control_enable, {Command::TorqueControlEnable};; TorqueControlEnableState);
    cmd_fn!(_torque_feedforward, {Command::TorqueFeedforward}; data: TorqueFeedforwardData; TorqueFeedforwardState);
    cmd_fn!(_servo_move, {Command::ServoMove}; data: ServoMoveData; ServoMoveState);
    cmd_fn!(_servo_j, {Command::ServoJ}; data: ServoJData; ServoJState);
    cmd_fn!(_servo_p, {Command::ServoP}; data: ServoPData; ServoPState);
    cmd_fn!(_get_data, {Command::GetData};; GetDataState);
    cmd_fn!(_rapid_rate, {Command::RapidRate}; data: RapidRateData; RapidRateState);
    cmd_fn!(_load_program, {Command::LoadProgram}; data: LoadProgramData; LoadProgramState);
    cmd_fn!(_get_loaded_program, {Command::GetLoadedProgram};; GetLoadedProgramState);
    cmd_fn!(_play_program, {Command::PlayProgram};; PlayProgramState);
    cmd_fn!(_pause_program, {Command::PauseProgram};; PauseProgramState);
    cmd_fn!(_resume_program, {Command::ResumeProgram};; ResumeProgramState);
    cmd_fn!(_stop_program, {Command::StopProgram};; StopProgramState);
    cmd_fn!(_get_program_state, {Command::GetProgramState};; GetProgramStateState);
    cmd_fn!(_set_digital_output, {Command::SetDigitalOutput}; data: SetDigitalOutputData; SetDigitalOutputState);
    cmd_fn!(_get_digital_input_status, {Command::GetDigitalInputStatus};; GetDigitalInputStatusState);
    cmd_fn!(_set_analog_output, {Command::SetAnalogOutput}; data: SetAnalogOutputData; SetAnalogOutputState);
    cmd_fn!(_set_tool_offsets, {Command::SetToolOffsets}; data: SetToolOffsetsData; SetToolOffsetsState);
    cmd_fn!(_set_tool_id, {Command::SetToolId}; data: SetToolIdData; SetToolIdState);
    cmd_fn!(_set_user_offsets, {Command::SetUserOffsets}; data: SetUserOffsetsData; SetUserOffsetsState);
    cmd_fn!(_set_user_id, {Command::SetUserId}; data: SetUserIdData; SetUserIdState);
    cmd_fn!(_get_extio_status, {Command::GetExtioStatus};; GetExtioStatusState);
    cmd_fn!(_get_funcdi_status, {Command::GetFuncdiStatus};; GetFuncdiStatusState);
    cmd_fn!(_drag_status, {Command::DragStatus};; DragStatusState);
    cmd_fn!(_query_user_defined_variable, {Command::QueryUserDefinedVariable};; QueryUserDefinedVariableState);
    cmd_fn!(_modify_user_defined_variable, {Command::ModifyUserDefinedVariable}; data: ModifyUserDefinedVariableData; ModifyUserDefinedVariableState);
    cmd_fn!(_protective_stop_status, {Command::ProtectiveStopStatus};; ProtectiveStopStatusState);
    cmd_fn!(_jog, {Command::Jog}; data: JogData; JogState);
    cmd_fn!(_move_l, {Command::MoveL}; data: MoveLData; MoveLState);
    cmd_fn!(_wait_complete, {Command::WaitComplete};; WaitCompleteState);
    cmd_fn!(_set_payload, {Command::SetPayload}; data: SetPayloadData; SetPayloadState);
    cmd_fn!(_get_payload, {Command::GetPayload};; GetPayloadState);
    cmd_fn!(_set_clsn_sensitivity, {Command::SetClsnSensitivity}; data: SetClsnSensitivityData; SetClsnSensitivityState);
    cmd_fn!(_get_clsn_sensitivity, {Command::GetClsnSensitivity};; GetClsnSensitivityState);
    cmd_fn!(_kine_forward, {Command::KineForward}; data: KineForwardData; KineForwardState);
    cmd_fn!(_kine_inverse, {Command::KineInverse}; data: KineInverseData; KineInverseState);
    cmd_fn!(_clear_error, {Command::ClearError};; ClearErrorState);
    cmd_fn!(_get_joint_pos, {Command::GetJointPos};; GetJointPosState);
    cmd_fn!(_get_tcp_pos, {Command::GetTcpPos};; GetTcpPosState);
}

impl RobotBehavior for JakaRobot {
    fn version(&self) -> String {
        JAKA_VERSION.to_string()
    }
    fn init(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn enable(&mut self) -> RobotResult<()> {
        self._power_on()?;
        self._enable()?;
        Ok(())
    }
    fn disable(&mut self) -> RobotResult<()> {
        self._disable()?;
        Ok(())
    }
    fn is_moving(&mut self) -> bool {
        self.is_moving
    }
    fn reset(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn pause(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn resume(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn shutdown(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
}

impl ArmBehavior<JAKA_DOF> for JakaRobot {
    type State = RobotState;
    fn read_state(&mut self) -> RobotResult<Self::State> {
        let state = self.robot_state.read().unwrap();
        Ok(state.clone())
    }
    fn state(&mut self) -> RobotResult<ArmState<JAKA_DOF>> {
        unimplemented!()
    }
    fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
        unimplemented!()
    }
}

impl ArmPreplannedMotion<JAKA_DOF> for JakaRobot {
    fn move_to(&mut self, _target: MotionType<JAKA_DOF>, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_to_async(&mut self, _target: MotionType<JAKA_DOF>, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_rel(&mut self, _rel: MotionType<JAKA_DOF>, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_rel_async(&mut self, _rel: MotionType<JAKA_DOF>, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_path(&mut self, _path: Vec<MotionType<JAKA_DOF>>, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn control_with(&mut self, _control: ControlType<JAKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }
}

impl ArmPreplannedMotionExt<JAKA_DOF> for JakaRobot {
    fn move_path_prepare(&mut self, _path: Vec<MotionType<JAKA_DOF>>) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_path_start(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_joint(&mut self, target: &[f64; JAKA_DOF], speed: f64) -> RobotResult<()> {
        let move_data = JointMoveData {
            joint_position: *target,
            speed,
            accel: 1.0, // 未定义参数
            relflag: 0,
        };
        self._joint_move(move_data)?;
        Ok(())
    }
    fn move_joint_async(&mut self, target: &[f64; JAKA_DOF], speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_joint_rel(&mut self, target: &[f64; JAKA_DOF], speed: f64) -> RobotResult<()> {
        let move_data = JointMoveData {
            joint_position: *target,
            speed,
            accel: 1.0, // 未定义参数
            relflag: 1,
        };
        self._joint_move(move_data)?;
        Ok(())
    }
    fn move_joint_rel_async(&mut self, target: &[f64; JAKA_DOF], speed: f64) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_cartesian(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        // 统一转换为Isometry3格式，再提取位置和欧拉角
        let iso = target.quat();
        let translation = iso.translation.vector;
        let euler = iso.rotation.euler_angles();

        let end_position = [
            translation.x,
            translation.y,
            translation.z,
            euler.0, // roll
            euler.1, // pitch
            euler.2, // yaw
        ];

        // 创建末端运动数据
        let move_data = EndMoveData {
            end_position,
            speed,
            accel: 1.0, // 未定义参数
        };
        self._end_move(move_data)?;
        Ok(())
    }
    fn move_cartesian_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_cartesian_rel(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_cartesian_rel_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
}
// impl ArmStreamingMotion for JakaRobot {}

// impl ArmStreamingMotionExt for JakaRobot {}

// impl ArmRealtimeControl for JakaRobot {}

// impl ArmRealtimeControlExt for JakaRobot {}
