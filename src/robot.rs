use crate::{
    JAKA_DOF, JAKA_FREQUENCY, JAKA_ROBOT_MAX_CARTESIAN_VEL, JAKA_ROBOT_MAX_JOINT_VEL, JAKA_VERSION,
    network::NetWork, types::*,
};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotion, ArmPreplannedMotionExt, ArmRealtimeControl,
    ArmRealtimeControlExt, ArmState, ArmStreamingHandle, ArmStreamingMotion, ControlType,
    LoadState, MotionType, Pose, RobotBehavior, RobotException, RobotResult, utils::combine_array,
};
use std::{
    sync::{Arc, Mutex, RwLock},
    thread::sleep,
    time::Duration,
};

/// # Jaja Robot (节卡机器人)
pub struct JakaRobot {
    network: NetWork,
    robot_state: Arc<RwLock<RobotState>>,
    is_moving: bool,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty; $ret_type:ty) => {
        pub fn $fn_name(&mut self, arg: $arg_type) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> =
                self.network
                    .send_and_recv(Request::<$command, $arg_type>::from(arg))?;
            Ok(response.state)
        }
    };
    ($fn_name:ident, $command:expr;; $ret_type:ty) => {
        pub fn $fn_name(&mut self) -> RobotResult<$ret_type> {
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

    pub fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        let rapid_rate_data = RapidRateData { rate_value: speed };
        self._rapid_rate(rapid_rate_data)?;
        Ok(())
    }
}

impl RobotBehavior for JakaRobot {
    type State = RobotState;
    fn version(&self) -> String {
        JAKA_VERSION.to_string()
    }
    fn init(&mut self) -> RobotResult<()> {
        self._power_on()?.into()
    }
    fn shutdown(&mut self) -> RobotResult<()> {
        self._power_off()?.into()
    }
    fn enable(&mut self) -> RobotResult<()> {
        let _ = self._power_on()?;
        self._enable()?.into()
    }
    fn disable(&mut self) -> RobotResult<()> {
        self._disable()?.into()
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
    fn stop(&mut self) -> RobotResult<()> {
        self._stop_program()?.into()
    }
    fn emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        self._clear_error()?.into()
    }
    fn read_state(&mut self) -> RobotResult<Self::State> {
        let state = self.robot_state.read().unwrap();
        Ok(state.clone())
    }
}

impl ArmBehavior<JAKA_DOF> for JakaRobot {
    fn state(&mut self) -> RobotResult<ArmState<JAKA_DOF>> {
        let data = self._get_data()?;
        let joint: [f64; JAKA_DOF] = data.joint_actual_position[..6].try_into().unwrap();
        let pose_o_to_ee = Pose::Euler(
            data.actual_position[0..3].try_into().unwrap(),
            data.actual_position[3..6].try_into().unwrap(),
        );
        let arm_state = ArmState {
            joint: Some(joint),
            joint_vel: None,
            joint_acc: None,
            pose_o_to_ee: Some(pose_o_to_ee),
            pose_ee_to_k: None,
            pose_f_to_ee: None,
            cartesian_vel: None,
            load: None,
            tau: None,
        };
        Ok(arm_state)
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        let set_load_data = SetPayloadData {
            mass: load.m,
            centroid: load.x,
        };
        self._set_payload(set_load_data)?.into()
    }
}

impl ArmPreplannedMotion<JAKA_DOF> for JakaRobot {
    fn move_to(&mut self, target: MotionType<JAKA_DOF>, speed: f64) -> RobotResult<()> {
        match target {
            MotionType::Joint(joint) => self.move_joint(&joint, speed)?,
            MotionType::Cartesian(pose) => self.move_cartesian(&pose, speed)?,
            _ => unimplemented!(),
        };
        Ok(())
    }
    fn move_to_async(&mut self, target: MotionType<JAKA_DOF>, speed: f64) -> RobotResult<()> {
        match target {
            MotionType::Joint(joint) => self.move_joint_async(&joint, speed)?,
            MotionType::Cartesian(pose) => self.move_cartesian_async(&pose, speed)?,
            _ => unimplemented!(),
        };
        Ok(())
    }
    fn move_rel(&mut self, rel: MotionType<JAKA_DOF>, speed: f64) -> RobotResult<()> {
        match rel {
            MotionType::Joint(joint) => self.move_joint_rel(&joint, speed)?,
            _ => unimplemented!(),
        };
        Ok(())
    }
    fn move_rel_async(&mut self, rel: MotionType<JAKA_DOF>, speed: f64) -> RobotResult<()> {
        match rel {
            MotionType::Joint(joint) => self.move_joint_rel_async(&joint, speed)?,
            MotionType::Cartesian(pose) => self.move_cartesian_rel_async(&pose, speed)?,
            _ => unimplemented!(),
        };
        Ok(())
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
    fn move_joint(&mut self, _target: &[f64; JAKA_DOF], _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_joint_async(&mut self, target: &[f64; JAKA_DOF], speed: f64) -> RobotResult<()> {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let speed = speed * JAKA_ROBOT_MAX_JOINT_VEL[0];

        let move_data = JointMoveData {
            joint_position: *target,
            speed,
            accel: 1.0, // 未定义参数
            relflag: 0,
        };
        self._joint_move(move_data)?;

        self.is_moving = false;
        Ok(())
    }

    fn move_joint_rel(&mut self, _target: &[f64; JAKA_DOF], _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_joint_rel_async(&mut self, target: &[f64; JAKA_DOF], speed: f64) -> RobotResult<()> {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let speed = speed * JAKA_ROBOT_MAX_JOINT_VEL[0];

        let move_data = JointMoveData {
            joint_position: *target,
            speed,
            accel: 1.0,
            relflag: 1,
        };
        self._joint_move(move_data)?;

        self.is_moving = false;
        Ok(())
    }

    fn move_cartesian(&mut self, _target: &Pose, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_cartesian_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let speed = speed * JAKA_ROBOT_MAX_CARTESIAN_VEL;

        let (tran, rot) = target.euler();
        let move_data = MoveLData {
            cart_position: combine_array(&tran, &rot),
            speed,
            accel: 1.0,
            relflag: 0,
        };
        self._move_l(move_data)?;

        self.is_moving = false;
        Ok(())
    }
    fn move_cartesian_rel(&mut self, _target: &Pose, _speed: f64) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_cartesian_rel_async(&mut self, target: &Pose, speed: f64) -> RobotResult<()> {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let speed = speed * JAKA_ROBOT_MAX_CARTESIAN_VEL;

        let (tran, rot) = target.euler();
        let move_data = MoveLData {
            cart_position: combine_array(&tran, &rot),
            speed,
            accel: 1.0,
            relflag: 0,
        };
        self._move_l(move_data)?;

        self.is_moving = false;
        Ok(())
    }
}

pub struct JakaStreamingHandle {
    motion: Arc<Mutex<Option<MotionType<JAKA_DOF>>>>,
}

impl ArmStreamingHandle<JAKA_DOF> for JakaStreamingHandle {
    fn move_to(&mut self, target: MotionType<JAKA_DOF>) -> RobotResult<()> {
        *self.motion.lock().unwrap() = Some(target);
        Ok(())
    }
    fn last_motion(&self) -> RobotResult<MotionType<JAKA_DOF>> {
        unimplemented!()
    }
    fn control_with(&mut self, _control: ControlType<JAKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }
    fn last_control(&self) -> RobotResult<ControlType<JAKA_DOF>> {
        unimplemented!()
    }
}

impl ArmStreamingMotion<JAKA_DOF> for JakaRobot {
    type Handle = JakaStreamingHandle;

    fn start_streaming(&mut self) -> RobotResult<Self::Handle> {
        self._servo_move(ServoMoveData { relflag: 1 })?;

        Ok(JakaStreamingHandle {
            motion: Arc::new(Mutex::new(None)),
        })
    }

    fn end_streaming(&mut self) -> RobotResult<()> {
        self._servo_move(ServoMoveData { relflag: 0 })?;
        Ok(())
    }

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<JAKA_DOF>>>> {
        unimplemented!()
    }

    fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<JAKA_DOF>>>> {
        unimplemented!()
    }
}

// impl ArmStreamingMotionExt for JakaRobot {}

impl ArmRealtimeControl<JAKA_DOF> for JakaRobot {
    fn move_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<JAKA_DOF>, std::time::Duration) -> (MotionType<JAKA_DOF>, bool)
            + Send
            + 'static,
    {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;
        self._servo_move(ServoMoveData { relflag: 1 })?;

        loop {
            let start_time = std::time::Instant::now();
            let state = self.state()?;

            let (motion, finished) =
                closure(state.clone(), Duration::from_secs_f64(1. / JAKA_FREQUENCY));

            if finished {
                break;
            }

            match motion {
                MotionType::Joint(joint) => {
                    let data = ServoJData {
                        joint_angles: joint,
                        relflag: 0,
                    };
                    self._servo_j(data)?;
                }
                MotionType::Cartesian(pose) => {
                    let (tran, rot) = pose.euler();
                    let data = ServoPData {
                        cat_position: combine_array(&tran, &rot),
                        relflag: 0,
                    };
                    self._servo_p(data)?;
                }
                _ => {
                    return Err(RobotException::CommandException(
                        "Invalid motion type".to_string(),
                    ));
                }
            }

            sleep(Duration::from_secs_f64(1. / JAKA_FREQUENCY) - start_time.elapsed());
        }

        self._servo_move(ServoMoveData { relflag: 1 })?;
        self.is_moving = false;
        Ok(())
    }
    fn control_with_closure<FC>(&mut self, mut _closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<JAKA_DOF>, std::time::Duration) -> (ControlType<JAKA_DOF>, bool)
            + Send
            + 'static,
    {
        unimplemented!()
    }
}

impl ArmRealtimeControlExt<JAKA_DOF> for JakaRobot {}
