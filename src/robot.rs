use crate::{JAKA_FREQUENCY, JAKA_VERSION, network::NetWork, robot_impl::RobotImpl, types::*};

use robot_behavior::{
    Arm, ArmState, CartesianPoseControl, ControlObservation, ControlObserver, ControlStep,
    ControlWith, Coord, EndPoint, FlangeSpace, JointPositionControl, JointSpace, JointState,
    Joints, LoadState, MoveTo, MoveTraj, OverrideOnce, Pose, Robot, RobotException, RobotResult,
    utils::rad_to_deg,
};
use rsruckig::{
    error::ThrowErrorHandler,
    prelude::{InputParameter, OutputParameter},
    result::RuckigResult,
    ruckig::Ruckig,
    util::DataArrayOrVec,
};
use serde::{Deserialize, Serialize};
use std::{
    marker::PhantomData,
    ops::ControlFlow,
    sync::{Arc, Mutex, RwLock},
    thread::{self, sleep},
    time::{Duration, Instant},
};

type JakaControlObservers = Arc<Mutex<Vec<ControlObserver<RobotState>>>>;

fn control_observers() -> JakaControlObservers {
    Arc::new(Mutex::new(Vec::new()))
}

fn notify_control_observers(
    observers: &JakaControlObservers,
    state: &RobotState,
    duration: Duration,
) {
    let mut observers = observers
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    for observer in observers.iter_mut() {
        observer(state, duration);
    }
}
/// JAKA 机器人驱动实体。
///
/// `T` 是具体机器人型号标记，`N` 是关节数量。该类型同时实现
/// [`Robot`]、[`Arm`]、[`MoveTo`]、[`MoveTraj`] 和 [`ControlWith`] 等
/// `robot_behavior` 行为特征，把 JAKA TCP/JSON 指令转换为统一的行为接口。
///
/// 外部调用应优先使用 `robot_behavior::behavior::*` 中的统一入口，例如
/// `move_to::<JointSpace<N>>()` 和 `control_with::<JointPositionControl<N>, _>()`。
pub struct JakaRobot<T, const N: usize> {
    pub(crate) marker: PhantomData<T>,
    pub robot_impl: RobotImpl<N>,
    pub(crate) robot_state: Arc<RwLock<RobotState>>,
    before_observers: JakaControlObservers,
    after_observers: JakaControlObservers,
    #[allow(dead_code)]
    pub(crate) streaming_handle: thread::JoinHandle<()>,
    pub(crate) is_moving: bool,
    pub(crate) coord: OverrideOnce<Coord>,
    pub(crate) max_vel: OverrideOnce<[f64; N]>,
    pub(crate) max_acc: OverrideOnce<[f64; N]>,
    pub(crate) max_cartesian_vel: OverrideOnce<f64>,
    pub(crate) max_cartesian_acc: OverrideOnce<f64>,
    pub(crate) max_rotation_vel: OverrideOnce<f64>,
    pub(crate) max_rotation_acc: OverrideOnce<f64>,
}

impl<T, const N: usize> JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    fn run_blocking_move<F>(&mut self, command: F) -> RobotResult<()>
    where
        F: FnOnce(&mut RobotImpl<N>) -> RobotResult<DefaultState>,
    {
        self.is_moving = true;

        command(&mut self.robot_impl)?;
        self.robot_impl._wait_complete()?;

        self.is_moving = false;
        Ok(())
    }

    pub fn set_tio_vout(&mut self, tio: TioVout) -> RobotResult<()> {
        let data = match tio {
            TioVout::Enable(mode) => match mode {
                TioVoutMode::V12V => SetTioVoutParamData { tio_vout_ena: 1, tio_vout_vol: 0 },
                TioVoutMode::V24V => SetTioVoutParamData { tio_vout_ena: 1, tio_vout_vol: 1 },
            },
            TioVout::Disable => SetTioVoutParamData { tio_vout_ena: 0, tio_vout_vol: 0 },
        };
        self.robot_impl._set_tio_vout_param(data)?.into()
    }

    pub fn get_tio_vout(&mut self) -> RobotResult<TioVout> {
        let state = self.robot_impl._get_tio_vout_param()?;
        let tio = match state.tio_vout_ena {
            0 => TioVout::Disable,
            1 => match state.tio_vout_vol {
                0 => TioVout::Enable(TioVoutMode::V12V),
                1 => TioVout::Enable(TioVoutMode::V24V),
                _ => {
                    return Err(RobotException::CommandException(
                        "Invalid TIO VOUT voltage".to_string(),
                    ));
                }
            },
            _ => {
                return Err(RobotException::CommandException(
                    "Invalid TIO VOUT enable status".to_string(),
                ));
            }
        };
        Ok(tio)
    }
}

impl<T, const N: usize> JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N> + EndPoint,
{
    /// 连接指定 IP 的 JAKA 控制器并创建机器人对象。
    ///
    /// 构造函数会初始化默认坐标系、速度/加速度限制，并使用较保守的
    /// `0.05` 缩放作为默认运动倍率。
    pub fn new(ip: &str) -> Self {
        let robot_state = NetWork::state_connect(ip);
        let mut robot = Self {
            marker: PhantomData,
            robot_impl: RobotImpl::new(ip),
            robot_state,
            before_observers: control_observers(),
            after_observers: control_observers(),
            streaming_handle: thread::spawn(|| {}),
            is_moving: false,
            coord: OverrideOnce::new(Coord::OCS),
            max_vel: OverrideOnce::new(Self::JOINT_VEL_BOUND),
            max_acc: OverrideOnce::new(Self::JOINT_ACC_BOUND),
            max_cartesian_vel: OverrideOnce::new(Self::CARTESIAN_VEL_BOUND),
            max_cartesian_acc: OverrideOnce::new(Self::CARTESIAN_ACC_BOUND),
            max_rotation_vel: OverrideOnce::new(Self::ROTATION_VEL_BOUND),
            max_rotation_acc: OverrideOnce::new(Self::ROTATION_ACC_BOUND),
        };
        let _ = robot.set_scale(0.05);
        robot
    }

    /// 设置下一次运动命令使用的坐标系。
    pub fn set_coord(&mut self, coord: Coord) {
        self.coord.set(coord);
    }

    /// 按比例缩放关节和笛卡尔空间的速度、加速度限制。
    pub fn set_scale(&mut self, scale: f64) {
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * scale);
    }
}

impl<T, const N: usize> Robot for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    type State = RobotState;
    const CONTROL_PERIOD: f64 = 1. / JAKA_FREQUENCY;

    fn version() -> String {
        format!("JAKA Robot v{JAKA_VERSION}")
    }
    fn read_state(&mut self) -> RobotResult<Self::State> {
        Ok(self.robot_state.read().unwrap().clone())
    }
    fn init(&mut self) -> RobotResult<()> {
        self.robot_impl._power_on()?.into()
    }
    fn shutdown(&mut self) -> RobotResult<()> {
        self.robot_impl._power_off()?.into()
    }
    fn enable(&mut self) -> RobotResult<()> {
        let _ = self.robot_impl._power_on()?;
        self.robot_impl._enable()?.into()
    }
    fn disable(&mut self) -> RobotResult<()> {
        self.robot_impl._disable()?.into()
    }
    fn is_moving(&mut self) -> RobotResult<bool> {
        if self.is_moving {
            self.is_moving = self.robot_impl._get_data()?.curr_tcp_trans_vel > 0.1;
        }
        Ok(self.is_moving)
    }
    fn waiting_for_finish(&mut self) -> RobotResult<()> {
        while self.is_moving()? {
            sleep(Duration::from_millis(100));
        }
        Ok(())
    }
    fn stop(&mut self) -> RobotResult<()> {
        let result: RobotResult<()> = self.robot_impl._stop_program()?.into();
        if result.is_ok() {
            self.is_moving = false;
        }
        result
    }
    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        self.robot_impl._clear_error()?.into()
    }
}

impl<T, const N: usize> ControlObservation for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    fn before<H>(&mut self, observer: H) -> &mut Self
    where
        H: FnMut(&Self::State, Duration) + Send + 'static,
    {
        self.before_observers
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(Box::new(observer));
        self
    }

    fn after<H>(&mut self, observer: H) -> &mut Self
    where
        H: FnMut(&Self::State, Duration) + Send + 'static,
    {
        self.after_observers
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(Box::new(observer));
        self
    }
}

impl<T, const N: usize> Arm<N> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N> + EndPoint,
{
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        let data = self.robot_impl._get_data()?;
        Ok(data.into())
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        let set_load_data = SetPayloadData { mass: load.m, centroid: load.x };
        self.robot_impl._set_payload(set_load_data)?.into()
    }

    fn get_joint(&self) -> [f64; N] {
        let mut robot_impl = self.robot_impl.clone();
        robot_impl
            ._get_data()
            .map(|data| Into::<ArmState<N>>::into(data).joint.meas.q.unwrap())
            .unwrap_or([0.; N])
    }
    fn get_endpoint(&self) -> Pose {
        let mut robot_impl = self.robot_impl.clone();
        robot_impl
            ._get_data()
            .map(|data| Into::<ArmState<N>>::into(data).flange.meas.pose.unwrap())
            .unwrap_or_default()
    }

    fn with_joint_vel(mut self, vel_bound: [f64; N]) -> Self {
        self.max_vel.once(vel_bound);
        self
    }
    fn with_joint_acc(mut self, acc_bound: [f64; N]) -> Self {
        self.max_acc.once(acc_bound);
        self
    }
    fn with_joint_jerk(self, _jerk_bound: [f64; N]) -> Self {
        self
    }
    fn with_torque(self, _torque_bound: [f64; N]) -> Self {
        self
    }
    fn with_torque_dot(self, _torque_dot_bound: [f64; N]) -> Self {
        self
    }

    fn with_cartesian_vel(mut self, vel_bound: f64) -> Self {
        self.max_cartesian_vel.once(vel_bound);
        self
    }
    fn with_cartesian_acc(mut self, acc_bound: f64) -> Self {
        self.max_cartesian_acc.once(acc_bound);
        self
    }
    fn with_cartesian_jerk(self, _jerk_bound: f64) -> Self {
        self
    }
    fn with_rotation_vel(mut self, vel_bound: f64) -> Self {
        self.max_rotation_vel.once(vel_bound);
        self
    }
    fn with_rotation_acc(mut self, acc_bound: f64) -> Self {
        self.max_rotation_acc.once(acc_bound);
        self
    }
    fn with_rotation_jerk(self, _jerk_bound: f64) -> Self {
        self
    }
}

impl<T, const N: usize> MoveTo<JointSpace<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N>,
{
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        let coord = self.coord.get();
        let move_data = JointMoveData::<N> {
            joint_position: rad_to_deg(target),
            speed: self.max_vel.get()[0].to_degrees(),
            accel: self.max_acc.get()[0].to_degrees(),
            relflag: u8::from(coord != Coord::OCS),
        };
        self.run_blocking_move(|robot| robot._joint_move(move_data))
    }
}

impl<T, const N: usize> MoveTo<FlangeSpace> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        let coord = self.coord.get();
        let move_data = MoveLData {
            cart_position: pose_to_jaka_cart(target),
            speed: self.max_cartesian_vel.get() * 1000.0, // m/s -> mm/s
            accel: self.max_cartesian_acc.get() * 1000.0,
            relflag: u8::from(coord != Coord::OCS),
        };
        self.run_blocking_move(|robot| robot._move_l(move_data))
    }
}

impl<T, const N: usize> MoveTraj<JointSpace<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N>,
{
    /// 跟随已经离散采样好的关节轨迹。
    ///
    /// 每个采样点会通过 `JointPositionControl` 的伺服通道发送。空轨迹
    /// 视为无操作；轨迹结束时会重复发送最后一个采样点并退出控制循环。
    fn move_traj(&mut self, traj: Vec<[f64; N]>) -> RobotResult<()> {
        if traj.is_empty() {
            return Ok(());
        }

        let last = *traj.last().unwrap();
        let mut iter = traj.into_iter();
        <Self as ControlWith<JointPositionControl<N>>>::control_with(self, move |_, _| {
            match iter.next() {
                Some(joint) => (joint, false),
                None => (last, true),
            }
        })
    }

    /// JAKA 驱动当前没有连续路径采样器。
    ///
    /// 如果需要从连续路径生成轨迹，请在上层先采样为离散轨迹后调用
    /// [`MoveTraj::move_traj`]，或使用 [`MoveTraj::move_waypoints`] 让 Ruckig
    /// 进行路点插值。
    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; N]>,
    {
        Err(RobotException::UnprocessableInstructionError(
            "JAKA has no continuous-path planner; use move_traj or move_waypoints".to_string(),
        ))
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; N]>) -> RobotResult<()> {
        let Some(first) = waypoints.first().copied() else {
            return Ok(());
        };

        let mut ruckig = Ruckig::<N, ThrowErrorHandler>::new(None, 1. / JAKA_FREQUENCY);
        let mut input = InputParameter::new(None);
        let mut output = OutputParameter::new(None);

        input.max_velocity = DataArrayOrVec::Stack(<Self as Joints<N>>::JOINT_VEL_BOUND);
        input.max_acceleration = DataArrayOrVec::Stack(<Self as Joints<N>>::JOINT_ACC_BOUND);
        input.current_position = DataArrayOrVec::Stack(first);

        let mut dense: Vec<[f64; N]> = Vec::new();
        for target in waypoints {
            input.target_position = DataArrayOrVec::Stack(target);
            loop {
                let result = ruckig.update(&input, &mut output);
                input.current_position = output.new_position.clone();
                input.current_velocity = output.new_velocity.clone();
                input.current_acceleration = output.new_acceleration.clone();
                dense.push(*output.new_position.as_array().unwrap());
                if let Ok(RuckigResult::Finished) = result {
                    break;
                }
            }
        }
        self.move_traj(dense)
    }
}

impl<T, const N: usize> ControlWith<JointPositionControl<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        state
            .cmd
            .q
            .or(state.des.q)
            .or(state.meas.q)
            .unwrap_or([0.; N])
    }

    /// 使用 JAKA `servo_j` 通道执行阻塞的关节位置实时控制。
    ///
    /// 每个周期读取机器人状态，将其中的关节状态传给 `closure`，再把
    /// 闭包返回的目标关节角发送给控制器。闭包返回 `done = true` 时，
    /// 当前周期命令仍会先下发，然后退出伺服模式。
    /// `Break(())` 则不发送本周期命令，直接执行伺服退出协议。
    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ControlStep<[f64; N]>,
    {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let robot = &mut self.robot_impl;
        let robot_state = self.robot_state.clone();
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let period = Duration::from_secs_f64(1. / JAKA_FREQUENCY);

        let result = (|| -> RobotResult<()> {
            let enter: RobotResult<()> = robot._servo_move(ServoMoveData { relflag: 1 })?.into();
            enter?;
            loop {
                let tick_start = Instant::now();
                let state: ArmState<N> = robot._get_data()?.into();
                let full_state = robot_state
                    .read()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .clone();
                notify_control_observers(&before_observers, &full_state, period);
                let step = closure(state.joint, period);
                notify_control_observers(&after_observers, &full_state, period);
                let ControlFlow::Continue((joint, finished)) = step else {
                    return Ok(());
                };

                let applied: RobotResult<()> = robot
                    ._servo_j(ServoJData::<N> { joint_angles: rad_to_deg(joint), relflag: 0 })?
                    .into();
                applied?;

                if finished {
                    return Ok(());
                }

                let elapsed = tick_start.elapsed();
                if elapsed < period {
                    sleep(period - elapsed);
                }
            }
        })();

        let stop_result: RobotResult<()> = match robot._servo_move(ServoMoveData { relflag: 0 }) {
            Ok(state) => state.into(),
            Err(err) => Err(err),
        };
        self.is_moving = false;
        match (result, stop_result) {
            (Ok(()), result) | (result, Ok(())) => result,
            (Err(primary), Err(cleanup)) => Err(RobotException::ControlSession {
                primary: Box::new(primary),
                cleanup: Box::new(cleanup),
            }),
        }
    }
}

impl<T, const N: usize> ControlWith<CartesianPoseControl<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    fn hold_command(state: &ArmState<N>) -> Pose {
        state
            .flange
            .cmd
            .pose
            .or(state.flange.des.pose)
            .or(state.flange.meas.pose)
            .unwrap_or_default()
    }

    /// 使用 JAKA `servo_p` 通道执行阻塞的笛卡尔位姿实时控制。
    ///
    /// 每个周期读取完整 [`ArmState`]，将闭包返回的法兰位姿从
    /// `robot_behavior` 约定的米/弧度转换为 JAKA 使用的毫米/角度后下发。
    /// 与关节控制一致，`done = true` 的周期仍会先发送命令再退出。
    /// `Break(())` 则不发送本周期命令，直接执行伺服退出协议。
    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<N>, Duration) -> ControlStep<Pose>,
    {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let robot = &mut self.robot_impl;
        let robot_state = self.robot_state.clone();
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let period = Duration::from_secs_f64(1. / JAKA_FREQUENCY);

        let result = (|| -> RobotResult<()> {
            let enter: RobotResult<()> = robot._servo_move(ServoMoveData { relflag: 1 })?.into();
            enter?;
            loop {
                let tick_start = Instant::now();
                let state: ArmState<N> = robot._get_data()?.into();
                let full_state = robot_state
                    .read()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .clone();
                notify_control_observers(&before_observers, &full_state, period);
                let step = closure(state, period);
                notify_control_observers(&after_observers, &full_state, period);
                let ControlFlow::Continue((pose, finished)) = step else {
                    return Ok(());
                };

                let applied: RobotResult<()> = robot
                    ._servo_p(ServoPData { cat_position: pose_to_jaka_cart(pose), relflag: 0 })?
                    .into();
                applied?;

                if finished {
                    return Ok(());
                }

                let elapsed = tick_start.elapsed();
                if elapsed < period {
                    sleep(period - elapsed);
                }
            }
        })();

        let stop_result: RobotResult<()> = match robot._servo_move(ServoMoveData { relflag: 0 }) {
            Ok(state) => state.into(),
            Err(err) => Err(err),
        };
        self.is_moving = false;
        match (result, stop_result) {
            (Ok(()), result) | (result, Ok(())) => result,
            (Err(primary), Err(cleanup)) => Err(RobotException::ControlSession {
                primary: Box::new(primary),
                cleanup: Box::new(cleanup),
            }),
        }
    }
}

/// 将 `robot_behavior` 的位姿单位（米/弧度）转换为 JAKA 笛卡尔指令单位
/// （毫米/角度）。
fn pose_to_jaka_cart(pose: Pose) -> [f64; 6] {
    let mut cart: [f64; 6] = pose.into();
    for i in 0..3 {
        cart[i] *= 1000.0; // m -> mm
        cart[i + 3] = cart[i + 3].to_degrees(); // rad -> deg
    }
    cart
}

#[cfg(test)]
#[path = "control_flow_tests.rs"]
mod control_flow_tests;
