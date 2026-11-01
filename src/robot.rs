use crate::{JAKA_FREQUENCY, JAKA_VERSION, network::NetWork, robot_impl::RobotImpl, types::*};

use roplat_behavior::{
    Arm, ArmState, CartesianPoseControl, ControlExit, ControlObservation, ControlObserver,
    ControlStep, ControlWith, EndPoint, EndPointPose, FlangeSpace, JointPositionControl,
    JointSpace, JointState, Joints, LoadState, MotionLimits, MoveTo, MoveTraj, Pose, Robot,
    RobotException, RobotResult,
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
/// The JAKA robot driver.
///
/// `T` marks the robot model and `N` is the joint count. The type implements the
/// `roplat_behavior` contracts [`Robot`], [`Arm`], [`MoveTo`], [`MoveTraj`] and
/// [`ControlWith`], turning JAKA TCP/JSON commands into the shared behavior interface.
///
/// Callers should prefer the shared entry points of `roplat_behavior::behavior::*`, such as
/// `move_to::<JointSpace<N>>()` and `control_with::<JointPositionControl<N>, _>()`.
pub struct JakaRobot<T, const N: usize> {
    pub(crate) marker: PhantomData<T>,
    pub robot_impl: RobotImpl<N>,
    pub(crate) robot_state: Arc<RwLock<RobotState>>,
    before_observers: JakaControlObservers,
    after_observers: JakaControlObservers,
    #[allow(dead_code)]
    pub(crate) streaming_handle: thread::JoinHandle<()>,
    pub(crate) is_moving: bool,
    /// The limits motion commands use; `scale` scales speed and acceleration.
    pub(crate) limits: MotionLimits<N>,
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
                TioVoutMode::V12V => SetTioVoutParamData {
                    tio_vout_ena: 1,
                    tio_vout_vol: 0,
                },
                TioVoutMode::V24V => SetTioVoutParamData {
                    tio_vout_ena: 1,
                    tio_vout_vol: 1,
                },
            },
            TioVout::Disable => SetTioVoutParamData {
                tio_vout_ena: 0,
                tio_vout_vol: 0,
            },
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

    /// The flange pose the controller reports; the identity when the read fails.
    pub(crate) fn flange_pose(&self) -> Pose {
        let mut robot_impl = self.robot_impl.clone();
        robot_impl
            ._get_data()
            .map(|data| Into::<ArmState<N>>::into(data).flange.meas.pose().unwrap())
            .unwrap_or_default()
    }

    /// Reads the flange pose from the controller, failing when the read fails. Relative motions start from it.
    pub(crate) fn read_flange_pose(&mut self) -> RobotResult<Pose> {
        let state: ArmState<N> = self.robot_impl._get_data()?.into();
        state.flange.meas.pose().ok_or_else(|| {
            RobotException::CommandException("JAKA reported no flange pose".to_string())
        })
    }
}

impl<T, const N: usize> JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N> + EndPoint,
{
    /// Connects to the JAKA controller at `ip` and creates the robot.
    ///
    /// The motion limits are the envelope the model declares, at a conservative
    /// default speed fraction of `0.05`; change them with [`Arm::set_motion_limits`].
    pub fn new(ip: &str) -> Self {
        let robot_state = NetWork::state_connect(ip);
        Self {
            marker: PhantomData,
            robot_impl: RobotImpl::new(ip),
            robot_state,
            before_observers: control_observers(),
            after_observers: control_observers(),
            streaming_handle: thread::spawn(|| {}),
            is_moving: false,
            limits: MotionLimits::envelope::<Self>().scaled(0.05),
        }
    }
}

impl<T, const N: usize> Robot for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    type State = RobotState;
    const CONTROL_PERIOD: Duration = Duration::from_nanos((1e9 / JAKA_FREQUENCY) as u64);

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
    Self: Joints<N> + EndPoint + EndPointPose,
{
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        let data = self.robot_impl._get_data()?;
        Ok(data.into())
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        let set_load_data = SetPayloadData {
            mass: load.mass,
            centroid: load.center_of_mass,
        };
        self.robot_impl._set_payload(set_load_data)?.into()
    }

    fn get_joint(&self) -> [f64; N] {
        let mut robot_impl = self.robot_impl.clone();
        robot_impl
            ._get_data()
            .map(|data| Into::<ArmState<N>>::into(data).joint.meas.q().unwrap())
            .unwrap_or([0.; N])
    }

    fn motion_limits(&self) -> MotionLimits<N> {
        self.limits
    }
    fn set_motion_limits(&mut self, limits: MotionLimits<N>) -> RobotResult<()> {
        limits.within(&MotionLimits::envelope::<Self>())?;
        self.limits = limits;
        Ok(())
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
        // A JAKA joint motion takes one speed and one acceleration: the tightest joint bound of each.
        let limits = self.limits.scaled_joint();
        let tightest = |bounds: [f64; N]| bounds.into_iter().fold(f64::INFINITY, f64::min);
        let move_data = JointMoveData::<N> {
            joint_position: target.map(f64::to_degrees),
            speed: tightest(limits.dq_max).to_degrees(),
            accel: tightest(limits.ddq_max).to_degrees(),
            relflag: 0,
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
        let limits = self.limits.scaled_cartesian();
        let move_data = MoveLData {
            cart_position: pose_to_jaka_cart(target),
            speed: limits.linear_velocity_max * 1000.0, // m/s -> mm/s
            accel: limits.linear_acceleration_max * 1000.0,
            relflag: 0,
        };
        self.run_blocking_move(|robot| robot._move_l(move_data))
    }
}

impl<T, const N: usize> MoveTraj<JointSpace<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N>,
{
    /// Follows an already sampled joint trajectory.
    ///
    /// Every sample goes out through the `JointPositionControl` servo channel. An empty
    /// trajectory is a no-op; at the end the last sample is sent again and the loop exits.
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

    /// The JAKA driver has no continuous-path sampler yet.
    ///
    /// To follow a continuous path, sample it into a trajectory first and call
    /// [`MoveTraj::move_traj`], or let Ruckig interpolate waypoints through
    /// [`MoveTraj::move_waypoints`].
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

        let mut ruckig =
            Ruckig::<N, ThrowErrorHandler>::new(None, Self::CONTROL_PERIOD.as_secs_f64());
        let mut input = InputParameter::new(None);
        let mut output = OutputParameter::new(None);

        // Ruckig treats an infinite jerk bound as no constraint.
        let limits = self.limits.scaled_joint();
        input.max_velocity = DataArrayOrVec::Stack(limits.dq_max);
        input.max_acceleration = DataArrayOrVec::Stack(limits.ddq_max);
        input.max_jerk = DataArrayOrVec::Stack(limits.dddq_max);
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

/// The `Duration` handed to the servo callback for one cycle.
///
/// The servo loop sends one command per nominal period. When a cycle overran
/// and `k` whole periods passed without a command, the next cycle reports
/// `(k + 1)` nominal periods; the first cycle reports the nominal period.
/// Summing the reported periods gives the time since the session began, in
/// whole command slots (the ruling in roplat-agents ecosystem C42, C44).
fn cycle_period(nominal: Duration, since_last: Option<Duration>) -> Duration {
    let Some(since_last) = since_last else {
        return nominal;
    };
    let slots = (since_last.as_nanos() / nominal.as_nanos()).max(1);
    nominal * u32::try_from(slots).unwrap_or(u32::MAX)
}

impl<T, const N: usize> ControlWith<JointPositionControl<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
{
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        state
            .cmd
            .q()
            .or(state.des.q())
            .or(state.meas.q())
            .unwrap_or([0.; N])
    }

    /// Blocking realtime joint-position control through the JAKA `servo_j` channel.
    ///
    /// Every cycle reads the robot state, hands its joint state to `closure` and sends
    /// the returned joint target to the controller. When the closure reports `done = true`,
    /// the cycle's command is still sent before servo mode is left.
    /// `Break` sends no command for the cycle and runs the servo exit protocol at once.
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
        let nominal = Self::CONTROL_PERIOD;

        let result = (|| -> RobotResult<()> {
            let enter: RobotResult<()> = robot._servo_move(ServoMoveData { relflag: 1 })?.into();
            enter?;
            let mut last_tick: Option<Instant> = None;
            loop {
                let tick_start = Instant::now();
                let period = cycle_period(
                    nominal,
                    last_tick.replace(tick_start).map(|t| tick_start - t),
                );
                let state: ArmState<N> = robot._get_data()?.into();
                let full_state = robot_state
                    .read()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .clone();
                notify_control_observers(&before_observers, &full_state, period);
                let step = closure(state.joint, period);
                notify_control_observers(&after_observers, &full_state, period);
                let (joint, finished) = match step {
                    ControlFlow::Continue(step) => step,
                    ControlFlow::Break(ControlExit::Stopped) => return Ok(()),
                    ControlFlow::Break(ControlExit::Failed(error)) => return Err(error),
                };

                let applied: RobotResult<()> = robot
                    ._servo_j(ServoJData::<N> {
                        joint_angles: joint.map(f64::to_degrees),
                        relflag: 0,
                    })?
                    .into();
                applied?;

                if finished {
                    return Ok(());
                }

                let elapsed = tick_start.elapsed();
                if elapsed < nominal {
                    sleep(nominal - elapsed);
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
            .pose()
            .or(state.flange.des.pose())
            .or(state.flange.meas.pose())
            .unwrap_or_default()
    }

    /// Blocking realtime Cartesian-pose control through the JAKA `servo_p` channel.
    ///
    /// Every cycle reads the full [`ArmState`] and sends the flange pose the closure returns,
    /// converted from `roplat_behavior`'s metres and radians to JAKA's millimetres and degrees.
    /// As in joint control, a `done = true` cycle sends its command before exiting.
    /// `Break` sends no command for the cycle and runs the servo exit protocol at once.
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
        let nominal = Self::CONTROL_PERIOD;

        let result = (|| -> RobotResult<()> {
            let enter: RobotResult<()> = robot._servo_move(ServoMoveData { relflag: 1 })?.into();
            enter?;
            let mut last_tick: Option<Instant> = None;
            loop {
                let tick_start = Instant::now();
                let period = cycle_period(
                    nominal,
                    last_tick.replace(tick_start).map(|t| tick_start - t),
                );
                let state: ArmState<N> = robot._get_data()?.into();
                let full_state = robot_state
                    .read()
                    .unwrap_or_else(|poisoned| poisoned.into_inner())
                    .clone();
                notify_control_observers(&before_observers, &full_state, period);
                let step = closure(state, period);
                notify_control_observers(&after_observers, &full_state, period);
                let (pose, finished) = match step {
                    ControlFlow::Continue(step) => step,
                    ControlFlow::Break(ControlExit::Stopped) => return Ok(()),
                    ControlFlow::Break(ControlExit::Failed(error)) => return Err(error),
                };

                let applied: RobotResult<()> = robot
                    ._servo_p(ServoPData {
                        cat_position: pose_to_jaka_cart(pose),
                        relflag: 0,
                    })?
                    .into();
                applied?;

                if finished {
                    return Ok(());
                }

                let elapsed = tick_start.elapsed();
                if elapsed < nominal {
                    sleep(nominal - elapsed);
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

/// Converts a `roplat_behavior` pose (metres, radians) to JAKA's Cartesian command
/// units (millimetres, degrees).
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
