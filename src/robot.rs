use crate::{JAKA_FREQUENCY, JAKA_VERSION, network::NetWork, robot_impl::RobotImpl, types::*};

use robot_behavior::{
    Arm, ArmState, ControlWith, Coord, EndPoint, FlangeSpace, JointPositionControl, JointSpace,
    JointState, Joints, LoadState, MoveTo, MoveTraj, OverrideOnce, Pose, Robot, RobotException,
    RobotResult, utils::rad_to_deg,
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
    sync::{Arc, RwLock},
    thread::{self, sleep},
    time::{Duration, Instant},
};
/// # JAKA 闁哄牆鎼▍鎺撶閻氬绀勯柤鍝勫€稿畷閬嶅嫉閸濆嫭鐝ゅù婊呭皑缁?///
/// 婵炲绋戦悗閿嬨仚閸楃偛袟闁挎稒鐡猅` 濞戞挸鎼悗鐑藉矗闁垮鍨奸悹浣稿簻缁辨紮N` 濞戞挸鎼崣褔鎳為崒婵嗘闁汇垹宕€规娊濡撮崒娑氭Ж濞戞搩浜滈崣鎸庢媴閹惧磭鈧兘宕ｉ悜瑙ｅ亾濮樺磭绠栧☉鎾虫惈閸欏墽鐚剧拠鑼偓鐑藉礆椤愩垺鍊?/// 闁挎稑鐗嗛々?`JakaZu5 = JakaRobot<_JakaZu5, 6>`闁挎稑顦悿鍕偝?[`Joints`]闁靛棔绠穈EndPoint`]闁?
/// [`RobotDescription`]闁靛棔绠穈ArmForwardKinematics`]闁挎稑鏈俊鎼佸礄閸濆嫬鑼冮柛娆忓€归弳鐔哥閵夈儳鍩楅梺鎻掔箰閿涙劙寮版惔鈥虫瘔闁哄鍎荤槐閬嶅箥閳ь剟寮?
/// 閻炴稑濂旂拹?trait闁挎稑婧俙Robot`]闁靛棔绠穈Arm`]闁靛棔绠穈MoveTo`]闁靛棔绠穈MoveTraj`]闁靛棔绠穈ControlWith`]闁?
/// 闁汇垼椴稿﹢鎵尵鐠囪尙鈧绱掗悢鍓侇伇婵炲绋戦悗椋庘偓鍦仧楠炲洭濡?
pub struct JakaRobot<T, const N: usize> {
    pub(crate) marker: PhantomData<T>,
    pub robot_impl: RobotImpl<N>,
    pub(crate) robot_state: Arc<RwLock<RobotState>>,
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
    /// 濞寸姰鍎崇划鎵偓?IP 闁革附婢樺鍐礆濞戞绱?`JakaRobot` 閻庡湱鍋樼欢銉╁Υ?
    pub fn new(ip: &str) -> Self {
        let robot_state = NetWork::state_connect(ip);
        let mut robot = Self {
            marker: PhantomData,
            robot_impl: RobotImpl::new(ip),
            robot_state,
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

    /// 閻犱礁澧介悿鍡涘矗閸屾績鍋撻崘銊︾稄闁哄秴娲ㄩ柈鎾晬閸喎鐦☉鏂挎噽閺佹捇寮崼顒傜闁?
    pub fn set_coord(&mut self, coord: Coord) {
        self.coord.set(coord);
    }

    /// 闁圭顦伴惁顔界瑹鐎ｎ剛绱氶柡鈧幆褍褰犻柤?缂佹绋戝畷杈╀焊閺冨牃鍋撻悢宄邦唺濞戞挸楠告慨鐐烘焻閻斿嘲顔婂☉鎾筹躬濡炬椽鏁嶉崼鐔风槷濞戞柨鎳愰弫鎾诲极閸剛绀嗛柕?
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
            self.is_moving = self.robot_impl._get_data().unwrap().curr_tcp_trans_vel > 0.1;
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
        self.robot_impl._stop_program()?.into()
    }
    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        self.robot_impl._clear_error()?.into()
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
        self.is_moving = true;

        let coord = self.coord.get();
        let move_data = JointMoveData::<N> {
            joint_position: rad_to_deg(target),
            speed: self.max_vel.get()[0].to_degrees(),
            accel: self.max_acc.get()[0].to_degrees(),
            relflag: u8::from(coord != Coord::OCS),
        };
        self.robot_impl._joint_move(move_data)?;
        Ok(())
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
        self.is_moving = true;

        let mut pose: [f64; 6] = target.into();
        for i in 0..3 {
            pose[i] *= 1000.0; // m -> mm
            pose[i + 3] = pose[i + 3].to_degrees();
        }

        let coord = self.coord.get();
        let move_data = MoveLData {
            cart_position: pose,
            speed: self.max_cartesian_vel.get() * 1000.0, // m/s -> mm/s
            accel: self.max_cartesian_acc.get() * 1000.0,
            relflag: u8::from(coord != Coord::OCS),
        };
        self.robot_impl._move_l(move_data)?;
        self.is_moving = false;
        Ok(())
    }
}

impl<T, const N: usize> MoveTraj<JointSpace<N>> for JakaRobot<T, N>
where
    [f64; N]: Serialize + for<'a> Deserialize<'a>,
    Self: Joints<N>,
{
    /// 閻犺櫣鍠栧▓銏＄▔閳ь剟寮堕垾鍐插殥缂佸濮撮惁鎴︽煂閸ャ劎澹夐柣銊ュ閸櫻囨嚍閸屾繂缂撻弶鈺冩缁变即鏌呴幇顒佸櫙闁哄牏鍠嶇粭鍛村矗閹搭垳绀夊璺虹Ф閺併倗鈧湱鍋炲鍌涘閻戞ɑ绠涢柛銉у仩閻箖濡?
    fn move_traj(&mut self, traj: Vec<[f64; N]>) -> RobotResult<()> {
        let mut iter = traj.into_iter();
        <Self as ControlWith<JointPositionControl<N>>>::control_with(self, move |state, _| {
            match iter.next() {
                Some(joint) => (joint, false),
                None => (
                    <Self as ControlWith<JointPositionControl<N>>>::hold_command(&state),
                    true,
                ),
            }
        })
    }

    /// JAKA 濡炵懓宕慨鈺呭嫉椤忓嫬鏁剁紓鍐惧枦缁绘稓绱掗锛勭唴鐎垫澘瀚～澶愬礆閹烘垶鐝ら柨娑欒壘缂嶅﹥绋夐埀顒勫礌閺嶎剛鐔呯€?`s 闁?target` 缂傚倸鎼惃顖炲籍閸洘锛?闂侇偆鍠庣€规娊宕洪崫鍕珯闁?
    /// 濞寸姾顔婄紞宥夊炊閸濆嫮鏆伴梺鎻掓处閻楅亶鎮抽崶顒€鍘撮柡鍕靛灥閸ｎ垶鏌呴悩顔瑰亾閸屾繍鍤為柡鈧崷顓熸殢 [`MoveTraj::move_traj`] 濞戞挸顑呰ぐ鍌氼啅閺屻儱娅氶柡宥囨焿瀵ょ儤娼婚惂鍝ョ闁瑰瓨鐗滈弫?    /// [`MoveTraj::move_waypoints`] 閻?Ruckig 闁革负鍔戝娲嵁閸涱剛鐟撻柟缁樺笩钘熼柕?
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

    /// 闁革负鍔庣€氼厾绮╃€ｎ剙娈犵紒瀣儎缁楀倹娼婚幇顖ｆ斀閻庡湱鍋炲鍌涘閻戞ɑ绠涢柛銉у仩閻箖鏁嶅宕囩闁稿繈鍎板閬嶅嫉瀹ュ枺浣割嚕?闁?闂侇偅鍔曢幊鍡涘嫉閻旀椿鍤㈤柣妯垮煐閳ь兛闄嶉埀顑挎祰閻ㄧ喖鎮?`closure`
    /// 婵懓鍊风粭鍛▔閳ь剟宕楃€圭姴螡闁圭娲ｉ幎銈夌嵁閺堢數鐟撻柛?闁?闁衡偓鐠哄搫鐓傞悗鐟版湰閸ㄦ岸寮介崶褏绠堕柛姘叄閳ь兘鍋撻柛鎴犲皑缁辨繈鐛捄渚綏缂備礁鐗撻埀顑藉亾闁告垼妗ㄥ閬嶅嫉瀹ュ枺浣割嚕韫囧簼绨板ǎ鍥ㄧ箚閻﹀鈧懓顦崣蹇涘Υ?
    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        if self.is_moving {
            return Err(RobotException::CommandException(
                "Robot is moving".to_string(),
            ));
        }
        self.is_moving = true;

        let mut robot = self.robot_impl.clone();
        let period = Duration::from_secs_f64(1. / JAKA_FREQUENCY);

        self.streaming_handle = thread::Builder::new()
            .name("jaka-servo-motion".to_string())
            .spawn(move || {
                let result = (|| -> RobotResult<()> {
                    let enter: RobotResult<()> =
                        robot._servo_move(ServoMoveData { relflag: 1 })?.into();
                    enter?;
                    loop {
                        let tick_start = Instant::now();
                        let state: ArmState<N> = robot._get_data()?.into();
                        let (joint, finished) = closure(state.joint, period);
                        if finished {
                            return Ok(());
                        }

                        let applied: RobotResult<()> = robot
                            ._servo_j(ServoJData::<N> {
                                joint_angles: rad_to_deg(joint),
                                relflag: 0,
                            })?
                            .into();
                        applied?;

                        let elapsed = tick_start.elapsed();
                        if elapsed < period {
                            sleep(period - elapsed);
                        }
                    }
                })();

                if let Err(err) = result {
                    eprintln!("JAKA realtime servo loop exited with error: {err}");
                }
                let _ = robot._servo_move(ServoMoveData { relflag: 0 });
            })
            .map_err(|err| RobotException::RealtimeException(err.to_string()))?;

        Ok(())
    }
}
