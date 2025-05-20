use nalgebra as na;
use std::{f64::consts::PI, sync::Arc, time::Duration};

use libjaka_rs::{JAKA_DOF, JakaRobot};
use robot_behavior::{
    ArmBehavior, ArmPreplannedMotionImpl, ArmRealtimeControl, MotionType, Pose, RobotBehavior,
    RobotResult,
};
use std::thread::sleep;

fn main() -> RobotResult<()> {
    let mut robot = JakaRobot::new("10.5.5.100");
    robot.enable()?;

    robot
        .with_cartesian_velocity(100.0)
        .move_cartesian_async(&Pose::Euler([300.0, 0.0, 30.0], [-180.0, 0.0, 180.0]))?;
    sleep(Duration::from_secs(2));
    // 根据预设参数做曲线规划
    let (d, curve) = cone_spiral_curve([300.0, 0.0, 30.0], 60.0, 3, 0.3, 0.3);

    // 使用笛卡尔速度上限做时间规划，需要确保 d 是笛卡尔空间内的总距离 ，最小时间为 t_min , 我不知道笛卡尔空间加速度是多少，先给了个 1.0 试试水
    let (t_min, f_t) = simple_4th_curve(1., 10. / d, 8. / d);
    let mut t = Duration::from_secs(0);
    let t_min = Duration::from_secs_f64(t_min);
    let closure = move |_, period| {
        t += period;
        (curve(f_t(t)), t > t_min)
    };

    robot.move_with_closure(closure)?;

    Ok(())
}

// 这个函数是设计用于生成一个圆锥螺旋线的轨迹，返回一个按进展返回运动的闭包和一个最大距离
fn cone_spiral_curve(
    vertex: [f64; 3],
    h: f64,
    loops: usize,
    theta: f64,
    alpha: f64,
) -> (f64, Arc<dyn Fn(f64) -> MotionType<JAKA_DOF> + Send + Sync>) {
    let r_base = h * theta.tan();
    let n = loops as f64;
    let sin_theta = theta.sin();

    // 解析计算总长度
    let k = 4.0 * PI.powi(2) * n.powi(2) * sin_theta.powi(2);
    let total_length = if k < 1e-6 {
        h / theta.cos() // 直线情况
    } else {
        let sqrt_k = k.sqrt();
        let sqrt_1_plus_k = (1.0 + k).sqrt();
        h / theta.cos() * (0.5 * sqrt_1_plus_k + 0.5 / sqrt_k * (sqrt_k + sqrt_1_plus_k).ln())
    };

    let closure = Arc::new(move |s: f64| {
        let t = s.clamp(0.0, 1.0);
        // --------------------------------------
        // let x = t.sqrt(); // 反解x = sqrt(t)
        // compute_point(x, vertex, h, loops, r_base, alpha)
        // --------------------------------------
        compute_point(t, vertex, h, loops, r_base, alpha)
    });

    (total_length, closure)
}

// gpt 写的函数，计算当前点的坐标和姿态
fn compute_point(
    t: f64,
    vertex: [f64; 3],
    h: f64,
    loops: usize,
    r_base: f64,
    alpha: f64,
) -> MotionType<JAKA_DOF> {
    // 位置计算
    let radius = r_base * t;
    let angle = 2.0 * PI * loops as f64 * t;
    let x = vertex[0] + radius * angle.cos();
    let y = vertex[1] + radius * angle.sin();
    let z = vertex[2] + h * t;

    // 方向计算
    let radial = na::Vector3::new(vertex[0] - x, vertex[1] - y, 0.0);
    let radial = radial.try_normalize(1e-6).unwrap_or(na::Vector3::zeros());

    let z_tool = (na::Vector3::new(0.0, 0.0, -1.0) * alpha.cos()) + (radial * alpha.sin());
    let z_tool = z_tool.normalize();

    // 坐标系构建
    let up = if z_tool.z.abs() < 0.99 {
        na::Vector3::y()
    } else {
        na::Vector3::x()
    };

    let x_dir = up.cross(&z_tool).normalize();
    let y_dir = z_tool.cross(&x_dir).normalize();
    let rot =
        na::Rotation3::from_matrix_unchecked(na::Matrix3::from_columns(&[x_dir, y_dir, z_tool]));

    // 欧拉角转换 这里获得的是弧度
    // let euler = rot.euler_angles();
    // let euler_deg = euler.map(|r| r.to_degrees()); // 转换为角度制

    // MotionType::Cartesian(Pose::Euler([x, y, z], euler_deg.into())) // 使用角度制发送
    let (roll, pitch, yaw) = rot.euler_angles();
    let euler_deg = (roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees());
    println!(
        "{},{},{},{},{},{}",
        x,
        y,
        z,
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees()
    );
    MotionType::Cartesian(Pose::Euler([x, y, z], euler_deg.into()))
}

// 这个函数用于生成四阶平滑曲线，在变量分离后作为时间函数使用
fn simple_4th_curve(
    delta: f64,
    v_max: f64,
    a_max: f64,
) -> (f64, Arc<dyn Fn(Duration) -> f64 + Send + Sync>) {
    if delta < 1e-6 {
        return (0., Arc::new(|_| 0.));
    }
    let mut v_max = v_max;
    if delta < 1.5 * v_max.powi(2) / a_max {
        v_max = (2. / 3. * delta * a_max).sqrt();
    }

    let t1 = 1.5 * v_max / a_max;
    let t_min = t1 + delta / v_max;

    let f = move |t: Duration| {
        let t = t.as_secs_f64();
        if t < t1 {
            (t / t1).powi(3) * (t1 - 0.5 * t) * v_max
        } else if t < t_min - t1 {
            t1 * v_max / 2. + (t - t1) * v_max
        } else if t < t_min {
            delta - ((t_min - t) / t1).powi(3) * (t1 - 0.5 * (t_min - t)) * v_max
        } else {
            delta
        }
    };

    (t_min, Arc::new(f))
}
