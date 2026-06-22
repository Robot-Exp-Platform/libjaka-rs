use nalgebra as na;
use std::{f64::consts::PI, sync::Arc};

use libjaka::JakaMini2;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100").with_cartesian_vel(1.0);
    robot.enable()?;

    // 先移动到曲线起点附近（平移单位：米，姿态单位：弧度）。
    robot.move_to_sync::<FlangeSpace>(Pose::Euler([0.3, 0.0, 0.03], [-PI, 0.0, PI]))?;

    // 生成圆锥螺旋线轨迹（内部以毫米 / 角度表示）。
    let (_length, curve) = cone_spiral_curve([300.0, 0.0, 30.0], 60.0, 3, 0.3, 0.3);

    // 均匀采样曲线，并以法兰直线段串流跟随该笛卡尔轨迹。
    const SAMPLES: usize = 200;
    for i in 0..=SAMPLES {
        let progress = i as f64 / SAMPLES as f64;
        robot.move_to_sync::<FlangeSpace>(to_si(curve(progress)))?;
    }

    Ok(())
}

/// 把曲线生成的“毫米 + 角度”位姿转换为 robot_behavior 约定的“米 + 弧度”。
fn to_si(pose: Pose) -> Pose {
    let Pose::Euler(tran, rot) = pose else {
        return pose;
    };
    Pose::Euler(tran.map(|v| v / 1000.0), rot.map(f64::to_radians))
}

// 这个函数是设计用于生成一个圆锥螺旋线的轨迹，返回一个按进展返回运动的闭包和一个最大距离
fn cone_spiral_curve(
    vertex: [f64; 3],
    h: f64,
    loops: usize,
    theta: f64,
    alpha: f64,
) -> (f64, Arc<dyn Fn(f64) -> Pose + Send + Sync>) {
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
fn compute_point(t: f64, vertex: [f64; 3], h: f64, loops: usize, r_base: f64, alpha: f64) -> Pose {
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
    Pose::Euler([x, y, z], euler_deg.into())
}
