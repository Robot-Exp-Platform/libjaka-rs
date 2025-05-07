use libjaka_rs::JakaRobot;
use robot_behavior::{ArmPreplannedMotionExt, Pose, RobotBehavior};
use serde::Deserialize;
use std::error::Error;
use std::fs::File;
use std::{thread::sleep, time::Duration};

// 定义 CSV 数据结构
#[derive(Debug, Deserialize)]
struct Point {
    x: f64,
    y: f64,
    z: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    // 1. 初始化机器人
    let mut robot = JakaRobot::new("10.5.5.100");
    let origin = [-6.0, -298.0, 26.0]; // 基准点
    let sleep_dur = Duration::from_millis(100);
    robot.enable()?;
    robot._stop_program()?;

    // 2. 读取 CSV 文件
    let file = File::open("examples/1.csv")?;
    let mut rdr = csv::Reader::from_reader(file);

    // 3. 遍历所有坐标点
    for result in rdr.deserialize() {
        let offset: Point = result?; // 解析 CSV 行

        // 计算目标位置 = 基准点 + 偏移量
        let target = [
            origin[0] + offset.x,
            origin[1] + offset.y,
            origin[2] + offset.z,
        ];

        // 执行运动指令
        robot.move_cartesian_async(&Pose::Euler(target, [180.0, 0.0, 90.0]), 100.0)?;

        // 暂停
        sleep(sleep_dur);
    }

    Ok(())
}
