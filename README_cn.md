# libjaka

[English](README.md) | [简体中文](README_cn.md)

`libjaka` 是 JAKA 控制器的非官方 Rust 驱动，通过 TCP/JSON 发送指令并适配 [`robot_behavior`](https://github.com/Robot-Exp-Platform/robot_behavior)，使应用能与其他 Robot-Exp 驱动共享带类型的运动和控制接口。

已发布版本为 **`libjaka 0.2.0`**，实现了控制器查询、目标运动，以及关节位置/笛卡尔位姿 servo 会话。机型别名提供参数和类型，不代表每种机型、固件组合均已验收。

## 设计与流程

`JakaRobot<T, N>` 持有控制器连接和关节数，`JakaMini2` 等别名提供关节限制与几何信息。行为 trait 描述应用操作，驱动处理 JSON 指令、响应及 servo 协议。

典型顺序是 **连接 → 查询机械臂状态 → 配置并使能控制器 → 选择目标运动或 servo 会话**。目标运动等待完成；servo 会话借用机器人，以现有的 125 Hz 循环将观测传入回调。该配置周期不等于实测实时性保证。

需要新鲜状态时使用 **`Arm::state()`**，它会发送 `GetData` 请求。当前 `Robot::read_state()` 只复制缓存，其状态流接收器未启用，不能当作新的硬件观测。因此下面的示例明确使用 `state()`。

## 安装

先运行 `cargo new jaka-read-state` 创建二进制项目，再加入：

```toml
[dependencies]
libjaka = "0.2.0"
robot_behavior = "0.6.1"
```

使用 Rust nightly，相关 crate 使用不稳定 Rust 特性。直接依赖行为库可导入 trait，并选择 0.6.1 修复。registry 安装不需要父级 `drives` 工作区，也不要求另装 JAKA C/C++ SDK。

控制器需提供兼容的 TCP/JSON 指令服务，且主机可访问 **10001** 端口。状态和运动权限取决于设备配置。本版本没有声明经过验证的主机操作系统/控制器固件矩阵。

| Feature | 用途 |
|---|---|
| 默认，无 feature | Rust 网络驱动。 |
| `debug` | 指令和响应诊断。 |
| `to_py` | PyO3 绑定，需要 Python 开发环境。 |
| `to_cxx` | CXX 绑定，需要 C++ 工具链。 |

绑定 feature 不是完整部署包。使用 Roplat 的 `ControlRhythm` 时，应在 **`robot_behavior`** 上启用 `roplat`，而不是在 `libjaka` 上启用。

## 第一个程序：查询机械臂状态

把代码放入 `src/main.rs`，决定实际运行前替换地址。程序连接并查询状态，不执行上电、使能或运动。

```rust,no_run
use libjaka::JakaMini2;
use robot_behavior::{Arm, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = JakaMini2::new("10.5.5.100");
    let state = robot.state()?;
    println!("joint position: {:?}", state.joint.meas.q);
    println!("flange pose: {:?}", state.flange.meas.pose);
    Ok(())
}
```

只检查编译，不建立连接：

```sh
cargo +nightly check
```

当前 `new()` 在初次连接失败时会 panic，后续查询返回 `RobotResult`。统一状态中的 `Option` 字段区分已有观测与未提供的量。使用控制器原始数据时应查阅驱动转换实现：JSON 协议与统一运动接口的表示方式并不完全相同。

## 运动与 servo 会话退出

实际控制器配置好并准备运动后，可查看[生命周期示例](examples/00_00_lifecycle.rs)中的使能/失能调用，以及[关节目标运动](examples/02_00_move_joint_default.rs)中的 `move_to::<JointSpace<6>>()`。这些程序会发送指令，应作为状态和机型配置确认之后的下一步。

反馈控制可查看[关节位置](examples/04_00_control_joint_position.rs)或[笛卡尔位姿](examples/04_04_control_cartesian_pose.rs)。Flow 回调返回 `ControlFlow<(), (Command, bool)>`：

- `Continue((command, false))` 发送并继续。
- `Continue((command, true))` 发送最后一条指令并结束。
- `Break(())` 跳过当前周期的 `servo_j`/`servo_p`，执行 `servo_move(0)` 退出协议。

`control_with_flow` 与 `control_with_flow_async` 均阻塞整次会话。后者允许异步回调，并非原生非阻塞设备会话 Future。元组回调方法包装同一循环。执行和清理错误可通过 `RobotException::ControlSession` 一并保留。这些软件退出语义不代表控制器物理停止行为已经验证。

## 下一步与限制

- [示例目录](examples)包含轨迹、笛卡尔运动、观测、模型和 I/O；旧的 `read_state` 缓存示例应结合前述区别阅读。
- [机器人实现](src/robot.rs)、[设备指令](src/robot_impl.rs)与[已发布 API](https://docs.rs/libjaka/0.2.0/libjaka/)说明已支持的能力。
- [行为库文档](https://github.com/Robot-Exp-Platform/robot_behavior)说明通用 trait 与 Roplat 集成。
- [Loopback 测试](src/control_flow_tests.rs)在无机器人条件下检查 JSON/servo 循环，不验证设备时序或固件覆盖。

本版本实现了关节位置和笛卡尔位姿控制，但没有原生 `AsyncControlWith` 会话。构造器错误处理及未启用的状态流缓存仍是限制。

## 源码、资源与许可

checkout manifest 将开发依赖固定到 Git revision，需要源码访问权限；registry 用户不需要相邻工作区。模型资源不包含在已发布 crate 中，显式获取方式见[资源模块](src/assets.rs)。检查和 CI 可设置 `ROPLAT_SKIP_ASSET_EXPORT=1` 跳过构建时的可选资源导出。

Rust 驱动由 Robot-Exp-Platform 维护，使用 [Apache-2.0](LICENSE)。JAKA 是设备厂商名称，本项目是非官方实现。另行获取的模型资源和厂商软件保留各自归属与条款。
