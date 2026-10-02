# Readme

[English](README.md) | [简体中文](README_cn.md)

非官方 `libjaka` 的 `Rust` 实现！

本库是[通用机器人驱动计划](https://github.com/Robot-Exp-Platform/robot_behavior)中的一员！我们立志于为更多的机器人平台提供 Rust 语言的驱动支持！**统一不同型号的机器人驱动接口，降低机器人学习成本，提供更高效的机器人控制方案！**

## 控制会话与退出

关节位置和笛卡尔位姿控制实现 `robot_behavior::ControlWith`。
`control_with_flow` / `control_with_flow_async` 的回调返回
`ControlFlow<(), (Command, bool)>`：`Continue((cmd, false))` 继续，
`Continue((cmd, true))` 发送最后命令后退出，`Break(())` 跳过本周期
`servo_j` / `servo_p`，直接执行 `servo_move(0)` 退出协议。
两个入口都是阻塞控制会话；`async` 仅描述每周期回调。

现有 tuple 回调接口继续使用同一循环；关闭失败会作为 `RobotResult` 返回。
执行和关闭同时失败时，`RobotException::ControlSession` 保留两个原因。
开启 `robot_behavior/roplat` 后可由公共 `ControlRhythm` 对接多层节律域。

`cargo test -p libjaka --lib` 包含仅连接 loopback 临时端口的 JSON/伺服循环测试；
不会连接机器人。真实停机效果及 125 Hz 控制时序仍需真机验收。
