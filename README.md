# Readme

[English](README.md) | [简体中文](README_cn.md)

unofficial `rust` implementation of `libjaka`!

This library is part of the [Universal Robot Driver Project](https://github.com/Robot-Exp-Platform/robot_behavior)! We are committed to providing Rust driver support for more robotic platforms! **Unifying driver interfaces across different robot models, reducing the learning curve for robotics, and delivering more efficient robot control solutions!**

## Control Sessions And Exit

Joint-position and Cartesian-pose control implement `robot_behavior::ControlWith`.
Callbacks for `control_with_flow` / `control_with_flow_async` return
`ControlFlow<(), (Command, bool)>`: `Continue((cmd, false))` continues,
`Continue((cmd, true))` sends the final command before exiting, and `Break(())`
skips this cycle's `servo_j` / `servo_p` and runs the `servo_move(0)` exit protocol.
Both entry points block for the session; `async` describes the per-cycle callback.

The existing tuple callback APIs wrap the same loop. Cleanup failures are returned
as `RobotResult`; simultaneous execution and cleanup failures are retained in
`RobotException::ControlSession`. Enable `robot_behavior/roplat` for the shared
`ControlRhythm` adapter and hierarchical rhythm domains.

`cargo test -p libjaka --lib` includes JSON and servo-loop tests using only ephemeral
loopback ports. It does not connect to a robot. Physical stopping behavior and
125 Hz control timing still require hardware acceptance.
