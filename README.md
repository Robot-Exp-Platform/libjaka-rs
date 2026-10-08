# libjaka

[English](README.md) | [简体中文](README_cn.md)

`libjaka` is an unofficial Rust driver for JAKA controllers. It sends TCP/JSON commands and adapts them to [`robot_behavior`](https://github.com/Robot-Exp-Platform/robot_behavior), letting applications share typed motion and control interfaces with other Robot-Exp drivers.

The published crate is **`libjaka 0.2.0`**. It implements controller queries, target motion, and joint-position/Cartesian-pose servo sessions. Model aliases supply parameters and types; their presence does not certify every model/firmware combination.

## Design and workflow

`JakaRobot<T, N>` holds the controller connection and joint count. A model alias such as `JakaMini2` supplies joint limits and geometry. Behavior traits describe the requested operation, while this driver handles JSON commands, replies, and the servo protocol.

The typical sequence is **connect → query arm state → configure and enable the controller → choose target motion or a servo session**. Target motion waits for completion. Servo sessions borrow the robot and feed observations to a callback using the existing 125 Hz loop; that configured period is not a measured real-time guarantee.

For fresh state, use **`Arm::state()`**, which sends a `GetData` request. Currently, `Robot::read_state()` only clones a cache whose state-stream receiver is inactive. Do not interpret that cache as a new hardware observation. The example below deliberately uses `state()`.

## Install

Create a binary project with `cargo new jaka-read-state`, then add:

```toml
[dependencies]
libjaka = "0.2.0"
robot_behavior = "0.6.1"
```

Use Rust nightly: the crates use unstable Rust features. The direct behavior dependency provides the imported traits and selects the 0.6.1 fixes. Registry installation needs neither the parent `drives` workspace nor a separate JAKA C/C++ SDK.

The controller must expose the compatible TCP/JSON command service on port **10001**, reachable from the host. State and motion permissions depend on its configuration. This release does not declare a verified host OS/controller firmware matrix.

| Feature | Purpose |
|---|---|
| Default, no features | Rust network driver. |
| `debug` | Command and response diagnostics. |
| `to_py` | PyO3 bindings; requires a Python development environment. |
| `to_cxx` | CXX bindings; requires a C++ toolchain. |

Binding features are not complete deployment packages. To use Roplat's `ControlRhythm`, enable `roplat` on **`robot_behavior`**, not on `libjaka`.

## First program: query arm state

Put this in `src/main.rs`. Replace the address before deliberately running it. It connects and queries state without powering on, enabling, or moving the robot.

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

Compile without opening a connection:

```sh
cargo +nightly check
```

`new()` currently panics if the initial connection fails; later queries return `RobotResult`. The common state's `Option` fields distinguish supplied measurements from unavailable quantities. Consult the driver's conversions when using raw controller data: the JSON protocol and common motion interface do not have identical representations.

## Motion and servo-session exit

Once the actual controller is configured for motion, [the lifecycle example](examples/00_00_lifecycle.rs) shows enable/disable calls and [joint target motion](examples/02_00_move_joint_default.rs) shows `move_to::<JointSpace<6>>()`. These are command-sending programs, a next step after confirming state and model configuration.

For feedback control, see [joint position](examples/04_00_control_joint_position.rs) or [Cartesian pose](examples/04_04_control_cartesian_pose.rs). Flow callbacks return `ControlFlow<(), (Command, bool)>`:

- `Continue((command, false))` sends and continues.
- `Continue((command, true))` sends the final command and completes.
- `Break(())` skips that cycle's `servo_j`/`servo_p` and runs the `servo_move(0)` exit protocol.

Both `control_with_flow` and `control_with_flow_async` block for the complete session. The latter permits an async callback, not a native nonblocking device-session future. Tuple callback methods wrap the same loop. Execution and cleanup errors can be retained together in `RobotException::ControlSession`. These software exits do not establish a controller's physical stopping behavior.

## Next steps and limits

- [Examples](examples) cover trajectories, Cartesian motion, observations, models, and I/O. Read older cache-based `read_state` examples with the distinction above in mind.
- [Robot implementation](src/robot.rs), [device commands](src/robot_impl.rs), and [published API](https://docs.rs/libjaka/0.2.0/libjaka/) describe supported capabilities.
- [Behavior documentation](https://github.com/Robot-Exp-Platform/robot_behavior) covers reusable traits and Roplat integration.
- [Loopback tests](src/control_flow_tests.rs) check JSON/servo loops without a robot, not device timing or firmware coverage.

This version implements joint-position and Cartesian-pose control, but no native `AsyncControlWith` session. Constructor error handling and the inactive state-stream cache remain limitations.

## Source builds, assets, and license

Checkout manifests pin development dependencies to Git revisions and require source access; registry users do not need a sibling workspace. Model assets are excluded from the published crate; see [the asset module](src/assets.rs) for explicit retrieval. `ROPLAT_SKIP_ASSET_EXPORT=1` skips optional build-time asset export during checks and CI.

The Rust driver is maintained by Robot-Exp-Platform under [Apache-2.0](LICENSE). JAKA is the manufacturer's name; this project is unofficial. Separately obtained model assets and vendor software retain their attribution and terms.
