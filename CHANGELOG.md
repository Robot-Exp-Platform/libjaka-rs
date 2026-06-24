# Change log (更新日志)

## 0.2.0

- Align `ControlWith<JointPositionControl<N>>` with current `robot_behavior` semantics: scoped, blocking controller closures and per-cycle command dispatch before `done` exits the loop.
- Align `move_traj` with the sibling Franka driver by treating an empty trajectory as a no-op and holding the final joint sample when completing the stream.
- Update crate and runtime JAKA driver version metadata to `0.2.0`.
