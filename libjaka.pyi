from robot_behavior import (
    Arm,
    ArmState,
    FlangeMotion,
    JointMotion,
    JointPositionControl,
    JointSample,
    JointState,
    LoadState,
    MotionType,
    Pose,
    SpatialSample,
    SpatialState,
    Vec,
)


class JakaMini2(Arm, JointMotion, FlangeMotion, JointPositionControl):
    def __init__(self, ip: str) -> None: ...
