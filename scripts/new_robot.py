import libjaka.libjaka


pose = libjaka.Pose.Euler([1,2,3],[2,1,3])
print(pose.quat())

robot = libjaka.JakaRobot("10.5.5.100")

robot.move_joint([0, 0, 0, 0, 0, 0], 1)
robot.move_cartesian_int(pose, 1)