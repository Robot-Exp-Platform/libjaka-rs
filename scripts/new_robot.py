import libjaka.libjaka


pose = libjaka.Pose.Euler([1,2,3],[2,1,3])
print(pose.quat())

robot = libjaka.JakaRobot("172.169.10.2")

robot.move_joint_rel([0, 0, 0, 0, 0, 0])

robot.move_cartesian(pose)