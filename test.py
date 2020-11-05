import time


from baseRobot import BaseRobot


robot = BaseRobot("test_robot_node")

robot.loop_start()

robot.init()
time.sleep(3)

print("In the fist time, angle = {} in {}".format(robot.read(), time.time()))

angel = [10, 10, 10, 10, 10]
robot.update(angel)
time.sleep(3)
print("In the second time, angle = {} in {}".format(robot.read(), time.time()))

pose = [10, 10, 10]
robot.set_pose(pose)
time.sleep(3)

robot.loop_stop()

