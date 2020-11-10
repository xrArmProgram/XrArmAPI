import time

from BaseRobot import SimpleRobot
from ArmRobot import ArmRobot
import xrarm_audio

robot = SimpleRobot("test_robot_node")
# robot = ArmRobot("test_robot_node")

robot.loop_start()

try:
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

    robot.speak(xrarm_audio.naruto, block=False)
    robot.speak(xrarm_audio.glory_of_Kings, block=False)

    pose = [0, 0, 0]
    robot.set_pose(pose)

    print("waiting for end")
    time.sleep(3)

except Exception as e:
    print(e)

robot.loop_stop()
