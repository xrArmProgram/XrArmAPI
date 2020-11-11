from time import sleep, time

from API.ArmRobot import ArmRobot
import xrarm_audio

# robot = SimpleRobot("test_robot_node")
robot = ArmRobot("test_robot_node")

robot.loop_start()

try:
    robot.init()
    sleep(3)

    print("In the fist time, angle = {} in {}".format(robot.read(), time()))

    angel = [10, 10, 10, 10, 10]
    robot.update(angel)
    sleep(3)
    print("In the second time, angle = {} in {}".format(robot.read(), time()))

    robot.speak(xrarm_audio.naruto, block=False)
    robot.speak(xrarm_audio.glory_of_Kings, block=False)
    robot.speak(xrarm_audio.robot_say_number)

    print("waiting for end")
    sleep(3)

except KeyboardInterrupt as e:
    print(e)

robot.speak(xrarm_audio.naruto)
robot.loop_stop()
