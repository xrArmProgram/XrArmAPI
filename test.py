from multiprocessing import Pipe
from threading import Thread
import rospy
from time import sleep, time
from traceback import print_exc
from math import pi

from API.BASE import BaseApp, SimpleRobot
from API import Voice, ArmRobot, PlaySoundSpeaker
import xrarm_audio


def pipe_io(local_app, master_pipe):
    while local_app.is_running():
        if master_pipe.poll(1):
            print("recv: {}".format(master_pipe.recv()))
            master_pipe.send("data got")


# robot = SimpleRobot("test_robot_node")
speaker = PlaySoundSpeaker()
robot = ArmRobot("test_robot_node", local_rospy=rospy, speaker=speaker)

try:
    robot.res_init()
    robot.loop_start()

    robot.init()

    print("In the fist time, angle = {} in {}".format(robot.read(), time()))
    sleep(5)

    angel = [0.0, 0.0, 0.0, 0.0, 1.0]
    robot.update(angel)

    print("In the second time, angle = {} in {}".format(robot.read(), time()))
    sleep(5)

    # angel = [1, 1, 1, 1, 1]
    # robot.update(angel)
    # print("In the third time, angle = {} in {}".format(robot.read(), time()))
    # sleep(3)

    # robot.speak(xrarm_audio.naruto, block=False)
    # robot.speak(xrarm_audio.naruto, block=False)
    # robot.speak(xrarm_audio.glory_of_Kings, block=False)
    # robot.speak(xrarm_audio.robot_say_number, block=False)
    pose = [0.1, 0.1, 0.1]
    robot.set_pose(pose)
    sleep(10)

    print("waiting for end")
    # sleep(3)
except Exception as e:
    print_exc(e)

# app = BaseApp()
# master_conn, slave_conn = Pipe()
# comes = "/dev/ttyUSB0"
# board = 9600
# app.start()
# recv_thread = Thread(target=pipe_io, args=(app, master_conn))
# recv_thread.start()
# voice = Voice(comes, board, slave_conn)
#
# try:
#     voice.init()
#     print("start app")
#     voice.run()
#
# except KeyboardInterrupt:
#     print("app stop")
#     voice.stop()
#
# except Exception as e:
#     print_exc(e)

# app.stop()
# recv_thread.join()
# robot.speak(xrarm_audio.naruto)
robot.loop_stop()



