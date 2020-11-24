from multiprocessing import Pipe
from threading import Thread
import rospy
from time import sleep, time
from traceback import print_exc
from math import pi

from API.BASE import BaseApp, SimpleRobot
from API import SNRVoice, ArmRobot, SoundSpeaker
import xrarm_audio
from API import RobotController, ArmBuilder
from sample import DetectorColor


def pipe_io(local_app, master_pipe):
    while local_app.is_running():
        if master_pipe.poll(1):
            print("recv: {}".format(master_pipe.recv()))
            master_pipe.send("data got")


# robot = SimpleRobot("test_robot_node")
speaker = SoundSpeaker()
robot = ArmRobot("test_robot_node", local_rospy=rospy, speaker=speaker)
#
# master_conn, slave_conn = Pipe()

# comes = "/dev/ttyUSB0"
# board = 9600
# voice = SNRVoice(comes, board, slave_conn)
# controller = RobotController(robot, master_conn)
# voice_thread = Thread(target=voice.run)
#
# try:
#     print("start test")
#     robot.res_init()
#     robot.loop_start()
#
#     voice.init()
#     voice_thread.start()
#
#     print("generate controller_iterator")
#     controller_iterator = controller.run_in_coroutine()
#     while True:
#         try:
#             controller_iterator.next()
#             # print("running")
#
#         except KeyboardInterrupt:
#             print("stop")
#             controller.stop()
#
#         except StopIteration:
#             print("break")
#             voice.stop()
#             voice_thread.join()
#             break
#
#     # robot.init()
#     #
#     # print("In the fist time, angle = {} in {}".format(robot.read(), time()))
#     # sleep(5)
#     #
#     # angel = [0.0, 0.0, 0.0, 0.0, 1.0]
#     # robot.update(angel)
#     #
#     # print("In the second time, angle = {} in {}".format(robot.read(), time()))
#     # sleep(5)
#
#     # angel = [1, 1, 1, 1, 1]
#     # robot.update(angel)
#     # print("In the third time, angle = {} in {}".format(robot.read(), time()))
#     # sleep(3)
#
#     # robot.speak(xrarm_audio.naruto, block=False)
#     # robot.speak(xrarm_audio.naruto, block=False)
#     # robot.speak(xrarm_audio.glory_of_Kings, block=False)
#     # robot.speak(xrarm_audio.robot_say_number, block=False)
#     # pose = [0.1, 0.1, 0.1]
#     # robot.set_pose(pose)
#     # sleep(10)
#
#     print("waiting for end")
#     # sleep(3)
# except Exception as e:
#     print_exc(e)
#
# robot.loop_stop()

color_find = DetectorColor(robot)
color_find.run()




