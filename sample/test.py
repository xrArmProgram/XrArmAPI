from multiprocessing import Pipe
from threading import Thread
import rospy

from API.BASE import BaseApp, SimpleRobot
from API import Voice, ArmRobot, PlaySoundSpeaker


def pipe_io(local_app, master_pipe):
    while local_app.is_running():
        if master_pipe.poll(1):
            print("recv: {}".format(master_pipe.recv()))
            master_pipe.send("data got")


# robot = SimpleRobot("test_robot_node")
robot = ArmRobot("test_robot_node", local_rospy=rospy, speaker=PlaySoundSpeaker)

# robot.loop_start()

# robot.init()
# sleep(3)
#
# print("In the fist time, angle = {} in {}".format(robot.read(), time()))
#
# angel = [10, 10, 10, 10, 10]
# robot.update(angel)
# sleep(3)
# print("In the second time, angle = {} in {}".format(robot.read(), time()))

# robot.speak(xrarm_audio.naruto, block=False)
# robot.speak(xrarm_audio.glory_of_Kings, block=False)
# robot.speak(xrarm_audio.robot_say_number)

# print("waiting for end")
# sleep(3)

app = BaseApp()
master_conn, slave_conn = Pipe()
comes = "/dev/ttyUSB0"
board = 9600
app.start()
recv_thread = Thread(target=pipe_io, args=(app, master_conn))
recv_thread.start()
voice = Voice(comes, board, slave_conn)

# try:
#     voice.run()
# except KeyboardInterrupt:
#     voice.stop()
try:
    voice.run()
except KeyboardInterrupt:
    voice.stop()

app.stop()
recv_thread.join()
# robot.speak(xrarm_audio.naruto)
# robot.loop_stop()
