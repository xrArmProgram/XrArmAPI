from time import sleep
from traceback import print_exc
from threading import Thread
import rospy

from xrarm_audio import robot_say_number
from API import PyAudioPlayer
from API.ArmBuilder import ArmBuilder
from API import ArmRobot
from simple import ObjectSorting
from API import ImgPlayer
from API import SoundSpeaker
from simple import ShapeAnalysis


# player = PyAudioPlayer()
# player.init()
#
# player.playsound(robot_say_number)
# player.stop()

class APP(object):
    def __init__(self):
        self._is_run = False

    def start(self):
        self._is_run = True

    def stop(self):
        self._is_run = False

    def is_run(self):
        return self._is_run


def task_runner(g_task, local_app):
    while local_app.is_run():
        g_task.next()
        # print("play img run {}".format(local_app.is_run()))
        sleep(0.03)


sound = PyAudioPlayer()
sound.init()

speaker = SoundSpeaker(sound.playsound)
player = ImgPlayer()
ansyc_paly = player.async_play()
app = APP()

runner_thread = Thread(target=task_runner, args=(ansyc_paly, app))


rospy.init_node("test arm node", anonymous=True, disable_signals=True)
robot = ArmRobot("test_arm_node", rospy, speaker, img_player=player)

task = ShapeAnalysis(robot, rospy)

task_run = Thread(target=task.run)


try:
    robot.res_init()

    app.start()
    robot.loop_start()
    task_run.start()
    runner_thread.start()

    rospy.spin()

except Exception as e:
    print_exc(e)

app.stop()
task.stop()
rospy.signal_shutdown("test end")
robot.loop_stop()
print("test end")
