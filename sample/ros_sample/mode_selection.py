from os import system

from std_msgs.msg import UInt8
from abc import abstractmethod

import xrarm_audio
from API.BASE import AbstractRunner


class BaseModeSelection(AbstractRunner):
    def __init__(self, robot, local_rospy):
        self.__rospy = local_rospy
        self.__mode_pub = local_rospy.Publisher("/mode_state", UInt8, queue_size=1)
        self.msg = UInt8()

        self._robot = robot

    def stop(self):
        self.msg.data = 0
        self.pub()

    def pub(self):
        self.__mode_pub.publish(self.msg)
    print("published")

    @abstractmethod
    def run(self):
        pass


class LearningMode(BaseModeSelection):
    def run(self):
        self.msg.data = 1
        self.pub()
        self._robot.speak(xrarm_audio.start_learning)

    def stop(self):
        self.msg.data = 2
        self.pub()


class ActionMode(BaseModeSelection):
    def run(self):
        self._robot.speak(xrarm_audio.run_learned_actions)
        self.msg.data = 3
        self.pub()


class BasicControlMode(BaseModeSelection):
    def run(self):
        self._robot.speak(xrarm_audio.control_model)
        self.msg.data = 0
        self.pub()


class VisualGrabbingMode(BaseModeSelection):
    def run(self):
        self._robot.speak(xrarm_audio.start_visual_capture)
        self.msg.data = 5
        self.pub()
        system("gnome-terminal -x bash -i -c 'roslaunch xr_capture xr_capture.launch;'")

