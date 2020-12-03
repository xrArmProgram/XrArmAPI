from os import system

import rospy
from std_msgs.msg import UInt8
from abc import abstractmethod

from API.BASE import AbstractRunner


class BaseModeSelection(AbstractRunner):
    def __init__(self, robot, local_rospy):
        self.__rospy = rospy
        self.__mode_pub = rospy.Publisher("/mode_state", UInt8)
        self.__msg = UInt8()

    def stop(self):
        self.__msg.data = 0
        self.__pub()

    def __pub(self):
        self.__mode_pub.publish(self.__msg)

    @abstractmethod
    def run(self):
        pass


class LearningMode(BaseModeSelection):
    def run(self):
        self.__msg.data = 2
        self.__pub()


class ActionMode(BaseModeSelection):
    def run(self):
        self.__msg = 3
        self.__pub()


class BasicControlMode(BaseModeSelection):
    def run(self):
        self.__msg = 1
        self.__pub()


class VisualGrabbingMode(BaseModeSelection):
    def run(self):
        self.__msg = 5
        self.__pub()
        system("roslaunch xr_capture xr_capture.launch")
