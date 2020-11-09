from threading import Lock, Thread
from collections import Iterable

import rospy


from BaseRobot import AbstractRobot


lock = Lock()


class ArmRobot(AbstractRobot, object):
    """XR M1 arm robot controller"""
    __instance = None

    # Singleton
    def __new__(cls):
        with lock:
            if not cls.__instance:
                cls.__instance = super().__new__(cls, *args, **kwargs)
            return cls.__instance

    def __init__(self, node_name="xr_arm_controller"):
        rospy.init_node(name=node_name, anonymous=True)
        self.__loop_thread = Thread(target=self.loop)

    def loop(self):
        rospy.spin()

    def loop_start(self):
        self.__loop_thread.start()

    def loop_stop(self):
        rospy.signal_shutdown("user stopped controller")
        self.__loop_thread.join()

    def set_pose(self, pose):
        pass

    def update(self, angle):
        pass

    def read(self):
        pass

    def init(self):
        pass
