from time import sleep

import xrarm_audio
from API.BASE import AbstractRunner


class TestSimple(AbstractRunner):
    def stop(self):
        self._is_running = False

    def run(self):
        self._is_running = True
        angle = [0.0, 0.53, -0.99, 0.0, 0.0]
        for t in range(10):
            angle[0] -= 0.1
            self._robot.update(angle)
            sleep(0.3)

        for t in range(20):
            angle[0] += 0.1
            self._robot.update(angle)
            sleep(0.3)

        # for t in range(10):
        #     angle[0] += 0.1
        #     self._robot.update(angle)
        #     sleep(0.3)

        for t in range(10):
            angle[4] += 0.1
            self._robot.update(angle)
            sleep(0.3)

        for t in range(10):
            angle[4] -= 0.1
            self._robot.update(angle)
            sleep(0.3)

    def __init__(self, robot, local_rospy):
        self.__rospy = local_rospy
        self._robot = robot
        self._is_running = False
    

