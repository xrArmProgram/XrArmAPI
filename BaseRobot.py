from abc import abstractmethod, ABCMeta
from time import sleep
import threading
from collections import Iterable


class AbstractRobot:
    """Abstract class of manipulator, which defines the basic operation interface of manipulator."""
    __metaclass__ = ABCMeta

    @abstractmethod
    def init(self):
        """Return the manipulator to its original position."""
        pass

    @abstractmethod
    def update(self, angle):
        """ Modifying the pose of the manipulator.
            angle:An array of angle parameters for each joint."""
        pass

    @abstractmethod
    def read(self):
        """Read the angle array of each joint of the manipulator."""
        pass

    @abstractmethod
    def loop(self):
        """Loop handle manipulator feedback"""
        pass

    @abstractmethod
    def loop_start(self):
        """Loop the handle manipulator feedback in the background"""
        pass

    @abstractmethod
    def loop_stop(self):
        """Stop background loop"""
        pass

    @abstractmethod
    def set_pose(self, pose):
        """ Set the position of the manipulator end effector
            pose: [x,y,z]"""
        pass


class CycleRepetitionError(Exception):
    def __str__(self):
        print("Cycle Repetition Error,Do not run loop repeatedly")


class SimpleRobot(AbstractRobot):
    """A simple example class"""
    def __init__(self, node_name):
        # Initialize robot parameters
        self.__angle = [0, 0, 0, 0, 0]
        self.__node_name = node_name
        self.__pose = None
        self.__is_run = False
        self.__loop_thread = threading.Thread(target=self.loop)

        print("Robot initialization successful")

    def init(self):
        self.__angle = [0, 0, 0, 0, 0]
        print("Resetting robot now")

    def update(self, angle):
        # Data validity detection
        if not (angle, Iterable):
            print("{} is required, but the input is {}".format(type(Iterable), type(angle), ))
            raise TypeError(angle)

        try:
            for i in range(len(self.__angle)):
                self.__angle[i] = angle[i]
        except IndexError:
            # Data validity detection
            print("The robot has {} joints, but it inputs {}".format(len(self.__angle), len(angle), ))

    def read(self):
        return self.__angle

    def loop(self):
        # If the user runs the loop repeatedly, an exception is thrown
        if self.__is_run:
            raise CycleRepetitionError

        self.__is_run = True

        # Enter cycle wait
        while self.__is_run:
            try:
                print("im robot, im running.")
                sleep(0.5)

            except KeyboardInterrupt:
                # User termination procedure
                print("Program stop")
                self.__is_run = False
                break

    def loop_start(self):
        # Run loop in the background
        self.__loop_thread.start()
        print("The cycle begins")

    def loop_stop(self):
        # Stop background loop
        self.__is_run = False
        self.__loop_thread.join()
        print("Program stop")

    def set_pose(self, pose):
        print("set pose from {} to {}".format(self.__pose, pose))
        self.__pose = pose



