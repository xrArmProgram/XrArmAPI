from multiprocessing import Pipe
from threading import Thread

import rospy
from API.BASE import OperationRepetitionError, NotBuiltError

from BASE import AbstractBuilder


class ArmBuilder(AbstractBuilder):
    def __init__(self, controller_class, robot_class, speaker_class, voice_class, voice_dev, voice_board):
        self.__robot_class = robot_class
        self.__speaker_class = speaker_class
        self.__voice_class = voice_class
        self.__voice_dev = voice_dev
        self.__voice_board = voice_board
        self.__controller_class = controller_class

        self.__robot = None
        self.__voice = None
        self.__speaker = None
        self.__controller = None

        self.__ros_spin_thread = None
        self.__voice_run_thread = None

        self.__is_Built = False

    def build(self):
        if self.__is_Built:
            raise OperationRepetitionError

        self.__speaker = self.__speaker_class()

        # Initialize ros node
        rospy.init_node(name="robot_controller",
                        anonymous=True,
                        disable_signals=True)
        self.__ros_spin_thread = Thread(target=rospy.spin)

        # robot API
        self.__robot = self.__robot_class(local_rospy=rospy, speaker=self.__speaker)
        self.__robot.res_init()

        # create pipe
        master_pipe, slave_pipe = Pipe(duplex=True)

        # voice API
        self.__voice = self.__voice_class(self.__voice_dev, self.__voice_board, slave_pipe)
        self.__voice.init()
        self.__voice_run_thread = Thread(target=self.__voice.run)

        # controller API
        self.__controller = self.__controller_class(local_rospy=rospy, robot=self.__robot, channel_select_pipe=master_pipe)

        self.__is_Built = True

    def __main(self):
        if not self.__is_Built:
            raise NotBuiltError

        self.__ros_spin_thread.start()
        self.__voice_run_thread.start()

    def run(self):
        self.__main()
        self.__controller.run()

    def run_iterable(self):
        self.__main()
        return self.__controller.run_iterable()

    def destroy(self):
        print("start destroy")
        self.__controller.stop()
        self.__robot.loop_stop()
        self.__voice.stop()
        print("stop dev")
        if not rospy.is_shutdown():
            rospy.signal_shutdown("user stop it")
            print("stop ros")

        self.__voice_run_thread.join(timeout=3)
        self.__ros_spin_thread.join(timeout=3)
        print("destroy success")
