from json import loads
from threading import Thread
from time import sleep

from BASE.BaseController import AbstractController
from setting import channels, sys_channel
from API.BASE import BaseSingleton4py2
from API.BASE import only_run_once


class RobotController(AbstractController, BaseSingleton4py2):
    def __init__(self, local_rospy, robot, channel_select_pipe):
        self.__robot = robot
        self.__rospy = local_rospy
        self.__channel_select_pipe = channel_select_pipe
        self.__channel = channels['channel1']

        self.__function = None
        self.__task = None
        self.__controller_is_running = False

    def __build_function(self):
        # if input not a class, do nothing
        if type(self.__channel["functional_class"]).__name__ == 'type':
            self.__function = self.__channel["functional_class"](self.__robot)
            self.__task = Thread(target=self.__function.run)
            self.__task.start()

    def run(self):
        only_run_once(self.__controller_is_running)

        # start controller
        self.__controller_is_running = True
        while self.__controller_is_running:
            self.__main()

    def run_iterable(self):
        only_run_once(self.__controller_is_running)

        # start controller
        self.__controller_is_running = True
        while self.__controller_is_running:
            self.__main()
            yield

    def __main(self):
        # get channel_select_msg from a pipe and select channel
        channel_select_msg = self.__read_channel_msg_from_pipe()
        if channel_select_msg is None:
            return None

        self.__channel_select(channel_select_msg)

    def stop(self):
        # stop controller
        self.__controller_is_running = False
        if self.__function is not None:
            self.__function.stop()

    def __read_channel_msg_from_pipe(self):
        # Non blocking read pipe.(It's actually a wait timeout)
        if self.__channel_select_pipe.poll(1):
            sleep(0.05)
            data = loads(self.__channel_select_pipe.recv())
            if "voice_mode" in data:
                return data["voice_mode"]
        return None

    # select channel by channel dictionaries
    def __channel_select(self, channel):
        # print(channel)

        if channel in channels:
            # Prevent repeat operation function
            if self.__channel is channels[channel]:
                return None

            for command in channels[channel]['command']:
                # print(command)
                # print(self.__function)

                if callable(command):
                    command(local_rospy=self.__rospy,
                            robot=self.__robot,
                            last_function=self.__function)

            if channel in sys_channel:
                return None

            self.__channel = channels[channel]
            self.__build_function()

        else:
            for command in channels["channel_not_found"]['command']:
                if callable(command):
                    command(local_rospy=self.__rospy,
                            robot=self.__robot,
                            last_function=self.__function)
