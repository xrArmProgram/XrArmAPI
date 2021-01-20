from json import loads
from threading import Thread
from time import sleep, time

import xrarm_audio
from BASE.BaseController import AbstractController
from setting import channels, sys_channel
from API.BASE import BaseSingleton4py2, AbstractRunner
from API.BASE import only_run_once
from setting.channel2function import select_channel, custom_channel, exit_channel


class RobotController(AbstractController, BaseSingleton4py2):
    def __init__(self, local_rospy, robot, channel_select_pipe, img_player=None):
        self.__robot = robot
        self.__rospy = local_rospy
        self.__channel_select_pipe = channel_select_pipe
        self.__channel = channels['default_channel']

        self.__function = None
        self.__task = None
        self.__controller_is_running = False
        self.__in_custom_function_select = False
        self.__run_time_flag = time()
        self.__img_player = img_player

    def __build_function(self):
        # if input not a class, do nothing
        local_function_class = self.__channel["functional_class"]
        if local_function_class is not None and issubclass(local_function_class, AbstractRunner):
            self.__function = local_function_class(self.__robot, self.__rospy)
            self.__task = Thread(target=self.__function.run)
            self.__task.start()

    def run(self):
        only_run_once(self.__controller_is_running)

        # start controller
        self.__controller_is_running = True
        img_player_not_stop = True

        while self.__controller_is_running or img_player_not_stop:
            self.__main()
            try:
                self.__img_player.next()

            except StopIteration:
                img_player_not_stop = False
                print("\033[33mImg player has been turned off.No more images can be displayed",)

    def run_iterable(self):
        only_run_once(self.__controller_is_running)

        # start controller
        self.__controller_is_running = True
        img_player_not_stop = True

        while self.__controller_is_running or img_player_not_stop:
            self.__main()

            try:
                self.__img_player.next()

            except StopIteration:
                img_player_not_stop = False
                print("\033[33mImg player has been turned off.No more images can be displayed",)

            yield

    def __main(self):
        # get channel_select_msg from a pipe and select channel
        if not self.__in_custom_function_select:
            self.__run_time_flag = time()
        else:
            if time() - self.__run_time_flag > 15:
                self.__in_custom_function_select = False

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
        if self.__channel_select_pipe.poll(0.01):
            data = loads(self.__channel_select_pipe.recv())
            if "voice_mode" in data:
                return data["voice_mode"]
        return None

    # select channel by channel dictionaries
    def __channel_select(self, channel):
        if not self.__in_custom_function_select:
            if channel not in sys_channel:
                return None

        if channel in channels:
            # Prevent repeat operation function
            if self.__channel is channels[channel]:
                return None

            for command in channels[channel]['command']:
                if callable(command):
                    command(local_rospy=self.__rospy,
                            robot=self.__robot,
                            last_function=self.__function)

            if channel == select_channel:
                self.__in_custom_function_select = True

            elif channel == exit_channel:
                self.__channel = channels['default_channel']

            if channel in sys_channel:
                return None

            self.__channel = channels[channel]
            self.__build_function()
            self.__in_custom_function_select = False

        else:
            for command in channels["channel_not_found"]['command']:
                if callable(command):
                    command(local_rospy=self.__rospy,
                            robot=self.__robot,
                            last_function=self.__function)
