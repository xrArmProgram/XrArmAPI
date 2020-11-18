from threading import Thread


from BASE.BaseController import AbstractController
from setting import channels
from API.BASE import BaseSingleton4py2
from API.BASE import only_run_once


class RobotController(AbstractController, BaseSingleton4py2):
    def __init__(self, robot, channel_select_pipe):
        self.__robot = robot
        self.__channel_select_pipe = channel_select_pipe
        self.__channel = channels['channel1']

        self.__function = None
        self.__task = None
        self.__controller_is_running = False

    def build_function(self):
        # if input not a class, do nothing
        if type(self.__channel["functional_class"]).__name__ == 'classobj':
            self.__function = self.__channel["functional_class"](self.__robot)
            self.__task = Thread(target=self.__function.run())
            self.__task.start()

    def run(self):
        only_run_once(self.__controller_is_running)

        # start controller
        self.__controller_is_running = True
        while self.__controller_is_running:
            self.__main()

    def run_in_coroutine(self):
        only_run_once(self.__controller_is_running)

        # start controller
        self.__controller_is_running = True
        while self.__controller_is_running:
            self.__main()
            yield

    def __main(self):
        # get channel_select_msg from a pipe and select channel
        channel_select_msg = self.read_channel_msg_from_pipe()
        if channel_select_msg is None:
            return None

        self.channel_select(channel_select_msg)

    def stop(self):
        # stop controller
        self.__controller_is_running = False

    def read_channel_msg_from_pipe(self):
        # Non blocking read pipe.(It's actually a wait timeout)
        if self.__channel_select_pipe.poll(1):
            return self.__channel_select_pipe.recv()
        return None

    # select channel by channel dictionaries
    def channel_select(self, channel):
        if channel in channels:
            # Prevent repeat operation function
            if self.__channel is channels[channel]:
                return None

            for command in self.__channel['command']:
                if callable(command):
                    command(robot=self.__robot,
                            last_channel=self.__channel)

            self.__channel = channels[channel]
            self.build_function()

        else:
            for command in channels["channel_not_found"]['command']:
                if callable(command):
                    command(robot=self.__robot,
                            last_function=self.__function)
