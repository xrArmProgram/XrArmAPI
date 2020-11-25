from abc import ABCMeta, abstractmethod


class AbstractApp(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def is_running(self):
        pass


class AbstractRunner:
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass
    
    @abstractmethod
    def stop(self):
        pass


class BaseApp(AbstractApp):
    def __init__(self):
        self.__app_is_running = False

    def start(self):
        self.__app_is_running = True

    def stop(self):
        self.__app_is_running = False

    def is_running(self):
        return self.__app_is_running


class OperationRepetitionError(Exception):
    def __str__(self):
        print("Operation Repetition Error,Do not Operation this function repeatedly")


def only_run_once(running_flag):
    if running_flag:
        raise

