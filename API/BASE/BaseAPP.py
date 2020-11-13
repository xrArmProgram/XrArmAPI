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


class BaseApp(AbstractApp):
    def __init__(self):
        self.__app_is_running = False

    def start(self):
        self.__app_is_running = True

    def stop(self):
        self.__app_is_running = False

    def is_running(self):
        return self.__app_is_running