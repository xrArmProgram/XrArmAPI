from abc import ABCMeta, abstractmethod


class AbstractController:
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def stop(self):
        pass
