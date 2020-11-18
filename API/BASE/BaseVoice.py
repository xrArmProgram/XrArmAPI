from abc import ABCMeta, abstractmethod


class AbstractVoice:
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def init(self):
        pass

