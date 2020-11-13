from abc import ABCMeta, abstractmethod


class AbstractVoice:
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass

    def stop(self):
        pass

