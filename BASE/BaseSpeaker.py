from abc import ABCMeta, abstractmethod


class AbstractSpeaker:
    __metaclass__ = ABCMeta

    @abstractmethod
    def speak(self, audio_file, block=True):
        pass


