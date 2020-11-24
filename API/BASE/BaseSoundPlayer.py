from abc import ABCMeta, abstractmethod


class AbstractSoundPlayer:
    __metaclass__ = ABCMeta

    @abstractmethod
    def playsound(self, audio_file):
        pass
