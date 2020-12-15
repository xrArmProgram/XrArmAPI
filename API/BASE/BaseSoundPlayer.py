from abc import ABCMeta, abstractmethod


class AbstractSoundPlayer(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def playsound(self, audio_file):
        pass
