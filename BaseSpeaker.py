from abc import ABCMeta, abstractmethod
from threading import Thread
from playsound import playsound


class AbstractSpeaker:
    __metaclass__ = ABCMeta

    @abstractmethod
    def speak(self, audio_file, block=True):
        pass


class PlaySoundSpeaker(AbstractSpeaker):
    def __init__(self):
        self.__audio_list = []
        self.__is_playing = False

        self.__play_thread = Thread(target=self.__play_sound)

    def speak(self, audio_file, block=True):
        if self.__is_playing:
            self.__audio_list.append(audio_file)

            if not block:
                return None

            self.__play_thread.join()
        else:
            if not block:
                self.__audio_list.append(audio_file)
                self.__play_thread.start()
                self.__is_playing = True
                return None

        playsound(audio_file)

    def __play_sound(self):
        self.__is_playing = True
        while len(self.__audio_list) > 0:
            playsound(self.__audio_list.pop())

        self.__is_playing = False
