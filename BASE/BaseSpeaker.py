from abc import ABCMeta, abstractmethod
from threading import Thread, Lock
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
        self.lock = Lock()

        # self.__play_thread = Thread(target=self.__play_sound)
        self.__play_thread = None

    def speak(self, audio_file, block=True):
        # Reading __is_playing is not allowed when __is_playing is modified
        self.lock.acquire()

        # When audio is playing in the background, add to the play list
        if self.__is_playing:
            self.__audio_list.append(audio_file)

            # Non blocking execution
            if not block:
                self.lock.release()
                return None

            # Blocking execution
            self.lock.release()
            self.__play_thread.join()
            return None

        else:
            # When audio is not playing in the background, add to the play list
            #  and start the __play_thread
            if not block:
                self.__audio_list.append(audio_file)
                self.__play_thread = Thread(target=self.__play_sound)
                self.__is_playing = True
                self.__play_thread.start()
                self.lock.release()
                return None

        # Blocking execution
        playsound(audio_file)
        self.lock.release()

    def __play_sound(self):
        self.__is_playing = True

        # play all sound in audio list
        while len(self.__audio_list) > 0:
            playsound(self.__audio_list.pop(0))

            # Reading __is_playing is not allowed when __is_playing is modified
            with self.lock:
                if len(self.__audio_list) == 0:
                    self.__is_playing = False
