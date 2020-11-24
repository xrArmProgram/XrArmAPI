from time import sleep

from pyaudio import PyAudio
import wave

from API.BASE import AbstractSoundPlayer


class PyAudioPlayer(AbstractSoundPlayer):
    def __init__(self):
        self.__CHUNK = 1024
        self.__player = None

        self.__is_open = False

    def init(self):
        self.__player = PyAudio()

    def playsound(self, audio_file):
        self.__is_open = True
        wave_flow = wave.open(audio_file, 'rb')

        stream = self.__player.open(format=self.__player.get_format_from_width(wave_flow.getsampwidth()),
                                    channels=wave_flow.getnchannels(),
                                    rate=wave_flow.getframerate(),
                                    output=True)

        wave_data = wave_flow.readframes(self.__CHUNK)

        while wave_data != '' and self.__is_open:
            stream.write(wave_data)
            wave_data = wave_flow.readframes(self.__CHUNK)

        stream.stop_stream()
        stream.close()

    def stop(self):
        self.__is_open = False
        sleep(0.1)
        self.__player.terminate()
