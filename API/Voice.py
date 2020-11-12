import serial
from time import sleep
import select
from multiprocessing import Pipe

from BASE.BaseVoice import VOICE_MOD_SET, RECV_LEN, AbstractVoice


class Voice(AbstractVoice):
    def __init__(self, comes, board=9600, local_pipe=None):
        if not local_pipe:
            print("local_pipe is need but not given")
            raise ValueError

        self.__ser = serial.Serial(comes, board)
        self.__VOICE_MOD = None
        self.__recv_pipe, self.__send_pipe = local_pipe
        self.__read_list = []
        self.__write_list = []
        self.__data_is_update = False
        self.__KEEP_RUNNING = True

    def run(self):
        self.__read_list.append(self.__ser)
        self.__read_list.append(self.__recv_pipe)

        self.__write_list.append(self.__send_pipe)

        while self.__KEEP_RUNNING:
            read_table, write_table, error_table = select.select(
                self.__read_list,
                self.__write_list,
                [],
                __timeout=1
            )

            # handle read list
            for read in read_table:
                if read is self.__recv_pipe:
                    pass

                if read is self.__ser:
                    sleep(0.05)
                    n = read.inWaiting()
                    my_out = read.read(n)

                    result = self.__get_voice(my_out)
                    if result is None:
                        continue

                    self.__data_is_update = True

            # handle write list
            for write in write_table:
                if write is self.__send_pipe:
                    pass

    def stop(self):
        self.__KEEP_RUNNING = False

    @staticmethod
    def __get_voice(data):
        if len(data) < RECV_LEN:
            return None

        if data[0] == 0xff and data[len(data) - 1] == 0xff:
            buf = []
            for i in range(1, 4):
                buf.append(data[i])

            if buf[0] == 0xf5 and buf[1] == 0x01:
                if buf[2] in VOICE_MOD_SET:
                    return VOICE_MOD_SET[buf[2]]

        return None
