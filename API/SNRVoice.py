import serial
from time import sleep
import select
from json import dumps

from API.BASE import AbstractVoice
from setting import VOICE_MOD_SET, RECV_LEN, SNR8051_CONFIG


class SNRVoice(AbstractVoice):
    def __init__(self, comes, board=9600, slave_conn=None):
        if not slave_conn:
            print("local_pipe is need but not given")
            raise ValueError

        self.__slave_conn = slave_conn

        self.__VOICE_MOD = None
        self.__back_msg = None

        self.__ser = None
        self.__comes = comes
        self.__board = board

        self.__read_list = []
        self.__write_list = []

        self.__data_is_update = False
        self.__KEEP_RUNNING = True

    def init(self):
        self.__ser = serial.Serial(self.__comes, self.__board)

    # start to recv data from serial
    def run(self):
        if self.__ser is None:
            raise RuntimeError("You should call init before calling run.")

        self.__read_list.append(self.__ser)
        self.__read_list.append(self.__slave_conn)
        self.__write_list.append(self.__slave_conn)

        while self.__KEEP_RUNNING:
            # waiting for select event,timeout 1s later
            read_table, write_table, error_table = select.select(self.__read_list, self.__write_list, [], 1)

            # handle read list
            for read in read_table:
                # handle recv_pipe
                if read is self.__slave_conn:
                    self.__back_msg = read.recv()

                # handle serial
                if read is self.__ser:
                    # read data from serial
                    sleep(0.05)
                    n = read.inWaiting()
                    temp_str = read.read(n)
                    voice_out = []
                    # trans data to hex_str
                    for i in range(len(temp_str)):
                        hex_char = hex(ord(temp_str[i]))
                        if len(hex_char) == 3:
                            hex_char = "0x0" + hex_char[-1]

                        voice_out.append(hex_char)
                    # print(my_out)

                    # channel select
                    result = self.__get_voice(voice_out)
                    if result is None:
                        continue

                    # update VOICE_MOD
                    self.__VOICE_MOD = result
                    self.__data_is_update = True

            # handle write list
            for write in write_table:
                # handle send_pipe
                if write is self.__slave_conn:
                    if self.__data_is_update:
                        data = {"voice_mode": self.__VOICE_MOD}
                        write.send(dumps(data))
                        self.__data_is_update = False
                        self.__back_msg = None

            # print("selected")

    # stop voice
    def stop(self):
        self.__KEEP_RUNNING = False

    # translate data from serial
    @staticmethod
    def __get_voice(data):
        if len(data) < SNR8051_CONFIG["RECV_LEN"]:
            return None

        if data[0] == SNR8051_CONFIG["head"] and data[len(data) - 1] == SNR8051_CONFIG["tail"]:
            is_verify = True
            for index, value in SNR8051_CONFIG["special"]:
                if data[index-1] != value:
                    is_verify = False

            if is_verify:
                mode_flag = data[SNR8051_CONFIG["data_pose"]-1]

                if mode_flag in VOICE_MOD_SET:
                    return VOICE_MOD_SET[mode_flag]

        return None
