from abc import ABCMeta, abstractmethod


class AbstractVoice:
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass

    def stop(self):
        pass


VOICE_MOD_SET = {
    0x01: 0,
    0x02: 1,
    0x03: 2,
    0x04: 3,
    0x05: 4,
    0x06: 5,
    0x07: 6,
    0x08: 7,
    0x09: 8,
    0x0A: 9,
}

RECV_LEN = 5
