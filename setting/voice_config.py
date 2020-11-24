"""some config for voice"""


VOICE_MOD_SET = {
    '0x01': "active_channel",
    '0x02': "channel2",
    '0x03': "channel3",
    '0x04': "channel4",
    '0x05': "channel5",
    '0x06': "exit_function",
    '0x07': "channel7",
    '0x08': "channel8",
    '0x09': "channel9",
    '0x0a': "channel10",
    '0x0b': "channel_select",
    '0x0c': "channel12",
    '0x0d': "channel13",
    '0x0e': "channel14",
    '0x0f': "channel15",
    '0x10': "channel16",
    '0x11': "channel17",
    '0x12': "channel18",
    '0x13': "channel19",
    '0x14': "channel20",
    '0xff': "step_back",
}

RECV_LEN = 4

SNR_PROTOCOL_CONFIG = {
    "RECV_LEN": 4,
    "head": '0xf4',
    'tail': '0xff',
    'data_pose': 3,
    'special': [(2, '0x06'), ],
}
