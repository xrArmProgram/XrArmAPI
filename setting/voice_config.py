"""some config for voice"""


VOICE_MOD_SET = {
    '0x01': "active_channel",
    '0x02': "control_model",
    '0x03': "learning_model",
    '0x04': "operation_action",
    '0x05': "visual_grabbing",
    '0x06': "exit_function",
    '0x07': "color_recognition",
    '0x08': "shape_recognition",
    '0x09': "face_following",
    '0x0a': "sorting_mode",
    '0x0b': "channel_select",
    '0x0c': "channel12",
    '0x0d': "channel13",
    '0x0e': "channel14",
    '0x0f': "channel15",
    '0x10': "channel16",
    '0x11': "channel17",
    '0x12': "channel18",
    '0x13': "channel19",
    '0x19': "channel20",
    '0x00': "step_back",
}

SNR_PROTOCOL_CONFIG = {
    "RECV_LEN": 5,
    "head": ['0xff', '0xf5'],
    'tail': ['0xff'],
    'data_pose': 4,
    'special': [(3, '0x01'), ],
}
