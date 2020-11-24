from setting.conroller_commands import controller_commands

from sample import DetectorColor

channels = {
    "default_channel": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "control_model": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "learning_model": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "operation_action": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "visual_grabbing": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "color_recognition": {
        "functional_class": DetectorColor,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "shape_recognition": {
        "functional_class": None,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "face_following": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "sorting_mode": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },

    "exit_function": {
        "functional_class": None,
        "command": [controller_commands['exit_function']],
    },
    "channel_select": {
        "functional_class": None,
        "command": [controller_commands['channel_select']],
    },
    "active_channel": {
        "functional_class": None,
        "command": [controller_commands['active'], ],
    },
    "step_back": {
        "functional_class": None,
        "command": [controller_commands['back2background']],
    },
    "channel_not_found": {
        "functional_class": None,
        "command": [controller_commands['404']],
    },
}


sys_channel = ["channel_stop", "active_channel", "step_back"]
select_channel = "channel_select"

# custom channel define
custom_channel = ["channel{}".format(i) for i in xrange(12, 21)]

channels.update({
    "channel12": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    }})

