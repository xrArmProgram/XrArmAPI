from setting.conroller_commands import controller_commands

channels = {
    "channel1": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },

    "channel2": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel3": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel4": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel5": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel6": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel7": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel8": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "channel9": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },

    "channel_select": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "active_channel": {
        "functional_class": None,
        "command": [controller_commands['active']],
    },
    "channel_stop": {
        "functional_class": None,
        "command": [controller_commands['back2background']],
    },
    "channel_not_found": {
        "functional_class": None,
        "command": [controller_commands['404']],
    },
}

sys_channel = ["channel_stop", "active_channel", "channel_select"]
