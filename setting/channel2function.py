from setting.conroller_commands import controller_commands

from sample import DetectorColor

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
    "channel7": {
        "functional_class": DetectorColor,
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

