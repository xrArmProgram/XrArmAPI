from setting.conroller_commands import controller_commands

channels = {
    "channel1": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },

    "channel_not_found": {
        "functional_class": None,
        "command": [controller_commands['404']],
    },
}
