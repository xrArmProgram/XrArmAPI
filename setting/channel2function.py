from simple.object_sorting.ObjectSorting import ObjectSorting
from setting.conroller_commands import controller_commands

from simple import DetectorColor, FaceFollower, BasicControlMode, LearningMode, VisualGrabbingMode, \
    ActionMode, ShapeAnalysis

channels = {
    "default_channel": {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    },
    "control_model": {
        "functional_class": BasicControlMode,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "learning_model": {
        "functional_class": LearningMode,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "operation_action": {
        "functional_class": ActionMode,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "visual_grabbing": {
        "functional_class": VisualGrabbingMode,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "color_recognition": {
        "functional_class": DetectorColor,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "shape_recognition": {
        "functional_class": ShapeAnalysis,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "face_following": {
        "functional_class": FaceFollower,
        "command": [
            controller_commands['exit_function'],
            controller_commands['say_hello'],
        ],
    },
    "sorting_mode": {
        "functional_class": ObjectSorting,
        "command": [controller_commands['exit_function']],
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


select_channel = "channel_select"
exit_channel = "exit_function"
sys_channel = [exit_channel, "active_channel", "step_back", select_channel]

# define custom channel [channel12, channel20]
custom_channel = ["channel{}".format(i) for i in xrange(12, 21)]

channels.update({
    custom_channel[0]: {
        "functional_class": None,
        "command": [controller_commands['say_hello']],
    }})

