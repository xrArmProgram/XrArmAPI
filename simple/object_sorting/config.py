import xrarm_audio

color_recognition_sensitivity = 5

init_angle = [0.2, -0.25, -2.0, 0.73, 0.9]

object_pose = [[0.0, -0.73, -1.2, 0.58, 0.90], [0.0, -0.73, -1.2, 0.58, 0.27], [0.0, -0.4, -1.0, 0.58, 0.27]]

color_pose = {
    "yellow": [
        [0.74, -0.4, -1.0, 0.58, 0.27],
        [0.74, -1.0, -0.27, 0.23, 0.27],
        [0.74, -1.0, -0.27, 0.23, 0.9],
        [0.74, -0.4, -1.0, 0.58, 0.9],
    ],

    "blue": [
        [1.7, -0.4, -1.0, 0.58, 0.27],
        [1.7, -0.81, -0.71, 0.02, 0.27],
        [1.7, -0.81, -0.71, 0.02, 0.9],
        [1.7, -0.4, -1.0, 0.58, 0.9],
    ],

    "green": [
        [-1.5, -0.4, -1.0, 0.58, 0.27],
        [-1.5, -0.81, -0.71, 0.02, 0.27],
        [-1.5, -0.81, -0.71, 0.02, 0.9],
        [-1.5, -0.4, -1.0, 0.58, 0.9],
    ],

    "red": [
        [-0.8, -0.4, -1.0, 0.58, 0.27],
        [-0.8, -1.0, -0.27, 0.23, 0.27],
        [-0.8, -1.0, -0.27, 0.23, 0.9],
        [-0.8, -0.4, -1.0, 0.58, 0.9],
    ],
}

sound_of_colors = {
    "red": xrarm_audio.find_some_thing_red,
    "green": xrarm_audio.find_something_green,
    "yellow": xrarm_audio.find_something_yellow,
    "blue": xrarm_audio.find_something_blue,
}