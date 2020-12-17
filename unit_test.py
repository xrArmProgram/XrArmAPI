from xrarm_audio import robot_say_number
from API import PyAudioPlayer


player = PyAudioPlayer()
player.init()

player.playsound(robot_say_number)
player.stop()
