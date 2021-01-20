from xrarm_audio import robot_say_number
from API import PyAudioPlayer

from API.ArmBuilder import  ArmBuilder


player = PyAudioPlayer()
player.init()

player.playsound(robot_say_number)
player.stop()
