from traceback import print_exc

from API import ArmBuilder, RobotController, ArmRobot, SoundSpeaker, SNRVoice, PyAudioPlayer, ImgPlayer

comes = "/dev/xrvoice"
board = 9600

builder = ArmBuilder(RobotController, ArmRobot, SoundSpeaker, SNRVoice, comes, board, ImgPlayer, PyAudioPlayer)

builder.build()
builder_iterable = builder.run_iterable()

while True:
    try:
        builder_iterable.next()

    except StopIteration:
        break

    except KeyboardInterrupt:
        print("stop now")

    except Exception as e:
        print_exc(e)

builder.destroy()

