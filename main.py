from traceback import print_exc

from API import ArmBuilder, RobotController, ArmRobot, PlaySoundSpeaker, SNRVoice


comes = "/dev/ttyUSB0"
board = 9600

builder = ArmBuilder(RobotController, ArmRobot, PlaySoundSpeaker, SNRVoice, comes, board)

builder.build()
builder_iterable = builder.run_iterable()

while True:
    try:
        builder_iterable.next()

    except StopIteration:
        break

    except KeyboardInterrupt as e:
        print_exc(e)
        print("got error")

builder.destroy()

