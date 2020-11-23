import xrarm_audio


def say_hello(local_rospy, robot, last_function):
    print("hello world")
    robot.speak(audio_file=xrarm_audio.naruto, block=False)


def channel_not_found(local_rospy, robot, last_function):
    print("channel not found")


def stop_last_function(local_rospy, robot, last_function):
    print("last function is stopped")


def back2background(local_rospy, robot, last_function):
    print("back to background")


def active(local_rospy, robot, last_function):
    print("hello master")


def channel_select(local_rospy, robot, last_function):
    print("channel selecting")


def exit_function(local_rospy, robot, last_function):
    # print(last_function)
    if last_function is not None:
        print("exit_function")
        last_function.stop()
