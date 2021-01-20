import xrarm_audio


def say_hello(local_rospy, robot, last_function):
    print("hello world")


def channel_not_found(local_rospy, robot, last_function):
    print("channel not found")


def stop_last_function(local_rospy, robot, last_function):
    robot.speak(xrarm_audio.exit_function)
    print("last function is stopped")


def back2background(local_rospy, robot, last_function):
    robot.speak(xrarm_audio.back_to_the_background)
    print("back to background")


def active(local_rospy, robot, last_function):
    robot.speak(xrarm_audio.activation)
    print("hello master")


def channel_select(local_rospy, robot, last_function):
    robot.speak(xrarm_audio.selection_function)
    print("channel selecting")


def exit_function(local_rospy, robot, last_function):
    if last_function is not None:
        robot.speak(xrarm_audio.exit_function)
        print("exit_function")
        last_function.stop()
