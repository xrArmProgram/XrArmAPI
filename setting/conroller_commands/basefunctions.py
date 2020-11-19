def say_hello(local_rospy, robot, last_function):
    print("hello world")


def channel_not_found(local_rospy, robot, last_function):
    print("channel not found")


def stop_last_function(local_rospy, robot, last_function):
    print("last function is stopped")


def back2background(local_rospy, robot, last_function):
    print("back to background")


def active(local_rospy, robot, last_function):
    print("hello master")
