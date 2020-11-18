def say_hello(**kwargs):
    print("hello world")


def channel_not_found(**kwargs):
    print("channel not found")


def stop_last_function(**kwargs):
    kwargs["last_function"].stop()
    print("last function is stopped")
