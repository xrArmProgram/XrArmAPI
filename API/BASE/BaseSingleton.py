from threading import Lock

lock = Lock()


class BaseSingleton4py2(object):
    __instance = None

    def __new__(cls, *args, **kwargs):
        with lock:
            if not cls.__instance:
                cls.__instance = super(BaseSingleton4py2, cls).__new__(cls, *args, **kwargs)
            return cls.__instance
