from abc import ABCMeta, abstractmethod


class AbstractController(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def run_iterable(self):
        pass
