from abc import ABCMeta, abstractmethod


class AbstractBuilder(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def build(self):
        pass

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def run_iterable(self):
        pass

    @abstractmethod
    def destroy(self):
        pass


class NotBuiltError(RuntimeError):
    def __str__(self):
        print("the builder not built yet")
