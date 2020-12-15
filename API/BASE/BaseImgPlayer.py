from abc import ABCMeta, abstractmethod


class AbstractImgPlayer(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def show(self, img, window_name):
        pass

    @abstractmethod
    def async_play(self):
        pass

    @abstractmethod
    def destroy(self):
        pass
