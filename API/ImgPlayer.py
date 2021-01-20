from cv2 import imshow, waitKey, destroyAllWindows

from API.BASE import AbstractImgPlayer


class ImgPlayer(AbstractImgPlayer):

    def __init__(self):
        self._images = []
        self._is_running = False

    def show(self, window_name, img):
        if len(self._images) > 10:
            self._images = self._images[-10:]

        self._images.append((window_name, img))

    def async_play(self):
        self._is_running = True

        while self._is_running:
            if len(self._images) > 0:
                window_name, img = self._images.pop(0)
                imshow(window_name, img)

            yield waitKey(1) & 0xff

    def destroy(self):
        self._images = False
        destroyAllWindows()


