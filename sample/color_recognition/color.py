# coding: utf-8


import cv2
from time import sleep

from API.BASE import AbstractRunner
from config import color_recognition_sensitivity, empty_sensitivity, color_low_sensitivity
from xrarm_audio import color_sound


class DetectorColor(AbstractRunner):
    def __init__(self, robot, local_rospy):
        self.__robot = robot
        self.__color = None
        self.__color_count = 0

        self.__cap = cv2.VideoCapture(0)
        self.__cap.set(3, 480)
        self.__cap.set(4, 640)

        self.__is_run = False

    def run(self):
        self.__is_run = True
        empty_count = 0
        while self.__is_run:
            # Capture frame-by-frame
            ret, frame = self.__cap.read()
            if ret:
                # frame = imutils.resize(frame, width=600)
                # Our operations on the frame come here
                blurred = cv2.GaussianBlur(frame, (7, 7), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                # hsv = cv2.GaussianBlur(hsv, (7, 7), 0)

                # perform edge detection, then perform a dilation + erosion to
                # close gaps in between object edges
                # edged = cv2.Canny(hsv, 50, 100)
                # inRange()函数用于图像颜色分割
                # edged = cv2.inRange(hsv, Redlower, Redupper)
                edged = cv2.inRange(hsv, (0, 0, 0), (180, 255, 255))
                edged = cv2.dilate(edged, None, iterations=2)
                edged = cv2.erode(edged, None, iterations=2)
                # mask of green (36,0,0) ~ (70, 255,255)
                mask1 = cv2.inRange(hsv, (36, 43, 46), (70, 255, 255))

                # mask o blue (100,43,46) ~ (124, 255, 255)
                mask2 = cv2.inRange(hsv, (100, 43, 46), (155, 255, 255))

                # red
                mask3 = cv2.inRange(hsv, (156, 43, 46), (180, 255, 255))
                mask = {"green": mask1, "blue": mask2, "red": mask3}

                for color, mask in mask.items():

                    target = cv2.bitwise_and(edged, edged, mask=mask)
                    cnts = cv2.findContours(target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

                    color_count = self.__color_count

                    for c in cnts:
                        # if the contour is not sufficiently large, ignore it
                        if cv2.contourArea(c) < 10000:
                            continue

                        empty_count = 0
                        # y = i
                        # print(c, 'c type: ', type(c))
                        # print(cv2.contourArea(c))
                        if self.__color != color:
                            self.__color_count = color_low_sensitivity

                        self.__color = color
                        self.__color_count += 1
                        cv2.drawContours(frame, [c], 0, (255, 0, 0), thickness=3)

                        if self.__color_count > color_recognition_sensitivity:
                            self.__robot.speak(audio_file=color_sound[color], block=False)
                            self.__color_count = 0

                    if color_count == self.__color_count:
                        empty_count += 1

                    if empty_count > empty_sensitivity:
                        self.__color_count = color_low_sensitivity
                        empty_count = 0

                    # print("__color_count: ", self.__color_count, "and ", color_count, "empty_count: ", empty_count)

                self.__robot.show("color", frame)
                cv2.waitKey(10)
            else:
                print("无法读取摄像头！")

        cv2.destroyAllWindows()
        self.__cap.release()

    def stop(self):
        self.__is_run = False


