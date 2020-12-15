# coding: utf-8


import cv2
import numpy as np

# from API.BASE import AbstractRunner
from API.BASE import AbstractRunner


class ShapeAnalysis(AbstractRunner):
    def __init__(self, robot, local_rospy):
        self.__shapes = {'triangle': 0, 'rectangle': 0, 'polygons': 0, 'circles': 0}
        self.__thresh_min = 50  # Canny边缘检测，阈值最小值
        self.__thresh_max = 150  # Canny边缘检测，阈值最大值
        self.__binary_min = 0  # 二值化最小值
        self.__binary_max = 254  # 二值化最大值
        self.__epsilonProportion = 1  # 轮廓逼近精度
        self.__is_running = False
        self.__cap = cv2.VideoCapture(0)
        self.__cap.set(3, 480)  # 设置画面宽度
        self.__cap.set(4, 640)  # 设置画面长度
        self.__robot = robot

    def __edge_demo(self, image):
        # 高斯滤波(3, 3)表示高斯矩阵的长与宽都是3，标准差取0
        blurred = cv2.GaussianBlur(image, (3, 3), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)  # 将图像变为灰度图
        # 图像梯度计算
        grad_x = cv2.Sobel(gray, cv2.CV_16SC1, 1, 0)
        grad_y = cv2.Sobel(gray, cv2.CV_16SC1, 0, 1)

        self.__thresh_min = cv2.getTrackbarPos('threshMin', 'image')  # 获取滑动条值
        self.__thresh_max = cv2.getTrackbarPos('threshMax', 'image')
        # edge_output = cv2.Canny(grad_x, grad_y, self.thresh_min, self.thresh_max)  # 启动Canny边缘检测
        edge_output = cv2.Canny(grad_x, grad_y, self.__thresh_min, self.__thresh_max)  # 启动Canny边缘检测
        return edge_output

    def __callback(self):
        pass

    def __analysis(self, frame):
        thresh = self.__edge_demo(frame)
        # self.__binary_min = cv2.getTrackbarPos('binaryMin', 'image')  # 获取滑动条值
        # self.__binary_max = cv2.getTrackbarPos('binaryMax', 'image')
        ret, binary = cv2.threshold(thresh, self.__binary_min, self.__binary_max, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        # ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)  # 转为二值图
        # self.__robot.show("input image", frame)

        # contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        tmp_img, contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        draw_img = frame.copy()  # 复制原图
        # mask = cv2.drawContours(draw_img, contours, -1, (255, 255, 255), -1)  # 绘制轮廓，形成掩膜
        mask = cv2.drawContours(draw_img, contours, -1, (0, 0, 255), 2)  # 绘制轮廓，形成掩膜
        self.__robot.show("image", mask)

        if contours:  # 确认检测到形状块
            for cnt in range(len(contours)):
                # 提取与绘制轮廓
                # cv2.drawContours(result, contours, cnt, (0, 255, 0), 2)
                if not self.__is_running:
                    break

                # 轮廓逼近
                # self.__epsilonProportion = cv2.getTrackbarPos('epsilonProportion', 'image') * 0.01
                epsilon = self.__epsilonProportion * cv2.arcLength(contours[cnt], True)  # 计算轮廓周长
                approx = cv2.approxPolyDP(contours[cnt], epsilon, True)

                # 分析几何形状
                corners = len(approx)
                shape_type = ""
                if corners > 2:
                    if corners == 3:
                        count = self.__shapes['triangle']
                        count = count + 1
                        self.__shapes['triangle'] = count
                        shape_type = "三角形"

                    if corners == 4:
                        count = self.__shapes['rectangle']
                        count = count + 1
                        self.__shapes['rectangle'] = count
                        shape_type = "矩形"

                    if corners >= 10:
                        count = self.__shapes['circles']
                        count = count + 1
                        self.__shapes['circles'] = count
                        shape_type = "圆形"

                    if 4 < corners < 10:
                        count = self.__shapes['polygons']
                        count = count + 1
                        self.__shapes['polygons'] = count
                        shape_type = "多边形"

                    print("shape_type:%s ----%s" % (shape_type, corners))

    # self.__robot.show("input image", binary)

    def __draw_text_info(self, image):
        c1 = self.__shapes['triangle']
        c2 = self.__shapes['rectangle']
        c3 = self.__shapes['polygons']
        c4 = self.__shapes['circles']
        cv2.putText(image, "triangle: " + str(c1), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        cv2.putText(image, "rectangle: " + str(c2), (10, 40), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        cv2.putText(image, "polygons: " + str(c3), (10, 60), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        cv2.putText(image, "circles: " + str(c4), (10, 80), cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 0, 0), 1)
        return image

    def run(self):
        # cv2.namedWindow('image')
        # 创建俩个滑动条
        # cv2.createTrackbar('threshMin', 'image', 250, 255, self.__callback)  # 第一个参数时滑动条的名字，第二个参数是滑动条被放置的窗口的名字，
        # # 第三个参数是滑动条默认值，第四个参数时滑动条的最大值，第五个参数时回调函数，每次滑动都会调用回调函数。
        # cv2.createTrackbar('threshMax', 'image', 0, 255, self.__callback)
        # cv2.createTrackbar('binaryMin', 'image', 0, 255, self.__callback)  # 二值化最小值调节
        # cv2.createTrackbar('binaryMax', 'image', 254, 255, self.__callback)  # 二值化最大值调节
        # cv2.createTrackbar('epsilonProportion', 'image', 1, 20, self.__callback)  # 二值化最大值调节

        self.__is_running = True

        while self.__is_running:
            try:
                ret, img = self.__cap.read()
                if ret:
                    if img is not None:
                        self.__analysis(img)

            except KeyboardInterrupt:
                print("user stop app")
                self.stop()

        cv2.destroyAllWindows()
        self.__cap.release()

    def stop(self):
        self.__is_running = False

