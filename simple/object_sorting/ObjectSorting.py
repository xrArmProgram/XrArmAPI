# coding: utf-8


import cv2
import numpy as np
from time import sleep

import xrarm_audio
from API.BASE import AbstractRunner
from simple.object_sorting.config import color_recognition_sensitivity, init_angle, object_pose, color_pose, sound_of_colors


class ObjectSorting(AbstractRunner):
    def __init__(self, robot, local_rospy):
        self.__is_running = False
        self.__cap = cv2.VideoCapture(0)
        self.__cap.set(3, 480)  # 设置画面宽度
        self.__cap.set(4, 640)  # 设置画面长度
        self.__robot = robot
        self.__color_dist = {
            'green': {
                'Lower': np.array([35, 43, 46]),
                'Upper': np.array([90, 255, 255]),
            },
            'yellow': {
                'Lower': np.array([20, 150, 43]),
                'Upper': np.array([40, 256, 256])
            },
            'red': {
                'Lower': np.array([160, 128, 35]),
                'Upper': np.array([180, 255, 256]),
            },
            'blue':  {
                'Lower': np.array([90, 80, 46]),
                'Upper': np.array([100, 255, 255]),
            },
        }
        self.__idx_dist = {'red': 0, 'yellow': 1, 'blue': 2, 'green': 3}  # 颜色List
        self.__CNT = 10  # 设置识别率计算次数
        self.__rec_count = 0  # 自增值
        self.__precision = [0, 0, 0, 0]  # 识别度存储

        self.__color = None

    def analyse(self, frame, color_dict):
        dat = [0, 0, 0, 0]  # 根据颜色长度定义数组
        # print("color dict length: {}, {}".format(len(dat), len(color_dict)))
        for i in self.__color_dist.keys():
            # print(i)
            index = self.__idx_dist[i]  # 获取字典对应下标
            result = self.recognize_color(frame, self.__color_dist[i]['Lower'], self.__color_dist[i]['Upper'], 6000)
            if result:
                dat[index] = 1  # 对应颜色下标的数据赋值1
                print(i)
        # print(dat)
        return dat  # 返回数据组

    # @staticmethod
    def recognize_color(self, frame, lower, upper, area):
        result = False
        gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像

        erode_hsv = cv2.erode(hsv, None, iterations=2)  # 腐蚀粗的变细
        in_range_hsv = cv2.inRange(erode_hsv, lower, upper)  # 根据阀值，去除背景部分

        self.__robot.show('hsv', in_range_hsv)

        self.__robot.show('tiqu', hsv)
        counts = cv2.findContours(in_range_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 获得形状块

        if counts:  # 确认检测到形状块
            for c in counts:
                # if the contour is not sufficiently large, ignore it
                if cv2.contourArea(c) < area:  # 获取形状块的面积大小，过滤小面积
                    continue
                else:
                    result = True
                    c = max(counts, key=cv2.contourArea)  # 在边界中找出面积最大的区域
                    rect = cv2.minAreaRect(c)  # 形成最小外接矩形
                    box = cv2.boxPoints(rect)  # 获取4矩形4个顶点坐标
                    cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)  # 画出矩形

                    m = cv2.moments(c)  # 获取轮廓矩形属性字典
                    c_x = int(m["m10"] / m["m00"])  # 获取中心点X轴坐标
                    c_y = int(m["m01"] / m["m00"])  # 获取中心点Y轴坐标

                    cv2.putText(frame, ("(X:" + str(c_x) + " Y:" + str(c_y) + ")"), (c_x - 20, c_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)  # 文字显示方块中心点坐标
        return result

    def __get_color(self):
        color_count = 0
        while self.__is_running:
            ret, frame = self.__cap.read()
            if ret:
                if frame is not None:
                    if self.__rec_count < self.__CNT:  # 循环计算次数
                        self.__rec_count = self.__rec_count + 1
                        list_color = self.analyse(frame, self.__color_dist)  # 获取各个颜色识别的结果，返回list
                        # self.__precision[0] = self.__precision[0] + int(list_color[0])  # 计算下标为0的颜色识别的次数
                        # self.__precision[1] = self.__precision[1] + int(list_color[1])  # 计算下标为1的颜色识别的次数
                        # self.__precision[2] = self.__precision[2] + int(list_color[2])  # 计算下标为2的颜色识别的次数
                        for i in range(4):
                            self.__precision[i] += int(list_color[i])
                            # print("i: {}".format(i))
                    else:
                        idx = self.__precision.index(max(self.__precision))  # 最大值下标
                        # 打印识别颜色及识别度
                        success = max(self.__precision) / self.__CNT > 0.5

                        current_color = ""

                        for key in self.__idx_dist.keys():
                            if self.__idx_dist[key] == idx:
                                current_color = key

                        print("识别颜色为：{},识别率为:{}".format(current_color, success))

                        self.__precision = [0, 0, 0, 0]  # 清空识别度
                        self.__rec_count = 0

                        if self.__color != current_color and success:
                            self.__color = current_color
                            color_count = 0

                        else:
                            if self.__color is not None and success:
                                color_count += 1

                        if color_count > color_recognition_sensitivity:
                            return self.__color

                    self.__robot.show('camera1', frame)  # 显示窗口
                else:
                    print("无画面")
            else:
                print("无法读取摄像头！")

        return None

    def run(self):
        self.__is_running = True
        self.__robot.speak(xrarm_audio.start_sorting_mode)

        self.__robot.update(init_angle)
        sleep(3)
        while self.__is_running:
            local_color = self.__get_color()
            if local_color is None:
                break

            print("got it")
            self.__robot.speak(sound_of_colors[local_color])
            for angle in object_pose:
                self.__robot.update(angle)
                sleep(1)

            # print("color: {}, pose: {}".format(local_color, color_pose[local_color]))

            for angle in color_pose[local_color]:
                self.__robot.update(angle)
                sleep(1)

            self.__robot.update(init_angle)

        self.__cap.release()
        cv2.destroyAllWindows()

        print("function end")

    def stop(self):
        self.__is_running = False
