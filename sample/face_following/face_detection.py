'''
Descripttion: Scratch3.0 gui code
version: 1.2.1
Author: Secne
Date: 2020-11-05 15:56:17
'''
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    * @par Copyright (C): 2010-2020 Shenzhen XiaoRGEEK Tech
    * @file         face_tracking
    * @version      V1.0
    * @details
    * @par History
    
    @author: Ylstart
"""
# from __future__ import division
import cv2
import time


class faceDeteaction:

	def __init__(self):
		self.loop = True
		self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
		self.cap.set(3, 480)  # 设置画面宽度
		self.cap.set(4, 640)  # 设置画面长度
		# face.xml的位置要和本程序位于同一文件夹下
		self.face_cascade = cv2.CascadeClassifier('face.xml')

	def run(self):
		# 死循环，同学们可以积累一下，这里的死循环的写法还有哪些其他的
		while True:
			ret, frame = self.cap.read()
			if ret == True:
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				# 要先将每一帧先转换成灰度图，在灰度图中进行查找
				faces = self.face_cascade.detectMultiScale(gray)
				if len(faces) > 0:
					# print('face found!')
					for (x, y, w, h) in faces:
						# 参数分别是“目标帧”，“矩形”，“矩形大小”，“线条颜色”，“宽度”
						cv2.rectangle(frame, (x, y), (x + h, y + w), (0, 255, 0), 2)
						cv2.putText(frame, "X:" + str(x), (x, y + 20), cv2.FONT_HERSHEY_COMPLEX, 0.55, (0, 0, 255), 2)
						cv2.putText(frame, "Y:" + str(y), (x + 100, y + 20), cv2.FONT_HERSHEY_COMPLEX, 0.55,
									(0, 0, 255), 2)
						# max_face=w*h
						result = (x, y, w, h)
						x_middle = result[0] + w / 2  # x轴中心
						y_middle = result[1] + h / 2  # y轴中心
						print("x_middle-----%d" % x_middle)
						print("y_middle-----%d" % y_middle)

				self.__robot.show("capture", frame)

		self.cap.release()
		cv2.destroyAllWindows()

	def start(self):
		self.loop = True

	def stop(self):
		self.loop = False


if __name__ == "__main__":
	Face = faceDeteaction()
	Face.run()
