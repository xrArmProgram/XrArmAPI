import cv2
import numpy as np
import threading
import time


# from BaseRobot import SimpleRobot
# robot = SimpleRobot("CameraMotion")


class CameraMotion:
	def __init__(self):
		self.loop = True
		self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
		self.cap.set(3, 480)  # 设置画面宽度
		self.cap.set(4, 640)  # 设置画面长度
		self.color_dist = {'red': {'Lower': np.array([156, 128, 46]), 'Upper': np.array([180, 255, 255])},  # 较好
						   # color_dist = {'red': {'Lower': np.array([0, 128, 46]), 'Upper': np.array([5, 255, 255])},#较好
						   # color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},#不好
						   'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
						   'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
						   }
		self.idx_dist = ['red', 'blue', 'green']  # 颜色List
		self.CNT = 10  # 设置识别率计算次数
		self.rec_count = 0  # 自增值
		self.precision = [0, 0, 0]  # 识别度存储

	def analyse(self, frame, color_dict):
		dat = [0] * len(color_dict)  # 根据颜色长度定义数组
		for i in self.color_dist.keys():
			# print(i)
			index = {key: index for index, key in enumerate(self.color_dist)}.get(i)  # 获取字典对应下标
			result = self.recognizeColor(frame, self.color_dist[i]['Lower'], self.color_dist[i]['Upper'], 10000)
			if result:
				dat[index] = 1  # 对应颜色下标的数据赋值1
		# print(dat)
		return dat  # 返回数据组

	def recognizeColor(self, frame, lower, upper, area):
		result = False
		gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
		hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
		erode_hsv = cv2.erode(hsv, None, iterations=2)  # 腐蚀粗的变细
		inRange_hsv = cv2.inRange(erode_hsv, lower, upper)  # 根据阀值，去除背景部分
		cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 获得形状块
		if cnts:  # 确认检测到形状块
			for c in cnts:
				# if the contour is not sufficiently large, ignore it
				if cv2.contourArea(c) < area:  # 获取形状块的面积大小，过滤小面积
					continue
				else:
					result = True
					c = max(cnts, key=cv2.contourArea)  # 在边界中找出面积最大的区域
					rect = cv2.minAreaRect(c)  # 形成最小外接矩形
					box = cv2.boxPoints(rect)  # 获取4矩形4个顶点坐标
					cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)  # 画出矩形

					M = cv2.moments(c)  # 获取轮廓矩形属性字典
					cX = int(M["m10"] / M["m00"])  # 获取中心点X轴坐标
					cY = int(M["m01"] / M["m00"])  # 获取中心点Y轴坐标

					cv2.putText(frame, ("(X:" + str(cX) + " Y:" + str(cY) + ")"), (cX - 20, cY),
								cv2.FONT_HERSHEY_SIMPLEX,
								0.5, (255, 255, 255), 2)  # 文字显示方块中心点坐标
		return result

	def run(self):
		# while self.cap.isOpened():
		while self.loop:
			ret, frame = self.cap.read()
			if ret:
				if frame is not None:
					if self.rec_count < self.CNT:  # 循环计算次数
						self.rec_count = self.rec_count + 1
						list_color = self.analyse(frame, self.color_dist)  # 获取各个颜色识别的结果，返回list
						self.precision[0] = self.precision[0] + int(list_color[0])  # 计算下标为0的颜色识别的次数
						self.precision[1] = self.precision[1] + int(list_color[1])  # 计算下标为1的颜色识别的次数
						self.precision[2] = self.precision[2] + int(list_color[2])  # 计算下标为2的颜色识别的次数
					else:
						idx = self.precision.index(max(self.precision))  # 最大值下标
						print("识别颜色为：%s,识别率为:%d" % (self.idx_dist[idx], max(self.precision) / self.CNT))  # 打印识别颜色及识别度
						self.precision = [0, 0, 0]  # 清空识别度
						self.rec_count = 0
					cv2.imshow('camera', frame)  # 显示窗口
					if cv2.waitKey(1) == ord('q'):  # 当按键按下q键时退出
						break
				# return
				else:
					print("无画面")
			else:
				print("无法读取摄像头！")
		self.cap.release()
		# cv2.waitKey(0)
		cv2.destroyAllWindows()

	def start(self):
		self.loop = True

	def stop(self):
		self.loop = False


cmtion = CameraMotion()

if __name__ == "__main__":
	cmtion.run()
