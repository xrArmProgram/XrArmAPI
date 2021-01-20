# coding: utf-8

import numpy as np
import cv2


# 回调函数，未使用
def nothing(x):
	pass


# 将BGR图像转化为HSV图像
win_img = "new"
win_img_old = "old"
# pic = cv2.imread("pic.png", cv2.IMREAD_UNCHANGED)  # 自己想要分析的照片
pic = cv2.imread("pic.png")  # 自己想要分析的照片
pic1 = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)

# 显示原图像做对比
cv2.namedWindow(win_img_old, cv2.WINDOW_NORMAL)
cv2.imshow(win_img_old, pic)

# 新图像窗口
cv2.namedWindow(win_img, cv2.WINDOW_AUTOSIZE)
# 初始化滚动条
cv2.createTrackbar("H", win_img, 100, 150, nothing)
cv2.createTrackbar("S", win_img, 100, 150, nothing)
cv2.createTrackbar("V", win_img, 100, 150, nothing)

while True:
	# ESC按下退出
	if cv2.waitKey(10) == 27:
		print("finish adjust picture and quit")
		break

	# 读取滚动条现在的滚动条的HSV信息
	h_value = float(cv2.getTrackbarPos("H", win_img) / 100)
	s_value = float(cv2.getTrackbarPos("S", win_img) / 100)
	v_value = float(cv2.getTrackbarPos("V", win_img) / 100)
	# 拆分、读入新数据后，重新合成调整后的图片
	H, S, V = cv2.split(pic)
	new_pic = cv2.merge([np.uint8(H * h_value), np.uint8(S * s_value), np.uint8(V * v_value)])
	cv2.imshow(win_img, new_pic)

cv2.destroyAllWindows()
