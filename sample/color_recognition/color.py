import cv2
import imutils

# cap = cv2.VideoCapture(0)
# ## Read
# img = cv2.imread("D:/deng/ppp/3.png")
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, 480)
cap.set(4, 640)


def Detector_color():
	while True:
		# Capture frame-by-frame

		ret, frame = cap.read()
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
			## mask of green (36,0,0) ~ (70, 255,255)
			mask1 = cv2.inRange(hsv, (36, 43, 46), (70, 255, 255))

			## mask o blue (100,43,46) ~ (124, 255, 255)
			mask2 = cv2.inRange(hsv, (100, 43, 46), (155, 255, 255))

			##  red
			mask3 = cv2.inRange(hsv, (156, 43, 46), (180, 255, 255))
			mask = {"green": mask1, "blue": mask2, "red": mask3}
			for key, value in mask.items():
				target = cv2.bitwise_and(edged, edged, mask=value)
				cnts = cv2.findContours(target, cv2.RETR_EXTERNAL,
										cv2.CHAIN_APPROX_SIMPLE)[-2]

				for c in cnts:
					# if the contour is not sufficiently large, ignore it
					if cv2.contourArea(c) < 100:
						continue
					else:
						# y = i
						print(key)
						# return c
		else:
			print("无法读取摄像头！")


Detector_color()
