#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class image_converter:

	def __init__(self):
		self.cx_pub = rospy.Publisher("state", Float64, queue_size=10)
		self.cam_pub = rospy.Publisher("setpoint", Float64, queue_size=10)
		self.image_pub = rospy.Publisher("seg_output",Image, queue_size = 10)
		self.obj_pub = rospy.Publisher("lidar_stop", Bool, queue_size=100)
		self.sign_pub = rospy.Publisher("sign_start", Bool, queue_size=10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("csi_cam/image_raw", Image, self.callback)
		self.enable = 0
		self.pid_sub = rospy.Subscriber("pid_start", Bool, self.callback0)

	def callback0(self, data):
		self.enable = 1

	def callback(self, data):
		if(self.enable):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
				cv_image = cv2.medianBlur(cv_image, 5)
				#equ = cv2.equalizeHist(cv_image)
				#cv_image = np.hstack((cv_image, equ))  # stacking images side-by-side
				# cv_image = cv2.bilateralFilter(cv_image,9,50,50)
			except CvBridgeError as e:
				print(e)

			hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			# colors
			lower_blue = np.array([95, 50, 50])
			upper_blue = np.array([100, 255, 255])
			lower_yellow = np.array([25, 50, 50])
			upper_yellow = np.array([32, 255, 255])
			# Night-time
			lower_green = np.array([60, 50, 50])
			upper_green = np.array([95, 255, 255])

			# Day-time
			#lower_green = np.array([45, 50, 50])
			#upper_green = np.array([85, 255, 255])
			mask = cv2.inRange(hsv, lower_green, upper_green)
			output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

			#cv2.imshow("image segm", output)
			#cv2.waitKey(3)

			# contouring
			_, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


			cx = 0
			# Declarion of cam center's width
			width = np.size(output, 1) / 2
			# height = np.size(output, 0) / 2

			if len(contours) != 0:
				lcnt = contours[0]
				larea = cv2.contourArea(contours[0])

				for cnt in contours:  # sort!!
					area = cv2.contourArea(cnt)
					if  area > larea:
						lcnt = cnt
						larea = area
				if larea > 400:
					#cv2.drawContours(output, lcnt, -1, (0,255,0), 3)
					M = cv2.moments(lcnt)
					cx = int(M['m10'] / M['m00'])
					#cy = int(M['m01'] / M['m00'])
					print(width)
					print(cx)

				if larea > 50000:
					obj_detected = True
				else:
					obj_detected = False


				if larea > 25000:
					sign_detected = False
				else:
					sign_detected = True

				print(larea, obj_detected)
				x, y, w, h = cv2.boundingRect(lcnt)
				# Daylight
				#output = output[y + 25:y + h - 40, x + 20:x + w - 20]

				#Night-time
				output = output[y + 25:y + h, x + 20:x + w - 20]
				#top, bottom, left, right

			#cv2.circle(output, (int(cx), int(cy)), 1, (0, 255, 0), 1)  # draw state center
			#cv2.circle(output, (int(width), int(height)), 1, (0, 255, 0), 1)  # draw setpoint center

			#cv2.imshow("image segm", output)
			#cv2.waitKey(3)
			b = Bool()
			s = Bool()
			b = obj_detected
			s = sign_detected

			try:
				self.cx_pub.publish(cx)
				self.cam_pub.publish(width)
				self.obj_pub.publish(b)
				self.sign_pub.publish(s)
			    #self.cx_pub.publish(0)
				#self.cam_pub.publish(0)
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
			except CvBridgeError as e:
				print(e)


def main(args):
	ic = image_converter()
	rospy.init_node('VISION', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)
