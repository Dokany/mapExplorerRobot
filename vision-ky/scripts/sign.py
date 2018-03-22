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
		self.dir_pub = rospy.Publisher("sign_direction", Float32, queue_size=100)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("seg_output", Image, self.callback)
		self.enable = 0
		self.sign_sub = rospy.Subscriber("sign_start", Bool, self.callback0)

	def callback0(self, data):
		self.enable = data.data

	def callback(self, data):
		if(self.enable):
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
				#cv2.imshow("Crop Seg", cv_image)
				#cv2.waitKey(3)
			except CvBridgeError as e:
				print(e)

			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			edges = cv2.Canny(gray, 50, 150, apertureSize=3)
			circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT,1.5,1, param1=50,param2=30,minRadius=60,maxRadius=100)

			if circles is not None:
				#circles = np.round(circles[0, :]).astype("int")
				direction = 0
				#for (x, y, r) in circles:
					#cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
				print("circle")
				#cv2.imshow("Circle Image", cv_image)
				#cv2.waitKey(3)
			else:
				x = np.size(cv_image, 1)/2
				y = np.size(cv_image, 0)
				q1 = cv_image[:y/2, 0:x]
				q2 = cv_image[y/2:, 0:x]
				#cv2.imshow("Quarter Up", q1)
				#cv2.imshow("Quarter Down", q2)
				#cv2.waitKey(3)
				edges1 = cv2.Canny(q1, 50, 150, apertureSize=3)
				edges2 = cv2.Canny(q2, 50, 150, apertureSize=3)
				lines1 = cv2.HoughLines(edges1, 1, np.pi / 180, 20)
				lines2 = cv2.HoughLines(edges2, 1, np.pi / 180, 20)

				index  = 0
				sum = 0
				#print("Shape",lines1.shape)
				#print(lines1)

				rho1 = []
				theta1 = []
				rho2 = []
				theta2 = []

				for object in lines1:
					#theta = object[0][1]
					#rho = object[0][0]
					rho1.append(object[0][0])
					theta1.append(object[0][1])
					#sum = sum + rho
					#index = index + 1
				#avg1 = sum/index

				#index = 0
				#sum = 0
				#rho1 = np.array(rho1)
				mode1 = max(set(rho1), key=rho1.count)
				modet1 = max(set(theta1), key=theta1.count)
				print(modet1)

				#print(np.mean(rho1), np.var(rho1))

				for object in lines2:
					#theta = object[0][1]
					#rho = object[0][0]
					rho2.append(object[0][0])
					theta2.append(object[0][1])
					#sum = sum + rho
					#index = index + 1
				#avg2 = sum/index
				#rho2 = np.array(rho2)

				mode2 = max(set(rho2), key=rho2.count)
				modet2 = max(set(theta2), key=theta2.count)
				print(modet2)

				if np.abs(modet1 - modet2) < 0.5:
					direction = 2
					#print ("right")
				else:
					direction = 1
					#print ("left")

				#print(avg1, avg2)

				#cv2.imshow("sign image 1", edges1)
				#cv2.imshow("sign image 2", edges2)
				#cv2.waitKey(3)
			#direction = 2
			try:
				self.dir_pub.publish(direction)
			except CvBridgeError as e:
				print(e)

def main(args):
	ic = image_converter()

	rospy.init_node('SIGN', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)
