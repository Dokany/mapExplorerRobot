#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import serial
import string
import time
import math
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


    
class imu_reader:

	def __init__(self):
		
		self.imu_publish = rospy.Publisher("imu/data_raw",Imu, queue_size=10)	
		self.frame_id = "0"
		self.cnt = 0
		self.im = Imu()
		self.ser = serial.Serial(port='/dev/ttyACM0',baudrate=115200, timeout=1)
		
		
	def publish_imu(self):
		
		line = self.ser.readline()		
		while(line.find("!") != 0):
			line = self.ser.readline()
			
		head = Header()
		head.stamp=rospy.get_rostime() 
		head.seq = self.cnt
		head.frame_id =self.frame_id
		self.cnt+=1		
		
		gyro = line[:line.find("A")]
		gx=float(gyro[gyro.find("x=")+2:gyro.find("y=")])
		gy=float(gyro[gyro.find("y=")+2:gyro.find("z=")])
		gz=float(gyro[gyro.find("z=")+2:len(gyro)-2])

		g = Vector3()
		g.z=gz
		g.y=gy
		g.x=gx

		accel = line[line.find("A"):]
		ax=float(accel[accel.find("x=")+2:accel.find("y=")])
		ay=float(accel[accel.find("y=")+2:accel.find("z=")])
		az=float(accel[accel.find("z=")+2:])

		a = Vector3()
		a.z=az
		a.y=ay
		a.x=ax
		
		
		print (line)
		self.im.header=head
		self.im.linear_acceleration = a
		self.im.angular_velocity =g	
		
		try:
			self.imu_publish.publish(self.im)
		except Exception as e:
			print(e)
		

def main(args):
	
	ir = imu_reader()
	rospy.init_node('imu_data_raw', anonymous=True)
	while not rospy.is_shutdown():
		try:
			ir.publish_imu()
			#rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
	self.ser.close

if __name__ == '__main__':
	main(sys.argv)
