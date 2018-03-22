#!/usr/bin/env python

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import serial
import string
import time
import math

from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class lidar_obstacle:

    def __init__(self):

        self.lidar_sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.lidar_pub = rospy.Publisher("lidar_stop", Bool, queue_size=10)
        self.lidar_pub_2 = rospy.Publisher("sign_start", Bool, queue_size=10)

    def callback(self, data):

        ranges_temp = data.ranges
        length = len(ranges_temp)
        start = (176 * 3.14 / 180) // data.angle_increment
        end = (183 * 3.14 / 180) // data.angle_increment

        list = []
        # print ("end",end)
        # print("len of range", length)
        # print ('HI',data.angle_increment )
        for i in range(int(start), int(end)):
            if (data.ranges[i] < 15 and data.ranges[i] > 0.05):
               list.append(data.ranges[i])

        average = np.mean(list)
        print ('AVERAGE = ', average)

        stop = False
        read_sign = True
        if (average < 0.55):
            stop = True

        if(average < 0.8):
            read_sign = False
        print(stop)

        b = Bool()
        b.data = stop
        b2 = Bool()
        b2.data = read_sign
        try:
            #self.lidar_pub.publish(b)
            #self.lidar_pub_2.publish(b2)
        except Exception as e:
            print(e)


def main(args):
    lidar_obst = lidar_obstacle()
    rospy.init_node('SCAN', anonymous=True)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
