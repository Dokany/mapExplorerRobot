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

class imu_filter:

    def __init__(self):
        self.imu_publish = rospy.Publisher("imu_angles",Vector3Stamped, queue_size=10)

        self.frame_id = "angles"
        self.cnt = 0
        self.imu_sub = rospy.Subscriber("imu/data",Imu, self.callback)

    def callback(self,data):

        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w

        #print('x = %.2f, y = %.2f, z = %.2f, w =%.2f' % (data.orientation.x, data.orientation.y, z, w))

        sqr = y * y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + sqr)
        roll = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (sqr + z * z)
        yaw = math.degrees(math.atan2(t3, t4))

        head = Header()
        head.stamp=rospy.Time.now()
        head.seq = self.cnt
        head.frame_id =self.frame_id

        self.cnt+=1
        v2 = Vector3Stamped()
        v2.header=head
        v1 = Vector3()
        v1.x=roll
        v1.y=pitch
        v1.z=yaw
        v2.vector=v1
        print ('roll = %.2f, pitch = %.2f, yaw = %.2f' %(roll,pitch,yaw))
        try:
            self.imu_publish.publish(v2)
        except Exception as e:
            print(e)



def main(args):

    ir = imu_filter()
    rospy.init_node('IMU_ANGLES', anonymous=True)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
