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
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

ser = serial.Serial('/dev/ttyS0', 19200, timeout=1)


def forward(left_speed, right_speed):
    command_right = chr(0xC1)
    command_left = chr(0xC9)
    ser.write(command_right)
    ser.write(chr(right_speed))
    ser.write(command_left)
    ser.write(chr(left_speed))


def backward(speed):
    command_right = chr(0xC2)
    command_left = chr(0xCA)
    ser.write(command_right)
    ser.write(chr(speed))
    ser.write(command_left)
    ser.write(chr(speed))


def stop():
    command_right = chr(0xC2)
    command_left = chr(0xCA)
    ser.write(command_right)
    ser.write(chr(0))
    ser.write(command_left)
    ser.write(chr(0))



def rotate_left_90(left_speed, right_speed):
    command_right = chr(0xC1)
    command_left = chr(0xCA)
    ser.write(command_right)
    ser.write(chr(right_speed))
    ser.write(command_left)
    ser.write(chr(left_speed))


def rotate_right_90(left_speed, right_speed):
    command_right = chr(0xC2)
    command_left = chr(0xC9)
    ser.write(command_right)
    ser.write(chr(right_speed))
    ser.write(command_left)
    ser.write(chr(left_speed))


class wheels_controller:

    def __init__(self):

        self.wheels_sub = rospy.Subscriber("wheels_speed", Vector3Stamped, self.callback)

    def callback(self,data):

        left = int(data.vector.x)
        right = int(data.vector.y)
        direction = int(data.vector.z)

        if(right==0 and right==left):
            stop()

        elif(direction == 0):
            forward(left, right)

        elif (direction == 1):
            rotate_left_90(left, right)

        elif (direction == 2):
            rotate_right_90(left, right)

        else:
            stop()

        print (left, right, direction)



def main(args):

    wheels_cont = wheels_controller()
    rospy.init_node('WHEELS_CONTROLLER', anonymous=True)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    stop()


if __name__ == '__main__':
    main(sys.argv)
