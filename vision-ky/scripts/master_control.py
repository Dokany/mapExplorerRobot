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
import message_filters
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

IDLE_STATE = 0
DRIVE_STATE = 1
ROTATE_STATE = 2
READ_SIGN_STATE = 3
PID_RATIO = 40.0/1000.0
class master_control:

    def __init__(self):
        rospy.init_node('MASTER_CONTROL', anonymous=True)
        self.imu_publish = rospy.Publisher("wheels_speed",Vector3Stamped, queue_size=10)
        self.pid_publish = rospy.Publisher("pid_start", Bool, queue_size=10)
        self.angles_sub = rospy.Subscriber("imu_angles",Vector3Stamped, self.callback1)
        self.lidar_sub = rospy.Subscriber("lidar_stop",Bool, self.callback0)
        self.lw_sub = rospy.Subscriber("left_wheel/control_effort", Float64, self.callback2)
        self.rw_sub = rospy.Subscriber("right_wheel/control_effort", Float64, self.callback3)
        self.sd_sub = rospy.Subscriber("sign_direction", Float32, self.callback4)
        self.roll = 0
        self.pid = 0
        self.pitch = 0
        self.yaw = 0
        self.rotate = 0
        self.angle = 0
        self.cnt = 0
        self.start = 0
        self.rd = 0
        self.stop = 0
        self.current_state = 0
        self.next_state = 0
        self.lmotor = 0
        self.rmotor=0
        self.direction = 0

    def callback0(self, data): #lidar
        self.stop=data.data
        if(self.stop==1):
            print("STOOOOP")
        #self.work()

    def callback1(self,data): #imu

        self.roll = data.vector.x
        self.pitch = data.vector.y
        self.yaw = data.vector.z
        if (self.yaw < 0):
            self.yaw = 360 + self.yaw
        #self.work()

    def callback2(self, data): #left
        if(data.data>0):
            self.lmotor=0
        else:
            self.lmotor=int(abs(data.data)*PID_RATIO)

        #self.work()

    def callback3(self, data): #right
        if (data.data > 0):
            self.rmotor = 0
        else:
            self.rmotor = int(abs(data.data) * PID_RATIO)
        #self.work()

    def callback4(self, data):  # direciton\
        self.direction=int(data.data)
        #self.work()

    def work(self):
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            self.current_state=self.next_state
            print (self.current_state)
            left = 0
            right = 0
            dir = 0
            self.pid = 0
            if(self.current_state == IDLE_STATE):
                if(self.stop == 0):
                    self.next_state = DRIVE_STATE
                    left = 50
                    self.pid = 1
                    right = 50
                    dir = 0
                else:
                    self.next_state = IDLE_STATE


            elif(self.current_state == DRIVE_STATE):
                if (self.stop == 0):
                    self.next_state = DRIVE_STATE
                    left = 75 + self.lmotor
                    right = 75 + self.rmotor
                    self.pid = 1
                    if(self.lmotor==0 and self.rmotor==0):
                         left+=5
                         right+=5
                    self.rmotor = self.lmotor = 0

                    dir = 0
                else:
                    self.next_state = READ_SIGN_STATE
                    self.pid = 0


            elif(self.current_state == READ_SIGN_STATE):
                #rospy.sleep(1)
                print("DIRECTION: ", self.direction)
                self.rotate = self.direction
                if(self.direction==0 or self.direction==3):#end
                    self.next_state=IDLE_STATE
                else:
                    self.start = 0
                    self.next_state = ROTATE_STATE

            else:

                if(self.start==0):
                    #rospy.sleep(1)
                    self.angle=self.yaw
                    self.start = 1
                    dir = self.rotate

                print ('current = %f, yaw = %f'%(self.angle, self.yaw))


                if(abs(self.angle-self.yaw)>=90):
                    self.next_state = DRIVE_STATE
                    #rospy.sleep(1)

                else:
                    self.current_state = ROTATE_STATE
                    left = 50
                    right = 50
                    dir = self.rotate


            v3 = Vector3()
            v = Vector3Stamped()
            self.cnt+=1
            head = Header()
            head.stamp = rospy.Time.now()
            head.seq= self.cnt
            v3.x=left
            v3.y=right
            v3.z=dir
            v.header = head
            v.vector=v3
            pid_bool = Bool()
            pid_bool = self.pid
            try:
                self.imu_publish.publish(v)
                self.pid_publish.publish(pid_bool)
            except Exception as e:
                print(e)
            rate.sleep()




def main(args):
    ic = master_control()
    ic.work()


if __name__ == '__main__':
    main(sys.argv)
