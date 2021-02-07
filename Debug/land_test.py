# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import time
import roslib
import sys
import rospy
from ConfigColorDetecting import ColorDetecting
from led import led
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
            
def main():
  global col_det
  col_det = ColorDetecting()

DX_delivered = [] 

def D_delivered(number, tale):
    global DX_delivered
    DX_delivered.append('D{}_delivered to {} cargo'.format(number, tale))

def indication():
    global mas, col_det
    rospy.sleep(3.5)
    col_det.Num = True
    rospy.sleep(0.5)
    if col_det.number != -1:
        led(col_det.number)
        print navigate(x=self.x_dist, y=self.y_dist, z=self.startz.range-0.1, speed=0.5, frame_id='aruco_map')
        rospy.sleep(1)
        land()
        rospy.sleep(4)
        led(-1)
        rospy.sleep(2)
        print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
        rospy.sleep(3)
        print(col_det.message)
        D_delivered(col_det.number,mas[col_det.number])
        mas[col_det.number] = 0

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)
main()

print navigate(x=0, y=3*0.9, z=1.8, speed=0.5, frame_id='aruco_map')
rospy.sleep(6)

#navigate(x=0, y=2*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=1*0.9, y=2*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=2*0.9, y=2*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=3*0.9, y=2*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=4*0.9, y=2*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=4*0.9, y=1*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=3*0.9, y=1*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=2*0.9, y=1*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=1*0.9, y=1*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=0*0.9, y=1*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=0*0.9, y=0*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=1*0.9, y=0*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=2*0.9, y=0*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=3*0.9, y=0*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#navigate(x=4*0.9, y=0*0.9, z=1.8, speed=0.25, frame_id='aruco_map')
#indication()

#print navigate(x=0, y=0, z=1.8, speed=0.5, frame_id='aruco_map')
#rospy.sleep(7)

#rospy.sleep(3)

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(4)
main()

navigate(x=0.9, y=0.9, z=1.8, speed=0.5, frame_id='aruco_map')
rospy.sleep(5)

navigate(x=0.9*3, y=0.9, z=1.8, speed=0.5 , frame_id='aruco_map')
rospy.sleep(3)

navigate(x=0.9*5.5, y=0, z=0.5, speed=0.5, frame_id='aruco_map')
rospy.sleep(8)

navigate(x=0.9*5.5, y=0.9, z=0.5, speed=0.5 , frame_id='aruco_map')
rospy.sleep(4)

navigate(x=0.9*5.5, y=2*0.9, z=0.5, speed=0.5, frame_id='aruco_map')
rospy.sleep(4)

navigate(x=0.9*5.5, y=3*0.9, z=0.5, speed=0.5, frame_id='aruco_map')
rospy.sleep(4)

navigate(x=0, y=0, z=2.5, speed=0.5, frame_id='aruco_map')
rospy.sleep(14)

land()

land()
print('//////////////////////////////////')
print(DX_delivered[0])
print(DX_delivered[1])
print("Balance {} cargo".format(sum(mas)))
print('//////////////////////////////////')

print('End popitki')
