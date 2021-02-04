# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import time
import roslib
import sys
import rospy
from ConfigColorDetecting2 import ColorDetecting
from led import led
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

land = True # Esli land True
high_1 = 1.1
high_2 = 2

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
  col_det = ColorDetecting(False) # Esli simul True, esli real False

DX_delivered = [] 

def D_delivered(number, tale):
  global DX_delivered
  DX_delivered.append('D{}_delivered to {} cargo'.format(number, tale))
mas = [0]*4
def indication():
  global mas, col_det, land
  rospy.sleep(5.5)
  col_det.Num = True
  rospy.sleep(0.5)
  print(col_det.number)
  if col_det.number != -1:
    led(col_det.number)
    d = col_det.message
    mas[col_det.number] = 0
    D_delivered(col_det.number,mas[col_det.number])
    #print navigate(x=col_det.number_x, y=col_det.number_y, z=col_det.startz.range-0.1, speed=0.5, frame_id='aruco_map')
    if land == True:
        print navigate(x=0, y=0, z=-0.2, speed=0.5, frame_id='body')
        rospy.sleep(1)
        land()
        rospy.sleep(10)
        print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(5)
    led(-1)
    print(d)
    

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0, y=0, z=2, speed=0.5, frame_id='aruco_map')
rospy.sleep(9)
main()

print('resdy point land detect')
print navigate(x=0, y=3*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
rospy.sleep(10)

navigate(x=0, y=3*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=0.9, y=3*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=2*0.9, y=3*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=3*0.9, y=3*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=4*0.9, y=3*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=4*0.9, y=2*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=3*0.9, y=2*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=2*0.9, y=2*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=1*0.9, y=2*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=0*0.9, y=2*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=0*0.9, y=1*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=1*0.9, y=1*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=2*0.9, y=1*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=3*0.9, y=1*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=4*0.9, y=1*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=4*0.9, y=0*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=3*0.9, y=0*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=2*0.9, y=0*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=1*0.9, y=0*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

navigate(x=0*0.9, y=0*0.9, z=high_2, speed=0.4, frame_id='aruco_map')
indication()

#rint navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map')
#rospy.sleep(9)
land()

print('//////////////////////////////////')
try:
  print(DX_delivered[0])
  print(DX_delivered[-1])
except:pass
print("Balance {} cargo".format(sum(mas)))
