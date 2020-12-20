# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import time
import roslib
import sys
import rospy
from ConfigColorDetecting import ColorDetecting
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
            
def main():                                                                                                      # Начальная функция
  global col_det
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect

n = 4 # Parametry karti

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)
main()
print('ready')

for i in range (n+1):
    for j in range (n*2+1):
        if i % 2 == 0:
            print navigate(x=j*0.5, y=i, z=1, speed=0.25, frame_id='aruco_map')
        else:
            print navigate(x=n-j*0.5, y=i, z=1, speed=0.25, frame_id='aruco_map')
        rospy.sleep(2)
        col_det.Color = True
        rospy.sleep(0.5)
        col_det.Color = False


print navigate(x=0, y=0, z=1, speed=5, frame_id='aruco_map')
rospy.sleep(6)

land()
print(col_det.mas)

print('Podogdite obrabotky dannix') 
markers = col_det.point()

k = 1
f = open('Data.txt', 'w')
print(markers)
print('Nomer tochki       Coordinati                    Cvet raspoznanogo markera')
f.write('Nomer tochki       Coordinati                    Cvet raspoznanogo markera\n')

for j in markers.values():
    print("{}.             x={}                    y={}			{}".format(k, j[0], j[1],j[2]))
    f.write("{}.             x={}                    y={}		{}\n".format(k, j[0], j[1],j[2]) )
    k+=1
f.close()
