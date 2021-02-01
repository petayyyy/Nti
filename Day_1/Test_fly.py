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


class ColorDetecting():                                                                                              
    def __init__(self):                                                                                              
        rospy.init_node('Color_detect', anonymous=True)                                                              
        
        self.bridge = CvBridge()                                                                                     
        self.image_sub = rospy.Subscriber("main_camera/image_raw_throttled",Image,self.callback)  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.out = cv2.VideoWriter('Scinti_pogalyista.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240))

    def callback(self,data):  
        try:                                                 
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:pass
        self.out.write(img)
        
def main():
  global col_det
  col_det = ColorDetecting()

n = 5 # Parametry karti
b = 7

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)
main()
print('ready')

for i in range (b+1):
    for j in range (n*2+1):
        if i % 2 == 0:
            print navigate(x=j*0.45, y=i*0.9, z=1, speed=0.25, frame_id='aruco_map')
        else:
            print navigate(x=n-j*0.45, y=i*0.9, z=1, speed=0.25, frame_id='aruco_map')
        rospy.sleep(2)
        

print navigate(x=0, y=0, z=1, speed=5, frame_id='aruco_map')
rospy.sleep(6)

land()
