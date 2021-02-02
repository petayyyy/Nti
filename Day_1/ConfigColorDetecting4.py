# -*- coding: utf-8 -*-
import numpy as np
import math
import cv2
import time
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land = rospy.ServiceProxy('land', Trigger)

class ColorDetecting():                                                                                              
    def __init__(self):                                                                                              
        rospy.init_node('Color_detect', anonymous=True)                                                              
        self.image_pub = rospy.Publisher("Debug",Image,queue_size=10)                                               
        
        self.red_low = np.array([0, 70, 50])                                                                       
        self.red_high =  np.array([10, 255, 255])                                                                    
      
        self.yellow_low = np.array([21, 93, 86])                                                                    
        self.yellow_high = np.array([118, 255, 255])

        self.green_low = np.array([51,114,86])                                                                     
        self.green_high = np.array([142,255,255])

        self.x_dist = 0
        self.y_dist = 0
        
        self.out = cv2.VideoWriter('Scinti_pogalyista.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240))
        self.Color = False       
        self.mas = []
        self.bridge = CvBridge()                                                                                     
        self.image_sub = rospy.Subscriber("main_camera/image_raw_throttled",Image,self.callback)  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    def landing(self, x, y, color_flag):
        print navigate(x=x, y=y, z=1, speed=0.5, frame_id='aruco_map')
        #rospy.sleep(10)
        #navigate(x=x, y=y, z=1, frame_id='aruco_map')
        #print((abs(self.start.x-x)+abs(self.start.y-y))*2)
        rospy.sleep(10)
        
        #startz = rospy.wait_for_message('rangefinder/range', Range)
        self.color_flag = color_flag
        rospy.sleep(2)

        while self.startz.range > 0.6:
            if self.color_flag != -1:
                print(self.x_dist,self.y_dist)
                print navigate(x=self.x_dist, y=self.y_dist, z=self.startz.range-0.1, speed=0.5, frame_id='aruco_map')
                rospy.sleep(3)
		print(self.x_dist,self.y_dist)
            else:
                return False
        land()
        rospy.sleep(7)
        print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
        rospy.sleep(3)
        self.color_flag = -1
        self.x_dist = 0
        self.y_dist = 0
        return True
    
    def point(self):
        numberLine = 0
        dist = 0.5
        massiv_new = sorted(self.mas, key=lambda k: [k[1], k[0]])
        Points = {numberLine: [massiv_new[0]]}
        x_last, y_last = massiv_new[0][0], massiv_new[0][1]
        for line in massiv_new:
            if max(x_last, line[0]) - min(x_last, line[0]) <= dist and max(y_last, line[1]) - min(y_last, line[1]) <= dist:
                Points[numberLine].append(line)
            else:
                numberLine += 1
                Points[numberLine] = [line]
            x_last, y_last = line[0], line[1]
        Point = {}
        for numberLine in Points:
            Point[numberLine] = [abs(round(sum(x)/len(x), 2)) for x in zip(*Points[numberLine])]
        return Point
    def distance_x(self, x, z):
        if x >= 120:
            return (-(x - 120))* z / 87.43 + 0.05
        else:
            return (120 - x)* z / 87.43
    def distance_y(self, y, z):
        if y >= 160:
            return (y - 160)* z / 94.38
        else:
            return -(160 - y)* z / 94.38
    def callback(self,data):  
        try:                                                 
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except:pass
        self.out.write(img)
#            self.x_dist = 0
#            self.y_dist = 0

        self.start = get_telemetry(frame_id='aruco_map')
        self.startz = rospy.wait_for_message('rangefinder/range', Range)
        Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
        mask1 = cv2.inRange(Grey, self.red_low, self.red_high)          #Red
        mask2 = cv2.inRange(Grey, self.yellow_low, self.yellow_high)    #Yellow
        mask3 = cv2.inRange(Grey, self.green_low, self.green_high)      #Green
        
        st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
        st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))
        thresh = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
        
        _, red, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   #Red
        for c in red:    
            try:
                y,x = 0,0
                moments = cv2.moments(c, 1)       
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 500:
                    y = int(sum_x / sum_pixel)
                    x = int(sum_y / sum_pixel)
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
                    if self.color_flag == 3:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if math.sqrt(x_d**2+y_d**2) <= 1 and self.Color == True:
                        print('Red x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,0])
                    cv2.putText(img, 'Red', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))                       
                    cv2.drawContours(img, [c], 0, (193,91,154), 2)
            except:pass
        
        thresh = cv2.morphologyEx(mask2 - mask3, cv2.MORPH_CLOSE, st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
        _, yellow, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)          #Yellow
        for c in yellow:
            try:
                moments = cv2.moments(c, 1) 
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 500:
                    y = int(sum_x / sum_pixel)
                    x = int(sum_y / sum_pixel)
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
                    if self.color_flag == 0:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if math.sqrt(x_d**2+y_d**2) <= 1 and self.Color == True:
                        print('Yellow x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,1])
                    cv2.putText(img, 'Yellow', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                    cv2.drawContours(img, [c], 0, (193,91,154), 2)                         
            except:pass
        
        thresh = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
        _, green, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        #Green
        for c in green:
            try:   
                moments = cv2.moments(c, 1) 
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 500:
                    y = int(sum_x / sum_pixel)
                    x = int(sum_y / sum_pixel)
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
                    if self.color_flag == 1:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if math.sqrt(x_d**2+y_d**2) <= 1 and self.Color == True:
                        print('Green x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,2])
                    cv2.putText(img, 'Green', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                    cv2.drawContours(img, [c], 0, (193,91,154), 2)                    
            except:pass
        if self.x_dist == 0 and self.y_dist == 0:
            self.color_flag = -1
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8")) 
        except CvBridgeError as e:
            print(e)

et0 = cv2.imread("a0.png") 
et0 = cv2.inRange(et0,(45,100,40),(120,210,115)) 
et0 = cv2.resize(et0, (50,50))
et0 = cv2.medianBlur(et0, 11)

et1 = cv2.imread("a1.png") 
et1 = cv2.inRange(et1,(45,100,40),(120,210,115)) 
et1 = cv2.resize(et1, (50,50))
et1 = cv2.medianBlur(et1, 11)

et2 = cv2.imread("a2.png") 
et2 = cv2.inRange(et2,(45,100,40),(120,210,115)) 
et2 = cv2.resize(et2, (50,50))
et2 = cv2.medianBlur(et2, 11)

et3 = cv2.imread("a3.png") 
et3 = cv2.inRange(et3,(45,100,40),(120,210,115)) 
et3 = cv2.resize(et3, (50,50))
et3 = cv2.medianBlur(et3, 11)

def obrezka(mask):
    x = 0
    y = 0
    w = 0
    h = 0
    mask[1][1] = 255
    contours=cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        mask=mask[y:y+h, x:x+w]
        yy = y+h//2
        xx = x+w//2
    return [xx, yy, mask, x, y, w, h]

def blyat(frame,q,w,e,r,t,y):
    mask=cv2.inRange(frame,(q,w,e),(r,t,y))
    xx, yy, mask2, x, y, w, h = obrezka(mask)
    frame2=frame[y:y+h, x:x+w]
    mask=cv2.inRange(frame2, (q,w,e), (r,t,y))
    mask2= cv2.resize(mask, (50,50))
    mask2 = cv2.medianBlur(mask2, 11)
    etal0 = 0
    etal1 = 0
    etal2 = 0
    etal3 = 0
    for i in range(50):
        for j in range(50):
            if mask2[i][j]==et0[i][j]:
                etal0+=1
            if mask2[i][j]==et1[i][j]:
                etal1+=1
            if mask2[i][j]==et2[i][j]:
                etal2+=1
            if mask2[i][j]==et3[i][j]:
                etal3+=1) 
    qwerty = 777
    if (etal0>etal1) and (etal0>etal2) and (etal0>etal3) and  (etal0>1700):
        qwerty = 0
    if (etal1>etal0) and (etal1>etal2) and (etal1>etal3) and  (etal1>1900):
        qwerty = 1
    if (etal2>etal1) and (etal2>etal0) and (etal2>etal3) and  (etal2>1500):
        qwerty = 2
    if (etal3>etal1) and (etal3>etal2) and (etal3>etal0) and  (etal3>1500):
        qwerty = 3
    return [qwerty, mask2, [x + w//2, y + h//2]]

def detekt(frame):
    mask=cv2.inRange(frame,(0,90,0),(160,255,50))
    rez, img, xyc = blyat(frame,0,90,0,160,255,50)
    if (rez == 777):
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        rez, img, xyc = blyat(frame,0,90,0,160,255,50)
    if (rez == 777):
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        rez, img, xyc = blyat(frame,0,90,0,160,255,50)
    if (rez == 777):
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        rez, img, xyc = blyat(frame,0,90,0,160,255,50)
    if (rez==777):
        rez = False
    return [rez, xyc]

    
