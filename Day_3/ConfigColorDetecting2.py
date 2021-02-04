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
    def __init__(self, simulator):
        rospy.init_node('Color_detect', anonymous=True)                                                              
        self.image_pub = rospy.Publisher("Debug",Image,queue_size=10)                                               
        self.image_pub2 = rospy.Publisher("Debuuuuuug",Image,queue_size=10)
        
        self.simulator = simulator
        if self.simulator:
            self.red_low = np.array([0,0,240])                                                                             # ????????? ??????????? ??? ??????????? ????? ????? ??????? ?????:
            self.red_high = np.array([10,10,255])                                                                            # ????????

            self.blue_low = np.array([240,0,0])                                                                             # ??????
            self.blue_high = np.array([255,10,10])

            self.yellow_low = np.array([0,240,240])                                                                           # ? ???????
            self.yellow_high = np.array([10,255,255])

            self.green_low = np.array([0,240,0])                                                                           # ? ???????
            self.green_high = np.array([10,255,10])
        else:
            self.red_low = np.array([55,55,170])                                                                             # ????????? ??????????? ??? ??????????? ????? ????? ??????? ?????:
            self.red_high = np.array([135,125,25])                                                                            # ????????

            self.blue_low = np.array([120,90,0])                                                                             # ??????
            self.blue_high = np.array([210,140,80])

            self.yellow_low = np.array([10,160,160])                                                                            # ? ???????
            self.yellow_high = np.array([120,230,220])

            self.green_low = np.array([50,90,20])                                                                           # ? ???????
            self.green_high = np.array([115,160,95])

        self.x_dist = 0
        self.y_dist = 0
        self.number = -1
        self.message = ''
        self.e0 = 0
        self.e1 = 0
        self.e2 = 0
        self.e3 = 0
        self.etalon()
        
        self.out = cv2.VideoWriter('Scinti_pogalyista.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (320,240))
        self.out2 = cv2.VideoWriter('Scinti_pogalyista2.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 20, (70,70))
        self.Color = False   
        self.Num = False   
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
    def obrezka(self, mask):
        x = 0
        y = 0
        w = 0
        h = 0
#        mask[1][1] = 255
        mask[0][0] = 255
        _, contoours, yuu =cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#        contoours=contoours[0]
        #print(contoours)
        #print(_)
        self.cont = contoours
#        print(contoours)
        if True:
            #contoours=sorted(contoours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contoours[0])
            mask=mask[y:y+h, x:x+w]
            yy = y+h//2
            xx = x+w//2
            #cv2.drawContours(img, [contoours], 0, (193,91,154), 2)
        #print(w, h) 
        return(xx,yy,mask,x,y,w,h)
    def etalon(self):
        self.e0 = cv2.imread("g0.png")
        self.e0 =cv2.inRange(self.e0,(20,71,7),(80,120,50)) 
        self.e0 = cv2.medianBlur(self.e0, 5)
        self.e1 = cv2.imread("g1.png")
        self.e1=cv2.inRange(self.e1,(20,71,7),(80,120,50)) 
        self.e1 = cv2.medianBlur(self.e1, 5)
        self.e2 = cv2.imread("g2.png")
        self.e2=cv2.inRange(self.e2,(20,71,7),(80,120,50)) 
        self.e2 = cv2.medianBlur(self.e2, 5)
        self.e3 = cv2.imread("g3.png")
        self.e3=cv2.inRange(self.e3,(20,71,7),(80,120,50)) 
        self.e3 = cv2.medianBlur(self.e3, 5)

    def analiz(self,img1,img2):
        s = 0
        x = 100
        y = 100
        for i in range(x):
            for j in range(y):
                if img1[i][j] == img2[i][j]:
                    s+=1
        rez = (s*100//(x*y))
#        print(rez)
        return(rez)

    def detekt(self,img):
        img2= cv2.resize(img, (300,300))
        img2 = img2[15: 285, 15: 285] 
        img2 = cv2.rotate(img2, cv2.ROTATE_90_CLOCKWISE) # ?????? ???????
        mask=cv2.inRange(img2,(20,71,7),(80,125,80)) # ????? ??? ?? ?????
        mask = cv2.medianBlur(mask, 5)
        
        xx,yy,mask2,x,y,w,h = self.obrezka(mask)
        img3=img2[y:y+h, x:x+w]
        img3= cv2.resize(img3, (100,100))
        mask2=cv2.inRange(img3,(20,71,7),(80,125,80)) # ????? ??? ?? ?????
        mask2 = cv2.medianBlur(mask2, 5)
        #self.image_pub2.publish(self.bridge.cv2_to_imgmsg(mask2, "bgr8")) 
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(img3, "bgr8"))                                      # ????? ? ????????? ?????
        except CvBridgeError as e:
            print(e)
        rez = 777
        p = 1
        if self.simulator:
            sr = 70
        else:
            sr = 80
        if (self.analiz(mask2,self.e0)>sr): 
            rez = 0
            self.message = 'D0_delivered products'
        if (self.analiz(mask2,self.e1)>sr):
            rez = 1
            self.message = 'D1_delivered clothes'
        if (self.analiz(mask2,self.e2)>sr):
            rez = 2
            self.message = 'D2_delivered fragile packaging'
        if (self.analiz(mask2,self.e3)>sr):
            rez = 3
            self.message = 'D3_delivered correspondence'
        p0 = self.analiz(mask2,self.e0)
        p1 = self.analiz(mask2,self.e1)
        p2 = self.analiz(mask2,self.e2)
        p3 = self.analiz(mask2,self.e3)
        print(p0, p1, p2, p3, rez)
#        print(self.message)
#        print('g_analiz')
        return(rez, xx, yy)
    
    def point(self):
        numberLine = 0
        dist = 0.8
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
#            self.x_dist = 0
#            self.y_dist = 0
        if self.simulator == False:
            img = cv2.undistort( img,np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]), np.array([ 2.15356885e-01,  -1.17472846e-01,  -3.06197672e-04,-1.09444025e-04,  -4.53657258e-03,   5.73090623e-01,-1.27574577e-01,  -2.86125589e-02,   0.00000000e+00,0.00000000e+00,   0.00000000e+00,   0.00000000e+00,0.00000000e+00,   0.00000000e+00]),np.array([[166.23942373073172,0,162.19011246829268],[0,166.5880923974026,109.82227735714285],[0,0,1]]))
        self.out.write(img)
        self.start = get_telemetry(frame_id='aruco_map')
        self.startz = rospy.wait_for_message('rangefinder/range', Range)
        #Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img = img[85: 155, 125: 195]
        mask1 = cv2.inRange(img, self.red_low, self.red_high)          #Red
        mask2 = cv2.inRange(img, self.yellow_low, self.yellow_high)    #Yellow
        mask3 = cv2.inRange(img, self.green_low, self.green_high)      #Green
        mask4 = cv2.inRange(img, self.blue_low, self.blue_high)        #Blue
        
        st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
        st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))
        thresh = cv2.morphologyEx(mask4, cv2.MORPH_CLOSE, st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
        try:
            if self.Num == True:
                
                imGG= cv2.resize(img, (320,240))
#                print('////')
                rez, xx, yy = self.detekt(imGG)
                if rez >= 0 and rez <= 3:
                    self.number = rez
                    self.number_x = yy*320//270
                    self.number_y = 240 - xx*240//270
#                    cv2.rectangle(img, (x, y), (x+2, y+2), (255, 0, 255), 2)
                    self.number_x = self.distance_x(self.number_x,self.startz.range)
                    self.number_y = self.distance_y(self.number_y,self.startz.range)
#                    self.Num = False
#                    cv2.drawContours(img, [self.cont], 0, (193,91,154), 2)
#                    print(rez)
                else:
                    self.number = -1
        except:pass
        _, blue, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)   #Blue
        for c in blue:    
            try:
                y,x = 0,0
                moments = cv2.moments(c, 1)       
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                print('Blue:',sum_pixel)
                if sum_pixel > 300:
                    #print(sum_pixel)
                    y = int(sum_x / sum_pixel)*320//70
                    x = int(sum_y / sum_pixel)*240//70
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
#                    print(x_d,y_d)
                    if self.color_flag == 3:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if self.Color == True:
                        print('Blue x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.Color = False
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,2])
                    cv2.putText(img, 'Blue', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))                       
                    cv2.drawContours(img, [c], 0, (193,91,154), 2)
            except:pass
            
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
                print('Red:',sum_pixel)
                if sum_pixel > 300:
#                    print(sum_pixel)
                    y = int(sum_x / sum_pixel)*320//70
                    x = int(sum_y / sum_pixel)*240//70
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
#                    print(x_d,y_d) 

                    if self.color_flag == 3:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if self.Color == True:
                        print('Red x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.Color = False
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,3])
                    cv2.putText(img, 'Red', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))                       
                    cv2.drawContours(img, [c], 0, (193,91,154), 2)
            except:pass
        
        thresh = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, st1)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
        _, yellow, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)          #Yellow
        for c in yellow:
            try:
                moments = cv2.moments(c, 1) 
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                print('Yellow:',sum_pixel)
                if sum_pixel > 300:
#                    print(sum_pixel)
                    y = int(sum_x / sum_pixel)*320//70
                    x = int(sum_y / sum_pixel)*240//70
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
#                    print(x_d,y_d) 
                    if self.color_flag == 0:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if self.Color == True:
                        print('Yellow x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.Color = False
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,0])
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
                print('Green:',sum_pixel)
                if sum_pixel > 300:
#                    print(sum_pixel)
                    y = int(sum_x / sum_pixel)*320//70
                    x = int(sum_y / sum_pixel)*240//70
                    x_d = self.distance_x(x,self.startz.range)
                    y_d = self.distance_y(y,self.startz.range)
 #                   print(x_d,y_d) 
                    if self.color_flag == 1:
                        self.x_dist = self.start.x+x_d
                        self.y_dist = self.start.y-y_d
                    if self.Color == True:
                        print('Green x_d,y_d',self.start.x+x_d,self.start.y-y_d)
                        self.Color = False
                        self.mas.append([self.start.x+x_d,self.start.y-y_d,1])
                    #cv2.putText(img, 'Green', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                    #cv2.drawContours(img, [c], 0, (193,91,154), 2)
            except:pass
        self.out2.write(img)
        if self.x_dist == 0 and self.y_dist == 0:
            self.color_flag = -1
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8")) 
        except CvBridgeError as e:
            print(e)
