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
        self.image_pub = rospy.Publisher("Debug",Image,queue_size=10)                                               
        
        self.red_low = np.array([0, 70, 50])                                                                       
        self.red_high =  np.array([10, 255, 255])                                                                    
      
        self.yellow_low = np.array([21, 93, 86])                                                                    
        self.yellow_high = np.array([118, 255, 255])

        self.green_low = np.array([51,114,86])                                                                     
        self.green_high = np.array([142,255,255])
        
        self.Color = False       
        self.mas = []
        self.bridge = CvBridge()                                                                                     
        self.image_sub = rospy.Subscriber("main_camera/image_raw_throttled",Image,self.callback)  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    def point(self, makers):
        b = 0.50
        i,j = 0,0
        while i < len(self.mas):
            j = i+1
            while j < len(self.mas):
                if math.sqrt(abs(self.mas[i][0] - self.mas[j][0])**2 + abs(self.mas[i][1] - self.mas[j][1])**2) <= b:
                    self.mas[i][0] = (self.mas[i][0] + self.mas[j][0])/2
                    self.mas[i][1] = (self.mas[i][1] + self.mas[j][1])/2
                    del self.mas[j]
                j += 1
            if self.mas[i][2] == 0:
                markers['Red'].append([self.mas[i][0],self.mas[i][1]])
            elif self.mas[i][2] == 1:
                markers['Yellow'].append([self.mas[i][0],self.mas[i][1]])
            elif self.mas[i][2] == 2:
                markers['Green'].append([self.mas[i][0],self.mas[i][1]])
            i += 1
    def distance_x(self,x,z):
        if x >= 120:
            return (-(x - 120))* z / 87.43 + 0.05
        else:
            return (120 - x)* z / 87.43
    def distance_y(self,y,z):
        if y >= 160:
            return (y - 160)* z / 94.38
        else:
            return -(160 - y)* z / 94.38
    def distance_cam(self, rect):
        #print('koi')
        MARKER_SIDE1_SIZE = 0.5
        MARKER_SIDE2_SIZE = 0.5
        op = np.array([(-MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0), (-MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0)])
        cM = np.array([[92.37552065968626, 0.0, 160.5], [0.0, 92.37552065968626, 120.5], [0.0, 0.0, 1.0]])
        dC = np.zeros((8, 1), dtype="float64")
        retval, rvec, tvec = cv2.solvePnP(np.array(op, dtype="float32"), np.array(rect, dtype="float32"), cM, dC)
        print('cx_cam=',tvec[1][0], 'cy_cam=',tvec[0][0], 'cz_cam=',tvec[2][0])
        return [tvec[0][0],tvec[1][0]]
    
    def callback(self,data):                                                                                         # Основная функция (data- изображения из типа msg)
        if self.Color == True:
            try:                                                                                                         # Считывание и конвертация изображения в вид пригодный для дальнейшей работы (try- для отсутствия ошибки если топик будет пустой)
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except:pass
            start = get_telemetry(frame_id='aruco_map')
            startz = rospy.wait_for_message('rangefinder/range', Range)
            Grey = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            if self.Color == True:
                mask1 = cv2.inRange(Grey, self.red_low, self.red_high)                                                          # Создание облак точек для каждого цвета
                mask2 = cv2.inRange(Grey, self.yellow_low, self.yellow_high)
                mask3 = cv2.inRange(Grey, self.green_low, self.green_high)
                
                st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
                st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))
                thresh = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                
                _, red, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                     # Поиск контуров в облаке точек (Красном)
                for c in red:                                                                                                # Перебор каждого контура
                    try:
                        y,x = 0,0
                        moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 500:
                            y = int(sum_x / sum_pixel)
                            x = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,startz.range)
                            y_d = self.distance_y(y,startz.range)
                            approx = cv2.approxPolyDP(c, 0.01* cv2.arcLength(c, True), True)
                            points_img = np.array([np.array(p[0]) for p in approx])
                            y_d_2, x_d_2 = self.distance_cam(points_img)
                            
                            if math.sqrt(x_d**2+y_d**2) < 50 and x_d != 0 and y_d != 0:
                                print('x_d,y_d',start.x+x_d,start.y-y_d)
                                print('x_d_2,y_d_2', start.x-x_d_2,start.y-y_d_2)
                                #self.mas.append([start.x+x_d,start.y-y_d,0])
                                #print('x_d,y_d', x_d, y_d, type(x_d), type(y_d))
                                cv2.putText(img, 'Red', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))                       
                                cv2.drawContours(img, [c], 0, (193,91,154), 2)
                    except:pass
                
                thresh = cv2.morphologyEx(mask2 - mask3, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, yellow, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in yellow:
                    try:
                        moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 500:
                            y = int(sum_x / sum_pixel)
                            x = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,startz.range)
                            y_d = self.distance_y(y,startz.range)
                            print('x_d,y_d',start.x+x_d,start.y-y_d)
                            self.mas.append([start.x+x_d,start.y-y_d,1])
                            cv2.putText(img, 'Yellow', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                            cv2.drawContours(img, [c], 0, (193,91,154), 2)                         
                    except:pass
                
                thresh = cv2.morphologyEx(mask3, cv2.MORPH_CLOSE, st1)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, st2)
                _, green, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)                        # И желтого
                for c in green:
                    try:   
                        moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                        sum_y = moments['m01']
                        sum_x = moments['m10']
                        sum_pixel = moments['m00']
                        if sum_pixel > 500:
                            y = int(sum_x / sum_pixel)
                            x = int(sum_y / sum_pixel)
                            x_d = self.distance_x(x,startz.range)
                            y_d = self.distance_y(y,startz.range)
                            print('x_d,y_d',start.x+x_d,start.y-y_d)
                            self.mas.append([start.x+x_d,start.y-y_d,2])
                            cv2.putText(img, 'Green', (y, x), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
                            cv2.drawContours(img, [c], 0, (193,91,154), 2)                    
                    except:pass
                
                
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))                                           # Вывод конвертипованного изображения
                except CvBridgeError as e:
                    print(e)
                    
def main():                                                                                                      # Начальная функция
  global col_det
  col_det = ColorDetecting()                                                                                         # Обращение к классу Color_detect


print navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(3)

print navigate(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)
main()
print('ready')

print navigate(x=3.5, y=0.5, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(8)
col_det.Color = True
rospy.sleep(0.5)
#col_det.Color = False
print navigate(x=3.5, y=1, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(6)
col_det.Color = True
rospy.sleep(0.5)
#col_det.Color = False

print navigate(x=3.5, y=0, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(6)
col_det.Color = True
rospy.sleep(0.5)
#col_det.Color = False

print navigate(x=3, y=0.5, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(6)
col_det.Color = True
rospy.sleep(0.5)
#col_det.Color = False

print navigate(x=4, y=0.5, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(6)
col_det.Color = True
rospy.sleep(0.5)
#col_det.Color = False

print navigate(x=3, y=0.25, z=1, speed=0.5, frame_id='aruco_map')
rospy.sleep(6)
col_det.Color = True
rospy.sleep(0.5)
col_det.Color = False

land()
print(col_det.mas)
f = open('Data11.txt', 'w')
f.write('????? ?????       ??????????      ???? ???????????? ???????\n')
for i in range (len(col_det.mas)):
    print("{}.             x={}                    y={}".format(col_det.mas[i][2], col_det.mas[i][0], col_det.mas[i][1]))
    f.write("{}.             x={}                    y={}\n".format(col_det.mas[i][2], col_det.mas[i][0], col_det.mas[i][1]) )
f.close()

markers = {'Red':[],'Yellow':[],'Green':[]}
print('Подождите обработку данных') 
col_det.point(markers)

k = 1
f = open('Data10.txt', 'w')
print(markers)
print('????? ?????       ??????????      ???? ???????????? ???????')
f.write('Номер точки       Координаты      Цвет распознаного маркера\n')
for i in ['Red', 'Yellow', 'Green']:
    if len(markers[i])>0:
        for j in markers[i]:
            print("{}.             x={}                    y={}			{}".format(k, j[0], j[1],i))
            f.write("{}.             x={}                    y={}		{}\n".format(k, j[0], j[1],i) )
            k+=1
f.close()
