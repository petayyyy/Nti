# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time

cam = cv2.VideoCapture(0)

red_low = np.array([55,55,170])                                                                             # Параметры необходимые для определения облак точек каждого цвета:
red_high = np.array([120,125,235])                                                                            # Красного

blue_low = np.array([120,90,0])                                                                             # Синего
blue_high = np.array([160,135,20])

yellow_low = np.array([10,160,160])                                                                           # И желтого
yellow_high = np.array([65,200,200])

green_low = np.array([60,120,20])                                                                           # И желтого
green_high = np.array([100,160,60])
##
cam = cv2.VideoCapture('Scinti_pogalyista (3).avi')
from time import sleep
while True:
    try:	
        ret, img = cam.read()# Считывание изображения
        
        mask1 = cv2.inRange(img, red_low, red_high)                                                          # Создание облак точек для каждого цвета
        mask2 = cv2.inRange(img, blue_low, blue_high)
        mask3 = cv2.inRange(img, yellow_low, yellow_high)
        mask4 = cv2.inRange(img, green_low, green_high)

        res1 = cv2.bitwise_and(img, img, mask= mask1)                                                         # Метод для отображения облака точек в цвете
        cv2.imshow('Red',res1)                                                                                # Вывод в отдельное окно

        res2 = cv2.bitwise_and(img, img, mask= mask2)                                                         # И так с каждым цветом
        cv2.imshow('Blue',res2)
            
        res3 = cv2.bitwise_and(img, img, mask= mask3)
        cv2.imshow('Yellow',res3)

        res4 = cv2.bitwise_and(img, img, mask= mask4)
        cv2.imshow('green',res4)

        green = cv2.findContours(mask4,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                   # Поиск контуров в облаке точек (Красном)
        for c in green[0]:                                                                                      # Перебор каждого контура
            try:
                y,x = 0,0
                moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 100:                                                                           # Отсеивание помех(нужно подстроить под ваше разрешение камеры)
                    x = int(sum_x / sum_pixel)                                                                # Определение центра объекта
                    y = int(sum_y / sum_pixel)
##                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)                                               # Обвод контуров и вывод на изображение его цвет
##                    cv2.putText(img, 'green', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
        
        red = cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                   # Поиск контуров в облаке точек (Красном)
        for c in red[0]:                                                                                      # Перебор каждого контура
            try:
                y,x = 0,0
                moments = cv2.moments(c, 1)                                                                   # Метод создающий матрицу объекта
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 100:                                                                           # Отсеивание помех(нужно подстроить под ваше разрешение камеры)
                    x = int(sum_x / sum_pixel)                                                                # Определение центра объекта
                    y = int(sum_y / sum_pixel)
##                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)                                               # Обвод контуров и вывод на изображение его цвет
##                    cv2.putText(img, 'red', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
                
        blue = cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                  # Тоже самое для синего   
        mas2,m = [],0
        for c in blue[0]:
            try:
                moments = cv2.moments(c, 1)
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                   
                if sum_pixel > 100:
                    m += sum_pixel                                                                            # Только добавлен подсчет площади (+= так как я нахожу площадь каждой фигуры по отдельности и потом суммирую их)
                    x = int(sum_x / sum_pixel)
                    y = int(sum_y / sum_pixel)
                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)    
                    mas2.append([x,y])                                                                        # Добавление каждой фигуры для подсчета колличества (С сохранением координат объектов)
##                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5) 
##                    cv2.putText(img, 'blue', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
##        print('Blue elements: ', len(mas2))                                                                   # Вывод колличества и площади для синих в консоль
##        print('Blue area: ',m,' pix')
##        cv2.putText(img, ('Blue elements: '+str(len(mas2))), (60, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))# И в финальное изображение (В левом верхнем угле) 
##        cv2.putText(img, ('Blue area: '+str(int(m))+'pix'), (60, 110), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))


        yellow = cv2.findContours(mask3,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # И желтого
        for c in yellow[0]:
            try:
                moments = cv2.moments(c, 1)
                sum_y = moments['m01']
                sum_x = moments['m10']
                sum_pixel = moments['m00']
                if sum_pixel > 100:
                    x = int(sum_x / sum_pixel)
                    y = int(sum_y / sum_pixel)
##                    cv2.drawContours(img, [c], 0, (0, 0, 0), 5)    
##                    cv2.putText(img, 'yellow', (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
            except:pass
        cv2.imshow("camera", img)                                                                             # Вывод финального изображения на дисплей
        if cv2.waitKey(10) == 27:                                                                             # Вывод из программы на кнопку ESC
            break
        sleep(0.1)
    except:
        cam = cv2.VideoCapture('Scinti_pogalyista.avi')
cam.release()
cv2.destroyAllWindows()


##frame = cv2.imread("1234.png") 
##frame=cv2.inRange(img,(80,120,50),(120,180,100)) 
while (cv2.waitKey(1) != 27):
    ret, img = cam.read()
    cv2.imshow("img",img)
    img = cv2.inRange(img,(80,120,50),(120,180,100)) 
    cv2.imshow("frame",img)
    if cv2.waitKey(10) == 27:                                                                             # Вывод из программы на кнопку ESC
            break
    sleep(0.1)
