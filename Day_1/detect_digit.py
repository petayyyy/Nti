import cv2
import numpy as np

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

    

    
    

