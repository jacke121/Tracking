# -*- coding: utf-8 -*-
import numpy as np
import cv2
import sys
import time as ti
from time import time
from ptz_control import *
import kcftracker
from optflow import *
# #########################初始化视野，并捕捉目标


def capture1_aim():
    global get_aim, cap, interval, centerx, centery, capindex, aim_thresh
    no_aim_frame = None
    (x, y, w, h) = (-1, -1, -1, 1)
    while True:
        ret, frame = cap.read()
        if ret is False:
            break
        if capindex is 0:
            capindex = 2
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5,5), 0)#参数的意义
            if no_aim_frame is None:
                no_aim_frame = gray
                continue
            frameDelta = cv2.absdiff(no_aim_frame, gray)
            thresh = cv2.threshold(frameDelta,25, 255, cv2.THRESH_BINARY)[1]
            kernel = np.ones((20,20),np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            cv2.imshow('win3',thresh)
            (_,cnts,_)  = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < aim_thresh:
                    continue
                (x, y, w, h) = cv2.boundingRect(c)
                get_aim = True
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                print x,y,w,h
#                break
            no_aim_frame = gray
        # if get_aim is True:
        #   break
        capindex-=1
        cv2.rectangle(frame,(centerx-10,centery-10),(centerx+20,centery+20),(255,255,255),1,1)
        cv2.imshow('tracking',frame)
        if cv2.waitKey(interval) & 0xff == 27:
            (x,y,w,h)=(0,0,0,0)
            break
    return x,y,w,h,frame
###############################################################################
def capture2_aim():
    global get_aim,cap,interval,centerx,centery,capindex,aim_thresh
    no_aim_frame = None
    (x,y,w,h)=(-1,-1,-1,-1)
    while True:
        ret , frame = cap.read()
        if ret is False:
            break
        img = frame.copy()
        if capindex is 0:
            capindex = 3
            t1 = time()
            img = cv2.resize(img,(0,0),fx=1,fy=1)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if no_aim_frame is None:
                no_aim_frame = gray
                continue
            flow = cv2.calcOpticalFlowFarneback(no_aim_frame, gray, None, 0.5, 2, 20, 2, 5, 1.2, 0)
            #新参数for opencv331: prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma,flags
            #前帧，当前帧，None, 金字塔参数0.5，金字塔层数，窗口大小，迭代次数，像素邻域，高斯标准差，flags
            motion2color = motionToColor(flow);
            no_aim_frame = gray
            pic,hsv_v=draw_hsv(flow)
            # 显示稠密光流
            of = draw_flow(gray, flow)
            cv2.imshow('op_image', of)

            ret, thresh = cv2.threshold(hsv_v, 5, 255, cv2.THRESH_BINARY)
            #定义结构元素
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(20, 20))
            #开运算-先dilate后erode
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
            #闭运算-先erode后dilate
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
#            cv2.imshow('win1',thresh)
            #erode减小目标轮廓
            kernel1 = cv2.getStructuringElement(cv2.MORPH_CROSS,(15,15))
            thresh = cv2.erode(thresh, kernel1)
            cv2.imshow('win3',thresh)
            (_,cnts, _)  = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) < aim_thresh/4:
                    continue
                (x, y, w, h) = cv2.boundingRect(c)
                get_aim = True
                cv2.rectangle(frame, (2*x, 2*y), (2*x + 2*w, 2*y + 2*h), (0, 255, 0), 2)
                # print 2*x,2*y,2*w,2*h
#                break
#             if get_aim is True:
#                 break
            cv2.rectangle(frame,(centerx-10,centery-10),(centerx+20,centery+20),(255,255,255),1,1)
            cv2.imshow('tracking',frame)
            t2= time()

#            print t2-t1
        capindex-=1
        t3=time()
#        print 1,t3-t1
        if cv2.waitKey(interval) & 0xff == 27:
            (x,y,w,h)=(0,0,0,0)
            break
    return 2*x,2*y,2*w,2*h,frame
#############################云台控制函数
def start_ptz(coord):
    global count
    x=coord[0]
    y=coord[1]
    w=coord[2]
    h=coord[3]
    (centerw,centerh) = (x+w/2,y+h/2)
    rate = int(size[0]*size[1])/(w*h)
    x_move=abs(centerw-centerx)
    y_move=abs(centerh-centery)
    x_sign = centerw-centerx > 0
    y_sign = centerh-centery > 0
    x_sign_serial = str(x_sign)
    y_sign_serial = str(y_sign)
    run_thresh=(0.7*h+0.3*w)/4
    stop_thresh=(0.7*h+0.3*w)/6
    if count is 0:
        if (x_move>run_thresh) & (y_move>run_thresh):
            ptz2direction(com0,x_sign_serial,y_sign_serial)
            count = 1
        elif (y_move>run_thresh):
            ptz_v(com0,y_sign_serial)
            count = 1
        elif (x_move>run_thresh):
            ptz_h(com0,x_sign_serial)
            count = 1
    else:
        if (x_move>stop_thresh) & (y_move>stop_thresh):
            ptz2direction(com0,x_sign_serial,y_sign_serial)
        elif (y_move>stop_thresh):
            ptz_v(com0,y_sign_serial)
        elif (x_move>stop_thresh):
            ptz_h(com0,x_sign_serial)
        else:
            ptz_handle(com0,move_stop)
            count = 0
##############


if __name__ == '__main__':
    start_time=time()
    # cap = cv2.VideoCapture("rtsp://192.168.50.7:554/12/preview.asp")
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture('K:/ZJU/Code/data/videos/Walk.mpg')
    tracker = kcftracker.KCFTracker( False, True, True)  # TRUE: hog, fixed_window, multiscale
    cv2.namedWindow('tracking')
    ##################
    #分辨率，帧
    fps = cap.get(cv2.CAP_PROP_FPS)
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print (cap.isOpened(), fps, size)
    centerx=int(size[0]/2)
    centery=int(size[1]/2)
    #云台坐标，云台参数调整
    x_sign=None
    y_sign=None
    x_move=0
    y_move=0
    run_thresd=0
    aim_thresd=0
    count=0
    capindex=0
    ptz_index=0
    interval = 1
    duration = 0.01
    aim_thresh = 8000
    get_aim=False
    # com0=None
    # com0=ptz_init('com4')
    # set_speed(com0, 0x10)#速度设置‘0x10’中速，‘0x11’快速
    while(cap.isOpened()):
        detect_start = time()
        ret, frame = cap.read()
        if not ret:
            break
        if get_aim is False:
            # (xi,yi,wi,hi,frame) = capture1_aim()
            (xi,yi,wi,hi,frame) = capture2_aim()
            # print xi,yi,wi,hi
            if (xi,yi,wi,hi)==(0,0,0,0):
                print ("按键退出")
                break
            if (xi,yi,wi,hi)==(-1,-1,-1,-1):
                print ("无目标")
                break
            tracker.init([xi,yi,wi,hi],frame)#核心算法
        t0 = time()
        boundingbox = tracker.update(frame)#核心算法
        t1 = time()
        boundingbox = map(int, boundingbox)
        ###调用云台控制
        while ptz_index is 0:#每4帧计算一次位置
           ptz_index = 1
           start_ptz(boundingbox)
        ptz_index-=1
        ######
        if get_aim is True:
            cv2.rectangle(frame,(boundingbox[0],boundingbox[1]), (boundingbox[0]+boundingbox[2],boundingbox[1]+boundingbox[3]), (0,255,255), 1)
#            duration = 0.8*duration + 0.2*(t1-t0)
            duration = t1-t0
            cv2.putText(frame, 'FPS: '+str(1/duration)[:4].strip('.'), (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        if get_aim is True:
            [xn,yn,wn,hn] = boundingbox
            if count == 0 or (xn-0<-int(wn/1.5)) | (yn-0<-int(hn/1.5)) | (size[0]-xn<int(wn/1.5)) | (size[1]-yn<int(hn/1.5)):
                None
                # get_aim = False
        cv2.rectangle(frame,(centerx-10,centery-10),(centerx+20,centery+20),(255,255,255),1,1)
        cv2.imshow('tracking', frame)
        c = cv2.waitKey(interval) & 0xFF
        if c== 27:
#            break
            break
        detect_stop=time()
        run_time=time()
        print (detect_stop-detect_start)
        if run_time-start_time > 60:
            break
    # ptz_handle(com0,move_stop)
    # ti.sleep(1)
    # ptz_exit(com0)
    cap.release()
    cv2.destroyAllWindows()
