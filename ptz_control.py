# -*- coding: utf-8 -*-
import cv2
import sys
import serial
import numpy as np
import time as ti
#云台参量
preset=0x00
move_u=[0xff,0x01,0x00,0x08,0x00,0xff,0x08]
move_d=[0xff,0x01,0x00,0x10,0x00,0xff,0x10]
move_l=[0xff,0x01,0x00,0x04,0xff,0x00,0x04]
move_r=[0xff,0x01,0x00,0x02,0xff,0x00,0x02]
set_pre=[0xff,0x01,0x00,0x03,0x00,preset,(0x04+preset)%0x100]
run_pre=[0xff,0x01,0x00,0x07,0x00,preset,(0x08+preset)%0x100]
remove_pre=[0xff,0x01,0x00,0x05,0x00,preset,(0x06+preset)%0x100]
move_stop=[0xff,0x01,0x00,0x00,0x00,0x00,0x01]
speed_value={'1':0x0f,'2':0x10,'3':0x11}
move_h={'True':move_r,'False':move_l}
move_v={'True':move_d,'False':move_u}
vtime=0.03#垂直运动时间
htime=00

def ptz_init(portnum):
    
    port0=serial.Serial(portnum,baudrate=2400,bytesize=8,stopbits=1)
    return port0
    
def ptz_exit(port0):
    
    port0.close()
    
def ptz_handle(port0,move_order):

    port0.write(move_order)
#    ti.sleep(0.02)#注意：串口的两句写命令一定要留出间隔时间，否则上一句命令将被直接覆盖

def set_speed(port0,speed):
    global set_pre,run_pre,remove_pre
    set_pre=[0xff,0x01,0x00,0x03,0x00,speed,(0x04+speed)%0x100]
    run_pre=[0xff,0x01,0x00,0x07,0x00,speed,(0x08+preset)%0x100]
    remove_pre=[0xff,0x01,0x00,0x05,0x00,speed,(0x06+speed)%0x100]
    ptz_handle(port0,set_pre)
    ptz_handle(port0,run_pre)

def recover_speed(port0):
    ptz_handle(port0,remove_pre)

##两个方向同时转动需要结合两帧之间的时间差进行水平与垂直方向的运动时间分配
def ptz2direction(port0,xsign,ysign):
    ptz_handle(port0,move_v[ysign])
    ti.sleep(vtime)
    ptz_handle(port0,move_h[xsign])

def ptz_v(port0,ysign):
    ptz_handle(port0,move_v[ysign])
    
def ptz_h(port0,xsign):
    ptz_handle(port0,move_h[xsign])

