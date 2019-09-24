#!/usr/bin/python3
# -*- coding:utf-8 -*-
from Lib.Lib import Drive
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
mycar = Drive()
mycar.Motor_Init()
mycar.MHSensor_Init()
value = [0]*(6)
def pos_control(pp):
    if(value[1] == 1 and value[2] == 0):
        mycar.LMotorRun(-15-pp)
        mycar.RMotorRun(-15+pp)
    elif(value[1] == 0 and value[2] == 1):
        mycar.LMotorRun(-15+pp)
        mycar.RMotorRun(-15-pp)
    else:
        mycar.LMotorRun(-15)
        mycar.RMotorRun(-15)
 
    return
while 1:
    value = mycar.MHSensor()
    pos_control(10)
