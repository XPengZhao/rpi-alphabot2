#!/usr/bin/python3
# -*- coding:utf-8 -*-
import sys
sys.path.append("..")
from Lib.Lib import Drive
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
value = [0]*(6)
mycar = Drive()
mycar.MHSensor_Init()
while 1:
    value = mycar.MHSensor()
    print(value)
    time.sleep(0.5)

