#!/usr/bin/python3 
# -*- coding: utf-8 -*-
import socket
from Lib.Lib import Drive
import time
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BCM)
mycar = Drive()
mycar.Motor_Init()

class Ultra_Thread(threading.Thread):
    def __init__(self):
        super(Ultra_Thread, self).__init__()
    def run(self):
        myultra = Drive()
        myultra.Ultrasonic_Init()
        myultra.Buffer_Init()
        while True:
           dis = myultra.Ultrasonic();
           if dis == 10:
               myultra.Stop()
               myultra.Buffer_ON()
               sleep(1)
               myultra.Buffer_OFF()
           sleep(0.01)
          


t1 = Ultra_Thread()
t1.start()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 7777, ))
s.listen(5)
cli, addr = s.accept()
temp = 0
while True:
    if temp != '1':
        temp = cli.recv(1)
        temp = temp.decode()
    if temp:
        if temp == '1':
            while True:
                temp =cli.recv(1)
                temp = temp.decode()
                if temp:
                    if temp == 'a':
                        mycar.GoStraight()
                    elif temp == 'b':
                        mycar.GoBack()
                    elif temp == 'c':
                        mycar.TurnLeft()
                    elif temp == 'd':
                        mycar.TurnRight()
                    elif temp == 's':
                        mycar.Stop()
                    elif temp == '2':
                        mycar.Stop()
                        break
                    else:
                        mycar.Stop()
                    # print(temp)
                else:
                    mycar.Stop()
                    s.close()
                    break
        if temp == '2':
            while True:
                temp = cli.recv(1)
                temp = temp.decode()
                if temp:
                    if temp == '1':
                        break;

    else:
        mycar.Stop()
        s.close()
        break
