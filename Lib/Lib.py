#!/usr/bin/python3
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

class Drive(object):

    #MH_Sensor Variable
    MH_CS = 5
    MH_CLOCK = 25
    MH_ADDRESS = 24
    MH_DATAOUT = 23
    
    # PWMA-->P6 AIN1-->P12 AIN2-->P13
    # PWMB-->P26 BIN1-->P20 BIN2-->P21
    A1 = 12 
    A2 = 13
    CONTROLA = 6
    B1 = 20
    B2 = 21
    CONTROLB = 26

    #Buffer Pin
    BUFFER_PIN = 4

    #Ultrasonic
    TRIG = 22
    ECHO = 27

    def All_Init(self):
        MHSensor_Init()


    def MHSensor_Init(self):
        GPIO.setup((self.MH_CS,self.MH_CLOCK, self.MH_ADDRESS),GPIO.OUT)
        GPIO.setup(self.MH_DATAOUT,GPIO.IN,GPIO.PUD_UP)



    def MHSensor(self):
        value = [0]*(6)
        #Read Channel0 ~ channel5 AD value
        for j in range (0,6):
            GPIO.output(self.MH_CS,GPIO.LOW)
            for i in range(0,10):
                #sent 4-bit Address
                if (i < 4):
                    bit = (((j) >> (3 - i)) & 0x01)
                    GPIO.output(self.MH_ADDRESS,bit)
                #read 10-bit data
                value[j] <<= 1
                value[j] |= GPIO.input(self.MH_DATAOUT)
                GPIO.output(self.MH_CLOCK,GPIO.HIGH)
                GPIO.output(self.MH_CLOCK,GPIO.LOW)

            GPIO.output(self.MH_CS,GPIO.HIGH)
            time.sleep(0.0001)
        for k in range (0,6):
            if (value[k] < 200):
                value[k] = 0
            else:
                value[k] = 1
        return value [1:]   # invalid address for channel 0



    def Ultrasonic_Init(self):
        GPIO.setup(self.TRIG,GPIO.OUT)
        GPIO.setup(self.ECHO,GPIO.IN)


    def Ultrasonic(self):
        GPIO.output(self.TRIG,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TRIG,GPIO.LOW)

        GPIO.wait_for_edge(self.ECHO,GPIO.RISING)
        t1 = time.time()
        while GPIO.input(self.ECHO) == GPIO.HIGH:
            pass
        t2 = time.time()
        return (t2-t1)*34000/2

    def Buffer_Init(self):
        GPIO.setup(self.BUFFER_PIN, GPIO.OUT)
    
    def Buffer_ON(self):
        GPIO.output(self.BUFFER_PIN, GPIO.HIGH)

    def Buffer_OFF(self):
        GPIO.output(self.BUFFER_PIN, GPIO.LOW)
    
    def Motor_Init(self):
        GPIO.setup((self.A1,self.A2,self.CONTROLA),GPIO.OUT)
        GPIO.setup((self.B1,self.B2,self.CONTROLB),GPIO.OUT)
        self.pl = GPIO.PWM(self.CONTROLA,1000)
        self.pr = GPIO.PWM(self.CONTROLB,1000)
        self.pl.start(0)
        self.pr.start(0)

    def LMotorRun(self,speed):
        if speed >= 0:
            GPIO.output((self.A1,self.A2),(1,0))
            self.pl.ChangeDutyCycle(speed)
        else:
            GPIO.output((self.A1,self.A2),(0,1))
            self.pl.ChangeDutyCycle(-speed)
 
    def RMotorRun(self,speed):
        if speed >= 0:
            GPIO.output((self.B1,self.B2),(1,0))
            self.pr.ChangeDutyCycle(speed)
        else:
            GPIO.output((self.B1,self.B2),(0,1))
            self.pr.ChangeDutyCycle(-speed)

    def GoStraight(self):
        self.LMotorRun(15)
        self.RMotorRun(15)

    def GoBack(self):
        self.LMotorRun(-15)
        self.RMotorRun(-15)
 
    def Stop(self):
        self.LMotorRun(0)
        self.RMotorRun(0)
    
    def TurnLeft(self):
        self.LMotorRun(-15)
        self.RMotorRun(15)

    def TurnRight(self):
        self.LMotorRun(15)
        self.RMotorRun(-15)






