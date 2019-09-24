#!/usr/bin/python3
import tkinter as tk
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect(('192.168.208.124', 9999))
s.connect(('127.0.0.1', 7777))

def Up():
    a = 'a'
    s.send(a.encode())

def Down():
    a = 'b'
    s.send(a.encode())

def Left():
    a = 'c'
    s.send(a.encode())

def Right():
    a = 'd'
    s.send(a.encode())

def Stop():
    a = 's'
    s.send(a.encode())

def CtrlMode():
    a = '1'
    s.send(a.encode())

def PatrolMode():
    a = '2'
    s.send(a.encode())

win = tk.Tk()
win.geometry('300x500') # width x height
win.title('control GUI')

f_control = tk.Frame(height = 250, width =300)
f_vedio = tk.Frame(height = 250, width=300)

button_up = tk.Button(f_control, text = 'Up', command = Up, width = 3,height = 1, repeatdelay=10, repeatinterval=10)
button_down = tk.Button(f_control, text = 'Down', command = Down, width = 3,height = 1)
button_left = tk.Button(f_control, text = 'Left', command = Left, width = 3,height = 1)
button_right = tk.Button(f_control, text = 'Right', command = Right, width = 3,height = 1)
button_stop = tk.Button(f_control, text = 'Stop', command = Stop, width = 8, height = 3)
button_ctrl = tk.Button(f_control, text = 'Control Mode', command = CtrlMode, width = 8, height = 2)
button_patrol = tk.Button(f_control, text = 'Patrol Mode', command = PatrolMode, width = 8, height = 2)

f_control.grid(row = 1, column = 0)
f_vedio.grid(row = 0, column =0)
button_up.grid(row = 0, column = 2)
button_down.grid(row = 4, column = 2)
button_left.grid(row = 2, column = 0)
button_right.grid(row = 2, column = 4)
button_stop.grid(row = 8, column = 2)
button_ctrl.grid(row = 6, column = 0, sticky = tk.S)
button_patrol.grid(row = 6, column = 4, sticky = tk.S)


tk.mainloop()

