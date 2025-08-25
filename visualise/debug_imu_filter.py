#!/usr/bin/env python3
"""
Simple IMU Quaternion Filter
- Read serial data (blocking)
- Update quaternion with C functions
- Update 3D visualization  
- Repeat
"""

import ctypes
import serial
import time
import sys
from vpython import *
import ctype_bypass

# Setup serial
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ser = serial.Serial(port, 115200, timeout=1)
time.sleep(5)  # Wait for connection


# Initialize quaternion
q = ctype_bypass.Quaternion(1.0, 0.0, 0.0, 0.0)

print(f"Reading from {port}...")

# Initialize
last_time = time.time()

def update_q():
    try:
        line = ser.readline().decode().strip()
        if line:
            values = [float(x) for x in line.split(',')]
            if len(values)==6:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                gyro = values[3:]
                print(values)
                gyro_rates = (ctypes.c_float * 3)(gyro[0], gyro[1], gyro[2])
                ctype_bypass.qlib.updateQuaternionFromRate(ctypes.byref(q), gyro_rates, dt)
    except:
        pass
"""
# Main loop
while True:
    # Read serial data (blocking)
    line = ser.readline().decode().strip()
    
    try:
        # Parse: ax,ay,az,wx,wy,wz
        values = [float(x) for x in line.split(',')]
        if len(values) == 6:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            print(f"After gyro: {q}")
            accel = values[:3]
            gyro = values[3:]

            gyro_rates = (ctypes.c_float * 3)(gyro[0], gyro[1], gyro[2])
            accel_data = (ctypes.c_float * 3)(accel[0], accel[1], accel[2])
            ctype_bypass.qlib.updateQuaternionFromRate(ctypes.byref(q), gyro_rates, dt)
            #ctype_bypass.qlib.updateQuaternionFromAcc(ctypes.byref(q), accel_data)         
    except:
        pass  # Skip bad data

"""


import numpy as np
import math
time.sleep(1)

scene.range=5
scene.background=color.yellow
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)

scene.width=1200
scene.height=1080

xarrow=arrow(lenght=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(lenght=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))

frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))

bBoard=box(length=6,width=2,height=.2,opacity=.8,pos=vector(0,0,0,))
bn=box(length=1,width=.75,height=.1, pos=vector(-.5,.1+.05,0),color=color.blue)
nano=box(lenght=1.75,width=.6,height=.1,pos=vector(-2,.1+.05,0),color=color.green)
myObj=compound([bBoard,bn,nano])
while (True):
    try:
        rate(60)
        time.sleep(0.1)
        update_q()
        q0=q.w
        q1=-q.x
        q2=-q.y
        q3=-q.z

        roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=math.asin(2*(q0*q2-q3*q1))
        yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2

        rate(50)
        k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y=vector(0,1,0)
        s=cross(k,y)
        v=cross(s,k)
        vrot=v*cos(roll)+cross(k,v)*sin(roll)

        frontArrow.axis=k
        sideArrow.axis=cross(k,vrot)
        upArrow.axis=vrot
        myObj.axis=k
        myObj.up=vrot
        sideArrow.length=2
        frontArrow.length=4
        upArrow.length=1
    except KeyboardInterrupt:
        break
        
