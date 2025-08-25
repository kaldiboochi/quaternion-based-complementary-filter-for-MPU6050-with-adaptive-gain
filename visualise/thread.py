import threading
import time
import queue
import ctypes
import serial
import sys
from vpython import *
import ctype_bypass

# Thread-safe queue for communication between threads
data_queue = queue.Queue(maxsize = 5)
# Initialize quaternion

quaternion_que = queue.Queue(maxsize = 5)

def serial_reader(ser):
    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                data_queue.put(line)
        except Exception as e:
            print(f"Error reading serial: {e}")

def data_printer():
    last_time = time.time()
    q = ctype_bypass.Quaternion(1.0, 0.0, 0.0, 0.0)
    while True:
        try:
            line = data_queue.get()  # Blocks until there is data
            values = [float(x) for x in line.split(',')]
            if len(values)==6:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                gyro = values[3:]
                gyro_rates = (ctypes.c_float * 3)(gyro[0], gyro[1], gyro[2])
                ctype_bypass.qlib.updateQuaternionFromRate(ctypes.byref(q), gyro_rates, dt)
                quaternion_que.put([q.w , q.x , q.y , q.z])
                print([q.w , q.x , q.y , q.z])
        except Exception as e:
            print(f"Error printing data: {e}")
import numpy as np
import math
def main():
    # Change this to your serial port
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)  # Give time for the serial connection to initialize
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

    # Create threads
    thread_reader = threading.Thread(target=serial_reader, args=(ser,))
    thread_printer = threading.Thread(target=data_printer)

    # Start threads
    thread_reader.daemon = True
    thread_printer.daemon = True
    thread_reader.start()
    thread_printer.start()

    # Keep main thread alive
    try:
        while True:
            q = quaternion_que.get()
            q0=q[0]
            q1=q[1]
            q2=q[2]
            q3=q[3]

            roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
            pitch=math.asin(2*(q0*q2-q3*q1))
            yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
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
        ser.close()
        print("Exiting...")

if __name__ == '__main__':
    main()

