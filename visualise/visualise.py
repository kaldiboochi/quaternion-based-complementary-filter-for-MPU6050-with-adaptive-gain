import threading, time, queue, ctypes, serial, sys, math
from vpython import vector, box, arrow, rate, scene, color, compound, cos , sin , cross
from quaternion_ctypes import Quaternion, qlib
from quaternion import quaternion
import numpy as np
data_q = queue.Queue(maxsize=1)
quat_q = queue.Queue(maxsize=1)

def serial_reader(port):
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)
    while True:
        line = ser.readline().decode().strip()
        if line:
            data_q.put(line)

def filter_thread():
    last = time.time()
    q = Quaternion()
    while True:
        try:
            line = data_q.get()
            vals = [float(x) for x in line.split(',')]
            #vals = [0,0,0,1,1,1]
            if len(vals) == 6:
                now = time.time()
                dt = now - last
                last = now
                mg = (ctypes.c_float*3)(*vals[3:])
                ac = (ctypes.c_float*3)(*vals[:3])
                #for w in mg:
                #    print(f"{round(float(w),2)}",end="     ")
                #print("w")
                qptr = ctypes.byref(q)
                qlib.cf_update_from_gyro(qptr, mg, dt)
                qlib.cf_update_from_accel(qptr, ac)
                quat_q.put((q.w, q.x, q.y, q.z))
        except:
            pass

def main(port):
    threading.Thread(target=serial_reader, args=(port,), daemon=True).start()
    threading.Thread(target=filter_thread, daemon=True).start()


    scene.range=5
    scene.background=color.yellow
    toRad=2*np.pi/360
    toDeg=1/toRad
    scene.forward=vector(-1,-1,-1)
    scene.up=vector(0,0,1)
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
    px = quaternion(0, 1, 0, 0)
    py = quaternion(0, 0, 1, 0)
    pz = quaternion(0, 0, 0, 1)
    try:
        while True:
            q = quat_q.get()
            q0=q[0]
            q1=q[1]
            q2=q[2]
            q3=q[3]
            att = quaternion(q0, q1, q2, q3)
            c_att = att.conjugate()
            px_rot = (att * px * c_att).conjugate()
            py_rot = (att * py * c_att).conjugate()
            pz_rot = (att * pz * c_att).conjugate()
            
            x=vector(px_rot.x,px_rot.y,px_rot.z)
            y=vector(py_rot.x,py_rot.y,py_rot.z)
            z=vector(pz_rot.x,pz_rot.y,pz_rot.z)
            frontArrow.axis=x
            sideArrow.axis=y
            upArrow.axis=z
            myObj.axis=y
            myObj.up=z
            sideArrow.length=2
            frontArrow.length=4
            upArrow.length=1
            
            
            
            
            
            
    except KeyboardInterrupt:
        ser.close()
        print("Exiting...")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv)>1 else "/dev/ttyACM0"
    main(port)
