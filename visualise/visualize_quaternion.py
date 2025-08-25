from vpython import *
import subprocess
import numpy as np
import time
import sys

# Set up the scene
scene.title = "REAL MPU6050 Quaternion Filter Visualization"
scene.width = 1200
scene.height = 800
scene.background = color.black
scene.forward = vector(-1, -1, -1)
scene.range = 3

# Create coordinate axes
x_axis = arrow(pos=vector(0,0,0), axis=vector(2,0,0), shaftwidth=0.1, color=color.red, opacity=0.7)
y_axis = arrow(pos=vector(0,0,0), axis=vector(0,2,0), shaftwidth=0.1, color=color.green, opacity=0.7)
z_axis = arrow(pos=vector(0,0,0), axis=vector(0,0,2), shaftwidth=0.1, color=color.blue, opacity=0.7)

# Create labels
label(pos=vector(2.2,0,0), text='X', color=color.red, height=20)
label(pos=vector(0,2.2,0), text='Y', color=color.green, height=20)
label(pos=vector(0,0,2.2), text='Z', color=color.blue, height=20)

# Create the IMU representation (like a real MPU6050 breakout board)
board = box(length=4, width=2, height=0.2, color=color.gray(0.2), pos=vector(0,0,0))
chip = box(length=1, width=0.8, height=0.15, color=color.purple, pos=vector(0, 0, 0.175))
pin_strip1 = box(length=0.3, width=1.8, height=0.1, color=color.yellow, pos=vector(-1.7, 0, 0.1))
pin_strip2 = box(length=0.3, width=1.8, height=0.1, color=color.yellow, pos=vector(1.7, 0, 0.1))

# Combine into a compound object
imu = compound([board, chip, pin_strip1, pin_strip2])

# Create orientation arrows on the IMU
forward_arrow = arrow(pos=vector(0,0,0), axis=vector(2,0,0), shaftwidth=0.15, 
                     color=color.orange, length=2)
up_arrow = arrow(pos=vector(0,0,0), axis=vector(0,1,0), shaftwidth=0.15, 
                color=color.magenta, length=1.5)
side_arrow = arrow(pos=vector(0,0,0), axis=vector(0,0,1), shaftwidth=0.15, 
                  color=color.cyan, length=1.8)

# Add labels for IMU axes
label(pos=vector(2.5,0,0), text='Forward', color=color.orange, height=15)
label(pos=vector(0,2,0), text='Up', color=color.magenta, height=15)
label(pos=vector(0,0,2.3), text='Right', color=color.cyan, height=15)

# Status display
status_label = label(pos=vector(-3, 3, 0), text='Starting Real IMU...', 
                    color=color.white, height=20, box=False)

# Info label
info_label = label(pos=vector(-3, -3, 0), 
                  text='🚀 REAL MPU6050 DATA! 🚀\nMove your sensor to see the magic!', 
                  color=color.green, height=15, box=False)

def quaternion_to_rotation_vectors(q):
    """Convert quaternion to rotation vectors for visualization"""
    w, x, y, z = q
    
    # Normalize quaternion
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # Convert to rotation matrix elements we need
    # Forward vector (x-axis of IMU)
    forward = vector(
        1 - 2*(y*y + z*z),
        2*(x*y + w*z),
        2*(x*z - w*y)
    )
    
    # Up vector (y-axis of IMU) 
    up = vector(
        2*(x*y - w*z),
        1 - 2*(x*x + z*z), 
        2*(y*z + w*x)
    )
    
    # Right vector (z-axis of IMU)
    right = vector(
        2*(x*z + w*y),
        2*(y*z - w*x),
        1 - 2*(x*x + y*y)
    )
    
    return forward, up, right

def start_real_imu_program(device):
    """Start the real IMU C program and return the process"""
    try:
        # Start the real IMU stream program
        cmd = ['./build/real_imu_stream', device]
        process = subprocess.Popen(cmd, 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 universal_newlines=True,
                                 bufsize=1)
        return process
    except FileNotFoundError:
        print(f"Error: Could not find ./build/real_imu_stream")
        print("Make sure to build it first with:")
        print("cd build && make")
        return None

# Get serial device from command line or use default
import sys
device = '/dev/ttyACM0'  # Default for Raspberry Pi Pico
if len(sys.argv) > 1:
    device = sys.argv[1]

print(f"Starting REAL IMU visualization from {device}...")
print("Expected IMU data format: ax,ay,az,wx,wy,wz")

# Start the C program
process = start_real_imu_program(device)

if not process:
    print("Failed to start real IMU program. Exiting.")
    print(f"Make sure your device is connected to {device}")
    print("Common devices: /dev/ttyACM0, /dev/ttyUSB0, /dev/ttyUSB1")
    sys.exit(1)

frame_count = 0
start_time = time.time()

print("🚀 REAL MPU6050 Visualization started!")
print("Move your sensor around to see it in action!")
print("Press Ctrl+C to stop.")

try:
    while True:
        rate(100)  # Limit to 100 FPS
        
        # Read quaternion data from C program
        line = process.stdout.readline()
        if not line:
            # Check if process has errors
            if process.poll() is not None:
                error = process.stderr.read()
                print(f"Process ended with error: {error}")
                break
            continue
            
        try:
            # Parse quaternion data
            parts = line.strip().split(',')
            if len(parts) == 4:
                q = [float(p) for p in parts]
                w, x, y, z = q
                
                # Convert quaternion to rotation vectors
                forward, up, right = quaternion_to_rotation_vectors(q)
                
                # Update IMU orientation
                imu.axis = forward
                imu.up = up
                
                # Update orientation arrows
                forward_arrow.axis = forward * 2
                up_arrow.axis = up * 1.5  
                side_arrow.axis = right * 1.8
                
                # Update status
                frame_count += 1
                if frame_count % 50 == 0:  # Update every 50 frames
                    fps = frame_count / (time.time() - start_time)
                    status_label.text = f'FPS: {fps:.1f} | Real IMU q: [{w:.3f}, {x:.3f}, {y:.3f}, {z:.3f}]'
                
        except (ValueError, IndexError) as e:
            print(f"Error parsing quaternion data: {line.strip()} - {e}")
            continue
            
except KeyboardInterrupt:
    print("\n🛑 Stopping REAL IMU visualization...")
    
finally:
    # Clean up
    if process:
        process.terminate()
        process.wait()
    print("✅ Real IMU visualization stopped!")
