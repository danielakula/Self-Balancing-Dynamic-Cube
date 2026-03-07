import serial
import time
from vpython import *

# --- CONFIGURATION ---
PORT = 'COM4'
BAUD = 115200
SIDE = 2

# --- SCENE SETUP ---
scene = canvas(title="Cubli Quaternion Rotation (Mapped)", width=800, height=600, background=color.gray(0.3))

# Pivot point is at (0,0,0). We offset the box so the corner is the pivot.
initial_offset = vector(SIDE/2, SIDE/2, SIDE/2)

_box = box(
    pos=initial_offset,
    length=SIDE, height=SIDE, width=SIDE,
    color=color.cyan,
    opacity=1.0
)
board = compound([_box])
board.axis = vector(1, 0, 0)
board.up   = vector(0, 1, 0)

# Visual Indicators
sphere(pos=vector(0, 0, 0), radius=0.08, color=color.yellow)
arrow(axis=vector(3, 0, 0), color=color.red,   shaftwidth=0.05)
label(pos=vector(3.3, 0, 0), text='X (Roll)',   height=15, box=False, color=color.red)
arrow(axis=vector(0, 3, 0), color=color.green, shaftwidth=0.05)
label(pos=vector(0, 3.3, 0), text='Y (Yaw)',   height=15, box=False, color=color.green)
arrow(axis=vector(0, 0, 3), color=color.blue,  shaftwidth=0.05)
label(pos=vector(0, 0, 3.3), text='Z (Pitch)', height=15, box=False, color=color.blue)

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(port=PORT, baudrate=BAUD, timeout=2)
    ser.dtr = True
    time.sleep(2)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Connection Error: {e}"); exit()

def quaternion_to_basis(w, x, y, z):
    # Standard Quaternion to Basis Vector conversion
    x_axis = vector(
        1 - 2*(y*y + z*z),
        2*(x*y + w*z),
        2*(x*z - w*y)
    )
    y_axis = vector(
        2*(x*y - w*z),
        1 - 2*(x*x + z*z),
        2*(y*z + w*x)
    )
    return x_axis, y_axis

def rotate_vector_by_quaternion(v, w, x, y, z):
    # Rotates the position of the cube so it pivots around the corner
    t = vector(
        2 * (y*v.z - z*v.y),
        2 * (z*v.x - x*v.z),
        2 * (x*v.y - y*v.x)
    )
    return vector(
        v.x + w*t.x + y*t.z - z*t.y,
        v.y + w*t.y + z*t.x - x*t.z,
        v.z + w*t.z + x*t.y - y*t.x
    )

print("Running...")

while True:
    rate(100)
    try:
        raw = ser.readline()
        line = raw.decode('utf-8', errors='ignore').strip()

        if not line:
            continue

        data = [float(x) for x in line.strip(',').split(',') if x.strip()]

        if len(data) < 4:
            continue

        # --- THE AXIS REMAPPING BLOCK ---
        # IMU Outputs: raw_w, raw_x, raw_y, raw_z
        # We need to map IMU-Z (Up) to VPython-Y (Up)
        rw, rx, ry, rz = data[0], data[1], data[2], data[3]
        
        # MAPPING: 
        # IMU X -> VPython X
        # IMU Z -> VPython Y (Yaw/Up)
        # IMU Y -> VPython Z (Pitch)
        w = rw
        x = rx
        y = rz    
        z = -ry   # Inverted to maintain a Right-Handed Coordinate System
        # --------------------------------

        # Normalise
        n = sqrt(w*w + x*x + y*y + z*z)
        if n == 0: continue
        w, x, y, z = w/n, x/n, y/n, z/n

        # Calculate Basis and Offset
        x_axis, y_axis = quaternion_to_basis(w, x, y, z)
        offset = rotate_vector_by_quaternion(initial_offset, w, x, y, z)

        if mag(x_axis) > 0.5 and mag(y_axis) > 0.5:
            board.axis = x_axis
            board.up   = y_axis
            board.pos  = offset

    except Exception as e:
        print(f"ERROR: {e}")