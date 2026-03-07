from vpython import *
import serial
import time

# --- CONFIGURATION ---
PORT = 'COM4'
BAUD = 115200
ALPHA = 0.98
dt = 0.04

# --- SCENE SETUP ---
scene = canvas(title="IMU 3D Orientation (Fixed & Labeled)", width=800, height=600, background=color.black)
board = box(length=4, height=0.2, width=2, color=color.cyan, opacity=0.8)

# --- RE-ADDING AXIS ARROWS & LABELS ---
# X-Axis (Red) - Usually Roll
arrow(axis=vector(3,0,0), color=color.red, shaftwidth=0.05)
label(pos=vector(3.2,0,0), text='X (Roll)', height=15, box=False, color=color.red)

# Y-Axis (Green) - Usually Yaw
arrow(axis=vector(0,3,0), color=color.green, shaftwidth=0.05)
label(pos=vector(0,3.2,0), text='Y (Yaw)', height=15, box=False, color=color.green)

# Z-Axis (Blue) - Usually Pitch
arrow(axis=vector(0,0,3), color=color.blue, shaftwidth=0.05)
label(pos=vector(0,0,3.2), text='Z (Pitch)', height=15, box=False, color=color.blue)

# --- CALIBRATION VARIABLES ---
is_calibrated = False
gyro_offsets = [0.0, 0.0, 0.0]
samples_to_take = 50
samples_count = 0
roll, pitch, yaw = 0, 0, 0

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.dtr = True
    time.sleep(2)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Connection Error: {e}"); exit()

print("KEEP STILL - CALIBRATING...")

while True:
    rate(100)

    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            data = [float(x) for x in line.split(',')]

            if len(data) == 6:
                # --- PHASE 1: CALIBRATION ---
                if not is_calibrated:
                    gyro_offsets[0] += data[3]
                    gyro_offsets[1] += data[4]
                    gyro_offsets[2] += data[5]
                    samples_count += 1
                    if samples_count >= samples_to_take:
                        gyro_offsets = [x / samples_to_take for x in gyro_offsets]
                        is_calibrated = True
                        print("Calibrated! Running...")
                    continue

                # --- PHASE 2: NORMAL OPERATION ---
                # 1. Subtract Bias
                gx = data[3] - gyro_offsets[0]
                gy = data[4] - gyro_offsets[1]
                gz = data[5] - gyro_offsets[2]
                ax, ay, az = data[0], data[1], data[2]

                # 2. Calc Accel Angles
                acc_roll = atan2(ay, az)
                # We add a '-' here if the gravity vector math is inverted for your mount
                acc_pitch = atan2(-ax, sqrt(ay**2 + az**2))

                # 3. Filter & Integrate
                roll = ALPHA * (roll + gx * dt) + (1 - ALPHA) * acc_roll
                pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * acc_pitch

                if abs(gz) < 0.005: gz = 0
                yaw += gz * dt

                # --- 4. APPLY INVERSIONS ---
                # Pitch was inverted, so we multiply by -1 here
                roll_final  = roll * -1
                pitch_final = pitch
                yaw_final   = yaw

                # 5. Update 3D Object
                board.up = vector(0, 1, 0)
                board.axis = vector(1, 0, 0)

                board.rotate(angle=yaw_final,   axis=vector(0, 1, 0))
                board.rotate(angle=pitch_final, axis=vector(0, 0, 1))
                board.rotate(angle=roll_final,  axis=vector(1, 0, 0))

        except Exception as e:
            pass