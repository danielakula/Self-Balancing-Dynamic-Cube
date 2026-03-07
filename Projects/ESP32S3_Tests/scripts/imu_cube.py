from vpython import *
import serial
import time

# --- CONFIGURATION ---
PORT = 'COM4'
BAUD = 115200
ALPHA = 0.98
dt = 0.04
SIDE = 2  # Cube side length

# --- SCENE SETUP ---
scene = canvas(title="Cubli Corner Rotation", width=800, height=600, background=color.black)
scene.background = color.gray(0.3)

# Offset cube so one corner sits at the origin
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

# Set initial orientation ONCE - never touch axis/up again after this
board.axis = vector(1, 0, 0)
board.up   = vector(0, 1, 0)

# Pivot corner marker
sphere(pos=vector(0, 0, 0), radius=0.08, color=color.yellow)

# --- AXIS ARROWS & LABELS ---
arrow(axis=vector(3, 0, 0), color=color.red,   shaftwidth=0.05)
label(pos=vector(3.3, 0, 0), text='X (Roll)',  height=15, box=False, color=color.red)
arrow(axis=vector(0, 3, 0), color=color.green, shaftwidth=0.05)
label(pos=vector(0, 3.3, 0), text='Y (Yaw)',  height=15, box=False, color=color.green)
arrow(axis=vector(0, 0, 3), color=color.blue,  shaftwidth=0.05)
label(pos=vector(0, 0, 3.3), text='Z (Pitch)', height=15, box=False, color=color.blue)

# --- CALIBRATION VARIABLES ---
is_calibrated = False
gyro_offsets = [0.0, 0.0, 0.0]
samples_to_take = 50
samples_count = 0
roll, pitch, yaw = 0.0, 0.0, 0.0

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.dtr = True
    time.sleep(2)
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()  # Second flush to clear bootloader junk
except Exception as e:
    print(f"Connection Error: {e}"); exit()

print("KEEP STILL - CALIBRATING...")

# --- MAIN LOOP ---
while True:
    rate(100)
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            data = [float(x) for x in line.split(',')]
            if len(data) != 6:
                continue

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

            # 1. Subtract gyro bias
            gx = data[3] - gyro_offsets[0]
            gy = data[4] - gyro_offsets[1]
            gz = data[5] - gyro_offsets[2]
            ax, ay, az = data[0], data[1], data[2]

            # 2. Accelerometer angles
            acc_roll  = atan2(ay, az)
            acc_pitch = atan2(-ax, sqrt(ay**2 + az**2))

            # 3. Complementary filter
            roll  = ALPHA * (roll  + gx * dt) + (1 - ALPHA) * acc_roll
            pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * acc_pitch
            if abs(gz) < 0.005:
                gz = 0
            yaw += gz * dt

            # 4. Apply inversions (tune these for your IMU mount)
            roll_final  = roll * 1
            pitch_final = pitch * -1
            yaw_final   = yaw

            # 5. Rotate fresh basis vectors by current Euler angles
            #    Order: yaw -> pitch -> roll (applied to each basis vector)
            x_axis = vector(1, 0, 0)
            y_axis = vector(0, 1, 0)

            x_axis = x_axis.rotate(angle=yaw_final,   axis=vector(0, 1, 0))
            x_axis = x_axis.rotate(angle=pitch_final, axis=vector(0, 0, 1))
            x_axis = x_axis.rotate(angle=roll_final,  axis=vector(1, 0, 0))

            y_axis = y_axis.rotate(angle=yaw_final,   axis=vector(0, 1, 0))
            y_axis = y_axis.rotate(angle=pitch_final, axis=vector(0, 0, 1))
            y_axis = y_axis.rotate(angle=roll_final,  axis=vector(1, 0, 0))

            # 6. Rotate the corner offset vector by the same rotation
            #    so the pinned corner stays at the origin
            offset = initial_offset.rotate(angle=yaw_final,   axis=vector(0, 1, 0))
            offset = offset.rotate(angle=pitch_final, axis=vector(0, 0, 1))
            offset = offset.rotate(angle=roll_final,  axis=vector(1, 0, 0))

            # 7. Apply to board — dimensions never touched, only axis/up/pos
            board.axis = x_axis
            board.up   = y_axis
            board.pos  = offset

        except Exception as e:
            pass