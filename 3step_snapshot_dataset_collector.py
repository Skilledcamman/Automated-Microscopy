import cv2
import datetime
import os
import serial
import time

# Camera and serial settings
CAM_INDEX = 0
SERIAL_PORT = '/dev/serial0'  # Change to your COM port, e.g., 'COM3' on Windows
BAUDRATE = 9600
STEP_SIZE = 50
SAVE_DIR = 'snapshots'

# Create directory if it doesn't exist
os.makedirs(SAVE_DIR, exist_ok=True)

def arduino_send(cmd, ser, wait=1):
    ser.write((cmd.strip() + '\n').encode('ascii'))
    time.sleep(wait)
    lines = []
    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors='ignore').strip()
        except Exception:
            break
        if line:
            lines.append(line)
    return '\n'.join(lines)

import re
def get_position_from_response(resp):
    match = re.search(r'Position:\s*(\d+)', resp)
    if match:
        return int(match.group(1))
    return None

def confirm_move(ser, timeout=8, prev_position=None):
    # Wait for Arduino to confirm move by detecting Position value change
    start = time.time()
    last_position = prev_position
    while time.time() - start < timeout:
        ser.write(b'Q\n')
        time.sleep(0.5)
        resp = ''
        while ser.in_waiting:
            try:
                line = ser.readline().decode(errors='ignore').strip()
            except Exception:
                break
            if line:
                resp += line + '\n'
        if resp:
            print('Arduino response:', repr(resp))
            pos = get_position_from_response(resp)
            if last_position is not None and pos is not None and pos != last_position:
                print(f'Position changed: {last_position} -> {pos}')
                return pos
            elif last_position is None and pos is not None:
                last_position = pos
    print('Warning: Move not confirmed by Arduino.')
    return last_position

# Open camera
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

if not cap.isOpened():
    print('Error: Could not open camera.')
    exit(1)

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=2)
    time.sleep(2)
except Exception as e:
    print(f'Error: Could not open serial port: {e}')
    cap.release()
    exit(1)

def take_snapshot(label):
    ret, frame = cap.read()
    if not ret:
        print('Failed to grab frame')
        return
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = os.path.join(SAVE_DIR, f'snapshot_{label}_{timestamp}.jpg')
    cv2.imwrite(filename, frame)
    print(f'Saved: {filename}')


print('Taking 3 snapshots: normal, +50 steps, -50 steps')

# 1. Take normal position snapshot and get initial position
print('Moving to normal position...')
ser.write(b'Q\n')
time.sleep(0.5)
resp = ''
while ser.in_waiting:
    try:
        line = ser.readline().decode(errors='ignore').strip()
    except Exception:
        break
    if line:
        resp += line + '\n'
init_position = get_position_from_response(resp)
print(f'Initial position: {init_position}')
take_snapshot('normal')

# 2. Move +50 steps and take snapshot
print('Moving +50 steps...')
arduino_send(f'S{STEP_SIZE}', ser)
arduino_send('U', ser)
pos_plus = confirm_move(ser, prev_position=init_position)
take_snapshot('plus50')

# 3. Move -100 steps (to -50 from normal) and take snapshot
print('Moving -100 steps (to -50 from normal)...')
arduino_send(f'S{STEP_SIZE*2}', ser)
arduino_send('D', ser)
pos_minus = confirm_move(ser, prev_position=pos_plus)
take_snapshot('minus50')

# 4. Move +50 steps to return to normal
print('Returning to normal position...')
arduino_send(f'S{STEP_SIZE}', ser)
arduino_send('U', ser)
confirm_move(ser, prev_position=pos_minus)

cap.release()
ser.close()
cv2.destroyAllWindows()
print('Done.')
