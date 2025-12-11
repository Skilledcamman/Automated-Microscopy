# Automated Microscope Platform

This project provides a complete solution for automated microscopy using Arduino-controlled stepper motors and Python-based image acquisition, focus analysis, and web control.

## Main Features

- **Stepper Motor Control**: Arduino sketch for ULN2003 + 28BYJ-48 stepper, serial-controlled. Homing required on each boot; supports movement commands after homing.
- **Autofocus & Image Capture**: Python scripts for autofocus (Tenengrad method), video sweep, and snapshot collection.
- **Web Interface**: Flask app for remote control and live image analysis using YOLO object detection.

## Directory Structure

- `arduino_stepper_controller/arduino_stepper_controller.ino`: Arduino sketch for stepper control.
- `3step_snapshot_dataset_collector.py`: Python script for stepwise image capture.
- `app.py`: Flask web app for microscope control and object detection.
- `batch_rename_images.py`: Utility for renaming image files.
- `requirements.txt`: Python dependencies.

## Hardware Setup

- **Board**: Arduino Uno (or compatible)
- **Libraries**: Built-in `Stepper`, `EEPROM`, `SoftwareSerial`
- **Wiring**: ULN2003 to pins IN1=8, IN3=9, IN2=10, IN4=11. Serial RX=1, TX=2 for external UART.
- **Upload**: Use Arduino IDE to upload `arduino_stepper_controller.ino`.

## Python Environment

Install dependencies (Python 3.10+ recommended):

```
python -m pip install -r requirements.txt
```

Key packages: `opencv-python`, `pyserial`, `numpy`, `Flask`, `ultralytics`

## Usage

### Autofocus Sweep

Run autofocus sweep and save video:

```
python autofocus_tenengrad.py <serial_port> <camera_index> <step_chunk> <video_path> [raise_steps] [total_steps]
```

- `serial_port`: e.g. `COM5`
- `camera_index`: e.g. `0`
- `step_chunk`: steps per movement between frames (e.g. `16`)
- `video_path`: output video file (e.g. `sweep.mp4`)
- `raise_steps` (optional): steps to raise after homing (default `0`)
- `total_steps` (optional): total sweep steps (default `9000`)

Example:
```
python autofocus_tenengrad.py COM5 0 16 sweep.mp4 0 9000
```

**Flow:**
1. Home stepper (`Z<raise>`), set position to 0.
2. Move in steps, record video frames.
3. Compute Tenengrad focus for each frame.
4. Move to best-focus position.

**Notes:**
- Ensure Arduino is powered and connected.
- Camera index may vary; use `0` or list cameras via OS tools.
- Adjust `total_steps` for your objective/limits.

### Web App

Start the Flask web interface:

```
python app.py
```

Access the web UI for live control and object detection.

---

For more details, see comments in each script and the Arduino sketch.

```
python autofocus_tenengrad.py COM5 0 16 sweep.mp4 0 9000
```

Flow:
- Sends `Z<raise>` to hard-home and set pos=0.
- Performs repeated `G<step_chunk>` moves until `total_steps` are reached, recording frames to `sweep.mp4`.
- Computes Tenengrad per frame; finds best-focused frame index.
- Moves back from end-of-sweep to the corresponding step position of the best frame.

Notes:
- Ensure Arduino is powered and connected; `SoftwareSerial` on pins 1/2 is used for external UART.
- Camera index may vary; use `0` or list cameras via OS tools.
- Adjust `total_steps` to your objective/limits.