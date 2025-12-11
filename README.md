# Microscope Autofocus + Arduino Stepper

This workspace contains:

- `arduino_stepper_controller.ino`: Arduino sketch for ULN2003 + 28BYJ-48 stepper, serial-controlled. Requires homing (`Z`) on each boot; supports movement commands after homing.
- `autofocus_tenengrad.py`: Python script using OpenCV that homes first, performs a continuous sweep to the end while recording a video, computes Tenengrad focus per frame, then moves to the best-focus frame's corresponding step position.

## Arduino Upload

- Board: Arduino Uno (or compatible)
- Libraries: built-in `Stepper`, `EEPROM`, and `SoftwareSerial`
- Wiring: ULN2003 to pins IN1=8, IN3=9, IN2=10, IN4=11 (matching sketch order). Serial RX=1, TX=2 for `SoftwareSerial` external UART.
- Upload `arduino_stepper_controller.ino` using Arduino IDE.

## Python Dependencies

Install dependencies in a Python 3.10+ environment:

```
python -m pip install -r requirements.txt
```

`requirements.txt` includes `opencv-python` and `pyserial`.

## Run Autofocus Sweep

Example usage:

```
python autofocus_tenengrad.py <serial_port> <camera_index> <step_chunk> <video_path> [raise_steps] [total_steps]
```

- `serial_port`: e.g. `COM5`
- `camera_index`: e.g. `0`
- `step_chunk`: number of steps per movement between captured frames (e.g. `16`)
- `video_path`: where to save the sweep video (e.g. `sweep.mp4`)
- `raise_steps` (optional): steps to raise after hard home (default `0`)
- `total_steps` (optional): total travel steps to sweep (default `9000`)

Example:

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