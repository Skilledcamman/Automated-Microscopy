# Microscope Autofocus Web Panel

A Flask-based control panel for a microscope Z-axis (Arduino on COM9) with live camera stream and autofocus presets.

## Prerequisites

- Python 3.10+ (Windows or Linux)
- Packages:
	- Flask
	- opencv-python
	- numpy
	- pyserial
- Hardware:
	- Arduino-based Z-axis controller (e.g., ULN2003 + 28BYJ-48) on serial
	- USB camera supported by OpenCV

Install dependencies:

```bash
pip install Flask opencv-python numpy pyserial
```

## Project Layout

- `web/app.py`: Flask server
- `web/templates/index.html`: Frontend UI
- `web/static/`: CSS and icons
- `autofocus_tenengrad.py`: Autofocus + serial control utilities (imported by `app.py`)

## Configuration

- Serial port:
	- Windows: `COM9` by default (change in `web/app.py`, `SERIAL_PORT`)
	- Linux: typically `/dev/ttyUSB0` or `/dev/ttyACM0`
- Camera index: set `CAMERA_INDEX` in `web/app.py` (default `0`).
- Objectives presets (exposure / gain): see `DEFAULT_PRESETS` in `autofocus_tenengrad.py`.

## Running (Windows)

```powershell
cd C:\Users\ahmed\Documents\microscope\web
python app.py
```

Open the app in a browser: `http://127.0.0.1:5000` (or your host IP).

## Running (Linux)

1) Identify serial port:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Update `SERIAL_PORT` in `web/app.py` (e.g., `/dev/ttyUSB0`).

2) Serial permissions:

- Add your user to the `dialout` group (Debian/Ubuntu-based):

```bash
sudo usermod -a -G dialout $USER
# Log out and back in, or reboot
```

- Or set a temporary permission on the device:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

- Optional udev rule for persistent permissions (replace vendor/product IDs):

```bash
cat <<'RULE' | sudo tee /etc/udev/rules.d/99-arduino-serial.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666"
RULE
sudo udevadm control --reload-rules; sudo udevadm trigger
```

3) Camera device:

- Confirm the camera is accessible and note the index:

```bash
v4l2-ctl --list-devices 2>/dev/null || true
# Usually /dev/video0 maps to index 0
```

- If OpenCV fails to open with `cv2.CAP_DSHOW` (Windows-specific), Linux falls back automatically. Ensure `CAMERA_INDEX=0` (or adjust).

4) Install packages:

```bash
python3 -m pip install --user Flask opencv-python numpy pyserial
```

5) Run the server:

```bash
cd ~/Documents/microscope/web
python3 app.py
```

Open a browser to `http://127.0.0.1:5000`.

## Troubleshooting

- Camera not opening:
	- Verify another app isn’t using the camera.
	- Try different `CAMERA_INDEX` (0, 1, ...).
	- On Linux, ensure the user can access `/dev/video*`.
- Serial 400 errors on `/console`:
	- The frontend only sends non-empty commands; use the console input and press Enter.
	- Background serial output appears via `/console/drain` in the UI.
- No homing output in console:
	- Server buffers homing and move outputs; ensure `/console/drain` is reachable.
- Permissions:
	- Add user to `dialout` (Linux), or set device permissions as shown.

## Optional: Systemd Service (Linux)

Create a service to run on boot:

```bash
sudo tee /etc/systemd/system/microscope.service >/dev/null <<'UNIT'
[Unit]
Description=Microscope Web Panel
After=network.target

[Service]
WorkingDirectory=/home/<user>/Documents/microscope/web
ExecStart=/usr/bin/python3 app.py
Restart=always
User=<user>
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now microscope.service
```

Replace `<user>` with your username. Check logs with:

```bash
journalctl -u microscope.service -f
```

## Notes

- Development server is not for production. Use a proper WSGI server (gunicorn/uwsgi) behind a reverse proxy for deployment.
- If your Arduino uses a different help/console protocol, adjust the UI console logic in `index.html` accordingly.
# Microscope Z Autofocus (Tenengrad)

This adds a host-side Python autofocus routine for your Arduino-controlled Z axis.

- Arduino: `stepper_motor_z_axis/stepper_motor_z_axis.ino` (already supports homing `Z`, moves `G<n>`, position `P`, objective `O<n>`)
- Host script: `autofocus_tenengrad.py` (OpenCV Tenengrad sweep)

## Prerequisites
- Windows, Arduino on `COM9` (adjust with `--port`)
- Python 3.9+
- Camera at index `0` (adjust with `--camera`)

Install Python deps:

```powershell
cd "c:\Users\ahmed\Documents\microscope";
py -m venv .venv;
. .venv\Scripts\Activate.ps1;
python -m pip install --upgrade pip;
pip install -r requirements.txt;
```

## Run autofocus sweep

```powershell
cd "c:\Users\ahmed\Documents\microscope";
python autofocus_tenengrad.py --port COM9 --camera 0 --objective 10 --step 20 --rpm 12 --video --csv
```

- `--objective`: `4`, `10`, or `40` – sets exposure/gain only.
- `--step`: steps per sample (smaller = finer focus, slower)
- `--rpm`: motor speed during sweep
- `--video`: save `scan_YYYYMMDD_HHMMSS.avi`
- `--csv`: save `scan_YYYYMMDD_HHMMSS.csv` with `position_steps,tenengrad`
- `--roi x y w h`: optional center crop for focus metric

Workflow:
1. Homes Z with `Z` (pos 0 = physical stop)
2. Applies objective preset (exposure/gain only)
3. Queries max limit from Arduino with `Q`
4. Sweeps 0 → limit-50 in increments, computes Tenengrad, tracks best
5. Returns to best focus position

## Notes
- If exposure/gain fail to set (camera driver), the script warns and continues.
- You can change objective on the Arduino side anytime (O4/O10/O40). The script also sends it.
- To refine near best focus, rerun with smaller `--step` around the reported best position (future enhancement).
