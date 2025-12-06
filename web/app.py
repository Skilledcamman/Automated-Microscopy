import threading
import time
from typing import Optional

from flask import Flask, Response, render_template, request, jsonify
import cv2

# Reuse the autofocus logic by importing the script
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from autofocus_tenengrad import ZAxisController, tenengrad_focus, set_camera_properties, DEFAULT_PRESETS
try:
    from i2c_controller import I2CController
except Exception:
    I2CController = None
from collections import deque


app = Flask(__name__)

# Shared resources
SERIAL_PORT = os.environ.get("MICRO_SCOPE_SERIAL", "/dev/ttyACM0")
BAUD = int(os.environ.get("MICRO_SCOPE_BAUD", "115200"))
USE_I2C = os.environ.get("MICRO_SCOPE_USE_I2C", "1") == "1"  # default to I2C
I2C_ADDR = int(os.environ.get("MICRO_SCOPE_I2C_ADDR", "18"))  # 0x12 default
CAMERA_INDEX = 0
CURRENT_OBJECTIVE = "40"  # default objective
shared_serial: Optional[ZAxisController] = None
shared_i2c = None  # type: ignore
shared_cap: Optional[cv2.VideoCapture] = None
lock = threading.Lock()
serial_buffer = deque(maxlen=5000)


def _buffer_lines(lines):
    if not lines:
        return
    for ln in lines:
        # Normalize to string and strip trailing CR
        try:
            s = ln.decode('utf-8', errors='ignore') if isinstance(ln, (bytes, bytearray)) else str(ln)
        except Exception:
            s = str(ln)
        serial_buffer.append(s.rstrip('\r'))


def init_serial() -> ZAxisController:
    global shared_serial
    if shared_serial is None:
        shared_serial = ZAxisController(SERIAL_PORT, BAUD)
        shared_serial.open()
    return shared_serial


def init_i2c():
    if not I2CController:
        raise RuntimeError("I2CController module not available; install smbus2 and ensure i2c_controller.py exists")
    global shared_i2c
    if shared_i2c is None:
        shared_i2c = I2CController(I2C_ADDR)
    return shared_i2c


def init_camera(objective: str = "40") -> cv2.VideoCapture:
    global shared_cap
    if shared_cap is None:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            raise RuntimeError("Unable to open camera")
        # Set desired resolution 800x600
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        preset = DEFAULT_PRESETS.get(objective, DEFAULT_PRESETS["40"])
        set_camera_properties(cap, preset["exposure"], preset["gain"])
        shared_cap = cap
    return shared_cap


def apply_camera_preset_with_retry(cap: cv2.VideoCapture, exposure: float, gain: float, retries: int = 5, delay: float = 0.15) -> bool:
    """On Linux some drivers ignore property sets unless retried; attempt multiple times and verify."""
    ok = False
    for _ in range(retries):
        set_camera_properties(cap, exposure, gain)
        time.sleep(delay)
        # verify if backend reports props (may still fail silently on some drivers)
        got_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
        got_gain = cap.get(cv2.CAP_PROP_GAIN)
        if got_exp != 0 or got_gain != 0:
            ok = True
            break
    return ok


@app.route("/")
def index():
    return render_template("index.html")


def mjpeg_generator():
    cap = init_camera()
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")


@app.route("/video")
def video():
    return Response(mjpeg_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/console", methods=["POST"])
def console():
    data = request.get_json(force=True)
    cmd = data.get("cmd", "").strip()
    if not cmd:
        return jsonify({"ok": False, "error": "Empty command"}), 400
    if USE_I2C:
        i2c = init_i2c()
        with lock:
            try:
                out = i2c.send_ascii(cmd)
                if out:
                    _buffer_lines(out.splitlines())
                return jsonify({"ok": True, "output": out})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)}), 500
    else:
        ser = init_serial()
        with lock:
            ser._send(cmd)
            lines = ser._read_until(lambda l: True, timeout=0.4)
            _buffer_lines(lines)
        return jsonify({"ok": True, "output": "\n".join(lines)})


@app.route("/console/drain", methods=["GET"])  # return buffered serial output only
def console_drain():
    out_lines = []
    with lock:
        while serial_buffer:
            out_lines.append(serial_buffer.popleft())
    return jsonify({"ok": True, "output": "\n".join(out_lines)})


def autofocus_run(preset_level: str):
    """Run autofocus with shared serial and camera using preset levels."""
    ser = init_serial() if not USE_I2C else None
    cap = init_camera(CURRENT_OBJECTIVE)

    # Map preset to parameters
    if preset_level == "coarse":
        params = {"objective": CURRENT_OBJECTIVE, "step": 100, "rpm": 12, "fine_step": None, "adjust_threshold": 0.8, "adjust_iterations": 0}
    elif preset_level == "medium":
        params = {"objective": CURRENT_OBJECTIVE, "step": 100, "rpm": 12, "fine_step": 50, "adjust_threshold": 0.8, "adjust_iterations": 10}
    elif preset_level == "fine":
        params = {"objective": CURRENT_OBJECTIVE, "step": 100, "rpm": 12, "fine_step": 10, "adjust_threshold": 0.8, "adjust_iterations": 10}
    else:
        raise ValueError("Unknown preset level")

    # Configure objective and speed
    if USE_I2C:
        i2c = init_i2c()
        with lock:
            i2c.home()
            i2c.set_objective(params["objective"])
            max_limit = i2c.get_max_limit()
            i2c.set_rpm(params["rpm"])
    else:
        with lock:
            ser.home(raise_steps=0)
            ser.set_objective(params["objective"])
            max_limit = ser.get_max_limit()
            ser.set_speed_rpm(params["rpm"])

    # Ensure camera props match objective
    preset = DEFAULT_PRESETS[params["objective"]]
    set_camera_properties(cap, preset["exposure"], preset["gain"]) 

    # Start at 0
    with lock:
        pos = (init_i2c().get_position() if USE_I2C else ser.get_position())
        if pos != 0:
            (init_i2c().move_steps(-pos) if USE_I2C else ser.move_steps(-pos))
            pos = (init_i2c().get_position() if USE_I2C else ser.get_position())

    best_focus = -1.0
    best_pos = 0
    margin = 50
    target_end = max(0, max_limit - margin)

    # Sweep
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        fm = tenengrad_focus(gray)
        if fm > best_focus:
            best_focus = fm
            best_pos = pos
        if pos + params["step"] >= target_end:
            break
        with lock:
            ser.move_steps(params["step"])
        pos += params["step"]

    # Move to best and optionally fine adjust by sweeping
    with lock:
        current_pos = (init_i2c().get_position() if USE_I2C else ser.get_position())
        delta = best_pos - current_pos
        if delta != 0:
            (init_i2c().move_steps(delta) if USE_I2C else ser.move_steps(delta))

    ret, final_frame = cap.read()
    final_fm = -1.0
    if ret:
        final_fm = tenengrad_focus(cv2.cvtColor(final_frame, cv2.COLOR_BGR2GRAY))

    fine_step = params["fine_step"]
    adjust_threshold = params["adjust_threshold"]
    adjust_iterations = params["adjust_iterations"]

    if fine_step and adjust_iterations and best_focus > 0 and final_fm < best_focus * adjust_threshold:
        # Forward sweep
        steps_taken = 0
        for _ in range(adjust_iterations):
            with lock:
                ser.move_steps(fine_step)
            steps_taken += fine_step
            ret, frame = cap.read()
            if not ret:
                break
            fm_curr = tenengrad_focus(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if fm_curr >= best_focus * adjust_threshold:
                break
        else:
            # Return to start and backward sweep
            with lock:
                ser.move_steps(-steps_taken)
            steps_taken = 0
            for _ in range(adjust_iterations):
                with lock:
                    ser.move_steps(-fine_step)
                steps_taken += fine_step
                ret, frame = cap.read()
                if not ret:
                    break
                fm_curr = tenengrad_focus(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                if fm_curr >= best_focus * adjust_threshold:
                    break
            else:
                # Return to start
                with lock:
                    ser.move_steps(steps_taken)

    return {
        "best_focus": best_focus,
        "best_pos": best_pos,
        "final_focus": final_fm,
        "final_pos": (init_i2c().get_position() if USE_I2C else init_serial().get_position()),
    }


@app.route("/objective", methods=["POST"])
def set_objective_endpoint():
    """Set objective (4/10/40): updates camera exposure/gain and sends O<n> over serial."""
    global CURRENT_OBJECTIVE
    data = request.get_json(force=True)
    obj = str(data.get("objective", "")).strip()
    if obj not in ("4", "10", "40"):
        return jsonify({"ok": False, "error": "Invalid objective. Use 4, 10, or 40."}), 400
    # Update shared state
    CURRENT_OBJECTIVE = obj
    ser = init_serial() if not USE_I2C else None
    cap = init_camera(obj)
    # Apply camera props (retry on Linux) and send serial command
    preset = DEFAULT_PRESETS[obj]
    if USE_I2C:
        with lock:
            init_i2c().set_objective(obj)
    else:
        with lock:
            ser.set_objective(obj)
    ok_props = apply_camera_preset_with_retry(cap, preset["exposure"], preset["gain"]) 
    result = {"ok": True, "objective": obj, "cameraPropsSet": ok_props}
    return jsonify(result)


@app.route("/camera/properties", methods=["POST"])
def camera_properties_endpoint():
    """Set camera exposure/gain explicitly from UI controls."""
    data = request.get_json(force=True)
    try:
        exposure = float(data.get("exposure"))
        gain = float(data.get("gain"))
    except Exception:
        return jsonify({"ok": False, "error": "Invalid exposure/gain"}), 400
    cap = init_camera(CURRENT_OBJECTIVE)
    ok_props = apply_camera_preset_with_retry(cap, exposure, gain)
    return jsonify({"ok": ok_props, "exposure": exposure, "gain": gain})


@app.route("/autofocus", methods=["POST"])
def autofocus_endpoint():
    data = request.get_json(force=True)
    level = data.get("level", "coarse")
    try:
        result = autofocus_run(level)
        return jsonify({"ok": True, **result})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/home", methods=["POST"])
def home_endpoint():
    """Home Z axis via shared serial."""
    ser = init_serial() if not USE_I2C else None
    try:
        if USE_I2C:
            with lock:
                init_i2c().home()
        else:
            # send raw command to ensure output is capturable, then read and buffer for a short while
            with lock:
                ser._send("Z")
            # Read for up to 2 seconds collecting any lines
            t0 = time.time()
            lines = []
            while time.time() - t0 < 2.0:
                with lock:
                    chunk = ser._read_until(lambda l: True, timeout=0.2)
                if chunk:
                    lines.extend(chunk)
                else:
                    # small sleep to avoid busy loop
                    time.sleep(0.05)
            with lock:
                _buffer_lines(lines)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/move", methods=["POST"])
def move_endpoint():
    """Move Z axis by signed steps: { steps: 100 }"""
    data = request.get_json(force=True)
    steps = int(data.get("steps", 0))
    ser = init_serial() if not USE_I2C else None
    if steps == 0:
        return jsonify({"ok": False, "error": "steps must be non-zero"}), 400
    try:
        if USE_I2C:
            with lock:
                init_i2c().move_steps(steps)
                pos = init_i2c().get_position()
            return jsonify({"ok": True, "position": pos})
        else:
            with lock:
                ser.move_steps(steps)
            # try to capture any immediate lines for a short window
            with lock:
                chunk = ser._read_until(lambda l: True, timeout=0.2)
            _buffer_lines(chunk)
            with lock:
                pos = ser.get_position()
            return jsonify({"ok": True, "position": pos})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


def create_app():
    # Initialize shared resources on startup
    global USE_I2C
    if USE_I2C and I2CController is None:
        # Auto-fallback to serial if I2C unavailable
        USE_I2C = False
    if USE_I2C:
        init_i2c()
    else:
        init_serial()
    init_camera()
    return app


if __name__ == "__main__":
    create_app()
    app.run(host="0.0.0.0", port=5000, debug=False)
