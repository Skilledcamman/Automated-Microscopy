#!/usr/bin/env python3
"""
Flask webapp — serial console + camera + objective presets + autofocus subprocess launcher

Behavior improvements:
- When launching the autofocus CLI subprocess we stop the Camera capture and pause/close the serial port
  so the CLI can open them exclusively.
- When the subprocess exits we resume serial and restart the camera.
- After resuming serial we attempt to restore the controller's internal position from the last_homed_position.json
  file (written by the autofocus script) by sending H<pos> and E so the controller's position/homed state is preserved.
"""
import os
import threading
import time
import json
import re
import shlex
import subprocess
from queue import Queue, Empty
from flask import Flask, render_template, request, jsonify, Response, stream_with_context

import serial
import cv2

# ---- Configuration ----
SERIAL_PORT = os.environ.get("SERIAL_PORT", "COM9")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))

CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "0"))
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = float(os.environ.get("CAMERA_FPS", "15"))
CAMERA_JPEG_QUALITY = int(os.environ.get("CAMERA_JPEG_QUALITY", "60"))

AUTOFOCUS_CLI_SCRIPT = os.environ.get("AUTOFOCUS_CLI_SCRIPT", "autofocus_serial_match_reverse_presets.py")

LAST_HOMED_FILE = os.environ.get("LAST_HOMED_FILE", "last_homed_position.json")

DEFAULT_AF_ARGS = {
    "step_size": 50,
    "rpm": 8.0,
    "visualize": False,
    "verify_frames": 5,
    "sleep_after_move": 0.12,
    "fallback_granularity": 8,
    "focus_mode": "coarse",
}

app = Flask(__name__, template_folder="templates")
log_q = Queue()

def log(msg, kind="info"):
    try:
        payload = {"kind": kind, "msg": str(msg)}
        log_q.put(payload)
    except Exception:
        pass

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/stream")
def stream():
    def event_stream():
        while True:
            try:
                payload = log_q.get(timeout=0.5)
            except Empty:
                continue
            yield f"data: {json.dumps(payload)}\n\n"
    return Response(stream_with_context(event_stream()), mimetype="text/event-stream")

# ---------------- Serial management ----------------
ser = None
ser_lock = threading.Lock()
request_lock = threading.Lock()
response_cond = threading.Condition()
response_active = False
response_lines = []

serial_paused = threading.Event()
camera_paused = threading.Event()

def sanitize_text(s: str) -> str:
    if s is None:
        return ""
    return re.sub(r'[^\x09\x0A\x0D\x20-\x7E]', '?', s)

def serial_open():
    global ser
    if serial_paused.is_set():
        return None
    if ser is None:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            time.sleep(0.3)
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            log(f"Opened serial {SERIAL_PORT}@{SERIAL_BAUD}", kind="info")
        except Exception as e:
            ser = None
            log(f"serial_open failed: {e}", kind="error")
    return ser

def serial_close():
    global ser
    try:
        with ser_lock:
            if ser:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                log("Serial port closed for exclusive access", kind="info")
    except Exception:
        pass

def pause_serial():
    serial_paused.set()
    serial_close()

def resume_serial():
    serial_paused.clear()
    log("Serial reader resumed", kind="info")

def restore_homed_position_from_file(timeout=5.0):
    """
    If LAST_HOMED_FILE exists and contains a numeric 'position', send H<pos> and E to the controller
    to restore the internal position counter and persist it to EEPROM.
    This helps when the MCU resets on serial reopen (common on Arduino when DTR toggles).
    """
    path = LAST_HOMED_FILE
    if not os.path.exists(path):
        return False
    try:
        with open(path, "r", encoding="utf-8") as fp:
            data = json.load(fp)
        pos = data.get("position")
        if pos is None:
            return False
        # wait for serial to be available
        deadline = time.time() + timeout
        while time.time() < deadline:
            s = serial_open()
            if s:
                break
            time.sleep(0.1)
        s = serial_open()
        if s is None:
            log("Cannot restore homed position: serial not available", kind="error")
            return False
        with ser_lock:
            try:
                # H<n> sets internal counter without moving; E writes to EEPROM
                cmd_h = f"H{int(pos)}\n"
                s.write(cmd_h.encode("ascii"))
                s.flush()
                time.sleep(0.08)
                cmd_e = "E\n"
                s.write(cmd_e.encode("ascii"))
                s.flush()
                log(f"Restored controller position with {cmd_h.strip()} and persisted with E", kind="info")
                return True
            except Exception as e:
                log(f"Failed to write H/E to serial for restore: {e}", kind="error")
                return False
    except Exception as ex:
        log(f"Failed to read last homed file {path}: {ex}", kind="error")
        return False

def serial_reader():
    global ser, response_active, response_lines
    while True:
        try:
            if serial_paused.is_set():
                time.sleep(0.2)
                continue

            s = serial_open()
            if s is None:
                time.sleep(0.2)
                continue

            with ser_lock:
                raw = s.readline()

            if not raw:
                time.sleep(0.01)
                continue

            try:
                text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                text = repr(raw)

            text = sanitize_text(text)
            log(text, kind="serial")

            with response_cond:
                if response_active:
                    response_lines.append(text)
                    response_cond.notify_all()
        except Exception as exc:
            log(f"Serial reader error: {exc}", kind="error")
            try:
                if ser:
                    with ser_lock:
                        try:
                            ser.close()
                        except Exception:
                            pass
                        ser = None
            except Exception:
                pass
            time.sleep(1.0)

@app.route("/api/cmd", methods=["POST"])
def api_cmd():
    data = request.get_json(force=True)
    if not data or "cmd" not in data:
        return jsonify({"ok": False, "error": "missing cmd"}), 400

    cmd = str(data["cmd"])
    timeout = float(data.get("timeout", 0.6))
    cmd_to_send = cmd if cmd.endswith("\n") else (cmd + "\n")

    if not request_lock.acquire(blocking=False):
        return jsonify({"ok": False, "error": "another command in progress"}), 409

    try:
        s = serial_open()
        if s is None:
            return jsonify({"ok": False, "error": "cannot open serial port"}), 500

        with response_cond:
            global response_active, response_lines
            response_lines = []
            response_active = True

        with ser_lock:
            try:
                s.write(cmd_to_send.encode("ascii"))
                s.flush()
            except Exception as e:
                with response_cond:
                    response_active = False
                    response_cond.notify_all()
                return jsonify({"ok": False, "error": f"write failed: {e}"}), 500

        deadline = time.time() + timeout
        collected = []
        with response_cond:
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                if response_lines:
                    break
                response_cond.wait(timeout=remaining)
            while True:
                if response_lines:
                    collected.extend(response_lines)
                    response_lines = []
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                response_cond.wait(timeout=min(0.06, remaining))
            response_active = False

        return jsonify({"ok": True, "sent": cmd.strip(), "lines": collected})
    finally:
        request_lock.release()

# ---------------- Camera (with pause/resume) ----------------
class Camera:
    def __init__(self, index=0, width=640, height=480, target_fps=15.0, jpeg_quality=60):
        self.index = int(index)
        self.width = int(width)
        self.height = int(height)
        self.target_fps = float(target_fps)
        self.jpeg_quality = int(jpeg_quality)
        self.cap = None
        self.frame = None
        self.frame_id = 0
        self.frame_time = 0.0
        self.lock = threading.Lock()
        self.thread = None
        self.running = False
        self.exposure = None
        self.gain = None

    def start(self):
        with self.lock:
            if self.running:
                return True
            cap = cv2.VideoCapture(self.index)
            if not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                return False
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.width))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height))
                cap.set(cv2.CAP_PROP_FPS, float(self.target_fps))
            except Exception:
                pass
            self.cap = cap
            self.running = True
            self.thread = threading.Thread(target=self._reader, daemon=True)
            self.thread.start()
            log(f"Camera started index={self.index} {self.width}x{self.height}@{self.target_fps}fps q={self.jpeg_quality}", kind="info")
            if self.exposure is not None or self.gain is not None:
                try:
                    self._apply_controls_locked(self.exposure, self.gain)
                except Exception:
                    pass
            return True

    def _reader(self):
        frame_interval = 1.0 / max(0.1, self.target_fps)
        while True:
            with self.lock:
                if not self.running or self.cap is None:
                    break
                cap = self.cap
            t0 = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue
            try:
                ret2, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), max(20, min(95, self.jpeg_quality))])
            except Exception:
                try:
                    ret2, jpg = cv2.imencode('.jpg', frame)
                except Exception:
                    ret2 = False
                    jpg = None
            if ret2 and jpg is not None:
                with self.lock:
                    self.frame = jpg.tobytes()
                    self.frame_id += 1
                    self.frame_time = time.time()
            t_elapsed = time.time() - t0
            to_sleep = frame_interval - t_elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)
        with self.lock:
            try:
                if self.cap:
                    self.cap.release()
            except Exception:
                pass
            self.cap = None
            self.running = False
            self.thread = None
            self.frame = None
            self.frame_id = 0
            self.frame_time = 0.0
        log("Camera capture thread stopped", kind="info")

    def get_jpeg(self):
        with self.lock:
            return self.frame, self.frame_id, self.frame_time

    def stop(self):
        with self.lock:
            self.running = False
        t = None
        with self.lock:
            t = self.thread
        if t:
            t.join(timeout=0.5)
        with self.lock:
            try:
                if self.cap:
                    self.cap.release()
            except Exception:
                pass
            self.cap = None
            self.frame = None
            self.thread = None
            self.running = False
            self.frame_id = 0
        log("Camera stopped by request", kind="info")

    def _apply_controls_locked(self, exposure, gain):
        if not self.cap:
            self.exposure = exposure
            self.gain = gain
            return False
        ok_any = False
        if exposure is not None:
            try:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            except Exception:
                pass
            try:
                ok = self.cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
                ok_any = ok_any or bool(ok)
            except Exception:
                pass
        if gain is not None:
            try:
                ok = self.cap.set(cv2.CAP_PROP_GAIN, float(gain))
                ok_any = ok_any or bool(ok)
            except Exception:
                pass
        self.exposure = exposure
        self.gain = gain
        return ok_any

    def set_controls(self, exposure=None, gain=None):
        with self.lock:
            self.exposure = exposure
            self.gain = gain
            if self.cap:
                return self._apply_controls_locked(exposure, gain)
        started = self.start()
        if not started:
            return False
        with self.lock:
            return self._apply_controls_locked(exposure, gain)

camera = Camera(index=CAMERA_INDEX, width=CAMERA_WIDTH, height=CAMERA_HEIGHT,
                target_fps=CAMERA_FPS, jpeg_quality=CAMERA_JPEG_QUALITY)

@app.route("/video_feed")
def video_feed():
    started = camera.start()
    if not started:
        return "Failed to open camera", 500
    def mjpeg_generator():
        last_id = -1
        try:
            while True:
                frame, fid, ftime = camera.get_jpeg()
                if frame and fid != last_id:
                    last_id = fid
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                else:
                    time.sleep(0.01)
        except GeneratorExit:
            return
    return Response(stream_with_context(mjpeg_generator()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/camera/start", methods=["POST"])
def camera_start():
    ok = camera.start()
    return jsonify({"running": bool(ok)})

@app.route("/camera/stop", methods=["POST"])
def camera_stop():
    camera.stop()
    return jsonify({"running": False})

@app.route("/camera/status")
def camera_status():
    return jsonify({"running": bool(camera.running), "exposure": camera.exposure, "gain": camera.gain,
                    "camera_paused_for_af": bool(camera_paused.is_set())})

# ---------------- Objective presets (import from autofocus module) ----------------
OBJECTIVE_PRESETS = None
try:
    af_mod = __import__("autofocus_serial_match_reverse_presets")
    OBJECTIVE_PRESETS = getattr(af_mod, "DEFAULT_PRESETS", None)
    if isinstance(OBJECTIVE_PRESETS, dict):
        OBJECTIVE_PRESETS = {str(k): v for k, v in OBJECTIVE_PRESETS.items()}
        log("Loaded objective presets from autofocus module", kind="info")
    else:
        OBJECTIVE_PRESETS = None
except Exception:
    OBJECTIVE_PRESETS = None

FALLBACK_PRESETS = {
    "4":  {"exposure": -13, "gain": 0},
    "10": {"exposure": -12, "gain": 15},
    "40": {"exposure": -8, "gain": 8},
}
OBJECTIVE_TO_CMD = {"4": "O4", "10": "O10", "40": "O40"}

def normalize_obj_name(name):
    if name is None:
        return None
    s = str(name).strip().lower()
    if s.endswith('x'):
        s = s[:-1]
    return s.strip()

def get_preset_for_obj(obj_name):
    key = normalize_obj_name(obj_name)
    if key is None:
        return {"exposure": None, "gain": None}
    if OBJECTIVE_PRESETS and key in OBJECTIVE_PRESETS:
        p = OBJECTIVE_PRESETS[key]
        if isinstance(p, dict):
            return {"exposure": p.get("exposure"), "gain": p.get("gain")}
    return FALLBACK_PRESETS.get(key, {"exposure": None, "gain": None})

@app.route("/camera/objective", methods=["POST"])
def camera_objective():
    data = request.get_json(force=True) or {}
    obj_raw = str(data.get("objective", "")).strip()
    obj_key = normalize_obj_name(obj_raw)
    if obj_key not in OBJECTIVE_TO_CMD:
        return jsonify({"ok": False, "error": "unknown objective", "allowed": list(OBJECTIVE_TO_CMD.keys())}), 400
    preset = get_preset_for_obj(obj_key)
    exposure = preset.get("exposure")
    gain = preset.get("gain")
    cam_ok = False
    try:
        cam_ok = camera.set_controls(exposure=exposure, gain=gain)
    except Exception as ex:
        log(f"Failed to set camera controls: {ex}", kind="error")
        cam_ok = False
    cmd = OBJECTIVE_TO_CMD.get(obj_key, f"O{obj_key}")
    serial_sent = None
    try:
        s = serial_open()
        if s:
            with ser_lock:
                s.write((cmd + "\n").encode("ascii"))
                s.flush()
            serial_sent = cmd
    except Exception as e:
        log(f"Failed to send objective serial cmd: {e}", kind="error")
    return jsonify({"ok": True, "objective": f"{obj_key}x", "exposure": exposure, "gain": gain,
                    "camera_applied": bool(cam_ok), "serial_sent": serial_sent})

# ---------------- Subprocess-based autofocus launcher with serial+camera handoff ----------------
af_process = None
af_process_lock = threading.Lock()
af_reader_thread = None

def _start_af_subprocess(cmd_list):
    """
    Stop camera, pause/close serial, then start subprocess and forward its stdout->SSE.
    Resume serial and camera after subprocess exits. Restore homed position (H/E) if available.
    """
    # stop camera to free device
    try:
        camera.stop()
        camera_paused.set()
        log("Camera stopped for exclusive AF", kind="info")
    except Exception as e:
        log(f"Failed to stop camera before AF: {e}", kind="error")

    # pause serial to free COM port
    pause_serial()

    # start subprocess
    try:
        p = subprocess.Popen(cmd_list, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    except Exception as e:
        log(f"Failed to start autofocus subprocess: {e}", kind="error")
        # resume serial/camera on failure
        resume_serial()
        try:
            camera.start()
            camera_paused.clear()
            log("Camera restarted after AF start failure", kind="info")
        except Exception:
            pass
        return None

    def _reader(proc):
        log(f"Autofocus subprocess started pid={proc.pid}", kind="info")
        try:
            for line in proc.stdout:
                if line is None:
                    break
                log(line.rstrip("\n"), kind="autofocus")
        except Exception as e:
            log(f"Autofocus reader error: {e}", kind="error")
        finally:
            rc = proc.poll()
            log(f"Autofocus subprocess exited (pid={proc.pid}) rc={rc}", kind="info")
            # resume serial and camera
            resume_serial()
            # small delay to allow serial to open cleanly, then attempt restore from file
            time.sleep(0.25)
            restored = restore_homed_position_from_file(timeout=3.0)
            if restored:
                log("Restored homed position to controller after AF", kind="info")
            try:
                camera.start()
                camera_paused.clear()
                log("Camera restarted after AF", kind="info")
            except Exception as e:
                log(f"Failed to restart camera after AF: {e}", kind="error")
            with af_process_lock:
                global af_process
                if af_process is proc:
                    af_process = None

    t = threading.Thread(target=_reader, args=(p,), daemon=True)
    t.start()
    return p

@app.route("/autofocus/launch", methods=["POST"])
def autofocus_launch():
    data = request.get_json(force=True) or {}
    mode = str(data.get("mode", DEFAULT_AF_ARGS["focus_mode"])).strip().lower()
    if mode not in ("coarse", "medium", "fine"):
        return jsonify({"ok": False, "error": "invalid mode"}), 400

    args = {}
    args["step_size"] = int(data.get("step_size", DEFAULT_AF_ARGS["step_size"]))
    args["rpm"] = float(data.get("rpm", DEFAULT_AF_ARGS["rpm"]))
    args["visualize"] = bool(data.get("visualize", DEFAULT_AF_ARGS["visualize"]))
    args["verify_frames"] = int(data.get("verify_frames", DEFAULT_AF_ARGS["verify_frames"]))
    args["sleep_after_move"] = float(data.get("sleep_after_move", DEFAULT_AF_ARGS["sleep_after_move"]))
    args["fallback_granularity"] = int(data.get("fallback_granularity", DEFAULT_AF_ARGS["fallback_granularity"]))
    args["focus_mode"] = mode

    port = data.get("port", SERIAL_PORT)
    baud = int(data.get("baud", SERIAL_BAUD))

    with af_process_lock:
        global af_process
        if af_process is not None and af_process.poll() is None:
            return jsonify({"ok": False, "error": "autofocus process already running", "pid": af_process.pid}), 409

        cmd = ["python", AUTOFOCUS_CLI_SCRIPT,
               "--port", str(port),
               "--baud", str(baud),
               "--step-size", str(args["step_size"]),
               "--rpm", str(args["rpm"]),
               "--verify-frames", str(args["verify_frames"]),
               "--sleep-after-move", str(args["sleep_after_move"]),
               "--fallback-granularity", str(args["fallback_granularity"]),
               "--focus-mode", str(args["focus_mode"])
               ]
        if args["visualize"]:
            cmd.append("--visualize")

        p = _start_af_subprocess(cmd)
        if p is None:
            return jsonify({"ok": False, "error": "failed to spawn subprocess"}), 500
        af_process = p
        return jsonify({"ok": True, "pid": p.pid, "cmd": " ".join(shlex.quote(x) for x in cmd)})

@app.route("/autofocus/stop", methods=["POST"])
def autofocus_stop_subproc():
    with af_process_lock:
        global af_process
        if af_process is None or af_process.poll() is not None:
            af_process = None
            return jsonify({"ok": False, "error": "no autofocus subprocess running"}), 400
        try:
            af_process.terminate()
            log(f"Sent terminate to autofocus pid={af_process.pid}", kind="info")
            try:
                af_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                af_process.kill()
                log(f"Killed autofocus pid={af_process.pid}", kind="info")
            af_process = None
            # resume serial/camera in case stop didn't trigger reader's finally
            resume_serial()
            # attempt restore
            time.sleep(0.25)
            restore_homed_position_from_file(timeout=3.0)
            try:
                camera.start()
                camera_paused.clear()
                log("Camera restarted after AF stop", kind="info")
            except Exception:
                pass
            return jsonify({"ok": True, "msg": "stopped"})
        except Exception as e:
            log(f"Error stopping autofocus subprocess: {e}", kind="error")
            resume_serial()
            try:
                camera.start()
                camera_paused.clear()
            except Exception:
                pass
            return jsonify({"ok": False, "error": str(e)}), 500

@app.route("/autofocus/status")
def autofocus_status():
    with af_process_lock:
        global af_process
        running = af_process is not None and af_process.poll() is None
        pid = af_process.pid if running else None
        return jsonify({"running": bool(running), "pid": pid,
                        "serial_paused_for_af": bool(serial_paused.is_set()),
                        "camera_paused_for_af": bool(camera_paused.is_set())})

# ---------------- Start serial reader thread ----------------
if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False)