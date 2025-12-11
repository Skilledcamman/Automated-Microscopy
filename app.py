from flask import Flask, render_template_string, request, jsonify, Response
from ultralytics import YOLO
import cv2
import numpy as np
import time
import serial
import random

app = Flask(__name__)
model = YOLO(r"best.pt")

SERIAL_PORT = '/dev/serial0'
CAM_INDEX = 0
OBJECTIVES = {4: '4x', 10: '10x', 40: '40x'}
STEP_SIZES = {'coarse': 500, 'medium': 250, 'fine': 100}

def tenengrad_focus_measure(gray: np.ndarray) -> float:
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    g2 = gx * gx + gy * gy
    return float(np.mean(g2))

def arduino_send(cmd, ser):
    ser.write((cmd.strip() + '\n').encode('ascii'))
    time.sleep(1)
    lines = []
    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors='ignore').strip()
        except Exception:
            break
        if line:
            lines.append(line)
    return '\n'.join(lines)

def autofocus_sweep(objective, step_size):
    sweep_speed = 12
    video_path = "sweep.mp4"
    ser = serial.Serial(SERIAL_PORT, 9600, timeout=2)
    time.sleep(4)
    arduino_send(f'O{objective}', ser)
    arduino_send(f'V{sweep_speed}', ser)
    arduino_send('Z', ser)
    # Wait for homing complete
    homed = False
    for _ in range(60):
        time.sleep(0.5)
        resp = arduino_send('Q', ser)
        if 'Homing complete.' in resp or 'homed' in resp.lower():
            homed = True
            break
    # Get max limit from Arduino
    resp = arduino_send('Q', ser)
    max_steps = 9000
    for line in resp.splitlines():
        if 'Max limit:' in line:
            try:
                max_steps = int(line.split('Max limit:')[1].strip())
            except Exception:
                pass
    cap = cv2.VideoCapture(CAM_INDEX)
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        if not cap.isOpened():
            ser.close()
            return {'error': 'Could not open camera'}
        width = 1280
        height = 960
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(video_path, fourcc, 25, (width, height))
        frame_positions = []
        pos = 0
        while pos <= max_steps:
            ret, frame = cap.read()
            if not ret:
                break
            writer.write(frame)
            frame_positions.append(pos)
            if pos + step_size <= max_steps:
                arduino_send(f'G{step_size}', ser)
                time.sleep(1.5)
            pos += step_size
        writer.release()
    finally:
        cap.release()
    # Analyze video for best focus
    cap = cv2.VideoCapture(video_path)
    best_idx = -1
    best_score = -1.0
    idx = 0
    scores = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        score = tenengrad_focus_measure(gray)
        scores.append(score)
        if score > best_score:
            best_score = score
            best_idx = idx
        idx += 1
    cap.release()
    target_pos = frame_positions[best_idx] if best_idx < len(frame_positions) else 0
    delta = target_pos - frame_positions[-1]
    arduino_send(f'G{delta}', ser)
    ser.close()
    return {
        'frames': len(frame_positions),
        'best_idx': best_idx,
        'best_score': best_score,
        'best_step': frame_positions[best_idx] if best_idx < len(frame_positions) else '?',
        'objective': objective,
        'step_size': step_size
    }

console_log = []
live_detection_enabled = False
last_class_counts = {}

@app.route('/', methods=['GET', 'POST'])
def index():
    global live_detection_enabled
    result = None
    if request.method == 'POST':
        if 'toggle_detection' in request.form:
            live_detection_enabled = not live_detection_enabled
        else:
            # Pause live detection during autofocus
            live_detection_enabled = False
            mode = request.form.get('mode', 'medium')
            objective = int(request.form.get('objective', 40))
            step_size = STEP_SIZES.get(mode, 250)
            result = autofocus_sweep(objective, step_size)
            # Resume live detection after autofocus
            live_detection_enabled = True
            # Add result to console log
            console_log.append({'cmd': f'AUTOFOCUS ({OBJECTIVES.get(objective, objective)} {mode})', 'output': f'Result\n{result}'})
            if len(console_log) > 100:
                console_log.pop(0)
    glitter_result = None
    detection_summary = ""
    if request.args.get('pollen') == '1':
        # Run glitter detection on a single frame using YOLOv11
        cap = cv2.VideoCapture(CAM_INDEX)
        ret, frame = cap.read()
        cap.release()
        if ret:
            results = model(frame)
            if results and results[0].boxes:
                boxes = results[0].boxes
                confs = boxes.conf.tolist() if hasattr(boxes, 'conf') else []
                labels = boxes.cls.tolist() if hasattr(boxes, 'cls') else []
                xyxys = boxes.xyxy.tolist() if hasattr(boxes, 'xyxy') else []
                names = model.names if hasattr(model, 'names') else {0: 'Glitter'}
                class_counts = {}
                if confs and labels and xyxys:
                    for i, (xyxy, label_idx, conf) in enumerate(zip(xyxys, labels, confs)):
                        x1, y1, x2, y2 = map(int, xyxy)
                        label = names[int(label_idx)]
                        class_counts[label] = class_counts.get(label, 0) + 1
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                        cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                    # Save annotated frame to static file for display
                    cv2.imwrite(r'c:\Users\ahmed\Desktop\microscope\static\glitter_detect.jpg', frame)
                    glitter_result = f"<img src='/static/glitter_detect.jpg?{int(time.time())}' style='max-width:100%;border-radius:8px;border:2px solid #6ec6ff;'>"
                    detection_summary = "<ul style='margin-top:10px;'>" + "".join([f"<li><b>{cls}</b>: {count}</li>" for cls, count in class_counts.items()]) + "</ul>"
                else:
                    glitter_result = "No glitter detected."
                    detection_summary = ""
            else:
                glitter_result = "No glitter detected."
                detection_summary = ""
        else:
            glitter_result = "Camera error."
            detection_summary = ""
    return render_template_string('''
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Microscope Dashboard</title>
        <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
        <style>
            body { background: #181d23; color: #eaeaea; font-family: 'Segoe UI', Arial, sans-serif; }
            .fluidd-card { background: #23272e; border-radius: 10px; box-shadow: 0 2px 8px #0002; margin-bottom: 18px; padding: 18px; }
            .fluidd-title { font-size: 1.2rem; font-weight: 600; margin-bottom: 10px; color: #6ec6ff; }
            .fluidd-section { margin-bottom: 24px; }
            .fluidd-btn { background: #23272e; color: #6ec6ff; border: 1px solid #6ec6ff; border-radius: 6px; padding: 6px 16px; margin-right: 6px; }
            .fluidd-btn:hover { background: #6ec6ff; color: #23272e; }
            .fluidd-select { background: #23272e; color: #eaeaea; border: 1px solid #6ec6ff; border-radius: 6px; padding: 4px 8px; }
            .fluidd-console { background: #181d23; color: #eaeaea; border: 1px solid #6ec6ff; border-radius: 8px; height: 220px; overflow-y: auto; font-size: 0.95em; padding: 10px; }
            .fluidd-label { color: #b0b8c1; font-size: 0.95em; margin-right: 8px; }
            .fluidd-topbar { background: #23272e; padding: 10px 24px; color: #6ec6ff; font-size: 1.3rem; font-weight: 700; letter-spacing: 1px; margin-bottom: 18px; }
            .fluidd-sidebar { background: #23272e; position: fixed; left: 0; top: 0; bottom: 0; width: 56px; display: flex; flex-direction: column; align-items: center; padding-top: 18px; z-index: 10; }
            .fluidd-sidebar .icon { width: 32px; height: 32px; margin-bottom: 18px; filter: invert(60%) sepia(80%) saturate(500%) hue-rotate(180deg); }
            .fluidd-main { margin-left: 70px; }
            .fluidd-macro-btn { background: #23272e; color: #eaeaea; border: 1px solid #6ec6ff; border-radius: 6px; padding: 4px 10px; margin: 2px; font-size: 0.95em; }
            .fluidd-macro-btn:hover { background: #6ec6ff; color: #23272e; }
        </style>
    </head>
    <body>
        <div class="fluidd-sidebar">
            <img src="https://fluidd.xyz/img/fluidd-icon.svg" class="icon" alt="Fluidd">
        </div>
        <div class="fluidd-main">
            <div class="fluidd-topbar">microscope</div>
            <div class="container-fluid">
                <div class="row">
                    <div class="col-md-7">
                        <div class="fluidd-card fluidd-section">
                            <div class="fluidd-title">Camera</div>
                            <div style="display:flex; justify-content:center; align-items:center; width:100%;">
                                <img src="/video_feed" width="800" height="600" style="border-radius:8px;border:2px solid #23272e;">
                            </div>
                        </div>
                        <div class="fluidd-card fluidd-section">
                            <div class="fluidd-title">Glitter Detection (AI)</div>
                            <form method="post" style="margin-top:10px;">
                                <button type="submit" name="toggle_detection" class="fluidd-btn">{{ 'Turn Detection Off' if live_detection_enabled else 'Turn Detection On' }}</button>
                            </form>
                            <div style="margin-top:10px; color:#6ec6ff; font-size:1.1em;">Live Detection is <b>{{ 'ON' if live_detection_enabled else 'OFF' }}</b></div>
                            {% if glitter_result %}
                            <div style="margin-top:10px; color:#6ec6ff; font-size:1.1em;">{{ glitter_result|safe }}</div>
                            {% endif %}
                            <div style="margin-top:10px; color:#6ec6ff; font-size:1.05em;">
                                Detected Objects:
                                <ul id="detected-objects-list" style='margin-top:6px;'>
                                    <li>None</li>
                                </ul>
                            </div>
                            <script>
                            function updateDetectedObjects() {
                                fetch('/detection_summary').then(r => r.json()).then(data => {
                                    let list = document.getElementById('detected-objects-list');
                                    let html = '';
                                    let counts = data.counts || {};
                                    let hasAny = false;
                                    for (const [cls, count] of Object.entries(counts)) {
                                        html += `<li><b>${cls}</b>: ${count}</li>`;
                                        hasAny = true;
                                    }
                                    if (!hasAny) html = '<li>None</li>';
                                    list.innerHTML = html;
                                });
                            }
                            setInterval(updateDetectedObjects, 1000);
                            updateDetectedObjects();
                            </script>
                        </div>
                            <div class="fluidd-title">Autofocus</div>
                            <form method="post" class="row g-2 align-items-center">
                                <div class="col-auto">
                                    <span class="fluidd-label">Objective:</span>
                                    <select name="objective" class="fluidd-select">
                                        <option value="4">4x</option>
                                        <option value="10">10x</option>
                                        <option value="40" selected>40x</option>
                                    </select>
                                </div>
                                <div class="col-auto">
                                    <span class="fluidd-label">Mode:</span>
                                    <select name="mode" class="fluidd-select">
                                        <option value="coarse">Coarse (500)</option>
                                        <option value="medium" selected>Medium (250)</option>
                                        <option value="fine">Fine (100)</option>
                                    </select>
                                </div>
                                <div class="col-auto">
                                    <button type="submit" class="fluidd-btn">Run Autofocus</button>
                                </div>
                            </form>
                            <!-- Autofocus result removed from UI as requested -->
                        </div>
                    </div>
                    <div class="col-md-5">
                        <div class="fluidd-card fluidd-section">
                            <div class="fluidd-title">Z Axis Control</div>
                            <div class="mb-2 d-flex flex-column align-items-center" style="gap:8px;">
                                <button type="button" class="fluidd-btn" style="width:48px;height:48px;" onclick="moveUp()" title="Up">
                                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M12 5l-7 7h4v7h6v-7h4l-7-7z" fill="#6ec6ff"/></svg>
                                </button>
                                <div class="d-flex align-items-center" style="gap:8px;">
                                    <button type="button" class="fluidd-btn" style="width:48px;height:48px;" onclick="quickCmd('Z')" title="Home">
                                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M12 3l9 9-1.5 1.5L18 12.5V20a1 1 0 0 1-1 1h-3v-5H10v5H7a1 1 0 0 1-1-1v-7.5l-1.5 1.5L3 12l9-9z" fill="#6ec6ff"/></svg>
                                    </button>
                                </div>
                                <button type="button" class="fluidd-btn" style="width:48px;height:48px;" onclick="moveDown()" title="Down">
                                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M12 19l7-7h-4V5h-6v7H5l7 7z" fill="#6ec6ff"/></svg>
                                </button>
                                <div class="d-flex justify-content-center mt-2" id="step-size-group" style="gap:0;">
                                    <button type="button" class="step-btn" data-step="50">50</button>
                                    <button type="button" class="step-btn" data-step="100">100</button>
                                    <button type="button" class="step-btn active" data-step="250">250</button>
                                    <button type="button" class="step-btn" data-step="500">500</button>
                                    <button type="button" class="step-btn" data-step="1000">1000</button>
                                </div>
                                <button type="button" class="fluidd-btn mt-2" style="width:120px;" onclick="quickCmd('R')">Motor Off</button>
                            </div>
                                <style>
                                .step-btn {
                                    background: #23272e;
                                    color: #eaeaea;
                                    border: 1px solid #444a55;
                                    border-right: none;
                                    border-radius: 0;
                                    padding: 6px 18px;
                                    font-size: 1em;
                                    outline: none;
                                    transition: background 0.2s, color 0.2s;
                                }
                                .step-btn:last-child { border-right: 1px solid #444a55; }
                                .step-btn:first-child { border-top-left-radius: 6px; border-bottom-left-radius: 6px; }
                                .step-btn:last-child { border-top-right-radius: 6px; border-bottom-right-radius: 6px; }
                                .step-btn.active, .step-btn:focus {
                                    background: #6ec6ff;
                                    color: #23272e;
                                    border-color: #6ec6ff;
                                    z-index: 1;
                                }
                                </style>
                        </div>
                        <div class="fluidd-card fluidd-section">
                            <div class="fluidd-title">Console</div>
                            <div class="fluidd-console" id="console-log" style="border-bottom-left-radius:0;border-bottom-right-radius:0;"></div>
                            <form id="console-form" onsubmit="event.preventDefault(); sendCmd();" class="mb-2" autocomplete="off">
                                <input type="text" id="console-cmd" style="width:100%;background:#181d23;color:#eaeaea;border:1px solid #6ec6ff;border-top:none;border-bottom-left-radius:8px;border-bottom-right-radius:8px;padding:8px 12px;font-size:1em;outline:none;" placeholder="Enter Arduino command">
                            </form>
                        </div>
                        <!-- Macros section removed -->
                    </div>
                </div>
            </div>
        </div>
        <script>
        function sendCmd() {
            var cmd = document.getElementById('console-cmd').value;
            if (!cmd.trim()) return;
            fetch('/arduino_console', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: cmd})
            }).then(r => r.json()).then(data => {
                updateConsoleLog();
                document.getElementById('console-cmd').value = '';
            });
        }
        document.getElementById('console-cmd').addEventListener('keydown', function(e) {
            if (e.key === 'Enter') {
                e.preventDefault();
                sendCmd();
            }
        });
        function quickCmd(cmd) {
            fetch('/arduino_console', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: cmd})
            }).then(r => r.json()).then(data => {
                updateConsoleLog();
            });
        }
        let currentStep = 250;
        document.querySelectorAll('.step-btn').forEach(btn => {
            btn.addEventListener('click', function() {
                document.querySelectorAll('.step-btn').forEach(b => b.classList.remove('active'));
                this.classList.add('active');
                currentStep = this.getAttribute('data-step');
            });
        });
        function moveUp() {
            quickCmd('S' + currentStep);
            setTimeout(function() { quickCmd('U'); }, 300);
        }
        function moveDown() {
            quickCmd('S' + currentStep);
            setTimeout(function() { quickCmd('D'); }, 300);
        }
        function updateConsoleLog() {
            fetch('/console_log').then(r => r.json()).then(data => {
                document.getElementById('console-log').innerHTML = data.log.map(
                    l => `<div><b>&gt; ${l.cmd}</b><br><pre style='margin:0;padding:0;'>${l.output}</pre></div>`
                ).join('');
                document.getElementById('console-log').scrollTop = document.getElementById('console-log').scrollHeight;
            });
        }
        setInterval(updateConsoleLog, 1500);
        updateConsoleLog();
        </script>
    </body>
    </html>
    ''', result=result, glitter_result=glitter_result, last_class_counts=last_class_counts)


@app.route('/arduino_console', methods=['POST'])
def arduino_console():
    data = request.get_json()
    cmd = data.get('cmd', '')
    try:
        ser = serial.Serial(SERIAL_PORT, 9600, timeout=2)
        time.sleep(1)
        output = arduino_send(cmd, ser)
        ser.close()
    except Exception as e:
        output = f'Error: {e}'
    # Save to log
    console_log.append({'cmd': cmd, 'output': output})
    # Limit log size
    if len(console_log) > 100:
        console_log.pop(0)
    return jsonify({'output': output})

@app.route('/console_log')
def get_console_log():
    return jsonify({'log': console_log})

# --- Live Camera MJPEG Stream ---
def gen_frames():
    global live_detection_enabled, last_class_counts
    cap = cv2.VideoCapture(CAM_INDEX)
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        while True:
            success, frame = cap.read()
            if not success:
                break
            detection_summary = ""
            if live_detection_enabled:
                results = model(frame)
                if results and results[0].boxes:
                    boxes = results[0].boxes
                    confs = boxes.conf.tolist() if hasattr(boxes, 'conf') else []
                    labels = boxes.cls.tolist() if hasattr(boxes, 'cls') else []
                    xyxys = boxes.xyxy.tolist() if hasattr(boxes, 'xyxy') else []
                    names = model.names if hasattr(model, 'names') else {0: 'Glitter'}
                    class_counts = {}
                    if confs and labels and xyxys:
                        for i, (xyxy, label_idx, conf) in enumerate(zip(xyxys, labels, confs)):
                            x1, y1, x2, y2 = map(int, xyxy)
                            label = names[int(label_idx)]
                            class_counts[label] = class_counts.get(label, 0) + 1
                            color = get_class_color(label)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                        detection_summary = ", ".join([f"{cls}: {count}" for cls, count in class_counts.items()])
                        last_class_counts = class_counts.copy()
            # Only draw boxes and labels, do not overlay object counts on frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    finally:
        cap.release()

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def get_class_color(label):
    # Assign a unique color for each class label
    random.seed(hash(label) & 0xFFFFFFFF)
    return tuple(random.randint(64, 255) for _ in range(3))

if __name__ == '__main__':
    @app.route('/detection_summary')
    def detection_summary():
        return jsonify({'counts': last_class_counts})
    app.run(host='0.0.0.0', port=5000, debug=True)