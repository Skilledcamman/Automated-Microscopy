#!/usr/bin/env python3
"""
Record video during a single continuous sweep, map each frame to an estimated
step position, compute a focus metric per frame and move the motor to the best
position.

Key ideas:
 - Start: query start position from Arduino.
 - Command Arduino to perform a single long move (S<n> then U/D) without waiting.
 - Record frames to a video file while also saving per-frame timestamps.
 - After move finishes, query end position from Arduino.
 - Map frame timestamps linearly between the first/last captured timestamps to
   positions between start_pos and end_pos.
 - Compute focus metric (Laplacian variance or Tenengrad) per frame and choose best.
 - Optionally do a small discrete refinement around the best position and move there.

This script assumes the Arduino sketch supports the simple serial commands used
in previous examples: S<n>, U (or D), P, Q, V<n>, M<n>, O<n>, R.

Usage example:
  python stepper_video_autofocus.py --port COM9 --source 0 --steps 10000 --rpm 8 \
    --expected-fps 60 --video-out sweep.avi --metric laplacian --downscale 0.5 \
    --settle 0.01 --refine-range 500 --refine-step 50 --move-to-best --plot

Notes and tips:
 - Camera may not actually deliver the requested FPS. The script records actual
   timestamps and uses them for mapping.
 - Turn off autofocus/exposure on the camera if possible so frames are consistent.
 - Increase --rpm if the sweep is too slow; decrease if motor stalls.
 - If using STEPS_PER_REV different from 2048, edit STEPS_PER_REV constant.
"""

import argparse
import time
import os
import math
import csv
from collections import namedtuple

import cv2
import numpy as np
import serial
import re

INTEGER_RE = re.compile(r'(-?\d+)')
STEPS_PER_REV = 2048.0

FrameRecord = namedtuple("FrameRecord", ["timestamp", "frame_index", "filename", "score"])

def laplacian_variance_gray(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    return float(np.var(lap))

def tenengrad_score_gray(frame, ksize=3):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
    sy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
    gm = np.sqrt(sx*sx + sy*sy)
    return float(np.mean(gm))

class ArduinoSimple:
    def __init__(self, port, baud=115200, timeout=1.0, debug=False):
        self.debug = debug
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
        except Exception as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}")
        time.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def send(self, cmd, read_timeout=0.5):
        if self.debug:
            print("[SER OUT]", cmd)
        self.ser.write((cmd + "\n").encode("ascii"))
        self.ser.flush()
        end = time.time() + read_timeout
        lines = []
        while time.time() < end:
            raw = self.ser.readline()
            if not raw:
                time.sleep(0.005)
                continue
            try:
                s = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                s = ""
            if s:
                if self.debug:
                    print("[SER IN]", s)
                lines.append(s)
        return lines

    def write_no_wait(self, cmd):
        if self.debug:
            print("[SER OUT no-wait]", cmd)
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            self.ser.flush()
        except Exception as e:
            if self.debug:
                print("write_no_wait failed:", e)

    def query_status(self):
        lines = self.send("Q", read_timeout=0.6)
        pos = None
        max_limit = None
        for ln in lines:
            lnl = ln.lower()
            nums = re.findall(r'-?\d+', ln)
            if 'position' in lnl and nums:
                try:
                    pos = int(nums[0])
                except Exception:
                    pass
            if 'max' in lnl and 'limit' in lnl and nums:
                try:
                    max_limit = int(nums[-1])
                except Exception:
                    pass
        return {"position": pos, "max_limit": max_limit, "raw": lines}

    def get_position(self):
        st = self.query_status()
        if st.get("position") is not None:
            return st.get("position")
        lines = self.send("P", read_timeout=0.4)
        for ln in lines:
            m = INTEGER_RE.search(ln)
            if m:
                try:
                    return int(m.group(1))
                except Exception:
                    pass
        return None

    def set_speed(self, rpm):
        self.send(f"V{int(rpm)}", read_timeout=0.1)

    def set_steps_per_press(self, n):
        self.send(f"S{int(n)}", read_timeout=0.1)

    def start_continuous_move(self, steps, direction_positive=True):
        self.write_no_wait(f"S{int(abs(steps))}")
        cmd = "U" if direction_positive else "D"
        self.write_no_wait(cmd)

    def move_relative_and_wait(self, steps):
        s = int(abs(steps))
        if s == 0:
            return self.get_position()
        self.set_steps_per_press(s)
        if steps > 0:
            self.send("U", read_timeout=1.0)
        else:
            self.send("D", read_timeout=1.0)
        return self.get_position()

    def release(self):
        self.send("R", read_timeout=0.1)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

def open_capture(idx, req_fps=None, width=None, height=None):
    try:
        src = int(idx)
        cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)  # try DirectShow on Windows
    except Exception:
        cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video source: {idx}")

    # try to set fps and resolution where possible
    if req_fps:
        cap.set(cv2.CAP_PROP_FPS, float(req_fps))
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    # disable auto exposure if possible (platform/camera dependent)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # not universally supported
    return cap

def make_writer(path, fourcc_str, fps, frame_size):
    fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
    return cv2.VideoWriter(path, fourcc, float(fps), frame_size, True)

def parse_args():
    p = argparse.ArgumentParser(description="Record video during continuous sweep and map frames to steps")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--source", "-s", default="0", help="Camera index")
    p.add_argument("--steps", type=int, default=10000, help="Number of steps to request for continuous sweep")
    p.add_argument("--direction", choices=["up","down"], default="up")
    p.add_argument("--rpm", type=float, default=8.0, help="Motor RPM")
    p.add_argument("--expected-fps", type=float, default=60.0, help="Requested capture fps (actual fps will be measured)")
    p.add_argument("--video-out", default="sweep.avi", help="Path to save recorded video (use .avi/.mp4)")
    p.add_argument("--fourcc", default="MJPG", help="FourCC for VideoWriter (MJPG, XVID, mp4v, etc.)")
    p.add_argument("--metric", choices=["laplacian","tenengrad"], default="laplacian")
    p.add_argument("--downscale", type=float, default=1.0)
    p.add_argument("--ksize", type=int, default=3, choices=[1,3,5,7])
    p.add_argument("--settle", type=float, default=0.01)
    p.add_argument("--refine-range", type=int, default=500, help="Range in steps around best to do discrete refinement")
    p.add_argument("--refine-step", type=int, default=50, help="Step increment for discrete refinement")
    p.add_argument("--refine-frames", type=int, default=3, help="Frames to average per refinement position")
    p.add_argument("--move-to-best", action="store_true", help="Move motor to final best position at end")
    p.add_argument("--save-csv", default="sweep_frames.csv", help="Save per-frame timestamps and scores to CSV")
    p.add_argument("--debug-serial", action="store_true")
    return p.parse_args()

def compute_metric(frame, metric, ksize):
    if metric == "laplacian":
        return laplacian_variance_gray(frame)
    else:
        return tenengrad_score_gray(frame, ksize=ksize)

def main():
    args = parse_args()

    # open camera
    cap = open_capture(args.source, req_fps=args.expected_fps)
    # read one frame to get size
    ret, frame = cap.read()
    if not ret:
        cap.release()
        raise RuntimeError("Camera returned no frames")
    height, width = frame.shape[:2]
    frame_size = (width, height)
    # try to estimate actual fps by capturing a few frames before starting motor
    timestamps_probe = []
    for _ in range(10):
        ret, _ = cap.read()
        if not ret:
            break
        timestamps_probe.append(time.time())
        time.sleep(max(0, 1.0/args.expected_fps * 0.5))
    if len(timestamps_probe) >= 2:
        measured_fps = max(1.0, (len(timestamps_probe)-1) / (timestamps_probe[-1] - timestamps_probe[0]))
    else:
        measured_fps = args.expected_fps
    print(f"Measured camera FPS (estimate): {measured_fps:.2f}")

    # prepare video writer
    writer = make_writer(args.video_out, args.fourcc, measured_fps, frame_size)
    print(f"Recording to {args.video_out} using fourcc={args.fourcc} fps={measured_fps:.2f} size={frame_size}")

    # open serial
    ar = ArduinoSimple(args.port, baud=args.baud, debug=args.debug_serial)
    ar.set_speed(args.rpm)
    time.sleep(0.05)

    # query start pos and max
    status = ar.query_status()
    start_pos = status.get("position") or 0
    max_limit = status.get("max_limit") or 10000
    print(f"Arduino start_pos={start_pos}, max_limit={max_limit}")

    # compute actual target end (clamp)
    if args.direction == "up":
        target_end = min(int(max_limit), start_pos + args.steps)
        direction_positive = True
    else:
        target_end = max(0, start_pos - args.steps)
        direction_positive = False
    actual_steps = abs(target_end - start_pos)
    if actual_steps == 0:
        print("No movement requested or already at limit; exiting.")
        ar.close(); cap.release(); writer.release(); return

    # set steps-per-press to the whole sweep to avoid changing S during move
    ar.set_steps_per_press(actual_steps)
    time.sleep(0.01)

    # start continuous move (non-blocking)
    print(f"Starting continuous move {actual_steps} steps -> target_end={target_end}")
    ar.start_continuous_move(actual_steps, direction_positive=direction_positive)
    time.sleep(0.005)

    # capture frames until we detect end. We'll capture for estimated duration + margin.
    step_rate = (args.rpm * STEPS_PER_REV) / 60.0
    if step_rate <= 0:
        print("Invalid step rate; check rpm")
        ar.close(); cap.release(); writer.release(); return
    est_duration = actual_steps / step_rate
    capture_deadline = time.time() + est_duration + max(0.2, est_duration*0.08)
    print(f"Estimated sweep duration {est_duration:.2f}s, capturing until {capture_deadline:.3f}")

    frame_records = []
    frame_idx = 0

    # capture loop
    while time.time() < capture_deadline:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.001)
            continue
        ts = time.time()
        # optionally downscale metric computation, but store full frame to video
        proc = frame
        if args.downscale and args.downscale != 1.0:
            h,w = frame.shape[:2]
            proc = cv2.resize(frame, (int(w*args.downscale), int(h*args.downscale)), interpolation=cv2.INTER_AREA)
        score = compute_metric(proc, args.metric, args.ksize)
        # write frame to video
        writer.write(frame)
        frame_records.append({"timestamp": ts, "index": frame_idx, "score": score})
        frame_idx += 1

    # allow Arduino to finish then query end position
    time.sleep(0.05 + max(0.05, est_duration*0.03))
    end_pos = ar.get_position()
    if end_pos is None:
        end_pos = target_end
        print("Couldn't read end position; estimating end_pos =", end_pos)
    print(f"End position reported: {end_pos}")

    writer.release()

    if not frame_records:
        print("No frames recorded.")
        ar.close(); cap.release(); return

    # save CSV of frames
    with open(args.save_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_index", "timestamp", "score"])
        for r in frame_records:
            w.writerow([r["index"], f"{r['timestamp']:.6f}", f"{r['score']:.6f}"])
    print(f"Wrote per-frame CSV: {args.save_csv} ({len(frame_records)} frames)")

    # map frames to estimated step positions (linear mapping using first/last timestamps)
    t0 = frame_records[0]["timestamp"]
    t1 = frame_records[-1]["timestamp"]
    total_time = max(1e-6, t1 - t0)
    mapped = []
    for r in frame_records:
        frac = (r["timestamp"] - t0) / total_time
        frac = min(1.0, max(0.0, frac))
        est_pos = int(round(start_pos + frac * (end_pos - start_pos)))
        mapped.append((r["index"], r["timestamp"], r["score"], est_pos))

    # find best frame by score
    best = max(mapped, key=lambda x: x[2])
    best_frame_idx, best_ts, best_score, best_est_pos = best
    print(f"Best frame idx={best_frame_idx} score={best_score:.6f} est_pos={best_est_pos} ts={best_ts:.3f}")

    # optionally do a discrete refine sweep around best_est_pos
    if args.refine_range and args.refine_step and args.refine_step > 0:
        half = args.refine_range // 2
        refine_start = max(0, best_est_pos - half)
        refine_end = min(int(max_limit), best_est_pos + half)
        targets = list(range(refine_start, refine_end+1, args.refine_step))
        if targets[-1] != refine_end:
            targets.append(refine_end)
        print(f"Refinement sweep targets: {len(targets)} positions {targets[0]}..{targets[-1]} step {args.refine_step}")

        # move to first target
        cur = ar.get_position() or start_pos
        delta = targets[0] - cur
        if delta != 0:
            ar.move_relative_and_wait(delta)
            time.sleep(args.settle)
        best_ref_score = best_score
        best_ref_pos = best_est_pos
        best_ref_frame = None

        for t in targets:
            cur = ar.get_position() or cur
            delta = t - cur
            if delta != 0:
                ar.move_relative_and_wait(delta)
                time.sleep(args.settle)
            # average frames-per-position
            scores = []
            frame_sample = None
            for _ in range(args.refine_frames):
                ret, f = cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue
                proc = f
                if args.downscale and args.downscale != 1.0:
                    h,w = f.shape[:2]
                    proc = cv2.resize(f, (int(w*args.downscale), int(h*args.downscale)), interpolation=cv2.INTER_AREA)
                sc = compute_metric(proc, args.metric, args.ksize)
                scores.append(sc)
                frame_sample = f.copy()
            if not scores:
                print(f"[Refine] pos={t}: no frames")
                continue
            avg_sc = float(sum(scores)/len(scores))
            print(f"[Refine] pos={t} avg_score={avg_sc:.3f}")
            if avg_sc > best_ref_score:
                best_ref_score = avg_sc
                best_ref_pos = t
                best_ref_frame = frame_sample.copy() if frame_sample is not None else None

        print(f"Refinement best pos={best_ref_pos} score={best_ref_score:.6f}")

        if args.move_to_best:
            curpos = ar.get_position()
            if curpos is None:
                print("Cannot read current position; skipping move-to-best.")
            else:
                delta = best_ref_pos - curpos
                if delta != 0:
                    print(f"Moving {delta} steps to refined best {best_ref_pos}...")
                    ar.move_relative_and_wait(delta)
                    time.sleep(0.05)
                    print("Moved. New pos:", ar.get_position())

        # save refined best frame if available
        if best_ref_frame is not None:
            out_ref = os.path.splitext(args.video_out)[0] + "_refined.jpg"
            cv2.imwrite(out_ref, best_ref_frame)
            print("Saved refined best image to", out_ref)

    else:
        # optionally move to estimated best directly
        if args.move_to_best:
            curpos = ar.get_position()
            if curpos is None:
                print("Cannot read current position; skipping move-to-best.")
            else:
                delta = best_est_pos - curpos
                if delta != 0:
                    print(f"Moving {delta} steps to estimated best {best_est_pos}...")
                    ar.move_relative_and_wait(delta)
                    time.sleep(0.05)
                    print("Moved. New pos:", ar.get_position())

    # save best continuous-frame image (extract from video using index; simpler: reopen video)
    try:
        cap2 = cv2.VideoCapture(args.video_out)
        if cap2.isOpened():
            cap2.set(cv2.CAP_PROP_POS_FRAMES, best_frame_idx)
            ret, bf = cap2.read()
            if ret:
                out_best = os.path.splitext(args.video_out)[0] + "_best.jpg"
                cv2.imwrite(out_best, bf)
                print("Saved best continuous frame to", out_best)
            cap2.release()
    except Exception as e:
        print("Failed to extract best frame from video:", e)

    ar.close()
    cap.release()

if __name__ == "__main__":
    main()