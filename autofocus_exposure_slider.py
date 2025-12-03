#!/usr/bin/env python3
"""
autofocus_exposure_slider.py

Live camera window with exposure & gain sliders and immediate Tenengrad feedback.

Features
- OpenCV window with trackbars to control exposure and gain (best-effort; driver dependent).
- Live Tenengrad focus metric (and normalized metric) displayed on the frame.
- Adjustable averaging (number of frames used to compute the metric).
- Press 's' to save the current frame to disk, 'q' or ESC to quit.

Usage
  python autofocus_exposure_slider.py --camera 0 --exp-min -13 --exp-max -1 --gain-min 0 --gain-max 100

Notes
- CAP_PROP_EXPOSURE/CAP_PROP_GAIN semantics vary by camera/driver. For many webcams on Windows, exposure values are negative.
- The script tries to request manual exposure mode using CAP_PROP_AUTO_EXPOSURE but this may be ignored by the backend.
- If you have a camera control application, it's often best to lock exposure there first.
"""

import argparse
import time
import math
import cv2
import numpy as np
import sys
from datetime import datetime

EPS = 1e-9


def tenengrad_metric_gray(gray: np.ndarray) -> float:
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    fm = (gx * gx + gy * gy).sum()
    fm /= (gray.shape[0] * gray.shape[1])
    return float(fm)


def try_set_manual_exposure(cap, exposure_value, gain_value, verbose=False):
    """
    Best-effort: attempt to switch off auto exposure and set exposure/gain values.
    semantics vary across backends; multiple attempts are used.
    """
    # Try a few common auto-exposure toggles (driver dependent)
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)  # some backends interpret 1.0 as manual
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # another attempt for some backends
    except Exception:
        pass

    if exposure_value is not None:
        try:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value))
            if verbose:
                print("Requested exposure:", exposure_value)
            time.sleep(0.03)
        except Exception:
            if verbose:
                print("Warning: CAP_PROP_EXPOSURE not supported on this backend.")

    if gain_value is not None:
        try:
            cap.set(cv2.CAP_PROP_GAIN, float(gain_value))
            if verbose:
                print("Requested gain:", gain_value)
            time.sleep(0.03)
        except Exception:
            if verbose:
                print("Warning: CAP_PROP_GAIN not supported on this backend.")


def capture_avg_and_metrics(cap, frames=1, delay=0.0):
    """
    Capture up to `frames` frames (dropping failed reads), return last frame,
    mean intensity across the captured frames, and average Tenengrad metric.
    """
    metrics = []
    mean_intensities = []
    last_frame = None
    for _ in range(frames):
        ret, frame = cap.read()
        if not ret:
            # small wait then continue
            if delay > 0:
                time.sleep(delay)
            continue
        last_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        metrics.append(tenengrad_metric_gray(gray))
        mean_intensities.append(float(gray.mean()))
        if delay > 0:
            time.sleep(delay)
    if not metrics:
        return None, 0.0, 0.0
    return last_frame, float(np.mean(metrics)), float(np.mean(mean_intensities))


def run_slider_gui(args):
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("ERROR: Could not open camera index", args.camera)
        sys.exit(1)

    # Create window and trackbars
    winname = "Exposure Control"
    cv2.namedWindow(winname, cv2.WINDOW_NORMAL)

    # exposure trackbar maps track position [0..exp_range] -> value [exp_min..exp_max]
    exp_min = args.exp_min
    exp_max = args.exp_max
    if exp_min >= exp_max:
        raise ValueError("exp-min must be < exp-max")
    exp_range = int(exp_max - exp_min)
    # to allow integer trackbar steps; if range too big clamp resolution
    cv2.createTrackbar("Exposure", winname, int(args.initial_exposure - exp_min), exp_range, lambda v: None)

    # gain trackbar
    gain_min = args.gain_min
    gain_max = args.gain_max
    if gain_min >= gain_max:
        raise ValueError("gain-min must be < gain-max")
    gain_range = int(gain_max - gain_min)
    cv2.createTrackbar("Gain", winname, int(args.initial_gain - gain_min), gain_range, lambda v: None)

    # frames to average
    cv2.createTrackbar("AvgFrames", winname, args.avg_frames, args.max_avg_frames, lambda v: None)

    # initial apply
    try_set_manual_exposure(cap, args.initial_exposure, args.initial_gain, verbose=args.verbose)

    last_exp = args.initial_exposure
    last_gain = args.initial_gain
    last_avg = args.avg_frames

    # FPS measurement
    last_time = time.time()
    fps = 0.0

    print("Exposure slider started.")
    print(" - Move the 'Exposure' and 'Gain' sliders to change camera settings.")
    print(" - 'AvgFrames' sets the number of frames averaged when computing metrics.")
    print(" - Press 's' to save current frame, 'q' or ESC to quit.")
    print("Note: not all cameras accept programmatic exposure/gain changes. If sliders don't affect the image, try your camera vendor tools.")

    save_idx = 0
    while True:
        t0 = time.time()
        # read trackbars
        texp = cv2.getTrackbarPos("Exposure", winname)
        tgain = cv2.getTrackbarPos("Gain", winname)
        avg_frames = cv2.getTrackbarPos("AvgFrames", winname)
        if avg_frames < 1:
            avg_frames = 1

        exposure_value = exp_min + texp
        gain_value = gain_min + tgain

        # If changed, apply
        if exposure_value != last_exp or gain_value != last_gain:
            try_set_manual_exposure(cap, exposure_value, gain_value, verbose=args.verbose)
            last_exp = exposure_value
            last_gain = gain_value

        if avg_frames != last_avg:
            last_avg = avg_frames

        # capture averaged metrics and frame
        frame, metric, meanI = capture_avg_and_metrics(cap, frames=last_avg, delay=args.frame_delay)
        if frame is None:
            # try single read
            ret, frame = cap.read()
            if not ret:
                # blank image fallback
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, "Camera read failed", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow(winname, img)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                continue
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                metric = tenengrad_metric_gray(gray)
                meanI = float(gray.mean())

        # normalized metric
        norm_metric = metric / ((meanI + EPS) ** 2)

        # overlay text
        disp = frame.copy()
        h, w = disp.shape[:2]
        overlay_lines = [
            f"Exposure: {exposure_value}",
            f"Gain: {gain_value}",
            f"AvgFrames: {last_avg}",
            f"Tenengrad: {metric:.1f}",
            f"NormTenengrad: {norm_metric:.6f}",
            f"MeanI: {meanI:.1f}",
            f"FPS: {fps:.1f}",
        ]
        y = 20
        for line in overlay_lines:
            cv2.putText(disp, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y += 25

        cv2.imshow(winname, disp)

        # compute FPS
        dt = time.time() - last_time
        last_time = time.time()
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else (1.0 / dt)

        # key handling
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        elif key == ord("s"):
            # save current frame
            fname = f"exposure_saved_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{save_idx}.png"
            cv2.imwrite(fname, frame)
            print("Saved", fname)
            save_idx += 1

    cap.release()
    cv2.destroyAllWindows()


def parse_args():
    p = argparse.ArgumentParser(description="Exposure/gain slider with Tenengrad live feedback.")
    p.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    p.add_argument("--exp-min", type=int, default=-13, help="Minimum exposure slider value")
    p.add_argument("--exp-max", type=int, default=-1, help="Maximum exposure slider value")
    p.add_argument("--gain-min", type=int, default=0, help="Minimum gain slider value")
    p.add_argument("--gain-max", type=int, default=100, help="Maximum gain slider value")
    p.add_argument("--initial-exposure", type=int, default=-12, help="Initial exposure (should be within exp-min..exp-max)")
    p.add_argument("--initial-gain", type=int, default=0, help="Initial gain (should be within gain-min..gain-max)")
    p.add_argument("--avg-frames", type=int, default=5, help="Initial number of frames to average for metric")
    p.add_argument("--max-avg-frames", type=int, default=20, help="Maximum AvgFrames slider value")
    p.add_argument("--frame-delay", type=float, default=0.0, help="Delay (sec) between frames when averaging (small >0 sometimes helps)")
    p.add_argument("--verbose", action="store_true", help="Print debug/apply messages")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    # clamp initial values to ranges
    args.initial_exposure = max(min(args.initial_exposure, args.exp_max), args.exp_min)
    args.initial_gain = max(min(args.initial_gain, args.gain_max), args.gain_min)
    run_slider_gui(args)