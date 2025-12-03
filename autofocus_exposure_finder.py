#!/usr/bin/env python3
"""
autofocus_exposure_finder.py

Find a good manual camera exposure (and optional gain) for Tenengrad autofocus.

What it does:
 - Opens the specified camera index with OpenCV.
 - Attempts to disable auto-exposure/gain (best-effort; backend dependent).
 - Iterates a list or numeric range of exposure values (and optional gain list).
 - For each (exposure, gain) setting: sets the camera property, waits a bit,
   captures N frames, computes:
     - Tenengrad focus metric (raw)
     - Normalized Tenengrad = Tenengrad / (mean_intensity + eps)^2 (reduces bias toward brighter images)
   - Prints results and remembers the best setting by the chosen scoring metric (normalized by default).
 - Optionally saves the best image to disk.

Usage examples:
 - Range of negative exposures (common on Windows/V4L2 for many webcams):
     python autofocus_exposure_finder.py --camera 0 --exp-min -13 --exp-max -1 --exp-step 1
 - Specific list of exposures:
     python autofocus_exposure_finder.py --camera 0 --exp-list "-8,-6,-4" --frames 7 --verbose
 - Try different gains too (comma-separated):
     python autofocus_exposure_finder.py --camera 0 --exp-min -12 --exp-max -2 --gain-list "0,4,8" --frames 5
 - Save the best frame:
     python autofocus_exposure_finder.py --camera 0 --exp-min -12 --exp-max -2 --save-best best.jpg

Notes:
 - CAP_PROP_EXPOSURE/CAP_PROP_GAIN semantics vary by platform and driver. Values that work for your camera may be positive, negative or fractional.
 - If settings appear not to change the image, try a different range or use your camera vendor tool to discover supported exposure values.
 - The script chooses the best setting by normalized Tenengrad by default; set --score raw to choose raw Tenengrad.
"""
import argparse
import time
import cv2
import numpy as np
import sys
from typing import List, Tuple

EPS = 1e-9


def tenengrad_metric_gray(gray: np.ndarray) -> float:
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    fm = (gx * gx + gy * gy).sum()
    fm /= (gray.shape[0] * gray.shape[1])
    return float(fm)


def set_manual_exposure_and_gain(cap: cv2.VideoCapture, exposure, gain, verbose=False):
    """
    Best-effort attempt to disable auto-exposure and set manual exposure & gain.
    This is driver/backend dependent and may be ignored by some cameras.
    """
    # Try to set manual exposure mode (values differ across backends)
    try:
        # Many backends: CAP_PROP_AUTO_EXPOSURE: 1.0 = manual (some use 3.0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # another common attempt
    except Exception:
        pass
    # Exposure
    if exposure is not None:
        try:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
            if verbose:
                print(f"  Requested CAP_PROP_EXPOSURE = {exposure}")
        except Exception:
            if verbose:
                print("  Warning: failed to set CAP_PROP_EXPOSURE")
    # Gain
    if gain is not None:
        try:
            cap.set(cv2.CAP_PROP_GAIN, float(gain))
            if verbose:
                print(f"  Requested CAP_PROP_GAIN = {gain}")
        except Exception:
            if verbose:
                print("  Warning: failed to set CAP_PROP_GAIN")


def capture_average_frame(cap: cv2.VideoCapture, frames: int, delay: float = 0.02):
    """Capture a small series of frames and return the last frame and mean intensity."""
    last = None
    intensities = []
    for i in range(frames):
        ret, frame = cap.read()
        if not ret:
            # small delay and retry once
            time.sleep(0.02)
            ret, frame = cap.read()
            if not ret:
                continue
        last = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        intensities.append(float(gray.mean()))
        if delay:
            time.sleep(delay)
    mean_intensity = float(np.mean(intensities)) if intensities else 0.0
    return last, mean_intensity


def parse_list_arg(s: str) -> List[float]:
    if s is None or s.strip() == "":
        return []
    parts = s.split(",")
    vals = []
    for p in parts:
        try:
            vals.append(float(p.strip()))
        except Exception:
            pass
    return vals


def build_exposure_list(min_v, max_v, step, list_arg) -> List[float]:
    if list_arg:
        return list_arg
    if min_v is None or max_v is None or step is None:
        raise ValueError("Either provide --exp-list or all of --exp-min, --exp-max, --exp-step")
    if step == 0:
        raise ValueError("--exp-step must be non-zero")
    vals = []
    v = min_v
    if step > 0:
        while v <= max_v + 1e-12:
            vals.append(float(v))
            v += step
    else:
        while v >= max_v - 1e-12:
            vals.append(float(v))
            v += step
    return vals


def main():
    parser = argparse.ArgumentParser(description="Find a good manual exposure/gain for Tenengrad autofocus.")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--exp-min", type=float, default=-13.0, help="Minimum exposure (inclusive) for range mode")
    parser.add_argument("--exp-max", type=float, default=-1.0, help="Maximum exposure (inclusive) for range mode")
    parser.add_argument("--exp-step", type=float, default=1.0, help="Step for exposure range mode")
    parser.add_argument("--exp-list", type=str, default="", help="Comma-separated list of exposures to try (overrides range)")
    parser.add_argument("--gain-list", type=str, default="", help="Comma-separated list of gain values to try (optional)")
    parser.add_argument("--frames", type=int, default=5, help="Frames to average per test setting")
    parser.add_argument("--wait", type=float, default=0.15, help="Seconds to wait after setting exposure/gain before capturing")
    parser.add_argument("--score", choices=("normalized", "raw"), default="normalized", help="Which metric to optimize")
    parser.add_argument("--save-best", type=str, default="", help="Optional path to save best frame (JPEG/PNG).")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        if args.exp_list:
            exp_list = parse_list_arg(args.exp_list)
        else:
            exp_list = build_exposure_list(args.exp_min, args.exp_max, args.exp_step, None)
    except Exception as e:
        print("Error building exposure list:", e)
        sys.exit(2)
    gain_list = parse_list_arg(args.gain_list) if args.gain_list else [None]

    if args.verbose:
        print("Exposure candidates:", exp_list)
        print("Gain candidates:", gain_list)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("ERROR: Could not open camera index", args.camera)
        sys.exit(2)

    best = None  # tuple: (score, raw_metric, norm_metric, exposure, gain, frame)
    results = []

    for gain in gain_list:
        for exposure in exp_list:
            if args.verbose:
                print(f"Testing exposure={exposure} gain={gain} ...", end="", flush=True)
            # Try to set camera to manual/exposure/gain
            set_manual_exposure_and_gain(cap, exposure, gain, verbose=args.verbose)
            # Wait for settings to take effect
            time.sleep(args.wait)
            # Capture averaged frames
            frame, mean_intensity = capture_average_frame(cap, frames=max(1, args.frames), delay=0.01)
            if frame is None:
                if args.verbose:
                    print(" no frame")
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            raw = tenengrad_metric_gray(gray)
            norm = raw / ((mean_intensity + EPS) ** 2)
            results.append((exposure, gain, raw, norm, mean_intensity))
            score = norm if args.score == "normalized" else raw
            if best is None or score > best[0]:
                best = (score, raw, norm, exposure, gain, frame.copy())
            if args.verbose:
                print(f" raw={raw:.1f} norm={norm:.6f} meanI={mean_intensity:.1f}")

    # Print summary table
    print("\nResults:")
    print(f"{'Exposure':>10} {'Gain':>6} {'RawTenengrad':>14} {'NormTenengrad':>16} {'MeanI':>8}")
    for exposure, gain, raw, norm, meanI in results:
        gstr = str(gain) if gain is not None else "-"
        print(f"{str(exposure):>10} {gstr:>6} {raw:14.1f} {norm:16.6f} {meanI:8.1f}")

    if best is None:
        print("No successful samples were collected.")
        cap.release()
        sys.exit(1)

    score, raw_b, norm_b, exp_b, gain_b, frame_b = best
    chosen_metric = "normalized" if args.score == "normalized" else "raw"
    chosen_value = norm_b if args.score == "normalized" else raw_b
    print("\nBest by", chosen_metric, ":")
    print(f"  exposure = {exp_b}   gain = {gain_b}   raw={raw_b:.1f}   norm={norm_b:.6f}")

    if args.save_best:
        try:
            cv2.imwrite(args.save_best, frame_b)
            print("Saved best frame to", args.save_best)
        except Exception as e:
            print("Failed to save best frame:", e)

    print("\nRecommendation for autofocus script:")
    gain_flag = f"--gain {gain_b}" if gain_b is not None else ""
    print(f"  --exposure {exp_b} {gain_flag}")
    print("Example command:")
    print(f"  python autofocus_serial_tenengrad_full.py --port COM9 --baud 115200 --step-size 50 --rpm 8 --visualize --exposure {exp_b} {gain_flag} --verify-frames 7")

    cap.release()


if __name__ == "__main__":
    main()