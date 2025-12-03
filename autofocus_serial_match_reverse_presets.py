#!/usr/bin/env python3
"""
autofocus_serial_match_reverse_presets.py

Forward sweep + reverse retrace with objective-based exposure/gain presets,
plus three autofocus presets (coarse, medium, fine) and a standalone refocus mode.

Behavior change (robust homing):
 - The main autofocus flow will NOT start until the controller reports it is homed.
 - The script parses the controller Q output for a "Homed:" line (yes/no).
 - If the controller is not homed, the script will send the hard-home 'Z' command and
   poll Q until the controller reports Homed: yes (or until timeout).
 - The refocus-only flow (--refocus-only) is not affected and will run without forcing a home.

Notes:
 - The script no longer sends H0/E automatically at startup (that would falsely mark
   the position 0 without a physical homing). If you still want to set the controller
   position counter manually, use the controller's H<n> command from Serial Monitor
   (but do this only after you understand the consequences).
"""
import argparse
import json
import os
import re
import time
from collections import namedtuple

import cv2
import numpy as np
import serial

Result = namedtuple("Result", ["position", "metric", "frame", "timestamp"])
int_re = re.compile(r"(-?\d+)")
STEPS_PER_REV = 2048.0
DEFAULT_BAUD = 115200
HOME_PUSH_ESTIMATE = 10000  # used to estimate waiting time after sending Z

# ------------------- Presets -------------------
DEFAULT_PRESETS = {
    "4": {"exposure": -13, "gain": 0},
    "10": {"exposure": -12, "gain": 15},
    "40": {"exposure": -8, "gain": 8},
}


def load_presets(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {str(k): v for k, v in data.items()}


def save_presets(path, presets):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(presets, f, indent=2)


# -------------------- Focus metric --------------------
def tenengrad_metric(gray):
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    fm = (gx * gx + gy * gy).sum()
    fm /= (gray.shape[0] * gray.shape[1])
    return float(fm)


# ---------------- Serial helpers ----------------
def open_serial(port, baud, verbose=False):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.8)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    if verbose:
        boot = read_lines_for(ser, timeout=0.6)
        if boot:
            print("Initial serial boot lines:")
            for l in boot:
                print("  >", l)
    return ser


def send_command(ser, cmd):
    if not cmd.endswith("\n"):
        cmd = cmd + "\n"
    ser.write(cmd.encode("ascii"))
    ser.flush()


def read_lines_for(ser, timeout=0.4):
    deadline = time.time() + timeout
    lines = []
    while time.time() < deadline:
        try:
            line = ser.readline()
        except Exception:
            break
        if not line:
            time.sleep(0.005)
            continue
        try:
            s = line.decode("utf-8", errors="replace").strip()
        except Exception:
            s = str(line)
        if s:
            lines.append(s)
    return lines


def parse_position_from_lines(lines):
    for l in lines:
        if "position" in l.lower():
            m = int_re.search(l)
            if m:
                return int(m.group(1))
    for l in lines:
        m = int_re.search(l)
        if m:
            return int(m.group(1))
    return None


def parse_max_limit_from_lines(lines):
    post_max_re = re.compile(r'(?:max|limit)[^0-9\-]*(-?\d+)', re.IGNORECASE)
    for l in lines:
        m = post_max_re.search(l)
        if m:
            try:
                return int(m.group(1))
            except Exception:
                pass
    candidates = []
    for l in lines:
        for m in int_re.finditer(l):
            try:
                candidates.append(int(m.group(1)))
            except Exception:
                pass
    if candidates:
        return max(candidates)
    return None


def parse_objective_from_lines(lines):
    obj_re = re.compile(r'Objective[:\s]+(\d+)', re.IGNORECASE)
    for l in lines:
        m = obj_re.search(l)
        if m:
            return m.group(1)
    for l in lines:
        if 'objective' in l.lower():
            nums = re.findall(r'\d+', l)
            if nums:
                return nums[0]
    return None


def parse_homed_from_lines(lines):
    """
    Parse lines for homed status. Expects output like:
      'Homed: yes' or 'Homed: no'
    Returns True/False if found, else None.
    """
    homed_re = re.compile(r'Homed[:\s]+(yes|no|true|false|1|0)', re.IGNORECASE)
    for l in lines:
        m = homed_re.search(l)
        if m:
            g = m.group(1).lower()
            if g in ("yes", "true", "1"):
                return True
            if g in ("no", "false", "0"):
                return False
    # also try a looser match for lines that might say 'Position: UNHOMED' or similar
    for l in lines:
        if "unhomed" in l.lower() or "not homed" in l.lower():
            return False
    return None


def get_status_Q(ser, verbose=False):
    """
    Returns tuple: (pos, max_limit, objective, homed_flag, lines)
    homed_flag is True/False if parseable, else None.
    """
    send_command(ser, "Q")
    lines = read_lines_for(ser, timeout=0.6)
    if verbose and lines:
        print("[Q] reply:")
        for l in lines:
            print("  >", l)
    pos = parse_position_from_lines(lines)
    max_limit = parse_max_limit_from_lines(lines)
    objective = parse_objective_from_lines(lines)
    homed = parse_homed_from_lines(lines)
    return pos, max_limit, objective, homed, lines


def get_position(ser, verbose=False, max_attempts=3):
    for attempt in range(1, max_attempts + 1):
        send_command(ser, "P")
        lines = read_lines_for(ser, timeout=0.25 + 0.1 * attempt)
        if verbose and lines:
            print(f"[get_position attempt {attempt}] lines:")
            for l in lines:
                print("  >", l)
        pos = parse_position_from_lines(lines)
        if pos is not None:
            return pos
        # fallback to Q which may include more info
        send_command(ser, "Q")
        qlines = read_lines_for(ser, timeout=0.35 + 0.1 * attempt)
        if verbose and qlines:
            print(f"[get_position fallback Q attempt {attempt}] lines:")
            for l in qlines:
                print("  >", l)
        pos = parse_position_from_lines(qlines)
        if pos is not None:
            return pos
        time.sleep(0.08)
    leftover = read_lines_for(ser, timeout=0.12)
    if verbose and leftover:
        print("[get_position final leftover]:")
        for l in leftover:
            print("  >", l)
    return parse_position_from_lines(leftover)


def compute_wait_seconds(steps, rpm):
    if rpm <= 0:
        rpm = 5.0
    t = abs(steps) * (60.0 / (rpm * STEPS_PER_REV))
    return t + 0.12


def probe_G_support(ser, verbose=False):
    ser.reset_input_buffer()
    send_command(ser, "G0")
    lines = read_lines_for(ser, timeout=0.4)
    if verbose and lines:
        print("[G0 probe] reply:")
        for l in lines:
            print("  >", l)
    for l in lines:
        if "moved g" in l.lower() or "g<n>" in l.lower() or "g0" in l.lower() or "usage: g" in l.lower():
            return True
    for l in lines:
        if "unknown command" in l.lower():
            return False
    return False


def move_exact_G(ser, delta, rpm, verbose=False):
    if delta == 0:
        return get_position(ser, verbose=verbose)
    send_command(ser, f"G{int(delta)}")
    wait = compute_wait_seconds(delta, rpm)
    if verbose:
        print(f"Sent G{int(delta)}, waiting {wait:.3f}s")
    time.sleep(wait)
    return get_position(ser, verbose=verbose)


def move_by_granular(ser, delta, granularity, rpm, verbose=False):
    if delta == 0:
        return get_position(ser, verbose=verbose)
    cmd = "U" if delta > 0 else "D"
    n = abs(delta) // granularity
    for _ in range(int(n)):
        send_command(ser, cmd)
        time.sleep(compute_wait_seconds(granularity, rpm))
    return get_position(ser, verbose=verbose)


# ---------------- camera helpers ----------------
def try_set_exposure_and_gain(cap, exposure_value=None, gain_value=None, verbose=False):
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        time.sleep(0.03)
    except Exception:
        pass
    if exposure_value is not None:
        try:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value))
            if verbose:
                print(f"Requested CAP_PROP_EXPOSURE={exposure_value}")
            time.sleep(0.03)
        except Exception:
            if verbose:
                print("CAP_PROP_EXPOSURE not supported")
    if gain_value is not None:
        try:
            cap.set(cv2.CAP_PROP_GAIN, float(gain_value))
            if verbose:
                print(f"Requested CAP_PROP_GAIN={gain_value}")
            time.sleep(0.03)
        except Exception:
            if verbose:
                print("CAP_PROP_GAIN not supported")


def avg_metric_from_frames(cap, frames=5, delay=0.01):
    metrics = []
    last_frame = None
    for _ in range(frames):
        ret, frame = cap.read()
        if not ret:
            time.sleep(delay)
            continue
        last_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        metrics.append(tenengrad_metric(gray))
        time.sleep(delay)
    return (float(np.mean(metrics)) if metrics else 0.0), last_frame


# ---------------- forward sweep & local refine ----------------
def forward_sweep(ser, cap, step_size, rpm, use_G, fallback_granularity, verify_frames, visualize, verbose, sleep_after_move):
    send_command(ser, f"S{fallback_granularity}")
    time.sleep(0.06)
    send_command(ser, f"V{int(rpm)}")
    time.sleep(0.06)

    # forward_sweep expects the controller to be homed and position readable
    start_pos, max_limit, _, _, _ = get_status_Q(ser, verbose=verbose)
    if start_pos is None:
        raise RuntimeError("Unable to read start position from controller.")
    if max_limit is None:
        max_limit = 10**9
        if verbose:
            print("Could not parse max limit; using very large value.")

    if verbose:
        print(f"Forward sweep starting at {start_pos}, max_limit={max_limit}, step_size={step_size}")

    results = []
    cur = start_pos
    metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames))
    results.append(Result(position=cur, metric=metric, frame=(frame.copy() if frame is not None else None), timestamp=time.time()))
    print(f"Sampled pos={cur} metric={metric:.3f}")

    while cur < max_limit:
        if use_G:
            newpos = move_exact_G(ser, step_size, rpm, verbose=verbose)
        else:
            newpos = move_by_granular(ser, step_size, fallback_granularity, rpm, verbose=verbose)
        time.sleep(sleep_after_move)
        if newpos is None:
            newpos, max_limit, _, _, _ = get_status_Q(ser, verbose=verbose)
        if newpos is None:
            print("Can't read new position; stopping sweep.")
            break
        if newpos == cur:
            if verbose:
                print("Position unchanged after move (limit?). Stopping.")
            break
        cur = newpos
        metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames))
        results.append(Result(position=cur, metric=metric, frame=(frame.copy() if frame is not None else None), timestamp=time.time()))
        print(f"Sampled pos={cur} metric={metric:.3f}")

        if visualize and frame is not None:
            disp = cv2.resize(frame, (640, 480))
            cv2.putText(disp, f"pos={cur} m={metric:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("forward sweep", disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("User quit.")
                break

    return results


def local_refine_sweep(ser, cap, center_pos, window, step, use_G, fallback_granularity, rpm, verify_frames, sleep_after_move, visualize=False, verbose=False, max_limit=10**9):
    """
    Do a sweep from center_pos-window .. center_pos+window using increments 'step'.
    Returns the best Result found (position, metric, frame) and the full list of samples.
    """
    left = max(0, center_pos - abs(window))
    right = min(max_limit, center_pos + abs(window))
    if step <= 0:
        step = 1
    positions = list(range(left, right + 1, step))
    samples = []
    cur = get_position(ser, verbose=verbose)
    # Move to first pos
    if cur is None:
        raise RuntimeError("Cannot read current position for local_refine.")
    delta = positions[0] - cur
    if delta != 0:
        if use_G:
            move_exact_G(ser, delta, rpm, verbose=verbose)
        else:
            move_by_granular(ser, delta, fallback_granularity, rpm, verbose=verbose)
        time.sleep(sleep_after_move)
        cur = get_position(ser, verbose=verbose)

    best = None
    for p in positions:
        # ensure we're at p (controller may be precise)
        cur = get_position(ser, verbose=verbose)
        if cur != p:
            delta = p - (cur if cur is not None else 0)
            if delta != 0:
                if use_G:
                    move_exact_G(ser, delta, rpm, verbose=verbose)
                else:
                    move_by_granular(ser, delta, fallback_granularity, rpm, verbose=verbose)
                time.sleep(sleep_after_move)
        metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames))
        samples.append(Result(position=p, metric=metric, frame=(frame.copy() if frame is not None else None), timestamp=time.time()))
        if verbose:
            print(f"Local sample pos={p} metric={metric:.3f}")
        if best is None or metric > best.metric:
            best = samples[-1]

        if visualize and frame is not None:
            disp = cv2.resize(frame, (640, 480))
            cv2.putText(disp, f"local pos={p} m={metric:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 200, 0), 2)
            cv2.imshow("local refine", disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    # Move to best
    if best is not None:
        cur = get_position(ser, verbose=verbose)
        delta = best.position - (cur if cur is not None else 0)
        if delta != 0:
            if use_G:
                move_exact_G(ser, delta, rpm, verbose=verbose)
            else:
                move_by_granular(ser, delta, fallback_granularity, rpm, verbose=verbose)
            time.sleep(sleep_after_move)
    return best, samples


def reverse_retrace_until_match(ser, cap, recorded_results, best_idx, use_G, fallback_granularity,
                                verify_frames, visualize, verbose, sleep_after_move, match_tolerance=0.08):
    """
    Retrace recorded_results positions in reverse order.

    New behavior:
      - If a sampled Tenengrad metric is strictly greater than the forward-best
        metric, accept that position and stop.
      - Otherwise accept if relative difference is within match_tolerance.
    """
    forward_best = recorded_results[best_idx]
    forward_best_metric = forward_best.metric
    print(f"Forward best pos={forward_best.position} metric={forward_best_metric:.3f}")

    # ensure current pos known
    cur = get_position(ser, verbose=verbose)
    if cur is None:
        raise RuntimeError("Cannot get current position before reverse retrace.")

    positions = [r.position for r in recorded_results]

    for target in reversed(positions):
        cur = get_position(ser, verbose=verbose)
        if cur == target:
            metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames))
            print(f"At pos={cur} measured metric={metric:.3f}")
            # Accept if strictly higher than forward-best
            if metric > forward_best_metric:
                print(f"Found higher metric {metric:.3f} > forward-best {forward_best_metric:.3f}. Accepting pos={cur}")
                return Result(position=cur, metric=metric, frame=frame, timestamp=time.time())
            # Otherwise check relative tolerance
            rel = float("inf") if forward_best_metric <= 0 else abs(metric - forward_best_metric) / forward_best_metric
            if rel <= match_tolerance:
                print(f"Match by tolerance at pos={cur} (rel diff {rel:.3f} <= tol {match_tolerance})")
                return Result(position=cur, metric=metric, frame=frame, timestamp=time.time())
            continue

        # move to target
        delta = target - cur
        if use_G:
            moved = move_exact_G(ser, delta, rpm=8, verbose=verbose)
        else:
            moved = move_by_granular(ser, delta, fallback_granularity, rpm=8, verbose=verbose)
        time.sleep(sleep_after_move)
        cur = moved if moved is not None else get_position(ser, verbose=verbose)

        metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames))
        print(f"Moved to pos={cur} measured metric={metric:.3f}")

        if metric > forward_best_metric:
            print(f"Found higher metric {metric:.3f} > forward-best {forward_best_metric:.3f}. Accepting pos={cur}")
            return Result(position=cur, metric=metric, frame=frame, timestamp=time.time())

        rel = float("inf") if forward_best_metric <= 0 else abs(metric - forward_best_metric) / forward_best_metric
        if rel <= match_tolerance:
            print(f"Match by tolerance at pos={cur} (rel diff {rel:.3f} <= tol {match_tolerance})")
            return Result(position=cur, metric=metric, frame=frame, timestamp=time.time())

        if visualize and frame is not None:
            disp = cv2.resize(frame, (640, 480))
            cv2.putText(disp, f"pos={cur} m={metric:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("reverse retrace", disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("User quit retrace.")
                break

    print("Retrace finished without matching forward-best within tolerance or finding a higher metric.")
    return None


# -------------------- CLI --------------------
def main():
    parser = argparse.ArgumentParser(description="Forward sweep then reverse retrace until matching the forward best metric.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--step-size", type=int, default=50, help="Coarse sweep step size")
    parser.add_argument("--rpm", type=float, default=8.0)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--verify-frames", type=int, default=5, help="Frames averaged per sample/verification")
    parser.add_argument("--sleep-after-move", type=float, default=0.12)
    parser.add_argument("--fallback-granularity", type=int, default=8)
    parser.add_argument("--match-tolerance", type=float, default=0.08, help="Relative tolerance for Tenengrad matching")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--no-set-home", action="store_true", help="(deprecated) do not send H0 at startup - script no longer auto-sends H0")
    parser.add_argument("--exposure", type=float, default=None)
    parser.add_argument("--gain", type=float, default=None)
    parser.add_argument("--presets-file", type=str, default="", help="Load exposure/gain presets from JSON file")
    parser.add_argument("--save-presets", type=str, default="", help="Save the built-in presets to a JSON file and exit")

    # focus refinement presets
    parser.add_argument("--focus-mode", choices=("coarse", "medium", "fine"), default="coarse",
                        help="coarse = forward sweep only; medium = local refine after match; fine = smaller-step local refine")
    parser.add_argument("--medium-window", type=int, default=200, help="medium refine window +/- (steps)")
    parser.add_argument("--medium-step", type=int, default=100, help="medium refine local step")
    parser.add_argument("--fine-window", type=int, default=80, help="fine refine window +/- (steps)")
    parser.add_argument("--fine-step", type=int, default=20, help="fine refine local step")

    # refocus-only options
    parser.add_argument("--refocus-only", action="store_true", help="Run local refocus around current position and exit")
    parser.add_argument("--refocus-window", type=int, default=200, help="Refocus window +/- (steps)")
    parser.add_argument("--refocus-step", type=int, default=20, help="Refocus step size (steps)")
    parser.add_argument("--refocus-verify-frames", type=int, default=None, help="Frames to average during refocus (defaults to --verify-frames)")

    args = parser.parse_args()

    # Save presets requested?
    if args.save_presets:
        save_presets(args.save_presets, DEFAULT_PRESETS)
        print(f"Saved default presets to {args.save_presets}")
        return

    # Load presets if requested, else use defaults
    presets = DEFAULT_PRESETS.copy()
    if args.presets_file:
        if os.path.exists(args.presets_file):
            try:
                presets = load_presets(args.presets_file)
                print(f"Loaded presets from {args.presets_file}")
            except Exception as e:
                print("Failed to load presets file, falling back to defaults:", e)
        else:
            print("Presets file not found, using defaults.")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera index " + str(args.camera))

    ser = open_serial(args.port, args.baud, verbose=args.verbose)
    try:
        g_supported = probe_G_support(ser, verbose=args.verbose)
        print("G supported:", g_supported)

        # Query Q to find current objective (from EEPROM / controller) and homed state
        pos, max_limit, objective, homed, qlines = get_status_Q(ser, verbose=args.verbose)
        if args.verbose:
            print("Q lines:", qlines)
        if objective:
            print("Detected objective:", objective)
            preset = presets.get(str(objective))
            if preset:
                exp_v = preset.get("exposure")
                gain_v = preset.get("gain")
                print(f"Applying preset for objective {objective}: exposure={exp_v} gain={gain_v}")
                try_set_exposure_and_gain(cap, exposure_value=exp_v, gain_value=gain_v, verbose=args.verbose)
            else:
                print(f"No preset for objective {objective}; leaving camera exposure/gain as-is (or use --exposure/--gain to override).")
        else:
            print("Could not parse objective from controller Q response; not applying presets.")

        # If explicit exposure/gain flags supplied, they override presets
        if args.exposure is not None or args.gain is not None:
            print(f"Overriding exposure/gain from CLI: exposure={args.exposure} gain={args.gain}")
            try_set_exposure_and_gain(cap, exposure_value=args.exposure, gain_value=args.gain, verbose=args.verbose)

        # If refocus-only requested, run local refine around current position and exit
        if args.refocus_only:
            cur = get_position(ser, verbose=args.verbose)
            if cur is None:
                raise RuntimeError("Could not read current position for refocus.")
            vf = args.refocus_verify_frames if args.refocus_verify_frames is not None else args.verify_frames
            print(f"Running refocus around current pos={cur} +/-{args.refocus_window} step={args.refocus_step}")
            best, samples = local_refine_sweep(ser, cap, cur, window=args.refocus_window, step=args.refocus_step,
                                               use_G=g_supported, fallback_granularity=args.fallback_granularity,
                                               rpm=args.rpm, verify_frames=max(1, vf), sleep_after_move=args.sleep_after_move,
                                               visualize=args.visualize, verbose=args.verbose, max_limit=(max_limit if max_limit is not None else 10**9))
            if best:
                print(f"Refocus best pos={best.position} metric={best.metric:.3f}")
                try:
                    if best.frame is not None:
                        cv2.imshow("Refocus best frame", best.frame)
                        print("Press any key in image window to exit.")
                        cv2.waitKey(0)
                except Exception:
                    pass
            else:
                print("Refocus found no samples.")
            return

        # --- Ensure controller is homed before starting main autofocus ---
        if homed is True:
            if args.verbose:
                print("Controller reports homed. Proceeding.")
        else:
            # homed is False or None (unknown) => perform Z and wait for homed status
            print("Controller not reported homed. Sending hard-home (Z) and waiting for homed state...")
            send_command(ser, "Z")
            # wait a reasonable amount (estimate based on a large push)
            wait = compute_wait_seconds(HOME_PUSH_ESTIMATE, args.rpm) + 0.5
            if args.verbose:
                print(f"Waiting {wait:.1f}s for homing to complete (initial wait)...")
            time.sleep(wait)

            # Poll Q until homed==True or timeout
            deadline = time.time() + max(12.0, wait + 6.0)
            pos, max_limit, objective, homed, qlines = get_status_Q(ser, verbose=args.verbose)
            while (homed is not True) and time.time() < deadline:
                if args.verbose:
                    print(f"Homed state still not true (homed={homed}). Re-checking in 0.4s...")
                time.sleep(0.4)
                pos, max_limit, objective, homed, qlines = get_status_Q(ser, verbose=args.verbose)

            if homed is not True:
                raise RuntimeError("Controller did not report homed after issuing Z; aborting autofocus.")
            print("Controller reports homed. Proceeding with autofocus.")
            # refresh curpos variable from parsed pos
            # pos may be None if Q didn't include position; try P directly
            if pos is None:
                pos = get_position(ser, verbose=args.verbose)
            # use pos as start_pos for sweep

        # forward sweep (coarse)
        recorded = forward_sweep(ser, cap, step_size=args.step_size, rpm=args.rpm,
                                 use_G=g_supported, fallback_granularity=args.fallback_granularity,
                                 verify_frames=max(1, args.verify_frames), visualize=args.visualize,
                                 verbose=args.verbose, sleep_after_move=args.sleep_after_move)
        if not recorded:
            print("No forward samples collected.")
            return

        best_idx = int(np.argmax([r.metric for r in recorded]))
        best = recorded[best_idx]
        print(f"Forward-best: idx={best_idx} pos={best.position} metric={best.metric:.3f}")

        # Reverse retrace to find matching (or higher) position
        match = reverse_retrace_until_match(ser, cap, recorded_results=recorded, best_idx=best_idx,
                                            use_G=g_supported, fallback_granularity=args.fallback_granularity,
                                            verify_frames=max(1, args.verify_frames), visualize=args.visualize,
                                            verbose=args.verbose, sleep_after_move=args.sleep_after_move,
                                            match_tolerance=args.match_tolerance)
        if match:
            print(f"Retrace accepted pos={match.position} metric={match.metric:.3f}")
            final_result = match
            # if medium or fine, run a local refine sweep around the matched position
            if args.focus_mode == "medium":
                w = args.medium_window
                s = args.medium_step
                print(f"Running medium local refine around {match.position} +/-{w} step={s}")
                local_best, local_samples = local_refine_sweep(ser, cap, match.position, window=w, step=s,
                                                               use_G=g_supported, fallback_granularity=args.fallback_granularity,
                                                               rpm=args.rpm, verify_frames=max(1, args.verify_frames),
                                                               sleep_after_move=args.sleep_after_move, visualize=args.visualize,
                                                               verbose=args.verbose, max_limit=(max_limit if max_limit is not None else 10**9))
                if local_best:
                    print(f"Medium refine best pos={local_best.position} metric={local_best.metric:.3f}")
                    final_result = local_best
            elif args.focus_mode == "fine":
                w = args.fine_window
                s = args.fine_step
                print(f"Running fine local refine around {match.position} +/-{w} step={s}")
                local_best, local_samples = local_refine_sweep(ser, cap, match.position, window=w, step=s,
                                                               use_G=g_supported, fallback_granularity=args.fallback_granularity,
                                                               rpm=args.rpm, verify_frames=max(1, args.verify_frames),
                                                               sleep_after_move=args.sleep_after_move, visualize=args.visualize,
                                                               verbose=args.verbose, max_limit=(max_limit if max_limit is not None else 10**9))
                if local_best:
                    print(f"Fine refine best pos={local_best.position} metric={local_best.metric:.3f}")
                    final_result = local_best
        else:
            print("No match found during retrace.")
            final_result = best

        print(f"Final chosen pos={final_result.position} metric={final_result.metric:.3f}")

        # show forward best frame and final frame (if any)
        try:
            if best.frame is not None:
                cv2.imshow("Forward best frame", best.frame)
            if final_result is not None and final_result.frame is not None:
                cv2.imshow("Final result frame", final_result.frame)
            if best.frame is not None or (final_result and final_result.frame is not None):
                print("Press any key in image windows to exit.")
                cv2.waitKey(0)
        except Exception:
            pass

    finally:
        try:
            ser.close()
        except Exception:
            pass
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()