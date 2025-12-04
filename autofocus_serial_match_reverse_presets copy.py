#!/usr/bin/env python3
"""
autofocus_serial_match_reverse_presets.py

Forward sweep + reverse retrace autofocus with objective-based exposure/gain presets,
plus three autofocus presets (coarse, medium, fine) and a standalone refocus mode.

This file can be run as a CLI (same behavior as before) or imported and run in-process
via run_autofocus(ser, cap, options, log_callback=None, stop_event=None).

Key points for in-process usage:
 - Provide an already-open serial.Serial instance as `ser`.
 - Provide an already-open cv2.VideoCapture instance as `cap`.
 - Provide options via dict (same names as CLI arguments).
 - Provide log_callback(str) to receive progress/log messages (optional).
 - Provide a threading.Event stop_event to abort (optional).
"""

import argparse
import json
import os
import re
import time
from collections import namedtuple

import cv2
import numpy as np
import serial  # used by CLI runner; for in-process you still pass a ser instance

Result = namedtuple("Result", ["position", "metric", "frame", "timestamp"])
int_re = re.compile(r"(-?\d+)")
STEPS_PER_REV = 2048.0
DEFAULT_BAUD = 115200

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


# ---------------- Serial helpers that operate on a provided serial object ---------------
def send_command_ser(ser, cmd):
    """Send a command string to the provided serial.Serial instance."""
    if not cmd.endswith("\n"):
        cmd = cmd + "\n"
    ser.write(cmd.encode("ascii"))
    ser.flush()


def read_lines_ser(ser, timeout=0.4):
    """Read lines from the provided serial.Serial for `timeout` seconds."""
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
            s = repr(line)
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
    # also try a looser match
    for l in lines:
        if "unhomed" in l.lower() or "not homed" in l.lower():
            return False
    return None


def get_status_Q_ser(ser, verbose=False):
    """Send Q using provided ser and parse pos, max_limit, objective, homed flag, and raw lines."""
    send_command_ser(ser, "Q")
    lines = read_lines_ser(ser, timeout=0.6)
    if verbose and lines:
        print("[Q] reply:")
        for l in lines:
            print("  >", l)
    pos = parse_position_from_lines(lines)
    max_limit = parse_max_limit_from_lines(lines)
    objective = parse_objective_from_lines(lines)
    homed = parse_homed_from_lines(lines)
    return pos, max_limit, objective, homed, lines


def get_position_ser(ser, verbose=False, max_attempts=3):
    """Try P then Q fallback using provided ser."""
    for attempt in range(1, max_attempts + 1):
        send_command_ser(ser, "P")
        lines = read_lines_ser(ser, timeout=0.25 + 0.1 * attempt)
        if verbose and lines:
            print(f"[get_position attempt {attempt}] lines:")
            for l in lines:
                print("  >", l)
        pos = parse_position_from_lines(lines)
        if pos is not None:
            return pos
        # fallback to Q
        send_command_ser(ser, "Q")
        qlines = read_lines_ser(ser, timeout=0.35 + 0.1 * attempt)
        if verbose and qlines:
            print(f"[get_position fallback Q attempt {attempt}] lines:")
            for l in qlines:
                print("  >", l)
        pos = parse_position_from_lines(qlines)
        if pos is not None:
            return pos
        time.sleep(0.08)
    leftover = read_lines_ser(ser, timeout=0.12)
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


def probe_G_support_ser(ser, verbose=False):
    ser.reset_input_buffer()
    send_command_ser(ser, "G0")
    lines = read_lines_ser(ser, timeout=0.4)
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


def move_exact_G_ser(ser, delta, rpm, verbose=False):
    """Send G<delta> via provided ser and wait based on rpm, then read position."""
    if delta == 0:
        return get_position_ser(ser, verbose=verbose)
    send_command_ser(ser, f"G{int(delta)}")
    wait = compute_wait_seconds(delta, rpm)
    if verbose:
        print(f"Sent G{int(delta)}, waiting {wait:.3f}s")
    time.sleep(wait)
    return get_position_ser(ser, verbose=verbose)


def move_by_granular_ser(ser, delta, granularity, rpm, verbose=False):
    """Use U/D repeated commands via provided ser."""
    if delta == 0:
        return get_position_ser(ser, verbose=verbose)
    cmd = "U" if delta > 0 else "D"
    n = abs(delta) // granularity
    for _ in range(int(n)):
        send_command_ser(ser, cmd)
        time.sleep(compute_wait_seconds(granularity, rpm))
    return get_position_ser(ser, verbose=verbose)


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


def avg_metric_from_frames(cap, frames=5, delay=0.01, stop_event=None):
    metrics = []
    last_frame = None
    for _ in range(frames):
        if stop_event is not None and stop_event.is_set():
            break
        ret, frame = cap.read()
        if not ret:
            time.sleep(delay)
            continue
        last_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        metrics.append(tenengrad_metric(gray))
        time.sleep(delay)
    return (float(np.mean(metrics)) if metrics else 0.0), last_frame


# ---------------- forward sweep & local refine (use ser/cap) ----------------
def forward_sweep(ser, cap, step_size, rpm, use_G, fallback_granularity, verify_frames, visualize, verbose, sleep_after_move, stop_event=None):
    send_command_ser(ser, f"S{fallback_granularity}")
    time.sleep(0.06)
    send_command_ser(ser, f"V{int(rpm)}")
    time.sleep(0.06)

    start_pos, max_limit, _, _, _ = get_status_Q_ser(ser, verbose=verbose)
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
    metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames), stop_event=stop_event)
    results.append(Result(position=cur, metric=metric, frame=(frame.copy() if frame is not None else None), timestamp=time.time()))
    print(f"Sampled pos={cur} metric={metric:.3f}")

    while cur < max_limit:
        if stop_event is not None and stop_event.is_set():
            print("Forward sweep interrupted by stop_event.")
            break
        if use_G:
            newpos = move_exact_G_ser(ser, step_size, rpm, verbose=verbose)
        else:
            newpos = move_by_granular_ser(ser, step_size, fallback_granularity, rpm, verbose=verbose)
        time.sleep(sleep_after_move)
        if newpos is None:
            newpos, max_limit, _, _, _ = get_status_Q_ser(ser, verbose=verbose)
        if newpos is None:
            print("Can't read new position; stopping sweep.")
            break
        if newpos == cur:
            if verbose:
                print("Position unchanged after move (limit?). Stopping.")
            break
        cur = newpos
        metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames), stop_event=stop_event)
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


def local_refine_sweep(ser, cap, center_pos, window, step, use_G, fallback_granularity, rpm, verify_frames, sleep_after_move, visualize=False, verbose=False, stop_event=None, max_limit=10**9):
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
    cur = get_position_ser(ser, verbose=verbose)
    # Move to first pos
    if cur is None:
        raise RuntimeError("Cannot read current position for local_refine.")
    delta = positions[0] - cur
    if delta != 0:
        if use_G:
            move_exact_G_ser(ser, delta, rpm, verbose=verbose)
        else:
            move_by_granular_ser(ser, delta, fallback_granularity, rpm, verbose=verbose)
        time.sleep(sleep_after_move)
        cur = get_position_ser(ser, verbose=verbose)

    best = None
    for p in positions:
        if stop_event is not None and stop_event.is_set():
            print("Local refine interrupted by stop_event.")
            break
        # ensure we're at p (controller may be precise)
        cur = get_position_ser(ser, verbose=verbose)
        if cur != p:
            delta = p - (cur if cur is not None else 0)
            if delta != 0:
                if use_G:
                    move_exact_G_ser(ser, delta, rpm, verbose=verbose)
                else:
                    move_by_granular_ser(ser, delta, fallback_granularity, rpm, verbose=verbose)
                time.sleep(sleep_after_move)
        metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames), stop_event=stop_event)
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
        cur = get_position_ser(ser, verbose=verbose)
        delta = best.position - (cur if cur is not None else 0)
        if delta != 0:
            if use_G:
                move_exact_G_ser(ser, delta, rpm, verbose=verbose)
            else:
                move_by_granular_ser(ser, delta, fallback_granularity, rpm, verbose=verbose)
            time.sleep(sleep_after_move)
    return best, samples


def reverse_retrace_until_match(ser, cap, recorded_results, best_idx, use_G, fallback_granularity,
                                verify_frames, visualize, verbose, sleep_after_move, match_tolerance=0.08, stop_event=None):
    """
    Retrace recorded_results positions in reverse order.

    Behavior:
      - Accept strictly higher Tenengrad metric than forward-best.
      - Otherwise accept if relative difference <= match_tolerance.
    """
    forward_best = recorded_results[best_idx]
    forward_best_metric = forward_best.metric
    print(f"Forward best pos={forward_best.position} metric={forward_best_metric:.3f}")

    # ensure current pos known
    cur = get_position_ser(ser, verbose=verbose)
    if cur is None:
        raise RuntimeError("Cannot get current position before reverse retrace.")

    positions = [r.position for r in recorded_results]

    for target in reversed(positions):
        if stop_event is not None and stop_event.is_set():
            print("Retrace interrupted by stop_event.")
            break
        cur = get_position_ser(ser, verbose=verbose)
        if cur == target:
            metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames), stop_event=stop_event)
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
            moved = move_exact_G_ser(ser, delta, rpm=8, verbose=verbose)
        else:
            moved = move_by_granular_ser(ser, delta, fallback_granularity, rpm=8, verbose=verbose)
        time.sleep(sleep_after_move)
        cur = moved if moved is not None else get_position_ser(ser, verbose=verbose)

        metric, frame = avg_metric_from_frames(cap, frames=max(1, verify_frames), stop_event=stop_event)
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


# -------------------- run_autofocus in-process API --------------------
def run_autofocus(ser, cap, options, log_callback=None, stop_event=None):
    """
    Run the full autofocus flow in-process using provided serial.Serial and cv2.VideoCapture.

    options: dict of parameters (keys used below); defaults used when missing.
    log_callback: callable(str) used to emit logs (optional).
    stop_event: threading.Event used to request early stop (optional).

    Returns: Result(position, metric, frame, timestamp) for final chosen location (or None).
    Raises RuntimeError on fatal errors.
    """
    def log(msg):
        try:
            if log_callback:
                log_callback(str(msg))
            else:
                print(str(msg))
        except Exception:
            pass

    def should_stop():
        return (stop_event is not None and stop_event.is_set())

    # read options with defaults
    step_size = int(options.get("step_size", 50))
    rpm = float(options.get("rpm", 8.0))
    visualize = bool(options.get("visualize", False))
    verify_frames = int(options.get("verify_frames", 5))
    sleep_after_move = float(options.get("sleep_after_move", 0.12))
    fallback_granularity = int(options.get("fallback_granularity", 8))
    match_tolerance = float(options.get("match_tolerance", 0.08))
    focus_mode = options.get("focus_mode", "coarse")
    medium_window = int(options.get("medium_window", 200))
    medium_step = int(options.get("medium_step", 100))
    fine_window = int(options.get("fine_window", 80))
    fine_step = int(options.get("fine_step", 20))
    refocus_only = bool(options.get("refocus_only", False))
    refocus_window = int(options.get("refocus_window", 200))
    refocus_step = int(options.get("refocus_step", 20))
    exposure = options.get("exposure", None)
    gain = options.get("gain", None)
    presets_file = options.get("presets_file", "")

    # Load presets if requested
    presets = DEFAULT_PRESETS.copy()
    if presets_file:
        if os.path.exists(presets_file):
            try:
                presets = load_presets(presets_file)
                log(f"Loaded presets from {presets_file}")
            except Exception as e:
                log(f"Failed to load presets file, falling back to defaults: {e}")

    # Probe G support
    try:
        g_supported = probe_G_support_ser(ser, verbose=False)
    except Exception as e:
        raise RuntimeError(f"Failed to probe controller G support: {e}")

    log(f"G supported: {g_supported}")

    # Query Q to find current objective and homed state
    pos, max_limit, objective, homed, qlines = get_status_Q_ser(ser, verbose=False)
    if log_callback:
        log(f"Q lines: {qlines}")
    if objective:
        log(f"Detected objective: {objective}")
        preset = presets.get(str(objective))
        if preset:
            exp_v = preset.get("exposure")
            gain_v = preset.get("gain")
            log(f"Applying preset for objective {objective}: exposure={exp_v} gain={gain_v}")
            try_set_exposure_and_gain(cap, exposure_value=exp_v, gain_value=gain_v, verbose=False)
        else:
            log(f"No preset for objective {objective}; leaving camera exposure/gain as-is.")
    else:
        log("Could not parse objective from controller Q response; not applying presets.")

    # Override exposure/gain if provided explicitly
    if exposure is not None or gain is not None:
        log(f"Overriding exposure/gain from options: exposure={exposure} gain={gain}")
        try_set_exposure_and_gain(cap, exposure_value=exposure, gain_value=gain, verbose=False)

    # If refocus-only requested, run local_refine around current pos and return
    if refocus_only:
        cur = get_position_ser(ser, verbose=False)
        if cur is None:
            raise RuntimeError("Could not read current position for refocus.")
        vf = verify_frames
        log(f"Running refocus around current pos={cur} +/-{refocus_window} step={refocus_step}")
        best, samples = local_refine_sweep(ser, cap, cur, window=refocus_window, step=refocus_step,
                                           use_G=g_supported, fallback_granularity=fallback_granularity,
                                           rpm=rpm, verify_frames=max(1, vf), sleep_after_move=sleep_after_move,
                                           visualize=visualize, verbose=False, stop_event=stop_event,
                                           max_limit=(max_limit if max_limit is not None else 10**9))
        if best:
            log(f"Refocus best pos={best.position} metric={best.metric:.3f}")
        else:
            log("Refocus found no samples.")
        return best

    # --- Ensure controller is homed before starting main autofocus ---
    if homed is True:
        log("Controller reports homed. Proceeding.")
    else:
        log("Controller not reported homed. Sending hard-home (Z) and waiting for homed state...")
        send_command_ser(ser, "Z")
        wait = compute_wait_seconds(10000, rpm) + 0.5
        if should_stop():
            raise RuntimeError("Autofocus stopped during homing wait")
        # wait in chunks so stop_event can be honored
        deadline_wait = time.time() + wait
        while time.time() < deadline_wait:
            if should_stop():
                raise RuntimeError("Autofocus stopped during homing wait")
            time.sleep(0.1)

        # Poll Q until homed==True or timeout
        deadline = time.time() + max(12.0, wait + 6.0)
        pos, max_limit, objective, homed, qlines = get_status_Q_ser(ser, verbose=False)
        while (homed is not True) and time.time() < deadline:
            if should_stop():
                raise RuntimeError("Autofocus stopped while waiting for homing")
            time.sleep(0.4)
            pos, max_limit, objective, homed, qlines = get_status_Q_ser(ser, verbose=False)

        if homed is not True:
            raise RuntimeError("Controller did not report homed after issuing Z; aborting autofocus.")
        log("Controller reports homed. Proceeding with autofocus.")
        if pos is None:
            pos = get_position_ser(ser, verbose=False)

    # forward sweep (coarse)
    recorded = forward_sweep(ser, cap, step_size=step_size, rpm=rpm,
                             use_G=g_supported, fallback_granularity=fallback_granularity,
                             verify_frames=max(1, verify_frames), visualize=visualize,
                             verbose=False, sleep_after_move=sleep_after_move, stop_event=stop_event)
    if not recorded:
        log("No forward samples collected.")
        return None

    best_idx = int(np.argmax([r.metric for r in recorded]))
    best = recorded[best_idx]
    log(f"Forward-best: idx={best_idx} pos={best.position} metric={best.metric:.3f}")

    # Reverse retrace to find matching (or higher) position
    match = reverse_retrace_until_match(ser, cap, recorded_results=recorded, best_idx=best_idx,
                                        use_G=g_supported, fallback_granularity=fallback_granularity,
                                        verify_frames=max(1, verify_frames), visualize=visualize,
                                        verbose=False, sleep_after_move=sleep_after_move,
                                        match_tolerance=match_tolerance, stop_event=stop_event)
    if match:
        log(f"Retrace accepted pos={match.position} metric={match.metric:.3f}")
        final_result = match
        # if medium or fine, run a local refine sweep around the matched position
        if focus_mode == "medium":
            w = medium_window
            s = medium_step
            log(f"Running medium local refine around {match.position} +/-{w} step={s}")
            local_best, local_samples = local_refine_sweep(ser, cap, match.position, window=w, step=s,
                                                           use_G=g_supported, fallback_granularity=fallback_granularity,
                                                           rpm=rpm, verify_frames=max(1, verify_frames),
                                                           sleep_after_move=sleep_after_move, visualize=visualize,
                                                           verbose=False, stop_event=stop_event,
                                                           max_limit=(max_limit if max_limit is not None else 10**9))
            if local_best:
                log(f"Medium refine best pos={local_best.position} metric={local_best.metric:.3f}")
                final_result = local_best
        elif focus_mode == "fine":
            w = fine_window
            s = fine_step
            log(f"Running fine local refine around {match.position} +/-{w} step={s}")
            local_best, local_samples = local_refine_sweep(ser, cap, match.position, window=w, step=s,
                                                           use_G=g_supported, fallback_granularity=fallback_granularity,
                                                           rpm=rpm, verify_frames=max(1, verify_frames),
                                                           sleep_after_move=sleep_after_move, visualize=visualize,
                                                           verbose=False, stop_event=stop_event,
                                                           max_limit=(max_limit if max_limit is not None else 10**9))
            if local_best:
                log(f"Fine refine best pos={local_best.position} metric={local_best.metric:.3f}")
                final_result = local_best
    else:
        log("No match found during retrace.")
        final_result = best

    log(f"Final chosen pos={final_result.position} metric={final_result.metric:.3f}")

    # Optionally show frames if visualize and frames exist (caller may display)
    return final_result


# -------------------- CLI compatibility main --------------------
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
    parser.add_argument("--no-set-home", action="store_true", help="(deprecated) do not send H0 at startup")
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

    # open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera index " + str(args.camera))

    # open serial
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.8)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # build options dict for run_autofocus
    options = {
        "step_size": args.step_size,
        "rpm": args.rpm,
        "visualize": args.visualize,
        "verify_frames": args.verify_frames,
        "sleep_after_move": args.sleep_after_move,
        "fallback_granularity": args.fallback_granularity,
        "match_tolerance": args.match_tolerance,
        "focus_mode": args.focus_mode,
        "medium_window": args.medium_window,
        "medium_step": args.medium_step,
        "fine_window": args.fine_window,
        "fine_step": args.fine_step,
        "refocus_only": args.refocus_only,
        "refocus_window": args.refocus_window,
        "refocus_step": args.refocus_step,
        "exposure": args.exposure,
        "gain": args.gain,
        "presets_file": args.presets_file,
        "verbose": args.verbose,
    }

    try:
        result = run_autofocus(ser, cap, options, log_callback=print, stop_event=None)
        print("Autofocus finished:", result)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()