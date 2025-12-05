import argparse
import csv
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import serial
import serial.tools.list_ports


DEFAULT_PRESETS = {
    "4": {"exposure": -13, "gain": 0},
    "10": {"exposure": -12, "gain": 15},
    "40": {"exposure": -8, "gain": 8},
}


class ZAxisController:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2.0)
        self._drain()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _drain(self):
        if not self.ser:
            return
        try:
            while self.ser.in_waiting:
                _ = self.ser.readline()
        except Exception:
            pass

    def _send(self, cmd: str):
        if not cmd.endswith("\n"):
            cmd = cmd + "\n"
        self.ser.write(cmd.encode("utf-8"))

    def _read_until(self, predicate, timeout: float = 5.0):
        start = time.time()
        lines = []
        while time.time() - start < timeout:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                lines.append(line)
                if predicate(line):
                    return lines
        return lines

    def home(self, raise_steps: int = 0):
        # Z or Z<n>; wait for "Homing complete."
        self._send(f"Z{raise_steps}")
        lines = self._read_until(lambda l: "Homing complete" in l, timeout=20.0)
        return lines

    def set_objective(self, mag: str):
        self._send(f"O{mag}")
        time.sleep(0.05)
        self._drain()

    def set_speed_rpm(self, rpm: int):
        self._send(f"V{rpm}")
        time.sleep(0.05)
        self._drain()

    def move_steps(self, steps: int):
        # G<n> move signed steps; blocking on Arduino until done
        self._send(f"G{steps}")
        # wait for movement echo line "Moved G. pos=..." or limit message
        lines = self._read_until(lambda l: l.startswith("Moved G.") or "Limit reached" in l, timeout=5.0)
        # Try to parse position from echo
        for l in lines[::-1]:
            if l.startswith("Moved G.") and "pos=" in l:
                try:
                    return int(l.split("pos=", 1)[1].strip())
                except Exception:
                    pass
        return None

    def get_position(self) -> int:
        # Drain any stale output, then query position and parse robustly
        self._drain()
        self._send("P")
        lines = self._read_until(lambda l: l.startswith("Position:") or l.startswith("Moved G."), timeout=2.0)
        for l in lines[::-1]:
            if l.startswith("Position:"):
                try:
                    return int(l.split(":", 1)[1].strip())
                except Exception:
                    pass
            if l.startswith("Moved G.") and "pos=" in l:
                try:
                    return int(l.split("pos=", 1)[1].strip())
                except Exception:
                    pass
        # fallback: query Q
        self._send("Q")
        lines = self._read_until(lambda l: l.startswith("Position:") or l.startswith("Homed:"), timeout=2.0)
        for l in lines:
            if l.startswith("Position:"):
                try:
                    return int(l.split(":", 1)[1].strip())
                except Exception:
                    pass
        raise RuntimeError("Could not parse position from controller output")

    def get_max_limit(self) -> int:
        self._send("Q")
        lines = self._read_until(lambda l: l.startswith("EEPROM write interval") or l.startswith("RPM:"), timeout=1.5)
        limit = None
        for l in lines:
            if "Max limit:" in l:
                try:
                    limit = int(l.split("Max limit:", 1)[1].strip())
                except Exception:
                    pass
        if limit is None:
            raise RuntimeError("Could not determine max limit from controller (Q)")
        return limit

    def release(self):
        self._send("R")
        time.sleep(0.05)
        self._drain()


def tenengrad_focus(img_gray: np.ndarray) -> float:
    g_x = cv2.Sobel(img_gray, cv2.CV_64F, 1, 0, ksize=3)
    g_y = cv2.Sobel(img_gray, cv2.CV_64F, 0, 1, ksize=3)
    fm = (g_x ** 2 + g_y ** 2).mean()
    return float(fm)


def set_camera_properties(cap: cv2.VideoCapture, exposure: int, gain: int):
    # Apply exposure and gain only (as requested)
    ok_exp = cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
    ok_gain = cap.set(cv2.CAP_PROP_GAIN, float(gain))
    return ok_exp and ok_gain


def main():
    parser = argparse.ArgumentParser(description="Autofocus sweep using Tenengrad via OpenCV and Arduino Z-axis")
    parser.add_argument("--port", default="COM9", help="Arduino serial port (default COM9)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--objective", choices=["4", "10", "40"], default="4", help="Objective magnification")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index (default 0)")
    parser.add_argument("--step", type=int, default=20, help="Steps per sample (smaller = finer, slower)")
    parser.add_argument("--rpm", type=int, default=12, help="Motor speed RPM during sweep")
    parser.add_argument("--margin", type=int, default=50, help="Do not scan last N steps near limit")
    parser.add_argument("--video", action="store_true", help="Save scan video")
    parser.add_argument("--csv", action="store_true", help="Save CSV of position and focus")
    parser.add_argument("--roi", type=int, nargs=4, metavar=("x","y","w","h"), help="Optional ROI for focus calc")
    parser.add_argument("--live", action="store_true", help="Show live view window during sweep")
    parser.add_argument("--fine-step", type=int, default=10, help="Step size for post-move fine adjustments")
    parser.add_argument("--adjust-threshold", type=float, default=0.8, help="Trigger adjustment if final/best < threshold")
    parser.add_argument("--adjust-iterations", type=int, default=5, help="Max iterations for directional fine adjustment")
    args = parser.parse_args()

    preset = DEFAULT_PRESETS[args.objective]

    # Open serial
    z = ZAxisController(args.port, args.baud)
    z.open()

    # Home first (no raise so pos 0 = physical stop)
    print("Homing Z ...")
    z.home(raise_steps=0)

    # Select objective (sets internal max limit table)
    z.set_objective(args.objective)
    max_limit = z.get_max_limit()
    print(f"Max travel limit reported: {max_limit} steps")

    # Optionally set scanning speed
    z.set_speed_rpm(args.rpm)

    # Open camera
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            raise RuntimeError(f"Unable to open camera index {args.camera}")

    ok = set_camera_properties(cap, preset["exposure"], preset["gain"])
    if not ok:
        print("Warning: failed to set exposure/gain as requested; continuing.")

    # Prepare outputs (static names to avoid stacking)
    video_writer = None
    csv_file = None
    csv_writer = None
    if args.video:
        ret, frame0 = cap.read()
        if not ret:
            raise RuntimeError("Failed to read initial frame from camera")
        h, w = frame0.shape[:2]
        # Ensure writer opens; fallback to XVID if MJPG fails
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        video_writer = cv2.VideoWriter("scan.avi", fourcc, 20.0, (w, h))
        if not video_writer.isOpened():
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            video_writer = cv2.VideoWriter("scan.avi", fourcc, 20.0, (w, h))
            if not video_writer.isOpened():
                raise RuntimeError("Video writer failed to open. Try without --video or check codec support.")
        video_writer.write(frame0)
    if args.csv:
        csv_file = open("scan.csv", "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["position_steps", "tenengrad"]) 

    # Ensure at position 0
    pos = z.get_position()
    if pos != 0:
        z.move_steps(-pos)
        pos = z.get_position()

    best_focus = -1.0
    best_pos = 0
    history = []  # list of (pos, fm)

    # Sweep from 0 to max_limit - margin in increments
    target_end = max(0, max_limit - args.margin)
    print(f"Starting sweep: 0 -> {target_end} steps, step={args.step}")

    while True:
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed; aborting sweep.")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if args.roi:
            x, y, w, h = args.roi
            gray = gray[y:y+h, x:x+w]
        fm = tenengrad_focus(gray)

        # Live view overlay
        if args.live:
            overlay = frame.copy()
            txt = f"pos={pos}  tenengrad={fm:.2f}"
            cv2.putText(overlay, txt, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
            cv2.imshow("Microscope Live", overlay)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                print("User aborted via live view.")
                break

        # Record
        if video_writer is not None:
            video_writer.write(frame)
        if csv_writer is not None:
            csv_writer.writerow([pos, f"{fm:.6f}"])
        history.append((pos, fm))

        # Track best
        if fm > best_focus:
            best_focus = fm
            best_pos = pos

        # Step forward
        if pos + args.step >= target_end:
            break
        z.move_steps(args.step)
        pos += args.step

    print(f"Sweep complete. Best focus {best_focus:.3f} at pos {best_pos}")

    # Move to best position
    current_pos = z.get_position()
    delta = best_pos - current_pos
    if delta != 0:
        print(f"Moving to best position: delta {delta} steps")
        z.move_steps(delta)
    # After moving there, just check focus score and log
    ret, final_frame = cap.read()
    if ret:
        final_gray = cv2.cvtColor(final_frame, cv2.COLOR_BGR2GRAY)
        if args.roi:
            x, y, w, h = args.roi
            final_gray = final_gray[y:y+h, x:x+w]
        final_fm = tenengrad_focus(final_gray)
        final_pos = z.get_position()
        print(f"Final check at pos {final_pos}: tenengrad={final_fm:.6f}")
        if csv_writer is not None:
            csv_writer.writerow([final_pos, f"{final_fm:.6f}"])
        if video_writer is not None:
            video_writer.write(final_frame)

        # If final focus is significantly worse than best, perform sweeps until same or better
        if best_focus > 0 and final_fm < best_focus * args.adjust_threshold:
            print("Final focus below target; sweeping to find equal or better score.")
            start_pos = final_pos

            def measure_and_log() -> tuple[float, np.ndarray]:
                ok, frame_local = cap.read()
                if not ok:
                    return -1.0, None
                gray_local = cv2.cvtColor(frame_local, cv2.COLOR_BGR2GRAY)
                if args.roi:
                    x, y, w, h = args.roi
                    gray_local = gray_local[y:y+h, x:x+w]
                fm_local = tenengrad_focus(gray_local)
                cur_pos = z.get_position()
                print(f"Sweep check at pos {cur_pos}: tenengrad={fm_local:.6f}")
                if csv_writer is not None:
                    csv_writer.writerow([cur_pos, f"{fm_local:.6f}"])
                if video_writer is not None:
                    video_writer.write(frame_local)
                return fm_local, frame_local

            found = False
            # Sweep forward
            steps_taken = 0
            while steps_taken < args.adjust_iterations * args.fine_step:
                z.move_steps(args.fine_step)
                steps_taken += args.fine_step
                fm_curr, _ = measure_and_log()
                if fm_curr >= best_focus * args.adjust_threshold:
                    print("Reached equal or better focus during forward sweep.")
                    found = True
                    break
            if not found:
                # Return to start
                z.move_steps(-steps_taken)
                # Sweep backward
                steps_taken = 0
                while steps_taken < args.adjust_iterations * args.fine_step:
                    z.move_steps(-args.fine_step)
                    steps_taken += args.fine_step
                    fm_curr, _ = measure_and_log()
                    if fm_curr >= best_focus * args.adjust_threshold:
                        print("Reached equal or better focus during backward sweep.")
                        found = True
                        break
                if not found:
                    # Return to start
                    z.move_steps(steps_taken)
                    print("Sweep did not reach target focus; staying at start position.")
    else:
        print("Warning: unable to capture final frame for focus check.")

    # Cleanup
    if video_writer is not None:
        video_writer.release()
    if csv_file is not None:
        csv_file.close()
    cap.release()
    if args.live:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    z.release()
    z.close()
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(2)
