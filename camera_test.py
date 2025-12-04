#!/usr/bin/env python3
"""
Quick test to check OpenCV can open the camera and save one JPEG frame.

Usage:
  python camera_test.py        # uses index 0
  python camera_test.py 1      # use index 1, etc
"""
import sys
import cv2
idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
print("Opening camera index", idx)
cap = cv2.VideoCapture(idx)
if not cap.isOpened():
    print("FAILED: cannot open camera index", idx)
    sys.exit(2)

ret, frame = cap.read()
if not ret or frame is None:
    print("FAILED: read returned no frame")
    cap.release()
    sys.exit(3)

out_file = "camera_test_frame.jpg"
ok = cv2.imwrite(out_file, frame)
cap.release()
if ok:
    print("WROTE:", out_file)
    print("Open that file to verify the image.")
    sys.exit(0)
else:
    print("FAILED: cv2.imwrite failed")
    sys.exit(4)