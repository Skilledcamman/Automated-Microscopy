# Microscope Z Autofocus (Tenengrad)

This adds a host-side Python autofocus routine for your Arduino-controlled Z axis.

- Arduino: `stepper_motor_z_axis/stepper_motor_z_axis.ino` (already supports homing `Z`, moves `G<n>`, position `P`, objective `O<n>`)
- Host script: `autofocus_tenengrad.py` (OpenCV Tenengrad sweep)

## Prerequisites
- Windows, Arduino on `COM9` (adjust with `--port`)
- Python 3.9+
- Camera at index `0` (adjust with `--camera`)

Install Python deps:

```powershell
cd "c:\Users\ahmed\Documents\microscope";
py -m venv .venv;
. .venv\Scripts\Activate.ps1;
python -m pip install --upgrade pip;
pip install -r requirements.txt;
```

## Run autofocus sweep

```powershell
cd "c:\Users\ahmed\Documents\microscope";
python autofocus_tenengrad.py --port COM9 --camera 0 --objective 10 --step 20 --rpm 12 --video --csv
```

- `--objective`: `4`, `10`, or `40` – sets exposure/gain only.
- `--step`: steps per sample (smaller = finer focus, slower)
- `--rpm`: motor speed during sweep
- `--video`: save `scan_YYYYMMDD_HHMMSS.avi`
- `--csv`: save `scan_YYYYMMDD_HHMMSS.csv` with `position_steps,tenengrad`
- `--roi x y w h`: optional center crop for focus metric

Workflow:
1. Homes Z with `Z` (pos 0 = physical stop)
2. Applies objective preset (exposure/gain only)
3. Queries max limit from Arduino with `Q`
4. Sweeps 0 → limit-50 in increments, computes Tenengrad, tracks best
5. Returns to best focus position

## Notes
- If exposure/gain fail to set (camera driver), the script warns and continues.
- You can change objective on the Arduino side anytime (O4/O10/O40). The script also sends it.
- To refine near best focus, rerun with smaller `--step` around the reported best position (future enhancement).
