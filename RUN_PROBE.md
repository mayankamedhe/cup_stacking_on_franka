# Run the Touch Probe

This tests only the orientation check: the robot touches each cup and says whether it is rim-up or rim-down. It does **not** grab, flip or stack anything.

---

## 1. Check `config.py`
- `PROBE_MODE = "touch"` (the default).
- `GRIPPER_ROTATION` and `GRASP_Z` are set (already filled in from your earlier runs).
- `CUP_HEIGHT` is your cup's height in metres, for example `0.11`.

## 2. Measure the probe height (one number)
```bash
python calibrate.py
```
1. Stand one cup **upside-down** (rim on the table).
2. Press **c** to close the gripper.
3. Press **Enter**, then guide the closed fingertips down until they **just touch the top of the cup**. Let go and wait for the printout.
4. Copy the printed **z** into `config.py` as `PROBE_TOUCH_Z`.
5. Press **q** to quit.

## 3. Put the cups out and record them in `cups.json`
For each cup:
1. Run `python calibrate.py`, press **Enter**, and guide the gripper over the **centre** of the cup.
2. Copy x and y into its `"xy"`.
3. Set its `"orientation"` to `"down"` (rim on the table) or `"up"` (rim on top). This is only used to score whether the probe got it right.

Use a mix: some cups rim-up, some rim-down.

```json
{"id": "y1", "color": "yellow", "xy": [0.52, -0.25], "orientation": "down"},
{"id": "o1", "color": "orange", "xy": [0.52, -0.15], "orientation": "up"}
```

## 4. Run the probe
Keep a hand on the **e-stop**.
```bash
python main.py --probe-only
```
The robot touches each cup and prints one line per cup:
```
  y1: down (sank   0.3 mm)  cups.json: down
  o1: up   (sank  19.8 mm)  cups.json: up
5/5 match cups.json
```
- About **0 mm** means a solid base, so the cup is **down**.
- About **20 mm** means the fingers went into the cup, so it's **up**.

## If it's wrong
| What you see | Fix |
|---|---|
| Every cup says `up` | `PROBE_TOUCH_Z` is too high. Measure step 2 again. |
| Every cup says `down` | `PROBE_TOUCH_Z` is too low, or a cup's `xy` is off-centre. Measure again. |
| The probe pushes cups around | Lower `PROBE_Z_STIFFNESS` in `config.py`, for example to `100`. |

---

Once every cup matches, the probe works. The full build with flipping needs the other measurements in [SETUP_AND_RUN.md](SETUP_AND_RUN.md).
