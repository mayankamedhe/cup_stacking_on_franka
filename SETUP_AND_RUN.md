# Setup and Run Guide

This guide takes you from a fresh setup to a full pyramid build. Follow the steps in order. For how the system works, see [README.md](README.md).

Every number goes into [config.py](config.py) or [cups.json](cups.json). Units are **metres** in the **robot base frame**: +x is away from the robot, +z is up. Every value marked `PLACEHOLDER` must be measured before you run on the robot.

---

## 0. Before you start
- frankapy is working on the control PC (`from frankapy import FrankaArm` runs), and the arm is unlocked and in the right mode.
- Someone keeps a hand on the **e-stop** for every robot step below.
- Keep the table clear except for the cups. A flip needs about **15 cm of free space around the cup**, and about **10 cm behind it** (on the robot side).
- The offline tests pass. They don't need the robot:
  ```bash
  python test_planner.py
  ```

---

## 1. Measure the cup with a ruler or calipers (no robot)

| # | Measure | Write it down as | Used for |
|---|---|---|---|
| 1 | Outer diameter of the **rim** (open end) | `RIM` | `CUP_SPACING`, `GAUGE_WIDTH`, feasibility checks |
| 2 | Outer diameter of the **base** (closed end) | `BASE` | feasibility checks, `GAUGE_WIDTH` |
| 3 | Cup **height** | `CUP_HEIGHT` | travel height, flip set-down height |
| 4 | Stack one cup on two others. Height of the **top cup's rim** above the table | `LAYER_HEIGHT` | place heights |

Then set:
- `CUP_SPACING` = `RIM` + about 5 mm.
- `LAYER_HEIGHT` = the measured value. It's usually a few mm less than `CUP_HEIGHT`.

### Feasibility checks (do these before going further)
The Franka gripper opens to **80 mm at most**. The cup's width at a distance `d` from the **base** is:

```
width(d) = BASE + (RIM - BASE) * d / CUP_HEIGHT
```

| Check | Needs to be true | If it isn't |
|---|---|---|
| Normal top-down grasp: width 1.5 cm below the top of an upside-down cup = `width(0.015)` | ≤ 74 mm | Cups can't be picked at all. You need smaller cups. |
| Side grasp for the flip: width at the grip height of a rim-up cup = `width(SIDE_GRASP_Z − TABLE_Z)`. The grip height is about 4 cm, the lowest point where the hand still clears the table. | ≤ 74 mm, which leaves about 3 mm per side | **The automatic flip won't work.** Flip by hand: the robot asks you to whenever it can't flip a cup. Or rethink the flip. |
| Only for `PROBE_MODE = "gauge"` | `width(PROBE_DEPTH)` + 6 mm < `GAUGE_WIDTH` < `RIM` − 6 mm | Use `"touch"` mode, which has no width limits. |

Example: with `BASE` = 60 and `RIM` = 80 mm on a 110 mm cup, the side grasp at 4 cm is about 67 mm wide, which is fine. With `BASE` = 72 and `RIM` = 92 mm, it's about 79 mm, which doesn't fit.

---

## 2. Measure on the robot with `calibrate.py`

```bash
python calibrate.py
```
At the prompt:

| Key | Action |
|---|---|
| **Enter** | 10 s of guide mode (move the arm by hand), then print the pose |
| **c** | Close the gripper gently |
| **o** | Open the gripper |
| **p** | Print the pose now |
| **q** | Quit |

Each printout shows the **translation** (x, y, z), **rotation**, **gripper width** and **joints**.

| # | What to do | Copy into |
|---|---|---|
| 1 | Press **p** right after start. The arm is at its home pose. | `HOME_XY` = (x, y) |
| 2 | Guide the gripper to point **straight down**, fingers opening toward and away from the robot (world x). | `GRIPPER_ROTATION` = rotation |
| 3 | Stand a cup **upside-down** on the table. Guide the open fingers around its base, about 1.5 cm below the top. Press **c**, then **p**. | `GRASP_Z` = z. `GRASP_WIDTH` = width − about 5 mm. |
| 4 | Press **o**, lift, then press **c** with nothing in the gripper. Guide the closed fingertips down until they **just touch the top** of the same cup. | `PROBE_TOUCH_Z` = z |
| 5 | Closed fingertips **just touching the table**. | `TABLE_Z` = z |
| 6 | Turn a cup **rim-up**. Put the closed fingertips about **2 cm inside** the cup, at its centre. | `CENTER_Z` = z |
| 7 | Same rim-up cup. Press **o**. Guide the hand **horizontal**, pointing **away from the robot** (+x), fingers opening sideways (along y), around the cup **as low as possible without the hand touching the table**. Press **c**, then **p**. | `SIDE_ROTATION` = rotation. `SIDE_GRASP_Z` = z. `SIDE_GRASP_WIDTH` = width − about 4 mm. **Check that joint 7 (the last joint value) is near +1.57 or −1.57.** If it isn't, the 180° wrist turn won't fit. |
| 8 | Pick a free spot about 0.5 m in front of the robot for the **middle of the pyramid's base row**. Point the gripper at it. | `PYRAMID_CENTER` = (x, y) |
| 9 | Put out the cups. For each one, guide the gripper over its **centre**. | `cups.json`: `"xy": [x, y]` |
| 10 | In `cups.json`, set each cup's `"orientation"` (`"down"` = rim on table, `"up"` = rim on top). This is optional. It's used for `--skip-orientation-check` and to score probe accuracy. | `cups.json` |

Also check that every cup and the pyramid are inside `WORKSPACE`. `main.py` refuses to run otherwise.

### Quick hardware check (once)
Place a rim-up cup, put the closed fingers about 2 cm inside it (step 6), then in a Python shell:
```python
fa.goto_gripper(0.08, grasp=True, force=5.0)
```
The fingers should **spread and press on the inside wall**, which centres the cup. If they don't, tell whoever maintains the code. The flip's centring step depends on this.

---

## 3. Settings you tune by testing (don't measure these)

| Setting | Default | When to change it |
|---|---|---|
| `PROBE_MODE` | `"touch"` | Switch to `"gauge"` only if touch keeps getting it wrong. |
| `PROBE_DEPTH` | 0.02 | Lower it if the probe pushes cups around. Raise it if the sink values for "up" and "down" are too close. |
| `PROBE_FORCE` | 4 N | Contact threshold for the probe descent. Lower it if the probe dents cups. Raise it if rim-up cups stop early on force noise. |
| `GRASP_FORCE`, `CENTER_FORCE` | 5 N | Lower them if cups crush. Raise them if cups slip. |
| `FLIP_Z` | 0.25 | Raise it if the hand or cup comes close to anything during the wrist turn. |
| `SIDE_APPROACH` | 0.10 | Raise it if the horizontal hand touches the cup on the way in. |
| `*_DURATION` | 2–4 s | Raise them for slower, safer motion. |

---

## 4. Run it, slowest and safest first
Move to the next step only when the current one works.

```bash
# 1. Plan only. Shows the pyramid, the order, and which cups cups.json says are rim-up.
python main.py --goal "base layer of yellow, 2nd layer of orange, top red" --dry-run

# 2. Probe test, no building. Use a mix of rim-up and rim-down cups.
#    Expect "sank" ~0 mm for down and ~20 mm for up (touch mode). Fix PROBE_TOUCH_Z if not.
python main.py --probe-only

# 3. Full run, pausing before every probe, flip, pick and place.
#    For the FIRST flip, watch the side approach and the wrist turn closely.
python main.py --goal "..." --step

# 4. Full run without pauses.
python main.py --goal "..."
```

Other options:
- `--skip-orientation-check`: don't probe; trust `"orientation"` in `cups.json`.
- `--layer-by-layer`: finish each layer before starting the next.

A real run goes: **probe every cup the plan uses → flip the rim-up ones in place → build**. If the wrist can't turn 180° for a cup, the robot stops and asks you to **flip that cup by hand** (rim down, same spot), then press Enter.

After each run it asks whether the pyramid came out right, plus any notes, and appends a row to `experiments.csv`.

---

## 5. Troubleshooting

| What you see | Likely cause | Fix |
|---|---|---|
| Every cup reads `up` | `PROBE_TOUCH_Z` too high | Measure step 4 again. |
| Every cup reads `down` | `PROBE_TOUCH_Z` too low, or `cups.json` xy is off the cup centre | Measure step 4 or step 9 again. |
| Probe pushes or tips cups | Too stiff or too deep | Lower `PROBE_FORCE` or raise `PROBE_DURATION`. |
| Fingers hit the cup during the side grasp | Cup too wide at the grip height, or not centred | Check the feasibility table in section 1. Check that the centring step moves the cup. |
| Arm stops on low side moves with no contact | frankapy virtual walls | Check `FC.WORKSPACE_WALLS` in frankapy. |
| "Wrist can't roll 180°" on every cup | Joint 7 isn't near ±1.57 at the side grasp | Measure `SIDE_ROTATION` again (step 7) with the wrist turned so joint 7 is near ±1.57. |
| Flipped cup lands tilted or bounces | Set-down height off | Check `TABLE_Z`, `CUP_HEIGHT` and `SIDE_GRASP_Z`. |
| Upper cups slide off | `CUP_SPACING` or `LAYER_HEIGHT` off | Measure them again (section 1). |
