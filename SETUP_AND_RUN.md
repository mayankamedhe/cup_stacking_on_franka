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
| `PROBE_Z_STIFFNESS` | 150 N/m | Lower it if the probe dents cups. Raise it if base-up cups still read as about 10 mm sink. |
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
| Probe pushes or tips cups | Too stiff or too deep | Lower `PROBE_Z_STIFFNESS` or `PROBE_DEPTH`. |
| Fingers hit the cup during the side grasp | Cup too wide at the grip height, or not centred | Check the feasibility table in section 1. Check that the centring step moves the cup. |
| Arm stops on low side moves with no contact | frankapy virtual walls | Check `FC.WORKSPACE_WALLS` in frankapy. |
| "Wrist can't roll 180°" on every cup | Joint 7 isn't near ±1.57 at the side grasp | Measure `SIDE_ROTATION` again (step 7) with the wrist turned so joint 7 is near ±1.57. |
| Flipped cup lands tilted or bounces | Set-down height off | Check `TABLE_Z`, `CUP_HEIGHT` and `SIDE_GRASP_Z`. |
| Upper cups slide off | `CUP_SPACING` or `LAYER_HEIGHT` off | Measure them again (section 1). |

## 6. Nested rim-down stack source

This is an alternative to spreading source cups across the table. Keep all
source cups rim down, nested at one fixed XY, and start with an empty gripper.
Use a holder/fixture if the lower stack moves during extraction. Place the
source away from the pyramid and allow room for the hand's opening and shimmy.

Set these in `config.py`; the three `None` height settings intentionally require
measurements before any stack command can run:

| Setting | Measurement |
|---|---|
| `STACK_XY` | Centre of the source stack in robot base X/Y. |
| `STACK_SEARCH_TOP_Z` | Closed-finger **tool Z** safely above the tallest full stack, with room to establish a no-contact baseline. |
| `STACK_SEARCH_BOTTOM_Z` | Lowest safe probe tool Z, below the top of the final single cup but **above empty-table contact**. An empty stack must produce no contact inside this interval. |
| `STACK_GRASP_OFFSET_Z` | On one rim-down cup, measure closed-finger top-contact tool Z and your normal side-of-base grasp tool Z using the same orientation/tool offset. Enter **grasp Z minus contact Z**, a negative value. Do not enter cup height. |
| `STACK_MIN_TOOL_Z`, `STACK_MAX_TOOL_Z` | Application bounds for the whole routine. Maximum must allow `search top + CUP_HEIGHT + CLEARANCE` and the pyramid travel height. |

`PROBE_TOUCH_Z` is not used by this force search. Check `CUP_HEIGHT`, the normal
`GRASP_WIDTH` / `GRASP_FORCE`, and `GRIPPER_ROTATION` as well. Set the grasp offset
so only the exposed top cup is gripped, above the next nested cup's rim. If
there is not enough exposed cup wall for the fingertips, shaking cannot solve
the access problem; change fingers or use a dispenser fixture.

1. Update a copy of `stack_cups.example.json` with actual counts and colours,
   **topmost cup first**. Stack mode assumes all cups are rim-down and ignores
   table-layout orientation/XY annotations. A colour needed later cannot be
   accessed through cups above it.
2. Preview the access order without connecting:
   ```bash
   python main.py --from-stack --cups stack_cups.example.json \
     --goal "base yellow, middle orange, top red" --dry-run
   ```
3. Test height sensing alone. It closes the fingers, approaches from clearance,
   force-probes the current top, prints detected contact/grasp Z, and retracts
   without grasping:
   ```bash
   python main.py --stack-probe-only
   ```
   Confirm the reported top with a known single cup and again with a taller
   stack. Tune the contact threshold above the observed force noise. Default
   1.5 N is a detection threshold, not a guaranteed maximum force. Do not lower
   the search bottom to the table to make a missed stack "work".
4. Start with a one-cup goal and matching one-cup inventory, using `--from-stack
   --step`. It pauses with the cup held at clearance before placement. Inspect
   the grasp, the lower stack, and the separation motion. Adjust amplitude,
   duration, and rise gradually; defaults are 2 mm XY, 2° yaw, two loops.
5. For automated payload checks, collect repeated payload estimates from known
   **zero, one, and two cups**, with the same cup type, finger configuration,
   load model and clearance pose. Enter the one-cup centre in
   `STACK_SINGLE_WEIGHT` (N) and a repeatable `STACK_WEIGHT_TOLERANCE`. The band
   and observed noise must clearly separate all three cases. Noise allowance
   must fit within the tolerance, and tolerance must be less than half one
   cup's weight. If the estimates overlap, leave the check disabled and inspect
   manually, or add an independent counting sensor/dispenser. Never interpret
   successful shimmy completion as proof of a single cup.

On a search, extraction, grasp, or payload-check failure, the stack run stops
without placing, opening, or homing automatically. Check the physical robot
and recover before rerunning. Successful/failed runs log the contact height,
grasp height, measured payload, noise allowance, and band-check result when
available.
