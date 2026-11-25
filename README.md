# Cup Stacking: Task Planning

Give the robot a goal in plain English, like *"base layer of yellow, 2nd layer of orange, top red"*.
It plans which cup goes where, and in what order, then builds the pyramid with the Franka arm.

```
         [ red  ]
    [orange] [orange]
[yellow] [yellow] [yellow]
```

**To set up and run on the robot, follow [SETUP_AND_RUN.md](SETUP_AND_RUN.md). It lists every measurement and has a step-by-step first-run ladder.**

## Files
| File | What it does |
|---|---|
| `goal_parser.py` | English text → pyramid layers (list of colours, base first). Rule-based, offline. |
| `orientation.py` | Robot-free orientation logic: probe decision, 180° wrist-roll target, set-down height after a flip. |
| `stack_pickup.py` | Force-guided top detection and grasp, rising shimmy, optional payload-band verification for one rim-down source stack. |
| `stack_cups.example.json` | Example physical stack inventory, listed top to bottom. |
| `planner.py` | Builds the pyramid slot positions and runs an **A\*** planner. Returns an ordered list of (cup → slot). |
| `main.py` | Command line: parse → plan → print → run on the robot (frankapy) → log the trial. |
| `config.py` | **Every number you must measure** (cup size, grasp height, pyramid position, gripper rotation). |
| `cups.json` | Start position, colour and (optional) expected orientation of each cup. |
| `calibrate.py` | Hand-guide the arm, close/open the gripper, and print pose, width and joints, to fill in `config.py` and `cups.json`. |
| `test_planner.py` | Offline tests (no robot): parser cases, plan validity, optimality vs brute force, infeasibility. |

## Setup and assumptions
- Cups start on the table either **upside-down** (rim down, needed for stacking) or **rim-up**, spaced apart. Rim-up cups are detected and flipped in place before the build (see below). Keep about 15 cm clear around each cup for the flip.
- The pyramid is **2D**: a single row along world **y**, centred at `PYRAMID_CENTER`. Each upper cup sits across the upturned bases of the two cups below it.
- The gripper always comes from straight above, with fingers opening along **world x**. That is across the row, so the fingers never hit neighbouring cups.
- Slot 0 of each layer is at the **+y end**. For mixed layers (`"yellow red yellow"`), colours are listed from +y to −y.

## The planner (A\*)
| | |
|---|---|
| **State** | which slots are filled + which cups have been used + where the gripper is |
| **Action** | pick an unused cup and place it in an empty slot of the **same colour** whose **two support cups are already placed** |
| **Cost** | xy travel: gripper → cup + cup → slot |
| **Heuristic** | for each empty slot, the distance from the nearest unused cup of that colour, summed. This never overestimates, so the plan is the **shortest-travel** plan. |

Why A\*: the task is to choose *which* cup of a colour fills each slot (assignment) **and** the *order* (bottom layers first, precedence). A\* solves both together, optimally, in under 50 ms for a 10-cup pyramid. If the cups can't make the goal (not enough of a colour), it says so before the robot moves. This is the same approach as Lab 3.

`--layer-by-layer` makes the planner finish each layer before starting the next one. Without it, the planner may put an upper cup on top before the base row is complete, if that saves travel. That's an easy variable to compare in experiments.

## Picking from a nested rim-down stack

Use `--from-stack` when cups are nested at one fixed `STACK_XY`. All source cups
must already be **rim down**. This mode bypasses the table orientation scan and
flip passes. List the cups in physical **top-to-bottom order** in the cups JSON;
the planner only consumes that prefix in order and refuses plans that would
require reaching a buried cup. Remaining cups can stay at the bottom. JSON XY
values are overridden by `STACK_XY`, and table-orientation annotations are
ignored in favour of the rim-down stack assumption. Colours are inventory metadata,
not sensed; the physical stack must match the listed order.

```bash
python main.py --from-stack --cups stack_cups.example.json \
  --goal "base yellow, middle orange, top red" --dry-run

# After measuring the required STACK_* settings in config.py:
python main.py --stack-probe-only
python main.py --from-stack --cups stack_cups.example.json \
  --goal "base yellow, middle orange, top red" --step
```

For **each pickup**, the robot closes the empty fingers, takes a stationary
force baseline above the stack, and descends slowly with low Z stiffness. Three
consecutive upward base-frame Fz readings above `STACK_CONTACT_FORCE` trigger
a stop. The **measured tool Z at contact** sets the grasp height via
`STACK_GRASP_OFFSET_Z`. Heights are re-probed every time; no cup-height or
nesting-pitch subtraction is used. An empty/missed stack, excessive force,
aborted motion, or noisy baseline stops the operation instead of guessing Z.
Clearance height includes the maximum source-stack height and carried-cup height.

After grasping, a 3 mm initial lift is followed by two slow loops of ±2 mm XY
and alternating ±2° yaw, rising 4 mm per loop by default. The last waypoint
returns to the stack centre and original orientation; the cup is then raised
vertically to clearance. Tune `STACK_SHIMMY_*` for the cups and a fixture that
keeps the lower stack from sliding or tipping. These are bounded min-jerk
motions, not an impulsive high-speed shake. Extraction is monitored against
`STACK_MAX_FORCE` and `STACK_MAX_TORQUE`.

**Separation motion alone does not prove that exactly one cup was picked.**
The program reports a stationary payload estimate in N after each pickup.
Optional `STACK_SINGLE_WEIGHT` / `STACK_WEIGHT_TOLERANCE` compare that estimate
with a calibrated one-cup band. The empty and loaded samples use the same
clearance pose and similar finger width. Zero-cup / multi-cup readings outside
the band, or noise too large to resolve it, stop before placement. The band
tolerance must be less than half one cup's weight; measured noise adds an
uncertainty allowance. This remains an estimator-based check, sensitive to load
models, drift, and cup variability. For light cups below the arm estimator's
usable resolution, use an independent sensor/visual inspection or a mechanical
single-cup dispenser. With the band disabled, output explicitly says **cup count
UNVERIFIED**; `--step` lets you inspect the held cup before placement.

See [the stack setup instructions](SETUP_AND_RUN.md#6-nested-rim-down-stack-source)
for the three required height measurements. Failed stack runs request a stop,
keep the gripper commanded, and skip automatic opening/homing. Inspect the
robot before recovery. Monitoring runs through Python/ROS and cannot replace
the physical stop or guarantee a hard travel/force bound. Logs include measured
contact/grasp heights, payload estimates, and the payload-band result.

```bash
python -m unittest test_stack_pickup -v
```

## Cup orientation (probe, then flip)
A real run has three passes: **scan**, **flip**, **build**. Flipping in place doesn't move a cup, so the A\* plan is made first and stays valid.

1. **Scan (touch probe).** For each cup the plan uses, the closed gripper is lowered over the cup centre, aiming `PROBE_DEPTH` (2 cm) below the top of an upside-down cup. The arm is soft in z (`PROBE_Z_STIFFNESS`).
   - **Base up:** the solid base stops the fingers at the top, with a push of about 3 N.
   - **Rim up:** the fingers sink into the open rim.

   If they sink more than half of `PROBE_DEPTH`, the cup is rim-up.
   **Fallback, `PROBE_MODE = "gauge"` (diameter based):** the fingers are set to `GAUGE_WIDTH`, between the base and rim diameters. The narrow base of a base-up cup passes between them, so they sink. The wide rim of a rim-up cup stops them. It uses the same `PROBE_TOUCH_Z` and `PROBE_DEPTH`, but cup positions must be accurate to a few mm. Use `--probe-only` to compare the two modes.
   *Why not measure the diameter?* The taper is only about 1 cm (base about 70 mm, rim about 80 mm), and the Franka opens to 80 mm at most. Open fingers would land on the rim of a rim-up cup. The probe gives about a 2 cm signal and never has to reach around the rim.
2. **Flip each rim-up cup, in place.**
   1. *Centre:* the closed fingers go about 2 cm into the cup and **open** against the inner wall (the Franka can grip from the inside), once along x and once along y. The cup slides to within about 1 mm of the recorded xy. This matters because the next grasp has only about 3 mm of clearance per side.
   2. *Side grasp:* the hand is horizontal, pointing along +x, fingers opening along y, low on the cup (`SIDE_GRASP_Z`).
   3. Lift to `FLIP_Z`, then turn **joint 7 by 180°** (`goto_joints`). The cup is now upside-down.
   4. Lower until the rim touches the table, release, back out along −x, and go home.

   If joint 7 has no room for ±π (it must start near ±1.57), the robot puts the cup back, or never grasps it, and asks you to flip it by hand.
3. **Build** as before.

Flags:
- `--skip-orientation-check`: don't probe; trust `"orientation"` in `cups.json` (missing means `"down"`).
- `--probe-only`: probe every cup and compare with `cups.json`. This is the sensing-accuracy experiment.
- `--step`: also pauses before every probe and flip.

## Goal language
Separate layers with `,` `;` or `then`. Inside a layer, list colours with spaces or `and`.

| Example | Result (base first) |
|---|---|
| `base layer of yellow, 2nd layer of orange, top red` | Y Y Y / O O / R |
| `3 yellow at the bottom, then 2 orange, then 1 red` | same |
| `bottom: yellow red yellow; middle: blue blue; top: red` | Y R Y / B B / R |
| `base 2 red and 2 blue, then orange, then orange, top yellow` | R R B B / O O O / O O / Y |

Layer words: `base/bottom/first/1st`, `second/2nd`, `layer 3`, …, `middle`, `top/last`. A clause with no layer word goes on the next layer up. A single colour with no count fills the whole layer. Base size comes from any count you give; otherwise it equals the number of layers.

## Running it
```bash
python test_planner.py                                   # offline sanity check
python main.py --goal "base layer of yellow, 2nd layer of orange, top red" --dry-run
python main.py --goal "..." --step                       # on robot, Enter before each pick/place
python main.py --goal "..." --layer-by-layer
python main.py --probe-only                              # probe every cup, compare with cups.json
python main.py --goal "..." --skip-orientation-check     # trust cups.json orientations, no probing
```
`main.py` prints the plan and asks `Run on robot? [y/N]`. After the run it asks whether the pyramid came out correct, plus notes. It then appends a row to `experiments.csv` with goal, number of moves, moves completed, planned travel, plan time, execution time, success and notes. It also records `orientation_check`, `n_rim_up`, `n_flipped_robot`, `n_flipped_hand`, `orientation_mismatches` (probe result vs `cups.json`) and `probe_sink_mm` (how far the fingers sank for each cup). When new columns appear, older CSV files are rewritten with the merged header.

## Before the first real run (calibration checklist)
All values in `config.py` marked PLACEHOLDER. Use `python calibrate.py` (10 s of guide mode, then it prints the pose):
1. **`GRIPPER_ROTATION`**: guide the gripper pointing straight down, fingers opening toward/away from the robot (world x). Copy the printed rotation.
2. **`GRASP_Z`**: grasp a cup standing on the table by hand-guiding, with fingertips around the upturned base about 1.5 cm below its top. Copy z.
3. **`GRASP_WIDTH`**: the gripper width printed at that grasp, minus about 5 mm. Keep **`GRASP_FORCE`** low (about 5 N) so the cup isn't crushed.
4. **`CUP_HEIGHT`, `LAYER_HEIGHT`**: measure one cup, and the rim height of a cup resting on two others.
5. **`CUP_SPACING`**: rim diameter + about 5 mm. Check that an upper cup's rim still rests on both cups below it.
6. **`PYRAMID_CENTER`**: a free spot about 0.5 m in front of the robot.
7. **`cups.json`**: place each cup, guide the gripper to its centre, copy x, y.

8. **`PROBE_TOUCH_Z`**: close the gripper and guide the fingertips until they just touch the top of an upside-down cup. Copy z. Then run `--probe-only` on a mix of rim-up and rim-down cups. The `sink` values should be about 0 mm and about 20 mm.
9. **`TABLE_Z`**: fingertips touching the table. **`CENTER_Z`**: closed fingertips about 2 cm inside a rim-up cup.
10. **`SIDE_ROTATION`, `SIDE_GRASP_Z`, `SIDE_GRASP_WIDTH`**: guide the hand horizontal, pointing away from the robot, fingers around a rim-up cup as low as possible without the hand touching the table. Copy the rotation, z, and the width minus about 4 mm. Check the printed **joint 7 is near ±1.57**, otherwise the 180° roll won't fit. Test one flip with `--step` before trusting it. If low side moves stop unexpectedly, check frankapy's virtual walls (`FC.WORKSPACE_WALLS`).

First run: use `--dry-run`, then `--step` with your hand on the e-stop.

## Suggested experiments
- Success rate per pyramid size (3, 6, 10 cups), 5 to 10 trials each.
- Default planner vs `--layer-by-layer`: travel, time, failures.
- Parser accuracy: write about 20 goal sentences and count how many parse to the intended pyramid.
- Failure modes to note: missed grasp, cup crushed, upper cup slides off, structure knocked over.

## Limits and next steps
- Cup positions are measured by hand (no camera). Replacing `cups.json` with a perception step is the natural next step; the planner doesn't change.
- Open loop: the robot doesn't check whether a placement succeeded, or whether a flip worked. `--probe-only` afterwards can check flips.
- Cups lying on their side aren't handled.
- Only 2D pyramids. A 3D pyramid or nested tower would only need new slot geometry in `build_slots`.
