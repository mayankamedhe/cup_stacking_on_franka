# Cup Stacking: Task Planning

Give the robot a goal in plain English, like *"base layer of yellow, 2nd layer of orange, top red"*.
It plans which cup goes where, and in what order, then builds the pyramid with the Franka arm.

```
         [ red  ]
    [orange] [orange]
[yellow] [yellow] [yellow]
```

## Files
| File | What it does |
|---|---|
| `goal_parser.py` | English text → pyramid layers (list of colours, base first). Rule-based, offline. |
| `planner.py` | Builds the pyramid slot positions and runs an **A\*** planner. Returns an ordered list of (cup → slot). |
| `main.py` | Command line: parse → plan → print → run on the robot (frankapy) → log the trial. |
| `config.py` | **Every number you must measure** (cup size, grasp height, pyramid position, gripper rotation). |
| `cups.json` | Start position and colour of each cup. |
| `calibrate.py` | Hand-guide the arm and print its pose, to fill in `config.py` and `cups.json`. |
| `test_planner.py` | Offline tests (no robot): parser cases, plan validity, optimality vs brute force, infeasibility. |

## Setup and assumptions
- Cups stand **upside-down** on the table (rim down), spaced apart.
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
```
`main.py` prints the plan and asks `Run on robot? [y/N]`. After the run it asks whether the pyramid came out correct, plus notes. It then appends a row to `experiments.csv` with goal, number of moves, moves completed, planned travel, plan time, execution time, success and notes.

## Before the first real run (calibration checklist)
All values in `config.py` marked PLACEHOLDER. Use `python calibrate.py` (10 s of guide mode, then it prints the pose):
1. **`GRIPPER_ROTATION`**: guide the gripper pointing straight down, fingers opening toward/away from the robot (world x). Copy the printed rotation.
2. **`GRASP_Z`**: grasp a cup standing on the table by hand-guiding, with fingertips around the upturned base about 1.5 cm below its top. Copy z.
3. **`GRASP_WIDTH`**: the gripper width printed at that grasp, minus about 5 mm. Keep **`GRASP_FORCE`** low (about 5 N) so the cup isn't crushed.
4. **`CUP_HEIGHT`, `LAYER_HEIGHT`**: measure one cup, and the rim height of a cup resting on two others.
5. **`CUP_SPACING`**: rim diameter + about 5 mm. Check that an upper cup's rim still rests on both cups below it.
6. **`PYRAMID_CENTER`**: a free spot about 0.5 m in front of the robot.
7. **`cups.json`**: place each cup, guide the gripper to its centre, copy x, y.

First run: use `--dry-run`, then `--step` with your hand on the e-stop.

## Suggested experiments
- Success rate per pyramid size (3, 6, 10 cups), 5 to 10 trials each.
- Default planner vs `--layer-by-layer`: travel, time, failures.
- Parser accuracy: write about 20 goal sentences and count how many parse to the intended pyramid.
- Failure modes to note: missed grasp, cup crushed, upper cup slides off, structure knocked over.

## Limits and next steps
- Cup positions are measured by hand (no camera). Replacing `cups.json` with a perception step is the natural next step; the planner doesn't change.
- Open loop: the robot doesn't check whether a placement succeeded.
- Only 2D pyramids. A 3D pyramid or nested tower would only need new slot geometry in `build_slots`.
