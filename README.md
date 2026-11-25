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
| `franka_dashboard.py` | Desktop hardware GUI: live force/torque plots, arm/gripper controls, wrench pulses, CSV export. |
| `franka_dashboard_backend.py` | FrankaPy/ROS adapter, command validation, telemetry worker, and stop handling. |
| `requirements-gui.txt` | Additional Python dependencies for the desktop dashboard. |
| `test_franka_dashboard.py` | Hardware-free checks for limits, stale data, stop/dispatch behavior, and API mapping. |
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

## Live hardware dashboard

Run this in the **same ROS1 and FrankaPy environment** used for `main.py`, with
the existing Franka Interface and `franka_ros` gripper servers running. It does
not require, start, or provide a simulator. ROS2 alone cannot run this FrankaPy
adapter. The desktop must have a display and Tk (Ubuntu: `sudo apt install python3-tk`).

```bash
# Source your existing robot ROS1 workspace and activate its Python environment first.
python -m pip install -r requirements-gui.txt
python franka_dashboard.py

# If the robot also publishes a franka_msgs/FrankaState topic:
python franka_dashboard.py --state-topic /franka_state_controller/franka_states

# Different arm/gripper namespaces, or a different sample/history length:
python franka_dashboard.py --robot-num 2 --gripper-namespace /franka_gripper_2 --rate 20 --history 120
```

The dashboard connects without moving the robot. Review `GUI_*` limits in
`config.py`, inspect the live state, then click **Enable controls**. Targets copy
the first measured pose/joint state on startup. Use **Copy measured** again
before editing targets after the robot moves. Run one robot controller at a time;
do not run the stacking script while this dashboard has an active command.

- **Cartesian:** absolute XYZ and roll/pitch/yaw, plus base-frame XYZ jogs.
  Rotation follows `Rz(yaw) Ry(pitch) Rx(roll)`. Commands preserve FrankaPy's
  virtual-wall checks and use contact termination thresholds.
- **Joints:** seven position targets in degrees with bounded changes per command.
  These use FrankaPy's joint/collision checks; the GUI Cartesian box does not
  constrain the path of a joint motion.
- **Gripper:** move to a width, open, or grasp with a commanded force and width
  tolerance. Gripper commands use standard `franka_ros` move/grasp actions so
  their actual success/error results can be checked. Width comes from the
  gripper's `joint_states`, rather than the potentially stale FrankaPy state field.
  The installed gripper/controller may reject force values outside its supported range.
- **Force:** finite-duration, ramped Cartesian force/torque pulses using
  `FrankaArm.apply_effector_forces_torques()`. These are **commanded wrenches,
  not closed-loop contact-force regulation**. The underlying controller applies
  a wrench through joint torques. Axes use the robot base frame. Duration,
  force/torque norms, impulse, translation, and rotation are bounded. Displacement
  caps turn off the commanded wrench; they do not guarantee the arm stops at
  that displacement. This mode does not hold position on other axes.

Four plot selectors provide these channels. All channels are exported together
by **Export history CSV**, including the ones not currently displayed.

| Signal | Source | Units |
|---|---|---|
| Estimated external force XYZ and moment XYZ in base axes O | FrankaPy `ee_force_torque` → `O_F_ext_hat_K` | N, Nm |
| Measured joint torques J1–J7 | FrankaPy `joint_torques` → `tau_J` | Nm |
| Joint torque derivatives J1–J7 | FrankaPy `joint_torques_derivative` → `dtau_J` | Nm/s |
| Estimated external joint torques J1–J7 | Optional ROS `tau_ext_hat_filtered` | Nm |
| Desired joint torques J1–J7 | Optional ROS `tau_J_d` | Nm |
| Estimated external force and moment in stiffness axes K | Optional ROS `K_F_ext_hat_K` | N, Nm |
| Gripper width | ROS gripper joint states | mm |
| Last grasp-force command sent by this dashboard | Command record, **not a measurement** | N |

The external wrench is estimated from the arm's joint torque sensors at the
stiffness-frame origin K; the O/K versions express it in different axes.
The arm's measured joint torques include gravity/dynamics and are not pure
contact torques. **The standard Franka hand has no measured grip-force output**
in its state interface. `goto_gripper(..., grasp=True, force=...)` sets a grasp
command; `get_ee_force_torque()` does not measure finger squeezing force. An
external tactile/load sensor is needed to measure that separately.

FrankaPy's standard state service does not expose every `FrankaState` field.
The optional ROS channels appear only if that topic is already published by
your stack; do not start a second hardware controller just to obtain it. A
single published FrankaState topic is auto-selected for robot 1. Specify
`--state-topic` explicitly for multiple robots. Ensure the topic belongs to the
selected arm. Missing or stale channels stay **NaN / —**, never synthetic zeroes.

Plots refresh at about 5 Hz; telemetry defaults to 20 Hz. CSV exports the rolling
history (60 seconds by default), with Unix and relative timestamps and units in
column names. **Freeze plots** only freezes rendering; acquisition, stop
handling, and the rolling export buffer continue. Plots show raw values without
taring or filtering. The last grasp command is a historical command record,
not a live force setpoint.

**STOP / Escape** cancels the arm skill and gripper actions, requests the gripper
stop action, and disables controls. Closing also sends cancellation and waits
for acknowledgement. A failed stop keeps the window open and blocks enabling
controls until a later STOP succeeds. Missing arm telemetry for one second
automatically requests a stop. These are desktop/ROS safeguards, not a
real-time safety controller: use the physical stop for an emergency. Freshness
is based on RPC/callback arrival and cannot detect stale data inside an upstream
server. Workspace bounds do not model the table, cups, obstacles, or self-collision.

Reference APIs: [FrankaPy state mapping](https://github.com/iamlab-cmu/frankapy/blob/master/frankapy/franka_arm_state_client.py),
[FrankaPy control API](https://iamlab-cmu.github.io/frankapy/frankaarm.html),
[Franka gripper state](https://github.com/frankarobotics/libfranka/blob/main/include/franka/gripper_state.h),
and [ROS FrankaState fields](https://github.com/frankarobotics/franka_ros/blob/develop/franka_msgs/msg/FrankaState.msg).

```bash
python -m unittest test_franka_dashboard -v
```

## Suggested experiments
- Success rate per pyramid size (3, 6, 10 cups), 5 to 10 trials each.
- Default planner vs `--layer-by-layer`: travel, time, failures.
- Parser accuracy: write about 20 goal sentences and count how many parse to the intended pyramid.
- Failure modes to note: missed grasp, cup crushed, upper cup slides off, structure knocked over.

## Limits and next steps
- Cup positions are measured by hand (no camera). Replacing `cups.json` with a perception step is the natural next step; the planner doesn't change.
- Open loop: the robot doesn't check whether a placement succeeded.
- Only 2D pyramids. A 3D pyramid or nested tower would only need new slot geometry in `build_slots`.
