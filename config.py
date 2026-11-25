"""All numbers you must measure on the real setup live here (metres, robot base frame).

Use `python calibrate.py` to read poses by hand-guiding the arm.
Every value marked PLACEHOLDER must be re-measured before running on the robot.
"""
import numpy as np

# ---------------- Cup geometry (cups stand UPSIDE-DOWN, rim on the table) ----------------
CUP_HEIGHT = 0.085     # measured full cup height
LAYER_HEIGHT = 0.09   # PLACEHOLDER rim of layer k sits k * LAYER_HEIGHT above the table
                       #   (≈ CUP_HEIGHT; measure a 2-layer stack, it can be a few mm lower)
CUP_SPACING = 0.085    # PLACEHOLDER centre-to-centre distance of neighbouring base cups
                       #   (≈ rim diameter + ~5 mm gap). Upper cups sit centred between two.

# ---------------- Gripper ----------------
# z of the franka_tool frame when correctly grasping a cup that stands on the TABLE.
# (Fingertips around the upturned base, ~1.5 cm below its top.)
GRASP_Z = 0.0997         # PLACEHOLDER
OPEN_WIDTH = 0.08      # gripper opening before grasp (Franka max = 0.08)
GRASP_WIDTH = 0.0696    # PLACEHOLDER target width when closing on a cup (slightly < cup width there)
GRASP_FORCE = 5.0      # N. Keep low: plastic cups crush easily.

# Top-down gripper rotation with fingers opening along WORLD X (perpendicular to the pyramid row,
# so fingers never hit neighbouring cups). PLACEHOLDER: replace with fa.get_pose().rotation.
GRIPPER_ROTATION = np.array([[0, -1, 0],
                             [-1, 0, 0],
                             [0, 0, -1]], dtype=float)

# ---------------- Orientation check ----------------
# The gripper is lowered softly over the cup centre and we look at how far it got.
#   "touch": fingers CLOSED. Base-up cup stops them at the top; rim-up cup lets them sink inside.
#   "gauge": fingers at GAUGE_WIDTH. Base-up cup's narrow base passes between them; rim-up cup's
#            wide rim stops them. (Diameter-based fallback. Needs cups.json xy within a few mm.)
PROBE_MODE = "touch"
PROBE_TOUCH_Z = 0.131    # measured: tool z when the fingertips just touch the top of an upside-down cup
PROBE_DEPTH = 0.03       # aim this far below PROBE_TOUCH_Z (the arm stops ~1 cm short of targets)
GAUGE_WIDTH = 0.07       # PLACEHOLDER "gauge" only: midway between base and rim diameter, AND wider than
                         #   the base-up cup PROBE_DEPTH below its top (it widens a few mm there)
PROBE_FORCE = 4.0        # N: stop the probe descent when the sensed contact force rises this much
PROBE_DURATION = 5.0     # s for the PRE_PLACE + PROBE_DEPTH descent (~1 cm/s, so it stops gently)

# ---------------- Flip (rim-up cup -> shelf -> side grasp -> roll -> shelf, rim down) ----------------
# A rim-up cup is picked from the top by pinching its rim wall, set rim-up on the shelf, centred, gripped
# from the side, rolled 180 deg, set back on the shelf rim-down, then picked normally from the
# shelf and placed on the pyramid. The shelf lifts the cup so the flat (sideways) hand clears the table.
TABLE_Z = 0.046          # derived: PROBE_TOUCH_Z - CUP_HEIGHT (tool z when the fingertips touch the table)
SHELF_XY = [0.3333, -0.3396]  # measured side-grasp cup centre, moved 3 cm in -y (away from the shelf edge)
SHELF_Z = 0.5989         # robot-measured (guarded touch, closed fingers pointing down) at SHELF_XY
# Centring: closed fingers go inside the rim-up cup, then OPEN against the inner wall
# (Franka grips from the inside by spreading). Centring is done twice on the shelf (fingers along
# x, then y) so the cup slides to the centre and its xy is known to ~1 mm before the side grasp.
CENTER_Z = 0.096         # derived: PROBE_TOUCH_Z - 0.035 (fingertips ~3.5 cm inside a rim-up cup ON THE
                         #   TABLE, deep enough that the cup is narrower than the 80 mm max opening)
CENTER_FORCE = 5.0       # N, outward (centring)
# Rim pinch: a rim-up cup is picked from the top by squeezing the rim wall between the fingers
# (one inside the cup, one outside). Tolerant: the wall only has to land between the open fingers.
RIM_RADIUS = 0.04        # PLACEHOLDER radius of the rim wall from the cup axis (~ rim diameter / 2)
PINCH_DEPTH = 0.02       # fingertips this far below the rim
PINCH_FORCE = 15.0       # N squeezing the rim wall (the rolled rim is the stiffest part of the cup)
# Side grasp: gripper horizontal, pointing along -world y (away from the robot base, toward the
# shelf side), fingers opening along world x. Grasp low on the cup (narrowest part of a rim-up cup).
SIDE_ROTATION = np.array([[0, -1, 0],
                          [0, 0, -1],
                          [1, 0, 0]], dtype=float)  # measured (hand-guided, squared to exactly horizontal)
SIDE_GRASP_Z = 0.583     # measured 0.5908, lowered ~8 mm: tool z for the side grasp of a rim-up cup ON
                         #   THE SHELF (as low as possible: the cup is narrowest at the bottom)
# Joints of the hand-guided pose: flat hand ~SIDE_APPROACH back from the shelf cup, at grasp height.
# The flip moves here in JOINT space (so the arm keeps the guided shape), then slides in.
SIDE_APPROACH_JOINTS = [-0.016, -0.782, 0.043, -2.597, -1.575, 1.606, 2.649]  # measured (guided; tool at 0.324, -0.195, 0.586)
SIDE_JOINTS_DURATION = 8.0   # s for the joint move from home to SIDE_APPROACH_JOINTS
CENTER_ON_SHELF = True       # centre the cup along x on the shelf before the side grasp: the gripper only
                             #   opens ~75 mm, so a ~60 mm cup leaves ~7 mm spare per side
SIDE_APPROACH = 0.10     # start the side approach this far back from the cup centre, along the
                         #   gripper's pointing direction (3rd column of SIDE_ROTATION, projected on xy)
SIDE_SLIDE_SHORT = 0.01  # the slide-in stops this far before the cup centre (going all the way pushes the cup)
SIDE_GRASP_H = 0.0125    # height of the side-grasp point above the shelf surface (from the flip runs: 0.01
                         #   released the flipped cup ~0.5 cm too high; each +1 mm here lowers the release 2 mm)
SETDOWN_SHIFT = 0.04     # move the hand this far back TOWARD THE ARM before setting the flipped cup down
                         #   (keeps it off the shelf wall)
FLIP_LANDING_OFFSET = 0.045  # extra: the flipped cup lands this much further from the arm (-y) than the
                             #   measured hand positions predict. 0.01 left the pick ~one cup radius
                             #   (~3.5 cm) short, toward the arm (run 2026-11-25 16:27) -> 0.045.
SHELF_CLEAR = 0.15       # before homing from the shelf, move this far toward the arm at shelf height
SETDOWN_FORCE = 8.0      # N: safety stop while setting the flipped cup down (motion noise is ~3-4 N)
SIDE_GRASP_WIDTH = 0.057 # measured cup width 0.0597 at the side grasp, minus ~3 mm
FLIP_LIFT = 0.10         # lift this far above SIDE_GRASP_Z before the 180° wrist roll (>= CUP_HEIGHT,
                         #   so the rolling cup clears the shelf)

# ---------------- Pyramid placement ----------------
PYRAMID_CENTER = [0.5178, 0.0134]  # PLACEHOLDER [x, y] of the middle of the base row; row runs along world y
PLACE_DROP = 0.005            # release the cup this far above its support (don't push down)
CLEARANCE = 0.05              # extra height above the tallest thing when travelling
PRE_PLACE = 0.03              # slow down for the last few cm of a place
PLACE_SEARCH = 0.015          # a place may go this far below its nominal height, stopping on contact

# ---------------- Planner / safety ----------------
HOME_XY = [0.31, 0.0]         # gripper xy after fa.reset_joints() (start point for travel cost)
WORKSPACE = {"x": (0.25, 0.75), "y": (-0.40, 0.40)}  # every pick/place xy must be inside this box

# Stiffer Cartesian impedance [x, y, z, rx, ry, rz] for the shelf / flip moves. frankapy's default
# (600 N/m, 50 Nm/rad) stops ~1 cm short of targets; this gets much closer.
STIFF_IMPEDANCES = [1500, 1500, 1500, 120, 120, 120]
# Speed limits: every move's duration is stretched so its peak speed stays below these.
MAX_SPEED = 0.15       # m/s
MAX_ROT_SPEED = 0.8    # rad/s

# Motion durations (s)
TRAVEL_DURATION = 4.0
VERTICAL_DURATION = 2.5
SLOW_DURATION = 2.0
