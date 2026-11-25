"""All numbers you must measure on the real setup live here (metres, robot base frame).

Use `python calibrate.py` to read poses by hand-guiding the arm.
Every value marked PLACEHOLDER must be re-measured before running on the robot.
"""
import numpy as np

# ---------------- Cup geometry (cups stand UPSIDE-DOWN, rim on the table) ----------------
CUP_HEIGHT = 0.11      # PLACEHOLDER full cup height
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
PROBE_TOUCH_Z = 0.112    # PLACEHOLDER tool z when the fingertips just touch the top of an upside-down cup
PROBE_DEPTH = 0.02       # aim this far below PROBE_TOUCH_Z; sinking > PROBE_DEPTH/2 decides
GAUGE_WIDTH = 0.07       # PLACEHOLDER "gauge" only: midway between base and rim diameter, AND wider than
                         #   the base-up cup PROBE_DEPTH below its top (it widens a few mm there)
PROBE_Z_STIFFNESS = 150  # N/m in z during the probe -> pushes a base-up cup with only ~3 N
PROBE_DURATION = 3.0

# ---------------- Flip (rim-up cup -> upside-down, in place) ----------------
TABLE_Z = 0.005          # PLACEHOLDER tool z when the fingertips touch the table
# Centring: closed fingers go inside the rim-up cup, then OPEN against the inner wall
# (Franka grips from the inside by spreading). Done twice (fingers along x, then y) so the cup
# slides to the centre and its xy is known to ~1 mm before the tight side grasp.
CENTER_Z = 0.09          # PLACEHOLDER tool z for the fingertips ~2 cm inside a rim-up cup
CENTER_FORCE = 5.0       # N, outward
# Side grasp: gripper horizontal, pointing along +world x (away from the robot), fingers
# opening along world y. Grasp low on the cup (narrowest part of a rim-up cup, ~71-74 mm).
SIDE_ROTATION = np.array([[0, 0, 1],
                          [0, -1, 0],
                          [1, 0, 0]], dtype=float)  # PLACEHOLDER: replace with fa.get_pose().rotation
SIDE_GRASP_Z = 0.045     # PLACEHOLDER tool z for the side grasp. The hand must clear the table here.
SIDE_APPROACH = 0.10     # start the side approach this far back (−x) from the cup centre
SIDE_GRASP_WIDTH = 0.068 # PLACEHOLDER: slightly less than the cup width at SIDE_GRASP_Z
FLIP_Z = 0.25            # tool z for the 180° wrist roll (the hand sweeps ~10 cm around the grasp point)

# ---------------- Pyramid placement ----------------
PYRAMID_CENTER = [0.5178, 0.0134]  # PLACEHOLDER [x, y] of the middle of the base row; row runs along world y
PLACE_DROP = 0.005            # release the cup this far above its support (don't push down)
CLEARANCE = 0.05              # extra height above the tallest thing when travelling
PRE_PLACE = 0.03              # slow down for the last few cm of a place

# ---------------- Planner / safety ----------------
HOME_XY = [0.31, 0.0]         # gripper xy after fa.reset_joints() (start point for travel cost)
WORKSPACE = {"x": (0.25, 0.75), "y": (-0.40, 0.40)}  # every pick/place xy must be inside this box

# Motion durations (s)
TRAVEL_DURATION = 4.0
VERTICAL_DURATION = 2.5
SLOW_DURATION = 2.0
