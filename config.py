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

# Desktop dashboard limits (franka_dashboard.py). Review for your actual setup.
# These are application limits, not a collision model or hardware safety limits.
GUI_Z_MIN = 0.04           # m, franka_tool origin in robot base frame
GUI_Z_MAX = 0.75           # m
GUI_MAX_TRANSLATION = 0.05 # m per Cartesian command
GUI_MAX_ROTATION_DEG = 15  # degrees per Cartesian command
GUI_MAX_JOINT_STEP_DEG = 10 # degrees per joint per command
GUI_MAX_FORCE = 10.0       # N, norm of commanded Cartesian force
GUI_MAX_TORQUE = 1.0       # Nm, norm of commanded Cartesian moment
GUI_MAX_GRASP_FORCE = 20.0 # N; cup experiments should start with GRASP_FORCE
GUI_CONTACT_FORCE = 20.0  # N per axis, FrankaPy contact termination threshold
GUI_CONTACT_TORQUE = 3.0  # Nm per axis, FrankaPy contact termination threshold
