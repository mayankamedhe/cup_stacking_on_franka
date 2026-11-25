"""One-off timed calibrate.py: go home, set the gripper, guide mode, print pose.

    python3 _side_calib.py shelf   # gripper closed, guide 45 s, print
    python3 _side_calib.py side    # gripper open, guide 90 s, close on cup, print, open
"""
import sys
import time

import numpy as np
from frankapy import FrankaArm
from frankapy import FrankaConstants as FC

mode = sys.argv[1]
fa = FrankaArm()
fa.reset_joints()
if np.abs(fa.get_joints() - np.array(FC.HOME_JOINTS)).max() > 0.02:
    sys.exit("ABORT: arm did not reach home (motion aborted?). Not starting guide mode.")
print("arm is home", flush=True)

if mode == "shelf":
    fa.goto_gripper(0.0)
    secs = 45
else:
    fa.open_gripper()
    secs = 90
time.sleep(3)
print(f"GUIDE MODE ON for {secs} s", flush=True)
fa.run_guide_mode(secs, block=True)
print("guide mode OFF", flush=True)
if mode == "side":
    fa.goto_gripper(0.0, grasp=True, force=5.0)

p = fa.get_pose()
print("translation [x, y, z]:", [round(v, 4) for v in p.translation])
print("rotation:\n", p.rotation.round(3))
print("gripper width:", round(fa.get_gripper_width(), 4))
print("joints:", [round(v, 3) for v in fa.get_joints()])
if mode == "side":
    fa.open_gripper()
    print("gripper opened; arm holds this pose", flush=True)
