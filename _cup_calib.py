"""One-off: record one cup's centre by hand-guiding the closed fingertips over it.

    python3 _cup_calib.py <cup_id> [home]   # 'home' = reset_joints first; then 40 s guide mode
Prints a line 'CUP <id> <x> <y> <z>'.
"""
import sys
import time

import numpy as np
from frankapy import FrankaArm
from frankapy import FrankaConstants as FC

cup_id = sys.argv[1]
fa = FrankaArm()
if len(sys.argv) > 2 and sys.argv[2] == "home":
    fa.reset_joints()
    if np.abs(fa.get_joints() - np.array(FC.HOME_JOINTS)).max() > 0.02:
        sys.exit("ABORT: arm did not reach home (motion aborted?).")
fa.goto_gripper(0.0)
time.sleep(2)
print(f"GUIDE MODE ON for 40 s ({cup_id})", flush=True)
fa.run_guide_mode(40, block=True)
p = fa.get_pose().translation
print(f"CUP {cup_id} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
print("joints:", [round(v, 3) for v in fa.get_joints()])
