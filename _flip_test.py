"""One-off: run the rim-up flip pipeline one stage per call.

    python3 _flip_test.py probe     # touch-probe the cup on the table
    python3 _flip_test.py shelf     # inside-grasp the rim-up cup, set it rim-up on the shelf
    python3 _flip_test.py flip      # centre on the shelf, side grasp, roll, set down rim-down
    python3 _flip_test.py back      # normal top pick from the shelf, set it rim-down back on the table
"""
import sys

import config as C
from main import Robot, flipped_shelf_xy

XY = (0.5249, -0.2528)  # the cup's spot on the table

stage = sys.argv[1]
r = Robot(1)
print(f"travel_z {r.travel_z:.3f}, shelf_dz {r.shelf_dz:.3f}", flush=True)
r.home()
if stage == "probe":
    o, sink = r.probe(XY)
    print(f"RESULT: {o} (sank {sink * 1000:.1f} mm)")
elif stage == "shelf":
    r.to_shelf(XY)
    print("RESULT: to_shelf done; width after release", round(r.fa.get_gripper_width(), 4))
elif stage == "flip":
    print("RESULT: flip", "OK" if r.flip() else "FAILED (no joint-7 room; cup put back)")
elif stage == "shelfz":
    # Guarded touch on the BARE shelf at the cup spot: closed fingers, slow descent, stop on contact.
    import time
    import numpy as np
    r.fa.goto_gripper(0.0)
    r.goto(C.SHELF_XY, r.shelf_travel_z, C.TRAVEL_DURATION)
    r.goto(C.SHELF_XY, C.SHELF_Z + 0.04, C.VERTICAL_DURATION)
    time.sleep(0.5)
    f0 = np.mean([r.fa.get_ee_force_torque()[:3] for _ in range(20)], axis=0)
    r.goto(C.SHELF_XY, C.SHELF_Z - 0.04, 8.0, block=False)
    peak = 0.0
    while not r.fa.is_skill_done():
        peak = max(peak, np.linalg.norm(r.fa.get_ee_force_torque()[:3] - f0))
        if peak > C.PROBE_FORCE:
            r.fa.stop_skill()
            break
    z = r.fa.get_pose().translation[2]
    print(f"RESULT: shelf contact z {z:.4f} (peak {peak:.1f} N{', CONTACT' if peak > C.PROBE_FORCE else ', NO CONTACT'}); "
          f"config SHELF_Z {C.SHELF_Z:.4f}")
    r.goto(C.SHELF_XY, r.shelf_travel_z, C.VERTICAL_DURATION)
elif stage == "back":
    r.pick(flipped_shelf_xy(), r.shelf_dz)
    print("gripper width holding cup:", round(r.fa.get_gripper_width(), 4))
    r.place(XY, 0)
    print("RESULT: cup back on the table")
r.home()
