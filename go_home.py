"""Send the arm home and report whether it actually got there.

    python3 go_home.py

frankapy treats an ABORTED motion as done, so this checks the joints afterwards instead of
trusting the return. If it did not move: check Desk for an error, make sure the user-stop is
released (blue lights), and move the arm off any joint limit by hand first.
"""
import numpy as np
from frankapy import FrankaArm
from frankapy import FrankaConstants as FC

# Franka joint limits (rad)
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

fa = FrankaArm()
q0 = fa.get_joints()
print("start joints:", np.round(q0, 3))
near = [i + 1 for i in range(7) if min(q0[i] - Q_MIN[i], Q_MAX[i] - q0[i]) < 0.05]
if near:
    print(f"WARNING: joint(s) {near} within 0.05 rad of their limit; the controller may refuse to move.")

fa.open_gripper()
fa.reset_joints(duration=6)

q1 = fa.get_joints()
err = np.abs(q1 - np.array(FC.HOME_JOINTS)).max()
print("end joints:  ", np.round(q1, 3))
if err < 0.02:
    print("OK: arm is home.")
else:
    print(f"FAILED: still {err:.3f} rad from home (moved {np.abs(q1 - q0).max():.3f} rad). "
          "Motion was probably aborted: check Desk, release the user-stop, move the arm off "
          "the joint limit by hand, or restart start_control_pc.sh.")
