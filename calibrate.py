"""Hand-guide the arm and print its pose. Use it to fill in config.py and cups.json.

    python calibrate.py

Commands at the prompt:
    Enter  guide mode for 10 s, then print the pose
    c      close the gripper gently (on a cup it stops at the cup; 0 width if nothing is there)
    o      open the gripper fully
    p      print the pose now, without guide mode
    q      quit
"""
from frankapy import FrankaArm


def show(fa):
    p = fa.get_pose()
    print("translation [x, y, z]:", [round(v, 4) for v in p.translation])
    print("rotation:\n", p.rotation.round(3))
    print("gripper width:", round(fa.get_gripper_width(), 4))
    print("joints:", [round(v, 3) for v in fa.get_joints()],
          " (side grasp: joint 7 must be near +-1.57 so the 180 deg roll fits in +-2.9)")


if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()
    fa.open_gripper()
    while True:
        cmd = input("\nEnter = guide 10 s | c = close | o = open | p = print | q = quit: ").strip().lower()
        if cmd == "q":
            break
        if cmd == "c":
            fa.goto_gripper(0.0, grasp=True, force=5.0)
        elif cmd == "o":
            fa.open_gripper()
        elif cmd == "":
            fa.run_guide_mode(10, block=True)
        show(fa)
