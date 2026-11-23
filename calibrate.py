"""Hand-guide the arm and print its pose. Use it to fill in config.py and cups.json.

    python calibrate.py            # 10 s of guide mode per reading
"""
from frankapy import FrankaArm

if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()
    fa.open_gripper()
    while input("\nEnter = guide mode for 10 s, q = quit: ").strip().lower() != "q":
        fa.run_guide_mode(10, block=True)
        p = fa.get_pose()
        print("translation [x, y, z]:", [round(v, 4) for v in p.translation])
        print("rotation:\n", p.rotation.round(3))
        print("gripper width:", round(fa.get_gripper_width(), 4))
