"""Plan and (optionally) execute a cup pyramid on the Franka.

    python main.py --goal "base layer of yellow, 2nd layer of orange, top red" --dry-run
    python main.py --goal "..." --step          # press Enter before every pick/place
"""
import argparse
import csv
import datetime
import json
import os
import time

import numpy as np

import config as C
from goal_parser import GoalParseError, parse_goal
from planner import Cup, InfeasibleError, build_slots, plan, render


def load_cups(path):
    with open(path) as f:
        data = json.load(f)
    return [Cup(c["id"], c["color"].lower(), tuple(c["xy"])) for c in data["cups"]]


def check_workspace(points):
    (x0, x1), (y0, y1) = C.WORKSPACE["x"], C.WORKSPACE["y"]
    bad = [(name, xy) for name, xy in points if not (x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1)]
    if bad:
        raise SystemExit("Outside WORKSPACE (edit cups.json / config.py): " +
                         ", ".join(f"{n} {tuple(round(v, 3) for v in xy)}" for n, xy in bad))


class Robot:
    """Thin wrapper over frankapy: top-down pick and place of one cup."""

    def __init__(self, n_layers):
        from autolab_core import RigidTransform
        from frankapy import FrankaArm
        self.RT = RigidTransform
        self.fa = FrankaArm()
        # Carried cup's rim must clear the tallest possible structure.
        self.travel_z = C.GRASP_Z + (n_layers - 1) * C.LAYER_HEIGHT + C.CUP_HEIGHT + C.CLEARANCE

    def goto(self, xy, z, duration):
        pose = self.RT(rotation=C.GRIPPER_ROTATION, translation=np.array([xy[0], xy[1], z]),
                       from_frame="franka_tool", to_frame="world")
        self.fa.goto_pose(pose, duration=duration)

    def home(self):
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.fa.reset_joints()

    def pick(self, xy):
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(xy, self.travel_z, C.TRAVEL_DURATION)
        self.goto(xy, C.GRASP_Z, C.VERTICAL_DURATION)
        self.fa.goto_gripper(C.GRASP_WIDTH, grasp=True, force=C.GRASP_FORCE)
        self.goto(xy, self.travel_z, C.VERTICAL_DURATION)

    def place(self, xy, layer):
        z = C.GRASP_Z + layer * C.LAYER_HEIGHT + C.PLACE_DROP
        self.goto(xy, self.travel_z, C.TRAVEL_DURATION)
        self.goto(xy, z + C.PRE_PLACE, C.VERTICAL_DURATION)
        self.goto(xy, z, C.SLOW_DURATION)
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(xy, self.travel_z, C.VERTICAL_DURATION)


def log_trial(path, row):
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--goal", required=True, help='e.g. "base layer of yellow, 2nd layer of orange, top red"')
    ap.add_argument("--cups", default="cups.json", help="start positions and colours of cups")
    ap.add_argument("--dry-run", action="store_true", help="plan only, don't move the robot")
    ap.add_argument("--layer-by-layer", action="store_true", help="finish each layer before the next")
    ap.add_argument("--step", action="store_true", help="wait for Enter before every pick/place")
    ap.add_argument("--log", default="experiments.csv", help="CSV file to append the trial result to")
    args = ap.parse_args()

    try:
        layers = parse_goal(args.goal)
    except GoalParseError as e:
        raise SystemExit(f"Could not understand goal: {e}")
    cups = load_cups(args.cups)
    slots = build_slots(layers, C.PYRAMID_CENTER, C.CUP_SPACING)

    print("Goal pyramid:\n" + render(layers) + "\n")
    t0 = time.perf_counter()
    try:
        steps, cost = plan(cups, slots, C.HOME_XY, args.layer_by_layer)
    except InfeasibleError as e:
        raise SystemExit(f"Infeasible: {e}")
    plan_ms = (time.perf_counter() - t0) * 1000
    print(f"Plan: {len(steps)} moves, {cost:.2f} m xy travel, found in {plan_ms:.0f} ms")
    for i, (cup, slot) in enumerate(steps, 1):
        print(f"  {i:2d}. {cup.id:>4} ({cup.color}) {tuple(round(v, 3) for v in cup.xy)}"
              f"  ->  layer {slot.layer + 1} pos {slot.index} {tuple(round(v, 3) for v in slot.xy)}")
    check_workspace([(c.id, c.xy) for c in cups] + [(f"slot L{s.layer + 1}.{s.index}", s.xy) for s in slots])
    if args.dry_run:
        return

    if input("\nRun on robot? [y/N] ").strip().lower() != "y":
        return
    robot = Robot(len(layers))
    robot.home()
    t0 = time.perf_counter()
    done = 0
    try:
        for i, (cup, slot) in enumerate(steps, 1):
            if args.step:
                input(f"[{i}/{len(steps)}] Enter to pick {cup.id} ...")
            robot.pick(cup.xy)
            if args.step:
                input(f"[{i}/{len(steps)}] Enter to place on layer {slot.layer + 1} pos {slot.index} ...")
            robot.place(slot.xy, slot.layer)
            done = i
    finally:
        robot.home()
        exec_s = time.perf_counter() - t0
        ok = input("Did the pyramid end up correct? [y/n] ").strip().lower()
        notes = input("Notes (failure mode, which step, etc.): ").strip()
        log_trial(args.log, {
            "time": datetime.datetime.now().isoformat(timespec="seconds"),
            "goal": args.goal, "layer_by_layer": args.layer_by_layer, "n_cups_available": len(cups), "n_moves": len(steps),
            "moves_completed": done, "planned_travel_m": round(cost, 3),
            "plan_ms": round(plan_ms, 1), "exec_s": round(exec_s, 1),
            "success": ok, "notes": notes,
        })
        print(f"Logged to {args.log}")


if __name__ == "__main__":
    main()
