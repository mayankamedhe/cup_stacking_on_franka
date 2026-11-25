"""Plan and (optionally) execute a cup pyramid on the Franka.

    python main.py --goal "base layer of yellow, 2nd layer of orange, top red" --dry-run
    python main.py --goal "..." --step          # press Enter before every pick/place
    python main.py --probe-only                 # just touch-probe every cup and report orientation

Before building, every cup the plan uses is touch-probed. Rim-up cups are flipped in place
(centre -> side grasp -> 180 deg wrist roll -> set down), then the pyramid is built.
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
from orientation import ORIENTATIONS, classify_probe, flipped_grasp_z, roll_target_q7
from planner import Cup, InfeasibleError, build_slots, plan, render


def load_cups(path):
    with open(path) as f:
        data = json.load(f)
    cups = []
    for c in data["cups"]:
        o = c.get("orientation", "down").lower()
        if o not in ORIENTATIONS:
            raise SystemExit(f"{path}: cup {c['id']} orientation must be one of {ORIENTATIONS}, got {o!r}")
        cups.append(Cup(c["id"], c["color"].lower(), tuple(c["xy"]), o))
    return cups


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


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

    def goto(self, xy, z, duration, rotation=None, **kw):
        rotation = C.GRIPPER_ROTATION if rotation is None else rotation
        pose = self.RT(rotation=rotation, translation=np.array([xy[0], xy[1], z]),
                       from_frame="franka_tool", to_frame="world")
        self.fa.goto_pose(pose, duration=duration, **kw)

    def home(self):
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.fa.reset_joints()

    def pick(self, xy):
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(xy, self.travel_z, C.TRAVEL_DURATION)
        self.goto(xy, C.GRASP_Z, C.VERTICAL_DURATION)
        self.fa.goto_gripper(C.GRASP_WIDTH, grasp=True, force=C.GRASP_FORCE)
        self.goto(xy, self.travel_z, C.VERTICAL_DURATION)

    # ---------------- orientation ----------------
    def probe(self, xy):
        """Orientation probe (C.PROBE_MODE, see orientation.py). Returns (orientation, sink):
        sink = how far (m) the fingers got below the cup top (~0 or ~PROBE_DEPTH)."""
        self.fa.goto_gripper(0.0 if C.PROBE_MODE == "touch" else C.GAUGE_WIDTH)
        self.goto(xy, self.travel_z, C.TRAVEL_DURATION)
        self.goto(xy, C.PROBE_TOUCH_Z + C.PRE_PLACE, C.VERTICAL_DURATION)
        # Soft in z only, so pressing on a solid base stops the hand with a few newtons.
        self.goto(xy, C.PROBE_TOUCH_Z - C.PROBE_DEPTH, C.PROBE_DURATION,
                  cartesian_impedances=[600, 600, C.PROBE_Z_STIFFNESS, 50, 50, 50])
        z = self.fa.get_pose().translation[2]
        self.goto(xy, self.travel_z, C.VERTICAL_DURATION)
        return classify_probe(z, C.PROBE_TOUCH_Z, C.PROBE_DEPTH, C.PROBE_MODE), C.PROBE_TOUCH_Z - z

    def center(self, xy):
        """Rim-up cup: open the fingers against the INSIDE wall (along x, then along y). The cup
        slides until centred between the fingers, so its xy is known to ~1 mm for the side grasp."""
        above = C.PROBE_TOUCH_Z + C.CLEARANCE
        for rot in (C.GRIPPER_ROTATION, C.GRIPPER_ROTATION @ rot_z(np.pi / 2)):
            self.fa.goto_gripper(0.0)
            self.goto(xy, above, C.TRAVEL_DURATION, rot)
            self.goto(xy, C.CENTER_Z, C.VERTICAL_DURATION, rot)
            self.fa.goto_gripper(C.OPEN_WIDTH, grasp=True, force=C.CENTER_FORCE)  # spread outward
            self.fa.goto_gripper(0.0)
            self.goto(xy, above, C.VERTICAL_DURATION, rot)

    def flip(self, xy):
        """Rim-up cup -> upside-down, in place. Returns False (cup untouched or put back as it
        was) if joint 7 has no room for a 180 deg roll; then the cup must be flipped by hand."""
        x, y = xy
        back = (x - C.SIDE_APPROACH, y)
        self.center(xy)
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(back, self.travel_z, C.TRAVEL_DURATION, C.SIDE_ROTATION)
        if roll_target_q7(self.fa.get_joints()[6]) is None:
            self.home()
            return False
        self.goto(back, C.SIDE_GRASP_Z, C.VERTICAL_DURATION, C.SIDE_ROTATION)
        self.goto(xy, C.SIDE_GRASP_Z, C.SLOW_DURATION, C.SIDE_ROTATION)
        self.fa.goto_gripper(C.SIDE_GRASP_WIDTH, grasp=True, force=C.GRASP_FORCE)
        self.goto(xy, C.FLIP_Z, C.VERTICAL_DURATION, C.SIDE_ROTATION)

        q = self.fa.get_joints()
        q7 = roll_target_q7(q[6])  # re-check: the lift changed the joints a little
        if q7 is None:
            self.goto(xy, C.SIDE_GRASP_Z + C.PLACE_DROP, C.VERTICAL_DURATION, C.SIDE_ROTATION)
            self.fa.goto_gripper(C.OPEN_WIDTH)
            self.goto(back, C.SIDE_GRASP_Z, C.SLOW_DURATION, C.SIDE_ROTATION)
            self.goto(back, self.travel_z, C.VERTICAL_DURATION, C.SIDE_ROTATION)
            self.home()
            return False
        q[6] = q7
        self.fa.goto_joints(q, duration=C.TRAVEL_DURATION)  # the flip: roll about the horizontal tool axis

        rolled = self.fa.get_pose().rotation
        z = flipped_grasp_z(C.SIDE_GRASP_Z, C.TABLE_Z, C.CUP_HEIGHT) + C.PLACE_DROP
        self.goto(xy, z + C.PRE_PLACE, C.VERTICAL_DURATION, rolled)
        self.goto(xy, z, C.SLOW_DURATION, rolled)
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(back, z, C.SLOW_DURATION, rolled)
        self.goto(back, self.travel_z, C.VERTICAL_DURATION, rolled)
        self.home()
        return True

    def place(self, xy, layer):
        z = C.GRASP_Z + layer * C.LAYER_HEIGHT + C.PLACE_DROP
        self.goto(xy, self.travel_z, C.TRAVEL_DURATION)
        self.goto(xy, z + C.PRE_PLACE, C.VERTICAL_DURATION)
        self.goto(xy, z, C.SLOW_DURATION)
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(xy, self.travel_z, C.VERTICAL_DURATION)


def log_trial(path, row):
    """Append a row. If the file has older/different columns, rewrite it with the union."""
    rows, fields = [], []
    if os.path.exists(path):
        with open(path, newline="") as f:
            r = csv.DictReader(f)
            fields, rows = list(r.fieldnames or []), list(r)
    if not fields or any(k not in fields for k in row):
        fields += [k for k in row if k not in fields]
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, restval="")
            w.writeheader()
            w.writerows(rows + [row])
    else:
        with open(path, "a", newline="") as f:
            csv.DictWriter(f, fieldnames=fields, restval="").writerow(row)


def probe_only(cups):
    """Sensing experiment: probe every cup, compare with cups.json "orientation"."""
    robot = Robot(1)
    robot.home()
    wrong = 0
    try:
        for cup in cups:
            o, sink = robot.probe(cup.xy)
            ok = o == cup.orientation
            wrong += not ok
            print(f"  {cup.id:>4}: {o:<4} (sank {sink * 1000:5.1f} mm)  cups.json: {cup.orientation}"
                  + ("" if ok else "   <-- MISMATCH"))
    finally:
        robot.home()
    print(f"{len(cups) - wrong}/{len(cups)} match cups.json")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--goal", help='e.g. "base layer of yellow, 2nd layer of orange, top red"')
    ap.add_argument("--cups", default="cups.json", help="start positions, colours (and expected orientation) of cups")
    ap.add_argument("--dry-run", action="store_true", help="plan only, don't move the robot")
    ap.add_argument("--layer-by-layer", action="store_true", help="finish each layer before the next")
    ap.add_argument("--step", action="store_true", help="wait for Enter before every probe/flip/pick/place")
    ap.add_argument("--skip-orientation-check", action="store_true",
                    help='don\'t probe; trust "orientation" in cups.json (missing = "down")')
    ap.add_argument("--probe-only", action="store_true",
                    help="probe every cup in cups.json, compare with its \"orientation\", then stop")
    ap.add_argument("--log", default="experiments.csv", help="CSV file to append the trial result to")
    args = ap.parse_args()

    cups = load_cups(args.cups)
    if args.probe_only:
        check_workspace([(c.id, c.xy) for c in cups])
        probe_only(cups)
        return
    if not args.goal:
        ap.error("--goal is required (unless --probe-only)")

    try:
        layers = parse_goal(args.goal)
    except GoalParseError as e:
        raise SystemExit(f"Could not understand goal: {e}")
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
    used = [cup for cup, _ in steps]
    expected_up = [c.id for c in used if c.orientation == "up"]
    print("Orientation: " + ("not checked, trusting cups.json" if args.skip_orientation_check
                             else f"every used cup is probed first ({C.PROBE_MODE} mode)")
          + f"; cups.json marks {len(expected_up)} rim-up" + (f" ({', '.join(expected_up)})" if expected_up else ""))
    check_workspace([(c.id, c.xy) for c in cups]
                    + [(f"{c.id} side approach", (c.xy[0] - C.SIDE_APPROACH, c.xy[1])) for c in used]
                    + [(f"slot L{s.layer + 1}.{s.index}", s.xy) for s in slots])
    if args.dry_run:
        return

    if input("\nRun on robot? [y/N] ").strip().lower() != "y":
        return
    robot = Robot(len(layers))
    robot.home()
    t0 = time.perf_counter()
    done = 0
    found, sinks = {}, {}
    flipped_robot, flipped_hand = [], []

    def pause(msg):
        if args.step:
            input(msg)

    try:
        # 1) Scan: orientation of every cup the plan uses.
        for i, cup in enumerate(used, 1):
            if args.skip_orientation_check:
                found[cup.id] = cup.orientation
                continue
            pause(f"[probe {i}/{len(used)}] Enter to probe {cup.id} ...")
            found[cup.id], sinks[cup.id] = robot.probe(cup.xy)
            print(f"  {cup.id}: {found[cup.id]} (sank {sinks[cup.id] * 1000:.1f} mm)"
                  + ("" if found[cup.id] == cup.orientation else f"  [cups.json says {cup.orientation}]"))
        robot.home()

        # 2) Flip every rim-up cup in place.
        to_flip = [c for c in used if found[c.id] == "up"]
        for i, cup in enumerate(to_flip, 1):
            pause(f"[flip {i}/{len(to_flip)}] Enter to flip {cup.id} ...")
            if robot.flip(cup.xy):
                flipped_robot.append(cup.id)
            else:
                input(f"  Wrist can't roll 180 deg for {cup.id}. Flip it BY HAND (rim down, same spot), then Enter ...")
                flipped_hand.append(cup.id)

        # 3) Build.
        for i, (cup, slot) in enumerate(steps, 1):
            pause(f"[{i}/{len(steps)}] Enter to pick {cup.id} ...")
            robot.pick(cup.xy)
            pause(f"[{i}/{len(steps)}] Enter to place on layer {slot.layer + 1} pos {slot.index} ...")
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
            "orientation_check": "skipped" if args.skip_orientation_check else C.PROBE_MODE,
            "n_rim_up": sum(o == "up" for o in found.values()),
            "n_flipped_robot": len(flipped_robot), "n_flipped_hand": len(flipped_hand),
            # probe vs cups.json "orientation" (fill cups.json with the truth to measure probe accuracy)
            "orientation_mismatches": sum(found[c.id] != c.orientation for c in used if c.id in sinks),
            "probe_sink_mm": ";".join(f"{k}:{v * 1000:.1f}" for k, v in sinks.items()),
        })
        print(f"Logged to {args.log}")


if __name__ == "__main__":
    main()
