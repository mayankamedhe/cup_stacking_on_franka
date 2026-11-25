"""Plan and (optionally) execute a cup pyramid on the Franka.

    python main.py --goal "base layer of yellow, 2nd layer of orange, top red" --dry-run
    python main.py --goal "..." --step          # press Enter before every pick/place
    python main.py --probe-only                 # just touch-probe every cup and report orientation

Before building, every cup the plan uses is touch-probed. During the build, each rim-up cup is
picked by pinching its rim and set on the shelf, flipped there (centre -> side grasp -> 180 deg
wrist roll -> set down rim-down), then picked normally from the shelf and placed.
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
from orientation import ORIENTATIONS, classify_probe, roll_target_q7
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


def side_dir():
    """Unit xy direction the (horizontal) gripper points in SIDE_ROTATION."""
    a = np.asarray(C.SIDE_ROTATION, dtype=float)[:2, 2]
    return a / np.linalg.norm(a)


def side_approach_xy(xy, back=None):
    """Point `back` (default SIDE_APPROACH) behind xy, against the gripper's pointing direction."""
    a = side_dir()
    d = C.SIDE_APPROACH if back is None else back
    return (xy[0] - d * a[0], xy[1] - d * a[1])


def flipped_shelf_xy():
    """Where the flipped (rim-down) cup actually stands on the shelf: the hand shift toward the
    arm minus the observed landing offset (the next top-down pick goes here)."""
    return side_approach_xy(C.SHELF_XY, C.SETDOWN_SHIFT - C.FLIP_LANDING_OFFSET)


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
        self.shelf_dz = C.SHELF_Z - C.TABLE_Z  # shelf surface height above the table
        self.flipped_xy = None  # where the last flipped cup landed on the shelf (set by flip())
        # Carried cup's rim must clear the tallest possible structure.
        self.travel_z = C.GRASP_Z + (n_layers - 1) * C.LAYER_HEIGHT + C.CUP_HEIGHT + C.CLEARANCE
        # Shelf moves only: a carried cup (hanging at most CUP_HEIGHT below the fingertips) must
        # clear the shelf.
        self.shelf_travel_z = C.SHELF_Z + C.CUP_HEIGHT + C.CLEARANCE
        self.rot = C.GRIPPER_ROTATION  # current top-down rotation (see topdown())
        self.rot_valid = False         # True while the hand is (headed) in self.rot

    def start_topdown(self, *xys, force_home=False):
        """Choose the top-down rotation for the coming moves. Goes home first only if the hand
        isn't already top-down in that same rotation: switching between the two rotations
        directly is a 180 deg yaw that the wrist could take either way round, but from home
        both are a quarter turn away (predictable). Staying in one rotation, joint 7 just follows
        the base joint and stays well inside its limits."""
        rot = self.topdown(*xys)
        if force_home or not (self.rot_valid and np.allclose(rot, self.rot)):
            self.home()
        self.rot, self.rot_valid = rot, True

    def topdown(self, xy, *more_xy):
        """Top-down gripper rotation for working at xy, chosen when starting FROM HOME.
        GRIPPER_ROTATION is a quarter turn (yaw) from the home rotation, reached by turning joint 7
        from ~0.79 to ~2.36; on the +y side the arm adds about atan2(y, x) on top, which runs into
        the 2.90 limit. The same gripper turned 180 deg about vertical (fingers still along x) is
        reached the other way, joint 7 ~ -0.79 + atan2(y, x). Pick whichever keeps joint 7
        further from its limits at every given xy (e.g. a pick and the place that follows)."""
        q1s = [np.arctan2(p[1], p[0]) for p in (xy,) + more_xy]
        q7_home = 0.785
        flipped = C.GRIPPER_ROTATION @ rot_z(np.pi)
        worst = lambda off: max(abs(q7_home + off + q1) for q1 in q1s)
        return C.GRIPPER_ROTATION if worst(np.pi / 2) < worst(-np.pi / 2) else flipped

    def goto(self, xy, z, duration, rotation=None, **kw):
        if rotation is not None and not np.allclose(rotation, self.rot):
            self.rot_valid = False  # explicit other rotation (side grasp, centring, ...)
        rotation = self.rot if rotation is None else rotation
        pose = self.RT(rotation=rotation, translation=np.array([xy[0], xy[1], z]),
                       from_frame="franka_tool", to_frame="world")
        # Speed limit: stretch the duration so the min-jerk peak speed (1.875 * dist / T) stays
        # below MAX_SPEED / MAX_ROT_SPEED, however far the move is.
        now = self.fa.get_pose()
        dist = np.linalg.norm(now.translation - pose.translation)
        cos = (np.trace(now.rotation.T @ pose.rotation) - 1) / 2
        ang = np.arccos(np.clip(cos, -1.0, 1.0))
        duration = float(max(duration, 1.875 * dist / C.MAX_SPEED, 1.875 * ang / C.MAX_ROT_SPEED))
        self.fa.goto_pose(pose, duration=duration, **kw)

    def sgoto(self, xy, z, duration, rotation=None, **kw):
        """goto with stiffer impedance: the default one stops ~1 cm short of the target
        (joint friction), too much for the shelf / side-grasp moves."""
        self.goto(xy, z, duration, rotation, cartesian_impedances=C.STIFF_IMPEDANCES, **kw)

    def home(self):
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.fa.reset_joints()
        self.rot_valid = False

    def pick(self, xy, dz=0.0, to_xy=None):
        """Top-down pick of a rim-down cup standing dz above the table (dz = shelf_dz on the shelf)."""
        above = self.shelf_travel_z if dz > 0 else self.travel_z
        # Rotation chosen for the pick AND the place that follows (kept while carrying).
        self.start_topdown(xy, *([to_xy] if to_xy is not None else []))
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.goto(xy, above, C.TRAVEL_DURATION)
        # Stiff descent: the default impedance stops ~1 cm short (more when the arm comes
        # straight from the previous place instead of home), so the fingers closed too high.
        self.sgoto(xy, C.GRASP_Z + dz, C.VERTICAL_DURATION)
        self.fa.goto_gripper(C.GRASP_WIDTH, grasp=True, force=C.GRASP_FORCE)
        print(f"    pick: z {self.fa.get_pose().translation[2]:.4f} (cmd {C.GRASP_Z + dz:.4f}), "
              f"width {self.fa.get_gripper_width() * 1000:.1f} mm", flush=True)
        self.sgoto(xy, above, C.VERTICAL_DURATION)

    def guarded_down(self, xy, z_target, duration, rotation=None, force=None, **kw):
        """Slow descent to z_target that stops as soon as the sensed contact force rises by
        `force` (default PROBE_FORCE). Returns (z reached, peak force N, contact?)."""
        force = C.PROBE_FORCE if force is None else force
        time.sleep(0.5)
        f0 = np.mean([self.fa.get_ee_force_torque()[:3] for _ in range(20)], axis=0)
        self.goto(xy, z_target, duration, rotation, block=False, **kw)
        peak = 0.0
        while not self.fa.is_skill_done():
            peak = max(peak, np.linalg.norm(self.fa.get_ee_force_torque()[:3] - f0))
            if peak > force:
                self.fa.stop_skill()
                break
        return self.fa.get_pose().translation[2], peak, peak > force

    # ---------------- orientation ----------------
    def probe(self, xy):
        """Orientation probe (C.PROBE_MODE, see orientation.py). Returns (orientation, sink):
        sink = how far (m) the fingers got below the cup top (~0 or ~PROBE_DEPTH)."""
        self.start_topdown(xy)
        self.fa.goto_gripper(0.0 if C.PROBE_MODE == "touch" else C.GAUGE_WIDTH)
        self.goto(xy, self.travel_z, C.TRAVEL_DURATION)
        self.goto(xy, C.PROBE_TOUCH_Z + C.PRE_PLACE, C.VERTICAL_DURATION)
        # Guarded move: slow descent, stop as soon as the sensed contact force rises.
        # (Soft impedance stalls cm short on joint friction.)
        target = C.PROBE_TOUCH_Z - C.PROBE_DEPTH
        z, peak, contact = self.guarded_down(xy, target, C.PROBE_DURATION)
        print(f"    probe: reached z {z:.4f} (target {target:.4f}), "
              f"peak force {peak:.1f} N{' -> CONTACT' if contact else ''}")
        # Clean results: contact at ~PROBE_TOUCH_Z (down) or no contact well below it (up).
        # (Without contact the arm still stops ~1 cm short of the target: joint friction.)
        if contact and z < C.PROBE_TOUCH_Z - 0.005:
            print("    WARNING: contact below the cup top -> fingers hit the inside wall or "
                  "slid off the base edge. Cup is probably off-centre; re-measure its xy.")
        elif contact and z > C.PROBE_TOUCH_Z + 0.008:
            print("    WARNING: contact above the cup top -> force noise or wrong PROBE_TOUCH_Z.")
        self.goto(xy, self.travel_z, C.VERTICAL_DURATION)
        if C.PROBE_MODE == "touch":
            # Decide by contact, not depth: the depth reached is ~1 cm off either way.
            return ("down" if contact else "up"), C.PROBE_TOUCH_Z - z
        return classify_probe(z, C.PROBE_TOUCH_Z, C.PROBE_DEPTH, C.PROBE_MODE), C.PROBE_TOUCH_Z - z

    def to_shelf(self, xy):
        """Rim-up cup on the table -> rim-up on the shelf. Picked from the top by PINCHING THE RIM
        wall: the open gripper straddles the far (+x) rim, one finger inside the cup, one outside,
        then squeezes. (An inside spread grasp slips: the cup widens upward. An outside grasp from
        the top is impossible: the rim is wider than the gripper opens.)"""
        off = np.array([C.RIM_RADIUS, 0.0])  # fingers open along world x (either topdown() rotation)
        pinch_z = C.PROBE_TOUCH_Z - C.PINCH_DEPTH
        at = np.asarray(xy) + off
        # Always from home: the shelf moves (and the flip's homing afterwards) were only ever
        # proven with the arm folded the way it is when coming from home.
        self.start_topdown(at, force_home=True)  # rotation kept for the carry to the shelf
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.sgoto(at, self.travel_z, C.TRAVEL_DURATION)
        self.sgoto(at, pinch_z, C.VERTICAL_DURATION)
        self.fa.goto_gripper(0.0, grasp=True, force=C.PINCH_FORCE)
        w = self.fa.get_gripper_width()
        print(f"    rim pinch: fingers closed to {w * 1000:.1f} mm", flush=True)
        if w < 0.0005:
            # Fully closed = no wall between the fingers: nothing is held.
            self.fa.goto_gripper(C.OPEN_WIDTH)
            self.sgoto(at, self.travel_z, C.VERTICAL_DURATION)
            raise RuntimeError("rim pinch found no wall between the fingers. Check RIM_RADIUS / the cup xy.")
        self.sgoto(at, self.shelf_travel_z, C.VERTICAL_DURATION)  # straight up, then over
        shelf_at = np.asarray(C.SHELF_XY) + off
        z = pinch_z + self.shelf_dz + C.PLACE_DROP
        self.sgoto(shelf_at, self.shelf_travel_z, C.TRAVEL_DURATION)
        self.sgoto(shelf_at, z + C.PRE_PLACE, C.VERTICAL_DURATION)
        self.sgoto(shelf_at, z, C.SLOW_DURATION)
        self.fa.goto_gripper(C.OPEN_WIDTH)  # release the rim
        self.sgoto(shelf_at, self.shelf_travel_z, C.VERTICAL_DURATION)
        self.clear_shelf()

    def clear_shelf(self):
        """At shelf height, move SHELF_CLEAR toward the arm, away from the shelf: home is BELOW the
        shelf, and a joint-space homing move started right above the cup can swing down into it."""
        p = self.fa.get_pose().translation
        out = side_approach_xy(C.SHELF_XY, C.SHELF_CLEAR)
        self.sgoto(out, max(p[2], self.shelf_travel_z), C.TRAVEL_DURATION,
                   rotation=self.fa.get_pose().rotation)

    def center(self, xy, dz=0.0, x_only=False):
        """Rim-up cup standing dz above the table: open the fingers against the INSIDE wall (along
        x, then along y). The cup slides until centred between the fingers, so its xy is known to
        ~1 mm for the side grasp."""
        above = C.PROBE_TOUCH_Z + dz + C.CLEARANCE
        base = self.topdown(xy)  # fingers open along world x
        rots = (base,) if x_only else (base, base @ rot_z(np.pi / 2))
        for rot in rots:
            self.fa.goto_gripper(0.0)
            self.sgoto(xy, above, C.TRAVEL_DURATION, rot)
            self.sgoto(xy, C.CENTER_Z + dz, C.VERTICAL_DURATION, rot)
            self.fa.goto_gripper(C.OPEN_WIDTH, grasp=True, force=C.CENTER_FORCE)  # spread outward
            self.fa.goto_gripper(0.0)
            self.sgoto(xy, above, C.VERTICAL_DURATION, rot)

    def flip(self):
        """Rim-up cup on the shelf -> rim-down on the shelf, same spot. Returns False (cup
        untouched or put back as it was) if joint 7 has no room for a 180 deg roll; then the cup
        must be flipped by hand on the shelf."""
        xy = C.SHELF_XY
        back = side_approach_xy(xy)
        if C.CENTER_ON_SHELF:
            self.home()  # centring picks its own top-down rotation: start it from home
            # Along x only: the side-grasp fingers open along x with only ~7 mm spare per side;
            # along y (the slide-in direction) there is plenty of room.
            self.center(xy, self.shelf_dz, x_only=True)
            # Back home before the joint move: a joint-space move straight from above the cup
            # swings the hand through the cup. Clear the shelf first, then home.
            self.clear_shelf()
            self.home()
        self.fa.goto_gripper(C.OPEN_WIDTH)
        if C.SIDE_APPROACH_JOINTS is None:
            raise RuntimeError("config.py: SIDE_APPROACH_JOINTS not measured yet.")
        # Joint-space move to the hand-guided approach pose: keeps the arm folded the way it was
        # guided (a Cartesian move from home picks its own elbow/wrist and parked joint 7 at its limit).
        self.fa.goto_joints(list(C.SIDE_APPROACH_JOINTS), duration=C.SIDE_JOINTS_DURATION)
        print(f"    side approach: q7 {self.fa.get_joints()[6]:.3f}", flush=True)
        if roll_target_q7(self.fa.get_joints()[6]) is None:
            self.home()
            return False
        self.sgoto(back, C.SIDE_GRASP_Z, C.VERTICAL_DURATION, C.SIDE_ROTATION)
        # Stop SIDE_SLIDE_SHORT before the cup centre: going all the way pushes the cup away.
        xy = side_approach_xy(C.SHELF_XY, C.SIDE_SLIDE_SHORT)
        self.sgoto(xy, C.SIDE_GRASP_Z, C.SLOW_DURATION, C.SIDE_ROTATION)
        self.fa.goto_gripper(C.SIDE_GRASP_WIDTH, grasp=True, force=C.GRASP_FORCE)
        # Actual tool z at the grasp (the hand may rest on the shelf above the commanded z):
        # the set-down height after the flip is computed from this, not from SIDE_GRASP_Z.
        grasp_p = self.fa.get_pose().translation
        grasp_z = grasp_p[2]
        print(f"    side grasp: width {self.fa.get_gripper_width() * 1000:.1f} mm, "
              f"q7 {self.fa.get_joints()[6]:.3f}, z {grasp_z:.4f} (cmd {C.SIDE_GRASP_Z:.4f})", flush=True)
        self.sgoto(xy, C.SIDE_GRASP_Z + C.FLIP_LIFT, C.VERTICAL_DURATION, C.SIDE_ROTATION)

        q = self.fa.get_joints()
        q7 = roll_target_q7(q[6])  # re-check: the lift changed the joints a little
        if q7 is None:
            self.sgoto(xy, C.SIDE_GRASP_Z + C.PLACE_DROP, C.VERTICAL_DURATION, C.SIDE_ROTATION)
            self.fa.goto_gripper(C.OPEN_WIDTH)
            self.sgoto(back, C.SIDE_GRASP_Z, C.SLOW_DURATION, C.SIDE_ROTATION)
            self.sgoto(back, self.shelf_travel_z, C.VERTICAL_DURATION, C.SIDE_ROTATION)
            self.home()
            return False
        q[6] = q7
        self.fa.goto_joints(q, duration=C.TRAVEL_DURATION)  # the flip: roll about the horizontal tool axis

        rolled = self.fa.get_pose().rotation
        # The cup centre lies on the tool axis, so it keeps its xy through the roll: moving the
        # hand SETDOWN_SHIFT back moves the cup the same, toward the arm (= flipped_shelf_xy()).
        xy = side_approach_xy(C.SHELF_XY, C.SIDE_SLIDE_SHORT + C.SETDOWN_SHIFT)
        # Set down at the same xy, from the ACTUAL grasp height: gripped SIDE_GRASP_H above the
        # shelf -> after the roll that point is SIDE_GRASP_H below the new top, i.e. the hand must
        # be CUP_HEIGHT - 2*SIDE_GRASP_H above the grasp height. Stiff descent so it gets there;
        # the force check (SETDOWN_FORCE, above the ~3-4 N motion noise) is only a safety stop.
        z_set = grasp_z + C.CUP_HEIGHT - 2 * C.SIDE_GRASP_H + C.PLACE_DROP
        self.sgoto(xy, z_set + C.PRE_PLACE, C.VERTICAL_DURATION, rolled)
        z, peak, contact = self.guarded_down(xy, z_set, 4.0, rolled, force=C.SETDOWN_FORCE,
                                             cartesian_impedances=C.STIFF_IMPEDANCES)
        print(f"    set-down: target {z_set:.4f}, released at {z:.4f} "
              f"(peak {peak:.1f} N{', stopped on contact' if contact else ''})", flush=True)
        # Remember where the cup actually landed, from the ACTUAL hand positions: the cup keeps its
        # offset from the hand along the pointing axis through the roll (the sideways part flips
        # sign), plus the observed extra FLIP_LANDING_OFFSET away from the arm.
        rel_p = self.fa.get_pose().translation
        a = side_dir()
        off = np.asarray(C.SHELF_XY) - grasp_p[:2]
        along = off @ a
        self.flipped_xy = tuple(rel_p[:2] + along * a - (off - along * a) + C.FLIP_LANDING_OFFSET * a)
        print(f"    landed: grasp hand {np.round(grasp_p[:2], 4)}, release hand {np.round(rel_p[:2], 4)} "
              f"-> flipped cup at {np.round(self.flipped_xy, 4)}", flush=True)
        self.fa.goto_gripper(C.OPEN_WIDTH)
        self.sgoto(back, z, C.SLOW_DURATION, rolled)
        self.sgoto(back, self.shelf_travel_z, C.VERTICAL_DURATION, rolled)
        self.home()
        return True

    def place(self, xy, layer):
        z = C.GRASP_Z + layer * C.LAYER_HEIGHT + C.PLACE_DROP
        self.goto(xy, max(self.travel_z, self.fa.get_pose().translation[2]), C.TRAVEL_DURATION)
        self.sgoto(xy, z + C.PRE_PLACE, C.VERTICAL_DURATION)
        # Set down by touch (stiff, like the pick, so the two match): keep going up to
        # PLACE_SEARCH below the nominal height and release as soon as the cup presses on its
        # support. A soft descent stopped ~1 cm high and LAYER_HEIGHT is only an estimate:
        # upper-layer cups were dropped ~2 cm onto curved cup bottoms and fell.
        zr, peak, contact = self.guarded_down(xy, z - C.PLACE_SEARCH, 4.0, force=C.SETDOWN_FORCE,
                                              cartesian_impedances=C.STIFF_IMPEDANCES)
        print(f"    place L{layer + 1}: nominal {z:.4f}, released at {zr:.4f} "
              f"(peak {peak:.1f} N{', contact' if contact else ', no contact'})", flush=True)
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
    ap.add_argument("--shortest", action="store_true",
                    help="choose the cup order for the shortest travel (default: use cups in cups.json order)")
    ap.add_argument("--refill", action="store_true",
                    help="cups come one at a time from their start spot (refilled by hand): wait for "
                         "Enter, probe, (flip,) place, repeat")
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
        # cups.json order (default) also finishes each layer before the next (base, 2nd, 3rd)
        steps, cost = plan(cups, slots, C.HOME_XY, args.layer_by_layer or not args.shortest,
                           in_order=not args.shortest)
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
                    + [("shelf", C.SHELF_XY)]
                    + [(f"slot L{s.layer + 1}.{s.index}", s.xy) for s in slots])
    if C.SHELF_Z <= C.TABLE_Z + 0.02:
        raise SystemExit("config.py: SHELF_Z must be measured (tool z touching the shelf, above TABLE_Z).")
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
        # 1) Scan: orientation of every cup the plan uses, in cups.json order. (--refill: each cup
        #    is probed in the build loop instead, right after it is put on the start spot.)
        for i, cup in enumerate(sorted(used, key=cups.index), 1):
            if args.refill:
                break
            if args.skip_orientation_check:
                found[cup.id] = cup.orientation
                continue
            pause(f"[probe {i}/{len(used)}] Enter to probe {cup.id} ...")
            found[cup.id], sinks[cup.id] = robot.probe(cup.xy)
            print(f"  {cup.id}: {found[cup.id]} (sank {sinks[cup.id] * 1000:.1f} mm)"
                  + ("" if found[cup.id] == cup.orientation else f"  [cups.json says {cup.orientation}]"))
        robot.home()

        # 2) Build. A rim-up cup goes table -> shelf -> flip on the shelf -> picked from the shelf.
        #    (One shelf spot, so each rim-up cup is flipped right before it is placed.)
        for i, (cup, slot) in enumerate(steps, 1):
            src, dz = cup.xy, 0.0
            if args.refill:
                input(f"[{i}/{len(steps)}] Put a {cup.color.upper()} cup ({cup.id}) on the start spot "
                      f"{tuple(round(v, 3) for v in cup.xy)}, either way up, hands clear, then Enter ...")
                if args.skip_orientation_check:
                    found[cup.id] = cup.orientation
                else:
                    found[cup.id], sinks[cup.id] = robot.probe(cup.xy)
                    print(f"  {cup.id}: {found[cup.id]}")
            if found[cup.id] == "up":
                pause(f"[{i}/{len(steps)}] Enter to move rim-up {cup.id} to the shelf ...")
                robot.to_shelf(cup.xy)
                pause(f"[{i}/{len(steps)}] Enter to flip {cup.id} on the shelf ...")
                if robot.flip():
                    flipped_robot.append(cup.id)
                else:
                    input(f"  Wrist can't roll 180 deg for {cup.id}. Flip it BY HAND on the shelf "
                          "(rim down, same spot), then Enter ...")
                    flipped_hand.append(cup.id)
                src = robot.flipped_xy if robot.flipped_xy is not None else flipped_shelf_xy()
                dz = robot.shelf_dz
            pause(f"[{i}/{len(steps)}] Enter to pick {cup.id} ...")
            robot.pick(src, dz, slot.xy)
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
