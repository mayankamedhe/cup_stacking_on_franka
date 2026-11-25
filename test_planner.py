"""Offline checks, no robot needed:  python test_planner.py"""
import itertools
import math
import random

from goal_parser import GoalParseError, parse_goal
from orientation import classify_probe, flipped_grasp_z, roll_target_q7
from planner import Cup, InfeasibleError, build_slots, plan


def test_parser():
    Y, O, R, B = "yellow", "orange", "red", "blue"
    cases = {
        "base layer of yellow, 2nd layer of orange, top red": [[Y] * 3, [O] * 2, [R]],
        "3 yellow at the bottom, then 2 orange, then 1 red": [[Y] * 3, [O] * 2, [R]],
        "top red; bottom yellow; second orange": [[Y] * 3, [O] * 2, [R]],
        "bottom: yellow red yellow; middle: blue blue; top: red": [[Y, R, Y], [B, B], [R]],
        "base 2 red and 2 blue, then orange, then orange, top yellow": [[R, R, B, B], [O] * 3, [O] * 2, [Y]],
        "layer 1 yellow, layer 2 red": [[Y, Y], [R]],
        "4 blue, then red": [[B] * 4, [R] * 3],
    }
    for text, want in cases.items():
        got = parse_goal(text)
        assert got == want, f"{text!r}: {got}"
    for bad in ["make it nice", "base 3 red, then 3 blue", "layer 1 red, layer 3 blue"]:
        try:
            parse_goal(bad)
            raise AssertionError(f"should fail: {bad!r}")
        except GoalParseError:
            pass


def brute_force(cups, slots, start):
    """Try every order and cup assignment (tiny instances only)."""
    best = math.inf
    for order in itertools.permutations(range(len(slots))):
        pos = {s: i for i, s in enumerate(order)}
        if any(pos[t] > pos[s] for s in order for t in slots[s].supports):
            continue
        for assign in itertools.permutations(range(len(cups)), len(slots)):
            if any(cups[c].color != slots[s].color for c, s in zip(assign, order)):
                continue
            xy, cost = start, 0.0
            for c, s in zip(assign, order):
                cost += math.dist(xy, cups[c].xy) + math.dist(cups[c].xy, slots[s].xy)
                xy = slots[s].xy
            best = min(best, cost)
    return best


def test_optimal_and_valid():
    rng = random.Random(0)
    for trial in range(20):
        layers = parse_goal(rng.choice(["base yellow red yellow, then blue blue, top red",
                                        "base yellow, then red, top yellow"]))
        slots = build_slots(layers, (0.5, 0.0), 0.095)
        need = [s.color for s in slots] + rng.sample(["red", "blue", "yellow"], 1)  # + 1 spare cup
        cups = [Cup(f"c{i}", col, (rng.uniform(0.3, 0.7), rng.uniform(-0.35, 0.35)))
                for i, col in enumerate(need)]
        steps, cost = plan(cups, slots, (0.31, 0.0))
        placed = set()
        for cup, slot in steps:  # supports first, colours match, each cup once
            assert cup.color == slot.color and all(slots[t] in placed for t in slot.supports)
            placed.add(slot)
        assert len({c.id for c, _ in steps}) == len(steps) == len(slots)
        assert abs(cost - brute_force(cups, slots, (0.31, 0.0))) < 1e-9, trial


def test_infeasible():
    slots = build_slots(parse_goal("base red, top blue"), (0.5, 0.0), 0.095)
    try:
        plan([Cup("r", "red", (0.4, 0.2)), Cup("b", "blue", (0.4, -0.2))], slots, (0.31, 0.0))
        raise AssertionError("should be infeasible")
    except InfeasibleError as e:
        assert "1 more red" in str(e)


def test_big_is_fast():
    import time
    rng = random.Random(1)
    layers = parse_goal("base 2 yellow and 2 red, then orange, then blue, top red")  # 10 cups
    slots = build_slots(layers, (0.5, 0.0), 0.095)
    cols = [s.color for s in slots] + ["yellow", "red", "blue"]
    cups = [Cup(f"c{i}", c, (rng.uniform(0.3, 0.7), rng.uniform(-0.35, 0.35))) for i, c in enumerate(cols)]
    t = time.perf_counter()
    plan(cups, slots, (0.31, 0.0))
    dt = time.perf_counter() - t
    assert dt < 10, dt
    print(f"  10-cup pyramid, 13 cups available: {dt * 1000:.0f} ms")


def test_orientation():
    touch, depth = 0.112, 0.02
    assert classify_probe(touch + 0.001, touch, depth) == "down"   # stopped on the solid base
    assert classify_probe(touch - 0.004, touch, depth) == "down"   # base flexed a little
    assert classify_probe(touch - 0.019, touch, depth) == "up"     # sank into the open rim
    assert classify_probe(touch - 0.019, touch, depth, "gauge") == "down"  # narrow base fit the gap
    assert classify_probe(touch + 0.001, touch, depth, "gauge") == "up"    # wide rim stopped it
    try:
        classify_probe(touch, touch, depth, "diameter")
        raise AssertionError("bad mode should fail")
    except ValueError:
        pass
    assert abs(roll_target_q7(-1.5) - (-1.5 + math.pi)) < 1e-9
    assert abs(roll_target_q7(1.5) - (1.5 - math.pi)) < 1e-9
    assert roll_target_q7(0.0) is None                             # +-pi both outside +-2.897
    # gripped 4 cm above the table before the flip -> 4 cm below the new top after it
    assert abs(flipped_grasp_z(0.045, 0.005, 0.11) - (0.005 + 0.11 - 0.04)) < 1e-9


def test_cups_json_and_log():
    import csv, json, os, tempfile
    from main import load_cups, log_trial
    with tempfile.TemporaryDirectory() as d:
        cj = os.path.join(d, "cups.json")
        json.dump({"cups": [{"id": "a", "color": "Red", "xy": [0.5, 0]},
                            {"id": "b", "color": "red", "xy": [0.5, 0.1], "orientation": "UP"}]}, open(cj, "w"))
        a, b = load_cups(cj)
        assert (a.color, a.orientation, b.orientation) == ("red", "down", "up")
        log = os.path.join(d, "log.csv")
        log_trial(log, {"time": 1, "success": "y"})
        log_trial(log, {"time": 2, "success": "n", "n_flipped_robot": 1})   # new column -> header merged
        log_trial(log, {"time": 3, "success": "y", "n_flipped_robot": 0})
        rows = list(csv.DictReader(open(log)))
        assert [r["time"] for r in rows] == ["1", "2", "3"]
        assert [r["n_flipped_robot"] for r in rows] == ["", "1", "0"]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
