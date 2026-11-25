"""Cup orientation: robot-free logic, so it can be tested offline.

"down" = upside-down (rim on the table, solid base on top). This is what stacking needs.
"up"   = rim on top (open end up). Must be flipped before it can be stacked.

Both probes lower the gripper (soft in z) over the cup centre to PROBE_TOUCH_Z - PROBE_DEPTH
and look at how far it got. The top of the cup is at the same height either way up.
- "touch" (default): fingers CLOSED. A "down" cup has a solid base at the top, so the fingers
  stop there. An "up" cup is open, so the fingers drop inside.
- "gauge" (diameter fallback): fingers set to GAUGE_WIDTH, between base and rim diameter.
  A "down" cup's narrow base passes between them; an "up" cup's wide rim does not.
  Needs cups.json xy accurate to a few mm.
"""
import math

ORIENTATIONS = ("down", "up")

# Franka joint 7 limits (rad)
Q7_MIN, Q7_MAX = -2.8973, 2.8973


PROBE_MODES = ("touch", "gauge")


def classify_probe(z_reached, touch_z, depth, mode="touch"):
    """Did the fingers sink more than half the probe depth below the cup top?
    touch (closed fingers at the centre): sank = fell into the open rim      -> 'up'
    gauge (fingers set between base and rim width): sank = narrow base fits  -> 'down'"""
    sank = z_reached < touch_z - depth / 2
    if mode == "touch":
        return "up" if sank else "down"
    if mode == "gauge":
        return "down" if sank else "up"
    raise ValueError(f"PROBE_MODE must be one of {PROBE_MODES}, got {mode!r}")


def roll_target_q7(q7, margin=0.05):
    """Joint-7 angle for a 180 deg wrist roll from q7, staying inside the joint limits.
    Returns None if neither +pi nor -pi fits (then the cup must be flipped by hand)."""
    for target in (q7 + math.pi, q7 - math.pi):
        if Q7_MIN + margin <= target <= Q7_MAX - margin:
            return target
    return None


def flipped_grasp_z(side_grasp_z, table_z, cup_height):
    """Tool z for setting a just-flipped cup back on the table.
    Gripped h above the table before the flip -> h below the new top after it."""
    h = side_grasp_z - table_z
    return table_z + cup_height - h
