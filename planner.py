"""A* task planner for stacking cups into a 2D pyramid.

State  = (which slots are filled, which cups are used, where the gripper is).
Action = take an unused cup and put it in an empty slot of the same colour whose
         two supporting cups (below-left, below-right) are already placed.
Cost   = xy travel: gripper -> cup  +  cup -> slot.
h(s)   = sum over empty slots of the distance from the nearest unused matching cup.
         Every empty slot still needs one carry of at least that length, so h never
         overestimates (admissible) and the returned plan is the shortest-travel plan.
"""
import heapq
import itertools
import math
from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True)
class Cup:
    id: str
    color: str
    xy: tuple
    orientation: str = "down"  # expected: "down" (rim on table) or "up". Not used by the planner.


@dataclass(frozen=True)
class Slot:
    layer: int       # 0 = base
    index: int       # 0 = most +y cup of the layer
    color: str
    xy: tuple
    supports: tuple  # indices (into the slot list) of the two cups underneath


class InfeasibleError(Exception):
    pass


def build_slots(layers, center, spacing):
    """Pyramid layers (base first) -> list of Slots. The row runs along world y."""
    n = len(layers[0])
    slots, idx = [], {}
    for k, layer in enumerate(layers):
        if len(layer) != n - k:
            raise ValueError(f"layer {k + 1} must have {n - k} cups, has {len(layer)}")
        for j, color in enumerate(layer):
            y = center[1] + ((n - 1) / 2 - j - k / 2) * spacing  # j=0 at +y end
            sup = () if k == 0 else (idx[(k - 1, j)], idx[(k - 1, j + 1)])
            idx[(k, j)] = len(slots)
            slots.append(Slot(k, j, color, (center[0], y), sup))
    return slots


def check_colors(cups, slots):
    have, need = Counter(c.color for c in cups), Counter(s.color for s in slots)
    short = {col: need[col] - have[col] for col in need if need[col] > have[col]}
    if short:
        raise InfeasibleError("not enough cups: " +
                              ", ".join(f"need {n} more {col}" for col, n in short.items()))


def plan(cups, slots, start_xy, layer_by_layer=False, in_order=False):
    """Returns (steps, cost): steps = [(Cup, Slot), ...] in execution order.
    layer_by_layer=True: finish each layer before starting the next (more conservative).
    in_order=True: the next cup is always the first unused cup in list order that can be placed
    right now (a free slot of its colour whose supports are filled); cups that can't go yet wait
    their turn. Only the slot for each cup is chosen."""
    check_colors(cups, slots)
    d = math.dist
    nS, nC = len(slots), len(cups)
    match = [[c for c in range(nC) if cups[c].color == slots[s].color] for s in range(nS)]
    carry = [[d(cups[c].xy, slots[s].xy) for s in range(nS)] for c in range(nC)]
    # reach[p][c]: gripper position p (slot index, or nS = start) -> cup c
    reach = [[d(slots[p].xy, cups[c].xy) for c in range(nC)] for p in range(nS)]
    reach.append([d(start_xy, cups[c].xy) for c in range(nC)])
    full = (1 << nS) - 1
    below = [sum(1 << t for t in range(nS) if slots[t].layer < slots[s].layer) for s in range(nS)]

    h_cache = {}

    def h(filled, used):
        key = (filled, used)
        if key not in h_cache:
            h_cache[key] = sum(min(carry[c][s] for c in match[s] if not used >> c & 1)
                               for s in range(nS) if not filled >> s & 1)
        return h_cache[key]

    start = (0, 0, nS)
    g, parent, tie = {start: 0.0}, {start: None}, itertools.count()
    frontier = [(h(0, 0), next(tie), start)]
    while frontier:
        f, _, state = heapq.heappop(frontier)
        filled, used, pos = state
        if filled == full:
            break
        if f > g[state] + h(filled, used) + 1e-12:
            continue  # stale entry
        for s in range(nS):
            if filled >> s & 1 or not all(filled >> t & 1 for t in slots[s].supports):
                continue
            if layer_by_layer and filled & below[s] != below[s]:
                continue
            if in_order:
                ready = {slots[t].color for t in range(nS) if not filled >> t & 1
                         and all(filled >> u & 1 for u in slots[t].supports)
                         and not (layer_by_layer and filled & below[t] != below[t])}
                nxt_c = next((c for c in range(nC) if not used >> c & 1 and cups[c].color in ready), None)
            for c in match[s]:
                if used >> c & 1 or (in_order and c != nxt_c):
                    continue
                nxt = (filled | 1 << s, used | 1 << c, s)
                cost = g[state] + reach[pos][c] + carry[c][s]
                if cost < g.get(nxt, math.inf) - 1e-12:
                    g[nxt], parent[nxt] = cost, (state, c, s)
                    heapq.heappush(frontier, (cost + h(nxt[0], nxt[1]), next(tie), nxt))
    else:
        raise InfeasibleError("no plan found")

    steps, cur = [], state
    while parent[cur]:
        prev, c, s = parent[cur]
        steps.append((cups[c], slots[s]))
        cur = prev
    return steps[::-1], g[state]


def render(layers):
    """ASCII picture of the goal pyramid (top layer printed first)."""
    w = max(len(c) for layer in layers for c in layer) + 2
    n = len(layers[0])
    lines = []
    for k in reversed(range(len(layers))):
        pad = " " * (k * (w + 1) // 2)
        lines.append(pad + " ".join(f"[{c:^{w - 2}}]" for c in layers[k]))
    lines.append("-" * (n * (w + 1)) + "  table  (left = +y end)")
    return "\n".join(lines)
