"""Rule-based parser: English text  ->  pyramid layers (list of colour lists, base first).

Examples that work:
    "base layer of yellow, 2nd layer of orange, top red"
    "3 yellow at the bottom, then 2 orange, then 1 red"
    "bottom: yellow red yellow; middle: blue blue; top: green"
    "base 2 red and 2 blue; then orange; then orange; top yellow"

Rules:
  * Layers are separated by  ,  ;  newline  or the word "then".
  * Inside one layer, list colours with spaces / "and" (e.g. "yellow red yellow").
  * Layer words: base/bottom/first/1st/layer 1, second/2nd, third/3rd, ..., middle, top/last.
    Clauses without a layer word take the next layer up. "top" is always the highest layer.
  * Pyramid shape: each layer has one cup fewer than the one below. Base size is taken from
    any explicit count (e.g. "3 yellow"), otherwise base size = number of layers (top = 1 cup).
  * A single colour without a count fills the whole layer.
"""
import re

COLORS = {"red", "orange", "yellow", "green", "blue", "purple", "pink",
          "white", "black", "brown", "gray", "grey", "clear"}
NUMBERS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
           "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, "a": 1, "an": 1}
ORDINALS = {"base": 0, "bottom": 0, "first": 0, "1st": 0, "lowest": 0,
            "second": 1, "2nd": 1, "third": 2, "3rd": 2, "fourth": 3, "4th": 3,
            "fifth": 4, "5th": 4}
TOP_WORDS = {"top", "last", "peak", "highest", "uppermost"}


class GoalParseError(ValueError):
    pass


def _clauses(text):
    parts = re.split(r"[;,\n]|\bthen\b", text.lower())
    return [p.strip() for p in parts if p.strip()]


def _parse_clause(clause):
    """Returns (marker, [(count|None, color), ...]). marker: int | 'top' | None."""
    words = re.findall(r"[a-z0-9]+", clause)
    marker = None
    m = re.search(r"\b(?:layer|level|row|tier)\s+(\d+)\b", clause)
    if m:
        marker = int(m.group(1)) - 1
    spec, pending = [], None
    for w in words:
        if marker is None and w in ORDINALS:
            marker = ORDINALS[w]
        elif marker is None and w in TOP_WORDS:
            marker = "top"
        elif w.isdigit() and not (m and w == m.group(1)):
            pending = int(w)
        elif w in NUMBERS and w not in ("a", "an"):
            pending = NUMBERS[w]
        elif w.rstrip("s") in COLORS or w in COLORS:
            color = w if w in COLORS else w.rstrip("s")
            spec.append((pending, "gray" if color == "grey" else color))
            pending = None
    return marker, spec


def _expand(spec, size, layer_no):
    counts = [c for c, _ in spec]
    if len(spec) == 1 and counts[0] is None:
        return [spec[0][1]] * size
    if all(c is None for c in counts) and len(spec) == size:
        return [col for _, col in spec]  # "yellow red yellow" = one each, in order
    missing = [i for i, c in enumerate(counts) if c is None]
    known = sum(c for c in counts if c is not None)
    if len(missing) == 1:
        counts[missing[0]] = size - known
    elif missing:
        raise GoalParseError(f"layer {layer_no}: give counts, e.g. '2 red and 1 blue'")
    if sum(counts) != size or min(counts) < 1:
        raise GoalParseError(f"layer {layer_no}: needs {size} cups, got {sum(counts)}")
    return [col for c, (_, col) in zip(counts, spec) for _ in range(c)]


def parse_goal(text):
    """English -> list of layers, base first. Each layer is a list of colour names."""
    numbered, tops, nxt = {}, [], 0
    for clause in _clauses(text):
        marker, spec = _parse_clause(clause)
        if not spec:
            continue  # e.g. "build a pyramid"
        if marker == "top":
            tops.append(spec)
            continue
        idx = nxt if marker is None else marker
        if idx in numbered:
            raise GoalParseError(f"layer {idx + 1} described twice")
        numbered[idx] = spec
        nxt = idx + 1
    specs = [numbered[k] for k in sorted(numbered)] + tops
    if not specs:
        raise GoalParseError("no colours found in goal text")
    if sorted(numbered) != list(range(len(numbered))):
        raise GoalParseError(f"layers are not contiguous from the base: {sorted(numbered)}")

    # Base size: from any explicit total count, else number of layers.
    n_layers, base = len(specs), None
    for k, spec in enumerate(specs):
        if all(c is not None for c, _ in spec):
            b = sum(c for c, _ in spec) + k
        elif len(spec) > 1 and all(c is None for c, _ in spec):
            b = len(spec) + k
        else:
            continue
        if base is not None and b != base:
            raise GoalParseError("layer sizes don't form a pyramid (each layer = one fewer cup)")
        base = b
    base = base or n_layers
    if base - (n_layers - 1) < 1:
        raise GoalParseError(f"{n_layers} layers can't fit on a base of {base}")
    return [_expand(spec, base - k, k + 1) for k, spec in enumerate(specs)]
