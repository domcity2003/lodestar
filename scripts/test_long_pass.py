"""Checks for the long-pass route helpers in long_pass.py.

Run:  .venv/Scripts/python.exe -X utf8 scripts/test_long_pass.py
"""
import math
import sys

import numpy as np

import long_pass as lp
from turn_path import rounded_path


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0


def headings(track):
    d = np.diff(track, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def test_simplify():
    spur = np.array([[0, 0], [100, 0], [200, 0], [300, 0], [200, 1], [100, 0], [0, 1]], float)
    kept = lp.simplify(spur, 5.0)
    check("spur tip kept by segment-distance simplification", any(np.allclose(p, [300, 0]) for p in kept))
    line = np.column_stack([np.linspace(0, 1000, 50), np.zeros(50)])
    check("straight line collapses to two points", len(lp.simplify(line, 1.0)) == 2)


def test_pursuit_turn_limit():
    # Switchbacks: 100 m legs reversing direction, 40 m apart.
    pts = []
    for k in range(8):
        y = 40.0 * k
        pts += [[0.0, y], [100.0, y]] if k % 2 == 0 else [[100.0, y], [0.0, y]]
    radius = 60.0
    track, path = lp.pursue(np.array(pts), radius, radius)
    turn = np.abs((np.diff(headings(track)) + math.pi) % math.tau - math.pi)
    check("pursuit never turns tighter than its radius", turn.max() <= lp.STEP / radius + 1e-9,
          f"max {turn.max():.4f} rad/step vs {lp.STEP / radius:.4f}")
    check("pursuit reaches the end of the path", math.dist(track[-1], path[-1]) < 3 * lp.STEP,
          f"{math.dist(track[-1], path[-1]):.1f} m short")


def test_pursuit_legs_fit():
    rng = np.random.default_rng(1)
    angles = np.cumsum(rng.normal(0, 1.2, 200))
    steps = rng.uniform(40, 150, 200)
    pts = np.vstack([[0, 0], np.cumsum(np.column_stack([steps * np.cos(angles), steps * np.sin(angles)]), axis=0)])
    legs = [(tuple(a), tuple(b)) for a, b in zip(pts[:-1], pts[1:])] + [(tuple(pts[-1]), (0.0, 0.0))]
    radius = 100.0
    _, base_loops = rounded_path(legs[:-1], radius)[1:]
    new, miss = lp.pursuit_legs(legs, radius, 1.0)
    home = next(i for i, (_, b) in enumerate(new) if b == (0.0, 0.0))
    _, loops = rounded_path(new[:home], radius)[1:]
    check("pursuit track needs far fewer turnaround loops on a wiggly trail", loops < base_loops / 5,
          f"{loops} vs {base_loops}")
    check("pursuit legs are contiguous", all(math.dist(a[1], b[0]) < 1e-6 for a, b in zip(new[:home], new[1:home + 1])))
    check("legs end with the original return home", new[home][1] == (0.0, 0.0))


def test_pass_ids():
    square = [((0, 0), (100, 0)), ((100, 0), (100, 100)), ((100, 100), (0, 100))]
    ids = lp.pass_ids(square)
    check("a 90 deg corner stays in one pass; the second 90 deg starts a new one", list(ids) == [0, 0, 1], str(ids))
    gentle =[((0.0, 0.0), (10.0, 0.0))] + [((10.0 * k, 0.0), (10.0 * (k + 1), 0.0)) for k in range(1, 5)]
    check("straight legs share one pass", len(set(lp.pass_ids(gentle))) == 1)
    gap = [((0, 0), (100, 0)), ((200, 0), (300, 0))]
    check("a gap starts a new pass", list(lp.pass_ids(gap)) == [0, 1])


if __name__ == "__main__":
    for f in (test_simplify, test_pursuit_turn_limit, test_pursuit_legs_fit, test_pass_ids):
        print(f"--- {f.__name__}")
        f()
    print(f"\n{check.failed} failed")
    sys.exit(1 if check.failed else 0)
