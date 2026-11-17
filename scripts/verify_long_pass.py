"""Archive checks for the long-pass runs and cross-archive invariants (reads saved runs; no replay).

  - out/wing_trade/long_pass_tune and long_pass_seed_20261114: expected row counts, unique keys, matched
    hikers across modes, help_1h <= help_3h <= found, found <=> cutoff == "found";
  - held-out draws really differ from the tuning draws (different hikers and winds);
  - every archive with a kit (flyable, flyable_fleet, long_pass_*): a kit is released only when found,
    only at >= DROP_HEIGHT above the person, exactly DROP_TIME + DROP_FALL_TIME after the find, and
    never by a competitor.

Run:  ../.venv/Scripts/python.exe -X utf8 verify_long_pass.py
"""
import sys

import numpy as np
import pandas as pd

import terrain_sim as t
from long_pass import HELD_OUT_SEED, PATTERNS, TUNING_SEED
from wing_mission import N, OUT

failed = 0


def check(name, ok, detail=""):
    global failed
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    failed += not ok


def common(runs, label, rows):
    keys = ["site", "pattern", "aircraft", "mode", "run"]
    check(f"{label}: {rows:,} rows", len(runs) == rows, f"{len(runs):,}")
    check(f"{label}: unique run keys", not runs.duplicated(keys).any())
    check(f"{label}: same hiker wind in every mode",
          runs.groupby(["site", "pattern", "aircraft", "run"]).ws.nunique().eq(1).all())
    check(f"{label}: 0 <= help_1h <= help_3h <= found",
          runs.help_1h.ge(0).all() and runs.help_1h.le(runs.help_3h + 1e-12).all()
          and runs.help_3h.le(runs.found + 1e-12).all())
    check(f"{label}: found <=> cutoff 'found'", runs.found.eq(runs.cutoff == "found").all())


def kit(runs, label):
    k = runs.dropna(subset=["t_kit"])
    delay = t.ms.DROP_TIME + t.DROP_FALL_TIME
    check(f"{label}: kit only after a find", k.found.astype(bool).all(), f"{len(k):,} releases")
    check(f"{label}: kit only >= {t.DROP_HEIGHT:g} m above the person", k.h_agl.ge(t.DROP_HEIGHT - 1e-9).all(),
          f"min {k.h_agl.min():.1f} m" if len(k) else "")
    check(f"{label}: kit lands DROP_TIME + fall time after the find",
          np.allclose(k.t_kit - k.t_find, delay, atol=1e-6))
    check(f"{label}: competitors never release a kit",
          runs.loc[~runs.aircraft.str.startswith("Lodestar"), "t_kit"].isna().all())


def main():
    tune = pd.read_csv(OUT / "long_pass_tune" / "runs.csv.gz")
    common(tune, "long_pass_tune", len(t.SITES) * 1 * 2 * 5 * N)
    check("long_pass_tune: trails 6 only, SiCore and Evo GOV",
          set(tune.pattern) == {"trails 6"} and set(tune.aircraft) == {"Lodestar SiCore", "DeltaQuad Evo GOV"})
    held = pd.read_csv(OUT / f"long_pass_seed_{HELD_OUT_SEED}" / "runs.csv.gz")
    common(held, f"long_pass_seed_{HELD_OUT_SEED}", len(t.SITES) * len(PATTERNS) * 7 * 3 * N)
    check("held-out modes are rounded_adaptive, rounded_passes, pursuit_1",
          set(held["mode"]) == {"rounded_adaptive", "rounded_passes", "pursuit_1"})

    # Seed hygiene: the held-out hikers are new. Compare wind speeds per (site, run).
    a = tune.drop_duplicates(["site", "run"]).set_index(["site", "run"]).ws
    b = held.drop_duplicates(["site", "run"]).set_index(["site", "run"]).ws
    same = (a - b.reindex(a.index)).abs().lt(1e-12).mean()
    check(f"held-out seed {HELD_OUT_SEED} draws differ from tuning seed {TUNING_SEED}", same < 0.01,
          f"{same:.1%} of runs share a wind")

    for label, runs in (("long_pass_tune", tune), (f"long_pass_seed_{HELD_OUT_SEED}", held),
                        ("flyable", pd.read_csv(OUT / "flyable" / "runs.csv.gz")),
                        ("flyable_fleet", pd.read_csv(OUT / "flyable_fleet" / "runs.csv.gz"))):
        kit(runs, label)
    print(f"\n{failed} failed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
