"""Re-run one site through long_pass.route_flight and match the archived runs exactly.

Checks that the route-mode refactor (long_pass.route_flight, flyable_mission.Flight.profile) did not
change any archived number: Lodestar SiCore, one site, trails 6, tuning seed, modes rounded_adaptive,
rounded_passes and pursuit_1.
  - all three modes must match out/wing_trade/long_pass_tune/runs.csv.gz run for run;
  - rounded_adaptive must also match out/wing_trade/flyable_fleet/runs.csv.gz run for run.
This is a replay (a few minutes), so run_all_tests.py runs it only with --full.

Run:  ../.venv/Scripts/python.exe -X utf8 verify_route_modes.py [site]
"""
import sys

import numpy as np
import pandas as pd

import flyable_mission as fm
import long_pass as lp

# The archives were built before the Step 2 physics fixes; reproduce them with the archived conventions.
fm.STRAIGHT_POWER_LOCAL_DENSITY = False
lp.CONVERGED_PLANNING_RADIUS = False
# Frozen with the archive (QA open issue 3): the compact-XPS wing mass the archived runs used.
COMPACT_XPS_WING_KG = 2.291031794091632
from wing_mission import OUT

SITE = "rainier_paradise"
AIRCRAFT = "Lodestar SiCore"
METRICS = ["found", "t_find", "t_kit", "help_1h", "help_3h", "searched_km"]


def match(new, archive, label):
    keys = ["site", "pattern", "aircraft", "mode", "run"]
    archive = archive.set_index(keys).loc[new.set_index(keys).index].reset_index()
    merged = new.merge(archive, on=keys, suffixes=("", "_archive"), validate="one_to_one")
    assert len(merged) == len(new), f"{label}: {len(merged)} of {len(new)} rows matched"
    for m in METRICS:
        if f"{m}_archive" not in merged:
            continue
        a, b = merged[m].astype(float), merged[f"{m}_archive"].astype(float)
        np.testing.assert_allclose(a.fillna(-1), b.fillna(-1), atol=1e-9, err_msg=f"{label}: {m}")
    print(f"PASS  {label}: {len(merged)} runs identical ({', '.join(m for m in METRICS if f'{m}_archive' in merged)})")


def main(site):
    mass = COMPACT_XPS_WING_KG
    print(f"compact-XPS wing mass (archive value): {mass:.4f} kg")
    runs, _ = lp.compare_site((site, mass, "trails 6", (lp.DEFAULT_LOOKAHEAD,), lp.TUNING_SEED, (AIRCRAFT,)))
    tune = pd.read_csv(OUT / "long_pass_tune" / "runs.csv.gz")
    tune = tune[(tune.site == site) & (tune.aircraft == AIRCRAFT)]
    match(runs, tune, "long_pass_tune, 3 modes")
    fleet = pd.read_csv(OUT / "flyable_fleet" / "runs.csv.gz")
    fleet = fleet[(fleet.site == site) & (fleet.aircraft == AIRCRAFT) & (fleet["mode"] == "rounded_adaptive")]
    match(runs[runs["mode"] == "rounded_adaptive"], fleet, "flyable_fleet, rounded_adaptive")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else SITE)
