"""Verify the fleet route replay: sharp-corner competitors must reproduce the archived terrain run
run-for-run, and Lodestar must reproduce the compact-XPS flyable replay."""
import sys

import numpy as np
import pandas as pd

from fleet_flyable import MODES, PATTERNS
from terrain_sim import ROOT, SITES
from wing_mission import N, OUT

FOLDERS = {1.0: ("flyable_fleet", "terrain"), 0.8: ("flyable_fleet_endurance_0.8", "terrain_endurance_0.8_trails3_trails6")}


def main(endurance):
    fleet_dir, archive_dir = FOLDERS[endurance]
    runs = pd.read_csv(OUT / fleet_dir / "runs.csv.gz")
    keys = ["pattern", "site", "aircraft", "run"]
    assert len(runs) == len(PATTERNS) * len(SITES) * 7 * N * len(MODES)
    assert not runs.duplicated([*keys, "mode"]).any()
    assert runs[["help_1h", "help_3h"]].ge(0).all().all()
    assert runs.help_1h.le(runs.help_3h + 1e-12).all() and runs.help_3h.le(runs.found + 1e-12).all()
    competitors = ~runs.aircraft.str.startswith("Lodestar")
    assert runs.loc[competitors, "t_kit"].isna().all(), "competitors carry no kit"

    metrics = ["found", "help_1h", "help_3h"]
    archive = pd.read_csv(ROOT / "out" / archive_dir / "terrain_runs.csv.gz")
    archive = archive[archive.pattern.isin(PATTERNS) & ~archive.aircraft.str.startswith("Lodestar")]
    sharp = runs[(runs["mode"] == "sharp") & competitors]
    merged = sharp.merge(archive, on=keys, suffixes=("", "_archive"), validate="one_to_one")
    assert len(merged) == len(sharp) == len(archive)
    for m in metrics:
        np.testing.assert_allclose(merged[m].astype(float), merged[f"{m}_archive"].astype(float), atol=1e-12)

    if endurance == 1.0:
        lodestar = runs[~competitors]
        reference = pd.read_csv(OUT / "flyable" / "runs.csv.gz")
        merged = lodestar.merge(reference, on=[*keys, "mode"], suffixes=("", "_ref"), validate="one_to_one")
        assert len(merged) == len(lodestar)
        for m in metrics:
            np.testing.assert_allclose(merged[m].astype(float), merged[f"{m}_ref"].astype(float), atol=1e-12)
    print(f"fleet replay verified (endurance {endurance:g}): {len(runs):,} rows")


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 1.0)
