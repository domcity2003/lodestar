"""Verify the saved seven-site compact-XPS route replay against its archived control."""
import numpy as np
import pandas as pd

from flyable_mission import MODES, N, OUT, PATTERN, paired_intervals
from terrain_sim import SITES


def main():
    directory = OUT / "flyable"
    runs = pd.read_csv(directory / "runs.csv.gz")
    keys = ["pattern", "site", "aircraft", "run"]
    assert len(runs) == 2 * len(SITES) * 2 * N * len(MODES)
    assert not runs.duplicated([*keys, "mode"]).any()
    assert runs.groupby(keys).ws.nunique().eq(1).all()
    assert runs.groupby(keys).mode.nunique().eq(len(MODES)).all()
    assert runs[["help_1h", "help_3h"]].ge(0).all().all()
    assert runs.help_1h.le(runs.help_3h).all() and runs.help_3h.le(runs.found).all()
    release = runs.dropna(subset=["t_kit"])
    assert release.found.all() and release.t_kit.gt(release.t_find).all()

    metrics = ["found", "help_1h", "help_3h"]
    archive = pd.read_csv(OUT / "terrain_by_site.csv").query("design == 'compact_xps'")
    archive = archive.set_index(["site", "aircraft"])[metrics].sort_index()
    sharp = runs.query("pattern == @PATTERN and mode == 'sharp'")
    np.testing.assert_allclose(sharp.groupby(["site", "aircraft"])[metrics].mean(), archive, atol=1e-12)

    rounded = runs.query("mode != 'sharp'")
    assert rounded.found.eq(rounded.source.notna()).all()
    assert rounded.found.eq(rounded.cutoff == "found").all()
    assert rounded.searched_km.ge(0).all()
    intervals = paired_intervals(runs)
    intervals.to_csv(directory / "paired_intervals.csv", index=False)
    print(f"Verified {len(runs):,} cases and exact archived compact-XPS site averages.")


if __name__ == "__main__":
    main()
