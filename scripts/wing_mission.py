"""Rerun the same terrain missions at each wing mass from wing_trade.py."""
from concurrent.futures import ProcessPoolExecutor
import math

import numpy as np
import pandas as pd

import terrain_sim as t


OUT = t.ROOT / "out" / "wing_trade"
PATTERN = "trails 6"
N = 500  # same hikers, random draws and pattern as the published terrain run


def compare_site(args):
    site, masses = args
    prep = t.prep(site, N, t.ms.RNG_SEED)
    altitude = prep["z_home"] + t.H_SEARCH
    hikers = prep["hikers"]
    spacing = t.CAMERAS["Lodestar"][1]
    pat = prep["pats"][(PATTERN, spacing)]
    dem = t.load(site)[0]
    cover, hide = t.cover_draws(site, hikers, t.ms.RNG_SEED)
    chute = pd.read_csv(t.ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")

    def reach(wind):
        return np.interp(wind, chute.wind, chute.p_reach)

    results = []
    for design, mass in masses:
        fleet = t.site_fleet(altitude, wing_mass_kg=mass, include_competitors=False)
        for ac in fleet:
            # Use this aircraft's mass to compute its own climb energy.
            climb = t.climb_k(ac, ac)
            tan_half = t.camera_tangent(ac)
            rows = []
            for i, h in enumerate(hikers.itertuples()):
                if h.on_map:
                    wind = h.ws * math.cos(h.wd), h.ws * math.sin(h.wd)
                    row = t.fly(ac, pat, (h.tx, h.ty), h.z, wind, h.draws, climb, tan_half,
                                prep["z_home"], dem, (cover[i], hide[i]))
                else:
                    row = dict(found=False, t_find=None, t_kit=None, h_agl=None)
                rows.append(row)
            runs = pd.DataFrame(rows).rename_axis("run").reset_index()
            scored = t.score(runs, hikers, reach)
            results.append(scored[["run", "found", "help_1h", "help_3h"]].assign(
                site=site, design=design, aircraft=ac.name))
    print(f"Completed {site}", flush=True)
    return pd.concat(results, ignore_index=True)


def main():
    trade = pd.read_csv(OUT / "summary.csv").set_index("design")
    masses = [("original mass budget", None), *trade.wing_mass_kg.items()]
    with ProcessPoolExecutor(max_workers=4) as pool:
        runs = pd.concat(pool.map(compare_site, [(site, masses) for site in t.SITES]), ignore_index=True)
    baseline = runs.loc[runs.design == "original mass budget", ["site", "aircraft", "run", "help_3h"]]
    paired = runs.merge(baseline.rename(columns={"help_3h": "baseline_help_3h"}),
                        on=["site", "aircraft", "run"], validate="many_to_one")
    paired["delta_help_3h"] = paired.help_3h - paired.baseline_help_3h
    columns = ["found", "help_1h", "help_3h", "delta_help_3h"]
    by_site = paired.groupby(["design", "aircraft", "site"])[columns].mean()
    by_site.to_csv(OUT / "terrain_by_site.csv")
    summary = by_site.groupby(["design", "aircraft"])[columns].mean()
    summary.to_csv(OUT / "terrain_summary.csv")
    print(summary.round(4).to_string(), flush=True)


if __name__ == "__main__":
    main()
