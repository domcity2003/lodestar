"""Step 2 improvement trades on the mission simulation (tuning draws only).

Each configuration changes one or two design or operating parameters and reruns Lodestar SiCore v1 (and the
DeltaQuad Evo GOV as a reference whenever the change applies to every aircraft) through the same continuous,
wind-adaptive pursuit route and replay as fleet_flyable.py. Only the tuning seed (20261111) is used here.
Winners are confirmed on the held-out seed (20261114), and the final headline numbers come from seed 20261115,
which no trade decision has looked at.

Configurations (CONFIGS below):
  base        v1 as frozen: one 50 deg Boson 640 nadir camera, 30 deg search-turn bank, 18 m/s search.
  bank35/40   search-turn bank limit raised for every aircraft (an autopilot setting). Stall margin is kept:
              the minimum turn speed already scales with 1/sqrt(cos bank), so the route speed rises if needed.
  v16 / v20   Lodestar search airspeed.
  twin        two 50 deg Boson 640 cores splayed +-22.5 deg across track (95 deg total). Each keeps its own
              pixel size, so detection off nadir follows the same Johnson-criteria model; the footprint
              widens and the track spacing grows with the no-blur sweep width. +0.09 kg, +$4.1k.
  twin_bank40 both.

Run:    .venv/Scripts/python.exe -X utf8 scripts/step2_trades.py [--configs base twin ...] [--workers 7]
Writes: out/step2_trades/runs.csv.gz, summary.csv, paired_vs_base.csv, spacing.csv
"""
import argparse
import os
from concurrent.futures import ProcessPoolExecutor
import math
from pathlib import Path

import numpy as np
import pandas as pd

import terrain_sim as t
from fleet_flyable import lodestar_structure_kg
from long_pass import TUNING_SEED, adaptive_bound, route_flight
from wing_mission import N

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "step2_trades"
ROUTE = "pursuit_1"
PATTERNS = ("trails 3", "trails 6")
TWIN_EXTRA_KG = 0.09     # second Boson 640 core with 8.7 mm lens (35-71 g) plus splayed mount

CONFIGS = {
    "base": dict(),
    "bank35": dict(bank=35.0, all_aircraft=True),
    "bank40": dict(bank=40.0, all_aircraft=True),
    "v16": dict(speed=16.0),
    "v20": dict(speed=20.0),
    "twin": dict(camera="twin"),
    "twin_bank40": dict(camera="twin", bank=40.0, all_aircraft=True),
}


def sweep_width(tan_half, pixel_tan, height=t.H_SEARCH, vg=0.0):
    x = np.linspace(0.0, height * tan_half, 400)
    return 2 * float(np.trapezoid([t.camera_probability(xi, vg, height, tan_half, pixel_tan) for xi in x], x))


# Defaults captured at import. Worker processes are reused across jobs, so apply() must restore them first;
# otherwise one configuration's overrides leak into the next job in the same worker.
DEFAULTS = dict(bank=t.ms.BANK, search_v=dict(t.SEARCH_V), cameras=dict(t.CAMERAS),
                pixel=dict(t.CAMERA_PIXEL_TAN))


def reset():
    t.ms.BANK = DEFAULTS["bank"]
    t.SEARCH_V.clear(); t.SEARCH_V.update(DEFAULTS["search_v"])
    t.CAMERAS.clear(); t.CAMERAS.update(DEFAULTS["cameras"])
    t.CAMERA_PIXEL_TAN.clear(); t.CAMERA_PIXEL_TAN.update(DEFAULTS["pixel"])


def twin_camera():
    """(footprint hfov, track spacing, pixel tangent) for the splayed twin, scaled from the single camera."""
    hfov, spacing = DEFAULTS["cameras"]["Lodestar"]
    tan1 = math.tan(math.radians(hfov / 2))
    tan2 = math.tan(math.radians(hfov / 2 + 22.5))
    ratio = sweep_width(tan2, tan1) / sweep_width(tan1, tan1)
    return hfov + 45.0, round(spacing * ratio, 1), tan1


def apply(cfg):
    """Set module-level overrides inside a worker (after restoring defaults). Returns extra Lodestar mass [kg]."""
    reset()
    extra = 0.0
    if "bank" in cfg:
        t.ms.BANK = math.radians(cfg["bank"])
    if "speed" in cfg:
        for k in t.SEARCH_V:
            t.SEARCH_V[k] = cfg["speed"]
    if cfg.get("camera") == "twin":
        hfov, spacing, pixel = twin_camera()
        t.CAMERAS["Lodestar"] = (hfov, spacing)
        t.CAMERA_PIXEL_TAN["Lodestar"] = pixel
        extra += TWIN_EXTRA_KG
    return extra


def run(args):
    name, site, pattern, seed, limit = args
    cache = OUT.parent / "step2_trades_cache" / (f"{name}_{site}_{pattern.replace(' ', '')}_{seed}_{limit}"
                                                f"_por{t.CROWN_POROSITY:g}_odds{t.CAMERA_ODDS_SCALE:.4g}.pkl")
    if cache.exists():
        return pd.read_pickle(cache)
    result = _run(name, site, pattern, seed, limit)
    cache.parent.mkdir(parents=True, exist_ok=True)
    pd.to_pickle(result, cache)
    return result


def _run(name, site, pattern, seed, limit):
    cfg = CONFIGS[name]
    extra = apply(cfg)
    mass = lodestar_structure_kg("v1") + extra
    prep = t.prep(site, N, seed)
    home, hikers = prep["z_home"], prep["hikers"]
    dem = t.load(site)[0]
    cover, hide = t.cover_draws(site, hikers, seed)
    if limit:
        hikers = hikers.iloc[:limit]
    chute = pd.read_csv(t.ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")
    altitude = home + t.H_SEARCH
    lodestar = t.site_fleet(altitude, wing_mass_kg=mass, include_competitors=False)[0]
    fleet = [lodestar]
    if cfg.get("all_aircraft") or name == "base":
        fleet.append(next(a for a in t.site_fleet(altitude)[2:] if a.name == "DeltaQuad Evo GOV"))
    ref = t.site_fleet(altitude, include_competitors=False)[0]
    out, routes = [], []
    for ac in fleet:
        is_lodestar = ac.rw_cfg is not None
        ac_ref = ac if is_lodestar else ref
        legs = prep["pats"][(pattern, t.CAMERAS["Lodestar" if is_lodestar else "other"][1])]["legs"]
        flights, rows = {}, []
        for i, h in enumerate(hikers.itertuples()):
            wind = np.array([h.ws * math.cos(h.wd), h.ws * math.sin(h.wd)])
            bound = adaptive_bound(h.ws)
            if bound not in flights:
                flights[bound] = route_flight(ac, legs, dem, home, bound, altitude, ROUTE, ref=ac_ref)[0]
                r = flights[bound].route
                routes.append(dict(config=name, site=site, pattern=pattern, aircraft=ac.name, wind_bound_mps=bound,
                                   airspeed_mps=r.speed, radius_m=r.radius, path_km=r.ds.sum() / 1000,
                                   straight_search_km=flights[bound].length.sum() / 1000, end_reason=r.end_reason))
            rows.append(flights[bound].replay(h, wind, (cover[i], hide[i]), dem, home))
        runs = pd.DataFrame(rows).rename_axis("run").reset_index()
        scored = t.score(runs, hikers, lambda w: np.interp(w, chute.wind, chute.p_reach))
        out.append(scored[[*runs.columns, "ws", "help_1h", "help_3h", "t_help"]]
                   .assign(config=name, site=site, aircraft=ac.name, pattern=pattern))
    print(f"Completed {name} / {site} / {pattern}", flush=True)
    return pd.concat(out, ignore_index=True), pd.DataFrame(routes)


def paired(runs, base="base", aircraft="Lodestar SiCore"):
    rows = []
    rng = np.random.default_rng(8803)
    lod = runs[runs.aircraft == aircraft]
    for pattern, group in lod.groupby("pattern"):
        for metric in ("found", "help_1h", "help_3h"):
            scores = group.pivot_table(index=["site", "run"], columns="config", values=metric, aggfunc="first")
            for cfg in scores.columns.drop(base):
                d = (scores[cfg].astype(float) - scores[base].astype(float)).groupby("run").mean().to_numpy()
                samples = rng.integers(0, len(d), (4000, len(d)))
                lo, hi = np.quantile(d[samples].mean(axis=1), [0.025, 0.975])
                rows.append(dict(pattern=pattern, metric=metric, config=cfg, delta_pp=100 * d.mean(),
                                 low_95_pp=100 * lo, high_95_pp=100 * hi))
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configs", nargs="+", choices=list(CONFIGS), default=list(CONFIGS))
    parser.add_argument("--seed", type=int, default=TUNING_SEED)
    parser.add_argument("--sites", nargs="+", choices=t.SITES, default=list(t.SITES))
    parser.add_argument("--hikers", type=int, default=0, help="first n hikers (smoke runs); 0 = all")
    parser.add_argument("--workers", type=int, default=7)
    parser.add_argument("--tag", default="")
    args = parser.parse_args()
    if args.seed != TUNING_SEED and not args.tag:
        parser.error("trades run on the tuning seed; pass --tag to confirm on another seed on purpose")
    configs = list(dict.fromkeys(["base", *args.configs]))
    jobs = [(c, s, p, args.seed, args.hikers) for c in configs for s in args.sites for p in PATTERNS]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        runs, routes = zip(*pool.map(run, jobs))
    runs = pd.concat(runs, ignore_index=True)
    if t.CROWN_POROSITY != 0.2 or os.environ.get("LODESTAR_ODDS_SCALE"):
        args.tag += f"_por{t.CROWN_POROSITY:g}_odds{t.CAMERA_ODDS_SCALE:.3g}"
    out = OUT if not (args.tag or args.hikers) else OUT.with_name(f"{OUT.name}{args.tag}{'_smoke' if args.hikers else ''}")
    out.mkdir(parents=True, exist_ok=True)
    runs.to_csv(out / "runs.csv.gz", index=False)
    pd.concat(routes, ignore_index=True).to_csv(out / "routes.csv", index=False)
    summary = (runs.groupby(["pattern", "config", "aircraft", "site"])[["found", "help_1h", "help_3h"]].mean()
               .groupby(level=["pattern", "config", "aircraft"]).mean())
    summary.to_csv(out / "summary.csv")
    paired(runs).to_csv(out / "paired_vs_base.csv", index=False)
    apply(CONFIGS["twin"])
    pd.DataFrame([dict(camera="single 50 deg", hfov=50.0, spacing_m=CAMERAS_SINGLE[1]),
                  dict(camera="twin splayed", hfov=t.CAMERAS["Lodestar"][0], spacing_m=t.CAMERAS["Lodestar"][1])]
                 ).to_csv(out / "spacing.csv", index=False)
    with pd.option_context("display.width", 200):
        print((100 * summary).round(2).to_string())
        print(paired(runs).round(2).to_string(index=False))


CAMERAS_SINGLE = t.CAMERAS["Lodestar"]

if __name__ == "__main__":
    main()
