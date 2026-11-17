"""Fleet comparison on continuous, wind-adaptive routes: Lodestar (v1 by default) against the competitors.

Every aircraft flies the same trails-first pattern for its own camera's track spacing, with fillets and
turnarounds sized to its own route airspeed and the scenario's wind band (flyable_mission.py). The
sharp-corner replay is kept as a control that must reproduce the archived terrain run.

Competitor modelling (no mass or drag model published):
- power curve: mission_sim's normalised curve, with induced power = half of cruise power at cruise speed;
  bank raises induced power by tan^2(bank), the same rule Lodestar uses;
- climb: Lodestar's climb-energy to cruise-power ratio (terrain_sim.climb_k), as in the archived run;
- turn speed floor: published-style minimum speed (0.8 x cruise) treated as a sea-level value and raised
  for density and 30 deg bank, the same 1.25 x stall convention Lodestar's v_min uses;
- no rescue kit.

Route model: --route picks the continuous route (long_pass.route_flight). The default is Lodestar's
baseline from 2026-11-14 on, long_pass.DEFAULT_ROUTE_MODE (pursuit tracker, per-pass draws). The archived
fleet run used --route rounded_adaptive; only that route writes to the archived folder names.

Run:  .venv/Scripts/python.exe -X utf8 scripts/fleet_flyable.py [--route rounded_adaptive] [--endurance 0.8] [--seed S]
Writes: out/wing_trade/flyable_fleet[_<route>][_<design>][_endurance_<f>][_seed_<s>]/
        (no _<route> suffix for rounded_adaptive, no _<design> suffix for compact_xps, no _seed suffix for
        the tuning seed). The v1 runs use the Step 2 physics fixes in flyable_mission and long_pass.
"""
import argparse
import os
from concurrent.futures import ProcessPoolExecutor
import math

import numpy as np
import pandas as pd

import terrain_sim as t
from long_pass import DEFAULT_ROUTE_MODE, TUNING_SEED, adaptive_bound, lookahead_of, route_flight
from wing_mission import N, OUT


def lodestar_structure_kg(design):
    """Wing mass passed to the Lodestar power model.

    v1 (Step 2 on): the frozen v1 wing (out/wing_trade/summary_v1.csv) plus the v1 added masses (pack
    thermal kit, two elevon balances), carried as extra structure as in wing_trade.py's performance table.
    compact_xps: the archived 2026-11-13/14 runs.
    """
    if design in ("v1", "v1.1"):
        import lodestar_v1 as v1
        # v1 keeps the v1.0 freeze wing (snapshot taken before the v1.1 wing_trade rerun) so archives reproduce.
        src = OUT / ("v1.0_freeze/summary_v1.csv" if design == "v1" else "summary_v1.csv")
        wing = pd.read_csv(src).set_index("design").wing_mass_kg["v1"]
        added = v1.ADDED_MASS["pack_thermal_kit_kg"] + 2 * v1.ADDED_MASS["elevon_balance_kg_each"]
        if design == "v1.1":
            # Step 2 changes (lodestar_v1.V11_MASS), carried as extra structure like the v1 added masses.
            added += sum(m for m, *_ in v1.V11_MASS.values())
        return float(wing + added)
    return float(pd.read_csv(OUT / "summary.csv").set_index("design").wing_mass_kg[design])

MODES = ("sharp", "rounded_adaptive")   # archived run (verify_fleet_flyable.py); --route replaces the second
PATTERNS = ("trails 3", "trails 6")


def apply_design(design):
    """Module-level settings for a design. v1.1: twin splayed camera, 40 deg search bank for every aircraft,
    skid + strobe drag. Runs inside each worker (Windows workers start fresh, so main's state is not inherited)."""
    if design != "v1.1":
        return
    import lodestar_v1 as v1
    c = v1.CAMERA_V11
    t.CAMERAS["Lodestar"] = (c["hfov_deg"], c["track_spacing_m"])
    t.CAMERA_PIXEL_TAN["Lodestar"] = c["pixel_tan"]
    t.ms.BANK = math.radians(c["search_bank_deg"])
    t.v4u.apply_v11()   # body drag increment, once per process


def compare_site(args):
    site, mass, pattern, endurance, route_mode, seed, limit, design = args   # limit: first n hikers (smoke runs)
    apply_design(design)
    prep = t.prep(site, N, seed)
    home, hikers = prep["z_home"], prep["hikers"]
    dem = t.load(site)[0]
    cover, hide = t.cover_draws(site, hikers, seed)
    if limit is not None:
        hikers = hikers.iloc[:limit]
    chute = pd.read_csv(t.ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")
    altitude = home + t.H_SEARCH
    lodestar = t.site_fleet(altitude, wing_mass_kg=mass, include_competitors=False)
    others = t.site_fleet(altitude, competitor_endurance=endurance)[2:]
    # The archived run scaled competitor climb cost from the original-mass SiCore.
    ref = t.site_fleet(altitude, include_competitors=False)[0]
    results, routes = [], []
    for ac in [*lodestar, *others]:
        is_lodestar = ac.rw_cfg is not None
        ac_ref = ac if is_lodestar else ref
        path = prep["pats"][(pattern, t.CAMERAS["Lodestar" if is_lodestar else "other"][1])]
        climb, tan = t.climb_k(ac, ac_ref), t.camera_tangent(ac)
        flights = {}
        for mode in ("sharp", route_mode):
            rows = []
            for i, h in enumerate(hikers.itertuples()):
                wind = np.array([h.ws * math.cos(h.wd), h.ws * math.sin(h.wd)])
                canopy = (cover[i], hide[i])
                if mode == "sharp":
                    row = t.fly(ac, path, (h.tx, h.ty), h.z, wind, h.draws, climb, tan, home, dem, canopy) \
                        if h.on_map else dict(found=False, t_find=None, t_kit=None, h_agl=None)
                else:
                    bound = adaptive_bound(h.ws)
                    if bound not in flights:
                        flights[bound] = route_flight(ac, path["legs"], dem, home, bound, altitude,
                                                      route_mode, ref=ac_ref)[0]
                        route = flights[bound].route
                        routes.append(dict(site=site, pattern=pattern, aircraft=ac.name, route=route_mode,
                                           wind_bound_mps=bound, airspeed_mps=route.speed, radius_m=route.radius,
                                           path_km=route.ds.sum() / 1000,
                                           straight_search_km=flights[bound].length.sum() / 1000,
                                           end_reason=route.end_reason))
                    row = flights[bound].replay(h, wind, canopy, dem, home)
                rows.append(row)
            runs = pd.DataFrame(rows).rename_axis("run").reset_index()
            scored = t.score(runs, hikers, lambda w: np.interp(w, chute.wind, chute.p_reach))
            scored = scored[[*runs.columns, "ws", "help_1h", "help_3h"]]
            results.append(scored.assign(site=site, aircraft=ac.name, pattern=pattern, mode=mode))
    print(f"Completed {site} / {pattern}", flush=True)
    return pd.concat(results, ignore_index=True), pd.DataFrame(routes)


def paired_vs(runs, reference="Lodestar SiCore"):
    """Each aircraft minus the reference on matched hiker runs; bootstrap run IDs across the seven sites."""
    rows = []
    rng = np.random.default_rng(8802)
    for (pattern, mode), group in runs.groupby(["pattern", "mode"]):
        for metric in ("help_1h", "help_3h"):
            scores = group.pivot(index=["site", "run"], columns="aircraft", values=metric).astype(float)
            for ac in scores.columns.drop(reference):
                paired = (scores[ac] - scores[reference]).groupby("run").mean().to_numpy()
                samples = rng.integers(0, len(paired), (4000, len(paired)))
                low, high = np.quantile(paired[samples].mean(axis=1), [0.025, 0.975])
                rows.append(dict(pattern=pattern, mode=mode, metric=metric, aircraft=ac, reference=reference,
                                 delta_pp=100 * paired.mean(), low_95_pp=100 * low, high_95_pp=100 * high))
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endurance", type=float, default=1.0, help="competitor published-endurance factor")
    parser.add_argument("--route", default=DEFAULT_ROUTE_MODE,
                        help=f"route mode: pursuit_<k> (default {DEFAULT_ROUTE_MODE}), rounded_adaptive (archived), "
                             "rounded_passes")
    parser.add_argument("--seed", type=int, default=TUNING_SEED,
                        help=f"hiker/wind/detection seed (tuning {TUNING_SEED}; held-out 20261114)")
    parser.add_argument("--sites", nargs="+", choices=t.SITES, default=list(t.SITES))
    parser.add_argument("--patterns", nargs="+", choices=PATTERNS, default=list(PATTERNS))
    parser.add_argument("--hikers", type=int, default=N, help=f"first n of the {N} cached hikers (smoke runs)")
    parser.add_argument("--workers", type=int, default=7)
    parser.add_argument("--design", default="v1", choices=("v1", "v1.1", "compact_xps"),
                        help="Lodestar wing (v1 from Step 2 on; compact_xps reproduces the archived runs)")
    args = parser.parse_args()
    if not 1 <= args.hikers <= N:
        parser.error(f"hikers must be in 1..{N}")
    lookahead_of(args.route)   # validate the name before starting workers
    mass = lodestar_structure_kg(args.design)
    print(f"Lodestar {args.design}: structure mass {mass:.4f} kg", flush=True)
    jobs = [(site, mass, pattern, args.endurance, args.route, args.seed, None if args.hikers == N else args.hikers,
             args.design)
            for site in dict.fromkeys(args.sites) for pattern in dict.fromkeys(args.patterns)]
    with ProcessPoolExecutor(max_workers=min(len(jobs), args.workers)) as pool:
        runs, routes = zip(*pool.map(compare_site, jobs))
    runs = pd.concat(runs, ignore_index=True)
    output = OUT / ("flyable_fleet" if args.route == "rounded_adaptive" else f"flyable_fleet_{args.route}")
    if args.design != "compact_xps":
        output = output.with_name(f"{output.name}_{args.design}")
    if args.endurance != 1.0:
        output = output.with_name(f"{output.name}_endurance_{args.endurance:g}")
    if args.seed != TUNING_SEED:
        output = output.with_name(f"{output.name}_seed_{args.seed}")
    if t.CROWN_POROSITY != 0.2:
        output = output.with_name(f"{output.name}_porosity_{t.CROWN_POROSITY:g}")
    if os.environ.get("LODESTAR_ODDS_SCALE"):
        output = output.with_name(f"{output.name}_odds_{t.CAMERA_ODDS_SCALE:g}")
    if args.hikers != N or set(args.sites) != set(t.SITES) or set(args.patterns) != set(PATTERNS):
        output = output.with_name(f"{output.name}_subset")   # never overwrite a full run with a subset
    output.mkdir(exist_ok=True)
    runs.to_csv(output / "runs.csv.gz", index=False)
    pd.concat(routes, ignore_index=True).to_csv(output / "routes.csv", index=False)
    metrics = ["found", "help_1h", "help_3h", "searched_km"]
    by_site = runs.groupby(["pattern", "mode", "aircraft", "site"])[metrics].mean()
    by_site.to_csv(output / "by_site.csv")
    summary = by_site.groupby(level=["pattern", "mode", "aircraft"]).mean()
    summary.to_csv(output / "summary.csv")
    paired_vs(runs).to_csv(output / "paired_vs_sicore.csv", index=False)
    # Inputs that produced this run, so freeze_check can tie mission results to the frozen design.
    apply_design(args.design)
    meta = dict(design=args.design, structure_kg=mass, seed=args.seed, route=args.route,
                competitor_endurance=args.endurance, crown_porosity=t.CROWN_POROSITY,
                camera_odds_scale=t.CAMERA_ODDS_SCALE, bank_deg=math.degrees(t.ms.BANK),
                lodestar_camera=list(t.CAMERAS["Lodestar"]), cda_body_m2=t.ms.d2.CDA_BODY)
    import json
    (output / "meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    print(summary.round(4).to_string(), flush=True)


if __name__ == "__main__":
    main()
