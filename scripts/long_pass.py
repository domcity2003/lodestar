"""Longer camera passes: fly the trails-first tour as a turn-limited, autopilot-style track.

Why: the trail tour (terrain_sim.trail_tour) follows OSM trail vertices simplified at 25 m. A
typical site has ~650 trail legs with a median heading change of 65 deg. On continuous routes
(flyable_mission) that means ~300 fillets plus ~150-340 turnaround loops per site, and the aircraft
flies about as far turning (camera off) as searching. Straightening the polyline does not fix it:
at Bear Lake, even a tolerance of the full camera half-swath (56 m) and dropping every loop-forcing
corner that tolerance allows still left 150 loops for Lodestar, because switchbacks and dead-end
spurs are real trail shape.

What: a pure-pursuit tracker (the standard path-following autopilot law) flies the trail tour at a
fixed lookahead, never turning tighter than 1.3x the route's ground radius for the wind band. The
flown track is then simplified at 10 m into legs whose corners the existing fillet builder can round,
so the aircraft follows switchbacks and spurs in smooth sweeps instead of stop-and-reverse loops.

Detection draws per pass: the replay gives every source leg an independent detection draw. A pursuit
track has many short legs, which would add independent looks at the same spot. Both the pursuit track
and a re-scored copy of the baseline (mode rounded_passes) therefore share one draw per *pass*; a new
pass starts after 90 deg of cumulative heading change or at a gap. Comparing rounded_passes with
rounded_adaptive shows how much the per-leg convention itself contributes.

Modes (all continuous, wind-adaptive routes):
  rounded_adaptive   existing trail tour, per-leg draws (baseline)
  rounded_passes     existing trail tour, per-pass draws
  pursuit_<k>        pursuit track with lookahead k x planning radius, per-pass draws

Run:
  tune:     .venv/Scripts/python.exe -X utf8 scripts/long_pass.py --tune
  held-out: .venv/Scripts/python.exe -X utf8 scripts/long_pass.py --lookahead 1 --seed 20261114

Default route for new work: route_flight(..., mode=DEFAULT_ROUTE_MODE) ("pursuit_1"). fleet_flyable.py
uses it through --route; archived modes stay selectable by name.
Writes: out/wing_trade/long_pass_<tag>/
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import math

import numpy as np
import pandas as pd

import terrain_sim as t
from flyable_mission import WIND_BOUNDS, Flight, build_route, min_turn_speed
from turn_path import ground_radius
from wing_mission import N, OUT

PATTERNS = ("trails 3", "trails 6")
STEP = 10.0             # m, tracker integration step and trail resampling
RADIUS_MARGIN = 1.3     # tracker turns no tighter than this x the route's ground radius
TRACK_EPS = 10.0        # m, simplification of the flown track into legs
PASS_TURN = math.pi / 2

# Seeds. The lookahead was tuned on TUNING_SEED draws only; HELD_OUT_SEED was evaluated once.
TUNING_SEED = t.ms.RNG_SEED     # 20261111
HELD_OUT_SEED = 20261114
assert TUNING_SEED != HELD_OUT_SEED

# Lodestar's baseline route model from 2026-11-14 on (research/2026-11-14-long-pass-routes.md):
# pursuit-tracker long camera passes at lookahead 1.0 x planning radius, one detection draw per pass.
# Archived modes stay selectable by name: "rounded_adaptive" (trail tour, per-leg draws) and
# "rounded_passes" (same tour, per-pass draws).
DEFAULT_LOOKAHEAD = 1.0
DEFAULT_ROUTE_MODE = f"pursuit_{DEFAULT_LOOKAHEAD:g}"
ARCHIVED_ROUTE_MODES = ("rounded_adaptive", "rounded_passes")


def simplify(points, eps):
    """Ramer-Douglas-Peucker with point-to-segment distance (keeps spur tips)."""
    points = np.asarray(points, float)
    keep = np.zeros(len(points), bool)
    keep[[0, -1]] = True
    stack = [(0, len(points) - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        a, b = points[i], points[j]
        ab = b - a
        q = points[i + 1:j] - a
        length2 = float(ab @ ab)
        s = np.clip(q @ ab / length2, 0, 1) if length2 > 0 else np.zeros(len(q))
        d = np.hypot(*(q - s[:, None] * ab).T)
        k = int(np.argmax(d))
        if d[k] > eps:
            m = i + 1 + k
            keep[m] = True
            stack += [(i, m), (m, j)]
    return points[keep]


def resample(points, step):
    seg = np.hypot(*np.diff(points, axis=0).T)
    s = np.r_[0, np.cumsum(seg)]
    grid = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(grid, s, points[:, 0]), np.interp(grid, s, points[:, 1])])


def pursue(points, radius, lookahead):
    """Pure-pursuit track along a polyline: steer at the point `lookahead` ahead of the nearest
    path point (searched forward only), turn rate limited to STEP / radius per step."""
    path = resample(points, STEP)
    ahead = max(1, int(round(lookahead / STEP)))
    window = 3 * ahead + 10
    last = len(path) - 1
    pos = path[0].copy()
    target = path[min(ahead, last)]
    heading = math.atan2(target[1] - pos[1], target[0] - pos[0])
    max_turn = STEP / radius
    track, j, closest = [pos.copy()], 0, math.inf
    for _ in range(4 * len(path)):
        span = path[j:min(j + window, last + 1)]
        j += int(np.argmin(np.hypot(*(span - pos).T)))
        if j >= last - 1:
            # Stop at the end, or once past the closest approach a turn-limited aircraft can make.
            gap = math.dist(pos, path[-1])
            if gap < STEP or gap > closest:
                break
            closest = gap
        target = path[min(j + ahead, last)]
        error = (math.atan2(target[1] - pos[1], target[0] - pos[0]) - heading + math.pi) % math.tau - math.pi
        heading += max(-max_turn, min(max_turn, error))
        pos = pos + STEP * np.array([math.cos(heading), math.sin(heading)])
        track.append(pos.copy())
    else:
        # Hitting the cap used to truncate the trail silently. QA 2026-11-14: never reached on any
        # archived site, pattern, speed (14-18.8 m/s) or wind band, so archived routes are unchanged.
        raise RuntimeError(f"pursuit tracker hit its {4 * len(path)}-step cap {math.dist(pos, path[-1]):.0f} m "
                           "short of the trail end")
    return np.array(track), path


def tour_points(legs):
    home = next(i for i, (_, b) in enumerate(legs) if b == (0.0, 0.0))
    return home, np.array([legs[0][0], *[b for _, b in legs[:home]]], float)


def pursuit_legs(legs, radius, lookahead):
    """Replace the trail tour with the simplified pursuit track; keep return leg and expanding square."""
    home, points = tour_points(legs)
    track, path = pursue(points, RADIUS_MARGIN * radius, lookahead * radius)
    pts = simplify(track, TRACK_EPS)
    tour = [(tuple(a), tuple(b)) for a, b in zip(pts[:-1], pts[1:]) if math.dist(a, b) > 1]
    # Trail coverage: distance from each resampled trail point to the flown track.
    flown = resample(track, STEP)
    from scipy.spatial import cKDTree
    miss = cKDTree(flown).query(path)[0]
    return tour + [(tour[-1][1], legs[home][1])] + list(legs[home + 1:]), miss


def pass_ids(legs):
    """One detection draw per pass: new pass at a gap or after 90 deg of cumulative turning."""
    ids, current, turned, previous = [], 0, 0.0, None
    for k, (a, b) in enumerate(legs):
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        if previous is not None:
            gap = math.dist(legs[k - 1][1], a) > 1e-6
            turned += abs((heading - previous + math.pi) % math.tau - math.pi)
            if gap or turned > PASS_TURN:
                current, turned = current + 1, 0.0
        ids.append(current)
        previous = heading
    return np.array(ids)


def lookahead_of(mode):
    """Pursuit lookahead (x planning radius) for a route mode name, or None for the trail-tour modes."""
    if mode in ARCHIVED_ROUTE_MODES:
        return None
    if not mode.startswith("pursuit_"):
        raise ValueError(f"unknown route mode {mode!r}; use {ARCHIVED_ROUTE_MODES} or pursuit_<k>")
    k = float(mode.removeprefix("pursuit_"))
    if not k > 0:
        raise ValueError(f"pursuit lookahead must be positive, got {mode!r}")
    return k


def adaptive_bound(wind_speed):
    """Wind-adaptive planning band: the next enclosing bound from WIND_BOUNDS."""
    return next(b for b in WIND_BOUNDS if wind_speed <= b)


# True from Step 2 on; verify_route_modes.py sets False to reproduce the archived runs.
CONVERGED_PLANNING_RADIUS = True


def route_flight(ac, base_legs, dem, home, bound, altitude, mode=DEFAULT_ROUTE_MODE, ref=None):
    """Build one aircraft's continuous route for a wind band and route mode, ready to replay.

    base_legs is the trails-first pattern (prep()["pats"][...]["legs"]) for the aircraft's camera.
    Returns (Flight, legs actually flown, trail offset from the flown track [m]; zeros for trail modes).
    Every fleet or mission script should get its routes here so the route model stays in one place.
    """
    k = lookahead_of(mode)
    miss = np.zeros(1)
    if k is None:
        legs = base_legs
    else:
        speed = max(ac.v_cruise, min_turn_speed(ac, altitude))
        for _ in range(6):
            legs, miss = pursuit_legs(base_legs, ground_radius(speed, bound, t.ms.BANK, t.ms.G), k)
            legs = legs[:t.sp.MAX_LEGS]
            route = build_route(ac, legs, dem, home, bound, altitude, ref=ref)
            # Mission-sim QA open issue 1: the archived runs planned the tracker radius from the
            # pre-convergence speed. From Step 2 on, replan until the planning speed covers the route speed.
            if not CONVERGED_PLANNING_RADIUS or route.speed <= speed + 1e-9:
                break
            speed = route.speed
        else:
            raise ValueError("pursuit planning radius did not converge")
    if k is None:
        legs = legs[:t.sp.MAX_LEGS]
        route = build_route(ac, legs, dem, home, bound, altitude, ref=ref)
    if mode != "rounded_adaptive":
        ids = pass_ids(legs)
        route.pieces = [p if p.source is None else replace(p, source=int(ids[p.source]))
                        for p in route.pieces]
    return Flight(ac, route, ref=ref), legs, miss


def compare_site(args):
    site, mass, pattern, lookaheads, seed, names = args
    prep = t.prep(site, N, seed)
    home, hikers = prep["z_home"], prep["hikers"]
    dem = t.load(site)[0]
    cover, hide = t.cover_draws(site, hikers, seed)
    chute = pd.read_csv(t.ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")
    altitude = home + t.H_SEARCH
    fleet = [*t.site_fleet(altitude, wing_mass_kg=mass, include_competitors=False),
             *t.site_fleet(altitude)[2:]]
    ref = t.site_fleet(altitude, include_competitors=False)[0]
    results, routes = [], []
    for ac in fleet:
        if names and ac.name not in names:
            continue
        is_lodestar = ac.rw_cfg is not None
        ac_ref = ac if is_lodestar else ref
        half_swath = t.H_SEARCH * t.camera_tangent(ac)
        base = prep["pats"][(pattern, t.CAMERAS["Lodestar" if is_lodestar else "other"][1])]["legs"]
        for mode in [*ARCHIVED_ROUTE_MODES, *(f"pursuit_{k:g}" for k in lookaheads)]:
            flights, rows = {}, []
            for i, h in enumerate(hikers.itertuples()):
                wind = np.array([h.ws * math.cos(h.wd), h.ws * math.sin(h.wd)])
                bound = adaptive_bound(h.ws)
                if bound not in flights:
                    flights[bound], legs, miss = route_flight(ac, base, dem, home, bound, altitude, mode, ac_ref)
                    route = flights[bound].route
                    routes.append(dict(site=site, pattern=pattern, aircraft=ac.name, mode=mode,
                                       wind_bound_mps=bound, legs=len(legs), airspeed_mps=route.speed,
                                       radius_m=route.radius, fillets=route.fillets, loops=route.loops,
                                       path_km=route.ds.sum() / 1000,
                                       straight_search_km=flights[bound].length.sum() / 1000,
                                       passes=len(set(flights[bound].source)),
                                       trail_in_frame=float((miss <= half_swath).mean()),
                                       trail_offset_p95_m=float(np.percentile(miss, 95)),
                                       end_reason=route.end_reason))
                rows.append(flights[bound].replay(h, wind, (cover[i], hide[i]), dem, home))
            runs = pd.DataFrame(rows).rename_axis("run").reset_index()
            scored = t.score(runs, hikers, lambda w: np.interp(w, chute.wind, chute.p_reach))
            scored = scored[[*runs.columns, "ws", "help_1h", "help_3h"]]
            results.append(scored.assign(site=site, aircraft=ac.name, pattern=pattern, mode=mode))
    print(f"Completed {site} / {pattern}", flush=True)
    return pd.concat(results, ignore_index=True), pd.DataFrame(routes)


def paired(runs, reference):
    """Each mode minus `reference` per aircraft on matched runs; run IDs bootstrapped across sites."""
    rows = []
    rng = np.random.default_rng(8803)
    for (pattern, aircraft), group in runs.groupby(["pattern", "aircraft"]):
        for metric in ("help_1h", "help_3h"):
            scores = group.pivot(index=["site", "run"], columns="mode", values=metric).astype(float)
            for mode in scores.columns.drop(reference):
                d = (scores[mode] - scores[reference]).groupby("run").mean().to_numpy()
                samples = rng.integers(0, len(d), (4000, len(d)))
                low, high = np.quantile(d[samples].mean(axis=1), [0.025, 0.975])
                rows.append(dict(pattern=pattern, aircraft=aircraft, metric=metric, mode=mode, reference=reference,
                                 delta_pp=100 * d.mean(), low_95_pp=100 * low, high_95_pp=100 * high))
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tune", action="store_true",
                        help="trails 6, SiCore and Evo GOV, lookahead 0.5/1/2, original seed")
    parser.add_argument("--lookahead", nargs="+", type=float, default=[DEFAULT_LOOKAHEAD])
    parser.add_argument("--seed", type=int, default=TUNING_SEED)
    parser.add_argument("--workers", type=int, default=7)
    args = parser.parse_args()
    mass = pd.read_csv(OUT / "summary.csv").set_index("design").wing_mass_kg["compact_xps"]
    if args.tune:
        patterns, lookaheads, names, seed, tag = ("trails 6",), (0.5, 1.0, 2.0), \
            ("Lodestar SiCore", "DeltaQuad Evo GOV"), TUNING_SEED, "tune"
    else:
        patterns, lookaheads, names, seed = PATTERNS, tuple(args.lookahead), (), args.seed
        tag = f"seed_{seed}"
    jobs = [(site, mass, p, lookaheads, seed, names) for site in t.SITES for p in patterns]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        runs, routes = zip(*pool.map(compare_site, jobs))
    runs = pd.concat(runs, ignore_index=True)
    output = OUT / f"long_pass_{tag}"
    output.mkdir(exist_ok=True)
    runs.to_csv(output / "runs.csv.gz", index=False)
    pd.concat(routes, ignore_index=True).to_csv(output / "routes.csv", index=False)
    metrics = ["found", "help_1h", "help_3h", "searched_km"]
    by_site = runs.groupby(["pattern", "mode", "aircraft", "site"])[metrics].mean()
    by_site.to_csv(output / "by_site.csv")
    summary = by_site.groupby(level=["pattern", "mode", "aircraft"]).mean()
    summary.to_csv(output / "summary.csv")
    pd.concat([paired(runs, "rounded_adaptive"), paired(runs, "rounded_passes")]).to_csv(
        output / "paired.csv", index=False)
    print(summary.round(4).to_string(), flush=True)


if __name__ == "__main__":
    main()
