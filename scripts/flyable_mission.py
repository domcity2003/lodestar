"""Compare compact-XPS search routes with explicit bounded-curvature flight paths."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
import math

import aerosandbox as asb
import numpy as np
import pandas as pd

import terrain_sim as t
from turn_path import ground_radius, rounded_path
from wing_mission import N, OUT, PATTERN


STEP = t.CELL / 2
WIND_BOUNDS = (2, 4, 6, 8, 10)
MODES = ("sharp", "rounded_fixed", "rounded_adaptive")


@dataclass
class Route:
    pieces: list
    offsets: np.ndarray
    tangents: np.ndarray
    ds: np.ndarray
    altitude: np.ndarray
    dz: np.ndarray
    curvature: np.ndarray
    search: np.ndarray
    return_ascent: np.ndarray
    speed: float
    radius: float
    fillets: int
    loops: int
    end_reason: str


def in_map(points):
    return np.all(np.abs(points) <= t.HALF - t.CELL / 2)


def return_ascents(dem, ends, altitude, home):
    """Batch the same direct-return terrain envelope used by terrain_sim."""
    distances = np.linalg.norm(ends, axis=1)
    intervals = np.maximum(1, np.ceil(distances / t.CELL).astype(int))
    result = np.empty(len(ends))
    for n in np.unique(intervals):
        indices = np.flatnonzero(intervals == n)
        fraction = np.linspace(0, 1, n + 1)
        xy = ends[indices, None, :] * (1 - fraction[None, :, None])
        distance = distances[indices, None] * fraction
        minimum = t.dem_at(dem, xy[:, :, 0], xy[:, :, 1]) + t.H_SEARCH
        minimum[:, 0] = np.maximum(minimum[:, 0], altitude[indices])
        minimum[:, -1] = np.maximum(minimum[:, -1], home + t.H_SEARCH)
        climb = minimum - t.CLIMB_GRAD * distance
        descend = minimum + t.DESC_GRAD * distance
        altitude_path = np.maximum(np.maximum.accumulate(climb[:, ::-1], axis=1)[:, ::-1] + t.CLIMB_GRAD * distance,
                                   np.maximum.accumulate(descend, axis=1) - t.DESC_GRAD * distance)
        result[indices] = (np.maximum(altitude_path[:, 0] - altitude[indices], 0)
                           + np.maximum(np.diff(altitude_path, axis=1), 0).sum(axis=1))
    at_home = distances == 0
    result[at_home] = np.maximum(home + t.H_SEARCH - altitude[at_home], 0)
    return result


def terrain_route(legs, dem, home, speed, wind_bound, max_distance=math.inf):
    radius = ground_radius(speed, wind_bound, t.ms.BANK, t.ms.G)
    # An unmapped waypoint ends the usable route. Do not build the remaining
    # expanding square or let it trim the preceding in-map straight.
    limit = next((i for i, leg in enumerate(legs) if not in_map(leg)), len(legs))
    end_reason = "map edge" if limit < len(legs) else "route end"
    pieces, fillets, loops = rounded_path(legs[:limit], radius)
    # Include transit to the first search point and its heading change if needed.
    if math.hypot(*pieces[0].start) > 1:
        transit = ((0.0, 0.0), tuple(pieces[0].start))
        pieces, fillets, loops = rounded_path([transit, *legs[:limit]], radius)
        pieces = [replace(p, source=p.source - 1 if p.source not in (0, None) else None) for p in pieces]
    points, tangents, steps, curvature = [], [], [], []
    travelled = 0.0
    for piece in pieces:
        xy, direction, distance = piece.sample(STEP)
        # Bound the complete arc as well as its sampled coordinates.
        padding = 0 if piece.center is None else radius * (1 - math.cos(STEP / (2 * radius)))
        if not np.all(np.abs(xy) + padding <= t.HALF - t.CELL / 2):
            end_reason = "map edge"
            break
        points.append(xy[:-1])
        tangents.append(direction)
        steps.append(np.diff(distance))
        curvature.append(np.full(len(direction), 0 if piece.center is None else 1 / radius))
        travelled += distance[-1]
        if travelled > max_distance:
            end_reason = "energy envelope"
            break
    count = len(points)
    if not count:
        raise ValueError("first path segment lies outside the terrain grid")
    last = pieces[count - 1].end
    xy = np.vstack([*points, last])
    offsets = np.cumsum([0, *map(len, steps)])
    ds = np.concatenate(steps)
    altitude = t.clearance_profile(t.dem_at(dem, *xy.T) + t.H_SEARCH, np.cumsum(np.r_[0, ds]))
    retained = pieces[:count]
    search = np.array([i for i, p in enumerate(retained) if p.source is not None], dtype=int)
    return_ascent = return_ascents(dem, xy[offsets[1:]], altitude[offsets[1:]], home)
    return Route(retained, offsets, np.vstack(tangents), ds, altitude, np.diff(altitude),
                 np.concatenate(curvature), search, return_ascent, speed, radius, fillets, loops,
                 end_reason)


# Mission-sim QA open issue 2 (2026-11-14): straight-flight power was referenced to the site density for
# the whole route. True from Step 2 on; the archived-run reproduction (verify_route_modes.py) sets False.
STRAIGHT_POWER_LOCAL_DENSITY = True


def min_turn_speed(ac, altitude):
    density = float(asb.Atmosphere(altitude=altitude).density())
    # Competitor minimum speeds are published-style sea-level values.
    reference = 1.225 if ac.rw_cfg is None else ac.rw_cfg["rho"]
    return ac.v_min * math.sqrt(reference / (density * math.cos(t.ms.BANK)))


def build_route(ac, legs, dem, home, wind_bound, initial_altitude, ref=None):
    # A route-wide speed avoids acceleration transients. Recheck at the new path's
    # highest altitude in case loops introduce a new terrain constraint.
    speed = max(ac.v_cruise, min_turn_speed(ac, initial_altitude))
    for _ in range(12):
        if speed > ac.v_max:
            raise ValueError(f"route requires {speed:.2f} m/s above {ac.v_max:g} m/s limit")
        # A sortie cannot reach beyond this distance even with all usable energy
        # spent outbound, direct tailwind, zero turn drag and full descent credit.
        descent_credit = t.climb_k(ac, ref or ac) * max(0, home - float(np.min(dem)) - t.H_SEARCH)
        max_distance = (ac.energy * (1 - t.ms.RESERVE) + descent_credit) * 3600 * (
            speed + wind_bound) / float(ac.power(speed))
        route = terrain_route(legs, dem, home, speed, wind_bound, max_distance)
        needed = min_turn_speed(ac, route.altitude.max())
        if needed <= speed:
            return route
        # Round upward to avoid needless rebuilds caused by tiny changes in radius.
        speed = math.ceil(needed * 100) / 100
    raise ValueError("route speed and terrain envelope did not converge")


def ground_kinematics(speed, tangents, curvature, wind):
    along = tangents @ wind
    across = tangents[:, 0] * wind[1] - tangents[:, 1] * wind[0]
    air_along = np.sqrt(speed**2 - across**2)
    gs = along + air_along
    # d(air heading)/d(ground heading) = ground speed / projected airspeed.
    bank_tan = speed * curvature * gs**2 / (t.ms.G * air_along)
    return gs, bank_tan


class Flight:
    """Reusable path costs and straight-leg geometry for one aircraft and wind draw."""
    def __init__(self, ac, route, ref=None):
        self.route = route
        self.ac = ac
        # Competitors have no mass model; climb_k scales Lodestar's (ref's) climb cost.
        self.k_climb = t.climb_k(ac, ref or ac)
        self.tan_half = t.camera_tangent(ac)
        self.pixel_tan = t.camera_pixel_tangent(ac)
        self.budget = ac.energy * (1 - t.ms.RESERVE)
        self.power_ref = float(ac.power(route.speed))
        self.idle = t.ms.sz.A.p_avionics_w if ac.rw_cfg is not None else 0.0
        self.ends = np.stack([p.end for p in route.pieces])
        self.start = np.stack([route.pieces[i].start for i in route.search])
        self.vector = self.ends[route.search] - self.start
        self.length = np.linalg.norm(self.vector, axis=1)
        self.direction = self.vector / self.length[:, None]
        self.source = np.array([route.pieces[i].source for i in route.search], dtype=int)
        self.home_range = np.linalg.norm(self.ends, axis=1)
        self.home_direction = -self.ends / np.maximum(self.home_range[:, None], 1e-12)
        # Turn energy uses the local induced-power increment with constant cruise
        # efficiency. The straight power table is still referenced to site altitude.
        c = ac.rw_cfg
        rho = np.asarray(asb.Atmosphere(altitude=(route.altitude[:-1] + route.altitude[1:]) / 2).density())
        rho_pts = np.asarray(asb.Atmosphere(altitude=route.altitude).density())
        rho_ref = c["rho"] if c is not None else getattr(ac, "rho_ref", None)
        if c is None:
            # Competitor curve: induced power is half of cruise power at cruise speed, scaling as 1/V.
            induced_ref = 0.5 * ac.p_cruise * ac.v_cruise / route.speed
            fixed = 0.0
            self.induced_power = np.full(len(route.ds), induced_ref)
        else:
            induced_ref = (ac.mass * t.ms.G)**2 / (
                0.5 * c["rho"] * route.speed * c["S"] * math.pi * c["e"] * c["AR"] * t.ms.sz.A.eta_cruise)
            fixed = t.ms.sz.A.p_avionics_w   # the power table includes avionics; cfg p_av may be 0
            self.induced_power = (ac.mass * t.ms.G)**2 / (
                0.5 * rho * route.speed * c["S"] * math.pi * c["e"] * c["AR"] * t.ms.sz.A.eta_cruise)
        if STRAIGHT_POWER_LOCAL_DENSITY and rho_ref is not None:
            # Straight-flight power at the local density and constant airspeed: parasite power scales
            # with density, induced power inversely, avionics not at all (mission-sim QA open issue 2).
            # Competitors get the same split of their published-style curve, and their turn induced
            # power is corrected the same way.
            parasite_ref = self.power_ref - induced_ref - fixed

            def straight(r):
                return fixed + parasite_ref * r + induced_ref / r
            self.power = straight(rho / rho_ref)              # per sample interval
            self.power_pts = straight(rho_pts / rho_ref)      # at each sample point
            if c is None:
                self.induced_power = induced_ref * rho_ref / rho
        else:
            self.power = self.power_ref
            self.power_pts = np.full(len(route.altitude), self.power_ref)

    def profile(self, wind, home):
        """Time and energy along the sampled route for one steady wind (QA hook; replay uses it).

        Returns gs, dt, de per sample interval; time [s] and energy [Wh] at each sample point from
        launch; and the return-home energy [Wh] at each piece end plus the full-circle allowance turn_wh.
        """
        route = self.route
        gs, bank_tan = ground_kinematics(route.speed, route.tangents, route.curvature, wind)
        # Written so NaN fails too: a crosswind above airspeed gives NaN ground speed, which the
        # original `(gs <= 1).any()` test let through silently (QA 2026-11-14; no archived run affected).
        if not (np.all(bank_tan <= math.tan(t.ms.BANK) + 1e-9) and np.all(gs > 1)):
            raise ValueError("wind and route violate the planned bank or groundspeed envelope")
        dt = route.ds / gs
        de = np.maximum((self.power + self.induced_power * bank_tan**2) * dt / 3600
                        + self.k_climb * route.dz, self.idle * dt / 3600)
        rise = max(0, route.altitude[0] - home)
        launch_time = rise / (t.CLIMB_GRAD * route.speed)
        time = launch_time + np.r_[0, dt.cumsum()]
        energy = self.k_climb * rise + self.power_pts[0] * launch_time / 3600 + np.r_[0, de.cumsum()]
        # Direct return energy plus a full-circle maneuver allowance. The return
        # corridor itself is still represented by a straight terrain profile.
        turn_wh = 2 * math.pi * route.speed / (t.ms.G * math.tan(t.ms.BANK)) * (
            self.power_pts.max() + self.induced_power.max() * math.tan(t.ms.BANK)**2) / 3600
        home_gs = t.ms.ground_speed(route.speed, wind, self.home_direction.T)
        home_wh = (self.power_pts[route.offsets[1:]] * self.home_range / home_gs / 3600
                   + self.k_climb * route.return_ascent)
        if not np.all(np.isfinite(home_wh)):
            raise ValueError("wind exceeds the airspeed on a return heading")
        return dict(gs=gs, bank_tan=bank_tan, dt=dt, de=de, time=time, energy=energy,
                    home_wh=home_wh, turn_wh=turn_wh)

    def replay(self, hiker, wind, cover, dem, home):
        miss = dict(found=False, t_find=None, t_kit=None, h_agl=None, source=None)
        if not hiker.on_map:
            return dict(miss, cutoff="off map", searched_km=0)
        route, ac = self.route, self.ac
        p = self.profile(wind, home)
        gs, dt, de, time, energy = p["gs"], p["dt"], p["de"], p["time"], p["energy"]
        home_wh, turn_wh = p["home_wh"], p["turn_wh"]
        offsets = route.offsets
        exhausted = np.flatnonzero(energy[offsets[1:]] + home_wh + turn_wh > self.budget)
        stop = int(exhausted[0]) if len(exhausted) else len(route.pieces)
        count = np.searchsorted(route.search, stop)
        searched_km = self.length[:count].sum() / 1000

        target = np.array([hiker.tx, hiker.ty])
        delta = target - self.start[:count]
        along = (delta * self.direction[:count]).sum(axis=1)
        cross = np.abs(delta[:, 0] * self.direction[:count, 1] - delta[:, 1] * self.direction[:count, 0])
        candidates = np.flatnonzero((along >= 0) & (along <= self.length[:count]) & (
            cross < self.tan_half * max(route.altitude.max() - hiker.z, 1)))
        for i in candidates:
            k = route.search[i]
            a, b = offsets[k:k + 2]
            fraction = along[i] / self.length[i]
            sample = a + fraction * (b - a)
            lo = min(int(sample), b - 1)
            f = sample - lo
            altitude = route.altitude[lo] + f * route.dz[lo]
            hag = max(altitude - hiker.z, 1)
            if hiker.draws[self.source[i]] >= t.camera_probability(cross[i], gs[lo], hag, self.tan_half, self.pixel_tan):
                continue
            if cover[1] >= t.canopy_clear(cover[0], hag / math.hypot(hag, cross[i])):
                continue
            here = self.start[i] + self.direction[i] * along[i]
            if not t.visible_from(dem, here, altitude, target, hiker.z + t.PERSON_H):
                continue
            found_at = time[lo] + f * dt[lo]
            found_energy = energy[lo] + f * de[lo]
            kit_at = None
            drop_alt = hiker.z + t.DROP_HEIGHT
            if ac.kit and altitude >= drop_alt >= float(t.dem_at(dem, *here)) + t.DROP_HEIGHT:
                _, return_wh, _ = t.ms.leg_cost(ac, here, (0.0, 0.0), wind, False, Va=route.speed)
                return_wh += self.k_climb * (1 - ac.kit_mass / ac.mass) * t.return_climb(
                    dem, here, drop_alt, home + t.H_SEARCH)
                drop_wh = self.power_pts[lo] * t.ms.DROP_TIME / 3600
                if found_energy + drop_wh + return_wh + turn_wh <= self.budget:
                    kit_at = found_at + t.ms.DROP_TIME + t.DROP_FALL_TIME
            searched_km = (self.length[:i].sum() + along[i]) / 1000
            return dict(found=True, t_find=found_at, t_kit=kit_at, h_agl=hag, source=self.source[i],
                        cutoff="found", searched_km=searched_km)
        cutoff = "reserve" if len(exhausted) else route.end_reason
        return dict(miss, cutoff=cutoff, searched_km=searched_km)


def compare_site(args):
    site, mass, pattern, n, modes = args
    prep = t.prep(site, n, t.ms.RNG_SEED)
    home, hikers = prep["z_home"], prep["hikers"]
    dem = t.load(site)[0]
    cover, hide = t.cover_draws(site, hikers, t.ms.RNG_SEED)
    path = prep["pats"][(pattern, t.CAMERAS["Lodestar"][1])]
    legs = path["legs"][:t.sp.MAX_LEGS]
    chute = pd.read_csv(t.ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")
    results, routes = [], []
    for ac in t.site_fleet(home + t.H_SEARCH, wing_mass_kg=mass, include_competitors=False):
        climb, tan = t.climb_k(ac, ac), t.camera_tangent(ac)
        flights = {}
        for mode in modes:
            rows = []
            for i, h in enumerate(hikers.itertuples()):
                wind = np.array([h.ws * math.cos(h.wd), h.ws * math.sin(h.wd)])
                canopy = (cover[i], hide[i])
                if mode == "sharp":
                    row = t.fly(ac, path, (h.tx, h.ty), h.z, wind, h.draws, climb, tan, home, dem, canopy) if h.on_map else {
                        "found": False, "t_find": None, "t_kit": None, "h_agl": None}
                else:
                    bound = WIND_BOUNDS[-1] if mode == "rounded_fixed" else next(b for b in WIND_BOUNDS if h.ws <= b)
                    if bound not in flights:
                        route = build_route(ac, legs, dem, home, bound, home + t.H_SEARCH)
                        flights[bound] = Flight(ac, route)
                        routes.append(dict(site=site, pattern=pattern, aircraft=ac.name, wind_bound_mps=bound,
                                           airspeed_mps=route.speed, radius_m=route.radius, max_altitude_m=route.altitude.max(),
                                           retained_arcs=sum(p.center is not None for p in route.pieces),
                                           planned_fillets=route.fillets, planned_loops=route.loops,
                                           path_km=route.ds.sum() / 1000,
                                           straight_search_km=flights[bound].length.sum() / 1000,
                                           end_reason=route.end_reason))
                    row = flights[bound].replay(h, wind, canopy, dem, home)
                rows.append(row)
            runs = pd.DataFrame(rows).rename_axis("run").reset_index()
            scored = t.score(runs, hikers, lambda wind: np.interp(wind, chute.wind, chute.p_reach))
            scored = scored[[*runs.columns, "ws", "help_1h", "help_3h"]]
            results.append(scored.assign(site=site, aircraft=ac.name, pattern=pattern, mode=mode))
    print(f"Completed {site} / {pattern}", flush=True)
    return pd.concat(results, ignore_index=True), pd.DataFrame(routes)


def paired_intervals(runs):
    """Bootstrap matched run IDs as blocks across the fixed set of locations."""
    rows = []
    rng = np.random.default_rng(8801)
    for (pattern, aircraft), group in runs.groupby(["pattern", "aircraft"]):
        for metric in ("found", "help_1h", "help_3h"):
            scores = group.pivot(index=["site", "run"], columns="mode", values=metric).astype(float)
            comparisons = [(mode, "sharp") for mode in MODES[1:]] + [(MODES[2], MODES[1])]
            for mode, reference in comparisons:
                paired = (scores[mode] - scores[reference]).groupby("run").mean().to_numpy()
                samples = rng.integers(0, len(paired), (4000, len(paired)))
                low, high = np.quantile(paired[samples].mean(axis=1), [0.025, 0.975])
                rows.append(dict(pattern=pattern, aircraft=aircraft, metric=metric, mode=mode,
                                 reference=reference, delta_pp=100 * paired.mean(),
                                 low_95_pp=100 * low, high_95_pp=100 * high))
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--patterns", nargs="+", choices=("trails 3", PATTERN), default=["trails 3", PATTERN])
    parser.add_argument("--sites", nargs="+", choices=t.SITES, default=list(t.SITES))
    parser.add_argument("--workers", type=int, default=7)
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("workers must be positive")
    mass = pd.read_csv(OUT / "summary.csv").set_index("design").wing_mass_kg["compact_xps"]
    jobs = [(site, mass, pattern, N, MODES) for site in dict.fromkeys(args.sites)
            for pattern in dict.fromkeys(args.patterns)]
    with ProcessPoolExecutor(max_workers=min(len(jobs), args.workers)) as pool:
        runs, routes = zip(*pool.map(compare_site, jobs))
    runs = pd.concat(runs, ignore_index=True)
    route_table = pd.concat(routes, ignore_index=True)
    full_replay = set(args.sites) == set(t.SITES) and set(args.patterns) == {"trails 3", PATTERN}
    output = OUT / ("flyable" if full_replay else "flyable_subset")
    if t.CROWN_POROSITY != 0.2:
        output = output.with_name(f"{output.name}_porosity_{t.CROWN_POROSITY:g}")
    output.mkdir(exist_ok=True)
    runs.to_csv(output / "runs.csv.gz", index=False)
    route_table.to_csv(output / "routes.csv", index=False)
    metrics = ["found", "help_1h", "help_3h", "searched_km"]
    by_site = runs.groupby(["pattern", "aircraft", "mode", "site"])[metrics].mean()
    by_site.to_csv(output / "by_site.csv")
    summary = by_site.groupby(level=["pattern", "aircraft", "mode"]).mean()
    summary.to_csv(output / "summary.csv")
    paired_intervals(runs).to_csv(output / "paired_intervals.csv", index=False)
    print(summary.round(4).to_string(), flush=True)


if __name__ == "__main__":
    main()
