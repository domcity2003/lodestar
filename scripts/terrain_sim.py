"""Real-terrain round: lost-hiker searches on real maps of busy US SAR areas.

The search-pattern round used a flat world with four straight "trails". Here each site is a real
30 x 30 km map (terrain_fetch.py): ~30 m elevation and OpenStreetMap trails, roads and streams.

What the real terrain changes:
  - Where hikers are: ISRID temperate-mountain distance from the trailhead (log-normal, as before),
    and find locations on the REAL features: trail 25%, road 13%, drainage (streams) 12%, the rest
    anywhere at that distance. 15 m scatter around a feature.
  - Flying: every aircraft follows the terrain at >= 120 m above the ground under it, climbing at
    most 15% and descending at most 20% of distance flown (the lowest path meeting both limits, so
    it starts climbing before a ridge). Climbing costs energy (m g dh / eta). Descending saves
    propulsion energy down to an avionics floor. Search time and energy include departure and
    the initial climb, and energy to the detection point is integrated along the sampled profile.
    The trip home is costed along the straight line over the terrain, including intervening ridges.
  - Camera: a person is seen only inside the nadir footprint (no detection tail outside the
    frame), with the Johnson-criteria pass probability at the true height above the person and
    the motion blur at the ground speed (calibrated to the inherited 55 m sweep width at 120 m,
    32 deg). Detection also requires bare-earth line of sight and a gap through the tree canopy
    at the person's location. A shared per-person draw keeps the canopy blockage correlated across views.
  - Ground team: walks the real terrain from the trailhead at Tobler's hiking speed
    (6 exp(-3.5 |slope + 0.05|) km/h on trails, 60% of that off trail), drives roads at 25 km/h.
    Shortest-time route over the 30 m grid (Dijkstra), sent at the moment of the find.
  - The kit: counts only if it lands within reach, with the drop-accuracy round's 7 m/s chute
    (out/drop/drop_sweep.csv). The release proxy is 40 m above the target at the detection position.
    It must clear the terrain there by at least 40 m. The return climb starts at release altitude
    and uses the post-drop mass.
  - Patterns: box (4 x 4 km lawnmower), spiral (expanding square), and "trails R": fly every mapped
    trail and road reachable within R km walking distance of the trailhead, in rings nearest first
    (edge-by-edge tour, straight hops between branches), then the expanding square.

Run: uv run python scripts/terrain_sim.py [N] (--help lists sensitivity options).
Use --plot with the original sample count to redraw the full fleet charts.
Writes terrain_runs.csv.gz, terrain_summary.csv and terrain_log.txt under out/terrain by default.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import json
import math
import os
import pickle
import sys

import numpy as np
import pandas as pd
import networkx as nx
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import v3_blur as vb  # noqa: E402
import search_patterns as sp  # noqa: E402
import v4_upgrade as v4u  # noqa: E402
from terrain_fetch import SITES, HALF, CELL, DATA  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "terrain"
OUT.mkdir(parents=True, exist_ok=True)

H_SEARCH = 120.0
CLIMB_GRAD, DESC_GRAD = 0.15, 0.20
DROP_HEIGHT = 40.0
DROP_FALL_TIME = DROP_HEIGHT / 7.0
FIND = {"trail": 0.25, "road": 0.13, "drainage": 0.12}          # rest: anywhere at that distance
SCATTER = 15.0
ROAD_KMH = 25.0
OFFTRAIL = 0.6
TAU = 0.010
PATTERNS = ["box", "spiral", "trails 3", "trails 6"]
N_DEFAULT = 400
SAMPLE = 60.0             # m between terrain samples along the flight path
PERSON_H = 1.0            # m: line-of-sight aim point above the ground at the person
# Share of sky visible through tree crowns (assumed). Sensitivity runs set LODESTAR_POROSITY (e.g. 0.1, 0.35),
# which also redirects output to out/terrain_porosity_<value>/ (env var so worker processes inherit it).
CROWN_POROSITY = float(os.environ.get("LODESTAR_POROSITY", 0.2))
if not 0.0 <= CROWN_POROSITY <= 1.0:
    raise ValueError("LODESTAR_POROSITY must be between 0 and 1")
if "LODESTAR_POROSITY" in os.environ:
    OUT = ROOT / "out" / f"terrain_porosity_{CROWN_POROSITY:g}"
    OUT.mkdir(parents=True, exist_ok=True)
SEARCH_V = {"Lodestar SiCore": 18.0, "Lodestar P50B": 18.0}   # m/s; 18 beats 20 for the 50 deg camera (spacing_check.py)
PREP_VERSION = 2          # bump when prep()'s cached contents change


# ---------------------------------------------------------------- grid helpers
def load(key):
    d = DATA / key
    return np.load(d / "dem.npy"), json.loads((d / "osm.json").read_text(encoding="utf-8")), \
        json.loads((d / "meta.json").read_text(encoding="utf-8"))


def rc(x, y, n):
    return (HALF - np.asarray(y)) / CELL - 0.5, (np.asarray(x) + HALF) / CELL - 0.5


def dem_at(dem, x, y):
    n = dem.shape[0]
    r, c = rc(x, y, n)
    r = np.clip(r, 0, n - 1.001)
    c = np.clip(c, 0, n - 1.001)
    r0, c0 = np.floor(r).astype(int), np.floor(c).astype(int)
    fr, fc = r - r0, c - c0
    return (dem[r0, c0] * (1 - fr) * (1 - fc) + dem[r0 + 1, c0] * fr * (1 - fc)
            + dem[r0, c0 + 1] * (1 - fr) * fc + dem[r0 + 1, c0 + 1] * fr * fc)


def densify(line, step):
    x, y = np.asarray(line["x"]), np.asarray(line["y"])
    seg = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < 1:
        return x[:1], y[:1]
    t = np.arange(0, s[-1], step)
    return np.interp(t, s, x), np.interp(t, s, y)


# ---------------------------------------------------------------- ground team (Dijkstra over the grid)
def team_times(dem, lines):
    n = dem.shape[0]
    cls = np.zeros((n, n), np.int8)                         # 0 off, 1 trail, 2 road
    for l in lines:
        if l["kind"] == "drainage":
            continue
        x, y = densify(l, 10.0)
        r, c = rc(x, y, n)
        r, c = np.round(r).astype(int), np.round(c).astype(int)
        ok = (r >= 0) & (r < n) & (c >= 0) & (c < n)
        v = 2 if l["kind"] == "road" else 1
        cls[r[ok], c[ok]] = np.maximum(cls[r[ok], c[ok]], v)
    idx = np.arange(n * n).reshape(n, n)
    rows, cols, cost = [], [], []
    for dr, dc in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
        a = idx[max(0, -dr):n - max(0, dr), max(0, -dc):n - max(0, dc)]
        b = idx[max(0, dr):n - max(0, -dr) or None, max(0, dc):n - max(0, -dc) or None]
        za = dem.ravel()[a.ravel()]
        zb = dem.ravel()[b.ravel()]
        dist = CELL * math.hypot(dr, dc)
        slope = (zb - za) / dist
        v = 6.0 * np.exp(-3.5 * np.abs(slope + 0.05)) / 3.6           # m/s on a trail
        ca, cb = cls.ravel()[a.ravel()], cls.ravel()[b.ravel()]
        both_road = (ca == 2) & (cb == 2)
        on = (ca > 0) & (cb > 0)
        v = np.where(both_road, ROAD_KMH / 3.6, np.where(on, v, v * OFFTRAIL))
        rows.append(a.ravel())
        cols.append(b.ravel())
        cost.append(dist / v)
    g = csr_matrix((np.concatenate(cost), (np.concatenate(rows), np.concatenate(cols))), shape=(n * n, n * n))
    src = idx[n // 2, n // 2]
    return dijkstra(g, indices=src, directed=True).reshape(n, n).astype(np.float32), cls


# ---------------------------------------------------------------- hikers on real features
def draw_hikers(n, rng, lines):
    pts = {}
    for k in FIND:
        xs, ys = [], []
        for l in lines:
            if l["kind"] == k:
                x, y = densify(l, 30.0)
                xs.append(x)
                ys.append(y)
        X, Y = np.concatenate(xs), np.concatenate(ys)
        pts[k] = (X, Y, np.hypot(X, Y))
    kinds = list(FIND) + ["anywhere"]
    probs = list(FIND.values()) + [1 - sum(FIND.values())]
    out = []
    for i in range(n):
        r = math.exp(rng.normal(sp.MU, sp.SIGMA)) * 1000.0
        k = kinds[rng.choice(4, p=probs)]
        x = y = None
        if k != "anywhere":
            X, Y, R = pts[k]
            m = np.flatnonzero(np.abs(R - r) <= 0.15 * r + 30)
            if len(m):
                j = rng.choice(m)
                x, y = X[j] + rng.normal(0, SCATTER), Y[j] + rng.normal(0, SCATTER)
        if x is None:
            th = rng.uniform(0, 2 * math.pi)
            x, y = r * math.cos(th), r * math.sin(th)
            k = "anywhere" if k == "anywhere" else k + " (none at that distance)"
        out.append(dict(kind=k, r_km=r / 1000, tx=x, ty=y, ws=rng.uniform(0, 10), wd=rng.uniform(0, 2 * math.pi)))
    df = pd.DataFrame(out)
    df["on_map"] = (df.tx.abs() < HALF - 300) & (df.ty.abs() < HALF - 300)
    return df


# ---------------------------------------------------------------- patterns on the real trail network
def rdp(pts, eps):
    pts = np.asarray(pts)
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    L = np.hypot(*ab)
    q = pts - a
    d = np.abs(ab[0] * q[:, 1] - ab[1] * q[:, 0]) / L if L > 0 else np.hypot(*q.T)
    i = int(np.argmax(d))
    if d[i] > eps:
        return np.vstack([rdp(pts[:i + 1], eps)[:-1], rdp(pts[i:], eps)])
    return np.vstack([a, b])


def chains_to_legs(chains, pos, legs):
    for ch in chains:
        pts = rdp(np.array(ch, float), 25.0)
        if math.dist(pos, pts[0]) > 1:
            legs.append((pos, tuple(pts[0])))
        for a, b in zip(pts[:-1], pts[1:]):
            legs.append((tuple(a), tuple(b)))
        pos = tuple(pts[-1])
    return pos


def trail_tour(lines, R, rings, spacing=sp.TRACK_SP):
    """Fly every trail and road reachable within R walking distance, in rings (nearest first).
    Inside a ring: edge-by-edge depth-first tour of each connected piece, nearest piece first."""
    G = nx.Graph()
    for l in lines:
        if l["kind"] == "drainage":
            continue
        p = list(zip(l["x"], l["y"]))
        for u, v in zip(p[:-1], p[1:]):
            if u != v:
                G.add_edge(u, v, w=math.dist(u, v))
    # start on the nearest node of a real network piece (>= 5 km), not a stray parking-lot fragment
    big = set().union(*(c for c in nx.connected_components(G)
                        if G.subgraph(c).size(weight="w") >= 5000))
    nodes = np.array([n for n in G.nodes if n in big])
    start = tuple(nodes[np.argmin(np.hypot(nodes[:, 0], nodes[:, 1]))])
    dist = nx.single_source_dijkstra_path_length(G, start, cutoff=R, weight="w")
    legs, pos, km = [], (0.0, 0.0), 0.0
    edges = [(u, v) for u, v in G.edges if u in dist and v in dist]
    bounds = list(rings) + [R]
    r0 = 0.0
    for r1 in bounds:
        E = [(u, v) for u, v in edges if r0 <= min(dist[u], dist[v]) < r1]
        r0 = r1
        H = nx.Graph()
        H.add_edges_from(E)
        km += sum(math.dist(u, v) for u, v in E) / 1000
        while H.number_of_edges():
            hn = np.array(list(H.nodes))
            src = tuple(hn[np.argmin(np.hypot(hn[:, 0] - pos[0], hn[:, 1] - pos[1]))])
            comp = H.subgraph(nx.node_connected_component(H, src)).copy()
            chains, cur = [], [src]
            for u, v in nx.edge_dfs(comp, src):
                if u != cur[-1]:
                    chains.append(cur)
                    cur = [u]
                cur.append(v)
            chains.append(cur)
            pos = chains_to_legs(chains, pos, legs)
            H.remove_edges_from(list(comp.edges))
            H.remove_nodes_from(list(nx.isolates(H)))
    legs.append((pos, (0.0, 0.0)))
    return legs + sp.expanding_square(spacing), km, math.dist(start, (0, 0))


def profile(dem, legs):
    """Terrain-following altitude along the whole pattern; per leg: (s, alt, total climb, per-segment dz)."""
    xs, ys, leg_id, s_in = [], [], [], []
    for k, (p0, p1) in enumerate(legs):
        L = math.dist(p0, p1)
        m = max(2, int(L / SAMPLE) + 1)
        t = np.linspace(0, 1, m)
        xs.append(p0[0] + t * (p1[0] - p0[0]))
        ys.append(p0[1] + t * (p1[1] - p0[1]))
        leg_id.append(np.full(m, k))
        s_in.append(t * L)
    X, Y, K, S = map(np.concatenate, (xs, ys, leg_id, s_in))
    ground = dem_at(dem, X, Y)
    want = ground + H_SEARCH
    step = np.concatenate([[0], np.hypot(np.diff(X), np.diff(Y))])
    alt = clearance_profile(want, step.cumsum())
    dz = np.concatenate([[0], np.diff(alt)])
    out = []
    for k in range(len(legs)):
        m = K == k
        d = dz[m][1:]
        out.append((S[m], alt[m], float(d[d > 0].sum()), d))
    return out


def partial_leg_energy(distance, altitude, stop, cruise_wh_per_m, climb_wh_per_m, idle_wh_per_m):
    """Integrate energy only up to the detection point on the sampled altitude profile."""
    last = np.searchsorted(distance, stop, side="left")
    points = np.append(distance[:last], stop)
    heights = np.append(altitude[:last], np.interp(stop, distance, altitude))
    dx = np.diff(points)
    dz = np.diff(heights)
    return float(np.maximum(cruise_wh_per_m * dx + climb_wh_per_m * dz, idle_wh_per_m * dx).sum())


def clearance_profile(minimum, distance):
    """Lowest sampled path satisfying both ground-distance gradient limits."""
    climb_bound = minimum - CLIMB_GRAD * distance
    descend_bound = minimum + DESC_GRAD * distance
    before_ridge = np.maximum.accumulate(climb_bound[::-1])[::-1] + CLIMB_GRAD * distance
    after_ridge = np.maximum.accumulate(descend_bound) - DESC_GRAD * distance
    return np.maximum(before_ridge, after_ridge)


def line_samples(p0, p1):
    fraction = np.linspace(0, 1, max(1, math.ceil(math.dist(p0, p1) / CELL)) + 1)
    x = p0[0] + fraction * (p1[0] - p0[0])
    y = p0[1] + fraction * (p1[1] - p0[1])
    return fraction, x, y


def return_climb(dem, pos, altitude, home_altitude):
    """Total ascent on a direct return, including any climb before departure."""
    distance = math.hypot(*pos)
    if distance == 0:
        return max(0.0, home_altitude - altitude)
    fraction, x, y = line_samples(pos, (0.0, 0.0))
    minimum = dem_at(dem, x, y) + H_SEARCH
    minimum[0] = max(minimum[0], altitude)
    minimum[-1] = max(minimum[-1], home_altitude)
    path = clearance_profile(minimum, distance * fraction)
    return max(0.0, path[0] - altitude) + np.maximum(np.diff(path), 0).sum()


def return_climbs(dem, legs, profiles, z_home):
    return np.array([return_climb(dem, leg[1], profile[1][-1], z_home + H_SEARCH)
                     for leg, profile in zip(legs[:sp.MAX_LEGS], profiles)])


def canopy_clear(cover, cos_view):
    """Chance that enough of a person under tree cover `cover` (0-1) is visible along a ray at angle
    acos(cos_view) from vertical: gap fraction (1 - cover (1 - porosity)), path length 1 / cos."""
    return (1.0 - cover * (1.0 - CROWN_POROSITY)) ** (1.0 / max(cos_view, 0.05))


def cover_draws(key, hikers, seed):
    """Tree cover at each hiker and a per-hiker hiding draw. The draw is fixed for the whole mission:
    a person under a dense crown stays hidden on every pass (occlusion is not re-rolled per pass)."""
    f = DATA / key / "canopy.npy"
    cc = np.load(f)
    n = cc.shape[0]
    r, c = rc(hikers.tx.values, hikers.ty.values, n)
    r = np.clip(np.round(r).astype(int), 0, n - 1)
    c = np.clip(np.round(c).astype(int), 0, n - 1)
    rng = np.random.default_rng([seed, list(SITES).index(key), 7])
    return cc[r, c].astype(float), rng.uniform(0, 1, len(hikers))


def visible_from(dem, position, altitude, target, target_altitude):
    """Sample the straight viewing ray against bare-earth terrain."""
    fraction, x, y = line_samples(position, target)
    sightline = altitude + fraction * (target_altitude - altitude)
    return bool(np.all(sightline[:-1] > dem_at(dem, x[:-1], y[:-1])))


# ---------------------------------------------------------------- flier
def camera_probability(x, vg, height, half_angle_tangent, pixel_tangent=None):
    """Johnson-criteria pass probability inside the nadir camera footprint.

    pixel_tangent sets the nadir pixel size as 2 * height * pixel_tangent / 640. It defaults to the footprint
    half-angle tangent (one 640 px camera spanning the footprint). A splayed multi-camera array keeps its
    single-camera pixel size while the footprint widens (Step 2 camera trade).
    """
    half_width = height * half_angle_tangent
    if height <= 0 or x > half_width:
        return 0.0
    cosine = height / math.hypot(height, x)
    centre_gsd = 2 * height * (half_angle_tangent if pixel_tangent is None else pixel_tangent) / 640
    across_gsd = centre_gsd / cosine**2
    along_gsd = math.hypot(centre_gsd / cosine, vg * v4u.TAU)
    cycles = vb.D_CRIT / (2 * math.sqrt(across_gsd * along_gsd))
    odds = cycles ** (1.51 + 0.24 * cycles) * CAMERA_ODDS_SCALE if cycles < 100 else math.inf
    return 1 - 1 / (1 + odds)


def camera_calibration():
    """Fit the inherited 55 m reference by scaling odds, preserving probabilities in [0, 1]."""
    tangent = math.tan(math.radians(16))
    half_width = H_SEARCH * tangent
    x = np.linspace(-half_width, half_width, 401)
    cosine = H_SEARCH / np.hypot(H_SEARCH, x)
    cycles = vb.D_CRIT * cosine**1.5 / (2 * (2 * half_width / 640))
    odds = cycles ** (1.51 + 0.24 * cycles)
    from scipy.optimize import brentq
    return brentq(lambda scale: np.trapezoid(1 - 1 / (1 + scale * odds), x) - vb.W0, 0, 100)


# Detection calibration. Default (low case) = the original 55 m sweep calibration. Step 2 validation
# (research/2026-11-15-detection-validation.md) found it pessimistic; LODESTAR_ODDS_SCALE sets the nominal (1.5)
# or high (3.0) field-data cases. Env var so worker processes inherit it.
CAMERA_ODDS_SCALE = float(os.environ.get("LODESTAR_ODDS_SCALE", 0)) or camera_calibration()


def climb_k(ac, ref):
    """Wh per metre climbed. Competitors (no mass model) get Lodestar's ratio of climb energy to cruise power."""
    if ac.rw_cfg is not None:
        return ac.mass * ms.G / ms.sz.A.eta_max / 3600
    kL = ref.mass * ms.G / ms.sz.A.eta_max / 3600
    return kL / float(ref.power(ref.v_cruise)) * ac.p_cruise


def camera_tangent(ac):
    hfov = CAMERAS["Lodestar" if ac.name.startswith("Lodestar") else "other"][0]
    return math.tan(math.radians(hfov / 2))


def fly(ac, pat, target, z_target, wind, draws, k_climb, cam_tan, z_home, dem, cover=None):
    """One mission along the sampled terrain profiles in pat.

    cover is the tree cover at the person and their hiding draw, or None for bare earth.
    """
    legs, prof, ret = pat["legs"], pat["prof"], pat["ret"]
    budget = ac.energy * (1 - ms.RESERVE)
    kit_on = ac.kit
    cruise_power = float(ac.power(ac.v_cruise, kit_on))
    idle_power = ms.sz.A.p_avionics_w if ac.rw_cfg is not None else 0.0
    # Start the search at its first planned point. The departure climbs at the
    # search gradient in still air before the simulated search pattern begins.
    first_altitude = prof[0][1][0]
    transit_t, transit_e, _ = ms.leg_cost(ac, (0.0, 0.0), legs[0][0], wind, kit_on, Va=ac.v_cruise)
    if not math.isfinite(transit_t):
        return dict(found=False, t_find=None, t_kit=None, h_agl=None)
    launch_rise = max(first_altitude - z_home, 0.0)
    t = max(transit_t, launch_rise / (CLIMB_GRAD * ac.v_cruise))
    E = transit_e + k_climb * launch_rise + cruise_power * (t - transit_t) / 3600
    hdg = None
    for k, (p0, p1) in enumerate(legs[:sp.MAX_LEGS]):
        L = math.dist(p0, p1)
        if L < 1.0:
            continue
        s_arr, a_arr, up, dz = prof[k]
        h = sp.heading(p0, p1)
        if hdg is None:
            et = tt = 0.0
        else:
            d = abs((h - hdg + math.pi) % (2 * math.pi) - math.pi)
            r = ac.v_cruise**2 / (ms.G * math.tan(ms.BANK))
            tt = r * d / ac.v_cruise
            et = cruise_power * ms.TURN_POWER * tt / 3600
        tl, cruise_energy, gs = ms.leg_cost(ac, p0, p1, wind, kit_on, Va=ac.v_cruise)
        if not np.isfinite(tl):
            break
        cruise_wh_per_m = cruise_energy / L
        idle_wh_per_m = idle_power / (3600 * gs)
        credit = 0.0
        if a_arr[-1] - a_arr[0] < up:
            credit = np.minimum(k_climb * np.maximum(-dz, 0.0),
                                max(0.0, cruise_wh_per_m - idle_wh_per_m) * np.diff(s_arr)).sum()
        el = cruise_energy + k_climb * up - credit
        _, e_home, _ = ms.leg_cost(ac, p1, (0.0, 0.0), wind, kit_on)
        e_home += k_climb * ret[k]
        if E + et + el + e_home > budget:
            break
        t, E = t + tt, E + et
        ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L
        s = (target[0] - p0[0]) * ux + (target[1] - p0[1]) * uy
        if 0.0 <= s <= L:
            x = abs((target[0] - p0[0]) * uy - (target[1] - p0[1]) * ux)
            a_here = float(np.interp(s, s_arr, a_arr))
            hag = max(a_here - z_target, 1.0)
            here = (p0[0] + ux * s, p0[1] + uy * s)
            if (draws[k] < camera_probability(x, gs, hag, cam_tan)
                    and (cover is None or cover[1] < canopy_clear(cover[0], hag / math.hypot(hag, x)))
                    and visible_from(dem, here, a_here, target, z_target + PERSON_H)):
                found_t = t + s / gs
                E += partial_leg_energy(s_arr, a_arr, s, cruise_wh_per_m, k_climb, idle_wh_per_m)
                kit_t = None
                drop_altitude = z_target + DROP_HEIGHT
                if ac.kit and a_here >= drop_altitude >= float(dem_at(dem, *here)) + DROP_HEIGHT:
                    e_drop = cruise_power * ms.DROP_TIME / 3600
                    _, e_home2, _ = ms.leg_cost(ac, here, (0.0, 0.0), wind, False)
                    post_drop_climb = k_climb * (ac.mass - ac.kit_mass) / ac.mass
                    e_home2 += post_drop_climb * return_climb(dem, here, drop_altitude, z_home + H_SEARCH)
                    if E + e_drop + e_home2 <= budget:
                        kit_t = found_t + ms.DROP_TIME + DROP_FALL_TIME
                return dict(found=True, t_find=found_t, t_kit=kit_t, h_agl=hag)
        t, E = t + tl, E + el
        hdg = h
    return dict(found=False, t_find=None, t_kit=None, h_agl=None)


# ---------------------------------------------------------------- fleet at the site's altitude
# Lodestar v4: 50 deg search camera (FLIR Boson 640, 8.7 mm), 7 m/s chute; competitors keep their 32 deg-class camera.
CAMERAS = {"Lodestar": (v4u.HFOV_V4, round(v4u.camera(v4u.HFOV_V4)[1], 1)), "other": (32.0, sp.TRACK_SP)}
# Optional nadir pixel tangent per camera key (see camera_probability); empty = one camera per footprint.
CAMERA_PIXEL_TAN = {}


def camera_pixel_tangent(ac):
    key = "Lodestar" if ac.name.startswith("Lodestar") else "other"
    return CAMERA_PIXEL_TAN.get(key, camera_tangent(ac))


def site_fleet(alt, *, wing_mass_kg=None, include_competitors=True, competitor_endurance=1.0):
    """Lodestar's power model at the real air density; competitors' hours scaled by sqrt(density ratio)
    (power at a fixed lift coefficient goes as 1/sqrt(density)) and by the requested endurance factor."""
    import aerosandbox as asb
    v4u.apply_v4()
    fleet = []
    for lab, cell in (("Lodestar SiCore", "Amprius SiCore 6.3 Ah"), ("Lodestar P50B", "Molicel P50B")):
        v = SEARCH_V[lab]
        ac = vb.v3m.lodestar_v3(lab, cell, "kit", "halfway", 3.0, 7.0, alt=alt, wing_mass_kg=wing_mass_kg)
        ac.v_cruise, ac.search_h = v, ac.energy / float(ac.power(v))
        fleet.append(ac)
    sig = float(asb.Atmosphere(altitude=alt).density()) / 1.225
    if include_competitors:
        for c in list(ms.COMPETITORS) + [ms.FRONTIER]:
            hours = c.energy * competitor_endurance * math.sqrt(sig)
            a = ms.competitor(c.name, c.v_cruise, hours, c.v_max, c.note)
            a.rho_ref = sig * 1.225   # density its power curve refers to (flyable_mission local-density power)
            fleet.append(a)
    return fleet


# ---------------------------------------------------------------- jobs
def prep(key, n, seed):
    f = DATA / key / f"prep{PREP_VERSION}_{n}_{seed}.pkl"
    if f.exists():
        obj = pickle.loads(f.read_bytes())
        missing = sorted({c[1] for c in CAMERAS.values()} - {spc for _, spc in obj["pats"]})
        if missing:
            # A camera trade asked for a new track spacing: add its patterns (hikers and draws are unchanged,
            # they come first from the seeded generator). Not written back, so the cache stays as built.
            dem, lines, _ = load(key)
            for spc in missing:
                for p in PATTERNS:
                    if p == "box":
                        legs = sp.lawnmower_box(sp=spc)
                    elif p == "spiral":
                        legs = sp.expanding_square(spc)
                    else:
                        R = float(p.split()[1]) * 1000
                        legs = trail_tour(lines, R, [R / 3, 2 * R / 3] if R > 3000 else [R / 2], spc)[0]
                    prof = profile(dem, legs)
                    obj["pats"][(p, spc)] = dict(legs=legs, prof=prof, ret=return_climbs(dem, legs, prof,
                                                                                         obj["z_home"]))
        return obj
    dem, lines, meta = load(key)
    tt, cls = team_times(dem, lines)
    rng = np.random.default_rng(seed)
    hk = draw_hikers(n, rng, lines)
    nn = dem.shape[0]
    r, c = rc(hk.tx.values, hk.ty.values, nn)
    r = np.clip(np.round(r).astype(int), 0, nn - 1)
    c = np.clip(np.round(c).astype(int), 0, nn - 1)
    hk["t_team"] = tt[r, c]
    hk["z"] = dem_at(dem, hk.tx.values, hk.ty.values)
    gy, gx = np.gradient(dem, CELL)
    hk["slope_deg"] = np.degrees(np.arctan(np.hypot(gx, gy)))[r, c]
    hk["draws"] = list(rng.uniform(0, 1, (n, sp.MAX_LEGS)))
    pats, info = {}, {}
    z_home = float(dem_at(dem, 0.0, 0.0))
    for spc in sorted({c[1] for c in CAMERAS.values()}):
        for p in PATTERNS:
            if p == "box":
                legs = sp.lawnmower_box(sp=spc)
            elif p == "spiral":
                legs = sp.expanding_square(spc)
            else:
                R = float(p.split()[1]) * 1000
                legs, km, off = trail_tour(lines, R, [R / 3, 2 * R / 3] if R > 3000 else [R / 2], spc)
                info[p] = dict(trail_km=km, start_offset_m=off)
            prof = profile(dem, legs)
            pats[(p, spc)] = dict(legs=legs, prof=prof, ret=return_climbs(dem, legs, prof, z_home))
    obj = dict(hikers=hk, pats=pats, info=info, z_home=z_home, meta=meta, team_grid=tt[::4, ::4], cls=cls)
    f.write_bytes(pickle.dumps(obj))
    return obj


def job(args):
    key, pattern, n, seed, competitor_endurance = args
    P = prep(key, n, seed)
    hk = P["hikers"]
    dem = load(key)[0]
    cc, hide = cover_draws(key, hk, seed)
    fleet = site_fleet(P["z_home"] + H_SEARCH, competitor_endurance=competitor_endurance)
    ref = fleet[0]
    rows = []
    for ac in fleet:
        spc = CAMERAS["Lodestar" if ac.name.startswith("Lodestar") else "other"][1]
        pat = P["pats"][(pattern, spc)]
        kc = climb_k(ac, ref)
        tan_half = camera_tangent(ac)
        for i, h in hk.iterrows():
            if not h.on_map:
                rows.append(dict(found=False, t_find=None, t_kit=None, h_agl=None))
                continue
            wind = (h.ws * math.cos(h.wd), h.ws * math.sin(h.wd))
            rows.append(fly(ac, pat, (h.tx, h.ty), h.z, wind, h.draws, kc, tan_half, P["z_home"], dem,
                            (cc[i], hide[i])))
        for j, r in enumerate(rows[-len(hk):]):
            r.update(site=key, pattern=pattern, aircraft=ac.name, run=j)
    return pd.DataFrame(rows)


def score(df, hk, phit):
    d = df.merge(hk.drop(columns=["draws"]).reset_index().rename(columns={"index": "run"}), on="run")
    team = np.where(d.found, d.t_find + d.t_team, np.inf)
    tk = d.t_kit.fillna(np.inf).values
    p = np.where(d.t_kit.notna(), phit(d.ws.values), 0.0)
    for name, Wn in (("help_1h", 3600), ("help_3h", 3 * 3600)):
        ok_k, ok_t = tk <= Wn, team <= Wn
        d[name] = p * ok_k + (1 - p) * ok_t + p * (~ok_k) * ok_t
        d[name + "_perfect"] = np.where(d.t_kit.notna(), ok_k | ok_t, ok_t)
    d["t_help"] = np.where(d.t_kit.notna(), np.minimum(tk, team), team)
    return d


def main(n, patterns, competitor_endurance, workers):
    output = OUT
    if competitor_endurance != 1.0:
        output = OUT.with_name(f"{OUT.name}_endurance_{competitor_endurance:g}")
    if patterns != PATTERNS:
        suffix = "_".join(pattern.replace(" ", "") for pattern in patterns)
        output = output.with_name(f"{output.name}_{suffix}")
    for key in SITES:
        if not (DATA / key / "canopy.npy").is_file():
            raise FileNotFoundError(f"Missing canopy grid for {key}; run scripts/terrain_fetch.py --canopy")
    output.mkdir(parents=True, exist_ok=True)
    sys.stdout = Tee(output / "terrain_log.txt")
    print(f"Lodestar real-terrain replay. N = {n} hikers per site. "
          f"Tree canopy on (NLCD 2021, crown porosity {CROWN_POROSITY:g}); Lodestar search speed {SEARCH_V}. "
          f"Competitor published-endurance factor: {competitor_endurance:g}. Patterns: {', '.join(patterns)}.\n",
          flush=True)
    seed = ms.RNG_SEED
    for key in SITES:
        prep(key, n, seed)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        parts = list(ex.map(job, [(k, p, n, seed, competitor_endurance) for k in SITES for p in patterns]))
    runs = pd.concat(parts, ignore_index=True)
    tab = pd.read_csv(ROOT / "out" / "drop" / "drop_sweep.csv")
    t7 = tab[tab.config == "7 m/s chute"]
    phit = lambda w: np.interp(w, t7.wind.values, t7.p_reach.values)  # noqa: E731
    scored = []
    for key in SITES:
        P = prep(key, n, seed)
        scored.append(score(runs[runs.site == key], P["hikers"], phit))
    d = pd.concat(scored, ignore_index=True)
    d.drop(columns=["tx", "ty"]).to_csv(output / "terrain_runs.csv.gz", index=False)
    summ = d.groupby(["site", "pattern", "aircraft"], sort=False).agg(
        found=("found", "mean"), help_1h=("help_1h", "mean"), help_3h=("help_3h", "mean"),
        help_3h_perfect=("help_3h_perfect", "mean"),
        median_help_min_from_launch=("t_help", lambda x: np.nanmedian(np.where(np.isfinite(x), x, np.nan)) / 60)).reset_index()
    summ.round(4).to_csv(output / "terrain_summary.csv", index=False)

    print("Sites (DEM trailhead elevation vs published; trails reachable within 3 / 6 km walking):")
    for key in SITES:
        P = prep(key, n, seed)
        m, hk = P["meta"], P["hikers"]
        print(f"  {m['name']:32s} {m['trailhead_elev_dem']:5.0f} m (pub. {m['trailhead_elev_published']}), relief "
              f"{m['elev_min']:.0f}-{m['elev_max']:.0f} m; trail tour {P['info']['trails 3']['trail_km']:.0f} / "
              f"{P['info']['trails 6']['trail_km']:.0f} km; hikers on map {hk.on_map.mean():.0%}, "
              f"ground team reaches {(hk.t_team <= 3 * 3600).mean():.0%} within 3 h if it knew where; "
              f"median slope at hiker {hk.slope_deg.median():.0f} deg; "
              f"canopy at hiker median {np.median(cover_draws(key, hk, seed)[0]):.0%}")
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        for col, lab in (("help_3h", "Help within 3 h (%), realistic drops"), ("help_1h", "Help within 1 h (%)")):
            t = summ.pivot_table(index="aircraft", columns="pattern", values=col, sort=False) * 100
            print(f"\n{lab}, average of the 7 sites:")
            print(t[patterns].round(1).to_string())
        if "trails 6" not in patterns:
            return
        best = summ[summ.pattern == "trails 6"]
        print("\nHelp within 3 h (%) by site, trails 6 km first:")
        print((best.pivot_table(index="site", columns="aircraft", values="help_3h", sort=False) * 100).round(1).to_string())
        print("\nHelp within 1 h (%) by site, trails 6 km first:")
        print((best.pivot_table(index="site", columns="aircraft", values="help_1h", sort=False) * 100).round(1).to_string())
    f = d[(d.pattern == "trails 6") & d.found]
    print("\nMedian height above the person at detection (m), trails 6:",
          f.groupby("aircraft", sort=False).h_agl.median().round(0).to_dict())
    print("Median minutes from launch to help (helped cases), trails 6:",
          summ[summ.pattern == "trails 6"].groupby("aircraft", sort=False).median_help_min_from_launch.median().round(0).to_dict())


# ---------------------------------------------------------------- charts
def plot(n=N_DEFAULT):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "axes.edgecolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    seed = ms.RNG_SEED
    summ = pd.read_csv(OUT / "terrain_summary.csv")
    runs = pd.read_csv(OUT / "terrain_runs.csv.gz")

    # --- maps
    fig, axs = plt.subplots(2, 4, figsize=(16, 8.6))
    for ax, key in zip(axs.ravel(), SITES):
        dem, lines, meta = load(key)
        P = prep(key, n, seed)
        gy, gx = np.gradient(dem, CELL)
        az, alt = math.radians(315), math.radians(45)
        slope = np.arctan(np.hypot(gx, gy))
        aspect = np.arctan2(-gx, gy)
        hs = np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)
        ext = [-HALF / 1000, HALF / 1000, -HALF / 1000, HALF / 1000]
        ax.imshow(dem, cmap="terrain", extent=ext, alpha=0.55, vmin=dem.min() - 300, vmax=dem.max())
        ax.imshow(hs, cmap="gray", extent=ext, alpha=0.35)
        for l in lines:
            x, y = np.asarray(l["x"]) / 1000, np.asarray(l["y"]) / 1000
            if l["kind"] == "trail":
                ax.plot(x, y, color="#7a3b10", lw=0.45)
            elif l["kind"] == "road":
                ax.plot(x, y, color="#333333", lw=0.7)
        spc = CAMERAS["Lodestar"][1]
        legs = P["pats"][("trails 6", spc)]["legs"]
        k = len(legs) - len(sp.expanding_square(spc))
        for a, b in legs[:k]:
            ax.plot([a[0] / 1000, b[0] / 1000], [a[1] / 1000, b[1] / 1000], color="#2a78d6", lw=1.1, alpha=0.9)
        hk = P["hikers"]
        hk = hk[hk.on_map]
        ax.scatter(hk.tx / 1000, hk.ty / 1000, s=3, color="#eb6834", lw=0, alpha=0.8)
        ax.plot(0, 0, marker="*", ms=12, color="#ffd400", mec=INK, mew=0.8)
        ax.set_xlim(-HALF / 1000, HALF / 1000)
        ax.set_ylim(-HALF / 1000, HALF / 1000)
        ax.set_title(f"{meta['name']}\n{meta['elev_min']:.0f}-{meta['elev_max']:.0f} m", fontsize=9.5, color=INK, loc="left")
        ax.set_xticks([-10, 0, 10])
        ax.set_yticks([-10, 0, 10])
        ax.tick_params(labelsize=8)
    ax = axs.ravel()[-1]
    ax.axis("off")
    for y, (c, lab, kw) in enumerate([("#2a78d6", "Lodestar's first leg: trails and roads to 6 km", dict(lw=2)),
                                       ("#7a3b10", "mapped trails (OpenStreetMap)", dict(lw=1)),
                                       ("#333333", "roads", dict(lw=1)),
                                       ("#eb6834", "simulated lost hikers (ISRID)", dict(marker="o", lw=0, ms=4)),
                                       ("#ffd400", "trailhead (launch and ground team)", dict(marker="*", lw=0, ms=12, mec=INK))]):
        ax.plot([0.05, 0.18], [0.85 - y * 0.12] * 2, color=c, **kw)
        ax.text(0.22, 0.85 - y * 0.12, lab, va="center", fontsize=9.5, color=INK)
    ax.text(0.05, 0.12, "30 x 30 km around each trailhead, km on the axes.\nElevation: AWS terrain tiles (~30 m).",
            fontsize=8.5, color=MUTED)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    fig.suptitle("Seven busy US search areas, real elevation and trails", x=0.01, ha="left", fontsize=12, color=INK)
    fig.tight_layout()
    fig.savefig(OUT / "terrain_maps.png", dpi=120)

    # --- results
    names = {"Lodestar SiCore": ("Lodestar (SiCore)", "#2a78d6"), "DeltaQuad Evo GOV": ("DeltaQuad Evo GOV", "#eb6834"),
             "Quantum Trinity Pro": ("Quantum Trinity Pro", "#9a9890")}
    best = summ.groupby(["aircraft", "pattern"]).help_3h.mean().unstack()
    site_lab = {k: SITES[k][0].split(":")[0].replace("Great Smoky Mtns", "Smokies").replace("Mount Rainier", "Rainier")
                .replace("Rocky Mountain", "Rocky Mtn").replace("White Mtns NH", "White Mtns") for k in SITES}
    fig, axs = plt.subplots(1, 2, figsize=(15, 5))
    x = np.arange(len(SITES))
    for ax, col, ttl in ((axs[0], "help_3h", "Help within 3 hours"), (axs[1], "help_1h", "Help within 1 hour")):
        for j, (ac, (lab, c)) in enumerate(names.items()):
            pat = best.loc[ac].idxmax()
            v = [summ[(summ.site == k) & (summ.aircraft == ac) & (summ.pattern == pat)][col].iloc[0] * 100 for k in SITES]
            xs = x + (j - 1) * 0.27
            ax.bar(xs, v, 0.25, color=c, label=f"{lab} ({pat})")
            for xi, vi in zip(xs, v):
                ax.text(xi, vi + 0.5, f"{vi:.0f}", ha="center", fontsize=8, color=INK)
        ax.set_xticks(x, [site_lab[k] for k in SITES], fontsize=9)
        ax.set_ylabel("% of lost hikers")
        ax.set_title(ttl + ", each aircraft flying its best pattern", loc="left", fontsize=10, color=INK)
        ax.grid(axis="y", color=GRID)
        ax.legend(frameon=False, fontsize=8.5, loc="upper right")
    ymax = max(a.get_ylim()[1] for a in axs)
    for a in axs:
        a.set_ylim(0, ymax * 1.08)
    fig.tight_layout()
    fig.savefig(OUT / "terrain.png", dpi=130)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("n", nargs="?", type=int, default=N_DEFAULT, help="hikers per site")
    parser.add_argument("--plot", action="store_true", help="redraw charts from a full fleet replay")
    parser.add_argument("--pattern", action="append", choices=PATTERNS, help="search pattern to run (repeatable)")
    parser.add_argument("--competitor-endurance", type=float, default=1.0,
                        help="fraction of published competitor endurance before altitude and reserve adjustments")
    parser.add_argument("--workers", type=int, help="maximum number of replay processes")
    args = parser.parse_args()
    if args.n < 1 or (args.workers is not None and args.workers < 1):
        parser.error("hikers and workers must be positive")
    if not 0 < args.competitor_endurance <= 1:
        parser.error("competitor endurance must be in (0, 1]")
    if args.plot:
        plot(args.n)
    else:
        main(args.n, list(dict.fromkeys(args.pattern or PATTERNS)), args.competitor_endurance, args.workers)
