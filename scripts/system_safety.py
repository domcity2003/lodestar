"""Step 2, items G3 and G4: motor-out glide reach, link-loss energy, flight termination and the C2 link budget
on the 7 real-terrain sites, for the trails-first search routes (3 km and 6 km).

G3 contingency (physical side)
  glide polar   propulsion_margins.drag (the mission / v3 drag model: NeuralFoil MH45 profile x finish factor,
                induced drag, pod), prop folded and treated as drag-free. The "planning" polar scales drag so that the
                best L/D is 15, the low end of the 15-19 planning range in research/2026-11-14-aero-crosscheck.md.
                If out/sixdof_v1/glide_polar.csv exists (6-DOF model), its best L/D is printed next to ours.
  reach         from each route point (terrain-following altitude, 120 m AGL), glide in a straight line to the launch
                point, or to an open landing area away from people, with a headwind of 0 / 5 / 10 m/s on whatever
                heading is flown (worst case), keeping CLEARANCE_M over the terrain the whole way, Earth flat.
  landing area  30 m cells with tree canopy < 10%, slope < 12 deg, not a flat water surface (3 x 3 relief >= 0.3 m),
                >= 150 m from every mapped trail and road, and a full 3 x 3 block (90 x 90 m) of such cells.
  link loss     energy to return home from the farthest point of each route with the bingo rule of
                lodestar_v1.OPERATIONS: 1.2 x E_return(d, wind + 3 m/s) + the 10% landing reserve, plus the climb over
                ridges on the way home (terrain_sim.return_climb).
  termination   impact energy with no parachute (glide, tumbling descent, steep dive from 120 m AGL) against a
                parachute descent; Part 107 Subpart D energy thresholds for context.

G4 C2 link budget
  radios        RFD900x 915 MHz telemetry (30 dBm, -105 dBm at 64 kbps) and Microhard pMDDL2450 2.45 GHz video
                (30 dBm, -98 dBm at QPSK 1/2, 8 MHz, 2x2 MIMO, about 5.9 Mbps). Antennas 2 dBi on the aircraft,
                5 dBi omni at the ground; 1 dB cable loss at each end. Link counts when the margin over
                sensitivity is >= FADE_MARGIN_DB after free-space loss and knife-edge diffraction loss.
  terrain       straight path over the DEM, Earth bulge with k = 4/3, the single worst obstruction as an ITU-R P.526
                knife edge (J(v)); also reports geometric line of sight and 60% first-Fresnel-zone clearance.
  stations      ground station at the launch point, mast 2 m or 10 m; optional relay (2 m mast) on a high point
                within RELAY_RADIUS_M of the launch point and RELAY_ACCESS_M of a trail or road that sees the 10 m
                station and adds the most C2 coverage of the 6 km route (relay_site).

Route: terrain_sim.prep(site, 500, 20261114) cached patterns, Lodestar camera spacing, resampled every 60 m along
the flown path and cut at ROUTE_KM (about the distance flown on one sortie at 18 m/s before the bingo point).

Run:  ../.venv/Scripts/python.exe -X utf8 system_safety.py      (from scripts/)
Writes: out/safety/glide_polar.csv, glide_reach.csv, link_budget.csv, link_loss_energy.csv, termination.csv,
        radio_budget.csv, safety_log.txt
"""
from __future__ import annotations

from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
from scipy.ndimage import binary_erosion, distance_transform_edt, maximum_filter, minimum_filter
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).parent))
import lodestar_v1 as v1  # noqa: E402
import terrain_sim as t  # noqa: E402  (read-only use)
import propulsion_margins as pm  # noqa: E402
from terrain_fetch import SITES, CELL, HALF, DATA  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "safety"

G = 9.81
C_LIGHT = 299_792_458.0
R_EARTH = 6_371_000.0
K_FACTOR = 4.0 / 3.0

PATTERNS = ("trails 3", "trails 6")
PREP_N, PREP_SEED = 500, 20261114
ROUTE_STEP_M = 60.0
ROUTE_KM = 175.0                 # 0.9 x 3.0 h x 18 m/s x 3.6
V_SEARCH = 18.0                  # m/s, terrain_sim.SEARCH_V for Lodestar
WINDS = (0.0, 5.0, 10.0)
CLEARANCE_M = 30.0               # over terrain on the glide path (tree tops about 25 m) and at arrival
LD_PLANNING = 15.0
GLIDE_STEP = 2                   # every 2nd route sample (120 m) for the glide-reach search
LZ_CANOPY, LZ_SLOPE_DEG, LZ_PEOPLE_M, LZ_RELIEF_M = 0.10, 12.0, 150.0, 0.3
NEAR_TRAIL_M = 50.0
LZ_MAX_TRIES = 12

M_LOADED = v1.MASS["sicore_loaded_kg"]
M_RELEASED = v1.MASS["sicore_released_kg"]
S_REF = v1.GEOMETRY["S_ref"]

# radios (datasheet values; see the system-safety note for links)
RADIOS = {
    "C2 900 MHz (RFD900x)": dict(f_hz=915e6, p_tx_dbm=30.0, sens_dbm=-105.0, rate="64 kbps"),
    "video 2.4 GHz (pMDDL2450)": dict(f_hz=2.45e9, p_tx_dbm=30.0, sens_dbm=-98.0, rate="5.9 Mbps (QPSK 1/2, 8 MHz, MIMO)"),
}
G_AIR_DBI, G_GND_DBI, L_CABLE_DB = 2.0, 5.0, 1.0
FADE_MARGIN_DB = 10.0
MASTS = (2.0, 10.0)
RELAY_MAST, RELAY_RADIUS_M, RELAY_ACCESS_M = 2.0, 5000.0, 100.0


# ---------------------------------------------------------------- radio helpers
def fspl_db(d_m, f_hz):
    """Free-space path loss, 20 log10(4 pi d / lambda)."""
    d = np.maximum(np.asarray(d_m, float), 1.0)
    return 20.0 * np.log10(4.0 * math.pi * d * f_hz / C_LIGHT)


def fresnel_r1(d1, d2, f_hz):
    lam = C_LIGHT / f_hz
    d1, d2 = np.asarray(d1, float), np.asarray(d2, float)
    return np.sqrt(lam * d1 * d2 / np.maximum(d1 + d2, 1e-9))


def knife_edge_db(v):
    """ITU-R P.526 single knife-edge loss J(v), 0 for v <= -0.78."""
    v = np.asarray(v, float)
    j = 6.9 + 20.0 * np.log10(np.sqrt((v - 0.1) ** 2 + 1.0) + v - 0.1)
    return np.where(v > -0.78, j, 0.0)


def path_clearance(dem, a, za, pts, zp, f_hz, step=CELL):
    """Terrain clearance from one station (a, za MSL) to many points (pts N x 2, zp MSL).

    Returns per point: los (geometric line of sight), f60 (60% first Fresnel zone clear), v_max (worst
    Fresnel-Kirchhoff parameter), dist (horizontal metres). Earth bulge uses k = 4/3. End samples excluded.
    """
    pts = np.atleast_2d(np.asarray(pts, float))
    zp = np.asarray(zp, float)
    dx, dy = pts[:, 0] - a[0], pts[:, 1] - a[1]
    dist = np.hypot(dx, dy)
    n = int(max(3, math.ceil(dist.max() / step) + 1)) if len(dist) else 3
    f = np.linspace(0.0, 1.0, n)[1:-1]
    los = np.ones(len(pts), bool)
    f60 = np.ones(len(pts), bool)
    vmax = np.full(len(pts), -np.inf)
    chunk = max(1, 2_000_000 // max(len(f), 1))
    for i0 in range(0, len(pts), chunk):
        sl = slice(i0, i0 + chunk)
        X = a[0] + np.outer(dx[sl], f)
        Y = a[1] + np.outer(dy[sl], f)
        d = dist[sl, None]
        d1, d2 = d * f, d * (1.0 - f)
        terrain = t.dem_at(dem, X, Y) + d1 * d2 / (2.0 * K_FACTOR * R_EARTH)
        line = za + (zp[sl, None] - za) * f
        h = line - terrain
        r1 = np.maximum(fresnel_r1(d1, d2, f_hz), 1e-6)
        los[sl] = np.all(h > 0.0, axis=1)
        f60[sl] = np.all(h >= 0.6 * r1, axis=1)
        vmax[sl] = np.max(-math.sqrt(2.0) * h / r1, axis=1)
    return dict(los=los, f60=f60, v_max=vmax, dist=dist)


def slant(dist, za, zp):
    return np.hypot(dist, np.asarray(zp) - za)


def link_margin_db(radio, slant_m, v_max):
    r = RADIOS[radio] if isinstance(radio, str) else radio
    p_rx = (r["p_tx_dbm"] + G_AIR_DBI + G_GND_DBI - 2 * L_CABLE_DB
            - fspl_db(slant_m, r["f_hz"]) - knife_edge_db(v_max))
    return p_rx - r["sens_dbm"]


def longest_run(mask):
    """Longest run of True values."""
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0
        best = max(best, cur)
    return best


# ---------------------------------------------------------------- glide helpers
def glide_polar(alt_m, m, drag_scale=1.0, t_c=15.0, v_hi=32.0):
    ai = pm.air(alt_m, t_c)
    vs = pm.v_stall(m, ai)
    V = np.arange(round(1.15 * vs, 2), v_hi + 1e-9, 0.1)
    D = drag_scale * pm.drag(V, m, ai)
    sink = D * V / (m * G)
    return V, sink, vs


def best_ld(V, sink):
    ld = V / sink
    i = int(np.argmax(ld))
    return float(ld[i]), float(V[i])


def ground_glide_ratio(V, sink, headwind):
    """Best ground distance per metre of height into a headwind (speed-to-fly); 0 if it cannot penetrate."""
    gr = (np.asarray(V) - headwind) / np.asarray(sink)
    i = int(np.argmax(gr))
    return max(float(gr[i]), 0.0), float(V[i])


def drag_scale_for(alt_m, m, target_ld):
    V, sink, _ = glide_polar(alt_m, m)
    return best_ld(V, sink)[0] / target_ld


def glide_path_clear(dem, p0, z0, p1, z_end_min, gr, clearance=CLEARANCE_M, step=CELL):
    """True if a straight glide p0 -> p1 at ground glide ratio gr stays >= clearance above terrain and arrives
    at p1 at or above z_end_min (MSL)."""
    d = math.dist(p0, p1)
    if gr <= 0:
        return False
    if z0 - d / gr < z_end_min:
        return False
    n = max(2, int(math.ceil(d / step)) + 1)
    f = np.linspace(0.0, 1.0, n)[1:]
    x = p0[0] + f * (p1[0] - p0[0])
    y = p0[1] + f * (p1[1] - p0[1])
    h = z0 - f * d / gr
    return bool(np.all(h - t.dem_at(dem, x, y) >= clearance))


# ---------------------------------------------------------------- site data
def route(site, pattern):
    P = t.prep(site, PREP_N, PREP_SEED)
    pat = P["pats"][(pattern, t.CAMERAS["Lodestar"][1])]
    xs, ys, zs = [], [], []
    for (p0, p1), pr in zip(pat["legs"], pat["prof"]):
        L = math.dist(p0, p1)
        s, alt = pr[0], pr[1]
        fr = s / L if L > 0 else np.zeros_like(s)
        xs.append(p0[0] + fr * (p1[0] - p0[0]))
        ys.append(p0[1] + fr * (p1[1] - p0[1]))
        zs.append(alt)
    X, Y, Z = map(np.concatenate, (xs, ys, zs))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(X), np.diff(Y)))])
    s_end = min(s[-1], ROUTE_KM * 1000.0)
    q = np.arange(0.0, s_end, ROUTE_STEP_M)
    # trail part ends at the first return to the trailhead (the leg into (0, 0))
    i_home = next(i for i, (a, b) in enumerate(pat["legs"]) if b == (0.0, 0.0))
    trail_km = sum(math.dist(a, b) for a, b in pat["legs"][:i_home + 1]) / 1000.0
    return dict(x=np.interp(q, s, X), y=np.interp(q, s, Y), z=np.interp(q, s, Z), s=q,
                z_home=P["z_home"], trail_km=trail_km)


def people_distance(dem, lines):
    """Distance [m] from each cell to the nearest mapped trail or road."""
    n = dem.shape[0]
    mask = np.zeros((n, n), bool)
    for l in lines:
        if l["kind"] == "drainage":
            continue
        x, y = t.densify(l, 10.0)
        r, c = t.rc(x, y, n)
        r, c = np.round(r).astype(int), np.round(c).astype(int)
        ok = (r >= 0) & (r < n) & (c >= 0) & (c < n)
        mask[r[ok], c[ok]] = True
    return distance_transform_edt(~mask) * CELL


def landing_zones(dem, canopy, dist_people):
    gy, gx = np.gradient(dem, CELL)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    relief = maximum_filter(dem, 3) - minimum_filter(dem, 3)
    ok = (canopy < LZ_CANOPY) & (slope < LZ_SLOPE_DEG) & (relief >= LZ_RELIEF_M) & (dist_people >= LZ_PEOPLE_M)
    ok = binary_erosion(ok, np.ones((3, 3), bool))
    sub = np.zeros_like(ok)
    sub[::3, ::3] = ok[::3, ::3]
    r, c = np.nonzero(sub)
    x = (c + 0.5) * CELL - HALF
    y = HALF - (r + 0.5) * CELL
    return np.column_stack([x, y]), dem[r, c], float(ok.mean())


def sample_raster(grid, x, y):
    n = grid.shape[0]
    r, c = t.rc(x, y, n)
    r = np.clip(np.round(r).astype(int), 0, n - 1)
    c = np.clip(np.round(c).astype(int), 0, n - 1)
    return grid[r, c]


# ---------------------------------------------------------------- G3: glide reach
def glide_reach(site, rt, dem, lz_xy, lz_z, tree, grs):
    """Per wind: fraction of route samples that can glide home, or home-or-landing-area."""
    idx = np.arange(0, len(rt["x"]), GLIDE_STEP)
    home = (0.0, 0.0)
    z_home_min = rt["z_home"] + CLEARANCE_M
    out = {}
    for w, gr in grs.items():
        h_ok = np.zeros(len(idx), bool)
        a_ok = np.zeros(len(idx), bool)
        for j, i in enumerate(idx):
            p0, z0 = (rt["x"][i], rt["y"][i]), rt["z"][i]
            if glide_path_clear(dem, p0, z0, home, z_home_min, gr):
                h_ok[j] = a_ok[j] = True
                continue
            if not len(lz_z):
                continue
            rmax = max(0.0, (z0 - lz_z.min() - CLEARANCE_M) * gr)
            cand = tree.query_ball_point(p0, rmax)
            if not cand:
                continue
            cand = np.asarray(cand)
            cand = cand[(z0 - np.hypot(lz_xy[cand, 0] - p0[0], lz_xy[cand, 1] - p0[1]) / gr) >= lz_z[cand] + CLEARANCE_M]
            if not len(cand):
                continue
            order = cand[np.argsort(np.hypot(lz_xy[cand, 0] - p0[0], lz_xy[cand, 1] - p0[1]))][:LZ_MAX_TRIES]
            for k in order:
                if glide_path_clear(dem, p0, z0, tuple(lz_xy[k]), lz_z[k] + CLEARANCE_M, gr):
                    a_ok[j] = True
                    break
        out[w] = (h_ok, a_ok)
    return idx, out


# ---------------------------------------------------------------- G3: link-loss energy
def link_loss_energy(site, rt, dem, wh_usable, cell, m=M_LOADED):
    d = np.hypot(rt["x"], rt["y"])
    i = int(np.argmax(d))
    dmax = float(d[i])
    ai = pm.air(rt["z_home"] + t.H_SEARCH, 20.0)
    vmax = pm.v_max_level(m, ai, cell, 20.0, pm.SOC_END + 0.10)
    climb = t.return_climb(dem, (rt["x"][i], rt["y"][i]), rt["z"][i], rt["z_home"] + t.H_SEARCH)
    k_climb = m * G / pm.sz.A.eta_max / 3600.0
    rows = []
    for w in WINDS:
        b = pm.rth_energy(dmax, w + 3.0, m, ai, cell, 20.0, vmax)
        e_ret = b[0] + k_climb * climb
        bingo = 1.2 * e_ret + v1.OPERATIONS["landing_reserve_frac"] * wh_usable
        rows.append(dict(site=site, d_max_km=dmax / 1000, return_climb_m=climb, headwind=w,
                         v_return=b[1], t_return_min=b[3], E_return_Wh=e_ret, bingo_Wh=bingo,
                         bingo_pct_usable=100 * bingo / wh_usable))
    return rows


# ---------------------------------------------------------------- G3: termination
def termination_table(alt_m=2000.0, h_agl=t.H_SEARCH):
    ai = pm.air(alt_m, 15.0)
    rho = ai["rho"]
    rows = []
    for label, m in (("kit on", M_LOADED), ("kit released", M_RELEASED)):
        V, sink, vs = glide_polar(alt_m, m)
        ld, vbg = best_ld(V, sink)
        rows.append(dict(case="controlled glide, belly landing at best-glide speed", mass=label, v_impact=vbg,
                         ke_J=0.5 * m * vbg**2, note="aircraft controlled; impact point chosen"))
        # tumbling or flat-spin descent: drag coefficient about 1.0 on the planform area (assumed)
        v_t = math.sqrt(2 * m * G / (rho * 1.0 * S_REF))
        rows.append(dict(case="tumbling or flat-spin descent (CD about 1.0 on planform)", mass=label, v_impact=v_t,
                         ke_J=0.5 * m * v_t**2, note="loss of control; impact within about 100 m of the track"))
        # steep dive from h_agl with near-zero-lift drag CD about 0.02 on S_ref
        v, z, dt = vbg, h_agl, 0.01
        k = 0.5 * rho * 0.02 * S_REF / m
        while z > 0:
            v += (G - k * v * v) * dt
            z -= v * dt
        rows.append(dict(case=f"steep dive from {h_agl:.0f} m AGL (CD0 about 0.02)", mass=label, v_impact=v,
                         ke_J=0.5 * m * v**2, note="worst case: control lost nose-down"))
        for vd in (5.0, 6.7):
            area = 2 * m * G / (rho * 0.8 * vd**2)
            rows.append(dict(case=f"terminal parachute, {vd:.1f} m/s (CD 0.8, {area:.1f} m2, "
                                  f"{2 * math.sqrt(area / math.pi):.1f} m flat diameter)",
                             mass=label, v_impact=vd, ke_J=0.5 * m * vd**2, note="not on v1"))
    df = pd.DataFrame(rows)
    df["x_cat3_34J"] = df.ke_J / 33.9      # 107.130: 25 ft-lb
    return df


# ---------------------------------------------------------------- G4: link budget
def relay_site(dem, dist_people, rt, n_cand=40, grid=10):
    """Relay site: among the highest cells within RELAY_RADIUS_M of the launch point and within RELAY_ACCESS_M of a
    trail or road (so a team can carry the relay there), sampled on a 300 m grid, keep those whose 900 MHz link to
    the 10 m ground station has line of sight and the fade margin, and pick the one that adds the most C2 coverage
    of the given route to the 10 m station alone. Falls back to the highest candidate."""
    n = dem.shape[0]
    xc = (np.arange(n) + 0.5) * CELL - HALF
    X, Y = np.meshgrid(xc, HALF - (np.arange(n) + 0.5) * CELL)
    m = (np.hypot(X, Y) <= RELAY_RADIUS_M) & (dist_people <= RELAY_ACCESS_M)
    m[np.arange(n) % grid != 0, :] = False
    m[:, np.arange(n) % grid != 0] = False
    cand = np.flatnonzero(m.ravel())
    cand = cand[np.argsort(dem.ravel()[cand])[::-1][:n_cand]]
    rr = RADIOS["C2 900 MHz (RFD900x)"]
    z_g = float(t.dem_at(dem, 0.0, 0.0)) + 10.0
    pts = np.column_stack([rt["x"], rt["y"]])
    c0 = path_clearance(dem, (0.0, 0.0), z_g, pts, rt["z"], rr["f_hz"])
    base = link_margin_db(rr, slant(c0["dist"], z_g, rt["z"]), c0["v_max"]) >= FADE_MARGIN_DB
    best, best_gain = None, -1.0
    for i in cand:
        xy, zr = (float(X.ravel()[i]), float(Y.ravel()[i])), float(dem.ravel()[i]) + RELAY_MAST
        cg = path_clearance(dem, (0.0, 0.0), z_g, np.array([xy]), np.array([zr]), rr["f_hz"])
        mg = link_margin_db(rr, slant(cg["dist"], z_g, np.array([zr])), cg["v_max"])[0]
        if not (cg["los"][0] and mg >= FADE_MARGIN_DB):
            continue
        c = path_clearance(dem, xy, zr, pts, rt["z"], rr["f_hz"])
        ok = link_margin_db(rr, slant(c["dist"], zr, rt["z"]), c["v_max"]) >= FADE_MARGIN_DB
        gain = float((ok & ~base).mean())
        if gain > best_gain:
            best, best_gain = (xy, zr - RELAY_MAST), gain
    if best is None:
        i = cand[0]
        best = ((float(X.ravel()[i]), float(Y.ravel()[i])), float(dem.ravel()[i]))
    return best


def link_runs(site, rt, dem, relay):
    pts = np.column_stack([rt["x"], rt["y"]])
    rows, masks = [], {}
    gs_xy = (0.0, 0.0)
    z_g0 = float(t.dem_at(dem, 0.0, 0.0))
    r_xy, r_z = relay
    for radio, rr in RADIOS.items():
        f = rr["f_hz"]
        res, strict = {}, {}
        for mast in MASTS:
            za = z_g0 + mast
            c = path_clearance(dem, gs_xy, za, pts, rt["z"], f)
            marg = link_margin_db(rr, slant(c["dist"], za, rt["z"]), c["v_max"])
            res[f"GS {mast:g} m"] = (c, marg >= FADE_MARGIN_DB, marg)
            strict[f"GS {mast:g} m"] = c["los"] & (marg >= FADE_MARGIN_DB)
        # relay: aircraft to relay, and relay to the 10 m ground station
        zr = r_z + RELAY_MAST
        c = path_clearance(dem, r_xy, zr, pts, rt["z"], f)
        marg = link_margin_db(rr, slant(c["dist"], zr, rt["z"]), c["v_max"])
        cg = path_clearance(dem, gs_xy, z_g0 + 10.0, np.array([r_xy]), np.array([zr]), f)
        mg = float(link_margin_db(rr, slant(cg["dist"], z_g0 + 10.0, np.array([zr])), cg["v_max"])[0])
        relay_ok = mg >= FADE_MARGIN_DB
        ok10 = res["GS 10 m"][1]
        res["GS 10 m + relay"] = (c, ok10 | ((marg >= FADE_MARGIN_DB) & relay_ok), np.maximum(res["GS 10 m"][2], marg))
        relay_strict = relay_ok and bool(cg["los"][0])
        strict["GS 10 m + relay"] = strict["GS 10 m"] | (c["los"] & (marg >= FADE_MARGIN_DB) & relay_strict)
        for cfg, (c, ok, marg) in res.items():
            run = longest_run(~ok)
            st = strict[cfg]
            rows.append(dict(site=site, radio=radio, station=cfg,
                             los_frac=float(c["los"].mean()) if cfg != "GS 10 m + relay" else np.nan,
                             f60_frac=float(c["f60"].mean()) if cfg != "GS 10 m + relay" else np.nan,
                             link_frac=float(ok.mean()),
                             link_frac_los_only=float(st.mean()),
                             longest_outage_los_only_min=longest_run(~st) * ROUTE_STEP_M / V_SEARCH / 60.0,
                             longest_outage_min=run * ROUTE_STEP_M / V_SEARCH / 60.0,
                             n_outages=int(np.sum(np.diff(np.concatenate([[0], (~ok).astype(int)])) == 1)),
                             max_range_km=float(c["dist"].max() / 1000),
                             relay_x=r_xy[0], relay_y=r_xy[1], relay_dist_km=math.hypot(*r_xy) / 1000,
                             relay_rise_m=r_z - z_g0, relay_to_gs_margin_db=mg))
            masks[(radio, cfg)] = ok
    return rows, masks


def radio_budget_table():
    rows = []
    for radio, r in RADIOS.items():
        for d_km in (1, 5, 10, 15):
            fs = float(fspl_db(d_km * 1000, r["f_hz"]))
            m = float(link_margin_db(r, d_km * 1000, -10.0))
            rows.append(dict(radio=radio, distance_km=d_km, fspl_db=fs,
                             p_rx_dbm=m + r["sens_dbm"], margin_db=m, sens_dbm=r["sens_dbm"], rate=r["rate"]))
        # range at which the free-space margin equals the fade margin
        budget = r["p_tx_dbm"] + G_AIR_DBI + G_GND_DBI - 2 * L_CABLE_DB - r["sens_dbm"] - FADE_MARGIN_DB
        d = C_LIGHT / (4 * math.pi * r["f_hz"]) * 10 ** (budget / 20)
        rows.append(dict(radio=radio, distance_km=d / 1000, fspl_db=budget, p_rx_dbm=np.nan,
                         margin_db=FADE_MARGIN_DB, sens_dbm=r["sens_dbm"], rate="free-space range at fade margin"))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------- main
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    sys.stdout = t.Tee(OUT / "safety_log.txt")
    print("Lodestar v1 system safety: G3 glide / link-loss / termination, G4 C2 link budget")
    print(f"route cut at {ROUTE_KM:.0f} km, sampled every {ROUTE_STEP_M:.0f} m; glide clearance {CLEARANCE_M:.0f} m")

    # ---- glide polar (2,000 m design altitude) and the 6-DOF cross-check
    polar_rows = []
    for label, m in (("kit on", M_LOADED), ("kit released", M_RELEASED)):
        for alt in (0.0, 2000.0, 4400.0):
            V, sink, vs = glide_polar(alt, m)
            ld, vbg = best_ld(V, sink)
            scale = ld / LD_PLANNING
            for w in WINDS:
                gr_n, v_n = ground_glide_ratio(V, sink, w)
                gr_p, v_p = ground_glide_ratio(V, sink * scale, w)
                polar_rows.append(dict(mass=label, alt_m=alt, v_stall=vs, ld_model=ld, v_best_glide=vbg,
                                       sink_best_glide=float(sink[np.argmax(V / sink)]), min_sink=float(sink.min()),
                                       headwind=w, gr_ground_model=gr_n, v_fly_model=v_n,
                                       gr_ground_planning=gr_p, v_fly_planning=v_p,
                                       range_km_from_120m_planning=gr_p * t.H_SEARCH / 1000))
    polar = pd.DataFrame(polar_rows)
    polar.to_csv(OUT / "glide_polar.csv", index=False)
    print("\nGlide polar (drag model; planning polar scaled to L/D 15):")
    print(polar.round(2).to_string(index=False))
    six = ROOT / "out" / "sixdof_v1" / "glide_polar.csv"
    if six.exists():
        try:
            g = pd.read_csv(six)
            cols = {c.lower(): c for c in g.columns}
            ldc = next((cols[c] for c in cols if "l/d" in c or c in ("ld", "l_d", "lift_to_drag")), None)
            if ldc is not None:
                ld_ref = polar[(polar.mass == "kit on") & (polar.alt_m == 2000.0)].ld_model.iloc[0]
                print(f"\n6-DOF cross-check ({six.name}): best L/D {g[ldc].max():.2f} vs model {ld_ref:.2f}")
            else:
                print(f"\n6-DOF file present ({six}); columns {list(g.columns)}: compare by hand")
        except Exception as e:  # noqa: BLE001
            print(f"\n6-DOF file present but unreadable: {e}")
    else:
        print("\n6-DOF glide polar (out/sixdof_v1/glide_polar.csv) not present yet: cross-check pending.")

    # ---- termination
    term = termination_table()
    term.to_csv(OUT / "termination.csv", index=False)
    print("\nTermination impact energy (2,000 m):")
    print(term.round(1).to_string(index=False))

    # ---- radios
    rb = radio_budget_table()
    rb.to_csv(OUT / "radio_budget.csv", index=False)
    print("\nRadio budget, free space:")
    print(rb.round(1).to_string(index=False))

    cell_name = "Amprius SiCore 6.3 Ah"
    cell = pm.CELLS[cell_name]
    _, wh = pm.baseline(cell_name)
    wh_usable = wh * 0.85
    glide_rows, link_rows, energy_rows = [], [], []
    for site in SITES:
        dem, lines, _ = t.load(site)
        canopy = np.load(DATA / site / "canopy.npy").astype(float)
        dpeople = people_distance(dem, lines)
        lz_xy, lz_z, lz_frac = landing_zones(dem, canopy, dpeople)
        tree = cKDTree(lz_xy) if len(lz_xy) else None
        relay = relay_site(dem, dpeople, route(site, "trails 6"))
        for pattern in PATTERNS:
            rt = route(site, pattern)
            alt = rt["z_home"] + t.H_SEARCH
            near = sample_raster(dpeople, rt["x"], rt["y"]) <= NEAR_TRAIL_M
            for polar_name in ("planning L/D 15", "model polar"):
                V, sink, _ = glide_polar(alt, M_LOADED)
                if polar_name.startswith("planning"):
                    sink = sink * best_ld(V, sink)[0] / LD_PLANNING
                grs = {w: ground_glide_ratio(V, sink, w)[0] for w in WINDS}
                idx, res = glide_reach(site, rt, dem, lz_xy, lz_z, tree, grs)
                tr = rt["s"][idx] <= rt["trail_km"] * 1000.0
                for w, (h_ok, a_ok) in res.items():
                    glide_rows.append(dict(site=site, pattern=pattern, polar=polar_name, headwind=w,
                                           gr_ground=grs[w], home_frac=float(h_ok.mean()),
                                           home_or_lz_frac=float(a_ok.mean()),
                                           home_frac_trail_part=float(h_ok[tr].mean()),
                                           home_or_lz_frac_trail_part=float(a_ok[tr].mean()),
                                           near_trail_frac=float(near.mean()),
                                           lz_area_frac=lz_frac, route_km=rt["s"][-1] / 1000,
                                           trail_km=rt["trail_km"]))
            lr, masks = link_runs(site, rt, dem, relay)
            tr = rt["s"] <= rt["trail_km"] * 1000.0
            for r in lr:
                r["pattern"] = pattern
                r["link_frac_trail_part"] = float(masks[(r["radio"], r["station"])][tr].mean())
            link_rows += lr
            for r in link_loss_energy(site, rt, dem, wh_usable, cell):
                r["pattern"] = pattern
                energy_rows.append(r)
            print(f"  {site:26s} {pattern}: route {rt['s'][-1] / 1000:5.1f} km, "
                  f"glide home (L/D 15, still air) {glide_rows[-6]['home_frac']:.2f}, "
                  f"home or LZ {glide_rows[-6]['home_or_lz_frac']:.2f}; "
                  f"C2 link GS 2 m {lr[0]['link_frac']:.2f}, 10 m {lr[1]['link_frac']:.2f}, relay {lr[2]['link_frac']:.2f}",
                  flush=True)

    gdf = pd.DataFrame(glide_rows)
    ldf = pd.DataFrame(link_rows)
    edf = pd.DataFrame(energy_rows)
    gdf.to_csv(OUT / "glide_reach.csv", index=False)
    ldf.to_csv(OUT / "link_budget.csv", index=False)
    edf.to_csv(OUT / "link_loss_energy.csv", index=False)

    print("\nGlide reach, mean over the 7 sites:")
    print(gdf.groupby(["polar", "pattern", "headwind"])[["home_frac", "home_or_lz_frac", "near_trail_frac"]]
          .mean().round(3).to_string())
    print("\nGlide reach over the trail part of the route only, mean over the 7 sites:")
    print(gdf.groupby(["polar", "pattern", "headwind"])[["home_frac_trail_part", "home_or_lz_frac_trail_part"]]
          .mean().round(3).to_string())
    print("\nGlide reach by site (planning L/D 15):")
    print(gdf[gdf.polar.str.startswith("planning")].pivot_table(
        index=["site", "pattern"], columns="headwind", values=["home_frac", "home_or_lz_frac"]).round(2).to_string())
    print("\nLink, by site:")
    print(ldf.pivot_table(index=["site", "pattern"], columns=["radio", "station"], values="link_frac").round(2).to_string())
    print("\nLongest outage [min] at 18 m/s:")
    print(ldf.pivot_table(index=["site", "pattern"], columns=["radio", "station"],
                          values="longest_outage_min").round(1).to_string())
    print("\nLink with geometric line of sight required (no diffraction credit):")
    print(ldf.pivot_table(index=["site", "pattern"], columns=["radio", "station"],
                          values="link_frac_los_only").round(2).to_string())
    print("\nLongest outage [min], line of sight required:")
    print(ldf.pivot_table(index=["site", "pattern"], columns=["radio", "station"],
                          values="longest_outage_los_only_min").round(1).to_string())
    print("\nLink, mean over sites:")
    print(ldf.groupby(["radio", "station", "pattern"])[["los_frac", "f60_frac", "link_frac", "longest_outage_min",
                                                         "link_frac_los_only", "longest_outage_los_only_min"]]
          .mean().round(3).to_string())
    print("\nRelay sites and C2 link over the trail part only:")
    print(ldf[ldf.radio.str.startswith("C2")].pivot_table(index=["site", "pattern"], columns="station",
                                                          values="link_frac_trail_part").round(2).to_string())
    print(ldf.drop_duplicates("site")[["site", "relay_dist_km", "relay_rise_m", "relay_to_gs_margin_db"]]
          .round(1).to_string(index=False))
    print("\nLink-loss return energy (bingo rule, farthest route point):")
    print(edf.round(2).to_string(index=False))
    print(f"\nusable energy {wh_usable:.1f} Wh (SiCore, 85% depth of discharge)")


if __name__ == "__main__":
    main()
