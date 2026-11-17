"""Known-answer checks for system_safety.py (G3 glide and termination, G4 link budget).

Run:  ../.venv/Scripts/python.exe -X utf8 test_system_safety.py      (from scripts/)
Synthetic 30 x 30 km maps: flat ground at 1000 m, optionally with a 400 m ridge along x = 3 km.
"""
from pathlib import Path
import math
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import system_safety as ss  # noqa: E402
import terrain_sim as t  # noqa: E402

N = int(2 * t.HALF / t.CELL)
X_COL = (np.arange(N) + 0.5) * t.CELL - t.HALF


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0


def flat(z=1000.0):
    return np.full((N, N), z)


def ridge(height=400.0, x0=3000.0, half_width=150.0):
    dem = flat()
    dem[:, np.abs(X_COL - x0) <= half_width] += height
    return dem


def test_free_space():
    # FSPL = 20 log10(d_km) + 20 log10(f_MHz) + 32.45
    for d_km, f_mhz in ((1.0, 915.0), (10.0, 2450.0), (5.0, 915.0)):
        ref = 20 * math.log10(d_km) + 20 * math.log10(f_mhz) + 32.45
        got = float(ss.fspl_db(d_km * 1000, f_mhz * 1e6))
        check(f"free-space path loss {d_km:g} km, {f_mhz:g} MHz = {ref:.2f} dB", abs(got - ref) < 0.02, f"{got:.2f}")
    # 6 dB per doubling of distance
    check("free-space loss rises 6.02 dB per doubling",
          abs(float(ss.fspl_db(2000, 915e6) - ss.fspl_db(1000, 915e6)) - 6.02) < 0.01)
    # link margin at 10 km, 900 MHz: 30 + 2 + 5 - 2 - 111.68 + 105 = 28.3 dB
    m = float(ss.link_margin_db("C2 900 MHz (RFD900x)", 10_000.0, -10.0))
    check("900 MHz free-space margin at 10 km = 28.3 dB", abs(m - 28.3) < 0.1, f"{m:.2f} dB")


def test_fresnel_and_knife_edge():
    lam = ss.C_LIGHT / 915e6
    r = float(ss.fresnel_r1(5000.0, 5000.0, 915e6))
    ref = 0.5 * math.sqrt(lam * 10_000.0)
    check("first Fresnel radius at mid-path, 10 km, 915 MHz = 28.6 m", abs(r - ref) < 0.01 and abs(r - 28.6) < 0.1,
          f"{r:.2f} m")
    j0 = float(ss.knife_edge_db(0.0))
    check("knife-edge loss at grazing (v = 0) = 6.0 dB (ITU-R P.526)", abs(j0 - 6.03) < 0.05, f"{j0:.2f} dB")
    check("knife-edge loss is zero for v <= -0.78", float(ss.knife_edge_db(-1.0)) == 0.0)
    j = float(ss.knife_edge_db(2.4))
    check("knife-edge loss for v = 2.4 near 13 + 20 log10(v)", abs(j - (13 + 20 * math.log10(2.4))) < 0.6, f"{j:.2f} dB")


def test_line_of_sight():
    dem = flat()
    pts = np.array([[5000.0, 0.0], [0.0, 8000.0]])
    c = ss.path_clearance(dem, (0.0, 0.0), 1002.0, pts, np.array([1120.0, 1120.0]), 915e6)
    check("flat DEM: line of sight clear from a 2 m mast to 120 m AGL at 5 and 8 km", bool(c["los"].all()))
    # 2 m mast: the 60% zone is just clear at 5 km but grazes the ground about 130 m from the mast at 8 km
    # (hand value: 3.91 m clearance vs 0.6 r1 = 3.94 m); a 10 m mast clears both
    check("flat DEM, 2 m mast: 60% first Fresnel zone clear at 5 km, not at 8 km (915 MHz)",
          bool(c["f60"][0]) and not bool(c["f60"][1]))
    c10 = ss.path_clearance(dem, (0.0, 0.0), 1010.0, pts, np.array([1120.0, 1120.0]), 915e6)
    check("flat DEM, 10 m mast: 60% first Fresnel zone clear at 5 and 8 km", bool(c10["f60"].all()))
    check("flat DEM: no diffraction loss", float(ss.knife_edge_db(c["v_max"]).max()) == 0.0)
    dem = ridge()
    c = ss.path_clearance(dem, (0.0, 0.0), 1002.0, np.array([[6000.0, 0.0]]), np.array([1120.0]), 915e6)
    check("400 m ridge at 3 km blocks the path to 6 km", not bool(c["los"][0]) and not bool(c["f60"][0]))
    loss = float(ss.knife_edge_db(c["v_max"])[0])
    check("blocked path has a large diffraction loss (> 25 dB)", loss > 25.0, f"{loss:.1f} dB")
    m = float(ss.link_margin_db("C2 900 MHz (RFD900x)", 6000.0, c["v_max"][0]))
    check("ridge pushes the 900 MHz margin below the 10 dB fade margin", m < ss.FADE_MARGIN_DB, f"{m:.1f} dB")
    # same ridge, aircraft on the near side: clear
    c = ss.path_clearance(dem, (0.0, 0.0), 1002.0, np.array([[2000.0, 0.0]]), np.array([1120.0]), 915e6)
    check("same ridge, aircraft on the near side: clear", bool(c["los"][0]))
    # Earth bulge: two 2 m masts 15 km apart on flat ground; mid-path bulge 7.5^2 / (2 x 4/3 x 6371) km = 3.3 m
    c = ss.path_clearance(flat(), (-7500.0, 0.0), 1002.0, np.array([[7500.0, 0.0]]), np.array([1002.0]), 915e6)
    check("Earth bulge (k = 4/3) blocks two 2 m masts 15 km apart", not bool(c["los"][0]))


def test_glide():
    V, sink, vs = ss.glide_polar(2000.0, ss.M_LOADED)
    ld, vbg = ss.best_ld(V, sink)
    gr0, v0 = ss.ground_glide_ratio(V, sink, 0.0)
    check("still-air ground glide ratio equals best L/D", abs(gr0 - ld) < 1e-9 and v0 == vbg, f"L/D {ld:.2f}")
    check("model best L/D inside the 15-19 planning range", 15.0 <= ld <= 19.0, f"{ld:.2f}")
    scale = ss.drag_scale_for(2000.0, ss.M_LOADED, ss.LD_PLANNING)
    gr_p, _ = ss.ground_glide_ratio(V, sink * scale, 0.0)
    check("planning polar has best L/D 15", abs(gr_p - 15.0) < 1e-6, f"{gr_p:.3f}")
    grs = [ss.ground_glide_ratio(V, sink, w)[0] for w in (0.0, 5.0, 10.0)]
    vfs = [ss.ground_glide_ratio(V, sink, w)[1] for w in (0.0, 5.0, 10.0)]
    check("headwind lowers the ground glide ratio and raises speed-to-fly",
          grs[0] > grs[1] > grs[2] > 0 and vfs[0] < vfs[1] < vfs[2], f"{[round(g, 2) for g in grs]}")
    # flat ground: from 120 m AGL, keeping 30 m clearance, the glide reaches (120 - 30) x L/D
    dem = flat()
    reach = (120.0 - ss.CLEARANCE_M) * 15.0
    ok_in = ss.glide_path_clear(dem, (0.0, 0.0), 1120.0, (0.99 * reach, 0.0), 1000.0 + ss.CLEARANCE_M, 15.0)
    ok_out = ss.glide_path_clear(dem, (0.0, 0.0), 1120.0, (1.01 * reach, 0.0), 1000.0 + ss.CLEARANCE_M, 15.0)
    check("flat ground: glide distance = height above the arrival floor x L/D (1,350 m at L/D 15)", ok_in and not ok_out)
    check("glide into a 400 m ridge is blocked even inside still-air range",
          not ss.glide_path_clear(ridge(), (2500.0, 0.0), 1120.0, (3500.0, 0.0), 1000.0 + ss.CLEARANCE_M, 15.0))
    check("no penetration (ground glide ratio 0) cannot reach", not ss.glide_path_clear(dem, (0, 0), 1120.0, (10, 0), 0, 0.0))


def test_termination():
    df = ss.termination_table()
    ke_ok = np.allclose(df.ke_J, 0.5 * np.where(df.mass == "kit on", ss.M_LOADED, ss.M_RELEASED) * df.v_impact**2)
    check("impact energy = 1/2 m v^2 on every row", bool(ke_ok))
    tum = df[df.case.str.startswith("tumbling") & (df.mass == "kit on")].iloc[0]
    rho2000 = ss.pm.air(2000.0, 15.0)["rho"]
    v_ref = math.sqrt(2 * ss.M_LOADED * ss.G / (rho2000 * 1.0 * ss.S_REF))
    check("tumbling descent speed = sqrt(2 m g / (rho CD S))", abs(tum.v_impact - v_ref) < 1e-9, f"{v_ref:.2f} m/s")
    dive = df[df.case.str.startswith("steep dive") & (df.mass == "kit on")].iloc[0]
    v0 = df[df.case.str.startswith("controlled") & (df.mass == "kit on")].iloc[0].v_impact
    check("dive from 120 m: slower than drag-free fall, faster than the start",
          v0 < dive.v_impact < math.sqrt(v0**2 + 2 * ss.G * 120.0), f"{dive.v_impact:.1f} m/s")
    check("every no-parachute case exceeds the Part 107 Category 3 limit (25 ft-lb, 33.9 J)",
          bool((df[~df.case.str.startswith("terminal")].ke_J > 33.9).all()))


def test_landing_zones_and_helpers():
    # gently tilted plane (2 deg), no canopy, no trails: open landing area everywhere except the map edge
    slope_dem = 1000.0 + np.tan(np.radians(2.0)) * (X_COL[None, :] + t.HALF) * np.ones((N, 1))
    far = np.full((N, N), 1e6)
    xy, z, frac = ss.landing_zones(slope_dem, np.zeros((N, N)), far)
    check("open 2 deg slope: landing areas found", len(z) > 1000 and frac > 0.9, f"{frac:.3f}")
    _, _, frac_c = ss.landing_zones(slope_dem, np.full((N, N), 0.5), far)
    check("50% tree canopy: no landing areas", frac_c == 0.0)
    _, _, frac_w = ss.landing_zones(flat(), np.zeros((N, N)), far)
    check("perfectly flat surface (water): no landing areas", frac_w == 0.0)
    lines = [dict(kind="trail", x=[-15000.0, 15000.0], y=[0.0, 0.0])]
    d = ss.people_distance(flat(), lines)
    check("distance to a trail along y = 0 is about |y|", abs(float(t.dem_at(d, 0.0, 3000.0)) - 3000.0) < 31.0)
    _, _, frac_t = ss.landing_zones(slope_dem, np.zeros((N, N)), d)
    check("cells within 150 m of the trail are excluded", frac_t < frac)
    check("longest outage run", ss.longest_run([False, True, True, False, True, True, True, False]) == 3
          and ss.longest_run([]) == 0)


def test_route():
    rt = ss.route("rocky_bear_lake", "trails 3")
    step = np.diff(rt["s"])
    check("route resampled every 60 m and cut at the sortie distance",
          np.allclose(step, ss.ROUTE_STEP_M) and rt["s"][-1] <= ss.ROUTE_KM * 1000.0)
    check("route starts at the launch point", math.hypot(rt["x"][0], rt["y"][0]) < 1.0)
    dem = t.load("rocky_bear_lake")[0]
    agl = rt["z"] - t.dem_at(dem, rt["x"], rt["y"])
    check("route altitude is at least about 120 m AGL (terrain following)", float(np.percentile(agl, 1)) > 110.0,
          f"1st percentile {np.percentile(agl, 1):.0f} m")


if __name__ == "__main__":
    for f in (test_free_space, test_fresnel_and_knife_edge, test_line_of_sight, test_glide, test_termination,
              test_landing_zones_and_helpers, test_route):
        print(f"--- {f.__name__}")
        f()
    print(f"\n{check.failed} failed")
    sys.exit(1 if check.failed else 0)
