"""Checks for the terrain-aware flight, camera and line-of-sight helpers in terrain_sim.py.

Run:  uv run python -X utf8 scripts/test_terrain_sim.py
Synthetic 30 x 30 km maps: flat ground at 1000 m, optionally with a 400 m ridge along x = 3 km.
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import terrain_sim as t  # noqa: E402
import search_patterns as sp  # noqa: E402
import v3_blur as vb  # noqa: E402

N = int(2 * t.HALF / t.CELL)
X_COL = (np.arange(N) + 0.5) * t.CELL - t.HALF          # x of each column centre


def flat():
    return np.full((N, N), 1000.0)


def ridge(height=400.0, x0=3000.0, half_width=150.0):
    dem = flat()
    dem[:, np.abs(X_COL - x0) <= half_width] += height
    return dem


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0


def test_clearance_profile():
    dem = ridge()
    legs = [((0.0, 0.0), (6000.0, 0.0))]
    S, alt, up, dz = t.profile(dem, legs)[0]
    ground = t.dem_at(dem, S, np.zeros_like(S))
    grad = np.diff(alt) / np.diff(S)
    check("path never below 120 m over ground", np.all(alt >= ground + t.H_SEARCH - 1e-6),
          f"min clearance {np.min(alt - ground):.1f} m")
    check("climb gradient <= 15%", grad.max() <= t.CLIMB_GRAD + 1e-9, f"max {grad.max():.3f}")
    check("descent gradient <= 20%", -grad.min() <= t.DESC_GRAD + 1e-9, f"max {-grad.min():.3f}")
    start_climb = S[np.argmax(alt > alt[0] + 1)]
    check("starts climbing before the ridge", start_climb < 3000 - 150 - 400 / t.CLIMB_GRAD + 100,
          f"climb starts at {start_climb:.0f} m, ridge face at 2850 m")
    check("total climb ~ ridge height", abs(up - 400) < 20, f"{up:.0f} m")


def test_partial_leg_energy():
    distance = np.array([0.0, 100.0, 200.0])
    altitude = np.array([120.0, 140.0, 118.0])
    energy = lambda stop: t.partial_leg_energy(distance, altitude, stop, 0.06, 1.0, 0.01)
    check("departure point consumes no leg energy", energy(0) == 0)
    check("mid-climb detection pays the actual climb", math.isclose(energy(50), 13))
    check("descent retains the idle-power floor", math.isclose(energy(150), 26.5))
    check("end-of-leg energy accounts for both gradients", math.isclose(energy(200), 27))


def test_return_climb():
    dem = ridge()
    z_home = 1000.0 + t.H_SEARCH
    rc = t.return_climb(dem, (6000.0, 0.0), z_home, z_home)
    check("return over a 400 m ridge climbs ~400 m", abs(rc - 400) < 20, f"{rc:.0f} m")
    rc0 = t.return_climb(flat(), (6000.0, 0.0), z_home, z_home)
    check("return over flat ground climbs nothing", rc0 < 1e-6, f"{rc0:.2f} m")


def test_line_of_sight():
    dem = ridge(height=400.0, x0=3000.0, half_width=60.0)
    z = 1000.0
    check("clear view over flat ground", t.visible_from(flat(), (0.0, 0.0), z + 120, (60.0, 0.0), z + 1))
    check("ridge between aircraft and person blocks view",
          not t.visible_from(dem, (2850.0, 0.0), z + 120, (3150.0, 0.0), z + 1))
    check("person on the near side of the ridge visible",
          t.visible_from(dem, (2700.0, 0.0), z + 120, (2760.0, 0.0), z + 1))


def test_camera():
    tan16 = math.tan(math.radians(16))
    half = t.H_SEARCH * tan16
    x = np.linspace(0, half, 2001)
    p = np.array([t.camera_probability(xi, 0.0, t.H_SEARCH, tan16) for xi in x])
    W = 2 * np.trapezoid(p, x)
    check("32 deg camera at 120 m, no blur, keeps the 55 m sweep width", abs(W - vb.W0) < 0.5,
          f"{W:.2f} m vs {vb.W0:.2f} m")
    check("outside the frame: zero", t.camera_probability(half + 1, 0.0, t.H_SEARCH, tan16) == 0.0)
    check("probabilities in [0, 1]", p.min() >= 0 and p.max() <= 1)
    p_hi = t.camera_probability(0.0, 0.0, 400.0, tan16)
    check("higher above the person: lower centre probability", p_hi < p[0], f"{p_hi:.2f} vs {p[0]:.2f}")
    p_blur = t.camera_probability(0.0, 25.0, t.H_SEARCH, tan16)
    check("motion blur lowers probability", p_blur < p[0], f"{p_blur:.2f} vs {p[0]:.2f}")


def test_fly():
    fleet = t.site_fleet(1000.0 + t.H_SEARCH, include_competitors=False)
    ac = fleet[0]
    dem = flat()
    legs = [((0.0, 0.0), (4000.0, 0.0)), ((4000.0, 0.0), (0.0, 0.0))]
    prof = t.profile(dem, legs)
    pat = dict(legs=legs, prof=prof, ret=t.return_climbs(dem, legs, prof, 1000.0))
    kc = t.climb_k(ac, ac)
    tan_half = t.camera_tangent(ac)
    draws = np.zeros(sp.MAX_LEGS)                       # a draw of 0 detects whenever p > 0
    hit = t.fly(ac, pat, (2000.0, 10.0), 1000.0, (0.0, 0.0), draws, kc, tan_half, 1000.0, dem)
    check("person under the track is found", hit["found"], f"t_find {hit['t_find']:.0f} s")
    v = ac.v_cruise
    expected = 2000.0 / v + t.H_SEARCH / (t.CLIMB_GRAD * v)        # flight to the person + launch climb
    check("find time includes the initial climb", math.isclose(hit["t_find"], expected),
          f"{hit['t_find']:.1f} s vs {expected:.1f} s at {v:g} m/s")
    check("kit arrival uses the modeled 7 m/s chute",
          math.isclose(hit["t_kit"] - hit["t_find"], t.ms.DROP_TIME + 40 / 7))
    ac.energy = 1
    limited = t.fly(ac, pat, (2000.0, 10.0), 1000.0, (0.0, 0.0), draws, kc, tan_half, 1000.0, dem)
    check("low energy prevents departure onto the search leg", not limited["found"])
    off = t.H_SEARCH * tan_half + 20
    ac.energy = 1000
    miss = t.fly(ac, pat, (2000.0, off), 1000.0, (0.0, 0.0), draws, kc, tan_half, 1000.0, dem)
    check("person just outside the frame is never found", not miss["found"], f"{off:.0f} m off track")

    valley = flat()
    row, _ = t.rc(0, 40, N)
    valley[:math.floor(row) + 1] -= 20
    target = (2000.0, 40.0)
    below = t.fly(ac, pat, target, float(t.dem_at(valley, *target)), (0.0, 0.0),
                  draws, kc, tan_half, 1000.0, valley)
    check("terrain can block the modeled release without blocking the find",
          below["found"] and below["t_kit"] is None)


def test_canopy():
    check("open ground: always visible", t.canopy_clear(0.0, 1.0) == 1.0)
    full = t.canopy_clear(1.0, 1.0)
    check("full cover at nadir: visible only through crown gaps", abs(full - t.CROWN_POROSITY) < 1e-12, f"{full:.2f}")
    slant = t.canopy_clear(0.6, math.cos(math.radians(25)))
    check("slant view through cover sees less than nadir", slant < t.canopy_clear(0.6, 1.0),
          f"{slant:.3f} vs {t.canopy_clear(0.6, 1.0):.3f}")
    fleet = t.site_fleet(1000.0 + t.H_SEARCH, include_competitors=False)
    ac = fleet[0]
    dem = flat()
    legs = [((0.0, 0.0), (4000.0, 0.0)), ((4000.0, 0.0), (0.0, 0.0))]
    prof = t.profile(dem, legs)
    pat = dict(legs=legs, prof=prof, ret=t.return_climbs(dem, legs, prof, 1000.0))
    kc = t.climb_k(ac, ac)
    draws = np.zeros(sp.MAX_LEGS)
    args = (ac, pat, (2000.0, 10.0), 1000.0, (0.0, 0.0), draws, kc, t.camera_tangent(ac), 1000.0, dem)
    check("person under dense canopy with an unlucky hiding draw is never found",
          not t.fly(*args, (1.0, 0.99))["found"])
    check("same person in the open is found", t.fly(*args, (0.0, 0.99))["found"])
    hikers = pd.DataFrame(dict(tx=[0.0, 100.0], ty=[0.0, 0.0]))
    a = t.cover_draws("rainier_paradise", hikers, 1)[1]
    b = t.cover_draws("rainier_paradise", hikers, 1)[1]
    check("hiding draws are reproducible", np.array_equal(a, b))


def test_competitor_endurance():
    altitude = 1000.0 + t.H_SEARCH
    baseline = t.site_fleet(altitude)
    derated = t.site_fleet(altitude, competitor_endurance=0.8)
    check("endurance sensitivity leaves Lodestar energy at the baseline",
          all(a.energy == b.energy for a, b in zip(baseline[:2], derated[:2])))
    check("endurance sensitivity scales every competitor's energy budget",
          len(baseline) == len(derated) > 2 and all(
              a.name == b.name and a.v_cruise == b.v_cruise and math.isclose(b.energy, 0.8 * a.energy)
              for a, b in zip(baseline[2:], derated[2:])))


if __name__ == "__main__":
    for f in (test_clearance_profile, test_partial_leg_energy, test_return_climb, test_line_of_sight, test_camera, test_fly,
              test_canopy, test_competitor_endurance):
        print(f"--- {f.__name__}")
        f()
    print(f"\n{check.failed} failed")
    sys.exit(1 if check.failed else 0)
