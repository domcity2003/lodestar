"""Invariant checks for the continuous-route mission replay (flyable_mission, long_pass, terrain_sim.score).

Added in the 2026-11-14 mission-simulation QA. Synthetic flat 30 x 30 km map at 1000 m.
  - route modes: the default (pursuit_1) and archived modes build, name and number passes correctly;
  - turn radius never below the wind-band limit, bank never above 30 deg for any wind direction in the band;
  - energy along the route never negative or decreasing, time strictly increasing;
  - reserve: no search piece is credited once energy + return + turn allowance exceeds 90% of the battery;
  - kit release only after a find, at >= 40 m above the person, never for competitors;
  - help-within-1 h / 3 h windows are inclusive at exactly 3600 / 10800 s from launch;
  - units and density conventions (TAS turn speeds, 57.2 m still-air radius at 18 m/s);
  - envelope guards raise instead of letting NaN through;
  - seed hygiene (tuning 20261111 vs held-out 20261114).

Run:  ../.venv/Scripts/python.exe -X utf8 test_mission_invariants.py
"""
import copy
import math
import pickle
import sys
from types import SimpleNamespace

import aerosandbox as asb
import numpy as np
import pandas as pd

import flyable_mission as f
import long_pass as lp
import terrain_sim as t
from test_terrain_sim import flat
from turn_path import ground_radius

HOME = 1000.0
ALT = HOME + t.H_SEARCH
FLEET = t.site_fleet(ALT)            # built once: Lodestar SiCore, P50B, then competitors
SICORE, EVO = FLEET[0], FLEET[2]
failed = 0


def check(name, ok, detail=""):
    global failed
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    failed += not ok


def trail_pattern(n=120, seed=1):
    """Wiggly trail tour from the trailhead, return home, then two search legs (prep()-style legs)."""
    rng = np.random.default_rng(seed)
    angles = np.cumsum(rng.normal(0, 1.2, n))
    steps = rng.uniform(40, 150, n)
    pts = np.vstack([[0, 0], np.cumsum(np.column_stack([steps * np.cos(angles), steps * np.sin(angles)]), axis=0)])
    legs = [(tuple(map(float, a)), tuple(map(float, b))) for a, b in zip(pts[:-1], pts[1:])]
    return legs + [(legs[-1][1], (0.0, 0.0)), ((0.0, 0.0), (0.0, 800.0)), ((0.0, 800.0), (800.0, 800.0))]


def hiker(x, y, z=HOME):
    return SimpleNamespace(on_map=True, tx=x, ty=y, z=z, draws=np.zeros(t.sp.MAX_LEGS))


def test_route_modes():
    check("default route mode is pursuit_1", lp.DEFAULT_ROUTE_MODE == "pursuit_1")
    check("archived modes have no lookahead", all(lp.lookahead_of(m) is None for m in lp.ARCHIVED_ROUTE_MODES))
    check("pursuit_<k> parses k", lp.lookahead_of("pursuit_0.5") == 0.5 and lp.lookahead_of("pursuit_2") == 2)
    for bad in ("sharp", "rounded_fixed", "pursuit_0", "pursuit_x"):
        try:
            lp.lookahead_of(bad)
            check(f"unknown or invalid mode {bad!r} is rejected", False)
        except ValueError:
            check(f"unknown or invalid mode {bad!r} is rejected", True)
    check("adaptive band is the next enclosing bound",
          [lp.adaptive_bound(w) for w in (0, 2, 2.01, 9.99, 10)] == [2, 2, 4, 10, 10])


def test_routes_turns_energy():
    dem, base = flat(), trail_pattern()
    loops = {}
    for mode in ("rounded_adaptive", "rounded_passes", "pursuit_1"):
        for bound in (2, 10):
            flight, legs, miss = lp.route_flight(SICORE, base, dem, HOME, bound, ALT, mode)
            route = flight.route
            loops[mode, bound] = route.loops
            limit = ground_radius(route.speed, bound, t.ms.BANK, t.ms.G)
            radii = np.array([np.linalg.norm(p.start - p.center) for p in route.pieces if p.center is not None])
            tag = f"{mode}, {bound} m/s band"
            check(f"{tag}: every arc radius >= wind-band limit", len(radii) and radii.min() >= limit - 1e-6,
                  f"min {radii.min():.1f} m vs {limit:.1f} m")
            worst, ok = 0.0, True
            for angle in np.linspace(0, math.tau, 36, endpoint=False):
                wind = bound * np.array([math.cos(angle), math.sin(angle)])
                try:
                    p = flight.profile(wind, HOME)
                except ValueError:
                    ok = False
                    break
                worst = max(worst, float(p["bank_tan"].max()))
                e, time = p["energy"], p["time"]
                ok &= bool(e[0] >= 0 and np.all(np.diff(e) >= 0) and np.all(np.diff(time) > 0)
                           and np.all(p["de"] >= flight.idle * p["dt"] / 3600 - 1e-15))
            check(f"{tag}: bank <= 30 deg in every wind direction at the band edge",
                  ok and worst <= math.tan(t.ms.BANK) + 1e-9, f"max bank {math.degrees(math.atan(worst)):.2f} deg")
            check(f"{tag}: energy never negative or decreasing, time increasing", ok)
            sources = np.array([p.source for p in route.pieces if p.source is not None])
            if mode == "rounded_adaptive":
                check(f"{tag}: per-leg draws index the flown legs",
                      sources.min() >= 0 and sources.max() < len(legs) and np.all(np.diff(sources) > 0))
            else:
                check(f"{tag}: per-pass draws start at 0, never decrease, fit the draw array",
                      sources[0] == 0 and np.all(np.diff(sources) >= 0) and sources.max() < t.sp.MAX_LEGS)
            if mode == "pursuit_1":
                check(f"{tag}: pursuit trail coverage measured", len(miss) > 10 and np.all(miss >= 0))
    check("pursuit route needs fewer turnaround loops than the trail tour (10 m/s band)",
          loops["pursuit_1", 10] < loops["rounded_adaptive", 10],
          f"{loops['pursuit_1', 10]} vs {loops['rounded_adaptive', 10]}")


def test_reserve():
    ac = copy.copy(SICORE)
    legs = [((x, 0.0), (x + 500.0, 0.0)) for x in np.arange(0, 14000, 500.0)]
    power = float(ac.power(18.0))
    ac.energy = power * (12000 / 18 / 3600) / 0.9          # about 6 km out and back after reserve
    dem = flat()
    route = f.build_route(ac, legs, dem, HOME, 2, ALT)
    flight = f.Flight(ac, route)
    check("reserve: usable budget is 90% of battery energy", math.isclose(flight.budget, 0.9 * ac.energy))
    p = flight.profile(np.zeros(2), HOME)
    need = p["energy"][route.offsets[1:]] + p["home_wh"] + p["turn_wh"]
    over = np.flatnonzero(need > flight.budget)
    result = flight.replay(hiker(13000.0, 0.0), np.zeros(2), (0.0, 0.9), dem, HOME)
    check("reserve: far hiker is cut off by the reserve, not found", result["cutoff"] == "reserve" and not result["found"])
    stop = int(over[0])
    credited = sum(flight.length[i] for i, k in enumerate(route.search) if k < stop) / 1000
    check("reserve: searched distance ends at the last piece that keeps the reserve",
          math.isclose(result["searched_km"], credited) and np.all(need[:stop] <= flight.budget),
          f"{result['searched_km']:.2f} km")
    beyond = flight.replay(hiker(1000.0 * (credited + 0.75), 0.0), np.zeros(2), (0.0, 0.9), dem, HOME)
    check("reserve: a hiker just past the reserve limit is not found", not beyond["found"])
    inside = flight.replay(hiker(1000.0 * max(credited - 0.75, 0.3), 0.0), np.zeros(2), (0.0, 0.9), dem, HOME)
    check("reserve: a hiker inside the limit is found", inside["found"])


def test_kit():
    dem = flat()
    legs = [((0.0, 0.0), (4000.0, 0.0))]
    route = f.build_route(SICORE, legs, dem, HOME, 2, ALT)
    hit = f.Flight(SICORE, route).replay(hiker(2000.0, 0.0), np.zeros(2), (0.0, 0.9), dem, HOME)
    delay = t.ms.DROP_TIME + t.DROP_FALL_TIME
    check("kit: released after a find on open flat ground", hit["found"] and hit["t_kit"] is not None)
    check("kit: arrives DROP_TIME + fall time after the find", math.isclose(hit["t_kit"] - hit["t_find"], delay))
    check("kit: release height above the person >= DROP_HEIGHT", hit["h_agl"] >= t.DROP_HEIGHT)
    valley = flat()
    row, _ = t.rc(0, 40, valley.shape[0])
    valley[:math.floor(row) + 1] -= 20                    # person 20 m below the ground under the aircraft
    target = (2000.0, 40.0)
    z = float(t.dem_at(valley, *target))
    route = f.build_route(SICORE, legs, valley, HOME, 2, ALT)
    low = f.Flight(SICORE, route).replay(hiker(*target, z), np.zeros(2), (0.0, 0.9), valley, HOME)
    check("kit: no release when the drop altitude would be < 40 m above the ground below",
          low["found"] and low["t_kit"] is None)
    route = f.build_route(EVO, legs, dem, HOME, 2, ALT, ref=SICORE)
    comp = f.Flight(EVO, route, ref=SICORE).replay(hiker(2000.0, 0.0), np.zeros(2), (0.0, 0.9), dem, HOME)
    check("kit: competitor finds but never releases", comp["found"] and comp["t_kit"] is None)


def test_help_windows():
    hk = pd.DataFrame(dict(t_team=[0.0, 0.0, 4900.0, 0.0, 7200.0], ws=[0.0] * 5, draws=[None] * 5))
    runs = pd.DataFrame(dict(run=range(5), found=[True, True, True, False, True],
                             t_find=[3600.0, 3600.5, 100.0, np.nan, 3600.0],
                             t_kit=[np.nan, np.nan, 3600.0, np.nan, np.nan]))
    d = t.score(runs, hk, lambda w: np.full(np.shape(w), 0.7))
    check("help within 1 h is inclusive at exactly 3600 s", d.help_1h[0] == 1)
    check("help 0.5 s after the hour does not count", d.help_1h[1] == 0 and d.help_3h[1] == 1)
    check("kit at 3600 s counts with the chute's reach probability", math.isclose(d.help_1h[2], 0.7)
          and d.help_3h[2] == 1)
    check("no find, no help", d.help_1h[3] == 0 and d.help_3h[3] == 0)
    check("help within 3 h is inclusive at exactly 10800 s", d.help_3h[4] == 1 and d.help_1h[4] == 0)


def test_units_and_density():
    check("still-air radius at 18 m/s, 30 deg bank is 57.2 m",
          math.isclose(ground_radius(18, 0, t.ms.BANK, t.ms.G), 57.2, abs_tol=0.05))
    rho = float(asb.Atmosphere(altitude=ALT).density())
    check("Lodestar power model density is the site density (search altitude)",
          math.isclose(SICORE.rw_cfg["rho"], rho, rel_tol=1e-9))
    check("Lodestar turn floor at site altitude = v_min / sqrt(cos bank)",
          math.isclose(f.min_turn_speed(SICORE, ALT), SICORE.v_min / math.sqrt(math.cos(t.ms.BANK))))
    check("competitor turn floor is a sea-level value: v_min / sqrt(cos bank) at 0 m",
          math.isclose(f.min_turn_speed(EVO, 0.0), EVO.v_min / math.sqrt(math.cos(t.ms.BANK)), rel_tol=1e-6))
    check("turn floor (TAS) rises with altitude",
          f.min_turn_speed(SICORE, ALT + 1500) > f.min_turn_speed(SICORE, ALT) and
          f.min_turn_speed(EVO, ALT + 1500) > f.min_turn_speed(EVO, ALT))
    route = f.build_route(SICORE, [((0.0, 0.0), (2000.0, 0.0))], flat(), HOME, 2, ALT)
    flight = f.Flight(SICORE, route)
    check("Lodestar induced power is a fraction of straight-flight power",
          0 < flight.induced_power.max() < flight.power_ref, f"{flight.induced_power.max():.1f} W of {flight.power_ref:.1f} W")
    route = f.build_route(EVO, [((0.0, 0.0), (2000.0, 0.0))], flat(), HOME, 2, ALT, ref=SICORE)
    flight = f.Flight(EVO, route, ref=SICORE)
    check("competitor induced power = 0.5 x cruise power x V_cruise / V",
          np.allclose(flight.induced_power, 0.5 * EVO.p_cruise * EVO.v_cruise / route.speed))
    check("climb energy per metre (Wh/m) = m g / eta / 3600 for Lodestar",
          math.isclose(t.climb_k(SICORE, SICORE), SICORE.mass * t.ms.G / t.ms.sz.A.eta_max / 3600))


def test_guards():
    route = f.build_route(SICORE, [((0.0, 0.0), (2000.0, 0.0)), ((2000.0, 0.0), (2000.0, 2000.0))],
                          flat(), HOME, 2, ALT)
    flight = f.Flight(SICORE, route)
    for name, wind in (("wind above the planned band (bank limit)", np.array([0.0, 9.0])),
                       ("crosswind above airspeed (NaN ground speed)", np.array([0.0, 30.0]))):
        try:
            flight.profile(wind, HOME)
            check(f"guard: {name} raises instead of passing silently", False)
        except ValueError:
            check(f"guard: {name} raises instead of passing silently", True)
    try:
        lp.pursue(np.array([[0.0, 0.0], [5000.0, 0.0]]), 50.0, 50.0)
        check("guard: pursuit on a plain line finishes inside its step cap", True)
    except RuntimeError:
        check("guard: pursuit on a plain line finishes inside its step cap", False)


def test_seeds():
    check("tuning seed is mission_sim.RNG_SEED = 20261111", lp.TUNING_SEED == t.ms.RNG_SEED == 20261111)
    check("held-out seed is 20261114", lp.HELD_OUT_SEED == 20261114)
    site = "rocky_bear_lake"
    hk = pd.DataFrame(dict(tx=np.linspace(-5000, 5000, 50), ty=np.zeros(50)))
    a = t.cover_draws(site, hk, lp.TUNING_SEED)[1]
    b = t.cover_draws(site, hk, lp.HELD_OUT_SEED)[1]
    check("canopy hiding draws differ between tuning and held-out seeds", not np.allclose(a, b))
    check("canopy hiding draws are reproducible", np.array_equal(a, t.cover_draws(site, hk, lp.TUNING_SEED)[1]))
    files = [t.DATA / site / f"prep{t.PREP_VERSION}_{lp.N}_{s}.pkl" for s in (lp.TUNING_SEED, lp.HELD_OUT_SEED)]
    if all(p.exists() for p in files):
        h0, h1 = (pickle.loads(p.read_bytes())["hikers"] for p in files)
        check("cached held-out hikers differ from tuning hikers", not np.allclose(h0.tx, h1.tx))
        check("every hiker has one detection draw per possible leg or pass",
              all(len(d) == t.sp.MAX_LEGS for d in (*h0.draws, *h1.draws)))
    else:
        print("SKIP  prep caches for both seeds not found")


if __name__ == "__main__":
    for fn in (test_route_modes, test_routes_turns_energy, test_reserve, test_kit, test_help_windows,
               test_units_and_density, test_guards, test_seeds):
        print(f"--- {fn.__name__}")
        fn()
    print(f"\n{failed} failed")
    sys.exit(1 if failed else 0)
