"""Track spacing under the in-frame camera model: does spacing tracks at each camera's real sweep width pay?

The fleet runs space tracks at the zero-blur sweep width (Lodestar 67.5 m, 32 deg cameras 55 m). With
detection only inside the frame, the sweep width at search speed is smaller, which leaves gaps between
tracks. This reruns Lodestar SiCore and DeltaQuad Evo GOV (trails 3 and trails 6 patterns, same 500 hikers
per site) with spacing set to the sweep width at each aircraft's search airspeed, and Lodestar at 18 m/s.

Run:  uv run python -X utf8 scripts/spacing_check.py
Writes: out/terrain/spacing_check.csv
"""
from concurrent.futures import ProcessPoolExecutor
import math

import numpy as np
import pandas as pd

import terrain_sim as t
import search_patterns as sp

N = 500


def sweep_width(hfov, airspeed, height=t.H_SEARCH):
    tan = math.tan(math.radians(hfov / 2))
    x = np.linspace(0, height * tan, 2001)
    return 2 * float(np.trapezoid([t.camera_probability(xi, airspeed, height, tan) for xi in x], x))


def variants():
    L, O = t.CAMERAS["Lodestar"][0], t.CAMERAS["other"][0]
    return [
        ("current spacing", 20.0, t.CAMERAS["Lodestar"][1], t.CAMERAS["other"][1]),
        ("spacing = sweep width", 20.0, round(sweep_width(L, 20.0), 1), round(sweep_width(O, 16.54), 1)),
        ("spacing = sweep width, Lodestar 18 m/s", 18.0, round(sweep_width(L, 18.0), 1), round(sweep_width(O, 16.54), 1)),
    ]


def site(key):
    P = t.prep(key, N, t.ms.RNG_SEED)
    dem, lines, _ = t.load(key)
    chute = pd.read_csv(t.ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")
    phit = lambda w: np.interp(w, chute.wind, chute.p_reach)  # noqa: E731
    pats = {}

    def pattern(name, spc):
        if (name, spc) not in pats:
            R = float(name.split()[1]) * 1000
            legs, _, _ = t.trail_tour(lines, R, [R / 3, 2 * R / 3] if R > 3000 else [R / 2], spc)
            prof = t.profile(dem, legs)
            pats[(name, spc)] = dict(legs=legs, prof=prof, ret=t.return_climbs(dem, legs, prof, P["z_home"]))
        return pats[(name, spc)]

    out = []
    for label, v_lode, sp_lode, sp_other in variants():
        fleet = t.site_fleet(P["z_home"] + t.H_SEARCH)
        ref = fleet[0]
        for ac in fleet:
            if ac.name not in ("Lodestar SiCore", "DeltaQuad Evo GOV"):
                continue
            lode = ac.name.startswith("Lodestar")
            if lode:
                ac.v_cruise = v_lode
                ac.search_h = ac.energy / float(ac.power(v_lode))
            kc = t.climb_k(ac, ref)
            for name in ("trails 3", "trails 6"):
                pat = pattern(name, sp_lode if lode else sp_other)
                rows = []
                for h in P["hikers"].itertuples():
                    if not h.on_map:
                        rows.append(dict(found=False, t_find=None, t_kit=None, h_agl=None))
                        continue
                    wind = (h.ws * math.cos(h.wd), h.ws * math.sin(h.wd))
                    rows.append(t.fly(ac, pat, (h.tx, h.ty), h.z, wind, h.draws, kc, t.camera_tangent(ac), P["z_home"], dem))
                runs = pd.DataFrame(rows).rename_axis("run").reset_index()
                sc = t.score(runs, P["hikers"], phit)
                out.append(dict(site=key, variant=label, aircraft=ac.name, pattern=name,
                                spacing_m=sp_lode if lode else sp_other, help_1h=sc.help_1h.mean(), help_3h=sc.help_3h.mean()))
    print(f"done {key}", flush=True)
    return out


if __name__ == "__main__":
    for v in variants():
        print(v)
    with ProcessPoolExecutor() as ex:
        rows = [r for part in ex.map(site, list(t.SITES)) for r in part]
    df = pd.DataFrame(rows)
    df.to_csv(t.OUT / "spacing_check.csv", index=False)
    with pd.option_context("display.width", 200):
        for col in ("help_3h", "help_1h"):
            print(f"\n{col} (%), average of 7 sites")
            print((df.pivot_table(index=["variant", "pattern"], columns="aircraft", values=col, sort=False) * 100).round(1).to_string())
