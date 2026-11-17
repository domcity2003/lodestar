"""Known-answer checks for sixdof_v1.py (the v1 JSBSim model).

Run:  ../.venv/Scripts/python.exe -X utf8 test_sixdof_v1.py   (from scripts/)
Builds the model if out/sixdof_v1/ does not have it yet (needs AVL then); otherwise reuses the AVL sweep.
"""
import math
import sys

import numpy as np
import pandas as pd

import lodestar_v1 as v1
import sixdof_v1 as s6


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0
ROOT = s6.ROOT
AI = s6.isa_air(s6.ALT)


def fdm_props(kit):
    f = s6.trim_fdm(kit, AI)
    m = f["inertia/weight-lbs"] * s6.LB / s6.G0
    sl = 1.3558179                                        # slug ft2 -> kg m2
    return f, dict(m=m, x=f["inertia/cg-x-in"] * s6.IN, z=f["inertia/cg-z-in"] * s6.IN,
                   Ixx=f["inertia/ixx-slugs_ft2"] * sl, Iyy=f["inertia/iyy-slugs_ft2"] * sl, Izz=f["inertia/izz-slugs_ft2"] * sl)


def test_mass_cg_inertia():
    _, on = fdm_props(True)
    _, off = fdm_props(False)
    M = v1.MASS
    check("JSBSim mass, kit on = lodestar_v1 sicore_loaded_kg (1 g)", abs(on["m"] - M["sicore_loaded_kg"]) < 1e-3,
          f"{on['m']:.4f} vs {M['sicore_loaded_kg']} kg")
    check("JSBSim mass, kit off = lodestar_v1 sicore_released_kg (1 g)", abs(off["m"] - M["sicore_released_kg"]) < 1e-3,
          f"{off['m']:.4f} vs {M['sicore_released_kg']} kg")
    check("JSBSim CG x, kit on = lodestar_v1 x_cg_sicore_loaded (0.1 mm)", abs(on["x"] - M["x_cg_sicore_loaded"]) < 1e-4,
          f"{on['x']:.5f} vs {M['x_cg_sicore_loaded']} m")
    check("JSBSim CG x, kit off = lodestar_v1 x_cg_sicore_released (0.1 mm)", abs(off["x"] - M["x_cg_sicore_released"]) < 1e-4,
          f"{off['x']:.5f} vs {M['x_cg_sicore_released']} m")
    mb = pd.read_csv(ROOT / "out" / "wing_trade" / "aircraft_mass_balance_v1.csv").set_index(["cell", "kit"])
    for kit, d in ((True, on), (False, off)):
        ref = mb.loc[("SiCore", kit)].Ixx_kg_m2
        check(f"JSBSim Ixx, kit {'on' if kit else 'off'} = aircraft_mass_balance_v1.csv (1%)", abs(d["Ixx"] / ref - 1) < 0.01,
              f"{d['Ixx']:.4f} vs {ref:.4f} kg m2")
    ms = pd.read_csv(ROOT / "out" / "controls" / "mass_states.csv")
    for kit, d in ((True, on), (False, off)):
        r = ms[(ms.cell == "SiCore") & (ms.kit == kit)].iloc[0]
        ok = abs(d["Iyy"] / r.Iyy - 1) < 0.01 and abs(d["Izz"] / r.Izz - 1) < 0.01
        check(f"JSBSim Iyy, Izz, kit {'on' if kit else 'off'} = controls mass_states.csv from cad_v4 (1%)", ok,
              f"Iyy {d['Iyy']:.4f} vs {r.Iyy:.4f}, Izz {d['Izz']:.4f} vs {r.Izz:.4f} kg m2")
    cad = pd.read_csv(ROOT / "out" / "cad_v4" / "mass_cg.csv").set_index("item")
    mm = s6.mass_model()
    ok = all(abs(k.m - cad.loc[k.name].mass_kg) < 1e-6 and abs(k.x - cad.loc[k.name].x) < 1e-6 and abs(k.z - cad.loc[k.name].z) < 1e-6
             for k in mm["kit"])
    check("kit and parachute point masses = cad_v4 mass_cg.csv rows (mass, x, z)", ok and len(mm["kit"]) == 2)


def test_release_mass_delta():
    tr = s6.calm_trim(True, s6.V_DROP_TURB, s6.ALT)
    s = s6.Sim(s6.ALT, s6.H_REL, s6.V_DROP_TURB, alpha_deg=tr["alpha_deg"])
    s.set_trim(tr)
    s.step_until(2.0, release_at=1.0)
    d = pd.DataFrame(s.log)
    dm = d.mass.iloc[0] - d.mass.iloc[-1]
    kit = v1.MASS["rescue_kit_kg"] + v1.MASS["kit_chute_kg"]
    check("release mass drop = rescue kit + parachute (1 g)", abs(dm - kit) < 1e-3, f"{dm:.4f} vs {kit:.4f} kg")
    check("release mass drop = sicore_loaded - sicore_released (1 g)",
          abs(dm - (v1.MASS["sicore_loaded_kg"] - v1.MASS["sicore_released_kg"])) < 1e-3)


def test_trim_lift_weight():
    for kit in (True, False):
        f = s6.trim_fdm(kit, AI)
        tr = s6.trim(f, s6.V_SEARCH, s6.ALT + s6.H_SEARCH)
        g = f["accelerations/gravity-ft_sec2"] * s6.FT - 7.292115e-5**2 * (6378137.0 + s6.ALT + s6.H_SEARCH)
        a = math.radians(tr["alpha_deg"])
        # level, unaccelerated: lift + thrust component normal to the flight path = weight (local gravity less the
        # centripetal term of JSBSim's rotating Earth at latitude 0)
        ratio = (tr["lift_N"] + tr["T"] * math.sin(a)) / (tr["mass"] * g)
        check(f"trimmed 1-g level flight, kit {'on' if kit else 'off'}, 20 m/s: lift + T sin(alpha) = m g (0.1%)",
              tr["ok"] and abs(ratio - 1) < 0.001, f"ratio {ratio:.5f}, residual {tr['resid']:.1e}")
        check(f"trimmed level flight, kit {'on' if kit else 'off'}: thrust = drag + ... within 1% (T cos(alpha) = D)",
              abs(tr["T"] * math.cos(a) / tr["drag_N"] - 1) < 0.01, f"T cos a {tr['T'] * math.cos(a):.3f} vs D {tr['drag_N']:.3f} N")
        d_mission = float(s6.pm.drag(s6.V_SEARCH, tr["mass"], AI, n=(tr["lift_N"] / (tr["mass"] * s6.pm.G))))
        check(f"JSBSim drag = mission/propulsion_margins drag model, kit {'on' if kit else 'off'} (1%)",
              abs(tr["drag_N"] / d_mission - 1) < 0.01, f"{tr['drag_N']:.3f} vs {d_mission:.3f} N")
    sw = pd.read_csv(s6.OUT / "avl_sweep.csv")
    f = s6.trim_fdm(True, AI)
    tr = s6.trim(f, s6.V_SEARCH, s6.ALT + s6.H_SEARCH, T=0.0)       # no thrust moment: pure AVL trim
    a_avl = float(np.interp(tr["CL"], sw.CL_trim, sw.alpha_deg))
    de_avl = float(np.interp(tr["CL"], sw.CL_trim, sw.elevon_trim_deg))
    check("JSBSim trim alpha and elevon reproduce AVL trim at the same CL (0.05 deg)",
          abs(tr["alpha_deg"] - a_avl) < 0.05 and abs(tr["de_eff_deg"] - de_avl) < 0.05,
          f"alpha {tr['alpha_deg']:.3f} vs {a_avl:.3f}, elevon {tr['de_eff_deg']:.3f} vs {de_avl:.3f} deg")


def test_glide_ratio():
    bp = pd.read_csv(s6.OUT / "glide_best.csv")
    for kit in (True, False):
        cfg = "kit on" if kit else "kit off"
        b = bp[(bp.config == cfg) & (bp.alt_m == s6.ALT)].iloc[0]
        V = float(b.V_best_glide)
        f = s6.trim_fdm(kit, AI)
        tg = s6.trim(f, V, s6.ALT + 200, T=0.0, guess=(6, 0, -3))
        s = s6.Sim(s6.ALT, 300.0, V, kit=kit, alpha_deg=tg["alpha_deg"], gamma_deg=tg["gamma_deg"])
        s.set_trim(tg)
        s.T, s.motor = 0.0, False
        s.mode, s.V_cmd = "glide", V
        s.step_until(40.0)
        d = pd.DataFrame(s.log)
        st = d[d.t > 15]
        gr_sim = st.vg.mean() / (-st.hdot.mean())
        g = math.atan(1 / gr_sim)
        m = tg["mass"]
        ld = m * s6.pm.G * math.cos(g) / float(s6.pm.drag(st.V.mean(), m, AI, n=math.cos(g)))
        check(f"steady glide ratio in JSBSim = L/D of the mission drag model, {cfg} (5%)", abs(gr_sim / ld - 1) < 0.05,
              f"{gr_sim:.2f} vs {ld:.2f} at {st.V.mean():.2f} m/s")
    gp = pd.read_csv(s6.OUT / "glide_polar.csv")
    worst = 0.0
    for r in gp.itertuples():
        ai = s6.isa_air(r.alt_m)
        g = math.radians(r.gamma_deg)
        ld = r.mass_kg * s6.pm.G * math.cos(g) / float(s6.pm.drag(r.V_tas, r.mass_kg, ai, n=math.cos(g)))
        worst = max(worst, abs(r.glide_ratio / ld - 1))
    check("glide_polar.csv glide ratio = mission-model L/D on every row (5%)", worst < 0.05, f"worst {100 * worst:.2f}%")


def test_modes_vs_avl():
    mc = pd.read_csv(s6.OUT / "model_check.csv")
    ok = (mc.diff_pct.abs() < 10).all()
    check("JSBSim open-loop phugoid and Dutch-roll periods within 10% of AVL eigenvalues", bool(ok),
          ", ".join(f"{r.mode} {r.diff_pct:+.1f}%" for r in mc.itertuples()))


def test_mixer():
    de, da, sat, gmax = s6.mix(math.radians(30), 0.0)
    check("mixer clips a 30 deg elevon command to the 20 deg geometric travel", sat and abs(gmax - 20.0) < 1e-6
          and abs(math.degrees(de) - s6.cs.effective(20.0)) < 1e-3, f"effective {math.degrees(de):.3f} deg")


if __name__ == "__main__":
    s6.load(rebuild=not (s6.ACDIR / f"{s6.ACNAME}.xml").exists())
    test_mass_cg_inertia()
    test_release_mass_delta()
    test_trim_lift_weight()
    test_glide_ratio()
    test_modes_vs_avl()
    test_mixer()
    print(f"\n{check.failed} failed")
    sys.exit(1 if check.failed else 0)
