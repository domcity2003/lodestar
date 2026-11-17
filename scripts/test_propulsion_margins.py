"""Known-answer checks for propulsion_margins.py.

Run:  ../.venv/Scripts/python.exe -X utf8 test_propulsion_margins.py      (from scripts/)
Each check compares the model with an independent number: the ISA table, a maker's bench row, the UIUC data
file, the v3 propulsion chain, the baseline brief, or a closed-form identity.
"""
from pathlib import Path
import math
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import propulsion_margins as pm  # noqa: E402
import v3_propulsion as vp  # noqa: E402

SICORE, P50B = "Amprius SiCore 6.3 Ah", "Molicel P50B"


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0


def test_atmosphere():
    check("ISA sea-level density 1.225", math.isclose(pm.isa_density(0.0), 1.225, rel_tol=1e-3))
    # ISA table: 4,400 m -> 0.7768 kg/m3 (troposphere formula rho0 (1 - 2.25577e-5 h)^4.2559)
    rho44 = 1.225 * (1 - 2.25577e-5 * 4400) ** 4.2559
    check("ISA density at 4,400 m", math.isclose(pm.isa_density(4400.0), rho44, rel_tol=2e-3),
          f"{pm.isa_density(4400.0):.4f} vs {rho44:.4f}")
    hh = pm.hot_and_high()
    # 3,000 m pressure 70.11 kPa at 288.65 K -> 0.8462 kg/m3 -> density altitude about 3,690 m
    check("hot-and-high density 0.846", math.isclose(hh["rho"], 70108 / (287.05 * 288.65), rel_tol=3e-3),
          f"{hh['rho']:.4f}")
    check("hot-and-high density altitude 3,650-3,750 m", 3650 < hh["da"] < 3750, f"{hh['da']:.0f} m")
    check("speed of sound at -10 C = 325.2 m/s", math.isclose(pm.air(0, -10)["a"], 325.2, rel_tol=1e-3))


def test_motor_calibration():
    # T-Motor AT3520 KV550 bench, 100% throttle: 13x6.5 21.99 V 34.89 A 9252 rpm; 14x7 21.83 V 44.28 A 8762 rpm
    for prop, V, I, rpm in (("13x6.5", 21.99, 34.89, 9252), ("14x7", 21.83, 44.28, 8762)):
        rpm_model = pm.KV * (pm.K_FF * V - I * pm.RM20)
        check(f"full-throttle RPM vs T-Motor bench ({prop})", abs(rpm_model / rpm - 1) < 0.03,
              f"model {rpm_model:.0f} vs bench {rpm} rpm")


def test_prop_data():
    # UIUC static file ancf_15x10_static_0982od: 4940 rpm -> CT 0.108982, CP 0.042863
    ct, cp = pm.ctcp(4940.0, 0.0)
    check("static CT/CP equal the UIUC static file at 4,940 rpm",
          math.isclose(float(ct), 0.108982, rel_tol=1e-6) and math.isclose(float(cp), 0.042863, rel_tol=1e-6))
    # inside the tested range the vectorised lookup equals v3_propulsion.coeffs
    prop = vp.load_props()[pm.PROP_KEY]
    for rpm, J in ((3900, 0.40), (5000, 0.55), (3020, 0.70)):
        a = vp.coeffs(prop, rpm, J)
        b = pm.ctcp(rpm, J)
        check(f"CT/CP match v3 coeffs at {rpm} rpm, J {J}", np.allclose(a, [float(b[0]), float(b[1])], rtol=1e-9))
    # the blend is continuous at the first tested J of the nearest run
    j1 = pm.GROUPS[-1][1][0]
    lo, hi = pm.ctcp(4990.0, j1 - 1e-6), pm.ctcp(4990.0, j1 + 1e-6)
    check("static-to-tunnel blend is continuous", abs(float(lo[0]) - float(hi[0])) < 1e-3)


def v1_perf(cell):
    """The v1 performance row (out/wing_trade/performance_v1.csv) for a cell at 2,000 m."""
    import pandas as pd
    t = pd.read_csv(pm.ROOT / "out" / "wing_trade" / "performance_v1.csv")
    return t[(t.cell == cell) & (t.altitude_m == 2000)].iloc[0]


def test_chain_vs_v3():
    m, wh = pm.baseline(SICORE)
    ref = v1_perf(SICORE)
    check("baseline SiCore loaded mass matches performance_v1.csv", math.isclose(m, ref.takeoff_mass_kg, abs_tol=0.01),
          f"{m:.3f} vs {ref.takeoff_mass_kg:.3f} kg")
    ai = pm.air(2000.0, 2.0)                       # ISA temperature at 2,000 m
    T = float(pm.drag(20.0, m, ai))
    vp.RHO, vp.MU = ai["rho"], ai["mu"]
    s = vp.system(vp.load_props()[pm.PROP_KEY], "T-Motor AT3520-550", 20.0, T)
    r = pm.cruise_power(20.0, m, ai, pm.CELLS[SICORE], 20.0)
    check("battery power at 20 m/s matches the v3 chain within 0.5%",
          math.isclose(r["P_batt"], s["P_batt"] + pm.P_AV, rel_tol=5e-3), f"{r['P_batt']:.1f} vs {s['P_batt'] + 25:.1f} W")
    e = wh * 0.85 * 0.90 / r["P_batt"]
    check("design-case endurance at 20 m/s reproduces performance_v1.csv", math.isclose(e, ref.search_hours, abs_tol=0.03),
          f"{e:.3f} vs {ref.search_hours:.3f} h")


def test_battery():
    cell = pm.CELLS[P50B]
    I, V, vc = pm.bus(cell, 25.0, 0.5, 300.0)
    check("bus solution: V x I = P", math.isclose(float(V * I), 300.0, rel_tol=1e-9))
    E = 6 * float(pm.ocv(cell, 0.5))
    R = 6 * pm.r_cell(cell, 25.0, 0.5) / 5 + pm.R_WIRE
    check("bus solution: V = E - I R", math.isclose(float(V), E - float(I) * R, rel_tol=1e-9))
    check("P50B DC resistance 12.8 mOhm at 25 C, mid charge", math.isclose(pm.r_cell(cell, 25.0, 0.5), 0.0128, rel_tol=1e-9))
    check("resistance factor at -10 C between 2.5 and 4", 2.5 < pm.r_cell(cell, -10.0, 0.5) / 0.0128 < 4.0)
    _, _, bad = pm.bus(cell, 25.0, 0.5, 1e6)
    check("undeliverable power returns nan", bool(np.isnan(bad)))
    check("cold-battery note factors reused", pm.cap_factor(-10, "pre-warmed") == 0.90
          and pm.cap_factor(-10, "cold-soaked") == 0.80 and pm.cap_factor(20, "cold-soaked") == 1.0)
    T, q = pm.pack_self_heat_temp(cell, -10.0, 0.0)
    check("no current, no self-heating", T == -10.0 and q == 0.0)


def test_performance_identities():
    cell = pm.CELLS[P50B]
    m, _ = pm.baseline(P50B)
    ai = pm.air(2000.0, 20.0)
    vmax = pm.v_max_level(m, ai, cell, 20.0, pm.SOC_END)
    roc, _, _ = pm.climb(vmax, m, ai, cell, 20.0, pm.SOC_END, short=False)
    check("climb rate is zero at max level speed", abs(roc) < 0.05, f"{roc:.3f} m/s at {vmax:.2f} m/s")
    r1, _, _ = pm.climb(18.0, m, ai, cell, 20.0, pm.SOC_END, short=False)
    r2, _, _ = pm.climb(18.0, m, ai, cell, 20.0, pm.SOC_END, short=False, n=1 / math.cos(math.radians(30)))
    check("banked climb is worse than wings-level", r2 < r1, f"{r2:.2f} < {r1:.2f} m/s")
    vs = pm.v_stall(m, ai)
    ref = v1_perf(P50B).stall_tas_mps
    check("stall speed matches performance_v1.csv at 2,000 m", math.isclose(vs, ref, abs_tol=0.05), f"{vs:.2f} vs {ref:.2f}")
    la = pm.launch(m, ai, cell, 20.0, 1.0, 0.95 * vs)
    check("launch below stall is flagged (nan time)", math.isnan(la["t"]))
    a = pm.available(20.0, ai, cell, 20.0, pm.SOC_END, short=False)
    req = pm.required(20.0, 0.9 * a["T"], ai, cell, 20.0, pm.SOC_END)
    check("required() inverts the chain: thrust found", math.isclose(req["T"], 0.9 * a["T"], rel_tol=1e-3))
    check("available point respects every limit",
          a["I"] <= pm.I_MOTOR_PEAK and a["U"] <= pm.K_FF * a["V_bus"] + 1e-9 and a["v_cell"] >= pm.V_CELL_FLOOR
          and a["rpm"] <= pm.RPM_MAX and a["t_wind_ss"] <= pm.T_WIND_MAX)


if __name__ == "__main__":
    for f in (test_atmosphere, test_motor_calibration, test_prop_data, test_chain_vs_v3, test_battery,
              test_performance_identities):
        print(f"--- {f.__name__}")
        f()
    print(f"\n{check.failed} failed")
    sys.exit(1 if check.failed else 0)
