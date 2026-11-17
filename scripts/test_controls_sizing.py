"""Known-answer checks for controls_sizing.py.

Run:  ../.venv/Scripts/python.exe -X utf8 test_controls_sizing.py   (from scripts/)
"""
import math
import sys

import numpy as np
import pandas as pd
from scipy.integrate import quad
from scipy.optimize import brentq

import controls_sizing as cs


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0


def test_thin_airfoil_flap():
    # Glauert: cl_alpha = 2 pi; cl_delta = 2(pi - th + sin th); cm_c/4,delta = -(2 sin th - sin 2 th)/4, cos th = 2E - 1
    ta = cs.thin_airfoil_flap(0.25)
    th = math.acos(-0.5)
    cld = 2 * (math.pi - th + math.sin(th))
    cmd = -(2 * math.sin(th) - math.sin(2 * th)) / 4
    check("thin airfoil cl_alpha = 2 pi", abs(ta["cl_alpha"] - 2 * math.pi) < 1e-3, f"{ta['cl_alpha']:.4f}")
    check("thin airfoil cl_delta (E 0.25) = 3.826", abs(ta["cl_delta"] - cld) < 2e-3, f"{ta['cl_delta']:.4f} vs {cld:.4f}")
    check("thin airfoil cm_delta (E 0.25) = -0.6495", abs(ta["cm_delta"] - cmd) < 2e-3, f"{ta['cm_delta']:.4f} vs {cmd:.4f}")
    check("hinge moment derivatives negative (aerodynamic load pushes trailing edge back)",
          ta["ch_alpha"] < 0 and ta["ch_delta"] < 0, f"ch_a {ta['ch_alpha']:.3f}, ch_d {ta['ch_delta']:.3f}")


def test_roll_first_order():
    # No rate limit: phi(t) = pss (t - tau (1 - exp(-t/tau))); solve for 60 deg and compare
    pss_per_deg, tau, V, d = 0.012, 0.15, 15.0, 8.0
    p = pss_per_deg * cs.effective(d) * 2 * V / cs.B
    t_exact = brentq(lambda t: p * (t - tau * (1 - math.exp(-t / tau))) - math.radians(60), 1e-3, 20)
    t_num = cs.time_to_bank(60, pss_per_deg, tau, V, d, 0)
    check("time to bank matches first-order closed form", abs(t_num - t_exact) < 5e-3, f"{t_num:.4f} vs {t_exact:.4f} s")


def test_dryden_variance():
    # One-sided Dryden PSD integrates to sigma^2
    var = quad(lambda w: cs.dryden_w_psd(w, 1.7, 120.0, 18.0), 0, np.inf, limit=400)[0]
    check("Dryden PSD integrates to sigma^2", abs(var - 1.7**2) < 1e-3 * 1.7**2, f"{var:.5f} vs {1.7**2:.5f}")


def test_effective_inverse():
    ok = all(abs(cs.effective(cs.geometric(e)) - e) < 1e-6 for e in (-15.0, -8.0, 4.0, 12.0, 16.0))
    check("geometric() inverts effective()", ok)


def test_mass_reproduces_v1():
    cad, src, dx = cs.cad_table()
    if "cad_v4" not in src:
        check("v1 mass balance reproduction (skipped: cad_v4 missing)", True)
        return
    mb = pd.read_csv(cs.ROOT / "out" / "wing_trade" / "aircraft_mass_balance_v1.csv").set_index(["cell", "kit"])
    dcg = dm = 0.0
    for (cell, kit), (_, mi) in cs.mass_states(cad, dx).items():
        r = mb.loc[(cell, kit)]
        dcg, dm = max(dcg, abs(mi["x"] - r.cg_m) * 1000), max(dm, abs(mi["m"] - r.mass_kg) * 1000)
    check("mass states reproduce aircraft_mass_balance_v1.csv (CG within 0.1 mm, mass within 1 g)",
          dcg < 0.1 and dm < 1.0, f"worst CG {dcg:.3f} mm, mass {dm:.2f} g")


def test_wire_field():
    # mu0 I / (2 pi r): 10 A at 0.1 m = 20 uT
    check("straight-wire field 10 A at 0.1 m = 20 uT", abs(cs.wire_field_uT(10, 0.1) - 20.0) < 1e-6)


if __name__ == "__main__":
    test_thin_airfoil_flap()
    test_roll_first_order()
    test_dryden_variance()
    test_effective_inverse()
    test_mass_reproduces_v1()
    test_wire_field()
    print(f"\n{'all checks passed' if not check.failed else f'{check.failed} check(s) failed'}")
    sys.exit(1 if check.failed else 0)
