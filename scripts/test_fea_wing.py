"""Known-answer checks for fea_wing.py (CalculiX shell/solid wing model).

1. Cantilever thin-walled box of S4 shells, tip load: tip deflection and mid-span flange stress vs
   Euler-Bernoulli (within 3%).
2. Same box in the orthotropic +-45 glass skin material (G12 = 9 GPa, not E/2(1+nu)) with the material
   orientation used for the wing, tip torque: twist and wall shear stress vs Bredt (within 3%).
   This also checks that the orientation and local stress output behave as the wing post-processing assumes.
3. First bending frequency of the box vs Euler-Bernoulli (within 3%).
4. Load reconstruction: the discrete spar/rear-web forces reproduce the beam model's shear, bending and
   torque at every station (within 2% of the case maximum).
5. Wing FE structural mass vs the beam section table (within 3%).

Run (from scripts/):  ../.venv/Scripts/python.exe -X utf8 test_fea_wing.py
"""
import math

import numpy as np
import pandas as pd

import fea_wing as F
import structures as s

WORK = F.OUT / "tests"
L, B, H, T = 2.0, 0.1, 0.1, 1e-3
AXIS = (np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))


def _box(material=None):
    m, ids, ref, rot = F.box_model(L, B, H, T, n_len=40, n_side=4, material=material)
    m.nsets["REFN"] = [ref, rot]
    return m, ids, ref, rot


def _I():
    return 2 * T * H**3 / 12 + 2 * B * T * (H / 2) ** 2


def test_cantilever_bending():
    m, ids, ref, rot = _box()
    P = 100.0
    res, _ = F.run_ccx(m, [dict(type="static", cloads=[(ref, 3, P)], node_print=["REFN"], el_print=["SH"])],
                       WORK / "bend", "bend", ori=AXIS)
    U = F.disp_table(res, "REFN")
    w_fe = U.loc[ref, "uz"]
    w_eb = P * L**3 / (3 * 70e9 * _I())
    err_w = w_fe / w_eb - 1
    S = pd.DataFrame(F.blocks_of(res, "stresses")[0][3], columns=["eid", "s11", "s22", "s33", "s12", "s13", "s23"])
    meta = pd.DataFrame(m.meta).set_index("eid")
    S = S.astype({"eid": int}).set_index("eid").join(meta)
    j_mid = 19                                         # element row centred at y = 0.975 m
    y_mid = (j_mid + 0.5) * L / 40
    top = S[(S.j == j_mid) & (S.k.isin([9, 10]))]     # middle of the top flange (k = 8..11 is the top side)
    sig_eb = -P * (L - y_mid) * (H / 2) / _I()
    err_s = top.s11.mean() / sig_eb - 1
    print(f"bending: tip {w_fe * 1e3:.4f} mm vs EB {w_eb * 1e3:.4f} mm ({100 * err_w:+.2f}%), "
          f"flange stress {top.s11.mean() / 1e6:.3f} vs {sig_eb / 1e6:.3f} MPa ({100 * err_s:+.2f}%)")
    assert abs(err_w) < 0.03 and abs(err_s) < 0.03


def test_bredt_torsion_orthotropic():
    mat = F.ortho(F.SKIN_E, F.SKIN_E, F.SKIN_E, F.SKIN_NU, F.SKIN_NU, 0.3, F.SKIN_G, F.SKIN_G, F.SKIN_E / 2.6, 2375)
    m, ids, ref, rot = _box(mat)
    Tq = 50.0
    res, _ = F.run_ccx(m, [dict(type="static", cloads=[(rot, 2, Tq)], node_print=["REFN"], el_print=["SH"])],
                       WORK / "torsion", "torsion", ori=AXIS)
    U = F.disp_table(res, "REFN")
    phi_fe = U.loc[rot, "uy"]
    A = B * H
    J = 4 * A**2 * T / (2 * (B + H))
    phi_b = Tq * L / (F.SKIN_G * J)
    S = pd.DataFrame(F.blocks_of(res, "stresses")[0][3], columns=["eid", "s11", "s22", "s33", "s12", "s13", "s23"])
    meta = pd.DataFrame(m.meta).set_index("eid")
    S = S.astype({"eid": int}).set_index("eid").join(meta)
    mid = S[S.j == 20]
    tau_fe = mid.s12.abs().mean()
    tau_b = Tq / (2 * A * T)
    e1, e2 = phi_fe / phi_b - 1, tau_fe / tau_b - 1
    print(f"torsion: tip twist {phi_fe:.5f} rad vs Bredt {phi_b:.5f} ({100 * e1:+.2f}%), "
          f"wall shear {tau_fe / 1e6:.3f} vs {tau_b / 1e6:.3f} MPa ({100 * e2:+.2f}%)")
    assert abs(e1) < 0.03 and abs(e2) < 0.03


def test_box_frequency():
    m, ids, ref, rot = _box()
    res, _ = F.run_ccx(m, [dict(type="frequency", n=4, node_print=["REFN"])], WORK / "freq", "freq", ori=AXIS)
    mu = 2700 * T * 2 * (B + H)
    f_eb = 1.875104**2 / (2 * math.pi) * math.sqrt(70e9 * _I() / (mu * L**4))
    f_fe = res["eig"].freq_Hz.iloc[0]
    err = f_fe / f_eb - 1
    print(f"frequency: first bending {f_fe:.3f} Hz vs EB {f_eb:.3f} Hz ({100 * err:+.2f}%)")
    assert abs(err) < 0.03


def test_load_reconstruction():
    tab, loads = F.load_tables()
    g = F.governing_cases(loads)
    worst = 0.0
    for case in g.values():
        d = loads[loads.case == case].sort_values("y")
        rec = F.cut_loads(F.case_points(loads, case, factor=1.0), d.y.values)
        for col, k in (("V", 0), ("Mb", 1), ("T_spar", 2)):
            ref = d[col].values
            err = np.abs(rec[:, k] - ref).max() / np.abs(ref).max()
            worst = max(worst, err)
            assert err < 0.02, (case, col, err)
    print(f"load reconstruction: worst station error {100 * worst:.2f}% of case maximum ({len(g)} cases)")


def test_wing_mass():
    tab, _ = F.load_tables()
    w = F.build_wing(tab, **F.MESHES["coarse"], masses=False)
    fe, _ = F.fe_mass(w)
    beam = np.trapezoid(tab.m_per_m, tab.y)
    err = fe / beam - 1
    print(f"wing structure mass (half): FE {fe:.4f} kg vs section table {beam:.4f} kg ({100 * err:+.2f}%)")
    assert abs(err) < 0.03


if __name__ == "__main__":
    F.OUT.mkdir(parents=True, exist_ok=True)
    tests = [test_cantilever_bending, test_bredt_torsion_orthotropic, test_box_frequency, test_load_reconstruction,
             test_wing_mass]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {t.__name__}: {e}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    raise SystemExit(failed)
