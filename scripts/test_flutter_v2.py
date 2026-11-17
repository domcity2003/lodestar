"""2026-11-14

Checks for flutter_v2 (p-k method): known answers before the Lodestar results are trusted.

1. Two-DOF typical section, Hodges & Pierce case HP-1 (a = -1/5, e = -1/10, mu = 20, r^2 = 6/25,
   sigma = 2/5): flutter U/(b w_theta) = 2.165, w/w_theta = 0.6545 (reference uses Peters' finite-state
   aerodynamics; Theodorsen should agree within ~1%); divergence U/(b w_theta) = r sqrt(mu / (1 + 2a)) = 2.828.
2. Goland wing, clamped, strip Theodorsen: 137.1 m/s, 69.1 rad/s (published 2D-strip comparison).
3. Lodestar baseline: with sweep, free-free and joint switched off (straight clamped beam, 2 pi lift slope,
   elevon locked) the p-k first crossing must match the V-g screen in structures.flutter within 3%.
4. Flap aerodynamics: steady flap lift slope 2 T10 equals thin-airfoil theory; apparent-mass matrix symmetric.
5. Rigid-body kinematics: rigid pitch / roll of the free-free swept model stores no strain energy.

Run: ../.venv/Scripts/python.exe -X utf8 test_flutter_v2.py   (from scripts/)
"""
import math

import numpy as np
import pandas as pd

import flutter_v2 as f
import structures as s


def typical_section():
    a, e, mu, r2, sig = -0.2, -0.1, 20.0, 6 / 25, 0.4
    rho = b = 1.0
    m = mu * math.pi * rho * b * b
    S, I = m * (e - a) * b, r2 * m * b * b
    M = np.array([[m, -S], [-S, I]])
    K = np.diag([m * sig**2, I])

    def aero(om, V):
        return f.strip_q(om, V, rho, b, a, 1.0)[0][:2, :2]

    br = f.pk(M, K, aero, np.arange(0.1, 3.0, 0.02), b)
    V, fhz, _ = f.first_crossing(br, eps=1e-6, f_min=0.0)   # nondimensional: w_theta = 1 rad/s
    A0 = f.strip_q(0.0, 1.0, rho, b, a, 1.0, steady=True)[0][:2, :2].real / (0.5 * rho)
    lam = np.linalg.eigvals(np.linalg.solve(K, A0))
    q_div = 1 / lam.real.max()
    V_div = math.sqrt(2 * q_div / rho)
    return V, 2 * math.pi * fhz, V_div


def test_typical_section():
    V, w, V_div = typical_section()
    assert abs(V / 2.165 - 1) < 0.015, V
    assert abs(w / 0.6545 - 1) < 0.02, w
    assert abs(V_div / math.sqrt(8) - 1) < 1e-6, V_div
    print(f"typical section HP-1: U_F {V:.4f} (ref 2.165), w_F {w:.4f} (ref 0.6545), U_D {V_div:.4f} (2.828)")


def goland():
    ne, L, c = 30, 6.096, 1.8288
    one = np.ones(ne)
    m = 35.71 * one
    st = f.Strips(yn=np.linspace(0, L, ne + 1), EI=9.77e6 * one, GJ=0.99e6 * one, m=m, S_a=m * 0.1 * c,
                  I_a=8.64 * one, b=c / 2 * one, a=(2 * 0.33 - 1) * one, eta=one)
    mdl = f.Model(st, f.Config())
    _, _, br = mdl.flutter(s.RHO0, np.arange(100.0, 170.0, 2.0), n_modes=4)
    V, fhz, _ = f.first_crossing(br)
    return V, 2 * math.pi * fhz


def test_goland():
    V, w = goland()
    assert abs(V / 137.1 - 1) < 0.02 and abs(w / 69.12 - 1) < 0.03, (V, w)
    print(f"Goland: {V:.2f} m/s, {w:.2f} rad/s (ref 137.1 m/s, 69.1 rad/s)")


def baseline(design="v1"):
    import wing_trade as wt
    f.use_design(design)
    tab = pd.read_csv(f.SECTIONS)
    beam = s.wing_beam(tab, extra_masses=tuple(wt.COMPACT.masses()))
    _, _, V_vg, w_vg = s.flutter(*beam, s.RHO0)
    st = f.lodestar_strips(tab, sweep=False, eta=1.0)
    _, _, _, V, fhz, _ = f.run_case(st, f.Config(bc="clamped"), s.RHO0)
    return V_vg, w_vg, V, 2 * math.pi * fhz


def test_baseline_reproduction():
    for design in ("v1", "compact_eps20_light"):
        V_vg, w_vg, V, w = baseline(design)
        assert abs(V / V_vg - 1) < 0.03, (design, V, V_vg)
        assert abs(w / w_vg - 1) < 0.03, (design, w, w_vg)
        print(f"{design}: p-k {V:.2f} m/s {w:.1f} rad/s vs V-g {V_vg:.2f} m/s {w_vg:.1f} rad/s")
    f.use_design("v1")


def test_flap_aero():
    c = 0.5
    T = f.t_functions(0.0, c)
    th = math.acos(-c)
    assert abs(2 * T["T10"] - 2 * (math.pi - th + math.sin(th))) < 1e-12
    # apparent mass (coefficients of -s^2) must be symmetric in (w, alpha, beta)
    Q1 = f.strip_q(1.0, 1e-9, 1.0, 1.0, -0.3, 1.0, c)[0]
    Q2 = f.strip_q(2.0, 1e-9, 1.0, 1.0, -0.3, 1.0, c)[0]
    Ma = (Q2 - Q1).real / 3.0          # Q = omega^2 * M_app at V -> 0
    assert np.allclose(Ma, Ma.T, atol=1e-6), Ma
    assert np.all(np.linalg.eigvalsh(0.5 * (Ma + Ma.T)) > 0)
    print("flap aerodynamics: lift slope and apparent-mass symmetry OK")


def test_rigid_modes_strain_free():
    tab = pd.read_csv(f.SECTIONS)
    st = f.lodestar_strips(tab, eta=1.0)
    body, _ = f.aircraft_body(st)
    for bc, n in (("sym", 2), ("anti", 1)):
        mdl = f.Model(st, f.Config(bc=bc, body=body, elevon=True, k_hinge=20.0, joint_kb=3000.0))
        for c in mdl.rig_cols:
            u = np.zeros(mdl.K.shape[0])
            u[c] = 1.0
            # rigid DOF alone moves only the root: extend it to a rigid motion of the whole wing
            ur = rigid_extension(mdl, st, bc, c)
            e = ur @ mdl.K @ ur
            assert e < 1e-8 * np.abs(mdl.K).max(), (bc, e)
    print("rigid pitch/heave/roll of the free-free swept model: zero strain energy")


def rigid_extension(mdl, st, bc, col):
    """Reduced vector of a rigid motion (unit heave, pitch or roll) of the full model."""
    full = np.zeros(mdl.nf)
    cL, sL = math.cos(st.sweep), math.sin(st.sweep)
    ys = []
    nodes = len(st.yn) + (mdl.jn is not None)
    for nd in range(nodes):
        k = nd if mdl.jn is None or nd <= mdl.jn else nd - 1
        ys.append(st.yn[k])
    ys = np.array(ys)
    x = st.x_ea0 + ys * math.tan(st.sweep)
    rig = mdl.nf - mdl.n_rig + col
    full[rig] = 1.0
    if bc == "sym" and col == 0:
        full[0:3 * nodes:3] = 1.0
    elif bc == "sym":
        xr = mdl.cfg.body["x_ref"]
        full[0:3 * nodes:3] = -(x - xr)
        full[1:3 * nodes:3] = -sL
        full[2:3 * nodes:3] = cL
    else:
        full[0:3 * nodes:3] = ys
        full[1:3 * nodes:3] = cL
        full[2:3 * nodes:3] = sL
    red, *_ = np.linalg.lstsq(mdl.T, full, rcond=None)
    assert np.allclose(mdl.T @ red, full, atol=1e-9)
    return red


def test_freeplay_describing_function():
    assert f.deadband_ratio([1.0])[0] == 0.0
    assert abs(f.deadband_ratio([1e6])[0] - 1) < 1e-5
    r = f.amplitude_for_ratio(0.5)
    assert abs(f.deadband_ratio([r])[0] - 0.5) < 1e-9
    print(f"dead band: k_eff/k = 0.5 at amplitude {r:.3f} x half free play")


if __name__ == "__main__":
    test_flap_aero()
    test_freeplay_describing_function()
    test_typical_section()
    test_goland()
    test_rigid_modes_strain_free()
    test_baseline_reproduction()
    print("all flutter_v2 tests passed")
