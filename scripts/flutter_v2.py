"""2026-11-14

Second, independent flutter check of the Lodestar wing: p-k method on a free-free swept-beam aircraft model,
elevon (control-surface) flutter with hinge stiffness, free play and mass balance, plus divergence and reversal.

Why a second method: the first screen (structures.flutter) is a V-g (k) method on a straight, root-clamped beam
with the elevon locked. This script changes the solver and the model:

  - Solver: p-k (Hassig) iteration on the first-order state matrix at each true airspeed, instead of the
    k-method eigenvalue sweep. At zero damping both are exact harmonic solutions, so they must agree there.
  - Structure: Hermite bending + linear torsion finite elements laid along the swept elastic axis
    (24.4 deg, the 30.5%-chord shear-centre line), with streamwise aerodynamic and inertial strips.
    The streamwise strip pitch is alpha = phi cos(L) - w' sin(L): bending of an aft-swept wing washes the tip
    out, the classic sweep bending-torsion coupling. Normal-section torsion stiffness GJ_n = GJ_streamwise cos(L).
  - Aerodynamics: Theodorsen strip theory including the trailing-edge flap terms (Theodorsen 1935, NACA
    Report 496). Circulatory terms are scaled by eta(y) = cl_alpha(y) / 2 pi taken from AVL's spanwise lift-slope
    distribution of the actual planform (3D tip loss, sweep). Option: eta = cos(L) (simple sweep theory).
  - Free-free aircraft: symmetric half model with rigid heave and pitch of the body (pod, kit, gimbal, motor),
    so the short period can couple with wing first bending (body-freedom flutter). Antisymmetric half model with
    rigid roll. Root-clamped is kept for comparison.
  - Joint: the centre/outer break at y = 0.45 m is a duplicated node with a rotational spring in bending
    (and optionally torsion), swept from rigid to soft.
  - Elevon: hinge rotation is a third field, with its own torsion stiffness, the servo + linkage as a spring at
    the servo station, mass and static unbalance from the EPS + glass geometry, optional mass balance.
    Free play enters through the describing function of a dead band.

Run:   ../.venv/Scripts/python.exe -X utf8 flutter_v2.py   (from scripts/)
Tests: ../.venv/Scripts/python.exe -X utf8 test_flutter_v2.py
Writes: out/flutter_v2/ (CSV tables and figures)
"""
from __future__ import annotations

import math
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.linalg import eig, eigh, solve

import structures as s
import avl_run as av

ROOT = s.ROOT
OUT = ROOT / "out" / "flutter_v2"
DESIGNS = {
    # name: (section table, CAD mass table, battery span (m), battery x (m))
    "v1": ("v1_sections.csv", "cad_v4", (0.234, 0.376), 0.412),    # frozen layout (lodestar_v1.LAYOUT)
    "compact_eps20_light": ("compact_eps20_light_sections.csv", "cad_v3", (0.284, 0.426), 0.452),
}
DESIGN = "v1"
NOMINAL = "v1"
SECTIONS = ROOT / "out" / "wing_trade" / DESIGNS[DESIGN][0]
CAD_DIR = DESIGNS[DESIGN][1]


def use_design(name):
    """Point the module (and structures' battery placement, as wing_trade.main does) at a design."""
    global DESIGN, SECTIONS, CAD_DIR
    sec, cad, yb, xb = DESIGNS[name]
    DESIGN, SECTIONS, CAD_DIR = name, ROOT / "out" / "wing_trade" / sec, cad
    s.Y_BATT, s.X_BATT_GLOBAL = yb, xb


use_design(DESIGN)
RHO0, RHO_HI = s.RHO0, s.RHO_HI
SCREEN_EAS = 1.2 * s.VD          # 36 m/s EAS project flutter screen
SCREEN_115 = 1.15 * s.VD         # 34.5 m/s EAS (1.15 VD, CS-VLA / CS-23 style margin)
K_HINGE_V1 = 136.0               # N m/rad at the hinge, controls sizing (servo through a 2:1 linkage)
FREEPLAY_V1_DEG = 0.15           # total free play at the hinge, controls sizing
HINGE_XC = s.GEO.hinge_xc        # 0.75
Y_ELEVON = (s.Y_BREAK, s.GEO.span / 2)


# ============================================================================ Theodorsen strip aerodynamics
def t_functions(a, c):
    """Theodorsen's flap T-functions (NACA Report 496). a: elastic axis, c: hinge, both in semichords aft of
    mid-chord. a may be an array."""
    a = np.asarray(a, float)
    sq, ac = math.sqrt(1 - c * c), math.acos(c)
    T1 = -sq * (2 + c * c) / 3 + c * ac
    T3 = -(1 / 8 + c * c) * ac**2 + 0.25 * c * sq * ac * (7 + 2 * c * c) - (1 - c * c) * (5 * c * c + 4) / 8
    T4 = -ac + c * sq
    T5 = -(1 - c * c) - ac**2 + 2 * c * sq * ac
    T7 = -(1 / 8 + c * c) * ac + c * sq * (7 + 2 * c * c) / 8
    T8 = -sq * (2 * c * c + 1) / 3 + c * ac
    T9 = 0.5 * ((1 - c * c) ** 1.5 / 3 + a * T4)
    T10 = sq + ac
    T11 = ac * (1 - 2 * c) + sq * (2 - c)
    T12 = sq * (2 + c) - ac * (2 * c + 1)
    T13 = 0.5 * (-T7 - (c - a) * T1)
    return dict(T1=T1, T3=T3, T4=T4, T5=T5, T7=T7, T8=T8, T9=T9, T10=T10, T11=T11, T12=T12, T13=T13)


def strip_q(omega, V, rho, b, a, eta, c=0.5, steady=False):
    """Complex 3x3 strip force matrices per unit span for harmonic motion e^{i omega t}.

    Coordinates (all per strip, streamwise): w = plunge of the elastic axis, UP positive; alpha = pitch,
    nose-up positive; beta = flap rotation, trailing edge DOWN positive. Forces: L up, M nose-up about the
    elastic axis, H hinge moment in the beta direction. Returns array (G, 3, 3): F = Q @ [w, alpha, beta].
    Theodorsen's h is downward, so h = -w. Circulatory terms carry eta * C(k)."""
    b, a, eta = (np.atleast_1d(np.asarray(x, float)) for x in (b, a, eta))
    T = t_functions(a, c)
    if steady:
        sv, Ck = 0.0, np.ones_like(b)
    else:
        sv = 1j * omega
        Ck = s.theodorsen(np.maximum(omega * b / max(V, 1e-9), 1e-6))
    C = eta * Ck
    pi = math.pi
    nc = pi * rho * b**2                         # non-circulatory scale
    # bracket Q = hdot + V alpha + b(1/2 - a) alphadot + (V/pi) T10 beta + (b / 2pi) T11 betadot
    qw = -sv * np.ones_like(b)
    qa = V + b * (0.5 - a) * sv
    qb = V * T["T10"] / pi + b * T["T11"] * sv / (2 * pi)
    circ_L = 2 * pi * rho * V * b * C
    circ_M = 2 * pi * rho * V * b**2 * (a + 0.5) * C
    circ_H = -rho * V * b**2 * T["T12"] * C
    s2 = sv * sv
    Q = np.zeros((len(b), 3, 3), complex)
    # lift
    Q[:, 0, 0] = nc * (-s2) + circ_L * qw
    Q[:, 0, 1] = nc * (V * sv - b * a * s2) + circ_L * qa
    Q[:, 0, 2] = nc * (-(V / pi) * T["T4"] * sv - (b / pi) * T["T1"] * s2) + circ_L * qb
    # pitching moment about the elastic axis
    Q[:, 1, 0] = nc * (-b * a * s2) + circ_M * qw
    Q[:, 1, 1] = nc * (-V * b * (0.5 - a) * sv - b**2 * (1 / 8 + a**2) * s2) + circ_M * qa
    Q[:, 1, 2] = nc * (-(V**2 / pi) * (T["T4"] + T["T10"])
                       + (V * b / pi) * (-T["T1"] + T["T8"] + (c - a) * T["T4"] - 0.5 * T["T11"]) * sv
                       + (b**2 / pi) * (T["T7"] + (c - a) * T["T1"]) * s2) + circ_M * qb
    # hinge moment
    Q[:, 2, 0] = nc * ((b / pi) * T["T1"] * (-s2)) + circ_H * qw
    Q[:, 2, 1] = nc * ((V * b / pi) * (2 * T["T9"] + T["T1"] - (a - 0.5) * T["T4"]) * sv
                       - (2 * b**2 / pi) * T["T13"] * s2) + circ_H * qa
    Q[:, 2, 2] = nc * (-(V / pi) ** 2 * (T["T5"] - T["T4"] * T["T10"])
                       + (V * b / (2 * pi**2)) * T["T4"] * T["T11"] * sv
                       + (b / pi) ** 2 * T["T3"] * s2) + circ_H * qb
    return Q


# ============================================================================ p-k solver
def pk_roots(Mq, Kq, aero, V, b_ref, w_max, *, k_min=1e-3, n_grid=28, tol=1e-7, max_iter=80):
    """All p-k roots at one speed. The p-k equation (Hassig)
        M q'' - (A_I(w) / w) q' + (K - A_R(w)) q = 0,   A = generalized aero force matrix at frequency w,
    is solved for roots p whose imaginary part equals the w used in the aerodynamics. Instead of following one
    root per in-vacuo mode (which loses aerodynamic roots such as the short period), the frequency axis is
    scanned on a log grid; every eigenvalue near its own grid frequency seeds a fixed-point iteration, and
    converged roots are de-duplicated. Real (non-oscillatory) roots are taken at w = w_min.
    Returns list of (lambda, modal eigenvector)."""
    n = len(Mq)
    Minv = np.linalg.inv(Mq)
    I_, Z = np.eye(n), np.zeros((n, n))
    w_min = k_min * V / b_ref

    def solve_at(om):
        A = aero(om, V)
        sysm = np.block([[Z, I_], [Minv @ (A.real - Kq), Minv @ (A.imag / om)]])
        lam, vec = eig(sysm)
        return lam, vec[:n]

    roots = []

    def add(lam, vec):
        for l0, _ in roots:
            if abs(lam - l0) <= 1e-5 * (abs(l0) + 1.0):
                return
        roots.append((lam, vec / np.linalg.norm(vec)))

    # real roots
    lam, vec = solve_at(w_min)
    for l, v in zip(lam, vec.T):
        if abs(l.imag) <= w_min:
            add(complex(l.real, 0.0), v)
    # oscillatory roots
    seeds = []
    for om in np.geomspace(max(4 * w_min, 0.05), w_max, n_grid):
        lam, _ = solve_at(om)
        for l in lam:
            if l.imag > w_min and om / 1.35 <= l.imag <= om * 1.35:
                seeds.append(l)
    for l0 in seeds:
        if any(abs(l0 - r[0]) <= 1e-3 * (abs(r[0]) + 1.0) for r in roots):
            continue
        om, prev = l0.imag, l0
        lam_c = None
        for it in range(max_iter):
            lam, vec = solve_at(om)
            ok = lam.imag > 0
            if not ok.any():
                lam_c = None
                break
            cand = np.flatnonzero(ok)
            k = cand[np.argmin(np.abs(lam[cand] - prev))]
            lam_c, vec_c = lam[k], vec[:, k]
            om_new = lam_c.imag
            if om_new <= w_min:
                lam_c = None
                break
            if abs(om_new - om) <= tol * (om + 1.0):
                break
            prev = lam_c
            om = om_new if it < 30 else 0.5 * (om + om_new)
        else:
            lam_c = None
        if lam_c is not None:
            add(lam_c, vec_c)
    return roots


def pk(Mq, Kq, aero, speeds, b_ref, *, w_max=None, k_min=1e-3, labeler=None):
    """p-k sweep over true airspeed. Returns DataFrame of roots: V, branch, re, im (1/s), omega_hz, zeta,
    g (= 2 re / im, the V-g convention: positive = unstable), label. Branches are linked between speeds by
    nearest root (complex distance and eigenvector correlation), for plotting only; flutter detection uses
    all roots at each speed."""
    from scipy.optimize import linear_sum_assignment
    w2 = eigh(Kq, Mq, eigvals_only=True)
    if w_max is None:
        w_max = 1.3 * math.sqrt(max(w2.max(), 1.0))
    rows, prev, next_id = [], [], 0
    for V in speeds:
        roots = pk_roots(Mq, Kq, aero, V, b_ref, w_max, k_min=k_min)
        ids = [-1] * len(roots)
        if prev and roots:
            cost = np.array([[abs(l - pl) / (abs(pl) + 1.0) + (1 - abs(np.vdot(pv, v)) ** 2)
                              for (pl, pv, _) in prev] for (l, v) in roots])
            r, c = linear_sum_assignment(cost)
            for i, j in zip(r, c):
                if cost[i, j] < 0.6:
                    ids[i] = prev[j][2]
        for i in range(len(roots)):
            if ids[i] < 0:
                ids[i] = next_id
                next_id += 1
        prev = [(l, v, i) for (l, v), i in zip(roots, ids)]
        for (l, v), i in zip(roots, ids):
            im = abs(l.imag)
            rows.append(dict(V=V, branch=i, re=l.real, im=im, omega_hz=im / (2 * math.pi),
                             zeta=-l.real / max(abs(l), 1e-12), g=2 * l.real / im if im > 0 else -np.inf,
                             label=labeler(v) if labeler else "", vec=v))
    return pd.DataFrame(rows)


def first_crossing(roots, V_max=math.inf, eps=1e-3, f_min=0.3):
    """Lowest speed with an oscillatory root of positive real part. Linear interpolation of the real part
    against the nearest root (frequency) at the previous speed. Roots below f_min Hz (neutral rigid / free-elevon
    roots, numerically ~0) and growth rates below eps (1/s) are ignored. Returns (V_tas, freq_hz, label)."""
    speeds = np.sort(roots.V.unique())
    for i, V in enumerate(speeds):
        d = roots.loc[(roots.V == V) & (roots.omega_hz > f_min)]
        bad = d.loc[d.re > eps]
        if bad.empty:
            continue
        r = bad.loc[bad.re.idxmax()]
        if V > V_max:
            break
        if i == 0:
            return V, r.omega_hz, r.label
        p = roots.loc[(roots.V == speeds[i - 1]) & (roots.omega_hz > f_min)]
        if p.empty:
            return V, r.omega_hz, r.label
        q = p.iloc[int(np.argmin(np.abs(p.im.to_numpy() - r.im)))]
        t = -q.re / (r.re - q.re) if r.re != q.re else 1.0
        t = min(max(t, 0.0), 1.0)
        return (speeds[i - 1] + t * (V - speeds[i - 1]), q.omega_hz + t * (r.omega_hz - q.omega_hz), r.label)
    return math.inf, math.nan, ""


def real_root_instability(roots, eps=0.05):
    """Lowest speed with a positive real (non-oscillatory) root: static instability / divergence of the
    model (for the free-free aircraft, loss of the flexible static margin)."""
    d = roots.loc[(roots.im == 0) & (roots.re > eps)]
    return float(d.V.min()) if len(d) else math.inf


# ============================================================================ model description
@dataclass
class Strips:
    """Per-element properties on projected span (streamwise strips). Mass terms per metre of projected span.
    S_a: first mass moment about the elastic axis (positive aft); I_a: about the elastic axis."""
    yn: np.ndarray
    EI: np.ndarray
    GJ: np.ndarray            # streamwise-section value
    m: np.ndarray
    S_a: np.ndarray
    I_a: np.ndarray
    b: np.ndarray
    a: np.ndarray
    eta: np.ndarray
    sweep: float = 0.0        # elastic-axis sweep (rad); 0 = straight projected-span beam
    sweep_h: float = 0.0      # hinge-line sweep (rad)
    x_ea0: float = 0.0        # elastic-axis x at the root (m)
    gj_normal: bool = True    # GJ_n = GJ cos(sweep)
    # elevon, per element (zeros where no elevon)
    el: np.ndarray = None     # mask
    xi_h: np.ndarray = None   # hinge offset aft of the elastic axis (m)
    S_b: np.ndarray = None    # elevon first moment about hinge per m (positive aft = unbalanced)
    I_b: np.ndarray = None    # elevon inertia about hinge per m
    GJ_e: np.ndarray = None   # elevon torsion stiffness about the hinge line
    ch: float = 0.5           # hinge position in semichords aft of mid-chord


@dataclass
class Config:
    bc: str = "clamped"                    # clamped | sym | anti
    joint_kb: float | None = None          # joint bending rotational stiffness, N m/rad (None = rigid)
    joint_kt: float | None = None          # joint torsion stiffness, N m/rad (None = rigid)
    y_joint: float = s.Y_BREAK
    elevon: bool = False                   # elevon rotation DOF active (False = locked to the wing)
    k_hinge: float = K_HINGE_V1            # servo + linkage stiffness at the hinge, N m/rad
    y_servo: float = s.Y_SERVO
    body: dict = field(default_factory=dict)   # half-model rigid body: m, S (about x_ref, + aft), Iyy, Ixx, x_ref
    n_gauss: int = 4
    dCma: float = 0.0                      # rigid-body pitch-stiffness correction (per rad, on S and MAC), see pitch_fix
    dCmq: float = 0.0                      # rigid-body pitch-damping correction (on q c / 2V)


def gauss01(n):
    x, w = np.polynomial.legendre.leggauss(n)
    return (x + 1) / 2, w / 2


class Model:
    """Assembles the reduced mass/stiffness matrices and the strip-aero projection operators."""

    def __init__(self, st: Strips, cfg: Config):
        self.st, self.cfg = st, cfg
        yn = st.yn
        ne = len(yn) - 1
        cL, sL = math.cos(st.sweep), math.sin(st.sweep)
        cH = math.cos(st.sweep_h)
        has_el = cfg.elevon and st.el is not None and st.el.any()
        # ---- nodes (duplicate at the joint)
        jn = None
        if cfg.joint_kb is not None or cfg.joint_kt is not None:
            jn = int(np.argmin(np.abs(yn - cfg.y_joint)))
            if abs(yn[jn] - cfg.y_joint) > 1e-9 or jn in (0, ne):
                raise ValueError("The joint must coincide with an interior node.")
        self.jn = jn
        elem_nodes = [(e, e + 1) if (jn is None or e < jn) else (e + 1, e + 2) for e in range(ne)]
        n_nodes = ne + 1 + (jn is not None)
        nb = 3 * n_nodes
        # elevon nodes: projected-span nodes inside the elevon
        el_nodes = {}
        if has_el:
            for e in range(ne):
                if st.el[e]:
                    for y in (yn[e], yn[e + 1]):
                        key = round(float(y), 9)
                        if key not in el_nodes:
                            el_nodes[key] = nb + len(el_nodes)
        n_el = len(el_nodes)
        n_rig = {"clamped": 0, "sym": 2, "anti": 1}[cfg.bc]
        nf = nb + n_el + n_rig
        self.nf, self.nb, self.n_rig = nf, nb, n_rig
        rig0 = nb + n_el
        M = np.zeros((nf, nf))
        K = np.zeros((nf, nf))
        xg, wg = gauss01(cfg.n_gauss)
        P_list, w_list, b_list, a_list, eta_list = [], [], [], [], []
        self.y_g = []
        for e, (n1, n2) in enumerate(elem_nodes):
            dy = yn[e + 1] - yn[e]
            Ls = dy / cL
            idx = [3 * n1, 3 * n1 + 1, 3 * n1 + 2, 3 * n2, 3 * n2 + 1, 3 * n2 + 2]
            el_here = has_el and st.el[e]
            if el_here:
                idx += [el_nodes[round(float(yn[e]), 9)], el_nodes[round(float(yn[e + 1]), 9)]]
            nd = len(idx)
            # stiffness: bending (Hermite), torsion (linear) along the beam
            L = Ls
            kb = st.EI[e] / L**3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L**2, -6 * L, 2 * L**2],
                                             [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L**2, -6 * L, 4 * L**2]])
            gj = st.GJ[e] * (cL if st.gj_normal else 1.0)
            kt = gj / L * np.array([[1, -1], [-1, 1]])
            Ke = np.zeros((nd, nd))
            ib, it = [0, 1, 3, 4], [2, 5]
            Ke[np.ix_(ib, ib)] += kb
            Ke[np.ix_(it, it)] += kt
            if el_here:
                Lh = dy / cH
                Ke[np.ix_([6, 7], [6, 7])] += st.GJ_e[e] / Lh * np.array([[1, -1], [-1, 1]])
            K[np.ix_(idx, idx)] += Ke
            # mass and aero projections from streamwise strips at Gauss points
            Me = np.zeros((nd, nd))
            for xi, wi in zip(xg, wg):
                Nw = np.zeros(nd)
                Nb_ = np.zeros(nd)
                Np = np.zeros(nd)
                Nd = np.zeros(nd)
                Nw[[0, 1, 3, 4]] = [1 - 3 * xi**2 + 2 * xi**3, L * (xi - 2 * xi**2 + xi**3),
                                    3 * xi**2 - 2 * xi**3, L * (-xi**2 + xi**3)]
                Nb_[[0, 1, 3, 4]] = [(-6 * xi + 6 * xi**2) / L, 1 - 4 * xi + 3 * xi**2,
                                     (6 * xi - 6 * xi**2) / L, -2 * xi + 3 * xi**2]
                Np[[2, 5]] = [1 - xi, xi]
                Na = cL * Np - sL * Nb_
                if el_here:
                    Nd[[6, 7]] = [1 - xi, xi]
                Nf = cH * Nd                      # streamwise flap angle
                wgt = wi * dy
                Me += wgt * (st.m[e] * np.outer(Nw, Nw) - st.S_a[e] * (np.outer(Nw, Na) + np.outer(Na, Nw))
                             + st.I_a[e] * np.outer(Na, Na))
                if el_here:
                    Ib, Sb, xh = st.I_b[e], st.S_b[e], st.xi_h[e]
                    Me += wgt * (-Sb * (np.outer(Nw, Nf) + np.outer(Nf, Nw))
                                 + (Ib + xh * Sb) * (np.outer(Na, Nf) + np.outer(Nf, Na)) + Ib * np.outer(Nf, Nf))
                P = np.zeros((3, nf))
                P[0, idx], P[1, idx], P[2, idx] = Nw, Na, Nf
                P_list.append(P)
                w_list.append(wgt)
                b_list.append(st.b[e])
                a_list.append(st.a[e])
                eta_list.append(st.eta[e])
                self.y_g.append(yn[e] + xi * dy)
            M[np.ix_(idx, idx)] += Me
        # servo spring on the hinge rotation (linear interpolation between elevon nodes)
        if has_el:
            e = int(np.searchsorted(yn, cfg.y_servo) - 1)
            xi = (cfg.y_servo - yn[e]) / (yn[e + 1] - yn[e])
            n_ = np.zeros(nf)
            n_[el_nodes[round(float(yn[e]), 9)]] += 1 - xi
            n_[el_nodes[round(float(yn[e + 1]), 9)]] += xi
            self.servo_vec = n_
            K += cfg.k_hinge * np.outer(n_, n_)
        # joint springs between the duplicate nodes
        if jn is not None:
            for comp, k in ((1, cfg.joint_kb), (2, cfg.joint_kt)):
                if k is not None:
                    i1, i2 = 3 * jn + comp, 3 * (jn + 1) + comp
                    K[np.ix_([i1, i2], [i1, i2])] += k * np.array([[1, -1], [-1, 1]])
        # rigid body (half model)
        bd = cfg.body
        if cfg.bc == "sym":
            r = [rig0, rig0 + 1]
            M[np.ix_(r, r)] += np.array([[bd["m"], -bd["S"]], [-bd["S"], bd["Iyy"]]])
        elif cfg.bc == "anti":
            M[rig0, rig0] += bd.get("Ixx", 0.0)
        # ---- constraint transform full = T @ reduced
        cols = {}
        T = np.zeros((nf, nf))
        nc = 0

        def new():
            nonlocal nc
            nc += 1
            return nc - 1

        rig_cols = [new() for _ in range(n_rig)]
        for i, rc in enumerate(rig_cols):
            T[rig0 + i, rc] = 1.0
        # root node 0
        if cfg.bc == "sym":
            xr = bd["x_ref"]
            T[0, rig_cols[0]], T[0, rig_cols[1]] = 1.0, -(st.x_ea0 - xr)
            T[1, rig_cols[1]] = -sL
            T[2, rig_cols[1]] = cL
        elif cfg.bc == "anti":
            T[1, rig_cols[0]] = cL
            T[2, rig_cols[0]] = sL
        for nd_ in range(1, n_nodes):
            for comp in range(3):
                i = 3 * nd_ + comp
                if jn is not None and nd_ == jn + 1:
                    tied = comp == 0 or (comp == 1 and cfg.joint_kb is None) or (comp == 2 and cfg.joint_kt is None)
                    if tied:
                        T[i] = T[3 * jn + comp]
                        continue
                T[i, new()] = 1.0
        for key, i in el_nodes.items():
            T[i, new()] = 1.0
        T = T[:, :nc]
        self.T = T
        self.M = T.T @ M @ T
        self.K = T.T @ K @ T
        self.M_full, self.K_full = M, K
        self.P = np.array(P_list) @ T            # (G, 3, n_red)
        self.wg = np.array(w_list)
        self.bg, self.ag, self.etag = map(np.array, (b_list, a_list, eta_list))
        self.y_g = np.array(self.y_g)
        self.el_nodes = el_nodes
        self.rig_cols = rig_cols

    # ---------------------------------------------------------------- aero
    def aero_full(self, omega, V, rho, steady=False):
        Q = strip_q(omega, V, rho, self.bg, self.ag, self.etag, self.st.ch, steady=steady)
        return np.einsum("g,gim,gij,gjn->mn", self.wg, self.P, Q, self.P, optimize=True)

    def modal(self, n_modes):
        w2, Phi = eigh(self.K, self.M)
        Phi = Phi[:, :n_modes]
        Pm = self.P @ Phi
        return w2[:n_modes], Phi, Pm

    def flutter(self, rho, speeds, n_modes=16, b_ref=None, f_max=100.0):
        """p-k over speeds with n_modes in-vacuo modes as the basis; roots are searched up to f_max Hz."""
        w2, Phi, Pm = self.modal(n_modes)
        Mq, Kq = Phi.T @ self.M @ Phi, Phi.T @ self.K @ Phi
        Kq = 0.5 * (Kq + Kq.T)
        wg, bg, ag, eg, ch = self.wg, self.bg, self.ag, self.etag, self.st.ch
        n = Pm.shape[2]
        # precomputed projection: A = sum_g Q_g[i,j] * (w_g P_gi^T P_gj)
        B = np.ascontiguousarray(np.einsum("g,gim,gjn->gijmn", wg, Pm, Pm).reshape(len(wg) * 9, n * n))

        def aero(om, V):
            q = strip_q(om, V, rho, bg, ag, eg, ch).reshape(-1)
            A = (q.real @ B + 1j * (q.imag @ B)).reshape(n, n)
            if (self.cfg.dCma or self.cfg.dCmq) and self.cfg.bc == "sym":
                A = A + Phi.T @ self.pitch_fix(om, V, rho) @ Phi
            return A

        b_ref = b_ref or float(self.st.b[0])
        br = pk(Mq, Kq, aero, speeds, b_ref, w_max=2 * math.pi * f_max, labeler=self.labeler(Phi))
        return np.sqrt(np.maximum(w2, 0)), Phi, br

    def mode_labels(self, Phi, w=None):
        """Name each in-vacuo mode: 'rigid' (zero frequency, body motion), otherwise by where its strain
        energy sits (bending / torsion / elevon), with its frequency."""
        if w is None:
            w = np.sqrt(np.maximum(np.einsum("ij,ik,kj->j", Phi, self.K, Phi), 0))
        nb, nf, nr = self.nb, self.nf, self.n_rig
        labels = []
        for j in range(Phi.shape[1]):
            u = self.T @ Phi[:, j]
            if w[j] < 2 * math.pi * 0.3:
                ke = u * (self.M_full @ u)
                labels.append("elevon free" if ke[nb:nf - nr].sum() > 0.5 * ke.sum() else "rigid")
                continue
            se = u * (self.K_full @ u)
            parts = dict(bending=se[0:nb:3].sum() + se[1:nb:3].sum(), torsion=se[2:nb:3].sum(),
                         elevon=se[nb:nf - nr].sum())
            top = max(parts, key=lambda k: parts[k])
            labels.append(f"{top} {w[j] / 2 / math.pi:.1f} Hz")
        return labels

    def pitch_fix(self, om, V, rho):
        """Quasi-steady correction of the rigid-body pitching moment, so the strip model's pitch stiffness can be
        matched to AVL (the strips omit the pod and winglets and the 3D downwash at the root). Half model:
        dM = q (S/2) MAC [dCma alpha_body + dCmq thetadot MAC / (2V)], alpha_body = theta - zdot / V."""
        n = self.K.shape[0]
        A = np.zeros((n, n), complex)
        if self.cfg.bc != "sym":
            return A
        z, th = self.rig_cols
        c = s.GEO.mac
        f = 0.5 * rho * V**2 * (s.GEO.S / 2) * c
        A[th, th] += f * (self.cfg.dCma + self.cfg.dCmq * c / (2 * V) * 1j * om)
        A[th, z] += f * self.cfg.dCma * (-1j * om / V)
        return A

    def labeler(self, Phi):
        """Name a root by the in-vacuo modes that make up its (complex) mode shape, weighted by |q_j|^2
        (unit modal mass); rigid modes are pooled."""
        names = self.mode_labels(Phi)

        def lab(v):
            share = {}
            p = np.abs(v) ** 2
            p = p / p.sum()
            for nm, x in zip(names, p):
                share[nm] = share.get(nm, 0.0) + x
            ordered = sorted(share, key=lambda k: -share[k])
            return " + ".join(f"{k} ({share[k]:.0%})" for k in ordered[:2] if share[k] > 0.12)
        return lab


# ============================================================================ Lodestar inputs
def avl_lift_slope(geo=s.GEO, x_ref=0.422):
    """AVL spanwise section lift slope cl_alpha(y) (per rad) and rigid-aircraft derivatives at x_ref."""
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        av.write_avl(geo, x_ref, d / "rw.avl")
        keys = ["oper", "d1 d1 0", "d2 d2 0", "a a 0", "x", "fs", "fs0.txt", "a a 4", "x", "fs", "fs4.txt",
                "st", "st4.txt", "", "quit", ""]
        subprocess.run([str(av.AVL_EXE), "rw.avl"], cwd=d, input="\n".join(keys) + "\n", text=True,
                       capture_output=True, timeout=60)
        y0, _, cl0 = av.parse_strips((d / "fs0.txt").read_text())
        y4, _, cl4 = av.parse_strips((d / "fs4.txt").read_text())
        stab = av.parse_eq((d / "st4.txt").read_text())
    k = y0 >= 0
    order = np.argsort(y0[k])
    cla = (cl4[k] - cl0[k])[order] / math.radians(4)
    return y0[k][order], cla, stab


def elevon_section(c, core_rho=s.EPS20.rho, core_G=s.EPS20.G, lay=s.LAYUPS[0], hinge_line_mass=0.010):
    """Elevon (75-100% chord) mass, first and second moments about the hinge per metre of streamwise span,
    and its torsion stiffness about the hinge (open foam wedge + Bredt glass skin cell)."""
    x = np.linspace(HINGE_XC, 1.0, 201) * c
    t = np.array([s.thick(v / c) for v in x]) * c
    skin_areal = 2 * (lay.areal_ply * lay.n_ply + lay.finish_areal)       # both surfaces
    dm = core_rho * t + skin_areal
    r = x - x[0]
    m = np.trapezoid(dm, x) + hinge_line_mass
    S = np.trapezoid(dm * r, x)
    I = np.trapezoid(dm * (r**2 + t**2 / 12), x)
    A = np.trapezoid(t, x)
    P = 2 * (x[-1] - x[0]) + t[0]
    gt = lay.skin.G * lay.t_ply * lay.n_ply
    GJ = core_G * np.trapezoid(t**3, x) / 3 + 4 * A**2 / (P / gt)
    return dict(m=m, S=S, I=I, GJ=GJ, c_e=x[-1] - x[0], t_hinge=t[0])


def lodestar_strips(tab=None, ne=30, *, sweep=True, eta="avl", gj_normal=True, balance=None, gj_scale=1.0,
                    ei_scale=1.0):
    """Strips for the baseline wing. balance: dict(mass_kg per elevon, arm_m ahead of hinge, where='uniform'|'tip'|'servo')."""
    import wing_trade as wt
    tab = pd.read_csv(SECTIONS) if tab is None else tab
    yn, EI, GJ, m, S_a, I_a, b, a = s.wing_beam(tab, ne=ne, extra_masses=tuple(wt.COMPACT.masses()))
    yc = (yn[1:] + yn[:-1]) / 2
    g = s.GEO
    xsc_c = float(tab.xsc_c.iloc[0])
    x_ea = lambda y: g.x_le(y) + xsc_c * g.chord(y)            # noqa: E731
    lam = math.atan((x_ea(yn[-1]) - x_ea(0.0)) / yn[-1]) if sweep else 0.0
    x_h = lambda y: g.x_le(y) + HINGE_XC * g.chord(y)          # noqa: E731
    lam_h = math.atan((x_h(Y_ELEVON[1]) - x_h(Y_ELEVON[0])) / (Y_ELEVON[1] - Y_ELEVON[0])) if sweep else 0.0
    if isinstance(eta, str) and eta == "avl":
        cache = OUT / "avl_lift_slope.csv"
        if cache.exists():
            d = pd.read_csv(cache)
            ya, cla = d.y.to_numpy(), d.cl_alpha.to_numpy()
        else:
            ya, cla, _ = avl_lift_slope()
        eta_e = np.interp(yc, ya, cla) / (2 * math.pi)
    elif isinstance(eta, str) and eta == "cos":
        eta_e = np.full(len(yc), math.cos(lam))
    else:
        eta_e = np.full(len(yc), float(eta))
    el = (yc > Y_ELEVON[0]) & (yc < Y_ELEVON[1])
    props = [elevon_section(g.chord(y)) if f else dict(m=0, S=0, I=0, GJ=0) for y, f in zip(yc, el)]
    S_b = np.array([p["S"] for p in props])
    I_b = np.array([p["I"] for p in props])
    GJ_e = np.array([p["GJ"] for p in props])
    xi_h = np.array([(HINGE_XC - xsc_c) * g.chord(y) for y in yc])
    if balance:
        mb, arm, where = balance["mass_kg"], balance["arm_m"], balance.get("where", "uniform")
        dy = np.diff(yn)
        if where == "uniform":
            per = np.where(el, mb / (Y_ELEVON[1] - Y_ELEVON[0]), 0.0)
        else:
            y_at = Y_ELEVON[1] - 1e-6 if where == "tip" else s.Y_SERVO
            e = int(np.searchsorted(yn, y_at) - 1)
            per = np.zeros(len(yc))
            per[e] = mb / dy[e]
        # balance mass sits arm_m ahead of the hinge (in the wing cove / on a horn), moving with the elevon
        S_b = S_b - per * arm
        I_b = I_b + per * arm**2
        # it also adds to the wing section mass at x = hinge - arm
        xo = xi_h - arm
        m = m + per
        S_a = S_a + per * xo
        I_a = I_a + per * xo**2
    EI, GJ = EI * ei_scale, GJ * gj_scale
    return Strips(yn=yn, EI=EI, GJ=GJ, m=m, S_a=S_a, I_a=I_a, b=b, a=a, eta=eta_e, sweep=lam, sweep_h=lam_h,
                  x_ea0=x_ea(0.0), gj_normal=gj_normal and sweep, el=el, xi_h=xi_h, S_b=S_b, I_b=I_b, GJ_e=GJ_e)


def body_items(kit_on=True):
    """Rigid body (non-wing) items from the CAD v3 mass table: pod, gimbal, avionics, motor, kit, misc."""
    cad = pd.read_csv(ROOT / "out" / CAD_DIR / "mass_cg.csv")
    rows = []
    for r in cad.itertuples():
        if r.item.startswith("wing structure") or r.item.startswith("batteries") or r.item == "winglets":
            continue
        if r.item.startswith("elevon mass balance"):   # modelled on the elevon itself in the balance cases
            continue
        if not kit_on and r.item in ("rescue kit", "kit parachute"):
            continue
        rows.append((r.item, r.mass_kg, r.x, r.z))
    return rows


def aircraft_body(st: Strips, kit_on=True):
    """Half-model rigid body about the aircraft CG (wing strips + body items). Returns dict and totals."""
    yc = (st.yn[1:] + st.yn[:-1]) / 2
    dy = np.diff(st.yn)
    g = s.GEO
    x_ea = st.x_ea0 + yc * math.tan(st.sweep) if st.sweep else np.array([st.x_ea0 + 0 * y for y in yc])
    x_wing = x_ea + st.S_a / st.m
    m_wing = 2 * np.sum(st.m * dy)
    items = body_items(kit_on)
    m_body = sum(i[1] for i in items)
    x_cg = (2 * np.sum(st.m * dy * x_wing) + sum(i[1] * i[2] for i in items)) / (m_wing + m_body)
    S_body = sum(i[1] * (i[2] - x_cg) for i in items)
    I_body = sum(i[1] * ((i[2] - x_cg) ** 2 + i[3] ** 2) for i in items)
    Ixx_body = sum(i[1] * i[3] ** 2 for i in items)
    # wing pitch inertia about the CG for the record
    I_wing = 2 * np.sum(dy * (st.m * (x_ea - x_cg) ** 2 + 2 * st.S_a * (x_ea - x_cg) + st.I_a))
    body = dict(m=m_body / 2, S=S_body / 2, Iyy=I_body / 2, Ixx=Ixx_body / 2, x_ref=x_cg)
    totals = dict(mass_kg=m_wing + m_body, x_cg=x_cg, Iyy=I_wing + I_body, m_wing=m_wing, m_body=m_body)
    return body, totals


def tas(eas, rho):
    return eas * math.sqrt(RHO0 / rho)


def eas(tas_, rho):
    return tas_ * math.sqrt(rho / RHO0)


SPEEDS = np.concatenate([np.arange(2.0, 60.0, 2.0), np.arange(60.0, 180.1, 5.0)])


def run_case(st, cfg, rho, speeds=None, n_modes=16):
    speeds = SPEEDS if speeds is None else speeds
    mdl = Model(st, cfg)
    wn, Phi, br = mdl.flutter(rho, speeds, n_modes=n_modes)
    V, f, lab = first_crossing(br)
    return mdl, wn, br, V, f, lab


# ============================================================================ static aeroelasticity
def divergence_q(st, cfg):
    """Lowest positive divergence dynamic pressure (Pa) of the model, elevon locked; inf if none.
    Steady aero is linear in q = 0.5 rho V^2, so the result is EAS-based."""
    mdl = Model(st, replace(cfg, elevon=False))
    A0 = mdl.aero_full(0.0, 1.0, 2.0, steady=True).real       # per unit q (rho=2, V=1 -> q=1)
    lam = eig(A0, mdl.K, right=False)
    lam = lam[np.isfinite(lam)]
    pos = lam[(np.abs(lam.imag) < 1e-9 * np.abs(lam).max()) & (lam.real > 0)].real
    neg = lam[(np.abs(lam.imag) < 1e-9 * np.abs(lam).max()) & (lam.real < 0)].real
    q_d = 1 / pos.max() if len(pos) else math.inf
    q_neg = 1 / neg.min() if len(neg) else -math.inf
    return q_d, q_neg


def roll_effectiveness(st, q, *, cfg=None):
    """Rolling moment from a uniform antisymmetric elevon deflection on the flexible, root-clamped wing,
    divided by the rigid-wing value, at dynamic pressure q (Pa)."""
    cfg = cfg or Config(bc="clamped")
    mdl = Model(st, replace(cfg, elevon=True, k_hinge=0.0))
    # Separate wing DOFs (kept) from elevon DOFs (prescribed hinge rotation = 1 rad).
    el_cols = [c for c in range(mdl.T.shape[1]) if np.any(mdl.T[list(mdl.el_nodes.values()), c])]
    wing_cols = [c for c in range(mdl.T.shape[1]) if c not in el_cols]
    A0 = mdl.aero_full(0.0, 1.0, 2.0, steady=True).real
    dlt = np.zeros(mdl.T.shape[1])
    dlt[el_cols] = 1.0
    Kw = mdl.K[np.ix_(wing_cols, wing_cols)]
    Aww = A0[np.ix_(wing_cols, wing_cols)]
    f = A0[np.ix_(wing_cols, el_cols)] @ dlt[el_cols]
    u = np.zeros_like(dlt)
    u[el_cols] = 1.0
    u[wing_cols] = solve(Kw - q * Aww, q * f)
    # strip lift per unit q at each Gauss point: row 0 of the steady strip matrix
    Q = strip_q(0.0, 1.0, 2.0, mdl.bg, mdl.ag, mdl.etag, st.ch, steady=True).real
    x = np.einsum("gin,n->gi", mdl.P, u)
    lift = np.einsum("gj,gj->g", Q[:, 0, :], x)
    x_r = np.einsum("gin,n->gi", mdl.P, dlt)
    lift_r = np.einsum("gj,gj->g", Q[:, 0, :], x_r)
    return float(np.sum(mdl.wg * lift * mdl.y_g) / np.sum(mdl.wg * lift_r * mdl.y_g))


def reversal_q(st, q_hi=20000.0):
    from scipy.optimize import brentq
    qs = np.linspace(0, q_hi, 81)
    vals = [roll_effectiveness(st, q) for q in qs]
    for i in range(len(qs) - 1):
        if vals[i] > 0 >= vals[i + 1]:
            return brentq(lambda q: roll_effectiveness(st, q), qs[i], qs[i + 1]), qs, vals
    return math.inf, qs, vals


# ============================================================================ free play (describing function)
def deadband_ratio(A_over_d):
    """Describing-function stiffness ratio k_eff / k of a spring with a dead band of half-width d."""
    r = np.asarray(A_over_d, float)
    out = np.zeros_like(r)
    m = r > 1
    x = 1 / r[m]
    out[m] = 1 - (2 / math.pi) * (np.arcsin(x) + x * np.sqrt(1 - x * x))
    return out


def amplitude_for_ratio(ratio):
    """Inverse of deadband_ratio: amplitude / half free play giving k_eff/k = ratio (0..1)."""
    from scipy.optimize import brentq
    if ratio <= 0:
        return 1.0
    if ratio >= 1:
        return math.inf
    return brentq(lambda r: deadband_ratio([r])[0] - ratio, 1.0 + 1e-12, 1e7)


# ============================================================================ study helpers
def joint_stiffness(E_tube=100e9, D=0.020, d=0.016, engagement=0.160, EI_wing=None):
    """Extra bending compliance of the tube-in-sleeve joint relative to a continuous spar.
    The tube carries the whole moment at the break and hands it to the sleeves linearly over each engagement,
    so the added rotation is M * engagement * (1/EI_tube - 1/EI_wing). E_tube = 100 GPa is an ASSUMPTION
    (roll-wrapped, mostly unidirectional carbon tube; 70-120 GPa typical). Sleeve clearance, bond and foam
    bearing compliance are not included, so the real joint is softer: the sweep goes well below this value."""
    if EI_wing is None:
        tab = pd.read_csv(SECTIONS)
        EI_wing = float(np.interp(s.Y_BREAK, tab.y, tab.EI))
    EI_t = E_tube * math.pi / 64 * (D**4 - d**4)
    return 1 / (engagement * (1 / EI_t - 1 / EI_wing))


def strip_short_period(st, body, V=20.0, dCma=0.0, dCmq=0.0):
    """Rigid-aircraft short-period root (1/s) of the strip model: p-k on the two rigid modes only."""
    mdl = Model(st, Config(bc="sym", body=body, dCma=dCma, dCmq=dCmq))
    w2, Phi = eigh(mdl.K, mdl.M)
    Phi = Phi[:, :2]
    Pm = mdl.P @ Phi
    Mq, Kq = Phi.T @ mdl.M @ Phi, np.zeros((2, 2))

    def aero(om, V_):
        Q = strip_q(om, V_, RHO0, mdl.bg, mdl.ag, mdl.etag)
        return np.einsum("g,gim,gij,gjn->mn", mdl.wg, Pm, Q, Pm) + Phi.T @ mdl.pitch_fix(om, V_, RHO0) @ Phi

    roots = pk_roots(Mq, Kq, aero, V, float(st.b[0]), 2 * math.pi * 10)
    osc = [l for l, _ in roots if l.imag > 1e-3]
    return max(osc, key=lambda z: z.imag) if osc else complex(math.nan, math.nan)


def fit_pitch_correction(st, body, sp):
    """dCma, dCmq that make the strip model's rigid short period match AVL's (natural frequency, damping)."""
    from scipy.optimize import least_squares

    def r(x):
        l = strip_short_period(st, body, sp["V_tas"], *x)
        return [abs(l) / (2 * math.pi * sp["avl_wn_hz"]) - 1, -l.real / abs(l) - sp["avl_zeta"]]
    return least_squares(r, [0.2, 1.3], diff_step=0.02).x


def short_period_check(st, body, tot, V=20.0, dCma=0.0, dCmq=0.0):
    """Rigid-aircraft short period from the strip model (p-k on the two rigid modes only) vs AVL's eigenvalue
    for the same mass distribution."""
    import stability as stb
    l = strip_short_period(st, body, V, dCma, dCmq)
    items = []
    yc = (st.yn[1:] + st.yn[:-1]) / 2
    dy = np.diff(st.yn)
    x_ea = st.x_ea0 + yc * math.tan(st.sweep)
    for side in (-1, 1):
        for y, d, m_, Sa, Ia, xe in zip(yc, dy, st.m, st.S_a, st.I_a, x_ea):
            Iown = (Ia - Sa**2 / m_) * d
            items.append(av.MassItem("wing", m_ * d, xe + Sa / m_, side * y, 0.0, (0.0, Iown, Iown)))
    for name, mm, x, z in body_items(True):
        items.append(av.MassItem(name, mm, x, 0.0, z))
    r = av.run_trim(s.GEO, items, V, RHO0, modes=True)
    m = stb.classify_modes(r.eig) if r.ok else {}
    avl_wn, avl_z = m.get("short_period_wn", math.nan), m.get("short_period_zeta", math.nan)
    wn_strip = abs(l)
    mi = av.cg_and_inertia(items)
    return dict(V_tas=V, strip_wn_hz=wn_strip / 2 / math.pi, strip_zeta=-l.real / wn_strip,
                avl_wn_hz=avl_wn / 2 / math.pi, avl_zeta=avl_z, avl_Xnp=r.st.get("Xnp", math.nan),
                avl_Cma=r.st.get("Cma", math.nan), avl_Cmq=r.st.get("Cmq", math.nan),
                avl_CLa=r.st.get("CLa", math.nan), model_x_cg=tot["x_cg"], model_Iyy=tot["Iyy"],
                avl_items_Iyy=mi["Iyy"], model_mass=tot["mass_kg"])


def freeplay_table(el_df):
    """Free play as a dead band of total width 2d on the servo spring. Inside the band k_eff = 0; for an
    oscillation of amplitude A > d the describing-function stiffness is k * deadband_ratio(A/d), so a surface
    with free play passes through every stiffness from 0 up to k. Flutter-type limit-cycle oscillation (LCO)
    is possible at a speed V if the linear flutter speed at some stiffness below k is under V.
    k_crit: highest tested stiffness whose linear flutter speed is below the screen.
    LCO amplitude at the screen speed ~ d * amplitude_for_ratio(k_crit / k)."""
    rows = []
    base = el_df.loc[el_df["where"] == "none"]
    for (bc, alt), d in base.groupby(["bc", "altitude_m"]):
        for screen_eas in (SCREEN_EAS, SCREEN_115):
            d = d.sort_values("k_hinge")
            rho = RHO0 if alt == 0 else RHO_HI
            screen = tas(screen_eas, rho)
            kk, vf = d.k_hinge.to_numpy(), d.flutter_tas_mps.to_numpy()
            unstable = kk[vf < screen]
            k_crit = float(unstable.max()) if len(unstable) else 0.0
            stable_above = kk[(kk > k_crit) & (vf >= screen)]
            k_min_ok = float(stable_above.min()) if len(stable_above) else math.inf
            for k_nom in (5.0, 10.0, 20.0, 50.0, 100.0, K_HINGE_V1, 300.0):
                if k_crit <= 0:
                    amp = 0.0
                elif k_crit >= k_nom:
                    amp = math.inf
                else:
                    amp = amplitude_for_ratio(k_crit / k_nom)
                rows.append(dict(bc=bc, altitude_m=alt, screen_eas_mps=screen_eas, screen_tas_mps=screen,
                                 vf_free_floating_tas=float(vf[kk == 0][0]) if (kk == 0).any() else math.nan,
                                 vf_min_over_k_tas=float(vf.min()), k_crit_Nm_per_rad=k_crit,
                                 k_first_tested_ok=k_min_ok, k_nominal=k_nom,
                                 lco_amplitude_over_half_freeplay=amp))
    return pd.DataFrame(rows)


# ============================================================================ parallel jobs
_STRIP_CACHE = {}


def _strips(design, spec):
    key = (design, spec)
    if key not in _STRIP_CACHE:
        use_design(design)
        kw = dict(spec)
        if "balance" in kw and kw["balance"] is not None:
            mb, arm, where = kw["balance"]
            kw["balance"] = dict(mass_kg=mb, arm_m=arm, where=where)
        _STRIP_CACHE[key] = lodestar_strips(**kw)
    use_design(design)
    return _STRIP_CACHE[key]


def run_job(job):
    """One flutter case. job: name, design, spec (strip options as a tuple of items), bc, kit_on,
    cfg (Config keyword dict without body), rho, n_modes, keep (return roots)."""
    import os
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    st = _strips(job["design"], job["spec"])
    body, tot = aircraft_body(st, kit_on=job.get("kit_on", True))
    cfg = Config(bc=job["bc"], body=body, **job.get("cfg", {}))
    speeds = job.get("speeds", SPEEDS)
    mdl, wn, br, V, f, lab = run_case(st, cfg, job["rho"], speeds=speeds, n_modes=job.get("n_modes", 16))
    rho = job["rho"]
    out = {k: v for k, v in job.items() if k not in ("spec", "cfg", "speeds", "keep")}
    out.update(job.get("cfg", {}))
    out.update(altitude_m=0 if rho == RHO0 else 4400, flutter_tas_mps=V,
               flutter_eas_mps=eas(V, rho) if math.isfinite(V) else math.inf, freq_hz=f, mode=lab,
               margin_vs_36eas=eas(V, rho) / SCREEN_EAS if math.isfinite(V) else math.inf,
               static_instability_tas=real_root_instability(br), mass_kg=tot["mass_kg"], Iyy=tot["Iyy"])
    out.pop("rho", None)
    if job.get("keep"):
        out["roots"] = br.drop(columns="vec")
        out["modes_hz"] = list(wn / 2 / math.pi)
        out["mode_names"] = mdl.mode_labels(mdl.modal(job.get("n_modes", 16))[1], wn)
    return out


def run_jobs(jobs, workers=None):
    import os
    from concurrent.futures import ProcessPoolExecutor
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(run_job, jobs, chunksize=1))


# ============================================================================ main study
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    log = []

    def say(*a):
        txt = " ".join(str(x) for x in a)
        print(txt, flush=True)
        log.append(txt)

    import wing_trade as wt
    ya, cla, stab = avl_lift_slope()
    pd.DataFrame(dict(y=ya, cl_alpha=cla)).to_csv(OUT / "avl_lift_slope.csv", index=False)
    say(f"Lodestar flutter_v2 run, design {NOMINAL} (comparison: compact_eps20_light)")
    say(f"AVL wing: CLa {stab.get('CLa', math.nan):.3f}/rad; section cl_a root {cla[0]:.2f}, "
        f"y=0.75 m {np.interp(0.75, ya, cla):.2f}, tip {cla[-1]:.2f} /rad (2 pi cos L = "
        f"{2 * math.pi * math.cos(math.radians(24.4)):.2f})")
    tab = pd.read_csv(SECTIONS)
    st = _strips(NOMINAL, ())
    body, tot = aircraft_body(st, True)
    _, tot_off = aircraft_body(st, False)
    k_joint = joint_stiffness()
    say(f"Elastic-axis sweep {math.degrees(st.sweep):.2f} deg, hinge sweep {math.degrees(st.sweep_h):.2f} deg")
    say(f"Model aircraft (P50B packs, kit on): {tot['mass_kg']:.3f} kg, CG x {tot['x_cg']:.4f} m, "
        f"Iyy {tot['Iyy']:.3f} kg m2; kit off {tot_off['mass_kg']:.3f} kg, Iyy {tot_off['Iyy']:.3f} kg m2")
    say(f"Joint bending stiffness estimate (tube only) {k_joint:.0f} N m/rad")
    yc, dy = (st.yn[1:] + st.yn[:-1]) / 2, np.diff(st.yn)
    m_el = float(sum(elevon_section(s.GEO.chord(y))["m"] * d for y, d, f in zip(yc, dy, st.el) if f))
    S_el, I_el = float(np.sum(st.S_b * dy)), float(np.sum(st.I_b * dy))
    ep = elevon_section(s.GEO.chord(1.0))
    say(f"Elevon per side: {1000 * m_el:.1f} g, static moment {1000 * S_el:.2f} g.m aft of hinge (CG "
        f"{1000 * S_el / m_el:.1f} mm aft), I_hinge {1e6 * I_el:.0f} kg.mm2, GJ_e(y=1 m) {ep['GJ']:.3f} N m2")

    # ---------------- rigid short period vs AVL, and the pitch correction that matches it
    sp = short_period_check(st, body, tot)
    dCma, dCmq = fit_pitch_correction(st, body, sp)
    sp2 = short_period_check(st, body, tot, dCma=dCma, dCmq=dCmq)
    sp_tab = pd.DataFrame([dict(sp, correction="none", dCma=0.0, dCmq=0.0),
                           dict(sp2, correction="matched to AVL", dCma=dCma, dCmq=dCmq)])
    sp_tab.to_csv(OUT / "short_period_check.csv", index=False)
    say("\n[short period, 20 m/s sea level]")
    say(sp_tab[["correction", "dCma", "dCmq", "strip_wn_hz", "strip_zeta", "avl_wn_hz", "avl_zeta", "avl_Xnp",
                "model_x_cg", "model_Iyy", "avl_items_Iyy"]].round(4).to_string(index=False))
    cal = dict(dCma=float(dCma), dCmq=float(dCmq))

    # ---------------- V-g references (structures.flutter, serial)
    vg = {}
    for design in ("v1", "compact_eps20_light"):
        use_design(design)
        beam = s.wing_beam(pd.read_csv(SECTIONS), extra_masses=tuple(wt.COMPACT.masses()))
        for rho in (RHO0, RHO_HI):
            vg[(design, rho)] = s.flutter(*beam, rho)[2]
    use_design(NOMINAL)

    # ---------------- job list
    J = []
    both = (RHO0, RHO_HI)
    straight1 = (("sweep", False), ("eta", 1.0))
    for design in ("v1", "compact_eps20_light"):
        for rho in both:
            J.append(dict(group="repro", name="clamped straight, 2 pi, (repro of V-g)", design=design,
                          spec=straight1, bc="clamped", rho=rho))
    ladder = [
        ("clamped straight, AVL lift slope", (("sweep", False),), "clamped", True, {}),
        ("clamped swept, eta = cos L", (("eta", "cos"),), "clamped", True, {}),
        ("clamped swept, AVL lift slope", (), "clamped", True, {}),
        ("clamped swept, AVL, streamwise GJ", (("gj_normal", False),), "clamped", True, {}),
        ("free-free symmetric, kit on (nominal)", (), "sym", True, {}),
        ("free-free symmetric, kit released", (), "sym", False, {}),
        ("free-free antisymmetric, kit on", (), "anti", True, {}),
        ("free-free symmetric, joint tube stiffness", (), "sym", True, dict(joint_kb=k_joint)),
    ]
    for name, spec, bc, kit, cfg in ladder:
        for rho in both:
            keep = name.startswith("free-free symmetric, kit on") or name.startswith("free-free antisym")
            J.append(dict(group="ladder", name=name, design=NOMINAL, spec=spec, bc=bc, kit_on=kit, cfg=cfg,
                          rho=rho, keep=keep))
    for name, spec, bc in (("clamped swept, AVL lift slope", (), "clamped"),
                           ("free-free symmetric, kit on (nominal)", (), "sym"),
                           ("free-free antisymmetric, kit on", (), "anti")):
        for rho in both:
            J.append(dict(group="eps20", name=name, design="compact_eps20_light", spec=spec, bc=bc, rho=rho))
    for nm in (10, 24):
        J.append(dict(group="convergence", name=f"free-free symmetric, {nm} modes", design=NOMINAL, spec=(),
                      bc="sym", rho=RHO0, n_modes=nm))
    J.append(dict(group="convergence", name="free-free symmetric, 60 elements", design=NOMINAL,
                  spec=(("ne", 60),), bc="sym", rho=RHO0))
    for kb in (None, 20000.0, k_joint, 1000.0, 500.0, 250.0, 100.0):
        for bc in ("sym", "anti"):
            for rho in both:
                J.append(dict(group="joint", name="joint bending sweep", design=NOMINAL, spec=(), bc=bc, rho=rho,
                              cfg=dict(joint_kb=kb)))
    for kt in (2000.0, 500.0):
        for bc in ("sym", "anti"):
            J.append(dict(group="joint", name="joint torsion sweep", design=NOMINAL, spec=(), bc=bc, rho=RHO0,
                          cfg=dict(joint_kb=k_joint, joint_kt=kt)))
    for kh in (0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, K_HINGE_V1, 300.0, 1000.0):
        for bc in ("sym", "anti"):
            for rho in both:
                J.append(dict(group="elevon", name="elevon hinge stiffness sweep", design=NOMINAL, spec=(), bc=bc,
                              rho=rho, cfg=dict(elevon=True, k_hinge=kh), where="none", balance_g=0.0, arm_mm=0.0))
    for where, arm in (("uniform", 0.015), ("tip", 0.040), ("servo", 0.040)):
        for mb in (0.005, 0.010, 0.020, 0.040, 0.080):
            for kh in (0.0, K_HINGE_V1):
                for bc in ("sym", "anti"):
                    J.append(dict(group="elevon", name="elevon balance sweep", design=NOMINAL,
                                  spec=(("balance", (mb, arm, where)),), bc=bc, rho=RHO0,
                                  cfg=dict(elevon=True, k_hinge=kh), where=where, balance_g=1000 * mb,
                                  arm_mm=1000 * arm))
    y60 = s.Y_BREAK + 0.6 * (Y_ELEVON[1] - Y_ELEVON[0])
    for kh in (2.0, 5.0, 20.0, K_HINGE_V1):
        for bc in ("sym", "anti"):
            J.append(dict(group="servo", name=f"servo at y = {y60:.2f} m", design=NOMINAL, spec=(), bc=bc, rho=RHO0,
                          cfg=dict(elevon=True, k_hinge=kh, y_servo=y60)))
    for rho in both:
        for kit in (True, False):
            J.append(dict(group="calibrated", name=f"free-free sym, AVL-matched pitch, kit {'on' if kit else 'released'}",
                          design=NOMINAL, spec=(), bc="sym", kit_on=kit, rho=rho, cfg=dict(cal)))
        for kb in (k_joint, 1000.0, 500.0, 250.0):
            J.append(dict(group="calibrated", name="AVL-matched pitch, joint bending sweep", design=NOMINAL, spec=(),
                          bc="sym", rho=rho, cfg=dict(cal, joint_kb=kb)))
        for kh in (0.0, K_HINGE_V1):
            J.append(dict(group="calibrated", name="AVL-matched pitch, elevon", design=NOMINAL, spec=(), bc="sym",
                          rho=rho, cfg=dict(cal, elevon=True, k_hinge=kh)))
    say(f"Running {len(J)} p-k cases in parallel ...")
    res = run_jobs(J)
    roots = {(r["name"], r["bc"], r["altitude_m"]): r for r in res if "roots" in r}
    table = pd.DataFrame([{k: v for k, v in r.items() if k not in ("roots", "modes_hz", "mode_names")}
                          for r in res])
    table["vg_reference_tas"] = [vg.get((r["design"], RHO0 if r["altitude_m"] == 0 else RHO_HI), math.nan)
                                 if r["group"] == "repro" else math.nan for r in res]
    table.to_csv(OUT / "all_cases.csv", index=False)
    cols = ["design", "name", "bc", "altitude_m", "flutter_tas_mps", "flutter_eas_mps", "freq_hz", "mode",
            "margin_vs_36eas", "static_instability_tas"]
    for grp in ("repro", "ladder", "calibrated", "eps20", "convergence", "joint", "servo"):
        d = table.loc[table.group == grp]
        extra = [c for c in ("joint_kb", "joint_kt", "k_hinge", "elevon", "vg_reference_tas") if c in d and d[c].notna().any()]
        say(f"\n[{grp}]")
        say(d[cols + extra].round(3).to_string(index=False))
    summary = table.loc[table.group.isin(["repro", "ladder", "calibrated", "eps20", "convergence"])]
    summary[cols + ["vg_reference_tas", "mass_kg", "Iyy"]].to_csv(OUT / "summary.csv", index=False)
    table.loc[table.group == "joint"].to_csv(OUT / "joint_sweep.csv", index=False)
    el_df = table.loc[table.group.isin(["elevon", "servo"])].copy()
    el_df.to_csv(OUT / "elevon_sweep.csv", index=False)
    say("\n[elevon]")
    say(el_df[["name", "bc", "altitude_m", "k_hinge", "where", "balance_g", "arm_mm", "flutter_tas_mps",
               "flutter_eas_mps", "freq_hz", "mode"]].round(3).to_string(index=False))
    fp = freeplay_table(el_df.loc[el_df.group == "elevon"])
    fp.to_csv(OUT / "freeplay.csv", index=False)
    say("\n[free play]")
    say(fp.round(3).to_string(index=False))
    for key, r in roots.items():
        tag = f"{'sym' if key[1] == 'sym' else 'anti'}_{key[2]}m"
        r["roots"].to_csv(OUT / f"pk_roots_{tag}.csv", index=False)
        if key[2] == 0:
            pd.DataFrame(dict(mode=range(len(r["modes_hz"])), freq_hz=r["modes_hz"], name=r["mode_names"])).to_csv(
                OUT / f"modes_free_free_{key[1]}.csv", index=False)
            say(f"In-vacuo modes, {key[1]}: " + ", ".join(r["mode_names"][:8]))
    mclamp = Model(st, Config(bc="clamped"))
    w2c, Phic = eigh(mclamp.K, mclamp.M)
    wc = np.sqrt(w2c[:10])
    pd.DataFrame(dict(mode=range(10), freq_hz=wc / 2 / math.pi, name=mclamp.mode_labels(Phic[:, :10], wc))).to_csv(
        OUT / "modes_clamped.csv", index=False)
    say("In-vacuo clamped modes: " + ", ".join(mclamp.mode_labels(Phic[:, :8], wc[:8])))
    fea = ROOT / "out" / "fea" / "modes.csv"
    say(f"FE modes file {'found' if fea.exists() else 'not found'}: {fea}")


    # ---------------- static aeroelasticity
    say("\n[static aeroelasticity, root-clamped half wing, elevon locked or prescribed]")
    srows = []
    for name, spec in (("swept (model)", ()), ("swept, eta = cos L", (("eta", "cos"),)),
                       ("straight beam (no sweep coupling)", (("sweep", False),))):
        st_ = _strips(NOMINAL, spec)
        qd, qn = divergence_q(st_, Config(bc="clamped"))
        vd = math.sqrt(2 * qd / RHO0) if math.isfinite(qd) else math.inf
        qr, qs, vals = reversal_q(st_)
        vr = math.sqrt(2 * qr / RHO0) if math.isfinite(qr) else math.inf
        e_vd = roll_effectiveness(st_, 0.5 * RHO0 * s.VD**2)
        e_36 = roll_effectiveness(st_, 0.5 * RHO0 * SCREEN_EAS**2)
        srows.append(dict(model=name, divergence_q_pa=qd, divergence_eas_mps=vd,
                          divergence_tas_4400m_mps=tas(vd, RHO_HI) if math.isfinite(vd) else math.inf,
                          negative_q_root_pa=qn, reversal_q_pa=qr, reversal_eas_mps=vr,
                          reversal_tas_4400m_mps=tas(vr, RHO_HI) if math.isfinite(vr) else math.inf,
                          roll_effectiveness_at_vd=e_vd, roll_effectiveness_at_36eas=e_36))
        say(f"  {name:34s} divergence {vd:7.1f} m/s EAS; reversal {vr:6.1f} m/s EAS; "
            f"roll effectiveness {e_vd:.3f} at Vd, {e_36:.3f} at 36 m/s EAS")
        if name == "swept (model)":
            pd.DataFrame(dict(q_pa=qs, eas_mps=np.sqrt(2 * np.asarray(qs) / RHO0), roll_effectiveness=vals)
                         ).to_csv(OUT / "roll_effectiveness.csv", index=False)
    pd.DataFrame(srows).to_csv(OUT / "static_aeroelastic.csv", index=False)
    figures(roots, el_df, table)
    (OUT / "flutter_v2.log").write_text("\n".join(log) + "\n", encoding="utf-8")


def figures(roots, el_df, table):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    for col, alt in enumerate((0, 4400)):
        key = ("free-free symmetric, kit on (nominal)", "sym", alt)
        if key not in roots:
            continue
        br = roots[key]["roots"]
        br = br.loc[(br.im > 0) & (br.omega_hz < 40)]
        for b_, d in br.groupby("branch"):
            if len(d) < 3:
                continue
            d = d.sort_values("V")
            ax[0, col].plot(d.V, d.g.clip(-1.5, 0.5), lw=1.0)
            ax[1, col].plot(d.V, d.omega_hz, lw=1.0)
        rho = RHO0 if alt == 0 else RHO_HI
        for r in (0, 1):
            ax[r, col].axvline(tas(SCREEN_EAS, rho), color="k", ls="--", lw=0.8)
        ax[0, col].axhline(0, color="k", lw=0.6)
        ax[0, col].set_ylim(-1.5, 0.4)
        ax[0, col].set_title(f"Figure 1{'ab'[col]}. Damping g vs. true airspeed, by root, {alt} m", fontsize=10)
        ax[1, col].set_title(f"Figure 1{'cd'[col]}. Frequency vs. true airspeed, by root, {alt} m", fontsize=10)
        ax[1, col].set_xlabel("True airspeed, m/s")
        ax[0, col].set_ylabel("g = 2 Re(p) / Im(p), -")
        ax[1, col].set_ylabel("Frequency, Hz")
    fig.tight_layout()
    fig.savefig(OUT / "fig1_vg_vf_free_free_sym.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
    base = el_df.loc[(el_df.group == "elevon") & (el_df["where"] == "none")]
    for (bc, alt), d in base.groupby(["bc", "altitude_m"]):
        d = d.sort_values("k_hinge")
        ax[0].plot(d.k_hinge.clip(lower=0.2), d.flutter_eas_mps.clip(upper=150), marker="o",
                   label=f"{'symmetric' if bc == 'sym' else 'antisymmetric'}, {alt} m")
    ax[0].axhline(SCREEN_EAS, color="k", ls="--", lw=0.8)
    ax[0].set_xscale("log")
    ax[0].set_xlabel("Servo + linkage stiffness at the hinge, N m/rad (0 drawn at 0.2)")
    ax[0].set_ylabel("Flutter speed, m/s EAS (capped at 150)")
    ax[0].set_title("Figure 2a. Flutter speed vs. hinge stiffness,\nby symmetry and altitude", fontsize=10)
    ax[0].legend(fontsize=8)
    bal = el_df.loc[(el_df.group == "elevon") & (el_df["where"] != "none") & (el_df.k_hinge == 0.0)]
    for (where, bc), d in bal.groupby(["where", "bc"]):
        d = d.sort_values("balance_g")
        ax[1].plot(d.balance_g, d.flutter_eas_mps.clip(upper=150), marker="o",
                   label=f"{where}, {d.arm_mm.iloc[0]:.0f} mm, {bc}")
    ax[1].axhline(SCREEN_EAS, color="k", ls="--", lw=0.8)
    ax[1].set_xlabel("Balance mass per elevon, g")
    ax[1].set_ylabel("Flutter speed, m/s EAS (capped at 150)")
    ax[1].set_title("Figure 2b. Free-floating elevon flutter speed vs.\nbalance mass, by location, sea level",
                    fontsize=10)
    ax[1].legend(fontsize=7)
    jt = table.loc[(table.group == "joint") & (table.name == "joint bending sweep")].copy()
    jt["kb"] = jt.joint_kb.fillna(1e5)
    for (bc, alt), d in jt.groupby(["bc", "altitude_m"]):
        d = d.sort_values("kb")
        ax[2].plot(d.kb, d.flutter_eas_mps.clip(upper=150), marker="o", label=f"{bc}, {alt} m")
    ax[2].axhline(SCREEN_EAS, color="k", ls="--", lw=0.8)
    ax[2].set_xscale("log")
    ax[2].set_xlabel("Joint bending stiffness, N m/rad (rigid drawn at 1e5)")
    ax[2].set_ylabel("Flutter speed, m/s EAS (capped at 150)")
    ax[2].set_title("Figure 2c. Flutter speed vs. joint stiffness,\nby symmetry and altitude", fontsize=10)
    ax[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "fig2_elevon_and_joint.png", dpi=130)
    plt.close(fig)


def fe_stiffness_cases():
    """Sensitivity to the shell/solid FE natural frequencies (out/fea/modes.csv), which put clamped first torsion
    at 46.6 Hz and first bending at 12.3 Hz against this beam's 57.3 and 12.6 Hz. GJ and EI are scaled by the
    squared frequency ratios (same mass), and the key cases rerun."""
    fe = pd.read_csv(ROOT / "out" / "fea" / "modes.csv")
    fe = fe.loc[fe.config.str.startswith("P50B") & fe.source.str.startswith("FE")]
    f_b = float(fe.loc[fe.name == "bending 1", "freq_Hz"].iloc[0])
    f_t = float(fe.loc[fe.name == "torsion 1", "freq_Hz"].iloc[0])
    st = _strips(NOMINAL, ())
    mdl = Model(st, Config(bc="clamped"))
    w = np.sqrt(eigh(mdl.K, mdl.M, eigvals_only=True)[:4]) / 2 / math.pi
    names = mdl.mode_labels(eigh(mdl.K, mdl.M)[1][:, :4], 2 * math.pi * w)
    beam_b = w[[i for i, n in enumerate(names) if n.startswith("bending")][0]]
    beam_t = w[[i for i, n in enumerate(names) if n.startswith("torsion")][0]]
    gj_s, ei_s = round((f_t / beam_t) ** 2, 4), round((f_b / beam_b) ** 2, 4)
    spec = (("ei_scale", ei_s), ("gj_scale", gj_s))
    st2 = _strips(NOMINAL, spec)
    body, tot = aircraft_body(st, True)
    sp = short_period_check(st, body, tot)
    cal = dict(zip(("dCma", "dCmq"), map(float, fit_pitch_correction(st, body, sp))))
    m2 = Model(st2, Config(bc="clamped"))
    w2 = np.sqrt(eigh(m2.K, m2.M, eigvals_only=True)[:4]) / 2 / math.pi
    J = []
    for rho in (RHO0, RHO_HI):
        J += [dict(group="fe", name="clamped swept, FE-matched EI/GJ", design=NOMINAL, spec=spec, bc="clamped", rho=rho),
              dict(group="fe", name="free-free sym, FE-matched, strip pitch", design=NOMINAL, spec=spec, bc="sym",
                   rho=rho),
              dict(group="fe", name="free-free sym, FE-matched, AVL-matched pitch", design=NOMINAL, spec=spec,
                   bc="sym", rho=rho, cfg=dict(cal)),
              dict(group="fe", name="free-free sym, FE-matched, AVL pitch, joint 4170", design=NOMINAL, spec=spec,
                   bc="sym", rho=rho, cfg=dict(cal, joint_kb=joint_stiffness()))]
        for bc in ("sym", "anti"):
            for kh in (0.0, 2.0, 5.0, 10.0, 20.0, K_HINGE_V1):
                J.append(dict(group="fe", name="FE-matched, elevon", design=NOMINAL, spec=spec, bc=bc, rho=rho,
                              cfg=dict(elevon=True, k_hinge=kh)))
        for kh in (0.0, K_HINGE_V1):
            J.append(dict(group="fe", name="FE-matched, elevon, AVL-matched pitch", design=NOMINAL, spec=spec,
                          bc="sym", rho=rho, cfg=dict(cal, elevon=True, k_hinge=kh)))
        for kh in (7.0, 14.0):
            for bc in ("sym", "anti"):
                J.append(dict(group="fe", name="FE-matched, elevon", design=NOMINAL, spec=spec, bc=bc, rho=rho,
                              cfg=dict(elevon=True, k_hinge=kh)))
        for where, arm, mb in (("servo", 0.040, 0.020), ("servo", 0.040, 0.040), ("servo", 0.040, 0.060),
                               ("servo", 0.040, 0.080), ("uniform", 0.015, 0.040)):
            for bc in ("sym", "anti"):
                J.append(dict(group="fe", name=f"FE-matched, free elevon, balance {1000 * mb:.0f} g {where}",
                              design=NOMINAL, spec=spec + (("balance", (mb, arm, where)),), bc=bc, rho=rho,
                              cfg=dict(elevon=True, k_hinge=0.0)))
        bal60 = spec + (("balance", (0.060, 0.040, "servo")),)
        J.append(dict(group="fe", name="FE-matched, AVL pitch, free elevon, 60 g servo balance", design=NOMINAL,
                      spec=bal60, bc="sym", rho=rho, cfg=dict(cal, elevon=True, k_hinge=0.0)))
        J.append(dict(group="fe", name="FE-matched, AVL pitch, elevon 136, joint 4170, 60 g servo balance",
                      design=NOMINAL, spec=bal60, bc="sym", rho=rho,
                      cfg=dict(cal, elevon=True, k_hinge=K_HINGE_V1, joint_kb=joint_stiffness())))
        for kb in (joint_stiffness(), 2000.0, 1000.0):
            J.append(dict(group="fe", name="FE-matched, AVL pitch, elevon 136, joint sweep", design=NOMINAL, spec=spec,
                          bc="sym", rho=rho, cfg=dict(cal, elevon=True, k_hinge=K_HINGE_V1, joint_kb=kb)))
        J.append(dict(group="fe", name="FE-matched, antisymmetric, elevon 136, joint 4170", design=NOMINAL, spec=spec,
                      bc="anti", rho=rho, cfg=dict(elevon=True, k_hinge=K_HINGE_V1, joint_kb=joint_stiffness())))
    res = run_jobs(J)
    df = pd.DataFrame(res)
    df["ei_scale"], df["gj_scale"] = ei_s, gj_s
    df.to_csv(OUT / "fe_stiffness_sensitivity.csv", index=False)
    print(f"FE bending {f_b:.2f} Hz / torsion {f_t:.2f} Hz vs beam {beam_b:.2f} / {beam_t:.2f} Hz -> EI x{ei_s}, "
          f"GJ x{gj_s}; scaled beam clamped modes {np.round(w2, 2)} Hz")
    cols = ["name", "bc", "altitude_m", "k_hinge", "joint_kb", "flutter_tas_mps", "flutter_eas_mps", "freq_hz", "mode",
            "static_instability_tas"]
    print(df[[c for c in cols if c in df]].round(2).to_string(index=False))
    return df


if __name__ == "__main__":
    if sys.argv[1:] == ["--fe"]:
        OUT.mkdir(parents=True, exist_ok=True)
        fe_stiffness_cases()
    else:
        main()
        if (ROOT / "out" / "fea" / "modes.csv").exists():
            fe_stiffness_cases()
