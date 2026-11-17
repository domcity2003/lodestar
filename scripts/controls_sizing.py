"""Flight-control physical sizing for the Lodestar v1 flying wing (no autopilot software).

Questions (Step 1, 2026-11-14):
  1. CG envelope: aft limit (5% MAC static margin), forward limit (elevon authority in the flare and at VD),
     where the four mass states sit with the battery packs moved 25 mm forward / 25 mm inboard, kit-release shift.
  2. Elevon deflections for trim (12.3-30 m/s, sea level and 4,400 m), pull-up to n = 3.8 at Vc, and roll;
     hinge moments (AVL and Glauert thin-airfoil) -> servo torque, rate and size class for the servo bay.
  3. Roll performance (steady pb/2V, time to 45 and 60 deg bank) vs MIL-F-8785C, full and 2/3 roll share.
  4. Lateral-directional derivatives and modes (AVL), winglet adequacy, crosswind capability.
  5. Gust response: 1-cos discrete 7.62 m/s vertical and lateral, Dryden moderate turbulence, kit on / off.
  6. Power margin for a 30 deg banked climb at search speed at 4,400 m.
  7. Numbers for sensor placement: pitot static-pressure error ahead of the pod, prop inflow, magnetic
     field from power leads at the compass, IMU lever arm.

Mass model: wing_trade.mass_properties logic (EPS 20 light wing sections, compact joint) on the CAD layout.
If out/cad_v4/mass_cg.csv exists (FreeCAD rebuild with the moved packs) it is used directly; otherwise
out/cad_v3/mass_cg.csv with the packs shifted by battery_trim's (-25 mm, -25 mm).

Run (from scripts/):  ../.venv/Scripts/python.exe -X utf8 controls_sizing.py
Writes: out/controls/*.csv, *.png, run_log.txt
"""
from dataclasses import replace
from functools import lru_cache
from pathlib import Path
import math
import subprocess
import sys
import tempfile

import numpy as np
import pandas as pd
from scipy.integrate import quad, solve_ivp
from scipy.optimize import brentq

sys.path.insert(0, str(Path(__file__).parent))
import structures as s  # noqa: E402
import wing_trade as wt  # noqa: E402
import avl_run as av  # noqa: E402
import stability as stb  # noqa: E402
from design_v2 import N_WING, pack  # noqa: E402

ROOT = s.ROOT
OUT = ROOT / "out" / "controls"
G0 = 9.81
GEO = s.GEO                                   # pod body, 0.25 m winglets, 3 deg washout
MAC, B, SREF = GEO.mac, GEO.span, GEO.S
CBAR = SREF / B                               # mean geometric chord (gust formula)
X_MAC_LE = GEO.x_mac_le
RHO_SL, RHO_2K, RHO_HI = 1.225, stb.RHO, s.RHO_HI
CDP = stb.CD_NONINDUCED                       # profile + body drag for AVL mode damping

# ----------------------------------------------------------------------------- design choices (ASSUMPTIONS)
CL_MAX = 0.80                # sizing 3D CLmax (milestone 2: VLM first-strip limit ~1.0-1.17)
SM_MIN = 0.05                # aft limit: >= 5% MAC static margin in every state
TRAVEL = 20.0                # deg, mechanical travel each elevon (6-DOF model used +/-20)
ROLL_MIN = 5.0               # deg, minimum roll reserve kept on top of pitch demand
BATT_DX, BATT_DY = -0.025, -0.025   # battery_trim.py move (forward, inboard)
VC, VD = s.VC, s.VD          # 22, 30 m/s EAS
N_POS, N_NEG = s.N_MAN_POS, s.N_MAN_NEG
UDE = 7.62                   # m/s, CS-VLA / CS-23 design gust at VD; used here as the discrete gust at search speed
V_SEARCH = (18.0, 20.0)      # m/s TAS: P50B and SiCore search speeds (performance_eps20_light.csv)
SERVO_FREEPLAY_DEG = 0.3     # typical new metal-gear wing servo + clevis, ASSUMPTION
SERVO_HOLD_DEG = 1.5         # output deflection at which a digital servo reaches stall torque, ASSUMPTION
SERVO_MARGIN = 2.0           # stall torque / max hinge moment

# Elevon geometry: 25% chord, y = 0.45 m to tip, per side
Y_E0, Y_E1, EF = GEO.y_break, B / 2, 1 - GEO.hinge_xc


def _cf(y):
    return EF * GEO.chord(y)


ELEVON_AREA = quad(_cf, Y_E0, Y_E1)[0]                    # m^2 per side
ELEVON_CF2 = quad(lambda y: _cf(y) ** 2, Y_E0, Y_E1)[0]   # m^3, hinge moment = Ch q * this


def effective(delta):
    """Effective (linear-theory) deflection for a geometric deflection, deg. Plain-flap effectiveness K'
    stays 1 to 10 deg and falls linearly to 0.70 at 25 deg (shape of DATCOM's plain-flap correction, ASSUMPTION)."""
    d = abs(delta)
    if d <= 10:
        e = d
    else:
        k = lambda x: 1 - 0.3 * min(x - 10, 15) / 15  # noqa: E731
        e = 10 + quad(k, 10, d)[0]
    return math.copysign(e, delta)


def geometric(delta_eff):
    """Inverse of effective(): geometric deflection needed for a linear-theory (AVL) deflection."""
    if abs(delta_eff) <= 10:
        return delta_eff
    g = brentq(lambda d: effective(d) - abs(delta_eff), 10, 80)
    return math.copysign(g, delta_eff)


# ----------------------------------------------------------------------------- mass states
def cad_table():
    v4 = ROOT / "out" / "cad_v4" / "mass_cg.csv"
    if v4.exists():
        return pd.read_csv(v4), "out/cad_v4/mass_cg.csv (FreeCAD rebuild, packs already moved)", 0.0
    return pd.read_csv(ROOT / "out" / "cad_v3" / "mass_cg.csv"), \
        "out/cad_v3/mass_cg.csv with packs moved -25 mm x / -25 mm y (battery_trim.py); cad_v4 not yet written", BATT_DX


def aircraft_items(cad, loaded, cell, dx=BATT_DX, dy=BATT_DY):
    """Same item list as wing_trade.mass_properties (which returns only the totals), with the packs moved."""
    tab = pd.read_csv(wt.OUT / "v1_sections.csv")
    joint_masses = tuple(wt.COMPACT.masses())
    batt = 2.4205 if cell == "P50B" else pack("Amprius SiCore 6.3 Ah", N_WING)[1]
    if dx == 0.0:      # CAD v4 table: packs already at the frozen layout
        import lodestar_v1 as v1
        yb = (v1.LAYOUT["y_batt"] - 0.071, v1.LAYOUT["y_batt"] + 0.071)
    else:
        yb = (s.Y_BATT[0] + dy, s.Y_BATT[1] + dy)
    items = []
    for row in cad.itertuples():
        if row.item.startswith("wing structure") or (not loaded and row.item in ("rescue kit", "kit parachute")):
            continue
        if row.item.startswith("batteries"):
            for side in (-1, 1):
                for y in np.linspace(*yb, 5):
                    items.append(av.MassItem(row.item, batt / 10, row.x + dx, side * y, row.z))
        elif row.item.startswith("elevon mass balance"):
            import lodestar_v1 as v1
            for side in (-1, 1):
                items.append(av.MassItem(row.item, row.mass_kg / 2, row.x, side * v1.ADDED_MASS["elevon_balance_y"], row.z))
        elif row.item == "winglets":
            for side in (-1, 1):
                items.append(av.MassItem("winglet", row.mass_kg / 2, row.x, side * B / 2, row.z))
        else:
            items.append(av.MassItem(row.item, row.mass_kg, row.x, 0, row.z))
    ys = np.linspace(0, B / 2, 151)
    mid = (ys[:-1] + ys[1:]) / 2
    masses = np.interp(mid, tab.y, tab.m_per_m) * np.diff(ys)
    xs = np.array([GEO.x_le(y) for y in mid]) + np.interp(mid, tab.y, tab.xcg_c) * np.array([GEO.chord(y) for y in mid])
    for side in (-1, 1):
        for x, y, m in zip(xs, mid, masses):
            items.append(av.MassItem("wing section", m, x, side * y))
        for x, y, m in joint_masses:
            items.append(av.MassItem("joint", m, x, side * y))
        items.append(av.MassItem("servo", s.M_SERVO, GEO.x_le(s.Y_SERVO) + 0.6 * GEO.chord(s.Y_SERVO), side * s.Y_SERVO))
    return items


def mass_states(cad, dx):
    out = {}
    for cell in ("SiCore", "P50B"):
        for kit in (True, False):
            items = aircraft_items(cad, kit, cell, dx=dx)
            out[(cell, kit)] = (items, av.cg_and_inertia(items))
    return out


def pct_mac(x):
    return 100 * (x - X_MAC_LE) / MAC


# ----------------------------------------------------------------------------- AVL
def avl(items, V, rho, *, CL=None, x_cg=None, elevon=None, aileron=None, qnd=0.0, modes=False, geo=GEO):
    """One AVL run. CL None = level flight (c1); elevon None = trim Cm = 0; aileron None = no roll
    (roll rate 0, aileron trims roll moment), else fixed aileron with free steady roll rate (roll moment 0).
    qnd: steady pitch rate qc/2V (pull-up). Returns (st dict, per-side hinge moments N m, eigenvalues)."""
    mi = av.cg_and_inertia(items)
    x = mi["x"] if x_cg is None else x_cg
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        av.write_avl(geo, x, d / "rw.avl", cdp=CDP)
        av.write_mass(items, d / "rw.mass", rho)
        keys = ["mass rw.mass", "mset 0", "plop", "g", "", "oper", "c1", f"v {V}", f"m {mi['m']}", f"d {rho}",
                "g 9.81", f"x {x}", f"z {mi['z']}", ""]
        if CL is not None:
            keys.append(f"a c {CL:.6f}")
        keys.append("d1 pm 0" if elevon is None else f"d1 d1 {elevon}")
        keys += ["d2 rm 0", "r r 0"] if aileron is None else [f"d2 d2 {aileron}", "r rm 0"]
        keys += [f"p p {qnd:.6f}", "b b 0", "x", "st", "st.txt", "hm", "hm.txt", ""]
        if modes:
            keys += ["mode", "n", "w", "eig.txt", ""]
        keys += ["quit", ""]
        subprocess.run([str(av.AVL_EXE), "rw.avl"], cwd=d, input="\n".join(keys) + "\n", text=True,
                       capture_output=True, timeout=60)
        if not (d / "st.txt").exists():
            raise RuntimeError(f"AVL failed at V={V}, CL={CL}")
        st = av.parse_eq((d / "st.txt").read_text())
        hm = {}
        for line in (d / "hm.txt").read_text().splitlines():
            p = line.split()
            if len(p) == 4 and p[0] in ("elevon", "aileron"):
                hm[p[0]] = float(p[3])
        # elevon gain +1 both sides, aileron SgnDup -1: H_e = H_R + H_L, H_a = H_R - H_L
        hinge = ((hm["elevon"] + hm["aileron"]) / 2, (hm["elevon"] - hm["aileron"]) / 2)
        eig = []
        if modes and (d / "eig.txt").exists():
            for line in (d / "eig.txt").read_text().splitlines():
                p = line.split()
                if len(p) == 3 and not line.strip().startswith("#"):
                    try:
                        eig.append(complex(float(p[1]), float(p[2])))
                    except ValueError:
                        pass
    return st, hinge, eig


def cl_of(m, V, rho, n=1.0):
    return n * m * G0 / (0.5 * rho * V**2 * SREF)


def pull_qnd(n, V):
    """Non-dimensional pitch rate qc/2V of a steady wings-level pull-up at load factor n."""
    return (n - 1) * G0 / V * MAC / (2 * V)


def tas(v_eas, rho):
    return v_eas * math.sqrt(RHO_SL / rho)


# ----------------------------------------------------------------------------- thin-airfoil hinge moments
def flap_loading(theta, alpha, delta, theta_h):
    """Glauert thin-airfoil loading dCp(theta) for incidence alpha and flap deflection delta (rad),
    x/c = (1 - cos theta)/2, hinge at theta_h. Closed form of the flap Fourier series."""
    a0 = alpha + delta * (math.pi - theta_h) / math.pi
    series = delta / math.pi * math.log(abs(math.sin((theta + theta_h) / 2) / math.sin((theta - theta_h) / 2)))
    return 4 * (a0 * (1 + math.cos(theta)) / math.sin(theta) + series)


def thin_airfoil_flap(E=0.25):
    """Section coefficients per rad for a plain flap of chord ratio E: cl_alpha, cl_delta, cm_delta (c/4),
    and hinge-moment derivatives ch_alpha, ch_delta (flap-chord based, positive = trailing edge down)."""
    th = math.acos(2 * E - 1)
    xh = 1 - E

    def coeffs(alpha, delta):
        f = lambda t: flap_loading(t, alpha, delta, th) * math.sin(t) / 2  # noqa: E731  dCp dx/dtheta
        x = lambda t: (1 - math.cos(t)) / 2  # noqa: E731
        cl = quad(f, 1e-9, math.pi - 1e-9, points=[th], limit=400)[0]
        cm = quad(lambda t: -f(t) * (x(t) - 0.25), 1e-9, math.pi - 1e-9, points=[th], limit=400)[0]
        ch = quad(lambda t: -f(t) * (x(t) - xh), th, math.pi - 1e-9, limit=400)[0] / E**2
        return cl, cm, ch
    cla, _, cha = coeffs(1.0, 0.0)
    cld, cmd, chd = coeffs(0.0, 1.0)
    return dict(cl_alpha=cla, cl_delta=cld, cm_delta=cmd, ch_alpha=cha, ch_delta=chd)


# ----------------------------------------------------------------------------- roll and gust dynamics
def time_to_bank(phi_deg, pss_per_deg, tau, V, delta_cmd, rate_dps, t_max=10.0):
    """Time (s) to reach a bank angle from wings level, aileron stepping at a servo rate limit, roll subsidence
    with time constant tau, steady roll pb/2V = pss_per_deg * effective(delta)."""
    def rhs(t, y):
        d = min(rate_dps * t, delta_cmd) if rate_dps else delta_cmd
        p_ss = pss_per_deg * effective(d) * 2 * V / B
        return [(p_ss - y[0]) / tau, y[0]]
    ev = lambda t, y: y[1] - math.radians(phi_deg)  # noqa: E731
    ev.terminal = True
    sol = solve_ivp(rhs, (0, t_max), [0.0, 0.0], events=ev, max_step=0.005)
    return float(sol.t_events[0][0]) if len(sol.t_events[0]) else float("inf")


def dryden_w_psd(omega, sigma, L, V):
    """MIL-F-8785C Dryden vertical (also lateral) gust PSD, one-sided: integral over 0..inf = sigma^2."""
    x = L * omega / V
    return sigma**2 * L / (math.pi * V) * (1 + 3 * x**2) / (1 + x**2) ** 2


def dryden_low_alt(h_m, w20=15.43):
    """MIL-F-8785C low-altitude model: sigma_w = 0.1 W20, sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4,
    L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2 (h in ft). Moderate: W20 = 30 kt."""
    h_ft = h_m / 0.3048
    k = 0.177 + 0.000823 * h_ft
    sw = 0.1 * w20
    return dict(sigma_w=sw, sigma_v=sw / k**0.4, L_w=h_m, L_v=h_ft / k**1.2 * 0.3048)


def plunge_one_minus_cos(m, V, rho, cla, ude=UDE, length=25 * CBAR):
    """Peak incremental load factor and effective incidence for a 1-cos vertical gust, plunge only
    (pitch attitude held), quasi-steady lift. Returns (dn_peak, dalpha_peak rad)."""
    k = 0.5 * rho * V * SREF * cla / m      # 1/s
    def rhs(t, y):
        sx = V * t
        u = ude / 2 * (1 - math.cos(2 * math.pi * sx / length)) if sx <= length else 0.0
        return [k * (u - y[0])]
    t = np.linspace(0, 2 * length / V, 2001)
    sol = solve_ivp(rhs, (0, t[-1]), [0.0], t_eval=t, max_step=0.002)
    u = np.where(V * t <= length, ude / 2 * (1 - np.cos(2 * math.pi * V * t / length)), 0.0)
    da = (u - sol.y[0]) / V
    return float(np.max(k * (u - sol.y[0]) / G0)), float(np.max(da))


# ----------------------------------------------------------------------------- pitot and magnetics
def pod_axis_cp(x_p, V=1.0):
    """Static-pressure coefficient on the pod axis at x_p (upstream of the nose), slender-body sources
    sigma = V dA/dx on the pod's equivalent-radius distribution (structures.geometry)."""
    xr = np.array(GEO.body_xr)
    xs = np.linspace(0, xr[-1, 0], 801)
    r = np.interp(xs, xr[:, 0], xr[:, 1])
    a = math.pi * r**2
    dadx = np.gradient(a, xs)
    xs_g = xs + GEO.body_x0
    u = np.sum(V * dadx * (xs[1] - xs[0]) / (4 * math.pi * (xs_g - x_p) ** 2) * np.sign(xs_g - x_p) * -1)
    # a source upstream-of-point pushes flow +x; a downstream source slows the flow ahead of it (u < 0)
    return float(-2 * u / V - (u / V) ** 2)


def actuator_inflow(x_up, thrust, V, rho, R):
    """Axial induced velocity on the axis a distance x_up upstream of an actuator disk."""
    A = math.pi * R**2
    vi = 0.5 * (-V + math.sqrt(V**2 + 2 * thrust / (rho * A)))
    return vi * (1 - x_up / math.sqrt(x_up**2 + R**2))


MU0 = 4e-7 * math.pi


def wire_field_uT(I, r, pair_sep=None):
    """Field of a long straight wire (pair_sep None) or a go-and-return pair separated by pair_sep, microtesla."""
    if pair_sep is None:
        return 1e6 * MU0 * I / (2 * math.pi * r)
    return 1e6 * MU0 * I * pair_sep / (2 * math.pi * r**2)


# ============================================================================= study
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    sys.stdout = stb.Tee(OUT / "run_log.txt")
    print("Lodestar v1 flight-control sizing, 2026-11-14")
    cad, source, dx = cad_table()
    print(f"Mass source: {source}")
    states = mass_states(cad, dx)
    ta = thin_airfoil_flap(EF)
    print(f"Elevon per side: area {ELEVON_AREA:.4f} m2, integral cf^2 dy {ELEVON_CF2:.5f} m3; "
          f"thin airfoil cf/c 0.25: cl_d {ta['cl_delta']:.3f}, ch_a {ta['ch_alpha']:.3f}, ch_d {ta['ch_delta']:.3f} /rad\n")

    # --------------------------------------------------------- 1. neutral point, mass states, CG limits
    rows = []
    for (cell, kit), (items, mi) in states.items():
        for alt, rho, V in (("SL", RHO_SL, VC), ("2000 m", RHO_2K, V_SEARCH[1]), ("2000 m", RHO_2K, 15.0)):
            st, _, _ = avl(items, V, rho)
            rows.append(dict(cell=cell, kit=kit, mass=mi["m"], x_cg=mi["x"], cg_pct_mac=pct_mac(mi["x"]), alt=alt, V=V,
                             CL=st["CLtot"], x_np=st["Xnp"], np_pct_mac=pct_mac(st["Xnp"]),
                             sm_pct=100 * (st["Xnp"] - mi["x"]) / MAC, elevon=st["elevon"], alpha=st["Alpha"],
                             Ixx=mi["Ixx"], Iyy=mi["Iyy"], Izz=mi["Izz"]))
    ms = pd.DataFrame(rows)
    ms.round(5).to_csv(OUT / "mass_states.csv", index=False)
    x_np_min = ms.x_np.min()
    x_aft = x_np_min - SM_MIN * MAC
    print("1. Mass states and static margin (AVL with pod and winglets)")
    print(ms[["cell", "kit", "mass", "x_cg", "cg_pct_mac", "alt", "V", "CL", "np_pct_mac", "sm_pct", "elevon"]].round(3).to_string(index=False))
    print(f"   neutral point range {ms.x_np.min():.4f}-{ms.x_np.max():.4f} m; aft CG limit (5% MAC) x = {x_aft:.4f} m "
          f"({pct_mac(x_aft):.1f}% MAC)")

    # reference mass state for aero-only sweeps (aero trim depends on CL and CG only)
    ref_items, ref_mi = states[("SiCore", True)]
    off_items, off_mi = states[("SiCore", False)]
    kit_rows = []
    for cell in ("SiCore", "P50B"):
        on, off = states[(cell, True)][1], states[(cell, False)][1]
        a, _, _ = avl(states[(cell, True)][0], 15.0, RHO_2K)
        b_, _, _ = avl(states[(cell, False)][0], 15.0, RHO_2K)
        kit_rows.append(dict(cell=cell, m_on=on["m"], m_off=off["m"], dx_cg_mm=1000 * (off["x"] - on["x"]),
                             dz_cg_mm=1000 * (off["z"] - on["z"]), weight_jump=on["m"] / off["m"] - 1,
                             elevon_on=a["elevon"], elevon_off=b_["elevon"], d_elevon=b_["elevon"] - a["elevon"]))
    kr = pd.DataFrame(kit_rows)
    kr.round(4).to_csv(OUT / "kit_release.csv", index=False)
    print("\n   Kit release at 15 m/s, 2,000 m (same speed, controls re-trimmed):")
    print(kr.round(4).to_string(index=False))

    # --------------------------------------------------------- 2a. roll derivatives -> roll reserve
    st_r, _, eig_r = avl(ref_items, 15.0, RHO_2K, modes=True)
    cld = abs(st_r["Cld02"])            # per deg aileron (one surface up, other down)
    clp = st_r["Clp"]
    pss_per_deg = cld / -clp            # steady pb/2V per effective degree
    # MIL-F-8785C Class I, Category C (approach) Level 1: 30 deg in 1.3 s; tau_roll from AVL
    modes_ref = stb.classify_modes(eig_r)
    tau = modes_ref.get("roll_tau", -ref_mi["Ixx"] / (clp * 0.5 * RHO_2K * 15**2 * SREF * B**2 / (2 * 15)))
    v_app = 1.2 * math.sqrt(2 * off_mi["m"] * G0 / (RHO_2K * SREF * CL_MAX))
    need_c = brentq(lambda d: time_to_bank(30, pss_per_deg, tau, v_app, d, 0) - 1.3, 0.2, TRAVEL)
    roll_reserve = max(ROLL_MIN, need_c)
    pitch_budget = TRAVEL - roll_reserve
    print(f"\n2. Roll: Cl_da {cld:.5f}/deg, Clp {clp:.3f}; steady pb/2V {pss_per_deg:.4f} per deg; roll tau {tau:.3f} s")
    print(f"   Class I Cat C approach (30 deg in 1.3 s) at {v_app:.1f} m/s needs {need_c:.2f} deg (no servo lag) "
          f"-> roll reserve {roll_reserve:.1f} deg, pitch budget {pitch_budget:.1f} deg geometric")

    # --------------------------------------------------------- 2b. forward CG limit
    def elevon_at(x_cg, V, rho, CL, n=1.0, items=ref_items):
        st, _, _ = avl(items, V, rho, CL=CL, x_cg=x_cg, qnd=pull_qnd(n, V))
        return st["elevon"]

    fwd_cases = []
    m_land = off_mi["m"]
    vs_land = math.sqrt(2 * m_land * G0 / (RHO_SL * SREF * CL_MAX))
    vs_load = math.sqrt(2 * ref_mi["m"] * G0 / (RHO_SL * SREF * CL_MAX))
    fwd_cases.append(("1 g at CLmax (stall approach)", vs_load, RHO_SL, CL_MAX, 1.0))
    fwd_cases.append(("flare: 1.2 Vs, pull to CLmax (n 1.44)", 1.2 * vs_land, RHO_SL, CL_MAX, 1.44))
    fwd_cases.append((f"pull-up n {N_POS} at VD", VD, RHO_SL, cl_of(ref_mi["m"], VD, RHO_SL, N_POS), N_POS))
    fwd_cases.append((f"pull-up n {N_POS} at Vc", VC, RHO_SL, cl_of(ref_mi["m"], VC, RHO_SL, N_POS), N_POS))
    x0, x1 = 0.38, 0.43
    fwd = []
    for name, V, rho, CL, n in fwd_cases:
        e0, e1 = elevon_at(x0, V, rho, CL, n), elevon_at(x1, V, rho, CL, n)
        slope = (e1 - e0) / (x1 - x0)
        lim_eff = -effective(pitch_budget)
        x_lim = x0 + (lim_eff - e0) / slope
        fwd.append(dict(case=name, V=V, CL=CL, n=n, elevon_at_0p38=e0, elevon_at_0p43=e1, deg_per_cm=slope / 100,
                        x_fwd_limit=x_lim, fwd_limit_pct_mac=pct_mac(x_lim)))
    fwd = pd.DataFrame(fwd)
    fwd.round(4).to_csv(OUT / "forward_limit.csv", index=False)
    x_fwd = fwd.x_fwd_limit.max()
    print(f"\n   Forward limit: CG where the up-elevon (AVL, converted to geometric) reaches the pitch budget "
          f"-{pitch_budget:.1f} deg")
    print(fwd.round(3).to_string(index=False))
    print(f"   -> CG envelope x = {x_fwd:.4f} to {x_aft:.4f} m ({pct_mac(x_fwd):.1f} to {pct_mac(x_aft):.1f}% MAC), "
          f"{1000 * (x_aft - x_fwd):.0f} mm wide")

    # --------------------------------------------------------- 2c. trim across the speed range
    trim = []
    for (cell, kit), (items, mi) in states.items():
        for alt, rho in (("SL", RHO_SL), ("4400 m", RHO_HI)):
            vs = math.sqrt(2 * mi["m"] * G0 / (rho * SREF * CL_MAX))
            for V in [12.3, 14, 16, 18, 20, 22, 25, 28, 30]:
                if V < vs - 1e-6:
                    trim.append(dict(cell=cell, kit=kit, alt=alt, V=V, CL=cl_of(mi["m"], V, rho), elevon=np.nan,
                                     note=f"below 1 g stall {vs:.1f} m/s"))
                    continue
                st, h, _ = avl(items, V, rho)
                trim.append(dict(cell=cell, kit=kit, alt=alt, V=V, CL=st["CLtot"], alpha=st["Alpha"], elevon=st["elevon"],
                                 hinge_Nm=max(abs(h[0]), abs(h[1])), note=""))
    tr = pd.DataFrame(trim)
    tr.round(4).to_csv(OUT / "trim_sweep.csv", index=False)
    print("\n   Trim elevon (deg, + = trailing edge down) vs TAS, 1 g:")
    piv = tr.pivot_table(index="V", columns=["alt", "cell", "kit"], values="elevon")
    print(piv.round(2).to_string())

    # pull-up n = 3.8 at Vc (EAS) at SL and 4,400 m, at both CG limits and the mass states
    pu = []
    for alt, rho in (("SL", RHO_SL), ("4400 m", RHO_HI)):
        V = tas(VC, rho)
        for label, x in (("fwd limit", x_fwd), ("aft limit", x_aft)) + tuple(
                (f"{c} kit {'on' if k else 'off'}", states[(c, k)][1]["x"]) for c, k in states):
            CL = cl_of(ref_mi["m"], V, rho, N_POS)
            st, h, _ = avl(ref_items, V, rho, CL=CL, x_cg=x, qnd=pull_qnd(N_POS, V))
            pu.append(dict(alt=alt, V_tas=V, cg=label, x_cg=x, CL=CL, alpha=st["Alpha"], elevon_avl=st["elevon"],
                           elevon_geometric=geometric(st["elevon"]), hinge_Nm=max(abs(h[0]), abs(h[1]))))
    pu = pd.DataFrame(pu)
    pu.round(4).to_csv(OUT / "pullup.csv", index=False)
    print(f"\n   Pull-up n = {N_POS} at Vc {VC} m/s EAS (SiCore kit-on mass):")
    print(pu.round(3).to_string(index=False))

    # --------------------------------------------------------- 2d. hinge moments and servo
    def h_thin(V_eas, alpha_rad, delta_deg):
        q = 0.5 * RHO_SL * V_eas**2
        return abs((ta["ch_alpha"] * alpha_rad + ta["ch_delta"] * math.radians(delta_deg)) * q * ELEVON_CF2)

    hcases = []
    m = ref_mi["m"]
    for name, V, n, d_one in (
            ("VA (=Vc) 1 g, one elevon at full travel", VC, 1.0, -TRAVEL),
            ("VA (=Vc) n 3.8, one elevon at full travel", VC, N_POS, -TRAVEL),
            ("VD n 3.8 pull + 1/3 roll reserve", VD, N_POS, None),
            ("VD n -1.5 push + 1/3 roll reserve", VD, N_NEG, None),
            ("VD 1 g, full travel (runaway bound)", VD, 1.0, -TRAVEL)):
        CL = cl_of(m, V, RHO_SL, n)
        if d_one is None:
            st0, _, _ = avl(ref_items, V, RHO_SL, CL=CL, x_cg=x_fwd if n > 0 else x_aft, qnd=pull_qnd(n, V))
            de = st0["elevon"]
            da = roll_reserve / 3
        else:
            st0, _, _ = avl(ref_items, V, RHO_SL, CL=CL, x_cg=ref_mi["x"], qnd=pull_qnd(n, V))
            de = st0["elevon"]
            da = max(abs(d_one) - abs(de), 0.0)       # the more-deflected surface sits on the travel stop
        st, h, _ = avl(ref_items, V, RHO_SL, CL=CL, x_cg=ref_mi["x"], elevon=de, aileron=da, qnd=pull_qnd(n, V))
        worst_delta = max(abs(de + da), abs(de - da))
        alpha_loc = 1.1 * CL / (2 * math.pi)       # outboard section incidence, ASSUMPTION (local cl ~ 1.1 CL)
        ht = h_thin(V, alpha_loc, -worst_delta)
        hcases.append(dict(case=name, V_eas=V, n=n, CL=CL, elevon=de, aileron=da, max_surface_deg=worst_delta,
                           pb2V=st.get("pb/2V"), hinge_avl_Nm=max(abs(h[0]), abs(h[1])), hinge_thin_Nm=ht,
                           design_Nm=max(abs(h[0]), abs(h[1]), ht)))
    hc = pd.DataFrame(hcases)
    hc.round(4).to_csv(OUT / "hinge_moments.csv", index=False)
    h_design = hc[~hc.case.str.contains("runaway")].design_Nm.max()
    t_stall = SERVO_MARGIN * h_design
    print(f"\n   Hinge moments per elevon (AVL vortex lattice vs Glauert thin airfoil, cf/c 0.25, SiCore loaded):")
    print(hc.round(3).to_string(index=False))
    print(f"   design hinge moment {h_design:.3f} N m -> servo stall torque >= {t_stall:.2f} N m "
          f"({t_stall * 10.197:.1f} kg cm) at {SERVO_MARGIN:.0f}x")
    y_sv, c_sv = s.Y_SERVO, GEO.chord(s.Y_SERVO)
    t_bay = [float(s.AF.local_thickness(xc)) * c_sv * 1000 for xc in (0.563, 0.60, 0.637)]
    print(f"   servo bay y = {y_sv:.2f} m, chord {c_sv * 1000:.0f} mm, airfoil thickness at 56/60/64% chord "
          f"{t_bay[0]:.1f}/{t_bay[1]:.1f}/{t_bay[2]:.1f} mm (minus 2 x 1.5 mm skin allowance)")
    for ratio in (1.0, 1.5, 2.0):
        ts = t_stall / ratio
        k_out = ts / math.radians(SERVO_HOLD_DEG)
        print(f"   linkage ratio {ratio:.1f} (horn arm / servo arm): servo stall torque >= {ts:.2f} N m ({ts * 10.197:.1f} kg cm), "
              f"servo throw +/-{TRAVEL * ratio:.0f} deg; stiffness at hinge ~{k_out * ratio**2:.0f} N m/rad "
              f"(servo {k_out:.0f} N m/rad at its output, stall torque at ~{SERVO_HOLD_DEG} deg); "
              f"free play at surface ~{SERVO_FREEPLAY_DEG / ratio:.2f} deg; surface rate = servo rate / {ratio:.1f}")
    for trav in (15.0,):
        print(f"   if mechanical travel is cut to +/-{trav:.0f} deg: design hinge moment scales to about "
              f"{h_design * trav / TRAVEL:.2f} N m (linear in deflection)")
    k_servo = (t_stall / 2) / math.radians(SERVO_HOLD_DEG) * 4

    # --------------------------------------------------------- 3. roll performance vs MIL-F-8785C
    roll = []
    for V in (15.0, 18.0, 20.0):
        st, _, eig = avl(ref_items, V, RHO_2K, modes=True)
        md = stb.classify_modes(eig)
        tau_v = md.get("roll_tau", tau)
        pss = abs(st["Cld02"]) / -st["Clp"]
        avail = TRAVEL - abs(geometric(st["elevon"]))
        for share_name, d in (("full", avail), ("2/3", 2 / 3 * avail), ("roll reserve", roll_reserve)):
            for rate in (600.0, 300.0):      # deg/s: 0.10 and 0.20 s/60 deg servos
                roll.append(dict(V=V, share=share_name, aileron=d, servo_dps=rate, tau=tau_v,
                                 pb2V=pss * effective(d), p_dps=math.degrees(pss * effective(d) * 2 * V / B),
                                 t45=time_to_bank(45, pss, tau_v, V, d, rate), t60=time_to_bank(60, pss, tau_v, V, d, rate),
                                 Cn_over_Cl_da=st["Cnd02"] / st["Cld02"]))
    rl = pd.DataFrame(roll)
    rl.round(4).to_csv(OUT / "roll_performance.csv", index=False)
    print("\n3. Roll performance at 2,000 m (SiCore kit on); MIL-F-8785C Class I Level 1: 60 deg in 1.3 s (Cat A), "
          "1.7 s (Cat B); Class II-L: 45 deg in 1.4 s (A), 1.9 s (B)")
    print(rl.round(3).to_string(index=False))

    # --------------------------------------------------------- 4. lateral-directional
    lat = []
    for (cell, kit), (items, mi) in states.items():
        if cell != "SiCore":
            continue
        for alt, rho, V in (("2000 m", RHO_2K, 15.0), ("2000 m", RHO_2K, 20.0), ("2000 m", RHO_2K, tas(VC, RHO_2K)),
                            ("SL", RHO_SL, VD)):
            st, _, eig = avl(items, V, rho, modes=True)
            md = stb.classify_modes(eig)
            lat.append(dict(kit=kit, alt=alt, V=V, Cnb=st["Cnb"], Clb=st["Clb"], Clp=st["Clp"], Cnr=st["Cnr"],
                            Cnp=st.get("Cnp"), Cld=st["Cld02"], Cnd=st["Cnd02"], **{k: v for k, v in md.items()},
                            lateral_ok=stb.lat_ok(md), pitch_ok=stb.lon_ok(md)))
    la = pd.DataFrame(lat)
    la.round(4).to_csv(OUT / "lateral_modes.csv", index=False)
    print("\n4. Lateral-directional (AVL, SiCore):")
    cols = ["kit", "alt", "V", "Cnb", "Clb", "Cnr", "dutch_roll_wn", "dutch_roll_zeta", "roll_tau", "spiral_root",
            "spiral_t2", "spiral_t_half", "short_period_zeta", "phugoid_zeta", "lateral_ok", "pitch_ok"]
    print(la[[c for c in cols if c in la]].round(3).to_string(index=False))
    wl = []
    for h_w in (0.20, 0.25, 0.30):
        g = replace(GEO, winglet_h=h_w)
        for V in (tas(VC, RHO_2K), VD):
            rho = RHO_2K if V < 29 else RHO_SL
            st, _, eig = avl(ref_items, V, rho, modes=True, geo=g)
            md = stb.classify_modes(eig)
            wl.append(dict(winglet_h=h_w, V=V, Cnb=st["Cnb"], dr_zeta=md.get("dutch_roll_zeta"),
                           dr_zw=md.get("dutch_roll_zeta", 0) * md.get("dutch_roll_wn", 0), spiral_t2=md.get("spiral_t2"),
                           ok=stb.lat_ok(md)))
    wl = pd.DataFrame(wl)
    wl.round(4).to_csv(OUT / "winglet_check.csv", index=False)
    print("   Winglet height check, SiCore kit on (fast cases set the size):")
    print(wl.round(3).to_string(index=False))

    # crosswind: sideslip that full roll reserve can balance; weathervane from Cn_beta
    xw = []
    for V, label in ((v_app, "landing approach 1.2 Vs kit off (2,000 m)"), (15.0, "launch / climb-out 15 m/s"),
                     (20.0, "search 20 m/s")):
        st, _, _ = avl(off_items if "kit off" in label else ref_items, V, RHO_2K)
        avail = TRAVEL - abs(geometric(st["elevon"]))
        beta_max = abs(st["Cld02"]) * effective(avail) / abs(st["Clb"])   # rad (Clb per rad, Cld per deg)
        beta_max = min(beta_max, math.radians(45))   # beyond 45 deg the linear derivatives mean nothing
        xw.append(dict(case=label, V=V, roll_avail_deg=avail, Clb=st["Clb"], Cnb=st["Cnb"],
                       beta_max_deg=math.degrees(beta_max), crosswind_at_beta_max=V * math.sin(beta_max),
                       beta_12mps_deg=math.degrees(math.asin(min(1, 12 / V))) if V > 12 else 90.0,
                       wind_offset_ok_deg_12mps=math.degrees(math.asin(min(1, V * math.sin(beta_max) / 12)))))
    xw = pd.DataFrame(xw)
    xw.round(3).to_csv(OUT / "crosswind.csv", index=False)
    print("\n   Crosswind: largest sideslip that full lateral travel can hold wings-level against Cl_beta")
    print(xw.round(2).to_string(index=False))

    # --------------------------------------------------------- 5. gusts
    gusts = []
    turb = dryden_low_alt(120.0)
    sig_p = None
    for V in (15.0,) + V_SEARCH:
        for kit in (True, False):
            items, mi = states[("SiCore", kit)]
            st, _, _ = avl(items, V, RHO_2K)
            m, cla, CL1 = mi["m"], st["CLa"], st["CLtot"]
            dn, da = plunge_one_minus_cos(m, V, RHO_2K, cla)
            n_cap = (CL_MAX * s.OVERSHOOT) / CL1
            de_hold = abs(st["Cma"] * da / st["Cmd01"])      # elevon to hold attitude, deg (Cma per rad, Cmd per deg)
            # lateral 1-cos: beta = v/V, no weathervane relief (conservative)
            beta = math.atan(UDE / V)
            da_lat = abs(st["Clb"] * beta / st["Cld02"])
            # Dryden moderate at 120 m AGL
            k = 0.5 * RHO_2K * V * SREF * cla / m
            Hn = lambda w: (0.5 * RHO_2K * V**2 * SREF * cla / (m * G0 * V)) ** 2 * (w / k) ** 2 / (1 + (w / k) ** 2)  # noqa: E731
            Ha = lambda w: (1 / V) ** 2 * (w / k) ** 2 / (1 + (w / k) ** 2)  # noqa: E731
            sn = math.sqrt(quad(lambda w: Hn(w) * dryden_w_psd(w, turb["sigma_w"], turb["L_w"], V), 0, np.inf, limit=400)[0])
            sa = math.sqrt(quad(lambda w: Ha(w) * dryden_w_psd(w, turb["sigma_w"], turb["L_w"], V), 0, np.inf, limit=400)[0])
            sb = turb["sigma_v"] / V
            # rolling gust: one-sided spatial PSD (MIL-F-8785C) integrated -> sigma_p = sigma_w sqrt(0.987 (pi Lw/4b)^(1/3) / (Lw b))
            sig_p = turb["sigma_w"] * math.sqrt(0.8 * (math.pi * turb["L_w"] / (4 * B)) ** (1 / 3) * math.pi**2 / 8 / (turb["L_w"] * B))
            da_p = abs(st["Clp"] * sig_p * B / (2 * V) / st["Cld02"])
            avail = TRAVEL - abs(geometric(st["elevon"]))
            gusts.append(dict(V=V, kit=kit, mass=m, CL1=CL1, CLa=cla, n_1cos_unstalled=1 + dn, n_stall_cap=n_cap,
                              n_1cos=min(1 + dn, n_cap), stalls_1cos=1 + dn > n_cap, dalpha_1cos_deg=math.degrees(da),
                              elevon_hold_1cos_deg=de_hold, beta_1cos_deg=math.degrees(beta), aileron_1cos_deg=da_lat,
                              roll_avail_deg=avail, aileron_sat_1cos=geometric(da_lat) > avail,
                              sigma_n_dryden=sn, n_3sigma=1 + 3 * sn, dCL_3sigma=3 * sa * cla,
                              stall_margin_dCL=CL_MAX - CL1, sigma_beta_deg=math.degrees(sb),
                              aileron_3sigma_beta_deg=abs(st["Clb"] * 3 * sb / st["Cld02"]),
                              elevon_3sigma_hold_deg=abs(st["Cma"] * 3 * sa / st["Cmd01"]),
                              sigma_p_dps=math.degrees(sig_p), aileron_3sigma_p_deg=3 * da_p))
    gu = pd.DataFrame(gusts)
    gu.round(4).to_csv(OUT / "gusts.csv", index=False)
    print(f"\n5. Gusts at 2,000 m, SiCore. Discrete 1-cos {UDE} m/s over 25 chords; Dryden moderate (W20 30 kt) at 120 m AGL: "
          f"sigma_w {turb['sigma_w']:.2f}, sigma_v {turb['sigma_v']:.2f} m/s, L_w {turb['L_w']:.0f} m, L_v {turb['L_v']:.0f} m")
    print(gu.round(3).T.to_string())

    # --------------------------------------------------------- 6. climb in a 30 deg bank at 4,400 m
    import v3_propulsion as vp
    import aerosandbox as asb
    atm = asb.Atmosphere(altitude=4400.0)
    vp.RHO, vp.MU = float(atm.density()), float(atm.dynamic_viscosity())
    props = vp.load_props()
    pick = pd.read_csv(ROOT / "out" / "v3" / "propulsion_pick.csv", index_col=0)["value"]
    prop, motor = props[pick["prop"]], pick["motor"]
    u_max = 0.95 * vp.N_S * vp.V_CELL_SAG
    p_max = vp.MOTORS[motor][4]

    def max_thrust(V):
        D = prop["d_in"] * vp.IN
        best = (0.0, None)
        for rpm in np.arange(1500, 9001, 20):
            n = rpm / 60
            ct, cp = vp.coeffs(prop, rpm, V / (n * D))
            if np.isnan(ct):
                continue
            T = ct * vp.RHO * n**2 * D**4
            P = cp * vp.RHO * n**3 * D**5 / vp.INSTALL
            mp = vp.motor_point(motor, rpm, P / (rpm * 2 * math.pi / 60))
            if mp["U"] <= u_max and mp["P_elec"] <= p_max and T > best[0]:
                best = (T, dict(rpm=rpm, P_batt=mp["P_elec"] / vp.ESC_ETA, I=mp["I"]))
        return best

    climb = []
    for cell, V in (("SiCore", V_SEARCH[1]), ("P50B", V_SEARCH[0])):
        mi = states[(cell, True)][1]
        n = 1 / math.cos(math.radians(30))
        vs_b = math.sqrt(2 * mi["m"] * n * G0 / (vp.RHO * SREF * CL_MAX))
        D = vp.drag(V, mi["m"] * n)
        T, info = max_thrust(V)
        req = vp.system(prop, motor, V, D)
        climb.append(dict(cell=cell, V=V, mass=mi["m"], vs_30deg=vs_b, CL=cl_of(mi["m"], V, vp.RHO, n), drag_N=D,
                          P_batt_level_turn=req["P_batt"] if req else np.nan, T_avail_N=T,
                          P_batt_full=info["P_batt"] if info else np.nan, I_full_A=info["I"] if info else np.nan,
                          climb_rate_30deg=(T - D) * V / (mi["m"] * G0)))
    cl_df = pd.DataFrame(climb)
    cl_df.round(3).to_csv(OUT / "climb_in_bank.csv", index=False)
    print("\n6. 30 deg banked climb at 4,400 m (CAM 15x10 + AT3520-550, sagging 6S):")
    print(cl_df.round(2).to_string(index=False))

    # --------------------------------------------------------- 7. avionics placement numbers
    print("\n7. Placement numbers")
    for x_tip in (-0.28, -0.33, -0.40):
        print(f"   pod-axis static error at x = {x_tip:.2f} m ({(GEO.body_x0 - x_tip) * 1000:.0f} mm ahead of the nose): "
              f"Cp {pod_axis_cp(x_tip):+.4f} -> airspeed error {-50 * pod_axis_cp(x_tip):+.1f}%")
    vi_s = actuator_inflow(0.98, vp.drag(V_SEARCH[1], ref_mi["m"]), V_SEARCH[1], RHO_2K, 7.5 * 0.0254)
    T_full, _ = max_thrust(15.0)
    vi_f = actuator_inflow(0.98, T_full, 15.0, vp.RHO, 7.5 * 0.0254)
    print(f"   prop inflow at the pitot (0.98 m upstream): {vi_s:.3f} m/s at search, {vi_f:.3f} m/s at full thrust 15 m/s")
    I_full = cl_df.I_full_A.max()
    I_cruise = (95.9 + 25) / (6 * 3.6)
    gps = np.array([0.12, 0.0, -0.035 + 0.075 + 0.065])
    for name, p in (("ESC", (0.565, 0, -0.035)), ("motor", (0.69, 0, -0.035)), ("pack (near edge)", (0.427, 0.33 - 0.07, 0.008)),
                    ("power bus at wing root", (0.43, 0, -0.03))):
        r = float(np.linalg.norm(gps - np.array(p)))
        print(f"   compass to {name}: {r * 1000:.0f} mm; single lead {wire_field_uT(I_full, r):.1f} uT at {I_full:.0f} A "
              f"({wire_field_uT(I_cruise, r):.2f} uT cruise {I_cruise:.1f} A); twisted pair 6 mm {wire_field_uT(I_full, r, 0.006):.2f} uT")
    fc_x = 0.25
    print(f"   IMU (FC centre x = {fc_x} m) lever arm to CG: {1000 * (ref_mi['x'] - fc_x):.0f} mm forward "
          f"(INS_POS1_X = {ref_mi['x'] - fc_x:+.3f} m)")
    rpm_s = float(pick["rpm_search"])
    print(f"   prop excitation at search: shaft {rpm_s / 60:.0f} Hz, blade pass {2 * rpm_s / 60:.0f} Hz")
    print(f"   alpha at search 20 m/s 2,000 m: {ms[(ms.V == 20.0) & (ms.cell == 'SiCore') & ms.kit].alpha.iloc[0]:.1f} deg; "
          f"approach CLmax: see trim_sweep.csv")

    # --------------------------------------------------------- figure: CG envelope
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.axvspan(pct_mac(x_fwd), pct_mac(x_aft), color="#2a78d6", alpha=0.12, label="usable CG range")
    ax.axvline(pct_mac(x_np_min), color="#1baf7a", ls="--", lw=1.5, label="neutral point (min)")
    for i, ((cell, kit), (_, mi)) in enumerate(states.items()):
        ax.plot(pct_mac(mi["x"]), mi["m"], "o", ms=8, color="#eb6834" if kit else "#6b6a64")
        ax.annotate(f"{cell} kit {'on' if kit else 'off'}", (pct_mac(mi["x"]), mi["m"]), xytext=(6, -4 + 8 * (i % 2)),
                    textcoords="offset points", fontsize=8)
    ax.set_xlabel("CG position (% MAC from MAC leading edge)")
    ax.set_ylabel("Mass (kg)")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(OUT / "cg_envelope.png", dpi=150)

    pd.Series(dict(x_fwd=x_fwd, x_aft=x_aft, x_np_min=x_np_min, pitch_budget=pitch_budget, roll_reserve=roll_reserve,
                   h_design_Nm=h_design, servo_stall_Nm=t_stall, k_hinge_at_ratio2_Nm_rad=k_servo, tau_roll=tau,
                   pss_per_deg=pss_per_deg, mass_source=source)).to_csv(OUT / "summary.csv", header=["value"])
    print(f"\nWritten to {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
