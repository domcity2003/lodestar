"""Preliminary Lodestar wing loads, section sizing, stiffness and clamped-wing flutter.

Until now the wing structure was one number: 1.0 kg per m2 of wing (sizing.py). This script designs
the actual structure and checks it.

Loads (limit loads; ultimate = 1.5 x limit, the standard factor of safety):
  - Manoeuvre: +3.8 / -1.5 g at VD, project screening cases inspired by CS-VLA.
  - Gusts: Pratt-Walker formula with CS-VLA gust velocities (15.24 m/s at the design cruise
    speed Vc, 7.62 m/s at the design dive speed VD). A light wing (56 N/m2) feels gusts hard, so
    gusts set the design. Lift can't exceed what the wing can make before it stalls: the gust load
    factor is capped where the first wing section reaches its maximum lift (NeuralFoil), plus 10%
    for dynamic overshoot. Both the capped and uncapped values are reported.
  - Speeds: Vc = 22 m/s EAS (top level speed), VD = 30 m/s EAS. The flutter screen is 1.2 VD = 36 m/s EAS
    (CS-23.629 margin), at sea level and at 4,400 m.
  - Spanwise lift: AVL strip loads, trimmed at the case load factor; masses on the wing (structure,
    battery packs at the wing roots, winglets, servos) push back down (inertia relief).
  - Torsion: strip pitching moments and force offsets from the estimated elastic axis.
    Fixed aileron cases are reported separately. They omit roll acceleration and damping.

Structure (per side; materials are typical datasheet values, marked ASSUMPTION):
  - Spar on the quarter-chord line (swept 25 deg, where the CAD joiner already sits): pultruded carbon
    strip caps on a Rohacell 51 foam web block, wrapped in one ply of +-45 glass. Caps sized at each
    station for the ultimate bending moment, limited by compression strength or face wrinkling on the
    web foam, whichever is lower.
  - Foam core (XPS) with +-45 skins; a rear web at 72% chord closes the torsion box ahead of the elevon.
  - Joiner at the centre/outer break (y = 0.45 m): the CAD's 16 mm carbon tube is checked for the
    bending moment there.

Flutter: beam finite elements (bending + torsion) with Theodorsen unsteady strip aerodynamics, V-g
(k) method, wing clamped at the centreline. Checked first on the sea-level Goland strip-aero case.
The Lodestar beam approximates the swept wing as a straight projected-span cantilever. Free-flight
pitch coupling, elevon flutter, divergence and control reversal are outside this model.

Run:  uv run python scripts/structures.py
Writes: out/structures/ (log and CSVs)
"""
from contextlib import redirect_stdout
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import math

import numpy as np
import pandas as pd
from scipy.linalg import eig, eigh
from scipy.optimize import linear_sum_assignment
from scipy.special import hankel2

import aerosandbox as asb  # noqa: E402
import stability as st  # noqa: E402
import avl_run as av  # noqa: E402
import vsp_model as vm  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "structures"
G0 = 9.81
RHO0 = 1.225
RHO_HI = float(asb.Atmosphere(altitude=4400).density())

# ----------------------------------------------------------------------------- design inputs
M_DESIGN = 7.40                      # kg, heaviest flying configuration (v3/v4 with kit), rounded up
VC, VD = 22.0, 30.0                  # m/s
N_MAN_POS, N_MAN_NEG = 3.8, -1.5
UDE_C, UDE_D = 15.24, 7.62           # m/s, CS-23 / CS-VLA design gusts
FOS = 1.5
OVERSHOOT = 1.10                     # dynamic lift overshoot over the static section maximum, ASSUMPTION
CL_MIN_WING = -0.55                  # negative stall of the reflexed MH45 wing, ASSUMPTION

Y_BREAK = 0.45
SWEEP = math.radians(25.0)
X_SPAR = 0.25                        # spar on the quarter-chord line (as the CAD joiner)
X_REAR = 0.72                        # rear web, just ahead of the 75% hinge
AF = asb.Airfoil("mh45")
M_BATT_SIDE = 2.4205 / 2             # P50B wing-root packs (CAD); SiCore packs are lighter
Y_BATT = (0.284, 0.426)              # pack span (CAD: 0.355 +- 0.071)
X_BATT_GLOBAL = 0.452
M_WINGLET = 0.053
M_SERVO = 0.030
Y_SERVO = Y_BREAK + 0.4 * (1.5 - Y_BREAK)


@dataclass
class Mat:
    name: str
    rho: float          # kg/m3
    E: float = 0.0      # Pa
    G: float = 0.0
    s_t: float = 0.0    # design allowable, Pa (already knocked down; FOS applied on the load side)
    s_c: float = 0.0
    tau: float = 0.0


CARBON_UD = Mat("pultruded carbon strip", 1550, E=125e9, s_t=1000e6, s_c=700e6)       # ASSUMPTION: ~0.6x typical datasheet strength
GLASS45 = Mat("+-45 glass, 80 g/m2 twill, wet layup", 1900, E=11e9, G=9e9, tau=60e6)  # cured ply 0.08 mm
CARBON45 = Mat("+-45 carbon, 93 g/m2, wet layup", 1450, E=18e9, G=20e9, tau=80e6)     # cured ply 0.11 mm
XPS = Mat("XPS foam core", 33, E=12e6, G=6e6)                                         # Styrodur-type
ROHA51 = Mat("Rohacell 51 IG-F", 52, E=43e6, G=19e6, tau=0.8e6)                        # Evonik typical compression/shear data
EPS20 = Mat("20 kg/m3 EPS (EN 13163 EPS 100 class)", 20, E=4.0e6, G=2.5e6, tau=0.10e6)  # ASSUMPTION: low end of EPS 100 data
# Design coefficient on Hoff's sandwich wrinkling formula 0.5 (Ef Ec Gc)^(1/3); 0.3 is a common knocked-down value
# for imperfect hand-laminated skins (ASSUMPTION, to be replaced by a flatwise-compression coupon test).
WRINKLE_K = 0.3
# FE correlation on the skin principal stress (fea_wing.py, out/fea/envelope_fe.csv): the shell model runs up to
# 1.10x the beam value over y = 0.16-0.27 m, where the wing packs load the box; elsewhere it is at or below 1.0.
SKIN_FE_FACTOR_Y, SKIN_FE_FACTOR = 0.30, 1.10
# Cap ply drops: the FE (fea_wing, v1.1) shows the skin stress at the first station past a cap thickness step
# runs 1.08-1.20x the beam sizing value (load shed from the cap into the skin). With CAP_DROP_FACTOR > 1 the
# first thinner station must pass with the skin stress x this factor, otherwise the drop moves outboard.
CAP_DROP_FACTOR = 1.0          # 1.0 = off (archived designs); wing_trade sets lodestar_v1 value for v1
# Optional upper-skin doubler (one extra ply of the skin layup) over a span range (y0, y1); None = no doubler.
UPPER_DOUBLER_Y = None
UPPER_DOUBLER_TAPER = 0.0     # m outboard of the doubler at 1.5x skin thickness (ply drop-off)


def upper_skin_factor(y):
    """Upper-skin thickness / single-skin thickness at station y (doubler and its taper)."""
    if UPPER_DOUBLER_Y is None:
        return 1.0
    y0, y1 = UPPER_DOUBLER_Y
    if y0 <= y <= y1:
        return 2.0
    if y1 < y <= y1 + UPPER_DOUBLER_TAPER:
        return 1.5
    return 1.0


@dataclass
class Layup:
    name: str
    skin: Mat
    t_ply: float          # m, cured thickness of one skin ply
    areal_ply: float      # kg/m2, cloth + resin of one ply
    n_ply: int
    cap_w: float = 0.012  # m, spar cap width
    t_min: float = 0.0005  # m, minimum cap thickness (one 0.5 mm strip)
    finish_areal: float = 0.030   # kg/m2 per wetted area: film/paint + glue, ASSUMPTION


LAYUPS = [
    Layup("1 ply glass", GLASS45, 0.08e-3, 0.16, 1),
    Layup("2 ply glass", GLASS45, 0.08e-3, 0.16, 2),
    Layup("1 ply carbon", CARBON45, 0.11e-3, 0.17, 1),
]


def geometry():
    xr = [(0.0, 0.0)]
    for xl, w, h, rc in vm.POD_SECTIONS:
        xr.append((xl * vm.POD_LEN, math.sqrt((w * h - (4 - math.pi) * rc**2) / math.pi)))
    xr.append((vm.POD_LEN, 0.0))
    return st.Geometry(winglet_h=0.25, washout_deg=vm.WASHOUT, body_xr=tuple(xr), body_x0=vm.POD_X0,
                       body_z0=vm.POD_Z)


GEO = geometry()
S = GEO.S
WS = M_DESIGN * G0 / S


# ----------------------------------------------------------------------------- airfoil section properties
_XC = np.linspace(0, 1, 401)
_T = AF.local_thickness(_XC)


def thick(xc):
    return float(np.interp(xc, _XC, _T))


def section(c, lay: Layup, t_cap, core=XPS):
    """Per-unit-span properties of a streamwise section of chord c with caps of thickness t_cap.
    Returns dict: mass/length, chordwise CG (m from LE), I about LE, EI, GJ, shear centre (m from LE),
    cap lever arm h_eff, torsion-cell area."""
    x = _XC * c
    t = _T * c
    ts = lay.t_ply * lay.n_ply
    # foam core over the full chord (elevon included)
    A_core = np.trapezoid(t, x)
    xc_core = np.trapezoid(t * x, x) / A_core
    Ix_core = np.trapezoid(t * x**2, x)            # second moment about LE (area * x^2)
    # skins: upper + lower ~ 2 x chord length (with the thickness slope)
    ds = np.hypot(np.diff(x), np.diff(t / 2)) * 2
    xm = (x[1:] + x[:-1]) / 2
    P = ds.sum()
    m_skin_x = ds * (lay.areal_ply * lay.n_ply + lay.finish_areal)
    # spar: caps + web block
    h = thick(X_SPAR) * c
    h_eff = h - 2 * ts - t_cap
    m_caps = 2 * lay.cap_w * t_cap * CARBON_UD.rho
    m_web = lay.cap_w * max(h - 2 * ts - 2 * t_cap, 0) * ROHA51.rho + 2 * h * GLASS45.rho * 0.08e-3
    # rear web: one glass ply each side of 3 mm XPS
    h_r = thick(X_REAR) * c
    m_rear = h_r * (0.003 * XPS.rho + 2 * 0.16)
    m_hinge = 0.010                                 # kg/m, hinge tape + elevon edge, ASSUMPTION
    xs, xr = X_SPAR * c, X_REAR * c
    parts = [(A_core * core.rho, xc_core, Ix_core * core.rho - A_core * core.rho * xc_core**2),
             (m_skin_x.sum(), np.sum(m_skin_x * xm) / m_skin_x.sum(),
              np.sum(m_skin_x * xm**2) - m_skin_x.sum() * (np.sum(m_skin_x * xm) / m_skin_x.sum())**2),
             (m_caps, xs, 0.0), (m_web, xs, 0.0), (m_rear, xr, 0.0), (m_hinge, 0.75 * c, 0.0)]
    m = sum(p[0] for p in parts)
    xcg = sum(p[0] * p[1] for p in parts) / m
    I_le = sum(p[2] + p[0] * p[1]**2 for p in parts)
    # bending stiffness about the chord plane (caps dominate; skins add a little)
    EI_caps = 2 * CARBON_UD.E * lay.cap_w * t_cap * (h_eff / 2) ** 2
    zt = t / 2
    EI_skin = lay.skin.E * ts * np.sum(ds * np.repeat(((zt[1:] + zt[:-1]) / 2) ** 2, 1))
    EI = EI_caps + EI_skin
    # torsion: Bredt single cell from LE to the rear web
    k = x <= xr
    A_cell = np.trapezoid(t[k], x[k])
    P_cell = np.sum(ds[k[1:]]) + h_r
    web_gt = 2 * 0.08e-3 * GLASS45.G + 0.003 * XPS.G
    GJ = 4 * A_cell**2 / ((P_cell - h_r) / (lay.skin.G * ts) + h_r / web_gt)
    GJ += core.G * np.trapezoid(t**3, x) / 3      # open foam core, small
    # Uniform-wall, single-cell approximation for the elastic axis.
    xsc = shear_centre(x[k], t[k], xr, h_r)
    return dict(m=m, xcg=xcg, I_le=I_le, EI=EI, GJ=GJ, xsc=xsc, h_eff=h_eff, A_cell=A_cell,
                ts=ts, P=P, h=h)


def shear_centre(x, t, xr, h_r, n=200):
    """Shear centre (x from LE) of a single closed symmetric cell under a vertical shear, skin-only
    booms (uniform skin thickness), via open-section shear flow + closing flow for zero twist."""
    # contour: upper surface LE->rear web, web down, lower surface rear->LE (symmetric about z=0)
    xu = np.linspace(x[0], xr, n)
    zu = np.interp(xu, x, t / 2)
    zu[-1] = h_r / 2
    upper = np.column_stack([xu, zu])
    lower = upper[::-1] * [1, -1]
    pts = np.vstack([upper, lower, upper[:1]])
    seg = np.diff(pts, axis=0)
    L = np.hypot(seg[:, 0], seg[:, 1])
    mid = (pts[1:] + pts[:-1]) / 2
    Iyy = np.sum(L * mid[:, 1] ** 2)                       # per unit skin thickness
    increments = L * mid[:, 1]
    qb = -(np.cumsum(increments) - increments / 2) / Iyy   # flow at each segment midpoint
    # zero-twist closing flow: sum(q L) = 0 for uniform thickness
    q = qb - np.sum(qb * L) / np.sum(L)
    # moment of the shear flows about the origin (x=0, z=0): r x (q * ds)
    mom = np.sum(q * (mid[:, 0] * seg[:, 1] - mid[:, 1] * seg[:, 0]))
    # A vertical force at x_sc has the same moment, x_sc * Fz.
    Fz = np.sum(q * seg[:, 1])
    return float(mom / Fz)


# ----------------------------------------------------------------------------- loads
def strips_at(n, V, rho, aileron=None, *, mass=M_DESIGN, x_cg=0.422, roll_rate=0.0):
    """AVL strip loads trimmed at signed lift coefficient nW/qS and speed V.
    aileron: fixed differential deflection (deg); roll_rate is AVL's stability-axis p'b/2V."""
    import subprocess
    import tempfile
    x_batt, x_kit = st.place_for_cg(GEO, x_cg)
    items = st.mass_items(GEO, x_batt, x_kit, True)
    mi = av.cg_and_inertia(items)
    q = 0.5 * rho * V**2
    cl_target = n * mass * G0 / (q * S)
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        av.write_avl(GEO, mi["x"], d / "rw.avl", cdp=0.0)
        av.write_mass(items, d / "rw.mass", rho)
        roll = f"d2 d2 {aileron}" if aileron is not None else "d2 rm 0"
        keys = ["mass rw.mass", "mset 0", "plop", "g", "", "oper", "c1", f"v {V}", f"m {mass}", f"d {rho}",
                "g 9.81", f"x {x_cg}", f"z {mi['z']}", "", f"a c {cl_target:.8f}",
                "d1 pm 0", roll, f"r r {roll_rate}", "x", "st", "st.txt", "fs", "fs.txt",
                "", "quit", ""]
        process = subprocess.run([str(av.AVL_EXE), "rw.avl"], cwd=d, input="\n".join(keys) + "\n", text=True,
                                 capture_output=True, timeout=60, check=True)
        if not (d / "st.txt").is_file() or not (d / "fs.txt").is_file():
            raise RuntimeError(f"AVL did not produce trim and strip outputs:\n{process.stdout[-2000:]}")
        stt = av.parse_eq((d / "st.txt").read_text())
        rows, surf, cols = [], None, None
        for line in (d / "fs.txt").read_text().splitlines():
            if "Surface #" in line:
                surf = "R" if ("Wing" in line and "Winglet" not in line and "YDUP" not in line) else \
                       "L" if ("Wing (YDUP)" in line) else "WL" if "Winglet" in line else None
            p = line.split()
            if p[:1] == ["j"]:
                cols = p
                continue
            if surf and cols and len(p) == len(cols) and p[0].isdigit():
                v = dict(zip(cols, map(float, p)))
                rows.append(dict(side=surf, x_le=v["Xle"], y=v["Yle"], z=v["Zle"], c=v["Chord"], area=v["Area"],
                                 cl=v["cl"], cd=v["cd"], cm=v["cm_c/4"]))
    df = pd.DataFrame(rows)
    if (abs(stt.get("CLtot", math.inf) - cl_target) > 1e-4
            or abs(stt.get("Cmtot", math.inf)) > 1e-4
            or abs(stt.get("p'b/2V", math.inf) - roll_rate) > 1e-4
            or df.empty or set(df.side) != {"L", "R", "WL"}):
        raise RuntimeError(f"Invalid AVL trim/strips at {n:+.2f} g and {V:g} m/s: {stt}")
    alpha = math.radians(stt["Alpha"])
    df["L"] = q * df.area * (df.cl * math.cos(alpha) + df.cd * math.sin(alpha))
    df["M"] = q * df.area * df.c * df.cm           # N m, nose-up about the strip's c/4
    return df, stt, q


@lru_cache(maxsize=512)
def section_cl_max(c, V, rho=RHO0):
    mu = 1.81e-5
    Re = rho * V * c / mu
    a = AF.get_aero_from_neuralfoil(alpha=np.linspace(0, 18, 73), Re=Re, mach=V / 340, model_size="large")
    return float(np.max(a["CL"]))


def vn_cases(*, mass=M_DESIGN, rho=RHO0, x_cg=0.422):
    """Limit load factors for each design case."""
    # VC and VD are equivalent airspeeds for this load screen.
    v_factor = math.sqrt(RHO0 / rho)
    vc, vd = VC * v_factor, VD * v_factor
    ws = mass * G0 / S
    df, stt, q = strips_at(1.0, vc, rho, mass=mass, x_cg=x_cg)
    a3d = stt["CLa"]                               # per rad, trimmed aircraft
    cbar = S / GEO.span
    mu_g = 2 * ws / (rho * cbar * a3d * G0)
    kg = 0.88 * mu_g / (5.3 + mu_g)

    def dn(V, U):
        return kg * rho * U * V * a3d / (2 * ws)

    # stall cap: wing CL when the most loaded strip reaches its section maximum (+ overshoot)
    w = df[df.side == "R"]
    ratio = min(section_cl_max(c, vc, rho) / cl for c, cl in zip(w.c, w.cl) if cl > 0.05)
    CL1 = stt["CLtot"]
    CL_cap = CL1 * ratio * OVERSHOOT

    def cap_pos(V):
        return 0.5 * rho * V**2 * S * CL_cap / (mass * G0)

    def cap_neg(V):
        return 0.5 * rho * V**2 * S * CL_MIN_WING / (mass * G0)

    rows = []
    for name, V, n_raw in (("manoeuvre +", vd, N_MAN_POS), ("gust + at Vc", vc, 1 + dn(vc, UDE_C)),
                           ("gust + at VD", vd, 1 + dn(vd, UDE_D)), ("manoeuvre -", vd, N_MAN_NEG),
                           ("gust - at Vc", vc, 1 - dn(vc, UDE_C)), ("gust - at VD", vd, 1 - dn(vd, UDE_D))):
        n = min(n_raw, cap_pos(V)) if n_raw > 0 else max(n_raw, cap_neg(V))
        if name.startswith("manoeuvre"):
            n = n_raw
        rows.append(dict(case=name, V=V, n_formula=n_raw, n_stall_cap=cap_pos(V) if n_raw > 0 else cap_neg(V),
                         n_limit=n, n_ultimate=FOS * n))
    info = dict(a3d=a3d, mu_g=mu_g, kg=kg, CL_cap=CL_cap, Vs1=math.sqrt(2 * ws / (rho * CL_cap / OVERSHOOT)))
    return pd.DataFrame(rows), info


def spar_loads(df, n, wing_section, y_st, side="R", *, roll_accel=0.0, extra_masses=(),
               battery_mass_side=M_BATT_SIDE):
    """Shear, bending and torsion along the swept spar (right wing), from strip loads and inertia relief.
    Returns arrays at stations y_st (projected span)."""
    e = np.array([math.sin(SWEEP), math.cos(SWEEP)])          # spar axis in (x, y)
    nrm = np.array([math.cos(SWEEP), -math.sin(SWEEP)])
    wing = df[df.side == side]
    # lift acts at each strip's quarter chord; strip pitching moment about the same point
    pts = [(xl + 0.25 * c, abs(y), Lz, M)
           for xl, y, c, Lz, M in zip(wing.x_le, wing.y, wing.c, wing.L, wing.M)]
    # winglet side force has no vertical component; its root bending is local to the winglet
    # inertia relief: wing structure (by strip), batteries, winglet, servo (downward n*m*g)
    masses = []
    if wing_section is not None:
        yy = np.linspace(0, 1.5, 151)
        dy = yy[1] - yy[0]
        for y in (yy[1:] + yy[:-1]) / 2:
            mpl, xcg, _ = wing_section(y)
            masses.append((GEO.x_le(y) + xcg, y, mpl * dy))
    for yb in np.linspace(*Y_BATT, 5):
        masses.append((X_BATT_GLOBAL, yb, battery_mass_side / 5))
    masses.append((GEO.x_le(1.5) + 0.4 * GEO.chord(1.5), 1.5, M_WINGLET))
    masses.append((GEO.x_le(Y_SERVO) + 0.6 * GEO.chord(Y_SERVO), Y_SERVO, M_SERVO))
    masses.extend(extra_masses)
    side_sign = 1 if side == "R" else -1
    x_a, y_a, force_a, pitch = np.asarray(pts).T
    x_m, y_m, mass = np.asarray(masses).T
    x = np.r_[x_a, x_m]
    y = np.r_[y_a, y_m]
    # Positive AVL roll lowers the right wing in the z-up geometry.
    force = np.r_[force_a, -mass * (n * G0 - side_sign * roll_accel * y_m)]
    outboard = np.column_stack((y_a > y_st[:, None], y_m >= y_st[:, None]))
    forces = outboard * force
    x_spar = np.array([X_SPAR * GEO.chord(y) for y in y_st])
    x_cut = x_spar + np.array([GEO.x_le(y) for y in y_st])
    x_sc = x_spar if wing_section is None else np.array([wing_section(y)[2] for y in y_st])
    shear = forces.sum(axis=1)
    moment_x = ((y - y_st[:, None]) * forces).sum(axis=1)
    moment_y = ((x_cut[:, None] - x) * forces).sum(axis=1) + outboard[:, :len(y_a)] @ pitch
    bend = moment_x * nrm[0] + moment_y * nrm[1]
    torque = moment_x * e[0] + moment_y * e[1]
    return pd.DataFrame(dict(y=y_st, V=shear, Mb=bend, T=torque + shear * (x_sc - x_spar) * e[1],
                             T_spar=torque))


# ----------------------------------------------------------------------------- sizing
def size_wing(lay: Layup, env: pd.DataFrame, y_st, *, core_at=None):
    """Cap thickness per station for the ultimate bending envelope; returns the per-station table."""
    rows = []
    t_prev = None
    for _, r in env.iterrows():
        y = r.y
        c = GEO.chord(y)
        t_cap = lay.t_min
        sig_wr = 0.5 * (CARBON_UD.E * ROHA51.E * ROHA51.G) ** (1 / 3) * 0.8
        allow_c = min(CARBON_UD.s_c, sig_wr)
        core = XPS if core_at is None else core_at(y)
        # Skin wrinkling on the foam core (Hoff, design coefficient WRINKLE_K). The skin shares the cap strain,
        # scaled out to the surface; limit the cap stress so the compressed skin stays below its wrinkling stress.
        skin_wr = WRINKLE_K * (lay.skin.E * core.E * core.G) ** (1 / 3)
        for _ in range(60):
            s = section(c, lay, t_cap, core)
            if s["h"] - 2 * s["ts"] - 2 * t_cap <= 0:
                raise ValueError(f"Spar caps do not fit at y={y:.3f} m for {lay.name}.")
            A = lay.cap_w * t_cap
            # positive bending: top cap compression; negative: bottom cap compression
            s_top = abs(r.Mb_pos) / (A * s["h_eff"])
            s_bot = abs(r.Mb_neg) / (A * s["h_eff"])
            # Skin wrinkling uses the minimum principal stress of the compressed face: spanwise compression
            # (cap strain scaled to the surface) combined with the torsion shear in that face. The FE model
            # (fea_wing.py) showed the shear term governs inboard; spanwise stress alone is unconservative.
            f_up = upper_skin_factor(y)
            dbl = f_up > 1.0
            k_surf = lay.skin.E / CARBON_UD.E * s["h"] / s["h_eff"]
            p_face = 0.0
            for s_cap, ts_face in ((s_top, s["ts"] * f_up), (s_bot, s["ts"])):
                tau_face = abs(r.T_max) / (2 * s["A_cell"] * ts_face)
                s_face = s_cap * k_surf
                p_face = max(p_face, s_face / 2 + math.sqrt((s_face / 2) ** 2 + tau_face**2))
            p_face *= SKIN_FE_FACTOR if y < SKIN_FE_FACTOR_Y else 1.0
            if abs(r.T_max) / (2 * s["A_cell"] * s["ts"]) >= skin_wr:
                raise ValueError(f"Torsion shear alone wrinkles the skin at y={y:.3f} m for {lay.name}.")
            ok = (max(s_top, s_bot) <= allow_c and p_face <= skin_wr and max(s_top, s_bot) <= CARBON_UD.s_t)
            if ok and t_prev is not None and t_cap < t_prev - 1e-9:   # first station past a ply drop
                ok = p_face * CAP_DROP_FACTOR <= skin_wr
            if ok:
                break
            t_cap += 0.00025
        else:
            raise ValueError(f"No spar cap size meets the bending allowable at y={y:.3f} m for {lay.name}.")
        t_prev = t_cap
        tau_skin = abs(r.T_max) / (2 * s["A_cell"] * s["ts"])
        q_web = abs(r.V_max) / s["h_eff"]
        shear_strain = min(GLASS45.tau / GLASS45.G, ROHA51.tau / ROHA51.G)
        q_web_allow = (2 * 0.08e-3 * GLASS45.G + lay.cap_w * ROHA51.G) * shear_strain
        rows.append(dict(y=y, chord=c, h_spar_mm=s["h"] * 1e3, t_cap_mm=t_cap * 1e3, sigma_cap_MPa=max(s_top, s_bot) / 1e6,
                         allow_MPa=allow_c / 1e6, tau_skin_MPa=tau_skin / 1e6, skin_allow_MPa=lay.skin.tau / 1e6,
                         q_web_Nmm=q_web / 1e3, q_web_allow_Nmm=q_web_allow / 1e3,
                         m_per_m=s["m"] + lay.areal_ply * s["P"] / 2 * (f_up - 1.0), xcg_c=s["xcg"] / c, xsc_c=s["xsc"] / c, EI=s["EI"], GJ=s["GJ"],
                         I_le=s["I_le"],
                         skin_comp_MPa=max(s_top, s_bot) * lay.skin.E / CARBON_UD.E * s["h"] / s["h_eff"] / 1e6,
                         skin_principal_MPa=p_face / 1e6, upper_doubler=dbl,
                         skin_wrinkle_MPa=skin_wr / 1e6, core=core.name))
    return pd.DataFrame(rows)


def wing_mass(tab):
    """Structure mass of the whole wing (both sides, streamwise strips) incl. elevons, excl. winglets."""
    y = tab.y.values
    return 2 * np.trapezoid(tab.m_per_m.values, y) + 2 * M_SERVO


# ----------------------------------------------------------------------------- deflection
def tip_deflection(tab, bending_moment):
    """Tip deflection along the spar from M/EI, double integration (cantilever at the centreline)."""
    s = tab.y.values / math.cos(SWEEP)
    kappa = np.asarray(bending_moment) / tab.EI.to_numpy()
    slope = np.concatenate([[0], np.cumsum((kappa[1:] + kappa[:-1]) / 2 * np.diff(s))])
    w = np.concatenate([[0], np.cumsum((slope[1:] + slope[:-1]) / 2 * np.diff(s))])
    return w


# ----------------------------------------------------------------------------- flutter (beam FE + Theodorsen)
def theodorsen(k):
    k = np.maximum(k, 1e-6)
    H1, H0 = hankel2(1, k), hankel2(0, k)
    return H1 / (H1 + 1j * H0)


def beam_matrices(yn, EI, GJ, m, S_a, I_a):
    """Clamped-free beam: nodes yn, element properties (len(yn)-1). DOFs per node: w, w', theta.
    Hermite bending, linear torsion, consistent mass including the S_a coupling."""
    nn = len(yn)
    nd = 3 * nn
    K = np.zeros((nd, nd))
    M = np.zeros((nd, nd))
    for e in range(nn - 1):
        L = yn[e + 1] - yn[e]
        kb = EI[e] / L**3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L**2, -6 * L, 2 * L**2],
                                      [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L**2, -6 * L, 4 * L**2]])
        mb = m[e] * L / 420 * np.array([[156, 22 * L, 54, -13 * L], [22 * L, 4 * L**2, 13 * L, -3 * L**2],
                                        [54, 13 * L, 156, -22 * L], [-13 * L, -3 * L**2, -22 * L, 4 * L**2]])
        kt = GJ[e] / L * np.array([[1, -1], [-1, 1]])
        mt = I_a[e] * L / 6 * np.array([[2, 1], [1, 2]])
        ib = [3 * e, 3 * e + 1, 3 * e + 3, 3 * e + 4]
        it = [3 * e + 2, 3 * e + 5]
        K[np.ix_(ib, ib)] += kb
        M[np.ix_(ib, ib)] += mb
        K[np.ix_(it, it)] += kt
        M[np.ix_(it, it)] += mt
        # Integrate each Hermite shape function against each linear shape function.
        coupling = S_a[e] * L / 60 * np.array([[21, 9], [3 * L, 2 * L],
                                                 [9, 21], [-2 * L, -3 * L]])
        M[np.ix_(ib, it)] += coupling
        M[np.ix_(it, ib)] += coupling.T
    free = np.arange(3, nd)       # clamp node 0
    return K[np.ix_(free, free)], M[np.ix_(free, free)]


def flutter(yn, EI, GJ, m, S_a, I_a, b, a, rho, n_modes=6, cla=2 * math.pi, g_s=0.0, kmax=3.0):
    """V-g method. b, a per element (semichord, elastic axis position in semichords from mid-chord).
    Returns modal frequencies (rad/s), the V-g table and the lowest flutter speed where g crosses g_s."""
    K, M = beam_matrices(yn, EI, GJ, m, S_a, I_a)
    w2, Phi = eigh(K, M)
    if np.any(w2 <= 0):
        raise ValueError("The clamped beam has nonpositive squared natural frequencies.")
    wn = np.sqrt(w2[:n_modes])
    Phi = Phi[:, :n_modes]
    Kq = Phi.T @ K @ Phi
    Mq = Phi.T @ M @ Phi
    # node-based strip aero (lumped): strip width per node
    nn = len(yn)
    yc = (yn[1:] + yn[:-1]) / 2
    wid = np.zeros(nn)
    wid[1:-1] = (yn[2:] - yn[:-2]) / 2
    wid[-1] = (yn[-1] - yn[-2]) / 2
    bn = np.interp(yn, yc, b)
    an = np.interp(yn, yc, a)
    bref = bn[0]
    rows = []
    ks = np.geomspace(kmax, 0.005, 400)
    for step, k in enumerate(ks):
        A = np.zeros((3 * nn, 3 * nn), complex)
        for i in range(1, nn):
            bi, ai, kl = bn[i], an[i], k * bn[i] / bref
            C = theodorsen(kl) * cla / (2 * math.pi)
            f = math.pi * rho * bi**2 * wid[i]
            # L/omega^2 = f * [ -h + i(b/k)a? ... ] written for q = [h, alpha]
            Lh = f * (-1 + 2j * C / kl)
            La = f * (1j * bi / kl + bi * ai + 2 * C / kl * (bi / kl + 1j * bi * (0.5 - ai)))
            Mh = f * (-bi * ai + 2 * bi * (ai + 0.5) * C / kl * 1j)
            Ma = f * (bi**2 * (0.125 + ai**2) - 1j * bi**2 / kl * (0.5 - ai)
                      + 2 * bi * (ai + 0.5) * C / kl * (bi / kl + 1j * bi * (0.5 - ai)))
            ih, ia = 3 * i, 3 * i + 2
            # generalized forces [-L, M] = omega^2 * A q
            A[ih, ih] += -Lh
            A[ih, ia] += -La
            A[ia, ih] += Mh
            A[ia, ia] += Ma
        Af = A[3:, 3:]
        Aq = Phi.T @ Af @ Phi
        lam, shapes = eig(Mq + Aq, Kq)
        for l, shape in zip(lam, shapes.T):
            if l.real <= 0:
                continue
            om = 1 / math.sqrt(l.real)
            rows.append(dict(step=step, k=k, omega=om, g=l.imag / l.real, V=om * bref / k, shape=shape))
    vg = pd.DataFrame(rows)
    vg = track(vg)
    Vf, wf = math.inf, math.nan
    for _, d in vg.groupby("mode"):
        d = d.sort_values("step")
        g, speeds, steps = d.g.to_numpy(), d.V.to_numpy(), d.step.to_numpy()
        crossings = np.flatnonzero((np.diff(steps) == 1) & (np.diff(speeds) > 0)
                                   & (g[:-1] < g_s) & (g[1:] >= g_s))
        for i in crossings:
            t = (g_s - d.g.values[i]) / (d.g.values[i + 1] - d.g.values[i])
            V = d.V.values[i] + t * (d.V.values[i + 1] - d.V.values[i])
            if V < Vf:
                Vf, wf = V, d.omega.values[i] + t * (d.omega.values[i + 1] - d.omega.values[i])
    return wn, vg, Vf, wf


def track(vg):
    """Track modes between adjacent reduced-frequency samples by modal shape and frequency."""
    out = []
    prev = None
    next_id = 0
    for step, d in vg.groupby("step", sort=True):
        d = d.sort_values("omega").copy()
        ids = np.full(len(d), -1)
        if prev is not None and step == prev.step.iloc[0] + 1:
            shapes = np.stack(d["shape"])
            previous_shapes = np.stack(prev["shape"])
            overlap = np.abs(shapes @ previous_shapes.conj().T) ** 2
            frequency_distance = np.abs(np.log(d.omega.values[:, None] / prev.omega.values[None, :]))
            i, j = linear_sum_assignment(1 - overlap + frequency_distance)
            ids[i] = prev["mode"].to_numpy()[j]
        new = ids < 0
        ids[new] = np.arange(next_id, next_id + new.sum())
        next_id += new.sum()
        d["mode"] = ids
        out.append(d)
        prev = d
    return pd.concat(out, ignore_index=True).drop(columns="shape")


def goland(ne=30, n_modes=4):
    L = 6.096
    yn = np.linspace(0, L, ne + 1)
    c = 1.8288
    one = np.ones(ne)
    m = 35.71 * one
    xa = (0.43 - 0.33) * c
    wn, vg, Vf, wf = flutter(yn, 9.77e6 * one, 0.99e6 * one, m, m * xa, 8.64 * one, c / 2 * one,
                             (2 * 0.33 - 1) * one, RHO0, n_modes=n_modes)
    return wn, vg, Vf, wf


def wing_beam(tab, ne=30, *, extra_masses=()):
    """Beam properties along the projected span from the sized wing table (streamwise sections)."""
    yn = np.linspace(0, 1.5, ne + 1)
    yc = (yn[1:] + yn[:-1]) / 2
    f = lambda col: np.interp(yc, tab.y.values, tab[col].values)  # noqa: E731
    c = np.array([GEO.chord(y) for y in yc])
    m = f("m_per_m")
    xcg = f("xcg_c") * c
    xsc = f("xsc_c") * c
    I_le = f("I_le")
    # Distribute each component's mass over the elements it occupies.
    xb_local = np.array([X_BATT_GLOBAL - GEO.x_le(y) for y in yc])
    first_moment = m * xcg
    for e, y in enumerate(yc):
        L = yn[e + 1] - yn[e]
        lo, hi = max(yn[e], Y_BATT[0]), min(yn[e + 1], Y_BATT[1])
        components = []
        if hi > lo:
            mb = M_BATT_SIDE * (hi - lo) / (Y_BATT[1] - Y_BATT[0]) / L
            components.append((mb, xb_local[e], 0.0))
        if yn[e] <= Y_SERVO < yn[e + 1]:
            components.append((M_SERVO / L, 0.6 * c[e], 0.0))
        if e == ne - 1:
            components.append((M_WINGLET / L, 0.4 * c[e], GEO.winglet_h**2 / 3))
        for x, y_part, mass in extra_masses:
            if yn[e] <= y_part < yn[e + 1]:
                components.append((mass / L, x - GEO.x_le(y_part), 0.0))
        for mass_per_span, x, z_squared in components:
            m[e] += mass_per_span
            first_moment[e] += mass_per_span * x
            I_le[e] += mass_per_span * (x**2 + z_squared)
    S_a = first_moment - m * xsc
    I_a = I_le - 2 * xsc * first_moment + m * xsc**2
    b = c / 2
    a = 2 * xsc / c - 1
    return yn, f("EI"), f("GJ"), m, S_a, I_a, b, a


def performance_sensitivity(wing_mass_kg):
    """Propagate the new mass through the existing performance model with other inputs fixed."""
    from v3_mission import lodestar_v3

    rows = []
    for cell, speed in (("Amprius SiCore 6.3 Ah", 20.0), ("Molicel P50B", 18.0)):
        for altitude in (2000, 4400):
            for label, mass in (("original budget", None), ("lightest modeled wing", wing_mass_kg)):
                aircraft = lodestar_v3(cell, cell, "kit", "halfway", alt=altitude, wing_mass_kg=mass)
                stall = aircraft.v_min / 1.25
                eas = stall * math.sqrt(aircraft.rw_cfg["rho"] / RHO0)
                # The sizing convention keeps 10% reserve after the 85% usable battery fraction.
                search_hours = 0.90 * aircraft.energy / float(aircraft.power(speed))
                rows.append(dict(cell=cell, mass_case=label, altitude_m=altitude, takeoff_mass_kg=aircraft.mass,
                                 stall_eas_mps=eas, stall_tas_mps=stall, search_tas_mps=speed,
                                 meets_search_stall_margin=speed >= aircraft.v_min, search_hours=search_hours))
    return pd.DataFrame(rows)


def balance_sensitivity(tab, neutral_point):
    """Replace the CAD wing mass and longitudinal moment while holding all other items fixed."""
    cad = pd.read_csv(ROOT / "out" / "cad_v3" / "mass_cg.csv")
    wing = cad.item.str.startswith("wing structure (")
    if wing.sum() != 1:
        raise ValueError("Expected exactly one wing structure item in the CAD mass table.")
    baseline_mass = cad.mass_kg.sum()
    baseline_moment = (cad.mass_kg * cad.x).sum()
    x_sections = tab.y.map(GEO.x_le) + tab.xcg_c * tab.chord
    x_servo = GEO.x_le(Y_SERVO) + 0.6 * GEO.chord(Y_SERVO)
    revised_moment = ((cad.loc[~wing, "mass_kg"] * cad.loc[~wing, "x"]).sum()
                      + 2 * np.trapezoid(tab.m_per_m * x_sections, tab.y) + 2 * M_SERVO * x_servo)
    revised_mass = cad.loc[~wing, "mass_kg"].sum() + wing_mass(tab)
    rows = []
    for label, mass, moment in (("v3 CAD", baseline_mass, baseline_moment),
                                ("lightest modeled wing", revised_mass, revised_moment)):
        cg = moment / mass
        rows.append(dict(mass_case=label, takeoff_mass_kg=mass, cg_x_m=cg, neutral_point_x_m=neutral_point,
                         static_margin_pct_mac=100 * (neutral_point - cg) / GEO.mac))
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------- run
def main():
    print("Lodestar preliminary structures screen")
    print(f"Design mass {M_DESIGN} kg, wing area {S:.3f} m2, wing loading {WS:.1f} N/m2; Vc {VC} m/s, VD {VD} m/s\n")

    # 0. flutter code check on the Goland wing
    x_box = shear_centre(np.array([0.0, 2.0]), np.ones(2), 2.0, 1.0)
    np.testing.assert_allclose(x_box, 1.0, atol=1e-8)
    _, _, coarse_speed, coarse_frequency = goland(ne=15)
    wn, vg_g, Vf_g, wf_g = goland(ne=30, n_modes=6)
    # Sea-level, two-dimensional, unsteady-aero reference (DOI 10.3390/app16115687).
    np.testing.assert_allclose([Vf_g, wf_g], [137.1, 2 * math.pi * 11.0], rtol=0.02)
    np.testing.assert_allclose([Vf_g, wf_g], [coarse_speed, coarse_frequency], rtol=0.01)
    print(f"Goland at sea level: {Vf_g:.2f} m/s, {wf_g:.2f} rad/s; "
          f"15-element result {coarse_speed:.2f} m/s, {coarse_frequency:.2f} rad/s. "
          f"Coupled natural frequencies {np.round(wn[:3], 1)} rad/s.\n")
    vg_g.to_csv(OUT / "goland_flutter.csv", index=False)

    # 1. load cases
    vn, info = vn_cases()
    print(f"Gust parameters: lift slope {info['a3d']:.2f}/rad, mass ratio {info['mu_g']:.2f}, alleviation factor "
          f"{info['kg']:.3f}; wing CL where the first section stalls (+10%) {info['CL_cap']:.2f}")
    print(vn.round(2).to_string(index=False))
    vn.to_csv(OUT / "load_cases.csv", index=False)

    # Compute signed symmetric trims and retain each case in the station-by-station envelope.
    y_st = np.linspace(0.0, GEO.span / 2, 61)
    cases = [(r.case, r.n_limit, strips_at(r.n_limit, r.V, RHO0)[0]) for r in vn.itertuples()]
    uncapped = [(f"{r.case} uncapped", r.n_formula, strips_at(r.n_formula, r.V, RHO0)[0])
                for r in vn.itertuples() if r.n_formula != r.n_limit]
    level, level_trim, _ = strips_at(1.0, VC, RHO0)
    roll_n = 2 / 3 * N_MAN_POS
    roll, roll_trim, _ = strips_at(roll_n, VD, RHO0, aileron=-15.0)
    print(f"Fixed-aileron sensitivity: AVL roll coefficient {roll_trim['Cltot']:.3f}; "
          "angular inertia and roll damping excluded.\n")

    summaries = []
    for lay in LAYUPS:
        tab = None

        def section_at(y):
            c = GEO.chord(y)
            if tab is None:
                return 0.85, 0.42 * c, X_SPAR * c
            return (np.interp(y, tab.y, tab.m_per_m), np.interp(y, tab.y, tab.xcg_c) * c,
                    np.interp(y, tab.y, tab.xsc_c) * c)

        for _ in range(10):
            loads = {name: spar_loads(df, n, section_at, y_st) for name, n, df in cases}
            bending = np.stack([load.Mb for load in loads.values()])
            env = pd.DataFrame(dict(y=y_st, Mb_pos=FOS * np.maximum(0, bending.max(axis=0)),
                                    Mb_neg=FOS * np.minimum(0, bending.min(axis=0)),
                                    V_max=FOS * np.max([abs(load.V) for load in loads.values()], axis=0),
                                    T_max=FOS * np.max([abs(load["T"]) for load in loads.values()], axis=0)))
            previous = tab
            tab = size_wing(lay, env, y_st)
            if previous is not None and tab.t_cap_mm.equals(previous.t_cap_mm):
                break
        else:
            raise RuntimeError(f"Wing sizing did not converge for {lay.name}.")
        mw = wing_mass(tab)
        w_lim = max(abs(tip_deflection(tab, load.Mb)[-1]) for load in loads.values())
        w_1g = tip_deflection(tab, spar_loads(level, 1.0, section_at, y_st).Mb)[-1]
        # joiner tube at the break: 16 x 13 mm carbon tube, flexural allowable 500 MPa (ASSUMPTION)
        Mj = max(abs(float(np.interp(Y_BREAK, y_st, env[col]))) for col in ("Mb_pos", "Mb_neg"))
        Do, Di = 0.016, 0.013
        Ij = math.pi / 64 * (Do**4 - Di**4)
        sig_j = Mj * (Do / 2) / Ij
        sensitivities = {name: spar_loads(df, n, section_at, y_st) for name, n, df in uncapped}
        sensitivities.update({f"fixed aileron {side}": spar_loads(roll, roll_n, section_at, y_st, side)
                              for side in ("R", "L")})
        sensitivity_joiner_stress = {name: FOS * abs(np.interp(Y_BREAK, load.y, load.Mb)) * (Do / 2) / Ij / 1e6
                                     for name, load in sensitivities.items()}
        # flutter
        yn, EI, GJ, m, S_a, I_a, b, a = wing_beam(tab)
        stem = lay.name.replace(" ", "_")
        tab.merge(env, on="y").to_csv(OUT / f"{stem}_sections.csv", index=False)
        pd.concat(loads | sensitivities, names=["case"]).reset_index(level=0).to_csv(OUT / f"{stem}_loads.csv", index=False)
        fl = {}
        for lab, rho in (("sea level", RHO0), ("4,400 m", RHO_HI)):
            wn_w, vg, Vf, wf = flutter(yn, EI, GJ, m, S_a, I_a, b, a, rho, n_modes=6)
            _, _, Vf2, _ = flutter(yn, EI, GJ, m, S_a, I_a, b, a, rho, n_modes=6, g_s=0.02)
            fl[lab] = dict(Vf=Vf, wf=wf, Vf_g02=Vf2, wn=wn_w, rho=rho)
            vg.assign(site=lab).to_csv(OUT / f"{stem}_flutter_{'sea_level' if rho == RHO0 else '4400m'}.csv", index=False)
        summaries.append(dict(layup=lay.name, mass_kg=mw, tip_1g_mm=w_1g * 1000, tip_limit_mm=w_lim * 1000,
                              joiner_moment_Nm=Mj, joiner_stress_MPa=sig_j / 1e6,
                              joiner_uncapped_max_MPa=max(sensitivity_joiner_stress[name] for name, _, _ in uncapped),
                              joiner_fixed_aileron_max_MPa=max(sensitivity_joiner_stress[f"fixed aileron {side}"] for side in ("R", "L")),
                              max_skin_torsion_utilization=(tab.tau_skin_MPa / tab.skin_allow_MPa).max(),
                              max_web_utilization=(tab.q_web_Nmm / tab.q_web_allow_Nmm).max(),
                              flutter_sea_level_mps=fl["sea level"]["Vf"], flutter_4400m_mps=fl["4,400 m"]["Vf"]))
        print(f"--- Layup: {lay.name} skins ---")
        show = tab.iloc[::3][["y", "chord", "h_spar_mm", "t_cap_mm", "sigma_cap_MPa", "allow_MPa", "tau_skin_MPa",
                              "q_web_Nmm", "q_web_allow_Nmm", "m_per_m", "xsc_c", "xcg_c", "EI", "GJ"]]
        with pd.option_context("display.width", 220):
            print(show.round(3).to_string(index=False))
        print(f"Wing structure mass (both sides, incl. elevons, servos; excl. winglets): {mw:.3f} kg "
              f"(budget {1.0 * S:.3f} kg at 1.0 kg/m2)")
        print(f"Tip deflection: {w_1g * 1000:.0f} mm at 1 g, {w_lim * 1000:.0f} mm at limit load "
              f"({w_lim / 1.5 * 100:.1f}% of half-span)")
        print(f"Joiner tube 16x13 mm at the break: ultimate moment {Mj:.1f} N m, stress {sig_j / 1e6:.0f} MPa "
              f"(allowable 500 MPa)")
        print(f"Joiner sensitivities (MPa): { {name: round(stress) for name, stress in sensitivity_joiner_stress.items()} }")
        print(f"Max skin shear {tab.tau_skin_MPa.max():.1f} MPa (allow {lay.skin.tau / 1e6:.0f}); "
              f"max web shear flow {tab.q_web_Nmm.max():.2f} N/mm (allow {tab.q_web_allow_Nmm.min():.2f})")
        for lab, f in fl.items():
            speed = f"{f['Vf']:.1f} m/s TAS" if math.isfinite(f["Vf"]) else "no crossing identified"
            loss = f"{f['Vf_g02']:.1f} m/s TAS" if math.isfinite(f["Vf_g02"]) else "no crossing identified"
            screen_eas = 1.2 * VD
            screen_tas = screen_eas * math.sqrt(RHO0 / f["rho"])
            print(f"Flutter, {lab}: natural frequencies {np.round(f['wn'][:4] / 2 / math.pi, 1)} Hz; "
                  f"{speed} (zero loss), {loss} (loss factor 0.02); "
                  f"project screen {screen_eas:.0f} m/s EAS ({screen_tas:.1f} m/s TAS)")
        print()
    summary = pd.DataFrame(summaries).replace(math.inf, np.nan)
    summary.to_csv(OUT / "summary.csv", index=False)
    lightest = summary.loc[summary.mass_kg.idxmin()]
    lightest_stem = lightest.layup.replace(" ", "_")
    balance = balance_sensitivity(pd.read_csv(OUT / f"{lightest_stem}_sections.csv"), level_trim["Xnp"])
    balance.to_csv(OUT / "balance_sensitivity.csv", index=False)
    print("Longitudinal balance with the v3 CAD component locations:")
    print(balance.round(4).to_string(index=False))
    performance = performance_sensitivity(summary.mass_kg.min())
    performance.to_csv(OUT / "performance_sensitivity.csv", index=False)
    print("Mass sensitivity at fixed search speeds (foam-core drag assumptions):")
    print(performance.round(2).to_string(index=False))


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    log = Tee(OUT / "structures_log.txt")
    with log.f, redirect_stdout(log):
        main()
