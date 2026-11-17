"""Independent aerodynamic cross-check of the Lodestar wing against AVL.

Solvers:
  AVL 3.52            (vortex lattice; the project's primary tool, run through avl_run.py)
  AeroSandbox VLM     (independent vortex lattice, wing + winglets, camber-line elevons)
  AeroSandbox AeroBuildup (semi-empirical: lifting line + NeuralFoil sections + slender-body pod)
  VSPAERO 3.56 (VLM)  (OpenVSP's vortex lattice on the same .vsp3 geometry as vsp_model.py)

Questions:
  1. Do CL_alpha, Cm_alpha, neutral point, static margin, trim, span efficiency and the
     lateral derivatives agree between solvers (Table 1, Table 2)?
  2. 3-D CLmax and where stall starts on the span (strip cl vs local NeuralFoil clmax at the
     stall Reynolds number, with the elevons at their trim deflection).
  3. How Lodestar compares with the flight-identified Skywalker X8 model.
  4. Effect of aerodynamically neutral styling options (winglet cant/sweep/taper, pod nose).

Run:   .venv/Scripts/python.exe scripts/aero_crosscheck.py           (all stages)
       .venv/Scripts/python.exe scripts/aero_crosscheck.py --no-vsp  (reuse cached VSPAERO results)
Writes: out/aero_crosscheck/*.csv, *.png, run_log.txt, avl/ and vsp/ working folders.
2026-11-14.
"""
from dataclasses import replace
from pathlib import Path
import math
import subprocess
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from avl_run import Geometry, MassItem, wing_mass_items, run_trim, write_avl, parse_eq, parse_strips, AVL_EXE  # noqa: E402
import vsp_model as vm  # noqa: E402  (geometry constants and the vspscript geometry builder)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "aero_crosscheck"
OUT.mkdir(parents=True, exist_ok=True)
STYLE = ROOT / "style" / "lodestar.mplstyle"
if STYLE.exists():
    plt.style.use(str(STYLE))


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


# ----------------------------------------------------------------------------- conditions
MASS = 8.2                 # kg, loaded (8.0-8.4 kg range)
V_CRUISE = 19.0            # m/s
ALT = 2000.0
X_CG = 0.422               # m from root LE, design balance point
G0 = 9.81
ATMO = asb.Atmosphere(altitude=ALT)
RHO, MU = float(ATMO.density()), float(ATMO.dynamic_viscosity())
CLMAX_SIZING = 0.80


def pod_xr():
    xr = [(0.0, 0.0)]
    for xl, w, h, rc in vm.POD_SECTIONS:
        xr.append((xl * vm.POD_LEN, math.sqrt((w * h - (4 - math.pi) * rc**2) / math.pi)))
    xr.append((vm.POD_LEN, 0.0))
    return tuple(xr)


G_POD = Geometry(winglet_h=0.25, washout_deg=3.0, body_xr=pod_xr(), body_x0=vm.POD_X0, body_z0=vm.POD_Z)
G_NOPOD = replace(G_POD, body_xr=())
S, MAC, B = G_POD.S, G_POD.mac, G_POD.span
AR = B**2 / S
Q_CRUISE = 0.5 * RHO * V_CRUISE**2
CL_CRUISE = MASS * G0 / (Q_CRUISE * S)
ALPHAS = np.arange(-4.0, 14.01, 2.0)


def stab_axes(Cl_b, Cn_b, alpha_deg):
    """Body-axis roll/yaw moment (or derivative) -> stability axes (AVL reports stability axes)."""
    a = math.radians(alpha_deg)
    return Cl_b * math.cos(a) + Cn_b * math.sin(a), Cn_b * math.cos(a) - Cl_b * math.sin(a)


# ============================================================================= AVL
def avl_sweep(g: Geometry, alphas, tag, de=0.0, da=0.0, beta=0.0, trim_pitch=False):
    """One AVL session, one run per alpha. Elevon fixed at `de` unless trim_pitch (then d1 pm 0).
    Returns list of (st dict, strips) per alpha."""
    d = OUT / "avl" / tag
    d.mkdir(parents=True, exist_ok=True)
    for f in d.glob("*.txt"):
        f.unlink()
    write_avl(g, X_CG, d / "rw.avl", cdp=0.0)
    keys = ["plop", "g", "", "oper", f"b b {beta}", f"d2 d2 {da}"]
    keys += ["d1 pm 0"] if trim_pitch else [f"d1 d1 {de}"]
    for i, a in enumerate(alphas):
        keys += [f"a a {a}", "x", "st", f"st_{i}.txt", "fs", f"fs_{i}.txt"]
    keys += ["", "quit", ""]
    subprocess.run([str(AVL_EXE), "rw.avl"], cwd=d, input="\n".join(keys) + "\n", text=True,
                   capture_output=True, timeout=300)
    res = []
    for i, _ in enumerate(alphas):
        st = parse_eq((d / f"st_{i}.txt").read_text())
        strips = parse_strips((d / f"fs_{i}.txt").read_text())
        res.append((st, strips))
    return res


def avl_trim(g: Geometry):
    """True trimmed level flight at MASS, V_CRUISE, with the CG at X_CG (point masses only set CG)."""
    m_wing = 1.4
    items = wing_mass_items(g, m_wing)
    x_w = sum(i.m * i.x for i in items) / m_wing
    m_rest = MASS - m_wing
    items.append(MassItem("rest", m_rest, (X_CG * MASS - x_w * m_wing) / m_rest))
    r = run_trim(g, items, V_CRUISE, RHO, workdir=OUT / "avl" / "trim")
    return r.st


def avl_block(g: Geometry, tag):
    sw = avl_sweep(g, ALPHAS, tag)
    al = np.array([s["Alpha"] for s, _ in sw])
    CL = np.array([s["CLtot"] for s, _ in sw])
    Cm = np.array([s["Cmtot"] for s, _ in sw])
    m = (al >= 0) & (al <= 8)
    trim = avl_trim(g)
    a_t = trim["Alpha"]
    st_t = avl_sweep(g, [a_t], tag + "_at_trim")[0][0]
    out = dict(
        CLa=np.polyfit(np.radians(al[m]), CL[m], 1)[0],
        CL0=float(np.interp(0.0, al, CL)),
        Cm0=float(np.interp(0.0, al, Cm)),
        Cma=np.polyfit(np.radians(al[m]), Cm[m], 1)[0],
        x_np=trim["Xnp"],
        e=trim.get("e", np.nan),
        alpha_trim=a_t, de_trim=trim["elevon"],
        CLde=st_t.get("CLd01", np.nan) * 180 / math.pi, Cmde=st_t.get("Cmd01", np.nan) * 180 / math.pi,
        Cnb=st_t["Cnb"], Clb=st_t["Clb"], Clp=st_t["Clp"], Cnr=st_t["Cnr"],
        Clda=st_t.get("Cld02", np.nan) * 180 / math.pi, Cnda=st_t.get("Cnd02", np.nan) * 180 / math.pi,
    )
    out["SM"] = (out["x_np"] - X_CG) / MAC
    polar = pd.DataFrame(dict(alpha=al, CL=CL, Cm=Cm, CDi=[s.get("CDff", np.nan) for s, _ in sw]))
    return out, polar


# ============================================================================= AeroSandbox geometry
AF = asb.Airfoil("mh45")
AF_WL = asb.Airfoil("naca0008")


def _wx(g, y, af, cs=()):
    return asb.WingXSec(xyz_le=[g.x_le(y), y, g.z(y)], chord=g.chord(y), twist=g.twist(y), airfoil=af,
                        control_surfaces=list(cs))


WINGLET_GAP = 0.02   # m. ASB VLM only: with the winglet root touching the twisted wing tip, the two
                     # trailing legs nearly coincide and the solution diverges with refinement
                     # (CLa 4.5 -> 24 /rad). A 2 cm gap converges (x_np within 0.3 mm from 20x12 to 28x14).


def winglet_wing(g: Geometry, airfoil=AF_WL, gap=0.0):
    """Winglet as in avl_run.write_avl. `gap` (ASB VLM only) starts the surface `gap` up its own
    span line, so the top section and the leading-edge line are unchanged."""
    yt = g.span / 2
    xt, zt, ct = g.x_le(yt), g.z(yt), g.chord(yt)
    cant = math.radians(g.winglet_cant_deg)
    y2, z2 = yt + g.winglet_h * math.sin(cant), zt + g.winglet_h * math.cos(cant)
    x2 = xt + g.winglet_h * math.tan(math.radians(g.winglet_sweep_deg)) + 0.25 * ct * (1 - g.winglet_taper)
    f = gap / g.winglet_h
    x1, y1, z1 = xt + f * (x2 - xt), yt + f * (y2 - yt), zt + f * (z2 - zt)
    c1 = ct + f * (ct * g.winglet_taper - ct)
    return asb.Wing(name="winglet", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[x1, y1, z1], chord=c1, airfoil=airfoil),
        asb.WingXSec(xyz_le=[x2, y2, z2], chord=ct * g.winglet_taper, airfoil=airfoil)])


def vlm_airplane(g: Geometry, de=0.0, da=0.0):
    """Four lifting surfaces so the elevon panels are separate: inboard (symmetric), right and
    left outboard (elevon camber = MH45 with the aft 25% rotated), winglets.
    ASB VLM meshes the camber line, so a deflected airfoil is a deflected elevon."""
    yb, yt = g.y_break, g.span / 2

    def flap(d):
        return AF if d == 0 else AF.add_control_surface(deflection=d, hinge_point_x=g.hinge_xc)

    afR, afL = flap(de + da), flap(de - da)   # positive aileron: right TE down (AVL convention)
    inb = asb.Wing(name="inboard", symmetric=True, xsecs=[_wx(g, 0.0, AF), _wx(g, yb, AF)])
    right = asb.Wing(name="outR", symmetric=False, xsecs=[_wx(g, yb, afR), _wx(g, yt, afR)])
    left = asb.Wing(name="outL", symmetric=False, xsecs=[
        asb.WingXSec(xyz_le=[g.x_le(yt), -yt, g.z(yt)], chord=g.chord(yt), twist=g.twist(yt), airfoil=afL),
        asb.WingXSec(xyz_le=[g.x_le(yb), -yb, g.z(yb)], chord=g.chord(yb), twist=g.twist(yb), airfoil=afL)])
    wings = [inb, right, left]
    if g.winglet_h > 0:
        wings.append(winglet_wing(g, gap=WINGLET_GAP))
    return asb.Airplane(name="lodestar", xyz_ref=[X_CG, 0, 0], wings=wings, s_ref=g.S, c_ref=g.mac, b_ref=g.span)


VLM_RES = dict(spanwise_resolution=20, chordwise_resolution=12)


def vlm_run(g, alpha, de=0.0, da=0.0, beta=0.0, V=V_CRUISE, keep=False):
    op = asb.OperatingPoint(atmosphere=ATMO, velocity=V, alpha=alpha, beta=beta)
    v = asb.VortexLatticeMethod(vlm_airplane(g, de, da), op, **VLM_RES)
    r = v.run()
    return (r, v) if keep else r


def vlm_strips(v: asb.VortexLatticeMethod, nc=VLM_RES["chordwise_resolution"]):
    """Spanwise lift: group each run of `nc` chordwise panels into a strip (wing, not winglet)."""
    op = v.op_point
    F = v.forces_geometry
    Fw = np.array(op.convert_axes(*op.convert_axes(F[:, 0], F[:, 1], F[:, 2], "geometry", "body"), "body", "wind"))
    L = -Fw[2]
    n = len(L) // nc
    yc = v.vortex_centers[:, 1].reshape(n, nc).mean(1)
    zc = v.vortex_centers[:, 2].reshape(n, nc).mean(1)
    A = v.areas.reshape(n, nc).sum(1)
    Ls = L.reshape(n, nc).sum(1)
    w = np.abs(v.front_right_vertices[:, 1] - v.front_left_vertices[:, 1]).reshape(n, nc)[:, 0]
    q = op.dynamic_pressure()
    keep = (np.abs(zc) < 0.02) & (yc > 0)   # right wing, flat part only
    o = np.argsort(yc[keep])
    return yc[keep][o], (A / np.maximum(w, 1e-9))[keep][o], (Ls / (q * A))[keep][o]


def vlm_block(g: Geometry):
    pol = []
    for a in ALPHAS:
        r = vlm_run(g, a)
        pol.append(dict(alpha=a, CL=r["CL"], Cm=r["Cm"], CDi=r["CD"]))
    pol = pd.DataFrame(pol)
    m = (pol.alpha >= 0) & (pol.alpha <= 8)
    CLa = np.polyfit(np.radians(pol.alpha[m]), pol.CL[m], 1)[0]
    Cma = np.polyfit(np.radians(pol.alpha[m]), pol.Cm[m], 1)[0]
    CL0, Cm0 = float(np.interp(0, pol.alpha, pol.CL)), float(np.interp(0, pol.alpha, pol.Cm))
    # derivatives at the AVL trim alpha by central differences
    a0 = A_TRIM_REF
    h = 1.0
    rp, rm = vlm_run(g, a0, de=h), vlm_run(g, a0, de=-h)
    CLde, Cmde = (rp["CL"] - rm["CL"]) / (2 * h) * 180 / math.pi, (rp["Cm"] - rm["Cm"]) / (2 * h) * 180 / math.pi
    ap, am = vlm_run(g, a0, da=h), vlm_run(g, a0, da=-h)
    Clda_b, Cnda_b = (ap["Cl"] - am["Cl"]) / (2 * h) * 180 / math.pi, (ap["Cn"] - am["Cn"]) / (2 * h) * 180 / math.pi
    Clda, Cnda = stab_axes(Clda_b, Cnda_b, a0)
    bp, bm = vlm_run(g, a0, beta=h), vlm_run(g, a0, beta=-h)
    Clb_b, Cnb_b = (bp["Cl"] - bm["Cl"]) / (2 * h) * 180 / math.pi, (bp["Cn"] - bm["Cn"]) / (2 * h) * 180 / math.pi
    Clb, Cnb = stab_axes(Clb_b, Cnb_b, a0)
    # roll damping: ASB stability-derivative routine (rates non-dimensionalised p b / 2V)
    op = asb.OperatingPoint(atmosphere=ATMO, velocity=V_CRUISE, alpha=a0)
    sd = asb.VortexLatticeMethod(vlm_airplane(g), op, **VLM_RES).run_with_stability_derivatives(
        alpha=True, beta=False, p=True, q=False, r=True)
    Clp, _ = stab_axes(sd["Clp"], sd["Cnp"], a0)
    _, Cnr = stab_axes(sd["Clr"], sd["Cnr"], a0)
    r0 = vlm_run(g, a0)
    e = r0["CL"] ** 2 / (math.pi * AR * r0["CD"])
    a_t, de_t = solve_trim(CL0, CLa, CLde, Cm0, Cma, Cmde)
    out = dict(CLa=CLa, CL0=CL0, Cm0=Cm0, Cma=Cma, x_np=float(sd["x_np"]), e=e, alpha_trim=a_t, de_trim=de_t,
               CLde=CLde, Cmde=Cmde, Cnb=Cnb, Clb=Clb, Clp=Clp, Cnr=Cnr, Clda=Clda, Cnda=Cnda)
    out["SM"] = (out["x_np"] - X_CG) / MAC
    return out, pol


def solve_trim(CL0, CLa, CLde, Cm0, Cma, Cmde, CL=None):
    """Linear trim: CL0 + CLa a + CLde d = CL, Cm0 + Cma a + Cmde d = 0 (a, d in rad)."""
    CL = CL_CRUISE if CL is None else CL
    A = np.array([[CLa, CLde], [Cma, Cmde]])
    a, d = np.linalg.solve(A, [CL - CL0, -Cm0])
    return math.degrees(a), math.degrees(d)


# ============================================================================= AeroBuildup
def ab_airplane(g: Geometry, de=0.0, da=0.0, pod=True, nose=None):
    yb, yt = g.y_break, g.span / 2
    cs = [asb.ControlSurface("elevon", True, de, g.hinge_xc), asb.ControlSurface("aileron", False, da, g.hinge_xc)]
    wing = asb.Wing(name="wing", symmetric=True, xsecs=[_wx(g, 0.0, AF), _wx(g, yb, AF, cs), _wx(g, yt, AF)])
    wings = [wing] + ([winglet_wing(g)] if g.winglet_h > 0 else [])
    fus = []
    if pod:
        secs = list(vm.POD_SECTIONS)
        if nose is not None:
            secs[0] = nose
        stations = [(0.0, 0.0, 0.0)] + [(s[0], s[1], s[2]) for s in secs] + [(1.0, 0.0, 0.0)]
        fus.append(asb.Fuselage(name="pod", xsecs=[
            asb.FuselageXSec(xyz_c=[vm.POD_X0 + xl * vm.POD_LEN, 0, vm.POD_Z], width=max(w, 1e-3),
                             height=max(h, 1e-3), shape=2.5) for xl, w, h in stations]))
        gb = vm.GIMBAL
        fus.append(asb.Fuselage(name="gimbal", xsecs=[
            asb.FuselageXSec(xyz_c=[gb["x_nose"] + gb["length"] * t, 0, gb["z"]],
                             radius=max(gb["length"] / 2 * math.sin(math.pi * t), 1e-3)) for t in np.linspace(0, 1, 9)]))
    return asb.Airplane(name="lodestar", xyz_ref=[X_CG, 0, 0], wings=wings, fuselages=fus,
                        s_ref=g.S, c_ref=g.mac, b_ref=g.span)


def ab_run(g, alpha, de=0.0, da=0.0, beta=0.0, pod=True, V=V_CRUISE, nose=None):
    op = asb.OperatingPoint(atmosphere=ATMO, velocity=V, alpha=alpha, beta=beta)
    return asb.AeroBuildup(ab_airplane(g, de, da, pod, nose), op, model_size="large").run()


def f1(x):
    return float(np.ravel(x)[0])


def ab_block(g: Geometry, pod=True):
    al = np.arange(-4.0, 22.01, 1.0)
    rows = []
    for a in al:
        r = ab_run(g, a, pod=pod)
        q = Q_CRUISE
        rows.append(dict(alpha=a, CL=f1(r["CL"]), Cm=f1(r["Cm"]), CD=f1(r["CD"]),
                         CDp=f1(r["D_profile"]) / (q * S), CDi=f1(r["D_induced"]) / (q * S)))
    pol = pd.DataFrame(rows)
    m = (pol.alpha >= 0) & (pol.alpha <= 8)
    CLa = np.polyfit(np.radians(pol.alpha[m]), pol.CL[m], 1)[0]
    Cma = np.polyfit(np.radians(pol.alpha[m]), pol.Cm[m], 1)[0]
    CL0, Cm0 = float(np.interp(0, pol.alpha, pol.CL)), float(np.interp(0, pol.alpha, pol.Cm))
    a0, h = A_TRIM_REF, 1.0
    rp, rm = ab_run(g, a0, de=h, pod=pod), ab_run(g, a0, de=-h, pod=pod)
    CLde = (f1(rp["CL"]) - f1(rm["CL"])) / (2 * h) * 180 / math.pi
    Cmde = (f1(rp["Cm"]) - f1(rm["Cm"])) / (2 * h) * 180 / math.pi
    ap, am = ab_run(g, a0, da=h, pod=pod), ab_run(g, a0, da=-h, pod=pod)
    Clda, Cnda = stab_axes((f1(ap["Cl"]) - f1(am["Cl"])) / (2 * h) * 180 / math.pi,
                           (f1(ap["Cn"]) - f1(am["Cn"])) / (2 * h) * 180 / math.pi, a0)
    op = asb.OperatingPoint(atmosphere=ATMO, velocity=V_CRUISE, alpha=a0)
    sd = asb.AeroBuildup(ab_airplane(g, pod=pod), op, model_size="large").run_with_stability_derivatives(
        alpha=True, beta=True, p=True, q=False, r=True)
    Clb, Cnb = stab_axes(f1(sd["Clb"]), f1(sd["Cnb"]), a0)
    Clp, _ = stab_axes(f1(sd["Clp"]), f1(sd["Cnp"]), a0)
    _, Cnr = stab_axes(f1(sd["Clr"]), f1(sd["Cnr"]), a0)
    a_t, de_t = solve_trim(CL0, CLa, CLde, Cm0, Cma, Cmde)
    rt = ab_run(g, a_t, de=de_t, pod=pod)
    CL_t, CD_t = f1(rt["CL"]), f1(rt["CD"])
    CDi_t = f1(rt["D_induced"]) / (Q_CRUISE * S)
    e = CL_t**2 / (math.pi * AR * CDi_t)
    out = dict(CLa=CLa, CL0=CL0, Cm0=Cm0, Cma=Cma, x_np=f1(sd["x_np"]), e=e, alpha_trim=a_t, de_trim=de_t,
               CLde=CLde, Cmde=Cmde, Cnb=Cnb, Clb=Clb, Clp=Clp, Cnr=Cnr, Clda=Clda, Cnda=Cnda,
               CD_trim=CD_t, LD_trim=CL_t / CD_t)
    out["SM"] = (out["x_np"] - X_CG) / MAC
    return out, pol


# ============================================================================= VSPAERO (VLM mode)
VSP_DIR = OUT / "vsp"


def vsp_script(with_pod: bool, alphas, unsteady: int, betas=(0.0, 0.0, 1)):
    s = vm.vspscript(with_pod, "lodestar", (0, 1, 2), False, 0)
    s = s[:s.index("//==== VSPAERO")]
    s += "    SetSetFlag( w, 3, true );\n"
    if with_pod:
        s += "    SetSetFlag( f, 4, true );\n    SetSetFlag( gb, 4, true );\n"
    thick = "4" if with_pod else "SET_NONE"
    a0, a1, n = alphas
    s += f'''    Update();
    SetAnalysisInputDefaults( "VSPAEROComputeGeometry" );
    SetIntAnalysisInput( "VSPAEROComputeGeometry", "GeomSet", array<int> = {{ {thick} }} );
    SetIntAnalysisInput( "VSPAEROComputeGeometry", "ThinGeomSet", array<int> = {{ 3 }} );
    SetIntAnalysisInput( "VSPAEROComputeGeometry", "Symmetry", array<int> = {{ 0 }} );
    ExecAnalysis( "VSPAEROComputeGeometry" );
    string a = "VSPAEROSweep";
    SetAnalysisInputDefaults( a );
    SetIntAnalysisInput( a, "GeomSet", array<int> = {{ {thick} }} );
    SetIntAnalysisInput( a, "ThinGeomSet", array<int> = {{ 3 }} );
    SetIntAnalysisInput( a, "Symmetry", array<int> = {{ 0 }} );
    SetIntAnalysisInput( a, "RefFlag", array<int> = {{ 0 }} );
    SetDoubleAnalysisInput( a, "Sref", array<double> = {{ {S:.5f} }} );
    SetDoubleAnalysisInput( a, "cref", array<double> = {{ {MAC:.5f} }} );
    SetDoubleAnalysisInput( a, "bref", array<double> = {{ {B:.4f} }} );
    SetDoubleAnalysisInput( a, "Xcg", array<double> = {{ {X_CG:.4f} }} );
    SetDoubleAnalysisInput( a, "AlphaStart", array<double> = {{ {a0} }} );
    SetDoubleAnalysisInput( a, "AlphaEnd", array<double> = {{ {a1} }} );
    SetIntAnalysisInput( a, "AlphaNpts", array<int> = {{ {n} }} );
    SetDoubleAnalysisInput( a, "BetaStart", array<double> = {{ {betas[0]} }} );
    SetDoubleAnalysisInput( a, "BetaEnd", array<double> = {{ {betas[1]} }} );
    SetIntAnalysisInput( a, "BetaNpts", array<int> = {{ {betas[2]} }} );
    SetDoubleAnalysisInput( a, "MachStart", array<double> = {{ {V_CRUISE / float(ATMO.speed_of_sound()):.4f} }} );
    SetIntAnalysisInput( a, "MachNpts", array<int> = {{ 1 }} );
    SetDoubleAnalysisInput( a, "ReCref", array<double> = {{ {RHO * V_CRUISE * MAC / MU:.0f} }} );
    SetDoubleAnalysisInput( a, "Vinf", array<double> = {{ {V_CRUISE} }} );
    SetDoubleAnalysisInput( a, "Vref", array<double> = {{ {V_CRUISE} }} );
    SetDoubleAnalysisInput( a, "Rho", array<double> = {{ {RHO:.4f} }} );
    SetIntAnalysisInput( a, "UnsteadyType", array<int> = {{ {unsteady} }} );
    SetIntAnalysisInput( a, "WakeNumIter", array<int> = {{ 5 }} );
    SetIntAnalysisInput( a, "NCPU", array<int> = {{ 8 }} );
    ExecAnalysis( a );
    Print( "VSPAERO DONE" );
    while ( GetNumTotalErrors() > 0 ) {{ ErrorObj err = PopLastError(); Print( "VSPERROR: " + err.GetErrorString() ); }}
}}
'''
    return s


def vsp_run(name, with_pod, alphas, unsteady, betas=(0.0, 0.0, 1)):
    d = VSP_DIR / name
    d.mkdir(parents=True, exist_ok=True)
    vm.write_airfoil(d / "mh45.dat")
    (d / "run.vspscript").write_text(vsp_script(with_pod, alphas, unsteady, betas))
    r = subprocess.run([str(vm.VSPSCRIPT), "-script", "run.vspscript"], cwd=d, capture_output=True, text=True,
                       timeout=3600)
    (d / "stdout.txt").write_text(r.stdout + "\n" + r.stderr)
    return d


def read_stab(path: Path):
    """Parse the derivative table of a VSPAERO .stab file -> {coef: {col: value}}."""
    lines = path.read_text().splitlines()
    cols = ["Total", "Alpha", "Beta", "p", "q", "r", "Mach", "U"]
    out, res = {}, {}
    for l in lines:
        p = l.split()
        if len(p) == 9 and p[0] in ("CL", "CD", "CS", "CMl", "CMm", "CMn", "CFy"):
            try:
                out[p[0]] = dict(zip(cols, map(float, p[1:])))
            except ValueError:
                pass
        if len(p) >= 2 and p[0] in ("SM", "X_np"):
            res[p[0]] = float(p[1])
    return out, res


def vsp_block(with_pod: bool, run: bool):
    """Longitudinal derivatives and x_np from the alpha polar (Cm-alpha / CL-alpha); Cl_beta and Cn_beta
    from a +-2 deg beta sweep at the trim alpha (polar moments are in VSPAERO geometry axes, so
    Cl = -CMx, Cn = -CMz); roll/yaw damping from the stability run, used only if that run's own
    Cm_alpha agrees with the polar within 20% (its 0.01-deg finite differences were noisy)."""
    tag = "pod" if with_pod else "wing"
    a0 = round(A_TRIM_REF, 2)
    if run:
        vsp_run(f"{tag}_polar", with_pod, (-4.0, 12.0, 9), 0)
        vsp_run(f"{tag}_beta", with_pod, (a0, a0, 1), 0, betas=(-2.0, 2.0, 3))
        vsp_run(f"{tag}_stab", with_pod, (a0, a0, 1), 1)
    raw = vm.read_polar(VSP_DIR / f"{tag}_polar")
    pol = pd.DataFrame(dict(alpha=raw.AoA, CL=raw.CLtot, Cm=raw.CMytot, CDi=raw.CDiw, CDtot=raw.CDtot, Ew=raw.Ew))
    m = (pol.alpha >= 0) & (pol.alpha <= 8)
    CLa = np.polyfit(np.radians(pol.alpha[m]), pol.CL[m], 1)[0]
    Cma = np.polyfit(np.radians(pol.alpha[m]), pol.Cm[m], 1)[0]
    bt = vm.read_polar(VSP_DIR / f"{tag}_beta")
    kb = np.polyfit(np.radians(bt.Beta), -bt.CMxtot, 1)[0], np.polyfit(np.radians(bt.Beta), -bt.CMztot, 1)[0]
    Clb, Cnb = stab_axes(kb[0], kb[1], a0)
    d, res = read_stab(next((VSP_DIR / f"{tag}_stab").glob("*.stab")))
    ok = abs(d["CMm"]["Alpha"] / Cma - 1) < 0.2
    Clp, _ = stab_axes(d["CMl"]["p"], d["CMn"]["p"], a0)
    _, Cnr = stab_axes(d["CMl"]["r"], d["CMn"]["r"], a0)
    if not ok:
        print(f"  VSPAERO {tag}: stability-run Cm_alpha {d['CMm']['Alpha']:.3f} vs polar {Cma:.3f}; rate derivatives not used")
        Clp, Cnr = np.nan, np.nan
    e = float(np.interp(a0, pol.alpha, pol.Ew))
    x_np = X_CG - Cma / CLa * MAC
    out = dict(CLa=CLa, CL0=float(np.interp(0, pol.alpha, pol.CL)), Cm0=float(np.interp(0, pol.alpha, pol.Cm)),
               Cma=Cma, x_np=x_np, e=e, Cnb=Cnb, Clb=Clb, Clp=Clp, Cnr=Cnr,
               x_np_stabfile=res.get("X_np", np.nan), Cma_stabfile=d["CMm"]["Alpha"])
    out["SM"] = (out["x_np"] - X_CG) / MAC
    return out, pol


# ============================================================================= stall / CLmax
def clmax_table(Res, defl):
    """NeuralFoil MH45 clmax over Re and elevon deflection (deg, TE down positive)."""
    tab = np.zeros((len(defl), len(Res)))
    al = np.linspace(-2, 20, 89)
    for i, dd in enumerate(defl):
        af = AF if dd == 0 else AF.add_control_surface(deflection=dd, hinge_point_x=0.75)
        for j, Re in enumerate(Res):
            a = af.get_aero_from_neuralfoil(alpha=al, Re=Re, mach=0.04, model_size="large")
            tab[i, j] = float(np.max(a["CL"]))
    return tab


def interp2(tab, defl, Res, d, Re):
    col = np.array([np.interp(Re, Res, tab[i]) for i in range(len(defl))])
    return float(np.interp(d, defl, col))


# ============================================================================= main
if __name__ == "__main__":
    sys.stdout = Tee(OUT / "run_log.txt")
    run_vsp = "--no-vsp" not in sys.argv
    print("Lodestar aerodynamic cross-check, 2026-11-14")
    print(f"S {S:.4f} m2, MAC {MAC:.4f} m, b {B} m, AR {AR:.2f}; CG x = {X_CG} m")
    print(f"Cruise: {MASS} kg, {V_CRUISE} m/s, {ALT:.0f} m (rho {RHO:.4f}), q {Q_CRUISE:.1f} Pa, CL {CL_CRUISE:.4f}\n")

    # ---------------- AVL
    print("AVL (pod as slender body) ...")
    avl_pod, pol_avl_pod = avl_block(G_POD, "pod")
    A_TRIM_REF = avl_pod["alpha_trim"]
    print("AVL (no pod) ...")
    avl_np, pol_avl_np = avl_block(G_NOPOD, "nopod")

    # ---------------- AeroSandbox VLM
    print("AeroSandbox VLM (wing + winglets) ...")
    vlm, pol_vlm = vlm_block(G_NOPOD)
    # resolution check
    VLM_RES_SAVE = dict(VLM_RES)
    conv = []
    for sr, cr in ((8, 8), (12, 10), (20, 12), (28, 14)):
        VLM_RES.update(spanwise_resolution=sr, chordwise_resolution=cr)
        op = asb.OperatingPoint(atmosphere=ATMO, velocity=V_CRUISE, alpha=A_TRIM_REF)
        sd = asb.VortexLatticeMethod(vlm_airplane(G_NOPOD), op, **VLM_RES).run_with_stability_derivatives(
            alpha=True, beta=False, p=False, q=False, r=False)
        conv.append(dict(spanwise=sr, chordwise=cr, CLa=float(sd["CLa"]), x_np=float(sd["x_np"])))
    VLM_RES.update(VLM_RES_SAVE)
    conv = pd.DataFrame(conv)
    conv.round(5).to_csv(OUT / "vlm_resolution.csv", index=False)
    print(conv.round(4).to_string(index=False))

    # isolate the winglet modelling: plain wing, no winglets, no pod
    g_bare = replace(G_NOPOD, winglet_h=0.0)
    st_bare = avl_sweep(g_bare, [A_TRIM_REF], "bare")[0][0]
    op = asb.OperatingPoint(atmosphere=ATMO, velocity=V_CRUISE, alpha=A_TRIM_REF)
    sd_bare = asb.VortexLatticeMethod(vlm_airplane(g_bare), op, **VLM_RES).run_with_stability_derivatives(
        alpha=True, beta=False, p=False, q=False, r=False)
    print(f"Bare wing (no winglets, no pod) at alpha {A_TRIM_REF:.2f}: x_np AVL {st_bare['Xnp']:.4f} m, "
          f"ASB VLM {float(sd_bare['x_np']):.4f} m; CLa AVL {st_bare['CLa']:.3f}, ASB {float(sd_bare['CLa']):.3f} /rad")
    pd.Series(dict(x_np_avl=st_bare["Xnp"], x_np_vlm=float(sd_bare["x_np"]), CLa_avl=st_bare["CLa"],
                   CLa_vlm=float(sd_bare["CLa"]))).to_csv(OUT / "bare_wing_check.csv", header=["value"])

    # ---------------- AeroBuildup
    print("AeroBuildup (pod + gimbal) ...")
    ab, pol_ab = ab_block(G_POD, pod=True)
    print("AeroBuildup (no pod) ...")
    ab_np, pol_ab_np = ab_block(G_NOPOD, pod=False)

    # ---------------- VSPAERO
    print(f"VSPAERO VLM ({'running' if run_vsp else 'cached'}) ...")
    vsp, pol_vsp = vsp_block(False, run_vsp)
    try:
        vspp, pol_vspp = vsp_block(True, run_vsp)
    except Exception as ex:   # noqa: BLE001
        print("  VSPAERO with pod failed:", ex)
        vspp, pol_vspp = None, None

    # ---------------- Tables 1 and 2
    cols = {"AVL, pod": avl_pod, "AVL, no pod": avl_np, "ASB VLM, no pod": vlm, "VSPAERO VLM, no pod": vsp,
            "AeroBuildup, pod": ab, "AeroBuildup, no pod": ab_np}
    if vspp is not None:
        cols["VSPAERO VLM, pod"] = vspp
    ref_for = {"ASB VLM, no pod": "AVL, no pod", "VSPAERO VLM, no pod": "AVL, no pod",
               "AeroBuildup, no pod": "AVL, no pod", "AeroBuildup, pod": "AVL, pod", "VSPAERO VLM, pod": "AVL, pod"}
    keys = ["CLa", "CL0", "Cm0", "Cma", "x_np", "SM", "e", "alpha_trim", "de_trim", "CLde", "Cmde",
            "Cnb", "Clb", "Clp", "Cnr", "Clda", "Cnda"]
    T = pd.DataFrame({k: {q: c.get(q, np.nan) for q in keys} for k, c in cols.items()})
    T.to_csv(OUT / "table_solvers.csv")
    D = pd.DataFrame({f"{k} vs {ref_for[k]} [%]": (T[k] - T[ref_for[k]]) / T[ref_for[k]].abs() * 100
                      for k in cols if k in ref_for})
    D.loc["x_np"] = [(T.loc["x_np", k] - T.loc["x_np", ref_for[k]]) / MAC * 100 for k in cols if k in ref_for]
    D.loc["SM"] = [(T.loc["SM", k] - T.loc["SM", ref_for[k]]) * 100 for k in cols if k in ref_for]
    for q_ in ("alpha_trim", "de_trim"):   # degrees, absolute difference (de_trim is near zero)
        D.loc[q_] = [T.loc[q_, k] - T.loc[q_, ref_for[k]] for k in cols if k in ref_for]
    D.to_csv(OUT / "table_differences.csv")
    pd.set_option("display.width", 250)
    print("\nSolver comparison (moments about x = 0.422 m; derivatives per rad; lateral in stability axes):")
    print(T.round(4).to_string())
    print("\nDifferences vs the matching AVL case (% of AVL value; x_np and SM in % MAC points):")
    print(D.round(1).to_string())
    pd.concat([pol_avl_pod.assign(solver="AVL, pod"), pol_avl_np.assign(solver="AVL, no pod"),
               pol_vlm.assign(solver="ASB VLM, no pod"), pol_vsp.assign(solver="VSPAERO VLM, no pod"),
               pol_ab.assign(solver="AeroBuildup, pod"), pol_ab_np.assign(solver="AeroBuildup, no pod")]
              + ([pol_vspp.assign(solver="VSPAERO VLM, pod")] if pol_vspp is not None else [])
              ).to_csv(OUT / "polars_by_solver.csv", index=False)

    # ---------------- drag, L/D (AeroBuildup with pod)
    q = Q_CRUISE
    r0 = ab_run(G_POD, ab["alpha_trim"], de=ab["de_trim"])
    comp = []
    for c in r0["wing_aero_components"]:
        comp.append(dict(part="wing" if len(comp) == 0 else "winglets", CD=f1(c.D) / (q * S)))
    for i, c in enumerate(r0["fuselage_aero_components"]):
        comp.append(dict(part=["pod", "gimbal"][i], CD=f1(c.D) / (q * S)))
    comp = pd.DataFrame(comp)
    CDi_cruise = f1(r0["D_induced"]) / (q * S)
    CDp_cruise = f1(r0["D_profile"]) / (q * S)
    # zero-lift drag: profile drag at the zero-lift alpha
    a_zl = float(np.interp(0.0, pol_ab.CL, pol_ab.alpha))
    rz = ab_run(G_POD, a_zl)
    CD0 = f1(rz["CD"])
    # L/D vs speed, trimmed with the AeroBuildup linear trim model, 8.2 kg
    W = MASS * G0
    ld = []
    for V in np.arange(12.0, 28.01, 1.0):
        qv = 0.5 * RHO * V**2
        CLr = W / (qv * S)
        a_t, d_t = solve_trim(ab["CL0"], ab["CLa"], ab["CLde"], ab["Cm0"], ab["Cma"], ab["Cmde"], CL=CLr)
        rv = ab_run(G_POD, a_t, de=d_t, V=V)
        ld.append(dict(V=V, CL=f1(rv["CL"]), CD=f1(rv["CD"]), LD=f1(rv["CL"]) / f1(rv["CD"]), alpha=a_t, de=d_t))
    ld = pd.DataFrame(ld)
    ld.round(4).to_csv(OUT / "ld_vs_speed_aerobuildup.csv", index=False)
    i_max = int(ld.LD.idxmax())
    print(f"\nAeroBuildup drag at cruise trim (alpha {ab['alpha_trim']:.2f} deg, elevon {ab['de_trim']:+.2f} deg): "
          f"CD {f1(r0['CD']):.4f} = profile {CDp_cruise:.4f} + induced {CDi_cruise:.4f}")
    print(comp.round(5).to_string(index=False))
    print(f"CD0 (at zero lift, alpha {a_zl:.2f} deg) = {CD0:.4f}; trimmed L/D max {ld.LD.max():.1f} at {ld.V[i_max]:.0f} m/s; "
          f"L/D at 19 m/s {float(np.interp(19, ld.V, ld.LD)):.1f}")
    vsp_ld = (pol_vsp.CL / pol_vsp.CDtot)
    print(f"VSPAERO VLM (its own CDo estimate) L/D max {vsp_ld.max():.1f}")
    pd.Series(dict(CD0=CD0, CD_cruise=f1(r0["CD"]), CDp_cruise=CDp_cruise, CDi_cruise=CDi_cruise,
                   LDmax=ld.LD.max(), V_LDmax=ld.V[i_max], LD_19=float(np.interp(19, ld.V, ld.LD)),
                   **{f"CD_{p}": v for p, v in zip(comp.part, comp.CD)})).to_csv(OUT / "drag_aerobuildup.csv", header=["value"])

    # ---------------- CLmax and stall onset
    print("\nCLmax and stall onset ...")
    V_s08 = math.sqrt(2 * W / (RHO * S * CLMAX_SIZING))
    Res = np.array([1.2e5, 1.8e5, 2.5e5, 3.5e5, 4.5e5, 6.0e5])
    defl = np.array([-10.0, -6.0, -3.0, 0.0, 3.0])
    tab = clmax_table(Res, defl)
    pd.DataFrame(tab, index=defl, columns=Res).to_csv(OUT / "mh45_clmax_neuralfoil.csv")
    al_st = np.arange(4.0, 20.01, 1.0)
    sw = avl_sweep(G_POD, al_st, "stall_trimmed", trim_pitch=True)
    rows, strip_rows = [], []
    for a, (st, (y, c, cl)) in zip(al_st, sw):
        de = st["elevon"]
        CL = st["CLtot"]
        V_here = math.sqrt(2 * W / (RHO * S * max(CL, 0.1)))
        Re = RHO * V_here * c / MU
        clm = np.array([interp2(tab, defl, Res, de if yy > G_POD.y_break else 0.0, R) for yy, R in zip(y, Re)])
        ratio = cl / clm
        k = int(np.argmax(ratio))
        rows.append(dict(alpha=a, CL=CL, elevon=de, V=V_here, max_ratio=ratio[k], y_crit=y[k]))
        for yy, cc, cl_, cm_ in zip(y, c, cl, clm):
            strip_rows.append(dict(alpha=a, CL=CL, elevon=de, y=yy, chord=cc, cl=cl_, clmax=cm_))
    stall = pd.DataFrame(rows)
    strips = pd.DataFrame(strip_rows)
    stall.round(4).to_csv(OUT / "stall_onset_avl_trimmed.csv", index=False)
    strips.round(4).to_csv(OUT / "spanwise_cl_avl_trimmed.csv", index=False)
    j = int(np.argmax(stall.max_ratio.values >= 1.0)) if (stall.max_ratio >= 1.0).any() else len(stall) - 1
    # interpolate the CL where the first strip reaches its clmax
    if j > 0 and stall.max_ratio[j] >= 1.0:
        f = (1 - stall.max_ratio[j - 1]) / (stall.max_ratio[j] - stall.max_ratio[j - 1])
        CL_cs = stall.CL[j - 1] + f * (stall.CL[j] - stall.CL[j - 1])
        a_cs = stall.alpha[j - 1] + f * (stall.alpha[j] - stall.alpha[j - 1])
        de_cs = stall.elevon[j - 1] + f * (stall.elevon[j] - stall.elevon[j - 1])
    else:
        CL_cs, a_cs, de_cs = np.nan, np.nan, np.nan
    y_cs = stall.y_crit[j]
    print(stall.round(3).to_string(index=False))
    print(f"Critical-section (AVL, trimmed): first strip reaches clmax at alpha {a_cs:.1f} deg, CL {CL_cs:.3f}, "
          f"elevon {de_cs:+.1f} deg, at y = {y_cs:.2f} m ({y_cs / (B / 2):.0%} semispan)")

    # same check with the elevons held at zero (untrimmed), AVL and ASB VLM
    sw0 = avl_sweep(G_POD, al_st, "stall_fixed", de=0.0)
    vlm_rows = []
    cs0 = {}
    for name, getter in (("AVL", None), ("ASB VLM", None)):
        prev = None
        for a_i, a in enumerate(al_st):
            if name == "AVL":
                st, (y, c, cl) = sw0[a_i]
                CL = st["CLtot"]
            else:
                r, v = vlm_run(G_NOPOD, a, keep=True)
                y, c, cl = vlm_strips(v)
                CL = r["CL"]
            V_here = math.sqrt(2 * W / (RHO * S * max(CL, 0.1)))
            clm = np.array([interp2(tab, defl, Res, 0.0, RHO * V_here * cc / MU) for cc in c])
            ratio = cl / clm
            k = int(np.argmax(ratio))
            vlm_rows.append(dict(solver=name, alpha=a, CL=CL, max_ratio=ratio[k], y_crit=y[k]))
            if name == "ASB VLM":
                for yy, cl_, cm_ in zip(y, cl, clm):
                    strip_rows.append(dict(solver="ASB VLM", alpha=a, CL=CL, elevon=0.0, y=yy, cl=cl_, clmax=cm_))
            if prev is not None and prev[1] < 1.0 <= ratio[k] and name not in cs0:
                f = (1 - prev[1]) / (ratio[k] - prev[1])
                cs0[name] = (prev[0] + f * (CL - prev[0]), y[k])
            prev = (CL, ratio[k])
    fixed = pd.DataFrame(vlm_rows)
    fixed.round(4).to_csv(OUT / "stall_onset_elevon_zero.csv", index=False)
    pd.DataFrame(strip_rows).round(4).to_csv(OUT / "spanwise_cl_all.csv", index=False)
    for k_, (clv, yv) in cs0.items():
        print(f"Critical-section ({k_}, elevon 0): CL {clv:.3f}, first at y = {yv:.2f} m ({yv / (B / 2):.0%} semispan)")
    # AeroBuildup's own CL peak (NeuralFoil post-stall, crude)
    CLmax_ab = pol_ab.CL.max()
    a_ab = float(pol_ab.alpha[pol_ab.CL.idxmax()])
    # Raymer: CLmax ~ 0.9 clmax cos(sweep_qc) for a clean, moderate-AR wing
    clmax_mid = interp2(tab, defl, Res, 0.0, RHO * V_s08 * MAC / MU)
    CLmax_raymer = 0.9 * clmax_mid * math.cos(math.radians(G_POD.sweep_qc_deg))
    print(f"AeroBuildup peak CL {CLmax_ab:.3f} at alpha {a_ab:.0f} deg (untrimmed, pod)")
    print(f"Raymer 0.9 clmax cos(sweep): MH45 clmax {clmax_mid:.3f} at Re(MAC) {RHO * V_s08 * MAC / MU:,.0f} -> {CLmax_raymer:.3f}")
    pd.Series(dict(CL_crit_section_trimmed=CL_cs, alpha_crit_section=a_cs, de_crit_section=de_cs, y_first=y_cs,
                   CL_crit_avl_fixed=cs0.get("AVL", (np.nan,))[0], CL_crit_vlm_fixed=cs0.get("ASB VLM", (np.nan,))[0],
                   y_first_vlm_fixed=cs0.get("ASB VLM", (np.nan, np.nan))[1],
                   CLmax_aerobuildup=CLmax_ab, alpha_CLmax_aerobuildup=a_ab, CLmax_raymer=CLmax_raymer,
                   clmax_2d_mac=clmax_mid, V_stall_CL08=V_s08)).to_csv(OUT / "clmax_summary.csv", header=["value"])

    # washout sensitivity of the stall onset point
    wo = []
    for wdeg in (0.0, 3.0, 5.0):
        gw = replace(G_POD, washout_deg=wdeg)
        sww = avl_sweep(gw, al_st, f"stall_wo{wdeg:.0f}", trim_pitch=True)
        prev = None
        for a, (st, (y, c, cl)) in zip(al_st, sww):
            CL = st["CLtot"]
            V_here = math.sqrt(2 * W / (RHO * S * max(CL, 0.1)))
            clm = np.array([interp2(tab, defl, Res, st["elevon"] if yy > gw.y_break else 0.0, RHO * V_here * cc / MU)
                            for yy, cc in zip(y, c)])
            ratio = cl / clm
            k = int(np.argmax(ratio))
            if prev is not None and prev[1] < 1.0 <= ratio[k]:
                f = (1 - prev[1]) / (ratio[k] - prev[1])
                wo.append(dict(washout=wdeg, CL_first_stall=prev[0] + f * (CL - prev[0]), y_first=y[k],
                               elevon=st["elevon"]))
                break
            prev = (CL, ratio[k])
    wo = pd.DataFrame(wo)
    wo.round(4).to_csv(OUT / "washout_stall_onset.csv", index=False)
    print("\nWashout vs stall onset (AVL, trimmed, pod):")
    print(wo.round(3).to_string(index=False))

    # ---------------- Skywalker X8 comparison
    def helmbold(A, sweep_half_deg, eta=0.95):
        t = math.tan(math.radians(sweep_half_deg))
        return 2 * math.pi * A / (2 + math.sqrt(4 + A**2 / eta**2 * (1 + t**2)))

    # Lodestar half-chord sweep from the quarter-chord sweep and taper
    lam = G_POD.c_tip / G_POD.c_root
    t_hc = math.tan(math.radians(G_POD.sweep_qc_deg)) - 4 / AR * (0.25 * (1 - lam) / (1 + lam))
    sw_hc = math.degrees(math.atan(t_hc))
    x8 = dict(AR=2.10**2 / 0.75, CLa=2.573, Cma=-0.274, Cm0=0.027, CL0=-0.077, CD0=0.058, CLde=1.369, Cmde=-0.276)
    rows = []
    rows.append(dict(item="Aspect ratio", lodestar=AR, x8=x8["AR"]))
    rows.append(dict(item="CL_alpha [/rad]", lodestar=avl_pod["CLa"], x8=x8["CLa"]))
    rows.append(dict(item="CL_alpha / Helmbold (Lodestar sweep_c/2; X8 at 20-35 deg)",
                     lodestar=avl_pod["CLa"] / helmbold(AR, sw_hc),
                     x8=f"{x8['CLa'] / helmbold(x8['AR'], 20):.2f}-{x8['CLa'] / helmbold(x8['AR'], 35):.2f}"))
    rows.append(dict(item="Static margin [% MAC] = -Cma/CLa", lodestar=-avl_pod["Cma"] / avl_pod["CLa"] * 100,
                     x8=-x8["Cma"] / x8["CLa"] * 100))
    rows.append(dict(item="Cm0 (about CG)", lodestar=avl_pod["Cm0"], x8=x8["Cm0"]))
    rows.append(dict(item="CL0", lodestar=avl_pod["CL0"], x8=x8["CL0"]))
    rows.append(dict(item="Cm_de / CL_de [MAC]", lodestar=avl_pod["Cmde"] / avl_pod["CLde"], x8=x8["Cmde"] / x8["CLde"]))
    rows.append(dict(item="CL_de [/rad]", lodestar=avl_pod["CLde"], x8=x8["CLde"]))
    rows.append(dict(item="CD0", lodestar=CD0, x8=x8["CD0"]))
    x8t = pd.DataFrame(rows)
    x8t.to_csv(OUT / "skywalker_x8_comparison.csv", index=False)
    print(f"\nHelmbold: Lodestar sweep_c/2 = {sw_hc:.1f} deg -> {helmbold(AR, sw_hc):.3f}/rad; "
          f"X8 AR {x8['AR']:.2f} at 20/35 deg -> {helmbold(x8['AR'], 20):.3f}/{helmbold(x8['AR'], 35):.3f}/rad")
    print(x8t.to_string(index=False))

    # ---------------- Styling options (AVL with pod, ASB VLM without; AeroBuildup for drag)
    print("\nStyling options ...")
    import stability as stb  # noqa: E402  (mass model and mode classification; analyse() runs AVL with modes)
    variants = [
        ("baseline: 0.25 m, 35 deg sweep, taper 0.6, vertical", {}),
        ("winglet cant 15 deg outward", dict(winglet_cant_deg=15.0)),
        ("winglet cant 15 deg inward", dict(winglet_cant_deg=-15.0)),
        ("winglet sweep 45 deg", dict(winglet_sweep_deg=45.0)),
        ("winglet sweep 50 deg, taper 0.4 (shark fin)", dict(winglet_sweep_deg=50.0, winglet_taper=0.4)),
        ("winglet sweep 50 deg, taper 0.4, height 0.28 m", dict(winglet_sweep_deg=50.0, winglet_taper=0.4, winglet_h=0.28)),
        ("winglet taper 0.4", dict(winglet_taper=0.4)),
    ]
    sty = []
    for name, kw in variants:
        g = replace(G_POD, **kw)
        r = stb.analyse(g, X_CG, V=V_CRUISE, kit_on=True)
        gn = replace(G_NOPOD, **kw)
        rb, rbm = vlm_run(gn, A_TRIM_REF, beta=1.0), vlm_run(gn, A_TRIM_REF, beta=-1.0)
        _, cnb_v = stab_axes((rb["Cl"] - rbm["Cl"]) / 2 * 180 / math.pi, (rb["Cn"] - rbm["Cn"]) / 2 * 180 / math.pi, A_TRIM_REF)
        ra = ab_run(g, A_TRIM_REF)
        sty.append(dict(variant=name, x_np=r["x_np"], SM=r["static_margin"], elevon=r["elevon"], e=r["e"],
                        Cnb_avl=r["Cnb"], Cnb_vlm=cnb_v, Clb_avl=r["Clb"], dutch_roll_zeta=r.get("dutch_roll_zeta"),
                        spiral_t2=r.get("spiral_t2"), lateral_ok=r.get("lateral_ok"), CD_ab=f1(ra["CD"])))
    # pod nose: blunter vs. pointier first station (AeroBuildup only; AVL equivalent body)
    for name, nose in (("pod nose pointier (first station 0.12 L, 70 x 60 mm)", (0.12, 0.07, 0.06, 0.03)),
                       ("pod nose blunter (first station 0.05 L, 110 x 100 mm)", (0.05, 0.11, 0.10, 0.04))):
        secs = list(vm.POD_SECTIONS)
        secs[0] = nose
        xr = [(0.0, 0.0)] + [(xl * vm.POD_LEN, math.sqrt((w * h - (4 - math.pi) * rc**2) / math.pi)) for xl, w, h, rc in secs] + [(vm.POD_LEN, 0.0)]
        g = replace(G_POD, body_xr=tuple(xr))
        r = stb.analyse(g, X_CG, V=V_CRUISE, kit_on=True)
        ra = ab_run(G_POD, A_TRIM_REF, nose=nose)
        sty.append(dict(variant=name, x_np=r["x_np"], SM=r["static_margin"], elevon=r["elevon"], e=r["e"],
                        Cnb_avl=r["Cnb"], Cnb_vlm=np.nan, Clb_avl=r["Clb"], dutch_roll_zeta=r.get("dutch_roll_zeta"),
                        spiral_t2=r.get("spiral_t2"), lateral_ok=r.get("lateral_ok"), CD_ab=f1(ra["CD"])))
    sty = pd.DataFrame(sty)
    b0 = sty.iloc[0]
    sty["dx_np_mm"] = (sty.x_np - b0.x_np) * 1000
    sty["dCD_counts"] = (sty.CD_ab - b0.CD_ab) * 1e4
    sty.round(4).to_csv(OUT / "styling_options.csv", index=False)
    print(sty[["variant", "dx_np_mm", "SM", "elevon", "e", "Cnb_avl", "Cnb_vlm", "Clb_avl", "dutch_roll_zeta",
               "spiral_t2", "lateral_ok", "dCD_counts"]].round(4).to_string(index=False))

    # ---------------- Figures
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.9))
    styles = {"AVL, pod": ("k", "-"), "AVL, no pod": ("k", "--"), "ASB VLM, no pod": ("C0", "-"),
              "VSPAERO VLM, no pod": ("C2", "-"), "VSPAERO VLM, pod": ("C2", "--"),
              "AeroBuildup, pod": ("C3", "-"), "AeroBuildup, no pod": ("C3", "--")}
    pols = {"AVL, pod": pol_avl_pod, "AVL, no pod": pol_avl_np, "ASB VLM, no pod": pol_vlm,
            "VSPAERO VLM, no pod": pol_vsp, "AeroBuildup, pod": pol_ab, "AeroBuildup, no pod": pol_ab_np}
    if pol_vspp is not None:
        pols["VSPAERO VLM, pod"] = pol_vspp
    for k, p in pols.items():
        c, ls = styles[k]
        ax[0].plot(p.alpha, p.CL, color=c, ls=ls, lw=1, label=k)
        ax[1].plot(p.CL, p.Cm, color=c, ls=ls, lw=1, label=k)
    ax[0].axhline(CLMAX_SIZING, color="0.5", lw=0.6, ls=":")
    ax[0].text(-3.5, CLMAX_SIZING + 0.02, "CLmax 0.80 (sizing)", fontsize=7, color="0.4")
    ax[0].set_xlabel("angle of attack α [deg]")
    ax[0].set_ylabel("lift coefficient CL [-]")
    ax[1].set_xlabel("lift coefficient CL [-]")
    ax[1].set_ylabel("pitching moment Cm about CG [-]")
    ax[1].axhline(0, color="0.6", lw=0.5)
    ax[1].set_xlim(-0.3, 1.2)
    ax[1].legend(fontsize=6, frameon=False)
    fig.suptitle("Figure 1. CL vs. α and Cm vs. CL, by solver and pod model", fontsize=9)
    fig.text(0.01, 0.005, f"Elevon 0°; 19 m/s, 2,000 m; moment reference x = {X_CG} m; AeroBuildup includes pod + gimbal and NeuralFoil stall.",
             fontsize=6, color="0.35")
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(OUT / "fig1_cl_cm_by_solver.png", dpi=200)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.2, 3.2))
    al_show = [a for a in (8.0, 10.0, 12.0) if a <= a_cs + 1] + ([round(a_cs)] if not np.isnan(a_cs) else [])
    al_show = sorted(set(al_show))
    for i, a in enumerate(al_show):
        s_ = strips[strips.alpha == a]
        if s_.empty:
            continue
        c = f"C{i}"
        ax.plot(s_.y, s_.cl, color=c, lw=1.1, label=f"cl, α {a:.0f}° (CL {s_.CL.iloc[0]:.2f}, elevon {s_.elevon.iloc[0]:+.1f}°)")
        ax.plot(s_.y, s_.clmax, color=c, lw=0.8, ls="--")
    ax.axvline(G_POD.y_break, color="0.6", lw=0.5, ls=":")
    ax.text(G_POD.y_break + 0.01, 0.2, "elevon start", fontsize=6, color="0.4", rotation=90)
    ax.set_xlabel("spanwise station y [m]")
    ax.set_ylabel("section lift coefficient cl [-]")
    ax.set_ylim(0, None)
    ax.legend(fontsize=6, frameon=False, loc="lower left")
    ax.set_title("Figure 2. Section cl (solid) and local MH45 clmax (dashed) vs. span, by α", fontsize=8)
    fig.text(0.01, 0.005, "AVL strip loads, trimmed (elevon to Cm = 0), pod, CG 0.422 m, 8.2 kg, 2,000 m.\n"
             "clmax: NeuralFoil MH45 at the local Re for the speed at that CL, outboard strips with the trim elevon.",
             fontsize=5.5, color="0.35")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(OUT / "fig2_spanwise_cl_vs_clmax.png", dpi=200)
    plt.close(fig)
    print("\nDone. Outputs in", OUT)
