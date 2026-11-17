"""Lodestar v1 wing (wing_trade design "v1"): FE statics, load-case matrix, non-flight cases, joint checks and the margins table.

Called by fea_wing.py after the frequency runs (or alone):
    ../.venv/Scripts/python.exe -X utf8 fea_extra.py               statics + tables + margins (~5 min)
    ../.venv/Scripts/python.exe -X utf8 fea_extra.py --convergence three-mesh check (slow, ~1 h)
All loads are ultimate (limit x 1.5) unless a column says limit. Allowables are design values that are already
knocked down (the structures.py convention); no further factor is applied on the allowable side.
"""
from __future__ import annotations

import math
import sys

import numpy as np
import pandas as pd

import fea_wing as F
import structures as s
import wing_trade as wt

OUT = F.OUT
G0 = s.G0
FOS = s.FOS
JOINT = wt.COMPACT
MB = pd.read_csv(F.INPUTS / "aircraft_mass_balance_v1.csv")
CAD = pd.read_csv(F.ROOT / "out" / "cad_v4" / "mass_cg.csv")   # frozen v1 layout
M_WING_SIDE_ITEMS = ("wing structure", "winglets", "batteries", "pack thermal kit", "elevon mass balance")


def log(*a):
    F.log(*a)


# ============================================================================ allowables (design values)
ALLOW = {
    "cap_comp": (s.CARBON_UD.s_c, "pultruded carbon compression 700 MPa (structures.py, ~0.6x datasheet)", "assumed"),
    "cap_tens": (s.CARBON_UD.s_t, "pultruded carbon tension 1000 MPa (structures.py, ~0.6x datasheet)", "assumed"),
    "cap_wrinkle": (0.5 * (s.CARBON_UD.E * s.ROHA51.E * s.ROHA51.G) ** (1 / 3) * 0.8,
                    "cap face wrinkling on Rohacell 51 web, Hoff 0.5(EEG)^1/3 x 0.8 (structures.py)", "assumed"),
    "skin_wr_EPS": (s.WRINKLE_K * (s.GLASS45.E * s.EPS20.E * s.EPS20.G) ** (1 / 3),
                    "Hoff k=0.3 (E_f E_c G_c)^1/3, EPS 20 E=4 G=2.5 MPa assumed", "needs test"),
    "skin_wr_XPS": (s.WRINKLE_K * (s.GLASS45.E * s.XPS.E * s.XPS.G) ** (1 / 3),
                    "Hoff k=0.3, XPS E=12 G=6 MPa (Styrodur-type datasheet)", "assumed"),
    "skin_shear": (s.GLASS45.tau, "+-45 glass wet layup shear 60 MPa (structures.py)", "assumed"),
    "core_tau_EPS": (s.EPS20.tau, "EPS 100 class shear 0.10 MPa (low end of EN 13163 data)", "needs test"),
    "core_tau_XPS": (0.20e6, "XPS shear 0.20 MPa (low end of Styrodur-type data)", "assumed"),
    "core_comp_EPS": (0.07e6, "EPS 100 CS(10) 0.10 MPa x 0.7 (onset of crushing)", "assumed"),
    "web_gamma": (s.GLASS45.tau / s.GLASS45.G, "glass +-45 shear strain 60/9000 (web q_allow 11.1 N/mm)", "assumed"),
    "tube_bend": (wt.JOINT_ALLOW, "carbon tube flexural 500 MPa (screening value in wing_trade.py)", "needs test"),
    "tube_hoop": (150e6, "roll-wrapped tube transverse/hoop flexure 150 MPa, requires hoop plies", "needs test"),
    "sleeve_bearing": (200e6, "CFRP bearing 200 MPa (knocked-down laminate value)", "assumed"),
    "support_ply": (25e6, "1.5 mm birch-ply/G10 rib bearing 25 MPa", "assumed"),
    "support_roha": (0.9e6 * 0.8, "Rohacell 51 compression 0.9 MPa x 0.8", "assumed"),
    "bond_wrap": (0.8e6 * 0.8, "glass wrap bonded to Rohacell 51, core shear 0.8 MPa x 0.8", "assumed"),
    "key_pin_shear": (60e6, "4 mm pultruded carbon pin, transverse shear 60 MPa", "assumed"),
    "key_bearing": (150e6, "2 mm G10 key plate bearing 150 MPa", "assumed"),
    "nylon_m5": (600.0, "M5 nylon 6/6 bolt, 600 N tension each", "assumed"),
    "dowel_shear": (60e6 * math.pi / 4 * 0.006**2, "6 mm carbon dowel, transverse shear 60 MPa", "assumed"),
    "lid_screw": (150.0, "M3 nylon screw in G10 insert, 150 N shear each", "assumed"),
    "hinge_line": (2.0e3, "fabric/Kevlar hinge 2 N/mm (peel-dominated)", "needs test"),
    "tip_defl": (0.05 * 1.5, "project criterion: limit-load tip deflection <= 5% semispan (75 mm)", "criterion"),
}


def ms(allow, applied):
    return allow / applied - 1 if applied > 0 else np.inf


def status(ms_val, src):
    if ms_val < 0:
        return "NEGATIVE"
    return {"assumed": "OK-assumed allowable", "needs test": "OK-assumed allowable, needs test",
            "criterion": "OK"}.get(src, "OK")


# ============================================================================ 1. load-case matrix
def parse_case(case):
    p = case.split(", ")
    return dict(cell=p[0], altitude_m=int(p[1].split()[0]), kit=p[2], label=", ".join(p[3:-1]), side=p[-1])


def load_case_matrix(loads):
    gust = {}
    for r in MB.itertuples():
        for alt, rho in ((0, s.RHO0), (4400, s.RHO_HI)):
            g = wt.gust_cases(round(r.mass_kg, 3), rho, round(r.cg_m, 5))
            gust[(r.cell, bool(r.kit), alt)] = g.set_index("case")
    tab, _ = F.load_tables()
    nl = loads[~loads.sensitivity]
    gov = {}

    def mark(case, what):
        gov.setdefault(case, set()).add(what)

    for y, g in nl.groupby("y"):
        band = "root" if y < 0.2 else "inboard" if y < 0.45 else "break" if np.isclose(y, 0.45) else "outboard"
        mark(g.loc[g.Mb.idxmax(), "case"], f"+Mb {band} (upper cap/skin compression)")
        mark(g.loc[g.Mb.idxmin(), "case"], f"-Mb {band} (lower cap/skin compression)")
        mark(g.loc[g["T"].abs().idxmax(), "case"], f"torque {band} (skin shear, rear web)")
        mark(g.loc[g.V.abs().idxmax(), "case"], f"shear {band} (spar web)")
    br = nl[np.isclose(nl.y, s.Y_BREAK)]
    mark(br.loc[br.Mb.abs().idxmax(), "case"], "joint tube bending, sleeve line load")
    mark(br.loc[br.T_spar.abs().idxmax(), "case"], "aft key")
    defl = {c: abs(s.tip_deflection(tab, d.sort_values("y").Mb.values)[-1]) for c, d in nl.groupby("case")}
    mark(max(defl, key=defl.get), "tip deflection")
    rows = []
    for case, d in loads.groupby("case", sort=False):
        pc = parse_case(case)
        kit = pc["kit"] == "kit on"
        m = MB[(MB.cell == pc["cell"]) & (MB.kit == kit)].iloc[0]
        g = gust[(pc["cell"], kit, pc["altitude_m"])]
        lab = pc["label"]
        if lab.startswith("roll") or lab.startswith("fixed aileron"):
            n, v_eas = wt.ROLL_N, s.VD
        else:
            base = lab.replace(" uncapped", "")
            n = g.loc[base, "n_formula" if "uncapped" in lab else "n_limit"]
            v_eas = s.VC if "Vc" in base else s.VD
        rho = s.RHO0 if pc["altitude_m"] == 0 else s.RHO_HI
        d = d.sort_values("y")
        rows.append(dict(case=case, **pc, sensitivity=bool(d.sensitivity.iloc[0]), n_limit=n, V_EAS=v_eas,
                         V_TAS=v_eas * math.sqrt(s.RHO0 / rho), mass_kg=m.mass_kg,
                         root_Mb_ult=FOS * d.Mb.iloc[0], break_Mb_ult=FOS * d.Mb[np.isclose(d.y, s.Y_BREAK)].iloc[0],
                         max_abs_T_ult=FOS * d["T"].abs().max(), root_V_ult=FOS * d.V.iloc[0],
                         tip_defl_limit_mm=1e3 * abs(s.tip_deflection(tab, d.Mb.values)[-1]),
                         governs="; ".join(sorted(gov.get(case, [])))))
    lc = pd.DataFrame(rows)
    lc.to_csv(OUT / "load_cases.csv", index=False)
    fam = []
    for lab, g in lc.groupby("label", sort=False):
        fam.append(dict(case=lab, sensitivity=g.sensitivity.iloc[0], n_limit_min=g.n_limit.min(),
                        n_limit_max=g.n_limit.max(), V_EAS=g.V_EAS.iloc[0], V_TAS_max=g.V_TAS.max(),
                        mass_min=g.mass_kg.min(), mass_max=g.mass_kg.max(),
                        root_Mb_ult_min=g.root_Mb_ult.min(), root_Mb_ult_max=g.root_Mb_ult.max(),
                        break_Mb_ult_maxabs=g.break_Mb_ult.abs().max(), max_T_ult=g.max_abs_T_ult.max(),
                        governs="; ".join(sorted({x for v in g.governs for x in v.split("; ") if x}))))
    fam = pd.DataFrame(fam)
    fam.to_csv(OUT / "load_case_families.csv", index=False)
    return lc, fam


# ============================================================================ 2. FE statics and FE-vs-beam
def statics(tab, loads, mesh="medium"):
    g = F.governing_cases(loads)
    cases = {}
    for k, c in g.items():
        if c not in cases.values():
            cases[k] = c
    log("FE static cases:", cases)
    w, out, stiff = F.run_static(tab, loads, mesh, cases)
    cmp_, sdf, st = F.fe_vs_beam(tab, loads, w, out, stiff, cases)
    cmp_.to_csv(OUT / "fe_vs_beam.csv", index=False)
    log(cmp_.round(3).to_string())
    mid = st[(st.y > 0.2) & (st.y < 1.2)]
    log("stiffness (0.2-1.2 m):", (mid.GJ_stream_equiv / mid.GJ_beam).describe().round(3).to_dict(),
        (mid.EI_fe / mid.EI_beam).describe().round(3).to_dict())
    return w, cases, out, sdf, st, cmp_


# ============================================================================ 3. beam mass/modal tools for non-flight cases
def beam_parts(tab, battery="P50B"):
    with F.beam_v1(battery):
        yn, EI, GJ, m, S_a, I_a, b, a = s.wing_beam(tab, ne=60, extra_masses=F.joint_masses())
    return yn, EI, GJ, m, S_a, I_a


def static_moment(yn, m, y):
    """sum of m (y' - y) outboard of y, kg m (projected span), from element masses."""
    yc = 0.5 * (yn[1:] + yn[:-1])
    L = np.diff(yn)
    o = yc > y
    return float(np.sum(m[o] * L[o] * (yc[o] - y)))


def landing_dynamic(tab, battery, v_sink, crush, stations=(0.0, 0.125, 0.45)):
    """Sudden stop of the pod (rigid ground + crush pulse): clamped wing beam, base half-sine deceleration.
    Returns peak downward bending moment (limit, N m) at the stations and the peak root shear (N)."""
    yn, EI, GJ, m, S_a, I_a = beam_parts(tab, battery)
    K, M = s.beam_matrices(yn, EI, GJ, m, S_a, I_a)
    from scipy.linalg import eigh
    lam, Phi = eigh(K, M)                          # mass-normalised modes
    om = np.sqrt(lam)
    nd = len(yn) - 1
    r = np.zeros(3 * nd)
    r[0::3] = 1.0
    gam = Phi.T @ M @ r
    keep = om < 2 * math.pi * 400
    Phi, om, gam = Phi[:, keep], om[keep], gam[keep]
    td = 2 * crush / v_sink
    apk = math.pi * v_sink / (2 * td)
    dt = min(td / 200, 2 * math.pi / om.max() / 40)
    t = np.arange(0, max(0.35, 3 * td), dt)
    a_b = np.where(t < td, apk * np.sin(math.pi * t / td), 0.0)
    # Newmark average acceleration per mode, q'' + w^2 q = -gam a_b
    q = np.zeros((len(t), len(om)))
    qd = np.zeros(len(om))
    qdd = -gam * a_b[0]
    keff = om**2 + 4 / dt**2
    for i in range(1, len(t)):
        p = -gam * a_b[i] + (4 / dt**2) * q[i - 1] + (4 / dt) * qd + qdd
        q[i] = p / keff
        qdd_new = 4 / dt**2 * (q[i] - q[i - 1]) - 4 / dt * qd - qdd
        qd = qd + dt / 2 * (qdd + qdd_new)
        qdd = qdd_new
    u = q @ Phi.T                                     # (time, dofs) relative displacement, w up positive
    out = {}
    for ys in stations:
        e = int(np.argmin(np.abs(yn[:-1] - ys)))
        L = yn[e + 1] - yn[e]

        def dof(n, k):
            return None if n == 0 else 3 * (n - 1) + k
        w1 = u[:, dof(e, 0)] if e > 0 else 0.0
        t1 = u[:, dof(e, 1)] if e > 0 else 0.0
        w2, t2 = u[:, dof(e + 1, 0)], u[:, dof(e + 1, 1)]
        curv = (-6 * w1 - 4 * L * t1 + 6 * w2 - 2 * L * t2) / L**2
        Mt = EI[e] * curv
        out[ys] = float(Mt.min())
    f_int = (K @ u.T).T
    root_shear = f_int[:, 0::3].sum(axis=1)
    out["root_shear_min"] = float(root_shear.min())
    out["f1_Hz"] = om[0] / 2 / math.pi
    out["apk_g"] = apk / G0
    out["td_ms"] = 1e3 * td
    out["wing_side_mass"] = float(np.sum(m * np.diff(yn)))
    return out


def one_g_bending(loads, cell, kit):
    """Limit bending at 1 g (lift = weight) from the manoeuvre + case (n = 3.8) scaled by 1/3.8."""
    case = f"{cell}, 0 m, {kit}, manoeuvre +, R"
    d = loads[loads.case == case].sort_values("y")
    return d.y.values, d.Mb.values / s.N_MAN_POS


def section_cap(tab, y):
    r = tab.iloc[int(np.argmin(np.abs(tab.y - y)))]
    sec = s.section(r.chord, F.LAY, r.t_cap_mm / 1e3, s.EPS20 if "EPS" in r.core else s.XPS)
    A = F.CAP_W * r.t_cap_mm * 1e-3
    return A, sec["h_eff"], sec["h"], r


def inplane_I(y, structural_te=True):
    """E-weighted (glass-equivalent) in-plane second moment of the section about its chordwise centroid,
    for a moment about the vertical axis (fore-aft bending). Skins to the rear web where the elevon is
    separate (y >= 0.45), to the trailing edge inboard; caps; rear web."""
    c = s.GEO.chord(y)
    x = np.linspace(0, 1, 401) * c
    xe = s.X_REAR * c if (y >= s.Y_BREAK or not structural_te) else c
    k = x <= xe
    xs = x[k]
    zt = F.z_up(xs / c) * c
    zb = F.z_lo(xs / c) * c
    ds_u = np.hypot(np.diff(xs), np.diff(zt))
    ds_l = np.hypot(np.diff(xs), np.diff(zb))
    xm = 0.5 * (xs[1:] + xs[:-1])
    n = s.CARBON_UD.E / F.SKIN_E
    A_items = [(F.TS * ds_u, xm), (F.TS * ds_l, xm)]
    _, _, h, r = section_cap(F.load_tables()[0], y)
    Acap = n * F.CAP_W * r.t_cap_mm * 1e-3
    A_items.append((np.array([2 * Acap]), np.array([s.X_SPAR * c])))
    A_items.append((np.array([F.T_REAR * s.thick(s.X_REAR) * c]), np.array([s.X_REAR * c])))
    A = np.concatenate([a for a, _ in A_items])
    X = np.concatenate([xx for _, xx in A_items])
    xb = np.sum(A * X) / np.sum(A)
    return float(np.sum(A * (X - xb) ** 2)), xb, xe


# ============================================================================ 4. non-flight cases
def extra_cases(tab, loads, fe):
    rows = []
    res = {}
    tabs = F.load_tables()[0]

    def add(**kw):
        rows.append(kw)

    # --- belly landing (pod stops; wing with packs keeps moving) -------------------------------------------
    import lodestar_v1 as v1
    skid = v1.STRUCTURE["belly_skid_crush_stroke_min_mm"] / 1e3     # design: EPP belly skid stroke
    land = {}
    for battery in ("P50B", "SiCore"):
        for v in sorted({1.5, 2.13, v1.STRUCTURE["landing_design_sink_mps"]}):
            for crush in sorted({0.010, 0.030, skid}):
                land[(battery, v, crush)] = landing_dynamic(tab, battery, v, crush)
    yy, m1 = one_g_bending(loads, "SiCore", "kit off")
    mb1g = {y: float(np.interp(y, yy, m1)) for y in (0.0, 0.125, 0.45)}
    yn, EI, GJ, m, *_ = beam_parts(tab, "SiCore")
    S0 = {y: static_moment(yn, m, y) for y in (0.0, 0.125, 0.45)}
    for (battery, v, crush), o in land.items():
        add(case="belly landing, sudden stop (beam modes)", variant=f"{battery}, sink {v} m/s, crush {1e3 * crush:.0f} mm",
            pulse=f"{o['td_ms']:.1f} ms, peak {o['apk_g']:.1f} g",
            Mb_root_limit=mb1g[0.0] + o[0.0], Mb_y0125_limit=mb1g[0.125] + o[0.125],
            Mb_break_limit=mb1g[0.45] + o[0.45], equiv_g_root=-o[0.0] / (G0 * S0[0.0]),
            root_shear_limit_N=o["root_shear_min"])
    for nz in (10.0,):
        add(case="belly landing, quasi-static", variant=f"{nz:.0f} g vertical", pulse="-",
            Mb_root_limit=mb1g[0.0] - nz * G0 * S0[0.0], Mb_y0125_limit=mb1g[0.125] - nz * G0 * S0[0.125],
            Mb_break_limit=mb1g[0.45] - nz * G0 * S0[0.45], equiv_g_root=nz, root_shear_limit_N=-nz * G0 * float(np.sum(m * np.diff(yn))))
    res["landing"] = pd.DataFrame(rows)
    nom = land[("SiCore", v1.STRUCTURE["landing_design_sink_mps"], skid)]   # v1.1 design sink
    res["land_design"] = dict(Mb={y: FOS * (mb1g[y] + nom[y]) for y in (0.0, 0.125, 0.45)},
                              shear=FOS * nom["root_shear_min"], equiv_g=-nom[0.0] / (G0 * S0[0.0]),
                              f1=nom["f1_Hz"])
    res["land_design"]["crush_mm"] = skid * 1e3
    res["land_design"]["sink"] = v1.STRUCTURE["landing_design_sink_mps"]
    soft = land[("SiCore", 1.5, 0.010)]      # sensitivity: no skid, 10 mm stroke on hard ground
    res["land_soft"] = dict(Mb={y: FOS * (mb1g[y] + soft[y]) for y in (0.0, 0.125, 0.45)})
    # --- bungee launch: 10 g limit longitudinal -------------------------------------------------------------
    nx = 10.0
    launch = {}
    for y in (0.0, 0.125, 0.45):
        Mip = FOS * nx * G0 * static_moment(yn, m, y)
        I, xb, xe = inplane_I(y)
        c = s.GEO.chord(y)
        sig_aft = Mip * (xe - xb) / I               # glass-equivalent stress at the aftmost structural skin
        sig_cap = Mip * abs(s.X_SPAR * c - xb) / I * s.CARBON_UD.E / F.SKIN_E
        launch[y] = dict(Mip=Mip, skin=sig_aft, cap=sig_cap)
    res["launch"] = launch
    wing_both = float(MB.mass_kg.iloc[0] - CAD[~CAD.item.str.startswith(M_WING_SIDE_ITEMS)].mass_kg.sum())
    res["wing_both_kg"] = wing_both
    arm = (s.X_REAR - s.X_SPAR) * s.GEO.chord(s.Y_BREAK) * F.CL
    res["launch_key_N"] = launch[0.45]["Mip"] / arm
    res["launch_attach_N"] = FOS * nx * G0 * wing_both
    res["launch_batt_kPa"] = FOS * nx * G0 * F.battery_side_mass("SiCore") / (0.213 * 0.020) / 1e3
    # --- kit release ---------------------------------------------------------------------------------------
    kit = 1.5 + 0.124
    after = float(MB[(MB.cell == "SiCore") & (~MB.kit)].mass_kg.iloc[0])
    res["kit_dn"] = 2 * kit / after
    # --- ground handling: assembled aircraft lifted at both tips ---------------------------------------------
    W = float(MB.mass_kg.max()) * G0
    res["tip_lift_Mb_ult"] = FOS * (W / 2 * 1.5 - G0 * S0[0.0])
    # --- winglet side load at max sideslip (stall-capped at VD) ------------------------------------------------
    h, cr, lamw = s.GEO.winglet_h, s.GEO.chord(1.5), 0.6
    ct = cr * lamw
    Sw = h * (cr + ct) / 2
    zcp = h / 3 * (cr + 2 * ct) / (cr + ct)
    re = s.RHO0 * s.VD * (cr + ct) / 2 / 1.81e-5
    af = s.asb.Airfoil("naca0008")
    a = af.get_aero_from_neuralfoil(alpha=np.linspace(0, 20, 81), Re=re, mach=s.VD / 340, model_size="large")
    clmax = float(np.max(a["CL"]))
    q = 0.5 * s.RHO0 * s.VD**2
    beta_gust = math.degrees(math.atan(s.UDE_D / s.VD))
    a_wl = 2 * math.pi * (2 * h**2 / Sw) / (2 * h**2 / Sw + 2)     # endplate: effective AR ~ 2 x geometric
    cl_gust = min(a_wl * math.radians(beta_gust), clmax)
    Fy = FOS * q * Sw * clmax
    Mroot = Fy * zcp
    # skin-only NACA0008 root section (glass + XPS core), bending about the chord line
    xs_ = np.linspace(0, 1, 401) * cr
    zt = 5 * 0.08 * cr * (0.2969 * np.sqrt(xs_ / cr) - 0.126 * xs_ / cr - 0.3516 * (xs_ / cr) ** 2
                          + 0.2843 * (xs_ / cr) ** 3 - 0.1015 * (xs_ / cr) ** 4)
    ds = np.hypot(np.diff(xs_), np.diff(zt))
    zm = 0.5 * (zt[1:] + zt[:-1])
    Iwl = 2 * F.TS * np.sum(ds * zm**2)
    sig_wl = Mroot * zt.max() / Iwl
    Zfoam = 0.7 * cr * (2 * zt.max()) ** 2 / 6
    res["winglet"] = dict(S=Sw, zcp=zcp, Re=re, clmax=clmax, beta_gust=beta_gust, cl_gust=cl_gust, Fy_ult=Fy,
                          M_ult=Mroot, skin=sig_wl, foam_butt=Mroot / Zfoam,
                          wing_Mb_ult=Mroot * F.CL, wing_T_ult=Mroot * F.SL)
    # --- elevon hinge -----------------------------------------------------------------------------------------
    ye = np.linspace(s.Y_BREAK, 1.5, 200)
    ce = 0.25 * np.array([s.GEO.chord(y) for y in ye])
    Se = float(np.trapezoid(ce, ye))
    Fe = FOS * q * Se * 1.0                            # normal-force coefficient 1.0 on the elevon, ASSUMED upper bound
    hinge_len = (1.5 - s.Y_BREAK) / math.cos(math.atan(np.polyfit(ye, [s.GEO.x_le(y) + 0.75 * s.GEO.chord(y) for y in ye], 1)[0]))
    Hm = FOS * q * float(np.trapezoid(ce * 0.4 * ce, ye))   # hinge moment, centre of pressure at 40% elevon chord
    res["elevon"] = dict(S=Se, F_ult=Fe, line=Fe / hinge_len, H_ult=Hm, hinge_len=hinge_len)
    pd.DataFrame(rows).to_csv(OUT / "landing_cases.csv", index=False)
    return res


# ============================================================================ 5. joint checks
def joint_local(jc, extra):
    """Local tube/sleeve/key checks for every joint-cut case (ultimate) plus the landing and launch cases."""
    D, Di, Ds, Le = JOINT.outer, JOINT.inner, JOINT.sleeve, JOINT.engagement
    t = (D - Di) / 2
    Rm = (D + Di) / 4
    lc = D                                          # assumed bearing contact length at each sleeve end
    Z = JOINT.section_modulus
    rows = []
    for r in jc.itertuples():
        M, V = FOS * abs(r.Mb), FOS * abs(r.V)
        rows.append(dict(case=r.case, sensitivity=r.sensitivity, M_ult=M, V_ult=V, T_spar_ult=FOS * abs(r.T_spar)))
    lb = extra["land_design"]["Mb"][0.45]
    rows.append(dict(case=f"belly landing, SiCore, {extra['land_design']['sink']:g} m/s, {extra['land_design']['crush_mm']:.0f} mm skid", sensitivity=False, M_ult=abs(lb),
                     V_ult=abs(lb) / 0.5, T_spar_ult=0.0))
    df = pd.DataFrame(rows)
    df["tube_bend_MPa"] = df.M_ult / Z / 1e6
    df["R_mouth_N"] = df.V_ult + df.M_ult / Le          # rigid tube, end contacts: mouth reaction
    df["R_inner_N"] = df.M_ult / Le
    P_line = df.R_mouth_N / lc
    df["tube_hoop_MPa"] = 6 * (0.318 * P_line * Rm) / t**2 / 1e6    # ring with diametral line load (Roark), conservative
    df["sleeve_bearing_MPa"] = df.R_mouth_N / (lc * D) / 1e6
    df["support_on_EPS_MPa"] = df.R_mouth_N / (lc * Ds) / 1e6
    df["support_on_ply_MPa"] = df.R_mouth_N / (0.0015 * 2 * Ds) / 1e6     # two 1.5 mm ribs at the mouth, Ds wide
    df["bond_shear_EPS_MPa"] = (df.R_mouth_N + df.R_inner_N) / (0.5 * math.pi * Ds * Le) / 1e6
    arm = (s.X_REAR - s.X_SPAR) * s.GEO.chord(s.Y_BREAK) * F.CL
    df["aft_key_N"] = df.T_spar_ult / arm
    df.to_csv(OUT / "joint_checks.csv", index=False)
    return df


# ============================================================================ 6. margins
def envelope_fe(tab, loads, cases, sdf):
    """FE-corrected envelope per element row: beam envelope x (FE/beam ratio of the governing FE case),
    and the direct FE maximum over the analysed cases; the larger is used."""
    rows = []
    for (j, y), g in sdf[sdf.case_key != "sens"].groupby(["j", "y"]):   # sensitivity case reported separately
        if y < 0.06 or y > 1.46:
            continue
        A, heff, h, r = section_cap(tab, y)
        env = tab.iloc[int(np.argmin(np.abs(tab.y - y)))]
        env_i = {c: float(np.interp(y, tab.y, tab[c])) for c in ("Mb_pos", "Mb_neg", "T_max", "V_max")}
        top = env_i["Mb_pos"] / (A * heff) / 1e6      # MPa, beam envelope cap stress
        bot = -env_i["Mb_neg"] / (A * heff) / 1e6

        def ratio(key, fe_col, bm_col, sign=1):
            gg = g[g.case_key == key]
            if gg.empty:
                return 1.0
            fe, bm = sign * gg[fe_col].iloc[0], gg[bm_col].iloc[0]
            return fe / bm if abs(bm) > 2.0 else 1.0
        k_top = ratio("pos", "cap_U_MPa", "beam_cap_U_MPa")
        k_bot = ratio("neg", "cap_L_MPa", "beam_cap_L_MPa")
        skin_fac = F.SKIN_E / F.CARB_E1 * h / heff
        kst = ratio("pos", "skin_U_min_MPa", "beam_skin_comp_MPa", -1)
        ksb = ratio("neg", "skin_L_min_MPa", "beam_skin_comp_MPa", -1)
        ks11 = ratio("pos", "skin_U_s11_min_MPa", "beam_skin_comp_MPa", -1)
        sec = s.section(r.chord, F.LAY, r.t_cap_mm / 1e3)
        tau_env = env_i["T_max"] / (2 * sec["A_cell"] * F.TS)        # Pa
        ktau = max(ratio(k, "skin_U_tau_MPa", "beam_skin_tau_MPa") for k in g.case_key.unique())
        q_env = env_i["V_max"] / heff
        kq = max(ratio(k, "web_q_Nmm", "beam_web_q_Nmm") for k in g.case_key.unique())
        scale = max(env_i["Mb_pos"] / max(abs(gg.beam_Mb.iloc[0]), 1e-6) for _, gg in g.groupby("case_key"))
        scale = min(max(scale, 1.0), 3.0)
        rows.append(dict(
            y=y, core=F.core_name(y), t_cap_mm=r.t_cap_mm,
            cap_top_MPa=max(top * k_top, -g.cap_U_MPa.min()), cap_bot_MPa=max(bot * k_bot, -g.cap_L_MPa.min()),
            cap_tens_MPa=max(top, bot) * max(k_top, k_bot),
            skin_top_MPa=max(top * skin_fac * kst, -g.skin_U_min_MPa.min()),
            skin_bot_MPa=max(bot * skin_fac * ksb, -g.skin_L_min_MPa.min()),
            skin_top_s11_MPa=max(top * skin_fac * ks11, -g.skin_U_s11_min_MPa.min()),
            skin_tau_MPa=max(tau_env / 1e6 * ktau, g[["skin_U_tau_MPa", "skin_L_tau_MPa"]].max().max()),
            web_q_Nmm=max(q_env / 1e3 * kq, g.web_q_Nmm.max()),
            web_gamma=max(g.web_gamma.max() * scale, 0), rear_gamma=g.rear_gamma.max() * scale,
            core_tau_kPa=g.core_tau_kPa.max() * scale, core_comp_kPa=g.core_comp_kPa.max() * scale,
            k_top=k_top, k_bot=k_bot, k_skin_top=kst, k_skin_bot=ksb, k_tau=ktau, k_q=kq))
    e = pd.DataFrame(rows)
    e.to_csv(OUT / "envelope_fe.csv", index=False)
    return e


def margins(tab, loads, lc, env, extra, jl, sdf, cmp_):
    R = []
    A_ = ALLOW

    def add(element, mode, case, applied, unit, akey, note="", allow_override=None):
        allow, src, kind = A_[akey]
        if allow_override is not None:
            allow = allow_override
        scale = 1e6 if unit == "MPa" else 1e3 if unit in ("N/mm", "kPa") else 1.0
        a = allow / scale
        m_ = ms(a, applied)
        R.append(dict(element=element, failure_mode=mode, governing_case=case, applied_ult=applied, allowable=a,
                      unit=unit, allowable_source=src, MS=m_, status=status(m_, kind), note=note))

    nl = lc[~lc.sensitivity]
    gp = nl.loc[nl.root_Mb_ult.idxmax(), "label"]
    gn = nl.loc[nl.root_Mb_ult.idxmin(), "label"]
    gt = nl.loc[nl.max_abs_T_ult.idxmax(), "label"]

    def worst(col, mask=None):
        e = env if mask is None else env[mask]
        r = e.loc[e[col].idxmax()]
        return float(r[col]), float(r.y)

    v, y = worst("cap_top_MPa")
    add("spar cap, upper", "compression (fibre)", f"{gp} (envelope), y={y:.3f} m", v, "MPa", "cap_comp")
    add("spar cap, upper", "crippling / face wrinkling on Rohacell web", f"{gp} (envelope), y={y:.3f} m", v, "MPa", "cap_wrinkle")
    v2, y2 = worst("cap_bot_MPa")
    add("spar cap, lower", "compression / wrinkling on web (flight)", f"{gn} (envelope), y={y2:.3f} m", v2, "MPa", "cap_wrinkle")
    vt, yt = worst("cap_tens_MPa")
    add("spar caps", "tension", f"{gp} (envelope), y={yt:.3f} m", vt, "MPa", "cap_tens")
    for core, key in (("EPS", "skin_wr_EPS"), ("XPS", "skin_wr_XPS")):
        mask = env.core == core
        a1, ya = worst("skin_top_MPa", mask)
        add(f"upper skin on {core}", "face wrinkling (min principal)", f"{gp} (envelope), y={ya:.3f} m", a1, "MPa", key)
        b1, yb = worst("skin_bot_MPa", mask)
        add(f"lower skin on {core}", "face wrinkling (min principal), flight", f"{gn} (envelope), y={yb:.3f} m", b1, "MPa", key)
    a2, ya2 = worst("skin_top_s11_MPa", env.core == "EPS")
    add("upper skin on EPS", "face wrinkling, spanwise stress only (beam-model criterion, shear ignored)",
        f"{gp} (envelope), y={ya2:.3f} m", a2, "MPa", "skin_wr_EPS", "for comparison; the principal-stress row governs")
    vs_, ys_ = worst("skin_tau_MPa")
    add("skins", "shear (torsion)", f"{gt} (envelope), y={ys_:.3f} m", vs_, "MPa", "skin_shear")
    vc, yc = worst("core_tau_kPa", env.core == "EPS")
    add("EPS core", "shear", f"FE cases scaled to envelope, y={yc:.3f} m", vc, "kPa", "core_tau_EPS",
        "global load path only; pressure transfer to webs ~1-2 kPa by hand")
    vcx, ycx = worst("core_tau_kPa", env.core == "XPS")
    add("XPS core (servo bay, winglet root)", "shear", f"FE cases scaled, y={ycx:.3f} m", vcx, "kPa", "core_tau_XPS")
    vcc, ycc = worst("core_comp_kPa", env.core == "EPS")
    add("EPS core", "compression/crushing", f"FE cases scaled, y={ycc:.3f} m", vcc, "kPa", "core_comp_EPS",
        "excludes vacuum-bag cure pressure (process check)")
    q_allow = float(tab.q_web_allow_Nmm.iloc[0])
    vq, yq = worst("web_q_Nmm")
    add("spar web (Rohacell + glass)", "shear", f"envelope V, y={yq:.3f} m", vq, "N/mm", "web_gamma",
        "allowable = glass strain limit x web Gt", allow_override=q_allow * 1e3)
    vr, yr = worst("rear_gamma")
    add("rear web (XPS + glass)", "shear strain", f"FE torsion case scaled, y={yr:.3f} m", vr, "-", "web_gamma")
    # landing (lower surface at root), sudden stop
    ld = extra["land_design"]
    for ysta in (0.0, 0.125):
        A, heff, h, r = section_cap(tab, ysta)
        sig = abs(ld["Mb"][ysta]) / (A * heff)
        kb = float(np.interp(max(ysta, 0.0625), env.y, env.k_bot))
        ksk = float(np.interp(max(ysta, 0.0625), env.y, env.k_skin_bot))
        add("spar cap, lower", "compression / wrinkling on web (belly landing)",
            f"belly landing SiCore {ld['sink']:g} m/s, {ld['crush_mm']:.0f} mm skid, y={ysta:.3f}", sig * kb / 1e6, "MPa", "cap_wrinkle",
            f"Mb_ult={ld['Mb'][ysta]:.1f} N m; equivalent {ld['equiv_g']:.1f} g on the wing")
        add("lower skin on EPS", "face wrinkling (belly landing)", f"belly landing SiCore {ld['sink']:g} m/s, {ld['crush_mm']:.0f} mm skid, y={ysta:.3f}",
            sig * F.SKIN_E / F.CARB_E1 * h / heff * ksk / 1e6, "MPa", "skin_wr_EPS")
    soft = extra["land_soft"]["Mb"][0.125]
    A, heff, h, r = section_cap(tab, 0.125)
    add("lower skin on EPS", "face wrinkling (belly landing, SENSITIVITY: no skid, hard ground)", "belly landing SiCore 1.5 m/s, 10 mm crush, y=0.125",
        abs(soft) / (A * heff) * F.SKIN_E / F.CARB_E1 * h / heff / 1e6, "MPa", "skin_wr_EPS")
    # bungee launch in-plane bending and ground handling (root positive capacity = envelope / skin utilisation)
    la = extra["launch"]
    add("aft skin / rear web region on EPS", "face wrinkling, in-plane bending at launch", "bungee launch 10 g limit, y=0",
        la[0.0]["skin"] / 1e6, "MPa", "skin_wr_EPS", f"in-plane root moment {la[0.0]['Mip']:.0f} N m ult")
    add("wing centre section", "positive bending, aircraft lifted at both tips (1 g x 1.5)", "ground handling",
        extra["tip_lift_Mb_ult"], "N m", "skin_wr_EPS",
        "compared with the flight envelope root moment: covered by flight, inherits the flight skin margin",
        allow_override=float(tab.Mb_pos.iloc[0]))
    # joint
    nj = jl[~jl.sensitivity & ~jl.case.str.startswith("belly")]
    r = nj.loc[nj.tube_bend_MPa.idxmax()]
    add("joint tube 20x16", "bending", r.case, r.tube_bend_MPa, "MPa", "tube_bend")
    rs = jl[jl.sensitivity].sort_values("tube_bend_MPa").iloc[-1]
    add("joint tube 20x16", "bending, fixed-aileron sensitivity", rs.case, rs.tube_bend_MPa, "MPa", "tube_bend")
    rl = jl[jl.case.str.startswith("belly")].iloc[0]
    add("joint tube 20x16", "bending, belly landing", rl.case, rl.tube_bend_MPa, "MPa", "tube_bend")
    rh = jl.loc[jl[~jl.sensitivity].R_mouth_N.idxmax()]
    add("joint tube 20x16", "ovalization at sleeve mouth (ring bending)", rh.case, rh.tube_hoop_MPa, "MPa", "tube_hoop",
        f"mouth reaction {rh.R_mouth_N:.0f} N over 20 mm, diametral line load (conservative)")
    add("sleeve 24 mm", "bearing", rh.case, rh.sleeve_bearing_MPa, "MPa", "sleeve_bearing")
    add("sleeve end support", "bearing into EPS (no hard point)", rh.case, rh.support_on_EPS_MPa, "MPa", "core_comp_EPS",
        "design must not rely on foam", allow_override=A_["core_comp_EPS"][0])
    add("sleeve end support", "bearing on two 1.5 mm ply/G10 ribs", rh.case, rh.support_on_ply_MPa, "MPa", "support_ply",
        "proposed hard point inside the 45 g allowance")
    add("sleeve bond", "shear into EPS (bonded in foam only)", rh.case, rh.bond_shear_EPS_MPa, "MPa", "core_tau_EPS")
    add("sleeve bond", "shear via glass wrap onto Rohacell web / caps", rh.case, rh.bond_shear_EPS_MPa, "MPa", "bond_wrap")
    rk = jl.loc[jl[~jl.sensitivity].aft_key_N.idxmax()]
    kN = max(rk.aft_key_N, extra["launch_key_N"])
    kc = rk.case if rk.aft_key_N >= extra["launch_key_N"] else "bungee launch 10 g, in-plane moment at break"
    add("aft key (4 mm carbon pin in 2 mm G10)", "pin shear", kc, kN / (math.pi / 4 * 4**2), "MPa", "key_pin_shear")
    add("aft key (4 mm carbon pin in 2 mm G10)", "plate bearing", kc, kN / (4 * 2), "MPa", "key_bearing")
    # attachment to pod (concept: 2 x M5 nylon bolts + 2 x 6 mm dowels + saddle)
    pod_kg = float(CAD[~CAD.item.str.startswith(M_WING_SIDE_ITEMS)].mass_kg.sum())
    nmax = lc[~lc.sensitivity].n_limit.max()
    nmin = lc[~lc.sensitivity].n_limit.min()
    Ften = FOS * nmax * G0 * pod_kg
    add("wing-pod attachment, 2 x M5 nylon", "bolt tension (pod hangs from wing)", f"max flight n = {nmax:.2f}",
        Ften / 2, "N", "nylon_m5", f"pod-side mass {pod_kg:.2f} kg loaded")
    Fl = extra["launch_attach_N"]
    add("wing-pod attachment, 2 x 6 mm dowels", "dowel shear, bungee launch 10 g", "bungee launch, 10 g limit", Fl / 2, "N",
        "dowel_shear", f"wing side mass both {extra['wing_both_kg']:.2f} kg")
    Fv = abs(extra["land_design"]["shear"]) * 2
    add("wing-pod attachment saddle", "bearing on pod saddle, belly landing (2 x 60 x 40 mm pads on EPS)",
        f"belly landing SiCore {extra['land_design']['sink']:g} m/s, {extra['land_design']['crush_mm']:.0f} mm skid", Fv / (2 * 0.060 * 0.040) / 1e3, "kPa", "core_comp_EPS",
        "wing underside over the pod is EPS: needs a ply/G10 saddle plate (then ~10x margin)")
    # battery bay
    st = sdf[(sdf.case_key == "pos")]
    ry = st.iloc[int(np.argmin(np.abs(st.y - F.Y_BATT_V1)))]
    b_bay = 0.107
    D = F.SKIN_E * F.TS**3 / (12 * (1 - F.SKIN_NU**2))
    sig_cr = 4 * math.pi**2 * D / (b_bay**2 * F.TS)
    add("battery bay skin (if core removed, no lid)", "panel buckling, compression", f"{gp}, y={F.Y_BATT_V1}",
        -ry.skin_U_min_MPa, "MPa", "skin_wr_EPS", "unsupported 107 mm panel: sigma_cr = 4 pi^2 D / (b^2 t)",
        allow_override=sig_cr)
    A_cell = s.section(s.GEO.chord(F.Y_BATT_V1), F.LAY, 1e-3)["A_cell"]
    Tb = float(np.interp(F.Y_BATT_V1, tab.y, tab.T_max))
    qlid = Tb / (2 * A_cell)
    add("battery bay lid (structural, lower surface)", "screw shear, M3 at 30 mm pitch", f"{gt} (envelope T)",
        qlid * 0.030, "N", "lid_screw", f"shear flow q = T/2A = {qlid / 1e3:.2f} N/mm")
    add("battery bay walls", "pack inertia bearing on EPS, launch 10 g", "bungee launch", extra["launch_batt_kPa"], "kPa",
        "core_comp_EPS", "213 x 20 mm wall contact; packs also need straps")
    # servo bay: XPS around a cut-out in the lower skin, Kt = 3
    sv = sdf[(sdf.case_key == "neg")]
    rv = sv.iloc[int(np.argmin(np.abs(sv.y - s.Y_SERVO)))]
    envr = env.iloc[int(np.argmin(np.abs(env.y - s.Y_SERVO)))]
    add("servo bay (lower skin cut-out on XPS)", "wrinkling at cut-out edge, Kt = 3", f"{gn} (envelope)",
        3 * envr.skin_bot_MPa, "MPa", "skin_wr_XPS", "needs the XPS/EPS transition coupon")
    # winglet
    wl = extra["winglet"]
    add("winglet root skin (glass on XPS, no spar)", "face wrinkling, side load", f"stall-capped side force at VD, CLmax {wl['clmax']:.2f}",
        wl["skin"] / 1e6, "MPa", "skin_wr_XPS")
    add("winglet attachment (foam butt joint only)", "foam tension at root", "stall-capped side force at VD",
        wl["foam_butt"] / 1e6, "MPa", "core_tau_XPS", "XPS tension ~0.2 MPa: needs skin overlap tape / dowels",
        allow_override=0.20e6)
    lineN = wl["M_ult"] / (0.017 * 0.7 * s.GEO.chord(1.5))
    add("winglet attachment (2 x 25 mm glass overlap tapes)", "bond shear", "stall-capped side force at VD",
        lineN / 0.025 / 1e6, "MPa", "bond_wrap")
    # elevon hinge
    el = extra["elevon"]
    add("elevon hinge line", "line load", "elevon normal force coefficient 1.0 at VD", el["line"] / 1e3, "N/mm", "hinge_line")
    # deflection
    tipfe = cmp_[cmp_.y == 1.5].tip_w_limit_fe_mm.abs().max() / 1e3
    tipbm = lc[~lc.sensitivity].tip_defl_limit_mm.max() / 1e3
    add("wing", "tip deflection at limit load", "envelope (beam) scaled by FE/beam", max(tipbm * tipfe / max(
        cmp_[cmp_.y == 1.5].tip_w_limit_beam_mm.abs().max() / 1e3, 1e-9), tipbm), "m", "tip_defl")
    M = pd.DataFrame(R)
    M.to_csv(OUT / "margins.csv", index=False)
    return M


def main(tab=None, loads=None):
    OUT.mkdir(parents=True, exist_ok=True)
    if tab is None:
        tab, loads = F.load_tables()
    lc, fam = load_case_matrix(loads)
    log(fam.round(2).to_string())
    w, cases, out, sdf, st, cmp_ = statics(tab, loads)
    extra = extra_cases(tab, loads, out)
    log(extra["landing"].round(2).to_string())
    log({k: v for k, v in extra.items() if k not in ("landing",)})
    jc = pd.read_csv(F.INPUTS / f"{F.DESIGN}_joint.csv")
    jl = joint_local(jc, extra)
    env = envelope_fe(tab, loads, cases, sdf)
    M = margins(tab, loads, lc, env, extra, jl, sdf, cmp_)
    pd.set_option("display.width", 250)
    log(M[["element", "failure_mode", "applied_ult", "allowable", "unit", "MS", "status"]].round(3).to_string())
    pd.Series({k: (v if not isinstance(v, dict) else str(v)) for k, v in extra.items() if k != "landing"}).to_csv(OUT / "extra_cases.csv")


if __name__ == "__main__":
    if sys.argv[1:] == ["--convergence"]:
        tab, loads = F.load_tables()
        c = F.convergence(tab, loads, F.governing_cases(loads)["pos"])
        c.to_csv(OUT / "convergence.csv", index=False)
        print(c.round(3).to_string())
    else:
        main()
