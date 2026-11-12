"""Milestone 1: first sizing of the Lodestar.

Question: what is the lightest flying wing (span, aspect ratio, battery) that
flies >= 3 h with a thermal camera and a 1.5 kg rescue kit, stalls at <= 12 m/s
and still reaches >= 22 m/s to make headway in wind?

Method (plain version):
  1. Pick a span b, aspect ratio AR (span / average chord) and battery mass.
  2. Add up the weight: fixed items + wing structure + battery + motor.
  3. For a range of speeds, compute drag:
       airfoil drag (NeuralFoil, at the right Reynolds number)
     + induced drag (the cost of making lift, smaller for long thin wings)
     + body drag (camera ball and centre pod).
  4. Power = drag x speed / propulsion efficiency, plus electronics.
  5. Endurance = usable battery energy / power, at the best allowed speed.
  6. Keep designs that meet every requirement; pick the lightest.
  7. Cross-check the pick with AeroSandbox's AeroBuildup on the real 3D geometry.

Run:  uv run python scripts/sizing.py
Writes: out/sizing/*.csv, *.png, run_log.txt

Numbers marked ASSUMPTION are placeholders with sensitivity checks; DATA has a source
in the research note.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sizing"
OUT.mkdir(parents=True, exist_ok=True)

G = 9.81


class Tee:  # print to screen and run_log.txt
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()



# ---------------------------------------------------------------- environment
DESIGN_ALT = 2000.0  # m. ASSUMPTION: mountain search areas (e.g. Colorado trailheads at 2-3 km); thinner air is the hard case
atmo = asb.Atmosphere(altitude=DESIGN_ALT)
RHO = float(atmo.density())
MU = float(atmo.dynamic_viscosity())


# ---------------------------------------------------------------- assumptions
@dataclass
class Assumptions:
    # payload and fixed items
    m_gimbal: float = 0.60      # dual thermal+RGB gimbal (requirement budget)
    m_kit: float = 1.50         # rescue kit (requirement)
    m_kit_chute: float = 0.20   # kit parachute + release, ASSUMPTION, updated by kit_drop.py
    m_avionics: float = 0.35    # autopilot, GPS, telemetry radio, wiring harness, ASSUMPTION
    m_centerbody: float = 0.30  # payload bay / centre pod structure, ASSUMPTION
    misc_frac: float = 0.05     # glue, connectors, paint as a fraction of MTOW, ASSUMPTION
    # structure
    wing_kg_per_m2: float = 1.0  # composite-skinned foam wing incl. elevons, servos, winglets, ASSUMPTION (swept 0.8-1.3)
    # energy
    batt_wh_per_kg: float = 200.0  # Li-ion 21700 pack (cells about 250-270 Wh/kg), ASSUMPTION
    usable_frac: float = 0.85 * 0.90  # 85% depth of discharge, then 10% held as landing reserve
    p_avionics_w: float = 25.0  # gimbal (about 8 W) + autopilot + radios, ASSUMPTION
    # propulsion
    eta_cruise: float = 0.55    # prop 0.70 x motor 0.85 x ESC 0.95 at part load, ASSUMPTION
    eta_max: float = 0.60       # at full throttle
    prop_kw_per_kg: float = 2.0  # motor + ESC + prop + mount system, ASSUMPTION
    climb_rate: float = 3.0     # m/s required climb at search speed
    # aerodynamics
    airfoil: str = "mh45"       # reflexed flying-wing section (Martin Hepperle), near-zero pitching moment
    profile_factor: float = 1.15  # elevon gaps, winglets, skin roughness, ASSUMPTION
    e_oswald: float = 0.80      # flying wings use washout for stability, which costs span efficiency, ASSUMPTION
    cda_body: float = 0.004     # m^2: gimbal ball (about 0.0016) + centre pod (about 0.0015) + skid, ASSUMPTION
    cl_max_3d: float = 0.80     # swept reflexed wing with washout, ASSUMPTION (checked vs NeuralFoil below)
    # requirements
    endurance_h: float = 3.0
    v_stall_max: float = 12.0
    v_max_min: float = 22.0
    v_search_min: float = 14.0
    v_search_max: float = 20.0
    span_max: float = 3.0       # v1 asked for 2.5 m; no 2.5 m design reaches 3 h (see trade table), so raised to 3.0 m in 3 pieces
    stall_margin: float = 1.25  # fly no slower than 1.25 x stall speed


A = Assumptions()

# ---------------------------------------------------------------- airfoil polars from NeuralFoil
af = asb.Airfoil(A.airfoil)
RE_GRID = np.array([6e4, 1e5, 1.5e5, 2e5, 3e5, 4e5, 6e5, 8e5])
ALPHA = np.linspace(-4, 14, 73)
CL_TAB, CD_TAB = [], []
for Re in RE_GRID:
    a = af.get_aero_from_neuralfoil(alpha=ALPHA, Re=Re, mach=0.05, model_size="large")
    CL_TAB.append(np.asarray(a["CL"]))
    CD_TAB.append(np.asarray(a["CD"]))
CL_TAB, CD_TAB = np.array(CL_TAB), np.array(CD_TAB)
CLMAX_2D = CL_TAB.max(axis=1)


# Re-grid the polars onto a regular (log Re, CL) table, pre-stall branch only.
from scipy.interpolate import RegularGridInterpolator

CL_REG = np.linspace(-0.3, 1.2, 151)
CD_REG = np.array([np.interp(CL_REG, CL_TAB[k][:np.argmax(CL_TAB[k]) + 1], CD_TAB[k][:np.argmax(CL_TAB[k]) + 1])
                   for k in range(len(RE_GRID))])
_cd_interp = RegularGridInterpolator((np.log(RE_GRID), CL_REG), CD_REG, bounds_error=False, fill_value=None)


def cd_profile(cl, Re):
    """Airfoil drag at a given lift coefficient and Reynolds number (interpolated)."""
    cl, Re = np.broadcast_arrays(np.asarray(cl, float), np.asarray(Re, float))
    pts = np.column_stack([np.log(np.clip(Re, RE_GRID[0], RE_GRID[-1])).ravel(),
                           np.clip(cl, CL_REG[0], CL_REG[-1]).ravel()])
    return _cd_interp(pts).reshape(cl.shape)


# ---------------------------------------------------------------- one design point
V_SWEEP = np.linspace(10, 32, 111)


def evaluate(b, AR, m_batt, a: Assumptions = A):
    S = b**2 / AR
    c = b / AR
    m_fixed = a.m_gimbal + a.m_kit + a.m_kit_chute + a.m_avionics + a.m_centerbody
    m_wing = a.wing_kg_per_m2 * S
    m_prop = 0.3
    for _ in range(30):  # close the loop: motor size depends on weight, weight on motor size
        mtow = (m_fixed + m_wing + m_batt + m_prop) / (1 - a.misc_frac)
        W = mtow * G
        q = 0.5 * RHO * V_SWEEP**2
        CL = W / (q * S)
        Re = RHO * V_SWEEP * c / MU
        ok_cl = CL < 0.98 * CLMAX_2D.max()
        CD = np.where(ok_cl, cd_profile(np.minimum(CL, 1.2), Re) * a.profile_factor, np.nan) \
            + CL**2 / (math.pi * a.e_oswald * AR) + a.cda_body / S
        D = q * S * CD
        v_stall = math.sqrt(2 * W / (RHO * S * a.cl_max_3d))
        # max power: reach v_max_min in level flight, or climb at search speed
        i_vmax = np.argmin(abs(V_SWEEP - a.v_max_min))
        v_climb = max(a.v_search_min, a.stall_margin * v_stall)
        i_cl = np.argmin(abs(V_SWEEP - v_climb))
        p_max = max(D[i_vmax] * V_SWEEP[i_vmax], D[i_cl] * V_SWEEP[i_cl] + W * a.climb_rate) / a.eta_max
        new_prop = p_max / 1000 / a.prop_kw_per_kg + 0.05
        if abs(new_prop - m_prop) < 1e-5:
            break
        m_prop = new_prop
    p_elec = D * V_SWEEP / a.eta_cruise + a.p_avionics_w
    allowed = (V_SWEEP >= max(a.v_search_min, a.stall_margin * v_stall)) & (V_SWEEP <= a.v_search_max) & ~np.isnan(p_elec)
    if not allowed.any():
        return None
    j = np.nanargmin(np.where(allowed, p_elec, np.inf))
    energy = m_batt * a.batt_wh_per_kg * a.usable_frac
    endurance = energy / p_elec[j]
    LD = (CL / CD)[j]
    return dict(
        span_m=b, AR=AR, S_m2=S, chord_m=c, m_batt=m_batt, m_wing=m_wing, m_prop=m_prop,
        mtow=mtow, wing_loading=mtow / S, v_stall=v_stall, v_search=V_SWEEP[j], CL_search=CL[j],
        Re_search=Re[j], LD_search=LD, p_search_w=p_elec[j], p_max_w=p_max,
        batt_wh=m_batt * a.batt_wh_per_kg, endurance_h=endurance,
        ok=(endurance >= a.endurance_h) and (v_stall <= a.v_stall_max) and (b <= a.span_max),
    )


def sweep(a: Assumptions = A):
    rows = []
    for b in np.round(np.arange(1.6, 3.41, 0.1), 2):
        for AR in np.arange(5.0, 11.01, 0.5):
            for mb in np.round(np.arange(0.5, 5.01, 0.05), 2):
                r = evaluate(b, AR, mb, a)
                if r:
                    rows.append(r)
    return pd.DataFrame(rows)


def best(df):
    ok = df[df.ok]
    return ok.loc[ok.mtow.idxmin()] if len(ok) else None


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    sys.stdout = Tee(OUT / "run_log.txt")
    print(f"Lodestar first sizing, 2026-11-11")
    print(f"Design altitude {DESIGN_ALT:.0f} m: air density {RHO:.3f} kg/m^3 (sea level 1.225)")
    print(f"NeuralFoil {A.airfoil.upper()} 2D CLmax by Re: " +
          ", ".join(f"{Re/1e3:.0f}k:{m:.2f}" for Re, m in zip(RE_GRID, CLMAX_2D)))
    print(f"Assumed 3D CLmax {A.cl_max_3d} (about 75-85% of 2D for a swept wing with washout)\n")

    df = sweep()
    df.round(4).to_csv(OUT / "sweep.csv", index=False)

    # Requirement trade: best endurance reachable for each span cap and stall-speed cap
    print("Requirement trade: best endurance (h) reachable, by span cap (rows) and stall-speed cap (columns)")
    trade = pd.DataFrame({f"stall <= {vs} m/s": [df[(df.span_m <= sm + 1e-9) & (df.v_stall <= vs)].endurance_h.max()
                                                 for sm in (2.5, 2.8, 3.0, 3.2)] for vs in (12, 13, 14)},
                         index=[f"span <= {sm} m" for sm in (2.5, 2.8, 3.0, 3.2)])
    print(trade.round(2).to_string())
    trade.round(3).to_csv(OUT / "requirement_trade.csv")
    print()
    pick = best(df)
    print(f"{len(df)} designs evaluated, {int(df.ok.sum())} meet every requirement")
    cols = ["span_m", "AR", "S_m2", "chord_m", "m_batt", "m_wing", "m_prop", "mtow", "wing_loading",
            "v_stall", "v_search", "CL_search", "Re_search", "LD_search", "p_search_w", "p_max_w",
            "batt_wh", "endurance_h"]
    print("\nLightest design that meets every requirement:")
    print(pick[cols].astype(float).round(3).to_string())

    # Best design for each span (what span buys you)
    per_span = df[df.ok].loc[df[df.ok].groupby("span_m").mtow.idxmin()]
    print("\nLightest passing design at each span:")
    print(per_span[["span_m", "AR", "m_batt", "mtow", "v_stall", "v_search", "LD_search", "endurance_h"]]
          .round(2).to_string(index=False))
    per_span.round(4).to_csv(OUT / "best_per_span.csv", index=False)

    # Sensitivity: rerun the whole sizing with one assumption changed
    print("\nSensitivity (lightest passing design when one assumption changes):")
    sens_rows = []
    for label, kw in [("baseline", {}),
                      ("wing 0.8 kg/m2", dict(wing_kg_per_m2=0.8)),
                      ("wing 1.3 kg/m2", dict(wing_kg_per_m2=1.3)),
                      ("battery 170 Wh/kg", dict(batt_wh_per_kg=170)),
                      ("battery 240 Wh/kg", dict(batt_wh_per_kg=240)),
                      ("span efficiency 0.70", dict(e_oswald=0.70)),
                      ("body drag x2", dict(cda_body=0.008)),
                      ("CLmax 0.70", dict(cl_max_3d=0.70)),
                      ("propulsion eff 0.50", dict(eta_cruise=0.50)),
                      ("sea level", "SEA")]:
        if kw == "SEA":
            RHO_SAVE, MU_SAVE = RHO, MU
            RHO, MU = 1.225, 1.789e-5
            p = best(sweep())
            RHO, MU = RHO_SAVE, MU_SAVE
        else:
            a = Assumptions(**{**asdict(A), **kw})
            p = best(sweep(a))
        if p is None:
            print(f"  {label:<22} no design meets the requirements")
            sens_rows.append(dict(case=label))
            continue
        print(f"  {label:<22} span {p.span_m:.1f} m, AR {p.AR:.1f}, battery {p.m_batt:.2f} kg, "
              f"MTOW {p.mtow:.2f} kg, endurance {p.endurance_h:.2f} h")
        sens_rows.append(dict(case=label, span_m=p.span_m, AR=p.AR, m_batt=p.m_batt, mtow=p.mtow,
                              endurance_h=p.endurance_h))
    pd.DataFrame(sens_rows).round(3).to_csv(OUT / "sensitivity.csv", index=False)

    # ---------------------------------------------------------- cross-check with AeroBuildup on 3D geometry
    # Planform ASSUMPTIONS for this first pass (refined in milestone 2):
    # taper 0.45, quarter-chord sweep 25 deg, 3 deg washout, winglets omitted here.
    b, S = float(pick.span_m), float(pick.S_m2)
    taper = 0.45
    c_root = 2 * S / (b * (1 + taper))
    c_tip = taper * c_root
    sweep_deg = 25.0
    x_tip = (b / 2) * math.tan(math.radians(sweep_deg)) + 0.25 * c_root - 0.25 * c_tip
    wing = asb.Wing(
        name="Lodestar", symmetric=True,
        xsecs=[
            asb.WingXSec(xyz_le=[0, 0, 0], chord=c_root, twist=0, airfoil=af),
            asb.WingXSec(xyz_le=[x_tip, b / 2, 0], chord=c_tip, twist=-3, airfoil=af),
        ],
    )
    plane = asb.Airplane(name="Lodestar", xyz_ref=[0.25 * c_root + 0.1 * x_tip, 0, 0], wings=[wing], s_ref=S, c_ref=wing.mean_aerodynamic_chord(), b_ref=b)
    V = float(pick.v_search)
    W = float(pick.mtow) * G
    CL_need = W / (0.5 * RHO * V**2 * S)
    alphas = np.linspace(-2, 10, 49)
    ab = asb.AeroBuildup(airplane=plane, op_point=asb.OperatingPoint(atmosphere=atmo, velocity=V, alpha=alphas)).run()
    CLs, CDs = np.asarray(ab["CL"]), np.asarray(ab["CD"])
    a_trim = np.interp(CL_need, CLs, alphas)
    CD_ab = np.interp(CL_need, CLs, CDs) + A.cda_body / S
    LD_ab = CL_need / CD_ab
    print("\nCross-check: AeroBuildup on the 3D planform (taper 0.45, 25 deg sweep, 3 deg washout)")
    print(f"  root chord {c_root:.3f} m, tip chord {c_tip:.3f} m, MAC {wing.mean_aerodynamic_chord():.3f} m")
    print(f"  at search speed {V:.1f} m/s: CL needed {CL_need:.3f} at alpha {a_trim:.1f} deg")
    print(f"  L/D: sizing model {pick.LD_search:.1f}, AeroBuildup {LD_ab:.1f} "
          f"({(LD_ab / pick.LD_search - 1) * 100:+.0f}%)")
    end_ab = pick.batt_wh * A.usable_frac / (W * V / LD_ab / A.eta_cruise + A.p_avionics_w)
    print(f"  endurance implied by AeroBuildup L/D: {end_ab:.2f} h")

    # Search coverage (thermal camera)
    hfov = 32.0   # deg, FLIR Boson 640 with 13.6 mm lens class, ASSUMPTION
    h_agl = 120.0
    swath = 2 * h_agl * math.tan(math.radians(hfov / 2))
    gsd = swath / 640
    eff_swath = 0.8 * swath  # 20% overlap between passes
    search_h = pick.endurance_h - 0.5  # 15 min out and 15 min back, ASSUMPTION
    area_km2 = eff_swath * pick.v_search * search_h * 3600 / 1e6
    print(f"\nSearch coverage at {h_agl:.0f} m with a {hfov:.0f} deg thermal camera:")
    print(f"  swath {swath:.0f} m, {gsd * 100:.1f} cm per pixel, so a person (about 0.45 m) spans "
          f"{0.45 / gsd:.1f} pixels")
    print(f"  about {eff_swath * pick.v_search * 3.6 / 1000:.1f} km^2 per hour, {area_km2:.1f} km^2 per flight "
          f"({search_h:.1f} h on station)")

    geom = dict(c_root=c_root, c_tip=c_tip, taper=taper, sweep_qc_deg=sweep_deg, washout_deg=3,
                mac=wing.mean_aerodynamic_chord(), x_tip_le=x_tip, alpha_search=a_trim, LD_aerobuildup=LD_ab,
                swath_m=swath, gsd_m=gsd, area_per_flight_km2=area_km2)
    pd.Series({**pick.to_dict(), **geom}).to_csv(OUT / "design_point.csv", header=["value"])

    # ---------------------------------------------------------- charts
    BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})

    # Chart 1: endurance vs battery mass for the picked span/AR: the sweet spot
    d = df[(df.span_m == pick.span_m) & (np.isclose(df.AR, pick.AR))]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(d.m_batt, d.endurance_h, color=BLUE, lw=2)
    ax.axhline(A.endurance_h, color=MUTED, lw=1, ls="--")
    ax.text(d.m_batt.min(), A.endurance_h + 0.05, "3 h requirement", color=MUTED)
    ax.plot([pick.m_batt], [pick.endurance_h], "o", color=ORANGE, ms=9, mec="#fcfcfb", mew=2)
    ax.annotate(f"pick: {pick.m_batt:.2f} kg battery\n{pick.endurance_h:.2f} h, MTOW {pick.mtow:.1f} kg",
                (pick.m_batt, pick.endurance_h), xytext=(12, -32), textcoords="offset points", color=INK)
    ax.set_xlabel("Battery mass (kg)")
    ax.set_ylabel("Endurance (h)")
    ax.set_title(f"More battery, more endurance, with diminishing returns ({pick.span_m:.1f} m span, AR {pick.AR:.1f})",
                 color=INK, loc="left", fontsize=10)
    ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "endurance_vs_battery.png", dpi=150)

    # Chart 2: MTOW of the lightest passing design vs span
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(per_span.span_m, per_span.mtow, color=BLUE, lw=2, marker="o", ms=7)
    ax.set_xlabel("Span (m)")
    ax.set_ylabel("Take-off mass (kg)")
    ax.set_title("Lightest design meeting every requirement, by span", color=INK, loc="left")
    ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "mtow_vs_span.png", dpi=150)

    # Chart 3: power required vs speed for the pick
    W = pick.mtow * G
    q = 0.5 * RHO * V_SWEEP**2
    CL = W / (q * pick.S_m2)
    Re = RHO * V_SWEEP * pick.chord_m / MU
    cdp = cd_profile(np.minimum(CL, 1.2), Re) * A.profile_factor
    parts = {"airfoil drag": q * pick.S_m2 * cdp * V_SWEEP / A.eta_cruise,
             "induced drag": q * pick.S_m2 * CL**2 / (math.pi * A.e_oswald * pick.AR) * V_SWEEP / A.eta_cruise,
             "body drag": q * A.cda_body * V_SWEEP / A.eta_cruise}
    m = V_SWEEP >= pick.v_stall
    fig, ax = plt.subplots(figsize=(7, 4))
    for (k, v), col in zip(parts.items(), (BLUE, ORANGE, AQUA)):
        ax.plot(V_SWEEP[m], v[m], color=col, lw=2, label=k)
    tot = sum(parts.values()) + A.p_avionics_w
    ax.plot(V_SWEEP[m], tot[m], color=INK, lw=2, label="total incl. 25 W electronics")
    ax.axvline(pick.v_search, color=MUTED, lw=1, ls="--")
    ax.text(pick.v_search + 0.3, tot[m].max() * 0.55, "search speed\n" + f"{pick.v_search:.1f} m/s", color=MUTED)
    ax.set_xlabel("Airspeed (m/s)")
    ax.set_ylabel("Battery power (W)")
    ax.set_title("Where the power goes", color=INK, loc="left")
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(0.0, 1.0))
    ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "power_vs_speed.png", dpi=150)

    # Planform drawing
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for s in (1, -1):
        xs = [0, x_tip, x_tip + c_tip, c_root, 0]
        ys = [0, s * b / 2, s * b / 2, 0, 0]
        ax.fill(ys, xs, color=BLUE, alpha=0.15)
        ax.plot(ys, xs, color=BLUE, lw=2)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlabel("Span (m)")
    ax.set_ylabel("Length (m)")
    ax.set_title(f"Planform: {b:.1f} m span, {S:.2f} m2, 25 deg sweep", color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(OUT / "planform.png", dpi=150)
    print(f"\nCharts and CSVs written to {OUT.relative_to(ROOT)}")
