"""Design v3, lever 1: match a real propeller and motor instead of assuming 55% efficiency.

Where v2's power goes at search speed (2,000 m, kit on): 136 W from the battery, of which
about 50 W is lost in the motor, ESC and propeller (v2 assumed an overall 55% efficiency).
That's the biggest single loss, bigger than the wing's profile drag.

Method:
  - Propellers: measured wind-tunnel data (thrust and power coefficients vs advance ratio J,
    at several RPMs) from the UIUC Propeller Data Site, volumes 3 and 4:
      * Aeronaut CAM folding props (Dantsker, Caccamo, Deters & Selig, AIAA 2020-2762)
      * APC Thin Electric (rigid) props, as a reference
    A belly-landing pusher needs a folding prop (a rigid prop strikes the ground), so only the
    CAM props are eligible for the pick.
  - Motors: the standard three-constant brushless model (Kv, winding resistance Rm, no-load
    current I0) with values from the makers' spec pages (Scorpion SII-30/40 series,
    T-Motor AT3520).
  - Thrust needed = drag from the same model v2 uses (NeuralFoil MH45 + induced + pod), at
    2,000 m, so v2 and v3 are compared on equal terms.
  - Checks at full power, on a sagging battery (6 x 3.3 V): reach 22 m/s level, and climb
    3 m/s at 1.25 x stall speed.

Run:  uv run python scripts/v3_propulsion.py
Writes: out/v3/propulsion_*.csv, propulsion_map.png, propulsion_log.txt
"""
from pathlib import Path
import math
import re
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import sizing as sz  # noqa: E402
import design_v2 as d2  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "uiuc_props"
OUT = ROOT / "out" / "v3"
OUT.mkdir(parents=True, exist_ok=True)
G = 9.81
IN = 0.0254

ATM = asb.Atmosphere(altitude=2000.0)
RHO, MU = float(ATM.density()), float(ATM.dynamic_viscosity())
S, CHORD, AR = 3.0**2 / 7.0, 3.0 / 7.0, 7.0

# ---------------------------------------------------------------- motors (maker spec pages, Nov 2026)
# name: (Kv rpm/V, I0 A at 10 V, Rm ohm, mass kg, max continuous W)
MOTORS = {
    "Scorpion SII-3020-780": (780, 1.21, 0.0322, 0.166, 800),
    "Scorpion SII-3026-710": (710, 1.56, 0.022, 0.205, 1000),
    "T-Motor AT3520-550": (550, 1.10, 0.031, 0.218, 1100),
    "Scorpion SII-4020-420": (420, 0.91, 0.032, 0.288, 1500),
    "Scorpion SII-4025-330": (330, 0.74, 0.037, 0.353, 2000),
}
ESC_ETA = 0.95          # part-throttle ESC efficiency, ASSUMPTION (0.93-0.97 typical)
ESC_KG = 0.06           # 60 A class ESC, ASSUMPTION
INSTALL = 0.95          # pusher behind the pod and wing root: prop efficiency x 0.95, ASSUMPTION
N_S = 6
V_CELL_NOM, V_CELL_SAG = 3.6, 3.3   # P50B: nominal; loaded near the end of a flight
P_AV = sz.A.p_avionics_w


def prop_mass(name, d_in):
    """Prop + hub/spinner mass, ASSUMPTION: folding CAM blades + 40 mm hub + spinner, rigid APC E."""
    k = 0.070 if name.startswith("ancf") else 0.045
    return k * (d_in / 16) ** 2


# ---------------------------------------------------------------- propeller data
def load_props():
    props = {}
    for f in sorted(DATA.glob("*_*od_*.txt")):
        m = re.match(r"(ancf|apce)_(\d+)x(\d+)_\d+od_(\d+)\.txt", f.name)
        if not m:
            continue
        fam, d, p, rpm = m.group(1), m.group(2), m.group(3), int(m.group(4))
        d_in = float(d) / 10 if len(d) == 3 else float(d)          # "125" = 12.5 in
        key = f"{fam}_{d}x{p}"
        t = pd.read_csv(f, sep=r"\s+", comment="R")      # one file starts with an "RPM average" line
        if "J" not in t.columns or len(t) < 4:
            continue
        props.setdefault(key, dict(fam=fam, d_in=d_in, runs=[]))["runs"].append((rpm, t.sort_values("J")))
    return props


def coeffs(prop, rpm, J):
    """CT, CP at advance ratio J, from the measured run(s) nearest in RPM (Reynolds number).
    Below the lowest tested RPM, uses the lowest run (slightly optimistic: lower Re is worse)."""
    if "groups" not in prop:
        # UIUC splits each RPM into a low-J and a high-J tunnel run (e.g. 3002 and 3042 rpm): merge runs
        # within 150 rpm into one curve per RPM group
        groups = []
        for r, t in sorted(prop["runs"], key=lambda x: x[0]):
            if groups and r - groups[-1][0] <= 150:
                groups[-1][1].append(t)
            else:
                groups.append([r, [t]])
        prop["groups"] = [(r, pd.concat(ts).sort_values("J").drop_duplicates("J")) for r, ts in groups]
    cands = [(abs(r - rpm), t) for r, t in prop["groups"] if t.J.min() <= J <= t.J.max()]
    if not cands:
        return np.nan, np.nan
    t = min(cands, key=lambda c: c[0])[1]     # nearest tested RPM whose data covers this J
    return float(np.interp(J, t.J, t.CT)), float(np.interp(J, t.J, t.CP))


def prop_point(prop, V, T):
    """RPM, shaft power and torque that give thrust T at airspeed V (nan if outside the data)."""
    D = prop["d_in"] * IN
    rpms = np.arange(1500, 9001, 20)
    best = None
    prev = None
    for rpm in rpms:
        n = rpm / 60
        ct, cp = coeffs(prop, rpm, V / (n * D))
        if np.isnan(ct):
            prev = None
            continue
        thrust = ct * RHO * n**2 * D**4          # installation loss is applied to the power below
        if prev is not None and prev[1] < T <= thrust:
            f = (T - prev[1]) / (thrust - prev[1])
            rpm_s = prev[0] + f * (rpm - prev[0])
            p = prev[2] + f * (cp * RHO * n**3 * D**5 - prev[2])
            best = (rpm_s, p / INSTALL)
            break
        prev = (rpm, thrust, cp * RHO * n**3 * D**5)
    if best is None:
        return None
    rpm_s, P = best
    w = rpm_s * 2 * math.pi / 60
    return dict(rpm=rpm_s, P_shaft=P, Q=P / w, eta_prop=T * V / P)


def motor_point(motor, rpm, Q):
    kv, i0, rm, _, _ = MOTORS[motor]
    kv_rad = kv * 2 * math.pi / 60
    w = rpm * 2 * math.pi / 60
    I = Q * kv_rad + i0
    U = w / kv_rad + I * rm
    return dict(I=I, U=U, P_elec=U * I, eta_motor=Q * w / (U * I))


# ---------------------------------------------------------------- aircraft drag (same model as v2)
def drag(V, m, pf=d2.PF_MID):
    q = 0.5 * RHO * V**2
    CL = m * G / (q * S)
    Re = RHO * V * CHORD / MU
    CD = float(sz.cd_profile(min(CL, 1.2), Re)) * pf + CL**2 / (math.pi * sz.A.e_oswald * AR) + d2.CDA_BODY / S
    return q * S * CD


def system(prop, motor, V, T):
    pp = prop_point(prop, V, T)
    if pp is None:
        return None
    mp = motor_point(motor, pp["rpm"], pp["Q"])
    P_batt = mp["P_elec"] / ESC_ETA
    return dict(**pp, **mp, P_batt=P_batt, eta_sys=T * V / P_batt)


if __name__ == "__main__":
    sys.stdout = Tee(OUT / "propulsion_log.txt")
    print("Lodestar v3, propulsion matching, 2026-11-12\n")
    props = load_props()
    print(f"{len(props)} propellers with wind-tunnel data: "
          f"{sum(p['fam'] == 'ancf' for p in props.values())} Aeronaut CAM folding, "
          f"{sum(p['fam'] == 'apce' for p in props.values())} APC Thin Electric")

    m_v2 = 7.301                      # v2 P50B kit-on take-off mass, design case
    m_prop_v2 = 0.280                 # v2 propulsion mass (sizing rule 2 kW/kg)
    v_stall = math.sqrt(2 * m_v2 * G / (RHO * S * sz.A.cl_max_3d))
    V_S = 14.8
    cases = {  # name: (V, thrust)
        "search, kit on": (V_S, drag(V_S, m_v2)),
        "search, kit dropped": (V_S, drag(V_S, m_v2 - d2.M_KIT_TOTAL)),
        "transit 18 m/s": (18.0, drag(18.0, m_v2)),
    }
    v_cl = 1.25 * v_stall
    max_cases = {"top speed 22 m/s": (22.0, drag(22.0, m_v2)),
                 "climb 3 m/s": (v_cl, drag(v_cl, m_v2) + m_v2 * G * 3.0 / v_cl)}
    for k, (V, T) in {**cases, **max_cases}.items():
        print(f"  {k:<20} V {V:5.1f} m/s  thrust needed {T:5.2f} N  (thrust power {T * V:5.1f} W)")
    print()

    rows = []
    for pk, prop in props.items():
        for mk in MOTORS:
            r = dict(prop=pk, folding=prop["fam"] == "ancf", d_in=prop["d_in"], motor=mk)
            ok = True
            for cname, (V, T) in {**cases, **max_cases}.items():
                s = system(prop, mk, V, T)
                if s is None:
                    ok = False
                    break
                tag = cname.split(",")[0].split(" ")[0] if cname not in cases else cname
                if cname in cases:
                    r[f"P_batt [{cname}]"] = s["P_batt"]
                    r[f"eta_sys [{cname}]"] = s["eta_sys"]
                    if cname == "search, kit on":
                        r.update(rpm_search=s["rpm"], eta_prop=s["eta_prop"], eta_motor=s["eta_motor"],
                                 I_search=s["I"], U_search=s["U"])
                else:
                    r[f"U [{cname}]"] = s["U"]
                    r[f"P_elec [{cname}]"] = s["P_elec"]
                    r[f"rpm [{cname}]"] = s["rpm"]
            if not ok:
                continue
            u_max = max(r["U [top speed 22 m/s]"], r["U [climb 3 m/s]"])
            p_max = max(r["P_elec [top speed 22 m/s]"], r["P_elec [climb 3 m/s]"])
            r["U_needed_max"] = u_max
            r["volt_ok"] = u_max <= 0.95 * N_S * V_CELL_SAG
            r["power_ok"] = p_max <= MOTORS[mk][4]
            r["mass_kg"] = MOTORS[mk][3] + ESC_KG + prop_mass(prop["fam"], prop["d_in"])
            rows.append(r)
    df = pd.DataFrame(rows)
    df["feasible"] = df.volt_ok & df.power_ok
    df.round(4).to_csv(OUT / "propulsion_all.csv", index=False)
    # Rank by what matters, endurance: heavier propulsion raises the power needed (about mass^1.5 at fixed speed)
    df["P_equiv"] = df["P_batt [search, kit on]"] * ((m_v2 - m_prop_v2 + df.mass_kg) / m_v2) ** 1.5
    feas = df[df.feasible].sort_values("P_equiv")
    print(f"{len(df)} prop + motor pairs cover every flight condition in the data; {len(feas)} can also reach "
          f"top speed and climb on a sagging 6S battery\n")
    cols = ["prop", "d_in", "motor", "rpm_search", "eta_prop", "eta_motor", "eta_sys [search, kit on]",
            "P_batt [search, kit on]", "P_batt [search, kit dropped]", "U_needed_max", "mass_kg", "P_equiv"]
    print("Best 12 overall (P_equiv = search power at the battery, corrected for the propulsion mass):")
    print(feas[cols].head(12).round(3).to_string(index=False))
    fold = feas[feas.folding]
    print("\nBest 8 folding (eligible for belly landing):")
    print(fold[cols].head(8).round(3).to_string(index=False))

    # v2 reference: the 13 in folding prop in the milestone 5 CAD, with the best motor for it
    ref = feas[feas.prop.str.startswith("ancf_13x")].head(3)
    print("\nFor reference, best 13 in folding props (the size drawn in the milestone 5 CAD):")
    print(ref[cols].round(3).to_string(index=False))

    pick = fold.iloc[0]
    print(f"\nPick: {pick.prop} folding prop + {pick.motor}: system efficiency "
          f"{pick['eta_sys [search, kit on]']:.1%} at search (v2 assumed 55%), "
          f"{pick['P_batt [search, kit on]']:.1f} W at the battery incl. no avionics")
    v2_P = cases["search, kit on"][1] * V_S / 0.55
    print(f"  v2 assumption: {v2_P:.1f} W for the same thrust; saving {v2_P - pick['P_batt [search, kit on]']:.1f} W "
          f"({(v2_P - pick['P_batt [search, kit on]']) / (v2_P + P_AV):.1%} of total power incl. 25 W avionics)")
    print(f"  propulsion mass {pick.mass_kg:.3f} kg vs v2 {m_prop_v2:.3f} kg")
    pd.Series(pick).to_csv(OUT / "propulsion_pick.csv", header=["value"])

    # efficiency map of the pick vs airspeed (used by the v3 mission model)
    prop = props[pick.prop]
    rows = []
    for V in np.arange(12.0, 26.01, 0.5):
        for kit_on in (True, False):
            m = m_v2 - (0 if kit_on else d2.M_KIT_TOTAL)
            s = system(prop, pick.motor, V, drag(V, m))
            if s:
                rows.append(dict(V=V, kit_on=kit_on, eta_sys=s["eta_sys"], eta_prop=s["eta_prop"],
                                 eta_motor=s["eta_motor"], rpm=s["rpm"]))
    em = pd.DataFrame(rows)
    em.round(4).to_csv(OUT / "propulsion_eta_map.csv", index=False)

    # chart: search power at the battery for the folding props, best motor each, vs diameter
    BLUE, ORANGE, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#1f1f1e", "#6b6a64", "#e4e3de"
    best_each = feas.groupby("prop").head(1)
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    for fam, col, lab in (("ancf", BLUE, "Aeronaut CAM folding"), ("apce", MUTED, "APC Thin Electric (rigid, reference)")):
        d = best_each[best_each.prop.str.startswith(fam)]
        ax.scatter(d.d_in, d["eta_sys [search, kit on]"] * 100, s=40, color=col, label=lab, edgecolor="#fcfcfb", linewidth=1.5)
    ax.axhline(55, color=ORANGE, lw=1.5, ls="--")
    ax.text(12.0, 55.6, "v2 assumption: 55%", color=INK)
    ax.scatter([pick.d_in], [pick["eta_sys [search, kit on]"] * 100], s=140, facecolor="none", edgecolor=INK, lw=1.5)
    ax.annotate(f"pick: {pick.prop.replace('ancf_', 'CAM ')}\n{pick.motor}", (pick.d_in, pick["eta_sys [search, kit on]"] * 100),
                xytext=(-120, 8), textcoords="offset points", color=INK)
    ax.set_xlabel("Propeller diameter (in)")
    ax.set_ylabel("Battery-to-thrust efficiency at search (%)")
    ax.set_title("Measured props with a matched motor: the best folding props reach 62-64% (best motor for each prop)", color=INK, loc="left", fontsize=10)
    ax.grid(axis="y", color=GRID)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(OUT / "propulsion_map.png", dpi=150)
    print("\nWritten: out/v3/propulsion_all.csv, propulsion_pick.csv, propulsion_eta_map.csv, propulsion_map.png")
