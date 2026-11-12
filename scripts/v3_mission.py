"""Design v3: stability re-check and the milestone 4 mission simulation, v3 vs v2 vs competitors.

v3 = v2 airframe and cells, plus:
  1. matched propulsion: Aeronaut CAM 15x10 folding prop + T-Motor AT3520-550 (v3_propulsion.py),
     power from measured prop data and the motor model at every airspeed (not a fixed efficiency)
  2. moulded laminar-quality wing skins (the "laminar" end of the milestone 3 drag range)
Option (needs a requirement change): 3.4 m span, aspect ratio 9, same wing area (v3_planform.py).

Same missions, winds, targets and detection draws as milestone 4 (same random seed).

Run:  uv run python scripts/v3_mission.py
Writes: out/v3/stability_v3.csv, mission_summary.csv, mission_runs.csv, p_help.png, mission_log.txt
"""
from dataclasses import replace
from pathlib import Path
import math
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
import stability as st  # noqa: E402
import vsp_model as vm  # noqa: E402
import mission_sim as ms  # noqa: E402
import v3_propulsion as vp  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "v3"
PICK = pd.read_csv(OUT / "propulsion_pick.csv", index_col=0)["value"]
M_PROP_V3 = float(PICK["mass_kg"])
PROPS = vp.load_props()
PROP = PROPS[PICK["prop"]]
MOTOR = PICK["motor"]


def mass_and_energy(cell, config, b, AR, pf):
    """Take-off mass from the sizing model with v3 propulsion mass; wing structure scaled by span/3."""
    d2.at_altitude(0.0)
    wh_w, m_w = d2.pack(cell, d2.N_WING)
    a = replace(sz.A, profile_factor=pf, cda_body=d2.CDA_BODY, wing_kg_per_m2=1.0 * b / 3.0,
                prop_kw_per_kg=1e9, m_centerbody=sz.A.m_centerbody + M_PROP_V3 - 0.05)
    if config == "kit":
        m_b, wh = m_w, wh_w
    else:
        wh_b, m_bb = d2.pack(cell, d2.N_BAY)
        a = replace(a, m_kit=0.0, m_kit_chute=0.0)
        m_b, wh = m_w + m_bb, wh_w + wh_b
    a = replace(a, batt_wh_per_kg=wh / m_b)
    r = sz.evaluate(b, AR, m_b, a)
    return r["mtow"], wh


def rescue_wing_v3(name, cell, config, finish="laminar", b=3.0, AR=7.0, alt=0.0):
    pf = {"laminar": d2.PF_LAMINAR, "halfway": d2.PF_MID, "turbulent": d2.PF_TURB}[finish]
    mtow, wh = mass_and_energy(cell, config, b, AR, pf)
    atm = asb.Atmosphere(altitude=alt)
    rho, mu = float(atm.density()), float(atm.dynamic_viscosity())
    S = b**2 / AR
    cfg = dict(rho=rho, mu=mu, S=S, chord=b / AR, AR=AR, pf=pf, e=sz.A.e_oswald, cda=d2.CDA_BODY,
               eta=1.0, p_av=0.0)      # eta 1, no avionics: gives the thrust power, propulsion applied below
    v_stall = math.sqrt(2 * mtow * ms.G / (rho * S * sz.A.cl_max_3d))
    kit = config == "kit"
    ac = ms.Aircraft(name, 15.0, 1.25 * v_stall, 22.0, wh * 0.85, kit=kit, rw_cfg=cfg, mass=mtow,
                     kit_mass=d2.M_KIT_TOTAL if kit else 0.0,
                     note=f"v3: {cell}, bay: {'rescue kit' if kit else 'battery module'}, {finish} wing, "
                          f"{b:.1f} m span, CAM 15x10 + AT3520")
    vt = np.linspace(0.9 * ac.v_min, 30.0, 50)
    vp.RHO, vp.MU = rho, mu
    tabs = []
    for kit_on in (True, False):
        p = []
        for V in vt:
            T = float(ac.power(V, kit_on)) / V              # thrust = drag
            s = vp.system(PROP, MOTOR, V, T)
            p.append(s["P_batt"] + sz.A.p_avionics_w if s else np.nan)
        p = pd.Series(p).interpolate(limit_direction="both").values
        tabs.append(p)
    ac._tab = (vt, tabs[0], tabs[1])
    vg = np.linspace(ac.v_min, ac.v_max, 200)
    pv = ac.power(vg)
    ac.endurance_h = ac.energy / float(pv.min())
    ac.v_cruise = float(vg[np.argmin(pv / vg)])
    ac.search_h = ac.energy / float(ac.power(ac.v_cruise))
    return ac


def design_case_endurance(ac):
    """2,000 m, 10% reserve on top of the 85% depth of discharge (the requirement)."""
    a2 = rescue_wing_v3(ac.name, *ac._args, alt=2000.0)
    vg = np.linspace(a2.v_min, 20.0, 120)
    vg = vg[vg >= 14.0]
    return a2.energy * 0.90 / float(a2.power(vg).min()), a2


if __name__ == "__main__":
    sys.stdout = Tee(OUT / "mission_log.txt")
    print("Rescue Wing v3: stability re-check and mission simulation, 2026-11-12\n")

    # ------------------------------------------------------------ stability (AVL, with the pod)
    st.DP = st.DP.astype(object)
    st.DP["m_prop"] = M_PROP_V3
    xr = [(0.0, 0.0)]
    for xl, w, h, rc in vm.POD_SECTIONS:
        xr.append((xl * vm.POD_LEN, math.sqrt((w * h - (4 - math.pi) * rc**2) / math.pi)))
    xr.append((vm.POD_LEN, 0.0))
    rows = []
    for label, b, AR in (("v3, 3.0 m span", 3.0, 7.0), ("v3 option, 3.4 m span", 3.4, 9.0)):
        S = b**2 / AR
        taper = 0.45
        c_root = 2 * S / (b * (1 + taper))
        g = st.Geometry(span=b, c_root=c_root, c_tip=taper * c_root, washout_deg=vm.WASHOUT, winglet_h=0.25,
                        body_xr=tuple(xr), body_x0=vm.POD_X0, body_z0=vm.POD_Z)
        st.M_WING = 1.0 * S * b / 3.0
        _, m_w = d2.pack("Molicel P50B", d2.N_WING)
        st.M_BATT, st.M_KIT = m_w, d2.M_KIT_TOTAL
        x_np = st.analyse(g, 0.40, modes=False)["x_np"]
        x_cg = x_np - 0.06 * g.mac
        for case, kit_on, V in (("kit, search", True, 15.0), ("kit dropped, top speed", False, 22.0),
                                ("kit dropped, slow", False, 12.5), ("kit, top speed", True, 22.0)):
            r = st.analyse(g, x_cg, V=V, kit_on=kit_on)
            rows.append(dict(design=label, case=case, mass=r["mass"], x_cg=r["x_cg"], x_np=r["x_np"],
                             static_margin=r["static_margin"], elevon=r["elevon"], alpha=r["alpha"],
                             sp_zeta=r.get("short_period_zeta"), ph_zeta=r.get("phugoid_zeta"),
                             dr_zeta=r.get("dutch_roll_zeta"), spiral_t2=r.get("spiral_t2"),
                             roll_tau=r.get("roll_tau"), pitch_ok=r.get("pitch_ok"), lateral_ok=r.get("lateral_ok")))
    sdf = pd.DataFrame(rows)
    sdf.round(4).to_csv(OUT / "stability_v3.csv", index=False)
    print("Stability (AVL with the pod, 0.25 m winglets, 6% static margin, MIL-F-8785C Class I Level 1):")
    with pd.option_context("display.width", 220):
        print(sdf.round(3).to_string(index=False))
    print(f"All cases pass: {bool(sdf.pitch_ok.all() and sdf.lateral_ok.all())}\n")

    # ------------------------------------------------------------ fleet
    specs = [
        ("Rescue Wing v3 (P50B, kit)", ("Molicel P50B", "kit", "laminar", 3.0, 7.0)),
        ("Rescue Wing v3, halfway finish (P50B, kit)", ("Molicel P50B", "kit", "halfway", 3.0, 7.0)),
        ("Rescue Wing v3+ (SiCore, kit)", ("Amprius SiCore 6.3 Ah", "kit", "laminar", 3.0, 7.0)),
        ("Rescue Wing v3+ search-only", ("Amprius SiCore 6.3 Ah", "search", "laminar", 3.0, 7.0)),
        ("Rescue Wing v3 option 3.4 m (P50B, kit)", ("Molicel P50B", "kit", "laminar", 3.4, 9.0)),
        ("Rescue Wing v3+ option 3.4 m (SiCore, kit)", ("Amprius SiCore 6.3 Ah", "kit", "laminar", 3.4, 9.0)),
    ]
    v3 = []
    for name, args in specs:
        ac = rescue_wing_v3(name, *args)
        ac._args = args
        v3.append(ac)
    v2 = [ms.rescue_wing("Rescue Wing v2 (P50B, kit)", "Molicel P50B", "kit"),
          ms.rescue_wing("Rescue Wing v2+ (SiCore, kit)", "Amprius SiCore 6.3 Ah", "kit")]
    fleet = v3 + v2 + ms.COMPETITORS + [ms.FRONTIER]

    print("Endurance, design case (2,000 m, 10% reserve, kit carried throughout, searching at >= 14 m/s):")
    erows = []
    for ac in v3:
        e, a2 = design_case_endurance(ac)
        erows.append(dict(aircraft=ac.name, mtow=ac.mass, published_style_h=ac.endurance_h, design_case_h=e))
        print(f"  {ac.name:44s} MTOW {ac.mass:5.2f} kg | published-style {ac.endurance_h:4.2f} h | design case {e:4.2f} h")
    for ac, wh_name in zip(v2, ("Molicel P50B", "Amprius SiCore 6.3 Ah")):
        e = d2.endurance(wh_name, "kit", d2.PF_MID, 2000.0, 0.85 * 0.9)["endurance_h"]
        erows.append(dict(aircraft=ac.name, mtow=ac.mass, published_style_h=ac.endurance_h, design_case_h=e))
        print(f"  {ac.name:44s} MTOW {ac.mass:5.2f} kg | published-style {ac.endurance_h:4.2f} h | design case {e:4.2f} h")
    pd.DataFrame(erows).round(3).to_csv(OUT / "endurance_v3.csv", index=False)

    rng = np.random.default_rng(ms.RNG_SEED)
    N = 2000
    frames = [ms.run_scenario(fleet, sc, box, N, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
    df = ms.add_help_time(pd.concat(frames, ignore_index=True))
    df.to_csv(OUT / "mission_runs.csv", index=False)
    summ = ms.summarise(df, fleet)
    summ.round(3).to_csv(OUT / "mission_summary.csv", index=False)
    for sc in ms.SCENARIOS:
        print(f"\n{sc}  ({N} missions each, same targets and winds for every aircraft, sea level)")
        s = summ[summ.scenario == sc].drop(columns="scenario")
        with pd.option_context("display.width", 220):
            print(s.round(2).to_string(index=False))

    # ------------------------------------------------------------ chart: help within 3 h, scenario A
    BLUE, LIGHT, GREY, INK, MUTED, GRID, BG = "#2a78d6", "#86b6ef", "#b4b2a9", "#1f1f1e", "#6b6a64", "#e4e3de", "#fcfcfb"
    sc = list(ms.SCENARIOS)[0]
    s = summ[summ.scenario == sc]
    order = ["Rescue Wing v3+ (SiCore, kit)", "Rescue Wing v2+ (SiCore, kit)", "Rescue Wing v3 (P50B, kit)",
             "Rescue Wing v2 (P50B, kit)", "DeltaQuad Evo GOV", "DeltaQuad Evo-LE (press)", "C-Astral Bramor ppX",
             "Quantum Trinity Pro", "AgEagle eBee X"]
    s = s.set_index("aircraft").loc[order].reset_index().iloc[::-1]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    cols = [BLUE if "v3" in n else LIGHT if "v2" in n else GREY for n in s.aircraft]
    ax.barh(s.aircraft, s.p_help_3h * 100, color=cols, height=0.62)
    for yi, v in enumerate(s.p_help_3h):
        ax.text(v * 100 + 0.8, yi, f"{v:.0%}", va="center", color=INK, fontsize=9)
    ax.set_xlabel("Missions where help reaches the person within 3 h (%)")
    ax.set_xlim(0, max(s.p_help_3h) * 100 + 10)
    ax.grid(axis="x", color=GRID)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_title("Scenario A (lost hiker, 16 km2, 5 km out): v3 (dark blue) vs v2 (light blue) vs fielded drones",
                 color=INK, loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "p_help.png", dpi=160)
    print("\nWritten: out/v3/stability_v3.csv, endurance_v3.csv, mission_summary.csv, mission_runs.csv, p_help.png")
