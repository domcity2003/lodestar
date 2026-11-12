"""Design v2: make Lodestar competitive without touching the airframe.

The airframe (span, planform, pod, winglets) is validated through milestones 1-3, so v2 only
changes what goes into the existing slots:
  1. Better cells in the same 30 wing-root slots (21700 format, 5 x 3 per side, 6S5P).
  2. A mission-swappable kit bay: the bay sits on the balance point, so it can carry either
     the 1.5 kg rescue kit or an 18-cell battery module of the same weight (search-only sortie)
     without retrimming.

For each configuration it computes the endurance two ways:
  - "published style": sea level, calm air, battery to 85% depth of discharge (how makers quote it)
  - "design case": 2000 m altitude, with the 10% landing reserve (the requirement)
...and over the wing-drag range from milestone 3 (laminar as NeuralFoil -> fully turbulent).
Then it re-runs the AVL stability checks (with the pod) for the new mass layouts.

Run:  uv run python scripts/design_v2.py
Writes: out/v2/*.csv, out/v2/run_log.txt
"""
from dataclasses import replace
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb

sys.path.insert(0, str(Path(__file__).parent))
import sizing as sz  # noqa: E402
import stability as st  # noqa: E402
import vsp_model as vm  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "v2"
OUT.mkdir(parents=True, exist_ok=True)
M3 = pd.read_csv(ROOT / "out" / "vsp" / "m3_config.csv", index_col=0)["value"]

# ---------------------------------------------------------------- cells (real products)
# Pack mass = cells / 0.88: nickel strip, BMS, balance leads, wrap, XT connectors (typical drone Li-ion packs 0.85-0.90).
PACK_EFF = 0.88
CELLS = {
    # name: (Wh per cell, g per cell, source / status)
    "v1 assumption (200 Wh/kg pack)": (None, None, "milestone 1 assumption"),
    "Molicel P50B": (18.0, 71.0, "Molicel datasheet: 5.0 Ah, 3.6 V, 18.0 Wh, 71 g max; in volume production"),
    "Amprius SiCore 6.3 Ah": (6.3 * 3.6, 72.0, "Amprius: 315 Wh/kg 21700, 3C; evaluation samples, 3.6 V nominal assumed"),
}
N_WING = 30   # 6S5P in the wing roots (milestone 3 layout)
N_BAY = 18    # 6S3P module in the kit bay, sized to weigh no more than kit + chute
M_KIT_TOTAL = 1.50 + 0.204

PF_LAMINAR = 1.15                                                  # sizing's profile factor
PF_TURB = 1.15 * float(M3["cd_wing_turbulent"]) / float(M3["cd_wing_sizing"])
PF_MID = (PF_LAMINAR + PF_TURB) / 2
CDA_BODY = float(M3["f_body_m2"])                                  # milestone 3 pod + gimbal


def pack(cell, n):
    wh, g, _ = CELLS[cell]
    if wh is None:  # v1: 2.55 kg at 200 Wh/kg
        return 510.0 * n / N_WING, 2.55 * n / N_WING
    return n * wh, n * g / 1000 / PACK_EFF


def at_altitude(alt):
    a = asb.Atmosphere(altitude=alt)
    sz.RHO, sz.MU = float(a.density()), float(a.dynamic_viscosity())


def endurance(cell, config, pf, alt, usable):
    """config: 'kit' (rescue kit in the bay) or 'search' (battery module in the bay)."""
    at_altitude(alt)
    wh_w, m_w = pack(cell, N_WING)
    a = replace(sz.A, profile_factor=pf, cda_body=CDA_BODY, usable_frac=usable)
    if config == "kit":
        m_batt, wh = m_w, wh_w
    else:
        wh_b, m_b = pack(cell, N_BAY)
        a = replace(a, m_kit=0.0, m_kit_chute=0.0)
        m_batt, wh = m_w + m_b, wh_w + wh_b
    a = replace(a, batt_wh_per_kg=wh / m_batt)
    r = sz.evaluate(3.0, 7.0, m_batt, a)
    return dict(m_batt=m_batt, batt_wh=wh, mtow=r["mtow"], v_stall=r["v_stall"], v_best=r["v_search"],
                p_w=r["p_search_w"], endurance_h=r["endurance_h"], m_prop=r["m_prop"])


if __name__ == "__main__":
    sys.stdout = st.Tee(OUT / "run_log.txt")
    print("Lodestar design v2, 2026-11-11, claude-opus-5-5\n")
    print("Cells (all 21700 format, so they drop into the milestone 3 slots):")
    for k, (wh, g, src) in CELLS.items():
        if wh:
            print(f"  {k}: {wh:.1f} Wh, {g:.0f} g -> cell {wh / g * 1000:.0f} Wh/kg, pack {wh / g * 1000 * PACK_EFF:.0f} Wh/kg  [{src}]")
    print(f"Wing-drag range: profile factor {PF_LAMINAR:.2f} (laminar) / {PF_MID:.2f} (halfway) / {PF_TURB:.2f} (turbulent)\n")

    rows = []
    for cell in CELLS:
        for config in ("kit", "search"):
            for case, alt, usable in (("published style (sea level, 85% DoD)", 0.0, 0.85),
                                      ("design case (2000 m, with 10% reserve)", 2000.0, 0.85 * 0.90)):
                for drag, pf in (("laminar", PF_LAMINAR), ("halfway", PF_MID), ("turbulent", PF_TURB)):
                    r = endurance(cell, config, pf, alt, usable)
                    rows.append(dict(cell=cell, config=config, case=case, drag=drag, **r))
    df = pd.DataFrame(rows)
    df.round(4).to_csv(OUT / "endurance_matrix.csv", index=False)

    piv = df.pivot_table(index=["cell", "config"], columns=["case", "drag"], values="endurance_h", sort=False)
    print("Endurance (h) by cell, bay configuration, case and wing drag:")
    with pd.option_context("display.width", 220):
        print(piv.round(2).to_string())
    mt = df[(df.drag == "halfway") & df.case.str.startswith("design")][["cell", "config", "m_batt", "batt_wh", "mtow", "v_stall"]]
    print("\nMasses (design case):")
    print(mt.round(3).to_string(index=False))

    # ------------------------------------------------------------ stability for the v2 layouts
    print("\nStability re-check in AVL with the pod (0.25 m winglets), balance point at 6% margin:")
    xr = [(0.0, 0.0)]
    for xl, w, h, rc in vm.POD_SECTIONS:
        xr.append((xl * vm.POD_LEN, math.sqrt((w * h - (4 - math.pi) * rc**2) / math.pi)))
    xr.append((vm.POD_LEN, 0.0))
    g = st.Geometry(c_root=vm.C_ROOT, c_tip=vm.C_TIP, washout_deg=vm.WASHOUT, winglet_h=0.25,
                    body_xr=tuple(xr), body_x0=vm.POD_X0, body_z0=vm.POD_Z)
    x_cg = float(M3["x_cg"])
    srows = []
    for cell in ("Molicel P50B", "Amprius SiCore 6.3 Ah"):
        _, m_w = pack(cell, N_WING)
        _, m_b = pack(cell, N_BAY)
        for config, bay_mass, kit_on, V in (("kit, search speed", M_KIT_TOTAL, True, 15.0),
                                            ("kit dropped, top speed", M_KIT_TOTAL, False, 22.0),
                                            ("kit dropped, slow", M_KIT_TOTAL, False, 12.5),
                                            ("battery module in bay, search speed", m_b, True, 15.0)):
            st.M_BATT, st.M_KIT = m_w, bay_mass
            r = st.analyse(g, x_cg, V=V, kit_on=kit_on)
            srows.append(dict(cell=cell, config=config, V=V, mass=r["mass"], x_batt=r["x_batt"], x_cg=r["x_cg"],
                              static_margin=r["static_margin"], elevon=r["elevon"],
                              short_period_zeta=r.get("short_period_zeta"), phugoid_zeta=r.get("phugoid_zeta"),
                              dutch_roll_zeta=r.get("dutch_roll_zeta"), spiral_t2=r.get("spiral_t2"),
                              roll_tau=r.get("roll_tau"), pitch_ok=r.get("pitch_ok"), lateral_ok=r.get("lateral_ok")))
    sdf = pd.DataFrame(srows)
    sdf.round(4).to_csv(OUT / "stability_v2.csv", index=False)
    with pd.option_context("display.width", 220):
        print(sdf.drop(columns=["V"]).round(3).to_string(index=False))
    print(f"\nAll cases pass: {bool(sdf.pitch_ok.all() and sdf.lateral_ok.all())}")
