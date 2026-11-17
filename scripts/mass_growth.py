"""Lodestar v1 mass properties with a mass growth allowance (Step 2, gap G7).

Every line of the v1 mass balance is given a maturity (Estimated, Calculated, Measured/datasheet) and a
hardware category, and gets the growth allowance (MGA) that the schedule below sets for that pair:

    basic mass + MGA = predicted mass;   margin = 10 kg MTOW limit - predicted mass

The basic mass is the frozen v1 mass balance (`lodestar_v1.MASS`), rebuilt line by line from
`out/cad_v4/mass_cg_sicore.csv` with the CAD budget wing replaced by the spanwise wing model
(`out/wing_trade/summary_v1.csv`), exactly as `wing_trade.mass_properties` does.

CG under growth is shown three ways: every line grows at its own centroid; only the lines aft of the CG
grow (aft-biased); and the whole MGA lumped at the motor station (bounding). Static margin uses the AVL
neutral point of each mass state at Vc, sea level (the lowest-margin condition, `out/controls/mass_states.csv`).

MGA schedule sources:
- Estimated column: AIAA S-120-2006 MGA at "Estimated" maturity, as reproduced by MacKenzie and Burgess
  (ICEAA 2016, "Satellite Mass Growth"). The S-120A-2015 Table 1 itself is not openly published.
- Calculated column: ASSUMED taper between Estimated and Measured (about 0.4-0.5 x Estimated).
- Measured / datasheet: 3%, the order of the "existing hardware" (A5) allowance in the NASA MSFC 2013
  SAWE briefing; weighed parts carry only measurement uncertainty.
- The rescue kit is packed and weighed to its 1.50 kg requirement before each flight (0%). The unused part
  of the 0.6 kg gimbal budget is an allocation, not hardware (0%).

Run:  .venv/Scripts/python.exe -X utf8 scripts/mass_growth.py      (from the repo root)
Writes out/mass_growth/: lines.csv, summary.csv, schedule.csv, run_log.txt. Exit code 1 if a self-check fails.
"""
import re
import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import lodestar_v1 as v1  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "mass_growth"
LOG = []
FAILED = []


def say(text=""):
    print(text)
    LOG.append(text)


def check(name, ok, detail):
    say(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    if not ok:
        FAILED.append(name)


# ---------------------------------------------------------------- MGA schedule [% of basic mass]
SCHEDULE = {
    #             Estimated  Calculated  Measured/datasheet
    "structure":   (25.0,      10.0,       3.0),
    "mechanism":   (25.0,      10.0,       3.0),
    "electronics": (30.0,      15.0,       3.0),
    "battery":     (25.0,      10.0,       3.0),
    "harness":     (60.0,      30.0,       3.0),
    "propulsion":  (25.0,      10.0,       3.0),
    "thermal":     (25.0,      10.0,       3.0),
}
MATURITY = ("Estimated", "Calculated", "Measured")
CONTROLLED = 0.0       # weighed to a fixed number before flight, or an allocation that is not hardware

# ---------------------------------------------------------------- datasheet masses used to split lines
CELL_G = {"SiCore": 72.0, "P50B": 71.0}   # Amprius SA112 catalog 72 g; Molicel P50B datasheet 71 g max
N_CELLS = 30
MOTOR_KG = 0.218                          # T-Motor AT3520 KV550 spec page, 218 g
GIMBAL_DATASHEET_KG = 0.197               # SIYI ZT6 with anti-vibration board, SIYI spec page


PARA_ITEM = "aircraft parachute (RC1, ASTM F3322) + independent trigger"
V11_CATEGORY = {
    "backup avionics supply + locking connectors (RC2)": "electronics",
    "BMS, heater 50 C cut-out, cell liner (RC3)": "electronics",
    "ADS-B In receiver + anti-collision strobes (RC8)": "electronics",
    "second GNSS + antenna (RC10)": "electronics",
    "8 AWG pack leads (increment over 10 AWG)": "harness",
    "70 mm EPP belly skid (increment over 4 mm pad)": "structure",
}


def servo_mass():
    """structures.M_SERVO, read from the source so the heavy aero imports are not needed."""
    text = (ROOT / "scripts" / "structures.py").read_text(encoding="utf-8")
    return float(re.search(r"^M_SERVO\s*=\s*([0-9.]+)", text, re.M).group(1))


def lines(cell):
    """The mass balance as classified lines: (item, mass, x, maturity, category, basis)."""
    cad = pd.read_csv(ROOT / "out" / "cad_v4" / ("mass_cg_sicore.csv" if cell == "SiCore" else "mass_cg.csv"))
    cad = cad.set_index("item")
    summ = pd.read_csv(ROOT / "out" / "wing_trade" / "summary_v1.csv").iloc[0]
    wing, joints, servo = float(summ.wing_mass_kg), float(summ.joints_kg), servo_mass()
    allowance = 2 * v1.STRUCTURE["joint_allowance_kg"]
    batt_row = [i for i in cad.index if i.startswith("batteries")][0]
    pack = float(cad.loc[batt_row, "mass_kg"])
    cells = N_CELLS * CELL_G[cell] / 1000
    g = lambda item: (float(cad.loc[item, "mass_kg"]), float(cad.loc[item, "x"]))  # noqa: E731
    x_w = None   # wing x is solved below from the mass-balance CG
    rows = [
        ("wing shell, core, caps, elevons (spanwise model)", wing - joints - 2 * servo, x_w, "Calculated", "structure",
         "structures.size_wing strip masses; FE-checked; skin coupon T3 open"),
        ("wing joints: tubes, sleeves, ribs", joints - allowance, x_w, "Calculated", "structure",
         "CAD v4 volumes x density (wing_trade joint model)"),
        ("wing joint allowance (blocks, wraps, key, adhesive)", allowance, x_w, "Estimated", "structure",
         "lodestar_v1 45 g per break"),
        ("elevon servos (2)", 2 * servo, x_w, "Estimated", "mechanism",
         "structures.M_SERVO 30 g each; KST A12-S HV chosen in v1.1, datasheet mass to confirm"),
        ("winglets (2)", *g("winglets"), "Estimated", "structure", "area x 0.8 kg/m2 (cad_v4 budget)"),
        (f"cells, {cell} x 30", cells, g(batt_row)[1], "Measured", "battery", "cell datasheet mass"),
        ("pack build: nickel, BMS, leads, wrap", pack - cells, g(batt_row)[1], "Estimated", "battery",
         "cells / 0.88 rule (design_v2.PACK_EFF)"),
        ("pack thermal kit (foam + heater)", *g("pack thermal kit (foam + heater)"), "Estimated", "thermal",
         "upper end of the 50-100 g estimate"),
        ("elevon mass balance (2)", *g("elevon mass balance (2 per aircraft)"), "Calculated", "structure",
         "flutter_v2 static balance 2.23 g m on a 40 mm arm"),
        ("rescue kit", *g("rescue kit"), "Measured", None, "requirement mass; packed and weighed to 1.50 kg"),
        ("kit parachute", *g("kit parachute"), "Calculated", "mechanism", "drop study: canopy, lines, swivel"),
        ("kit container + release servo", *g("kit container + release"), "Estimated", "mechanism", "budget"),
        ("gimbal, SIYI ZT6 with damper", GIMBAL_DATASHEET_KG, g("gimbal (thermal + RGB)")[1], "Measured", "electronics",
         "SIYI spec page 197 g"),
        ("nadir thermal camera, Boson 640 50 deg + mount", v1.PAYLOAD["part_kg"] - GIMBAL_DATASHEET_KG,
         g("gimbal (thermal + RGB)")[1], "Calculated", "electronics", "Boson core + lens datasheet + mount estimate"),
        ("twin camera: second Boson core + splayed mount (v1.1)", v1.PAYLOAD["twin_extra_kg"],
         g("gimbal (thermal + RGB)")[1], "Estimated", "electronics", "step2_trades winner"),
        ("aircraft parachute + independent trigger (v1.1 RC1)", *g(PARA_ITEM), "Estimated", "mechanism",
         "vendor class ~0.32 kg for <= 9.3 kg UAV + launcher/trigger"),
        ("avionics: FC, GPS, radio, airspeed, Remote ID", *g("avionics (FC, GPS, radio)"), "Estimated", "electronics",
         "sizing.A.m_avionics budget"),
        ("pod structure, door, skid", *g("pod structure + door + skid"), "Estimated", "structure", "budget"),
        ("motor T-Motor AT3520-550", MOTOR_KG, g("motor + ESC + prop")[1], "Measured", "propulsion",
         "T-Motor spec page 218 g"),
        ("ESC, prop blades, hub, spinner", g("motor + ESC + prop")[0] - MOTOR_KG, g("motor + ESC + prop")[1],
         "Estimated", "propulsion", "v3 propulsion pick total 0.34 kg less the motor"),
        ("misc: wiring, connectors, glue", *g("misc (wiring, glue) 5%"), "Estimated", "harness",
         "fixed 0.257 kg from the 7.364 kg placeholder (3.1% of v1)"),
    ]
    for item, cat in V11_CATEGORY.items():   # remaining v1.1 rows, carried in the CAD table
        rows.append((item, *g(item), "Estimated", cat, cad.loc[item, "source"].split("; ", 1)[-1]))
    df = pd.DataFrame(rows, columns=["item", "basic_kg", "x_m", "maturity", "category", "basis"])
    df["mga_pct"] = [CONTROLLED if not isinstance(c, str) else SCHEDULE[c][MATURITY.index(m)]
                     for m, c in zip(df.maturity, df.category)]
    df["category"] = df.category.fillna("controlled")
    df["mga_kg"] = df.basic_kg * df.mga_pct / 100
    df["predicted_kg"] = df.basic_kg + df.mga_kg
    df["droppable"] = df.item.isin(["rescue kit", "kit parachute"])
    return df


def solve_wing_x(df, mb_loaded):
    """Effective x of the spanwise wing mass, from the loaded mass-balance CG."""
    known = df.x_m.notna()
    m_w = df.loc[~known, "basic_kg"].sum()
    moment = mb_loaded.mass_kg * mb_loaded.cg_m - (df.loc[known, "basic_kg"] * df.loc[known, "x_m"]).sum()
    return moment / m_w


def cg(m, x):
    return float((m * x).sum() / m.sum())


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    say("Lodestar v1 mass growth allowance (G7), 2026-11-15")
    mb = pd.read_csv(ROOT / "out" / "wing_trade" / "aircraft_mass_balance_v1.csv").set_index(["cell", "kit"])
    ms = pd.read_csv(ROOT / "out" / "controls" / "mass_states.csv")
    c = v1.CONTROLS
    mac, limit = v1.GEOMETRY["mac"], v1.MASS["mtow_limit_kg"]
    summ = pd.read_csv(ROOT / "out" / "wing_trade" / "summary_v1.csv").iloc[0]

    say("\nSelf-checks")
    check("wing mass in summary_v1.csv = lodestar_v1.STRUCTURE wing_mass_kg (1.5 g)",
          abs(summ.wing_mass_kg - v1.STRUCTURE["wing_mass_kg"]) <= 0.0015,
          f"{summ.wing_mass_kg:.4f} vs {v1.STRUCTURE['wing_mass_kg']}")
    all_lines, rows = {}, []
    for cell, key in (("SiCore", "sicore"), ("P50B", "p50b")):
        df = lines(cell)
        xw = solve_wing_x(df, mb.loc[(cell, True)])
        df["x_m"] = df.x_m.fillna(xw)
        all_lines[cell] = df
        check(f"{cell} every line classified with a maturity and an MGA",
              df.maturity.isin(MATURITY).all() and df.mga_pct.between(0, 100).all(), f"{len(df)} lines")
        for kit in (True, False):
            d = df if kit else df[~df.droppable]
            frozen = v1.MASS[f"{key}_{'loaded' if kit else 'released'}_kg"]
            check(f"{cell} kit {'on' if kit else 'off'}: sum of basic mass = lodestar_v1.MASS within 1 g",
                  abs(d.basic_kg.sum() - frozen) <= 0.001, f"{d.basic_kg.sum():.4f} vs {frozen}")
            check(f"{cell} kit {'on' if kit else 'off'}: basic CG = mass-balance CG within 0.1 mm",
                  abs(cg(d.basic_kg, d.x_m) - mb.loc[(cell, kit), "cg_m"]) <= 1e-4,
                  f"{cg(d.basic_kg, d.x_m):.5f} vs {mb.loc[(cell, kit), 'cg_m']:.5f} (wing x {xw:.4f} m)")
            st = ms[(ms.cell == cell) & (ms.kit == kit) & (ms.alt == "SL") & (ms.V == 22.0)].iloc[0]
            x_np = float(st.x_np)
            sm0 = 100 * (x_np - cg(d.basic_kg, d.x_m)) / mac
            check(f"{cell} kit {'on' if kit else 'off'}: basic static margin = controls mass_states within 0.02% MAC",
                  abs(sm0 - st.sm_pct) <= 0.02, f"{sm0:.3f} vs {st.sm_pct:.3f}")
            x0 = cg(d.basic_kg, d.x_m)
            aft = d.x_m > x0
            grow_nom = d.predicted_kg
            grow_aft = d.basic_kg + d.mga_kg.where(aft, 0.0)
            grow_fwd = d.basic_kg + d.mga_kg.where(~aft, 0.0)
            x_motor = float(d.loc[d.item.str.startswith("motor"), "x_m"].iloc[0])
            m_lump = d.predicted_kg.sum()
            x_lump = (d.basic_kg.sum() * x0 + d.mga_kg.sum() * x_motor) / m_lump
            cases = {"basic": (d.basic_kg.sum(), x0), "predicted, growth at each line": (grow_nom.sum(), cg(grow_nom, d.x_m)),
                     "predicted, aft lines only grow": (grow_aft.sum(), cg(grow_aft, d.x_m)),
                     "predicted, forward lines only grow": (grow_fwd.sum(), cg(grow_fwd, d.x_m)),
                     "predicted, all MGA at the motor station": (m_lump, x_lump)}
            # The 0.6 kg gimbal line is a budget; the costed gimbal (ZT6) is 0.197 kg. Without the unused part:
            no_alloc = grow_nom.where(~d.item.str.startswith("gimbal budget"), 0.0)
            cases["predicted, gimbal at ZT6 mass (allocation removed)"] = (no_alloc.sum(), cg(no_alloc, d.x_m))
            for case, (m, x) in cases.items():
                ballast = 0.0
                if x + c["cg_build_tol_m"] > c["cg_aft_m"]:
                    # nose ballast at the gimbal station that brings x + tolerance back to the aft limit
                    xg = float(d.loc[d.item.str.startswith("gimbal, SIYI"), "x_m"].iloc[0])
                    target = c["cg_aft_m"] - c["cg_build_tol_m"]
                    ballast = m * (x - target) / (target - xg)
                rows.append(dict(cell=cell, kit=kit, case=case, mass_kg=m, delta_vs_basic_kg=m - d.basic_kg.sum(),
                                 margin_to_10kg=limit - m, margin_pct_of_limit=100 * (limit - m) / limit,
                                 x_cg_m=x, dx_cg_mm=1000 * (x - x0), x_np_m=x_np,
                                 static_margin_pct=100 * (x_np - x) / mac,
                                 aft_limit_margin_mm=1000 * (c["cg_aft_m"] - x),
                                 inside_envelope_with_tol=(c["cg_fwd_m"] <= x - c["cg_build_tol_m"])
                                 and (x + c["cg_build_tol_m"] <= c["cg_aft_m"]),
                                 sm_ge_6=100 * (x_np - x) / mac >= 6.0, nose_ballast_kg=ballast))
    res = pd.DataFrame(rows)
    sic = all_lines["SiCore"]
    check("MGA total is positive and below 20% of basic (sanity)",
          0 < sic.mga_kg.sum() < 0.2 * sic.basic_kg.sum(), f"{sic.mga_kg.sum():.3f} kg")

    # mass that could be added at the motor station before x + tolerance reaches the aft limit (SiCore, loaded)
    d = sic
    x0, m0 = cg(d.predicted_kg, d.x_m), d.predicted_kg.sum()
    xm = float(d.loc[d.item.str.startswith("motor"), "x_m"].iloc[0])
    target = c["cg_aft_m"] - c["cg_build_tol_m"]
    room = m0 * (target - x0) / (xm - target)

    sched = pd.DataFrame([dict(category=k, **dict(zip([f"{m}_pct" for m in MATURITY], v))) for k, v in SCHEDULE.items()])
    sic.round(5).to_csv(OUT / "lines.csv", index=False)
    all_lines["P50B"].round(5).to_csv(OUT / "lines_p50b.csv", index=False)
    res.round(5).to_csv(OUT / "summary.csv", index=False)
    sched.to_csv(OUT / "schedule.csv", index=False)

    with pd.option_context("display.width", 220, "display.max_columns", 20, "display.max_colwidth", 52):
        say("\nLines, SiCore loaded")
        say(sic[["item", "basic_kg", "x_m", "maturity", "category", "mga_pct", "mga_kg", "predicted_kg"]]
            .round(4).to_string(index=False))
        say("\nBy maturity [kg]")
        say(sic.groupby("maturity")[["basic_kg", "mga_kg"]].sum().round(4).to_string())
        say("\nStates")
        say(res[["cell", "kit", "case", "mass_kg", "delta_vs_basic_kg", "margin_to_10kg", "x_cg_m", "dx_cg_mm", "static_margin_pct",
                 "aft_limit_margin_mm", "inside_envelope_with_tol", "nose_ballast_kg"]].round(4).to_string(index=False))
    say(f"\nMass that fits at the motor station (x {xm:.3f} m) before CG + {1000 * c['cg_build_tol_m']:.0f} mm "
        f"reaches the aft limit, SiCore loaded, from predicted: {room:.3f} kg")
    say(f"\n{'ALL CHECKS PASS' if not FAILED else 'FAILED: ' + '; '.join(FAILED)}")
    (OUT / "run_log.txt").write_text("\n".join(LOG) + "\n", encoding="utf-8")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
