"""Check that every script and output agrees with the frozen v1 parameters in lodestar_v1.py.

Run:  ../.venv/Scripts/python.exe -X utf8 freeze_check.py   (from scripts/)
Exit code 1 if anything disagrees. Each line names the source being checked.
"""
import math
import sys
from pathlib import Path

import pandas as pd

import lodestar_v1 as v1

ROOT = Path(__file__).resolve().parents[1]
FAILED = []


def same(name, actual, expected, tol=1e-6, rel=False):
    if isinstance(expected, str):
        ok = actual == expected
    else:
        err = abs(actual - expected) / (abs(expected) if rel and expected else 1)
        ok = err <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {actual} vs frozen {expected}")
    if not ok:
        FAILED.append(name)


def geometry():
    import avl_run
    import structures as s
    g, f = avl_run.Geometry(), v1.GEOMETRY
    for k in ("span", "c_root", "c_tip", "sweep_qc_deg", "washout_deg", "dihedral_deg", "y_break", "hinge_xc",
              "winglet_taper", "winglet_sweep_deg"):
        same(f"avl_run.Geometry.{k}", getattr(g, k), f[k])
    same("structures.GEO.winglet_h", s.GEO.winglet_h, f["winglet_h"])
    same("structures.GEO.washout_deg", s.GEO.washout_deg, f["washout_deg"])
    same("structures.S", s.S, f["S_ref"], 1e-4)
    same("structures.GEO.mac", s.GEO.mac, f["mac"], 1e-4)
    same("structures.Y_BREAK", s.Y_BREAK, f["y_break"])
    same("structures.SWEEP (deg)", math.degrees(s.SWEEP), f["sweep_qc_deg"])


def structure():
    import structures as s
    import wing_trade as wt
    f = v1.STRUCTURE
    same("structures.FOS", s.FOS, f["fos"])
    same("structures.N_MAN_POS", s.N_MAN_POS, f["n_pos"])
    same("structures.N_MAN_NEG", s.N_MAN_NEG, f["n_neg"])
    same("structures.VC", s.VC, f["vc_eas"])
    same("structures.VD", s.VD, f["vd_eas"])
    same("structures.WRINKLE_K", s.WRINKLE_K, f["wrinkle_k"])
    joint, core_at, lay = wt.DESIGNS[f["design"]]
    same("wing_trade joint tube OD", joint.outer, f["joint_tube_od"])
    same("wing_trade joint tube ID", joint.inner, f["joint_tube_id"])
    same("wing_trade joint sleeve OD", joint.sleeve, f["joint_sleeve_od"])
    same("wing_trade joint engagement", joint.engagement, f["joint_engagement"])
    same("wing_trade joint allowance", joint.allowance, f["joint_allowance_kg"])
    summ = pd.read_csv(ROOT / "out" / "wing_trade" / "summary_v1.csv").set_index("design").loc[f["design"]]
    same("out/wing_trade/summary_v1.csv wing mass", round(summ.wing_mass_kg, 3), f["wing_mass_kg"], 0.0015)


def mass():
    f = v1.MASS
    mb = pd.read_csv(ROOT / "out" / "wing_trade" / "aircraft_mass_balance_v1.csv").set_index(["cell", "kit"])
    for cell, key in (("SiCore", "sicore"), ("P50B", "p50b")):
        same(f"mass balance {cell} loaded", mb.loc[(cell, True), "mass_kg"], f[f"{key}_loaded_kg"], 0.0015)
        same(f"mass balance {cell} released", mb.loc[(cell, False), "mass_kg"], f[f"{key}_released_kg"], 0.0015)
        same(f"mass balance {cell} loaded static margin >= 6% MAC",
             float(mb.loc[(cell, True), "level_static_margin_pct_mac"] >= 6.0), 1.0)
        same(f"mass balance {cell} released static margin >= 6% MAC",
             float(mb.loc[(cell, False), "level_static_margin_pct_mac"] >= 6.0), 1.0)
        drop = mb.loc[(cell, True), "mass_kg"] - mb.loc[(cell, False), "mass_kg"]
        same(f"mass balance {cell} dropped mass = kit + chute", drop, f["rescue_kit_kg"] + f["kit_chute_kg"], 1e-3)
    same("MTOW under limit (loaded SiCore <= 10 kg)", float(f["sicore_loaded_kg"] <= f["mtow_limit_kg"]), 1.0)
    import design_v2
    # The performance model drops kit + container + the older 0.204 kg chute: conservative by 0.08 kg.
    same("design_v2.M_KIT_TOTAL is conservative (>= kit + chute)",
         float(design_v2.M_KIT_TOTAL >= f["rescue_kit_kg"] + f["kit_chute_kg"]), 1.0)


def operations():
    import sizing
    f = v1.OPERATIONS
    same("sizing.A.cl_max_3d", sizing.A.cl_max_3d, f["cl_max_3d"])
    same("sizing.A.stall_margin", sizing.A.stall_margin, f["stall_margin"])
    same("sizing.DESIGN_ALT", sizing.DESIGN_ALT, f["design_alt_m"])


def layout():
    f = v1.LAYOUT
    path = ROOT / "out" / "cad_v4" / "mass_cg.csv"
    if not path.exists():
        same("out/cad_v4/mass_cg.csv exists", "missing", "present")
        return
    cad = pd.read_csv(path)
    bat = cad[cad.item.str.startswith("batteries")].iloc[0]
    same("CAD v4 pack centroid x", round(bat.x, 4), f["x_batt"], 0.002)
    fit = pd.read_csv(ROOT / "out" / "cad_v4" / "fit_check.csv")
    same("CAD v4 fit check: every part inside its parent with clearance", float(fit.ok.all()), 1.0)
    mb = pd.read_csv(ROOT / "out" / "wing_trade" / "aircraft_mass_balance_v1.csv").set_index(["cell", "kit"])
    same("v1 SiCore loaded CG", round(mb.loc[("SiCore", True), "cg_m"], 4), v1.MASS["x_cg_sicore_loaded"], 2e-4)


def controls_and_propulsion():
    c, m = v1.CONTROLS, v1.MASS
    cs = pd.read_csv(ROOT / "out" / "controls" / "summary.csv", index_col=0)["value"]
    same("controls CG forward limit", round(float(cs["x_fwd"]), 3), c["cg_fwd_m"], 0.0006)
    same("controls CG aft limit", round(float(cs["x_aft"]), 3), c["cg_aft_m"], 0.0006)
    same("controls hinge stiffness at 2:1", round(float(cs["k_hinge_at_ratio2_Nm_rad"])), c["hinge_stiffness_nm_rad"], 0.6)
    same("servo torque spec >= 2:1 stall demand / 2", float(c["servo_min_torque_nm"] >= float(cs["servo_stall_Nm"]) / 2), 1.0)
    cad = pd.read_csv(ROOT / "out" / "cad_v4" / "mass_cg.csv").set_index("item")
    a = v1.ADDED_MASS
    same("CAD mass table carries the pack thermal kit", cad.loc["pack thermal kit (foam + heater)", "mass_kg"],
         a["pack_thermal_kit_kg"], 1e-4)
    same("CAD mass table carries the elevon balance", cad.loc["elevon mass balance (2 per aircraft)", "mass_kg"],
         2 * a["elevon_balance_kg_each"], 1e-4)
    for state in ("x_cg_sicore_loaded", "x_cg_sicore_released"):
        lo, hi = m[state] - c["cg_build_tol_m"], m[state] + c["cg_build_tol_m"]
        same(f"{state} +/- build tolerance inside CG envelope", float(c["cg_fwd_m"] <= lo and hi <= c["cg_aft_m"]), 1.0)
    p = v1.PROPULSION
    mg = pd.read_csv(ROOT / "out" / "propulsion_margins" / "margins.csv")
    need = mg[mg.requirement.str.startswith("Motor current needed")].value.max()
    same("ESC current limit covers worst requirement", float(need <= p["esc_current_limit_a"]), 1.0)
    fx = pd.read_csv(ROOT / "out" / "propulsion_margins" / "fixes_sicore_climb.csv").set_index("case")
    both = fx.loc[fx.index.str.startswith("both")].iloc[0]
    same("SiCore climb with heater + high-DA charge rule >= 3 m/s", float(both.roc_search >= 3.0), 1.0)
    same("SiCore banked gradient with heater + high-DA charge rule >= 15%", float(both.grad_bank30 >= 0.15), 1.0)


def flutter():
    f = v1.STRUCTURE
    screen = 1.15 * f["vd_eas"]
    t = pd.read_csv(ROOT / "out" / "flutter_v2" / "all_cases.csv")
    cal = t[t.group == "calibrated"]
    for alt in (0, 4400):
        c = cal[cal.altitude_m == alt]
        ff = c[c.name == "free-free sym, AVL-matched pitch, kit on"].flutter_eas_mps.min()
        same(f"flutter, free-free with AVL pitch, {alt} m >= 1.15 VD EAS", float(ff >= screen), 1.0)
        el = c[(c.name == "AVL-matched pitch, elevon") & (c.k_hinge == f["elevon_hinge_k_nm_rad"])].flutter_eas_mps.min()
        same(f"flutter, elevon on servo stiffness, {alt} m >= 1.15 VD EAS", float(el >= screen), 1.0)
        jk = c[(c.name == "AVL-matched pitch, joint bending sweep") & (c.joint_kb == f["joint_min_bending_k_nm_rad"])]
        same(f"flutter at the minimum joint stiffness, {alt} m >= 1.15 VD EAS", float(jk.flutter_eas_mps.min() >= screen), 1.0)


# Rows in out/fea/margins.csv that show why a construction detail is required. Each one has a passing
# design row (and an entry in lodestar_v1.STRUCTURE["construction_details"]), so it is not a design margin.
NAIVE_ROWS = ("no hard point", "bonded in foam only", "if core removed, no lid", "foam butt joint only",
              "pads on EPS", "SENSITIVITY", "beam-model criterion")


def structure_fe():
    m = pd.read_csv(ROOT / "out" / "fea" / "margins.csv")
    text = m.element + " " + m.failure_mode
    design = m[~text.apply(lambda t: any(k in t for k in NAIVE_ROWS))]
    worst = design.loc[design.MS.idxmin()]
    same(f"FEA: every design margin >= 0 (worst {worst.element}, {worst.failure_mode}, MS {worst.MS:+.3f})",
         float(design.MS.min() >= 0), 1.0)
    for k in NAIVE_ROWS[:5]:
        same(f"FEA: naive-detail row '{k}' present (documents a required detail)", float(text.str.contains(k).any()), 1.0)


# Step 2 headline and held-out mission runs (fleet_flyable.py --design v1.1). Each must have been flown with the
# frozen v1.1 inputs: power-model wing mass, twin camera, 40 deg search bank, v1.1 body drag, and its calibration.
MISSION_RUNS = {"flyable_fleet_pursuit_1_v1.1_seed_20261114": (0.735, 0.20),
                "flyable_fleet_pursuit_1_v1.1_seed_20261115": (0.735, 0.20),
                "flyable_fleet_pursuit_1_v1.1_seed_20261115_porosity_0.35_odds_1.5": (1.5, 0.35),
                "flyable_fleet_pursuit_1_v1.1_seed_20261115_porosity_0.5_odds_3": (3.0, 0.50),
                "flyable_fleet_pursuit_1_v1.1_endurance_0.8_seed_20261115": (0.735, 0.20)}


def missions():
    import json
    from fleet_flyable import lodestar_structure_kg
    import design_v2 as d2
    import v4_upgrade as v4u
    v4u.apply_v11()
    kg, c = lodestar_structure_kg("v1.1"), v1.CAMERA_V11
    for run, (odds, por) in MISSION_RUNS.items():
        f = ROOT / "out" / "wing_trade" / run / "meta.json"
        if not f.exists():
            same(f"{run}: meta.json present", 0.0, 1.0)
            continue
        m = json.loads(f.read_text(encoding="utf-8"))
        same(f"{run}: structure mass", m["structure_kg"], kg, 5e-4)
        same(f"{run}: camera (hfov, spacing)", float(m["lodestar_camera"] == [c["hfov_deg"], c["track_spacing_m"]]), 1.0)
        same(f"{run}: search bank", m["bank_deg"], c["search_bank_deg"], 1e-6)
        same(f"{run}: body drag", m["cda_body_m2"], d2.CDA_BODY, 1e-9)
        same(f"{run}: detection odds scale", m["camera_odds_scale"], odds, 2e-3)
        same(f"{run}: crown porosity", m["crown_porosity"], por, 1e-9)


if __name__ == "__main__":
    for section in (geometry, structure, mass, operations, layout, controls_and_propulsion, flutter, structure_fe, missions):
        print(f"--- {section.__name__}")
        section()
    print(f"\n{len(FAILED)} failed" + (": " + ", ".join(FAILED) if FAILED else ""))
    sys.exit(1 if FAILED else 0)
