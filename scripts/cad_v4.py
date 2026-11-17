"""Lodestar v4 CAD on headless FreeCAD (port of cad_model.py "v3", which needs the blocked OCP kernel).

Same parts and checks as cad_model.py v3 (15 in prop, motor+ESC+prop 0.34 kg, MTOW 7.364 kg placeholder),
built through the generic fcad helper (scripts/fcad). Geometry is computed here in the venv (airfoil
sections from AeroSandbox exactly as cad_model.airfoil_face: twist about the quarter chord, 0.3%-chord
trailing-edge opening) and sent to FreeCAD as a JSON job.

Changes against v3 (constants below):
  - wing packs 25 mm forward and 25 mm inboard (battery_trim.py: X 0.452 -> 0.427, Y 0.355 -> 0.330)
  - compact wing joint instead of the 16 mm x 0.30 m joiners: 20/16 mm carbon tube with 24 mm OD sleeves,
    160 mm engagement per panel each side of the break at y = +/-0.45 m, on the 25%-chord mean-camber axis
    (built exactly as wing_trade.check_packaging)
  - new checks: cell-to-sleeve, cell-to-skin and sleeve-to-skin gaps
  - mass/CG table for both cells (P50B, Amprius SiCore), kit on and off

Mass properties come from a fine mesh, not from OpenCASCADE's exact integration, which is wrong on
these B-spline lofts (see scripts/fcad/README.md); the archived v3 numbers carry that error.

Run:  .venv/Scripts/python.exe scripts/cad_v4.py            (v4 model, ~2-4 min)
      .venv/Scripts/python.exe scripts/cad_v4.py --v3       (v3 constants: reproduction check against out/cad_v3)
Writes: out/cad_v4/ lodestar_v4.step, alt_bay_battery_module.step, stl/, fit_check.csv, joint_gaps.csv,
        mass_cg.csv (P50B, v3 format), mass_cg_sicore.csv, cg_summary.csv, run_log.txt, renders/*.png;
        with --v3: out/cad_v4/v3_repro/ and out/cad_v4/v3_reproduction.csv
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb

sys.path.insert(0, str(Path(__file__).parent))
from avl_run import Geometry  # noqa: E402
from fcad.runner import run  # noqa: E402
from fcad.render import Item, render_views, read_stl  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
V3 = "--v3" in sys.argv

# ---------------------------------------------------------------- design constants
PROP_IN = 15.0                      # propeller diameter, in (v3 pick)
M_PROP = 0.34                       # motor + ESC + prop, kg
MTOW = 7.364                        # kit-on take-off mass with P50B, kg (placeholder from v3_planform.py)
X_BATT_V3, Y_BATT_V3 = 0.452, 0.355
BATT_DX, BATT_DY = -0.005, 0.0      # battery_trim.py convention: negative = forward / inboard.
                                    # v1.0: -0.040 / -0.050 (trim for the elevon balance); v1.1: back aft, the
                                    # nose parachute now supplies that trim (battery_trim.csv: skin gap 1.26 mm)
X_BATT = X_BATT_V3 + (0 if V3 else BATT_DX)     # 0.412 m in v4 (v1 freeze: re-trimmed for the elevon balance)
Y_BATT = Y_BATT_V3 + (0 if V3 else BATT_DY)     # 0.305 m in v4
CELL_R, CELL_L, CELL_DX, CELL_DY = 0.0105, 0.070, 0.0215, 0.071   # 21700, 5 chordwise x 3 spanwise per side
CELL_OPTIONS = {"P50B": 2.4205, "SiCore": None}  # pack masses, kg; SiCore filled from design_v2.pack below
# Wing joint
JOINT = "legacy" if V3 else "compact"
LEGACY_OD, LEGACY_ID, LEGACY_HALF = 0.016, 0.013, 0.30          # v3 joiner: 0.30 m in y each side of the break
TUBE_OD, TUBE_ID, SLEEVE_OD, ENGAGE = 0.020, 0.016, 0.024, 0.160  # compact joint (wing_trade.COMPACT)
X_SPAR = 0.25                       # joint axis on the 25% chord line
AXIS_HALF = 0.30                    # axis direction built from the 25% camber points at y_break +/- 0.30 m
# Layout (unchanged from v3)
G = Geometry(winglet_h=0.25)
M3 = pd.read_csv(ROOT / "out" / "vsp" / "m3_config.csv", index_col=0)["value"]
DP = pd.read_csv(ROOT / "out" / "sizing" / "design_point.csv", index_col=0)["value"]
X_CG = float(M3["x_cg"])
BAY_FWD = 0.006
SKIN = 0.0015
POD_X0, POD_LEN, POD_Z = -0.17, 0.82, -0.035
POD_SECTIONS = [(0.08, 0.09, 0.08, 0.035), (0.25, 0.16, 0.15, 0.04), (0.60, 0.16, 0.15, 0.04),
                (0.85, 0.16, 0.15, 0.04), (0.95, 0.09, 0.08, 0.03)]
AF = asb.Airfoil("mh45")
STL_TOL, STL_ANG = 0.0004, 0.15     # m, rad (as v3)

OUT = ROOT / "out" / "cad_v4" / ("v3_repro" if V3 else "")
import lodestar_v1 as v1  # noqa: E402
# misc line: v1 took MTOW (7.364 kg placeholder) minus the P50B budget = 0.2574 kg. v1.1 holds that figure as an
# explicit allowance (wiring, connectors, fasteners, adhesive) so it no longer depends on the old placeholder.
MISC_KG = 0.2574
PARA_ITEM = "aircraft parachute (RC1, ASTM F3322) + independent trigger"
PARA_SIZE = (0.13, 0.10, 0.042)       # m; 0.55 L canister (vendor packed volume to confirm)
PARA_CENTER = (0.035, 0.0, -0.045)   # under the wing root, above the gimbal ball
OUT.mkdir(parents=True, exist_ok=True)
TAG = "v3 reproduction" if V3 else "v4"


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


# ---------------------------------------------------------------- geometry (venv side)
def airfoil_points(y, chord, x_le, twist_deg, coords=AF.coordinates, te_gap=0.003):
    """Same as cad_model.airfoil_face: section in the plane y = const, twist about the quarter chord."""
    th = math.radians(twist_deg)
    xq = x_le + 0.25 * chord
    pts, n = [], len(coords)
    for i, (u, v) in enumerate(coords):
        v = v + (te_gap / 2) * u * (1 if i < n // 2 else -1)
        x, z = x_le + u * chord, v * chord
        dx = x - xq
        pts.append([xq + dx * math.cos(th) + z * math.sin(th), y, -dx * math.sin(th) + z * math.cos(th)])
    return dict(kind="spline_line", points=pts)


def wing_section(y):
    return airfoil_points(y, G.chord(y), G.x_le(y), G.twist(y))


def camber_z(xc, chord):
    up, lo = AF.upper_coordinates(), AF.lower_coordinates()
    zu = np.interp(xc, up[::-1, 0], up[::-1, 1])
    zl = np.interp(xc, lo[:, 0], lo[:, 1])
    return float((zu + zl) / 2 * chord), float((zu - zl) * chord)


def winglet(side):
    y = side * G.span / 2
    c0, c1 = G.chord(y), G.chord(y) * G.winglet_taper
    x0 = G.x_le(y)
    x1 = x0 + G.winglet_h * math.tan(math.radians(G.winglet_sweep_deg))
    sym = asb.Airfoil("naca0008").coordinates
    secs = []
    for z, c, xl in ((0.0, c0, x0), (G.winglet_h, c1, x1)):
        pts = [[xl + u * c, y + side * v * c, z] for u, v in sym]
        secs.append(dict(kind="spline_line", points=pts[:-1]))   # cad_model: Spline(pts[:-1]) + Line(pts[-2], pts[0])
    return dict(op="loft", ruled=True, sections=secs)


def hinge_plane(side):
    """Point and aft-pointing normal of the vertical plane through the elevon hinge line."""
    y0, y1 = side * G.y_break, side * G.span / 2
    p0 = np.array([G.x_le(y0) + G.hinge_xc * G.chord(y0), y0, 0])
    p1 = np.array([G.x_le(y1) + G.hinge_xc * G.chord(y1), y1, 0])
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    n = -np.cross([0, 0, 1], d)
    n = n if n[0] > 0 else -n
    return p0.tolist(), n.tolist()


def box(size, center, **kw):
    return dict(op="box", size=list(size), center=list(center), **kw)


def cyl(r, h, center, axis=(0, 0, 1)):
    return dict(op="cylinder", r=r, h=h, center=list(center), axis=list(axis))


def tube(od, idia, h, center, axis):
    return dict(op="cut", base=cyl(od / 2, h, center, axis), tools=[cyl(idia / 2, h, center, axis)])


def joint_axis(side):
    """wing_trade.check_packaging: axis through the 25%-chord mean-camber points at y_break -/+ 0.30 m."""
    def pt(y):
        c = G.chord(y)
        return np.array([G.x_le(y) + X_SPAR * c, y, camber_z(X_SPAR, c)[0]])
    a, b = pt(side * (G.y_break - AXIS_HALF)), pt(side * (G.y_break + AXIS_HALF))
    return a, b, (b - a) / np.linalg.norm(b - a), (a + b) / 2


def build_job():
    parts, defs = {}, {}
    yb = G.y_break
    parts["wing_centre"] = dict(op="loft", ruled=True, sections=[wing_section(-yb), wing_section(0.0), wing_section(yb)])
    for side, nm in ((1, "R"), (-1, "L")):
        defs[f"panel_out_{nm}"] = dict(op="loft", ruled=True, sections=[wing_section(side * yb), wing_section(side * G.span / 2)])
        defs[f"panel_in_{nm}"] = dict(op="loft", ruled=True, sections=[wing_section(0.0), wing_section(side * yb)])
        p, n = hinge_plane(side)
        ref = dict(op="ref", name=f"panel_out_{nm}")
        parts[f"wing_outer_{nm}"] = dict(op="split_by_plane", shape=ref, point=p, normal=n, keep="negative")
        parts[f"elevon_{nm}"] = dict(op="split_by_plane", shape=ref, point=p, normal=n, keep="positive")
        parts[f"winglet_{nm}"] = winglet(side)

    # pod (rounded-rectangle loft) and the bay door cut out of its belly
    stations = [(0.0, 0.016, 0.016, 0.007)] + POD_SECTIONS + [(1.0, 0.03, 0.03, 0.012)]
    defs["pod_solid"] = dict(op="rrect_loft", sections=[
        dict(origin=[POD_X0 + xl * POD_LEN, 0, POD_Z], normal=[1, 0, 0], xdir=[0, 1, 0], w=w, h=h, r=min(rc, 0.49 * min(w, h)))
        for xl, w, h, rc in stations])
    door = box((0.205, 0.110, 0.04), (X_CG, 0, POD_Z - 0.07))
    parts["pod"] = dict(op="cut", base=dict(op="ref", name="pod_solid"), tools=[door])
    parts["bay_door"] = dict(op="common", shapes=[dict(op="ref", name="pod_solid"), door])

    # kit bay
    kz = POD_Z - 0.003
    xb = X_CG - BAY_FWD
    parts["kit_container"] = dict(op="cut", base=box((0.22, 0.13, 0.12), (xb, 0, kz), fillet=0.012, fillet_edges_parallel=[1, 0, 0]),
                                  tools=[box((0.212, 0.122, 0.12), (xb, 0, kz - 0.004))])
    parts["rescue_kit"] = box((0.19, 0.11, 0.07), (X_CG, 0, kz - 0.06 + 0.035 + 0.001))
    parts["kit_parachute"] = box((0.19, 0.10, 0.035), (X_CG, 0, kz - 0.06 + 0.07 + 0.0175 + 0.002))
    parts["release_servo"] = box((0.012, 0.023, 0.016), (xb - 0.106 + 0.0065, 0, kz + 0.03))
    defs["alt_bay_battery"] = dict(op="compound", shapes=[
        cyl(CELL_R, CELL_L, (X_CG - 0.0525 + i * 0.021, -0.021 + j * 0.021, kz - 0.017)) for i in range(6) for j in range(3)])

    # wing-root packs: 5 chordwise x 3 spanwise per side, cell axes along the span
    cells = {}
    for side, nm in ((1, "R"), (-1, "L")):
        yc = side * Y_BATT
        zc, _ = camber_z((X_BATT - G.x_le(yc)) / G.chord(yc), G.chord(yc))
        cells[nm] = [cyl(CELL_R, CELL_L, (X_BATT + (i - 2) * CELL_DX, yc + (j - 1) * CELL_DY, zc), (0, 1, 0))
                     for i in range(5) for j in range(3)]
        parts[f"battery_{nm}"] = dict(op="compound", shapes=cells[nm])

    # wing joint
    joint_parts = {}
    for side, nm in ((1, "R"), (-1, "L")):
        if JOINT == "legacy":       # cad_model v3: 16 mm tube between the 25% camber points at y_break -/+ 0.30
            def pt(y):
                c = G.chord(y)
                return np.array([G.x_le(y) + 0.25 * c, y, camber_z(0.25, c)[0]])
            a, b = pt(side * (yb - LEGACY_HALF)), pt(side * (yb + LEGACY_HALF))
            d = b - a
            parts[f"spar_joiner_{nm}"] = tube(LEGACY_OD, LEGACY_ID, float(np.linalg.norm(d)), (a + b) / 2, d)
            joint_parts[nm] = [f"spar_joiner_{nm}"]
        else:                       # compact joint: tube 2 x engagement long, one sleeve per panel
            _, _, ax, mid = joint_axis(side)
            parts[f"joint_tube_{nm}"] = tube(TUBE_OD, TUBE_ID, 2 * ENGAGE, mid, ax)
            for k, sgn in (("in", -1), ("out", 1)):
                c = mid + ax * sgn * ENGAGE / 2
                parts[f"joint_sleeve_{k}_{nm}"] = tube(SLEEVE_OD, TUBE_OD, ENGAGE, c, ax)
                defs[f"sleeve_env_{k}_{nm}"] = cyl(SLEEVE_OD / 2, ENGAGE, c, ax)
            joint_parts[nm] = [f"joint_tube_{nm}", f"joint_sleeve_in_{nm}", f"joint_sleeve_out_{nm}"]

    for side, nm in ((1, "R"), (-1, "L")):
        ys = side * (yb + 0.4 * (G.span / 2 - yb))
        c = G.chord(ys)
        parts[f"elevon_servo_{nm}"] = box((0.030, 0.032, 0.010), (G.x_le(ys) + 0.60 * c, ys, camber_z(0.60, c)[0]))

    parts["flight_controller"] = box((0.10, 0.07, 0.035), (0.25, 0, POD_Z - 0.01))
    parts["esc"] = box((0.06, 0.035, 0.018), (0.565, 0, POD_Z))
    parts["gps_mast"] = cyl(0.004, 0.06, (0.12, 0, POD_Z + 0.075 + 0.03))
    parts["gps"] = cyl(0.03, 0.012, (0.12, 0, POD_Z + 0.075 + 0.065))
    parts["pitot"] = cyl(0.003, 0.14, (POD_X0 + 0.02 - 0.06, 0, POD_Z), (1, 0, 0))
    parts["gimbal_mount"] = cyl(0.03, 0.02, (0.07, 0, POD_Z - 0.065))
    parts["gimbal_ball"] = dict(op="sphere", r=0.045, center=[0.07, 0, -0.115])
    th = math.radians(-60)          # build123d Rot(0, -60, 0): z axis rotated about y
    parts["gimbal_lens"] = cyl(0.018, 0.012, (0.07 - 0.032, 0, -0.115 - 0.026), (math.sin(th), 0, math.cos(th)))

    xm = POD_X0 + POD_LEN
    parts["motor"] = cyl(0.026, 0.04, (xm + 0.02, 0, POD_Z), (1, 0, 0))
    parts["spinner"] = dict(op="cone", r1=0.022, r2=0.003, h=0.05, center=[xm + 0.07, 0, POD_Z], axis=[1, 0, 0])
    r_tip = PROP_IN * 0.0254 / 2
    parts["propeller"] = dict(op="compound", shapes=[box((0.006, r_tip - 0.015, 0.026), (0, s * (0.015 + r_tip) / 2, 0)) for s in (1, -1)],
                              transform=[dict(rotate=dict(axis=[1, 0, 0], deg=25, center=[0, 0, 0])),
                                         dict(translate=[xm + 0.055, 0, POD_Z])])
    if V3:
        parts["belly_skid"] = box((0.30, 0.025, 0.004), (0.27, 0, POD_Z - 0.076))
    else:   # v1.1: 70 mm EPP crush skid (lodestar_v1.STRUCTURE), 60 mm wide
        h_skid = v1.STRUCTURE["belly_skid_crush_stroke_min_mm"] / 1e3
        parts["belly_skid"] = box((0.30, 0.060, h_skid), (0.27, 0, POD_Z - 0.0755 - h_skid / 2))
        # v1.1 RC1: aircraft parachute canister under the wing root, above the gimbal mount
        parts["aircraft_parachute"] = box(PARA_SIZE, PARA_CENTER)
    defs["wing_all"] = dict(op="fuse", shapes=[dict(op="ref", name=n) for n in ("wing_centre", "wing_outer_R", "wing_outer_L")])
    defs["prop_disc"] = cyl(r_tip, 0.03, (xm + 0.055, 0, POD_Z), (1, 0, 0))

    # ---------------- checks
    fit = [("battery_R", "wing_all"), ("battery_L", "wing_all")]
    fit += [(p, "wing_all") for p in joint_parts["R"]]
    fit += [("elevon_servo_R", "wing_all"), ("kit_container", "pod_solid"), ("flight_controller", "pod_solid"),
            ("esc", "pod_solid"), ("alt_bay_battery", "pod_solid")]
    if not V3:
        fit.append(("aircraft_parachute", "pod_solid"))
    checks = []
    for p, par in fit:
        checks.append(dict(id=f"in:{p}", type="inside", part=p, parent=par))
        checks.append(dict(id=f"gap:{p}", type="distance", a=p, b=par, mode="shell"))
    checks.append(dict(id="prop_wing", type="distance", a="prop_disc", b="wing_all"))
    if not V3:   # canister must clear the wing root (the wing passes through the pod top)
        checks.append(dict(id="para_wing", type="distance", a="aircraft_parachute", b="wing_centre"))
    checks.append(dict(id="prop_elevon_R", type="distance", a="prop_disc", b="elevon_R"))
    checks.append(dict(id="prop_elevon_L", type="distance", a="prop_disc", b="elevon_L"))
    skins = {"wing_centre", "wing_outer_R", "wing_outer_L", "elevon_R", "elevon_L", "winglet_R", "winglet_L", "pod", "bay_door"}
    inner = [n for n in parts if n not in skins]
    attached = [["gimbal_mount", "gimbal_ball"], ["gimbal_ball", "gimbal_lens"], ["gps_mast", "gps"], ["spinner", "propeller"]]
    for nm in "RL":
        if JOINT == "compact":     # tube slides in the sleeves (zero-clearance contact by design)
            attached += [[f"joint_tube_{nm}", f"joint_sleeve_in_{nm}"], [f"joint_tube_{nm}", f"joint_sleeve_out_{nm}"],
                         [f"joint_sleeve_in_{nm}", f"joint_sleeve_out_{nm}"]]
    checks.append(dict(id="interference", type="interference", parts=inner, allow=attached))
    if JOINT == "compact":
        for nm in "RL":
            for k, panel in (("in", f"panel_in_{nm}"), ("out", f"panel_out_{nm}")):
                # wing_trade.check_packaging: sleeve envelope to its panel skin, panel end faces excluded
                checks.append(dict(id=f"sleeve_skin:{k}_{nm}", type="distance", a=f"sleeve_env_{k}_{nm}", b=panel,
                                   mode="shell", exclude_flat_normal=[0, 1, 0]))
                checks.append(dict(id=f"cell_sleeve:{k}_{nm}", type="distance", a=f"battery_{nm}", b=f"sleeve_env_{k}_{nm}"))
                # same at the v3 pack position (reproduces out/wing_trade/packaging.csv cell_gap_mm)
                side = 1 if nm == "R" else -1
                yc = side * Y_BATT_V3
                zc, _ = camber_z((X_BATT_V3 - G.x_le(yc)) / G.chord(yc), G.chord(yc))
                old = dict(op="compound", shapes=[cyl(CELL_R, CELL_L, (X_BATT_V3 + (i - 2) * CELL_DX, yc + (j - 1) * CELL_DY, zc), (0, 1, 0))
                                                  for i in range(5) for j in range(3)])
                checks.append(dict(id=f"cell_sleeve_v3pos:{k}_{nm}", type="distance", a=old, b=f"sleeve_env_{k}_{nm}"))
    stl_parts = list(parts)
    return dict(
        scale_to_mm=1000, parts=parts, defs=defs, checks=checks,
        outputs=dict(stl_dir=str(OUT / "stl"), stl_parts=stl_parts, stl_extra=["alt_bay_battery"],
                     stl_tolerance=STL_TOL, stl_angular=STL_ANG, props_extra=["alt_bay_battery", "wing_all", "pod_solid"],
                     step=[dict(path=str(OUT / f"lodestar_{'v3' if V3 else 'v4'}.step"), parts=list(parts)),
                           dict(path=str(OUT / "alt_bay_battery_module.step"), parts=["alt_bay_battery"])])), joint_parts


# ---------------------------------------------------------------- tables
def budget(joint_parts):
    return {
        "wing structure (centre, outer panels, elevons)": (float(DP["m_wing"]),
            ["wing_centre", "wing_outer_R", "wing_outer_L", "elevon_R", "elevon_L"]),
        "winglets": (2 * 0.25 * 0.266 * 0.8 * 1.0, ["winglet_R", "winglet_L"]),
        "batteries (P50B 30 cells)": (CELL_OPTIONS["P50B"], ["battery_R", "battery_L"]),
        "rescue kit": (1.50, ["rescue_kit"]),
        "kit parachute": (0.124, ["kit_parachute"]),
        "kit container + release": (0.08, ["kit_container", "release_servo"]),
        "gimbal (thermal + RGB)": (0.60 if V3 else v1.PAYLOAD["part_kg"] + v1.PAYLOAD["twin_extra_kg"],
                                   ["gimbal_ball", "gimbal_mount"]),
        **({} if V3 else {PARA_ITEM: (v1.V11_MASS[PARA_ITEM][0], ["aircraft_parachute"])}),
        "avionics (FC, GPS, radio)": (0.35, ["flight_controller", "gps"]),
        "pod structure + door + skid": (0.30, ["pod", "bay_door", "belly_skid"]),
        "motor + ESC + prop": (M_PROP, ["motor", "esc", "propeller", "spinner"]),
    }


def mass_table(P, joint_parts, key="com", battery_mass=None, battery_label=None):
    rows = []
    for item, (m, names) in budget(joint_parts).items():
        vk = "volume" if key == "com" else "brep_volume"
        vt = sum(P[n][vk] for n in names)
        x = sum(P[n][vk] * P[n][key][0] for n in names) / vt
        z = sum(P[n][vk] * P[n][key][2] for n in names) / vt
        if item.startswith("batteries") and battery_mass is not None:
            item, m = battery_label, battery_mass
        rows.append(dict(item=item, mass_kg=m, x=x, z=z, source="CAD centroid (volume-weighted)"))
    rows.append(dict(item="misc (wiring, glue) 5%", mass_kg=(MTOW - sum(m for m, _ in budget(joint_parts).values())) if V3 else MISC_KG, x=0.35, z=-0.02,
                     source="not modelled; same assumption as milestone 2"))
    if not V3:
        import lodestar_v1 as v1
        a = v1.ADDED_MASS
        rows.append(dict(item="pack thermal kit (foam + heater)", mass_kg=a["pack_thermal_kit_kg"], x=X_BATT,
                         z=rows[[r["item"].startswith("batteries") for r in rows].index(True)]["z"],
                         source="lodestar_v1.ADDED_MASS; carried at the pack centroid"))
        yb = a["elevon_balance_y"]
        xb = G.x_le(yb) + G.hinge_xc * G.chord(yb) - a["elevon_balance_arm"]
        rows.append(dict(item="elevon mass balance (2 per aircraft)", mass_kg=2 * a["elevon_balance_kg_each"], x=xb,
                         z=0.0, source="lodestar_v1.ADDED_MASS; flutter_v2 static balance at the servo station"))
        for item, (m, x, z, basis) in v1.V11_MASS.items():
            if item.startswith(("payload", "twin camera")) or item == PARA_ITEM:
                continue        # in the gimbal and parachute lines above, at CAD centroids
            rows.append(dict(item=item, mass_kg=m, x=x, z=z, source=f"lodestar_v1.V11_MASS; {basis}"))
    return pd.DataFrame(rows)


def cg(mc, kit=True):
    d = mc if kit else mc[~mc.item.isin(["rescue kit", "kit parachute"])]
    return d.mass_kg.sum(), (d.mass_kg * d.x).sum() / d.mass_kg.sum(), (d.mass_kg * d.z).sum() / d.mass_kg.sum()


def stl_props(path):
    t = read_stl(path)
    v = np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])) / 6
    return v.sum(), (v[:, None] * t.sum(1) / 4).sum(0) / v.sum()


# ---------------------------------------------------------------- main
def main():
    sys.stdout = Tee(OUT / "run_log.txt")
    print(f"Lodestar {TAG} CAD (FreeCAD headless via fcad), 2026-11-14; prop {PROP_IN:.0f} in")
    print(f"Packs at x = {X_BATT:.3f} m, y = +/-{Y_BATT:.3f} m; joint: {JOINT}\n")
    job, joint_parts = build_job()
    res = run(job, OUT / "fcad_work")
    print(f"FreeCAD {res['freecad_version']}: {len(res['parts'])} shapes in {res['seconds']:.0f} s\n")
    P, C = res["parts"], res["checks"]
    pd.DataFrame([dict(part=n, volume_cm3=p["volume"] * 1e6, x=p["com"][0], y=p["com"][1], z=p["com"][2],
                       brep_volume_cm3=p["brep_volume"] * 1e6, brep_x=p["brep_com"][0], brep_z=p["brep_com"][2])
                  for n, p in P.items()]).round(6).to_csv(OUT / "part_props.csv", index=False)

    # fit check (v3 format)
    rows = []
    fit_names = ["battery_R", "battery_L"] + joint_parts["R"] + ["elevon_servo_R", "kit_container", "flight_controller", "esc",
                                                                 "alt_bay_battery"]
    for n in fit_names:
        share = C[f"in:{n}"]["share_inside_min_solid"]
        d = C[f"gap:{n}"]["distance"]
        parent = "pod" if n in ("kit_container", "flight_controller", "esc", "alt_bay_battery", "aircraft_parachute") else "wing"
        label = "bay_battery_module (alt)" if n == "alt_bay_battery" else n
        rows.append(dict(part=label, parent=parent, share_inside=share, min_clearance_mm=d * 1000,
                         ok=share > 0.999 and d >= SKIN))
    if not V3:
        n = "aircraft_parachute"
        share, d = C[f"in:{n}"]["share_inside_min_solid"], C[f"gap:{n}"]["distance"]
        dw = C["para_wing"]["distance"]
        rows.append(dict(part=n, parent="pod", share_inside=share, min_clearance_mm=min(d, dw) * 1000,
                         ok=share > 0.999 and d >= SKIN and dw >= SKIN))
        print(f"Parachute canister: {share:.4f} inside the pod, {d * 1000:.1f} mm to the pod skin, "
              f"{dw * 1000:.1f} mm to the wing root")
    d_wing = C["prop_wing"]["distance"]
    d_elev = min(C["prop_elevon_R"]["distance"], C["prop_elevon_L"]["distance"])
    rows.append(dict(part=f"propeller disc {PROP_IN:.0f} in", parent="wing (gap)", share_inside=0.0,
                     min_clearance_mm=d_wing * 1000, ok=d_wing >= 0.02))
    fit = pd.DataFrame(rows)
    fit.round(4).to_csv(OUT / "fit_check.csv", index=False)
    clashes = C["interference"]["clashes"]
    n_inner = next(c for c in job["checks"] if c["type"] == "interference")["parts"]
    print(f"Interference between internal parts ({len(n_inner)} parts; "
          f"{C['interference']['pairs_tested']} pairs close enough to test): "
          + ("none" if not clashes else "; ".join(f"{c['a']} x {c['b']}: {c['volume'] * 1e6:.2f} cm3" for c in clashes)))
    print(f"Propeller disc ({PROP_IN:.0f} in, 30 mm thick) to the wing: {d_wing * 1000:.1f} mm; to the elevons: {d_elev * 1000:.1f} mm")
    print(f"Fit check (part fully inside its parent's outer surface, with >= {SKIN * 1000:.1f} mm for the skin):")
    print(fit.round(3).to_string(index=False))

    # joint and pack gaps
    if JOINT == "compact":
        trim = pd.read_csv(ROOT / "out" / "wing_trade" / "battery_trim.csv")
        tr = trim[np.isclose(trim.dx_mm, BATT_DX * 1000) & np.isclose(trim.dy_mm, BATT_DY * 1000)].iloc[0]
        pk = pd.read_csv(ROOT / "out" / "wing_trade" / "packaging.csv").query("tube_od_mm == 20")
        pk = pk.set_index("panel")
        g = []
        cell_sleeve = min(C[f"cell_sleeve:{k}_{nm}"]["distance"] for k in ("in", "out") for nm in "RL")
        cell_skin = min(C["gap:battery_R"]["distance"], C["gap:battery_L"]["distance"])
        g.append(dict(check="cell to sleeve (new pack position)", cad_mm=cell_sleeve * 1000, reference_mm=tr.sleeve_gap_mm,
                      reference=f"battery_trim.csv dx {BATT_DX*1000:.0f} dy {BATT_DY*1000:.0f} (capsule model, conservative)"))
        g.append(dict(check="cell to skin after 1.5 mm allowance (new position)", cad_mm=(cell_skin - SKIN) * 1000,
                      reference_mm=tr.skin_gap_after_allowance_mm, reference=f"battery_trim.csv dx {BATT_DX*1000:.0f} dy {BATT_DY*1000:.0f}"))
        for k, panel in (("in", "inboard"), ("out", "outboard")):
            sk = min(C[f"sleeve_skin:{k}_{nm}"]["distance"] for nm in "RL")
            g.append(dict(check=f"sleeve to skin, {panel} panel (raw)", cad_mm=sk * 1000, reference_mm=pk.loc[panel, "skin_gap_mm"],
                          reference="packaging.csv 20 mm joint"))
            g.append(dict(check=f"sleeve to skin, {panel} panel, after 1.5 mm allowance", cad_mm=(sk - SKIN) * 1000,
                          reference_mm=pk.loc[panel, "gap_after_skin_allowance_mm"], reference="packaging.csv 20 mm joint"))
            cv = min(C[f"cell_sleeve_v3pos:{k}_{nm}"]["distance"] for nm in "RL")
            g.append(dict(check=f"cell to {panel} sleeve at the OLD v3 pack position", cad_mm=cv * 1000,
                          reference_mm=pk.loc[panel, "cell_gap_mm"], reference="packaging.csv 20 mm joint (build123d)"))
            cn = min(C[f"cell_sleeve:{k}_{nm}"]["distance"] for nm in "RL")
            g.append(dict(check=f"cell to {panel} sleeve (new position)", cad_mm=cn * 1000, reference_mm=np.nan, reference=""))
        gaps = pd.DataFrame(g)
        gaps["diff_mm"] = gaps.cad_mm - gaps.reference_mm
        gaps.round(3).to_csv(OUT / "joint_gaps.csv", index=False)
        print("\nJoint and pack gaps (CAD vs earlier screening):")
        print(gaps.round(2).to_string(index=False))

    # mass and balance
    mc = mass_table(P, joint_parts)
    mc.round(4).to_csv(OUT / "mass_cg.csv", index=False)
    sic = CELL_OPTIONS["SiCore"]
    if sic is None:
        import design_v2
        sic = design_v2.pack("Amprius SiCore 6.3 Ah", design_v2.N_WING)[1]
    mcs = mass_table(P, joint_parts, battery_mass=sic, battery_label="batteries (SiCore 30 cells)")
    mcs.round(4).to_csv(OUT / "mass_cg_sicore.csv", index=False)
    print("\nBalance from the CAD layout (P50B):")
    print(mc.round(4).to_string(index=False))
    summ = []
    for cell, t in (("P50B", mc), ("SiCore", mcs)):
        for kit in (True, False):
            m, x, z = cg(t, kit)
            summ.append(dict(cell=cell, kit=kit, mass_kg=m, x_cg=x, z_cg=z, x_cg_minus_design_mm=(x - X_CG) * 1000,
                             x_cg_minus_design_pct_mac=(x - X_CG) / G.mac * 100))
    summ = pd.DataFrame(summ)
    summ.round(5).to_csv(OUT / "cg_summary.csv", index=False)
    print(f"\nCG (design balance point x = {X_CG:.4f} m; MAC {G.mac:.4f} m):")
    print(summ.round(4).to_string(index=False))
    print("Note: 'wing structure' is still the 1.29 kg budget at the CAD wing centroid; the joint has no separate line.")
    if V3:
        reproduce(P, C, fit, mc, joint_parts)
    else:
        renders()
    print(f"\nWrote {OUT.relative_to(ROOT)}")


# ---------------------------------------------------------------- v3 reproduction
def reproduce(P, C, fit, mc, joint_parts):
    arch = ROOT / "out" / "cad_v3"
    rows = []
    for n in ("wing_centre", "wing_outer_R", "elevon_R", "winglet_R", "pod", "bay_door", "battery_R", "spar_joiner_R",
              "kit_container"):
        v, c = stl_props(arch / "stl" / f"{n}.stl")
        rows.append(dict(quantity=f"{n} volume (cm3)", archived=v * 1e6, port=P[n]["volume"] * 1e6,
                         port_brep=P[n]["brep_volume"] * 1e6, archived_source="v3 STL mesh (0.4 mm)"))
        for i, a in enumerate("xyz"):
            rows.append(dict(quantity=f"{n} centroid {a} (mm)", archived=c[i] * 1000, port=P[n]["com"][i] * 1000,
                             port_brep=P[n]["brep_com"][i] * 1000, archived_source="v3 STL mesh (0.4 mm)"))
    af = pd.read_csv(arch / "fit_check.csv")
    for r in af.itertuples():
        name = r.part
        mine = fit[fit.part == name]
        if not len(mine):
            continue
        rows.append(dict(quantity=f"fit: {name} min clearance (mm)", archived=r.min_clearance_mm,
                         port=float(mine.min_clearance_mm.iloc[0]), port_brep=np.nan, archived_source="cad_v3/fit_check.csv"))
        rows.append(dict(quantity=f"fit: {name} share inside", archived=r.share_inside,
                         port=float(mine.share_inside.iloc[0]), port_brep=np.nan, archived_source="cad_v3/fit_check.csv"))
    am = pd.read_csv(arch / "mass_cg.csv")
    mb = mass_table(P, joint_parts, key="brep_com")
    for r in am.itertuples():
        for a in ("x", "z"):
            rows.append(dict(quantity=f"mass_cg: {r.item} {a} (mm)", archived=getattr(r, a) * 1000,
                             port=float(mc.loc[mc.item == r.item, a].iloc[0]) * 1000,
                             port_brep=float(mb.loc[mb.item == r.item, a].iloc[0]) * 1000, archived_source="cad_v3/mass_cg.csv"))
    acg = pd.read_csv(arch / "cg_summary.csv", index_col=0)["value"]
    for kit, key in ((True, "x_cg_cad"), (False, "x_cg_cad_kit_dropped")):
        rows.append(dict(quantity=f"aircraft x_cg {'kit on' if kit else 'kit off'} (mm)", archived=acg[key] * 1000,
                         port=cg(mc, kit)[1] * 1000, port_brep=cg(mb, kit)[1] * 1000, archived_source="cad_v3/cg_summary.csv"))
    rows.append(dict(quantity="aircraft z_cg kit on (mm)", archived=acg["z_cg_cad"] * 1000, port=cg(mc)[2] * 1000,
                     port_brep=cg(mb)[2] * 1000, archived_source="cad_v3/cg_summary.csv"))
    df = pd.DataFrame(rows)
    df["port_minus_archived"] = df.port - df.archived
    df["brep_minus_archived"] = df.port_brep - df.archived
    df.round(4).to_csv(ROOT / "out" / "cad_v4" / "v3_reproduction.csv", index=False)
    print("\nv3 reproduction (port = mesh-integrated, port_brep = OpenCASCADE exact integration as build123d used):")
    with pd.option_context("display.width", 200, "display.max_rows", 200):
        print(df.round(3).to_string(index=False))


# ---------------------------------------------------------------- renders
COLORS = {"wing": "#c9d1d9", "elevon": "#8fa8c8", "winglet": "#c9d1d9", "pod": "#d8d2c4", "bay_door": "#b8b0a0",
          "battery": "#d9534f", "joint_tube": "#222222", "joint_sleeve": "#e69f00", "spar_joiner": "#222222",
          "kit": "#5cb85c", "rescue": "#5cb85c", "release": "#337ab7", "elevon_servo": "#337ab7",
          "flight": "#337ab7", "esc": "#337ab7", "gps": "#6f42c1", "pitot": "#555555", "gimbal": "#444444",
          "motor": "#555555", "spinner": "#888888", "propeller": "#333333", "belly": "#7a5c3a"}
SKIN_PARTS = ("wing_", "elevon_R", "elevon_L", "winglet", "pod", "bay_door")


def color(n):
    for k, c in COLORS.items():
        if n.startswith(k):
            return c
    return "#999999"


def renders():
    stl = OUT / "stl"
    names = [p.stem for p in sorted(stl.glob("*.stl")) if p.stem != "alt_bay_battery"]
    rd = OUT / "renders"
    opaque = [Item(str(stl / f"{n}.stl"), color(n)) for n in names]
    render_views(opaque, rd / "lodestar_v4_views.png", title="Lodestar v4 (FreeCAD): front, side, top, isometric")
    see = [Item(str(stl / f"{n}.stl"), color(n), 0.18 if n.startswith(SKIN_PARTS) else 1.0) for n in names]
    see.append(Item(str(stl / "alt_bay_battery.stl"), "#f0ad4e", 0.0))
    render_views(see, rd / "lodestar_v4_internals.png", views=("top", "side", "front", "iso"),
                 title="Lodestar v4 internals (skins translucent)")
    # close-up of the right wing root: packs and joint (top: screen x = y, screen y = -x; front: screen x = -y)
    crops = {"top": ((0.15, -0.72), (0.80, -0.22)), "front": ((-0.80, -0.08), (-0.15, 0.07)),
             "side": ((0.20, -0.08), (0.75, 0.07))}
    near = [Item(str(stl / f"{n}.stl"), color(n), 0.15 if n.startswith(SKIN_PARTS) else 1.0) for n in names
            if n.startswith(("wing_", "battery_R", "joint_", "elevon_R", "elevon_servo_R"))]
    render_views(near, rd / "lodestar_v4_wing_root_closeup.png", views=("top", "front", "side"), crops=crops, ncols=3,
                 title="Right wing root: packs (red), joint tube (black) and sleeves (orange)",
                 labels={"top": "top (nose up)", "front": "front (looking aft; right wing on the left)", "side": "side"})
    print(f"Renders: {rd.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
