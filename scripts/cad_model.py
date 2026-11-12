"""Milestone 5a: detailed CAD of Rescue Wing v2 with its internal layout (build123d).

What it builds (all in metres, axes as in the aero models: x aft, y right, z up, wing-root LE at x = 0):
  - Wing in three car-portable pieces: a 0.9 m centre section and two outer panels joined at the
    elevon break (y = +/-0.45 m), MH45 airfoil, 25 deg quarter-chord sweep, 3 deg linear washout.
  - Elevons split off each outer panel at 75% chord, 0.25 m winglets (NACA 0008).
  - Centre pod (milestone 3 shape), with the kit-bay door cut in its belly.
  - Inside: 2 x 15 Molicel P50B 21700 cells in the wing roots (6S5P), kit container with its
    parachute and release servo, alternative 18-cell bay battery module, flight controller, GPS,
    ESC, elevon servos, carbon spar joiners, thermal/RGB gimbal, pitot, pusher motor and 13 in prop,
    belly skid.
Then it verifies the layout from the CAD itself:
  - fit: does every internal part sit inside its parent skin, and with how much clearance?
  - balance: the CG computed from the CAD part positions vs the x = 0.422 m design balance point.

Run:  uv run python scripts/cad_model.py
Writes: out/cad/rescue_wing_v2.step (assembly), out/cad/stl/*.stl (one per part, for Blender),
        out/cad/fit_check.csv, out/cad/mass_cg.csv, out/cad/run_log.txt
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb
from build123d import (fillet, Axis, Box, Compound, Cylinder, Face, Keep, Line, Location, Plane, Pos, Rot, Sphere, Spline,
                       Wire, Cone, Shell, export_step, export_stl, loft, split, RectangleRounded, Vector)

sys.path.insert(0, str(Path(__file__).parent))
from avl_run import Geometry  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
# Variant: "v2" (milestone 5) or "v3" (15 in folding prop + T-Motor AT3520, design v3): uv run python scripts/cad_model.py v3
VARIANT = sys.argv[1] if len(sys.argv) > 1 else "v2"
OUT = ROOT / "out" / ("cad" if VARIANT == "v2" else f"cad_{VARIANT}")
PROP_IN = 13.0 if VARIANT == "v2" else 15.0          # propeller diameter, in
M_PROP = 0.28 if VARIANT == "v2" else 0.34           # motor + ESC + prop, kg (v3: v3_propulsion.py pick)
MTOW = 7.30 if VARIANT == "v2" else 7.364            # kit-on take-off mass, kg (v3: v3_planform.py)
(OUT / "stl").mkdir(parents=True, exist_ok=True)
M3 = pd.read_csv(ROOT / "out" / "vsp" / "m3_config.csv", index_col=0)["value"]
DP = pd.read_csv(ROOT / "out" / "sizing" / "design_point.csv", index_col=0)["value"]


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


G = Geometry(winglet_h=0.25)
X_CG = float(M3["x_cg"])            # 0.422 m: design balance point, kit bay centred here
Y_BATT = 0.355                      # pack centre span station (milestone 3 layout)
BAY_FWD = 0.006                     # m, kit container shifted forward of the balance point (pod taper)
X_BATT = 0.452                      # wing-root pack centre: 8 mm forward of milestone 3 for skin clearance (wing is thicker there)
SKIN = 0.0015                       # m, composite skin + foam allowance used for clearance checks
POD_X0, POD_LEN, POD_Z = -0.17, 0.82, -0.035
POD_SECTIONS = [(0.08, 0.09, 0.08, 0.035), (0.25, 0.16, 0.15, 0.04), (0.60, 0.16, 0.15, 0.04),
                (0.85, 0.16, 0.15, 0.04), (0.95, 0.09, 0.08, 0.03)]
AF = asb.Airfoil("mh45")
AF_COORDS = AF.coordinates


# ---------------------------------------------------------------- geometry helpers
def airfoil_face(y, chord, x_le, twist_deg, coords=AF_COORDS, te_gap=0.003):
    """Airfoil section in the plane y = const. Twist rotates about the quarter chord (positive = nose up)."""
    th = math.radians(twist_deg)
    xq = x_le + 0.25 * chord
    pts = []
    n = len(coords)
    for i, (u, v) in enumerate(coords):
        # open the trailing edge slightly (te_gap of chord) so the loft has a clean TE face
        v = v + (te_gap / 2) * u * (1 if i < n // 2 else -1)
        x, z = x_le + u * chord, v * chord
        dx = x - xq
        pts.append(Vector(xq + dx * math.cos(th) + z * math.sin(th), y, -dx * math.sin(th) + z * math.cos(th)))
    spl = Spline(*pts)
    te = Line(pts[-1], pts[0])
    return Face(Wire([spl, te]))


def wing_section(y):
    return airfoil_face(y, G.chord(y), G.x_le(y), G.twist(y))


def camber_z(xc, chord):
    """Mean-line height of the airfoil at x/c (no twist)."""
    up = AF.upper_coordinates()
    lo = AF.lower_coordinates()
    zu = np.interp(xc, up[::-1, 0], up[::-1, 1])
    zl = np.interp(xc, lo[:, 0], lo[:, 1])
    return float((zu + zl) / 2 * chord), float((zu - zl) * chord)


def winglet(side):
    """Vertical NACA 0008 winglet on the tip, 0.25 m tall, taper 0.6, 35 deg LE sweep."""
    y = side * G.span / 2
    c0, c1 = G.chord(y), G.chord(y) * G.winglet_taper
    x0 = G.x_le(y)
    x1 = x0 + G.winglet_h * math.tan(math.radians(G.winglet_sweep_deg))
    sym = asb.Airfoil("naca0008").coordinates
    secs = []
    for z, c, xl in ((0.0, c0, x0), (G.winglet_h, c1, x1)):
        pts = [Vector(xl + u * c, y + side * v * c, z) for u, v in sym]
        pts[-1] = pts[0]
        secs.append(Face(Wire([Spline(*pts[:-1]), Line(pts[-2], pts[0])])))
    return loft(secs, ruled=True)


def pod():
    stations = [(0.0, 0.016, 0.016, 0.007)] + POD_SECTIONS + [(1.0, 0.03, 0.03, 0.012)]
    secs = []
    for xl, w, h, rc in stations:
        x = POD_X0 + xl * POD_LEN
        pl = Plane(origin=(x, 0, POD_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
        secs.append(pl * RectangleRounded(w, h, min(rc, 0.49 * min(w, h))))
    return loft(secs)


def hinge_plane(side):
    """Vertical plane through the elevon hinge line (75% chord, break to tip)."""
    y0, y1 = side * G.y_break, side * G.span / 2
    p0 = Vector(G.x_le(y0) + G.hinge_xc * G.chord(y0), y0, 0)
    p1 = Vector(G.x_le(y1) + G.hinge_xc * G.chord(y1), y1, 0)
    d = (p1 - p0).normalized()
    return Plane(origin=p0, x_dir=d, z_dir=Vector(0, 0, 1).cross(d) * -1)


def cell(x, y, z, axis="y"):
    """One 21700 cell, 21 mm x 70 mm."""
    c = Cylinder(0.0105, 0.070)
    rot = Rot(90, 0, 0) if axis == "y" else Rot(0, 0, 0)
    return Pos(x, y, z) * rot * c


# ---------------------------------------------------------------- build
def build():
    parts = {}
    # Wing: centre section (one piece, -0.45..0.45) and two outer panels
    yb = G.y_break
    parts["wing_centre"] = loft([wing_section(-yb), wing_section(0.0), wing_section(yb)], ruled=True)
    for side, nm in ((1, "R"), (-1, "L")):
        panel = loft([wing_section(side * yb), wing_section(side * G.span / 2)], ruled=True)
        pl = hinge_plane(side)
        a = split(panel, bisect_by=pl, keep=Keep.TOP)
        b = split(panel, bisect_by=pl, keep=Keep.BOTTOM)
        fwd, aft = (a, b) if a.center().X < b.center().X else (b, a)
        parts[f"wing_outer_{nm}"] = fwd
        parts[f"elevon_{nm}"] = aft
        parts[f"winglet_{nm}"] = winglet(side)

    # Pod, with the bay door cut out of its belly (door kept as its own part)
    p = pod()
    door_cut = Pos(X_CG, 0, POD_Z - 0.07) * Box(0.205, 0.110, 0.04)
    parts["pod"] = p - door_cut
    parts["bay_door"] = p & door_cut

    # Kit bay: container (220 x 130 x 120 mm outside, shifted forward of the balance point because the pod
    # narrows behind it), packed kit with its parachute pack on top (both centred on the balance point,
    # so the drop moves the CG as little as possible), release servo
    kz = POD_Z - 0.003
    xb = X_CG - BAY_FWD
    outer = Pos(xb, 0, kz) * fillet(Box(0.22, 0.13, 0.12).edges().filter_by(Axis.X), 0.012)
    parts["kit_container"] = outer - (Pos(xb, 0, kz - 0.004) * Box(0.212, 0.122, 0.12))
    parts["rescue_kit"] = Pos(X_CG, 0, kz - 0.06 + 0.035 + 0.001) * Box(0.19, 0.11, 0.07)
    parts["kit_parachute"] = Pos(X_CG, 0, kz - 0.06 + 0.07 + 0.0175 + 0.002) * Box(0.19, 0.10, 0.035)
    parts["release_servo"] = Pos(xb - 0.106 + 0.0065, 0, kz + 0.03) * Box(0.012, 0.023, 0.016)   # in the gap ahead of the kit

    # Alternative bay load (not in the main assembly): 18 cells upright, 6S3P, 126 x 63 mm floor
    mod = [Pos(X_CG - 0.0525 + i * 0.021, -0.021 + j * 0.021, kz - 0.017) * Cylinder(0.0105, 0.070)
           for i in range(6) for j in range(3)]
    bay_battery = Compound(children=mod)

    # Wing-root packs: per side 5 cells chordwise x 3 spanwise, axis along the span, one layer
    for side, nm in ((1, "R"), (-1, "L")):
        cells = []
        yc = side * Y_BATT
        xc_c = (X_BATT - G.x_le(yc)) / G.chord(yc)
        zc, _ = camber_z(xc_c, G.chord(yc))
        for i in range(5):
            for j in range(3):
                cells.append(cell(X_BATT + (i - 2) * 0.0215, yc + (j - 1) * 0.071, zc))
        parts[f"battery_{nm}"] = Compound(children=cells)

    # Spar joiners: 16 mm carbon tubes along the quarter-chord line, 0.30 m each side of the break
    for side, nm in ((1, "R"), (-1, "L")):
        y0, y1 = side * (yb - 0.30), side * (yb + 0.30)
        p0 = Vector(G.x_le(y0) + 0.25 * G.chord(y0), y0, camber_z(0.25, G.chord(y0))[0])
        p1 = Vector(G.x_le(y1) + 0.25 * G.chord(y1), y1, camber_z(0.25, G.chord(y1))[0])
        d = p1 - p0
        tube = Cylinder(0.008, d.length) - Cylinder(0.0065, d.length)
        parts[f"spar_joiner_{nm}"] = Location(Plane(origin=(p0 + p1) * 0.5, z_dir=d)) * tube

    # Elevon servos: thin wing servos lying flat, 60% along the elevon span, ahead of the hinge
    for side, nm in ((1, "R"), (-1, "L")):
        ys = side * (yb + 0.4 * (G.span / 2 - yb))
        c = G.chord(ys)
        xs = G.x_le(ys) + 0.60 * c
        zs, _ = camber_z(0.60, c)
        parts[f"elevon_servo_{nm}"] = Pos(xs, ys, zs) * Box(0.030, 0.032, 0.010)

    # Avionics in the pod
    parts["flight_controller"] = Pos(0.25, 0, POD_Z - 0.01) * Box(0.10, 0.07, 0.035)   # moved aft 50 mm to balance the packs
    parts["esc"] = Pos(0.565, 0, POD_Z) * Box(0.06, 0.035, 0.018)
    parts["gps_mast"] = Pos(0.12, 0, POD_Z + 0.075 + 0.03) * Cylinder(0.004, 0.06)
    parts["gps"] = Pos(0.12, 0, POD_Z + 0.075 + 0.065) * Cylinder(0.03, 0.012)
    parts["pitot"] = Pos(POD_X0 + 0.02 - 0.06, 0, POD_Z) * Rot(0, 90, 0) * Cylinder(0.003, 0.14)

    # Gimbal under the nose: mount + 90 mm ball + lens window
    parts["gimbal_mount"] = Pos(0.07, 0, POD_Z - 0.065) * Cylinder(0.03, 0.02)
    parts["gimbal_ball"] = Pos(0.07, 0, -0.115) * Sphere(0.045)
    parts["gimbal_lens"] = Pos(0.07 - 0.032, 0, -0.115 - 0.026) * Rot(0, -60, 0) * Cylinder(0.018, 0.012)

    # Pusher motor, spinner and a 13 in two-blade prop behind the pod tail
    xm = POD_X0 + POD_LEN
    parts["motor"] = Pos(xm + 0.02, 0, POD_Z) * Rot(0, 90, 0) * Cylinder(0.026, 0.04)
    parts["spinner"] = Pos(xm + 0.07, 0, POD_Z) * Rot(0, 90, 0) * Cone(0.022, 0.003, 0.05)
    # two blades in one straight line, clocked 25 deg (blade from 15 mm hub radius to the tip)
    r_tip = PROP_IN * 0.0254 / 2
    blades = [Pos(0, s * (0.015 + r_tip) / 2, 0) * Box(0.006, r_tip - 0.015, 0.026) for s in (1, -1)]
    parts["propeller"] = Pos(xm + 0.055, 0, POD_Z) * Rot(25, 0, 0) * Compound(children=blades)

    # Belly skid for landings (replaceable Kevlar strip)
    parts["belly_skid"] = Pos(0.27, 0, POD_Z - 0.076) * Box(0.30, 0.025, 0.004)
    return parts, bay_battery


# ---------------------------------------------------------------- checks
def clearance(part, parent):
    """Min distance from part to the parent's outer skin, and the share of the part's volume inside."""
    inside = (part & parent).volume / part.volume if part.volume > 0 else float("nan")
    d = part.distance_to(Shell(parent.faces()))
    return inside, d


def main():
    sys.stdout = Tee(OUT / "run_log.txt")
    print(f"Rescue Wing {VARIANT} CAD, 2026-11-12, claude-opus-5-5 (build123d); prop {PROP_IN:.0f} in\n")
    parts, bay_battery = build()

    pod_solid = pod()
    wing_all = parts["wing_centre"] + parts["wing_outer_R"] + parts["wing_outer_L"]
    rows = []
    checks = [("battery_R", "wing", wing_all), ("battery_L", "wing", wing_all),
              ("spar_joiner_R", "wing", wing_all), ("elevon_servo_R", "wing", wing_all),
              ("kit_container", "pod", pod_solid), ("flight_controller", "pod", pod_solid),
              ("esc", "pod", pod_solid)]
    for name, pname, parent in checks:
        part = parts[name]
        solids = part.solids() if hasattr(part, "solids") else [part]
        worst_in, worst_d = 1.0, 1e9
        for s in solids:
            i, d = clearance(s, parent)
            worst_in, worst_d = min(worst_in, i), min(worst_d, d)
        rows.append(dict(part=name, parent=pname, share_inside=worst_in, min_clearance_mm=worst_d * 1000,
                         ok=worst_in > 0.999 and worst_d >= SKIN))
    skins = {"wing_centre", "wing_outer_R", "wing_outer_L", "elevon_R", "elevon_L", "winglet_R", "winglet_L",
             "pod", "bay_door"}
    inner = [n for n in parts if n not in skins]
    clashes, tested = [], 0
    ATTACHED = [{"gimbal_mount", "gimbal_ball"}, {"gimbal_ball", "gimbal_lens"}, {"gps_mast", "gps"},
                {"spinner", "propeller"}]   # parts that are bolted together, so they touch by design
    bbs = {n: parts[n].bounding_box() for n in inner}

    def bb_overlap(a, b):
        return (a.min.X < b.max.X and b.min.X < a.max.X and a.min.Y < b.max.Y and b.min.Y < a.max.Y
                and a.min.Z < b.max.Z and b.min.Z < a.max.Z)

    for ii, a_ in enumerate(inner):
        for b_ in inner[ii + 1:]:
            if {a_, b_} in ATTACHED:
                continue
            if not bb_overlap(bbs[a_], bbs[b_]):
                continue
            tested += 1
            v = (parts[a_] & parts[b_]).volume
            if v > 1e-9:
                clashes.append((a_, b_, v * 1e6))
    print(f"\nInterference between internal parts ({len(inner)} parts; {tested} pairs close enough to test): "
          + ("none" if not clashes else "; ".join(f"{a_} x {b_}: {v:.2f} cm3" for a_, b_, v in clashes)))
    i, d = clearance(bay_battery, pod_solid)
    rows.append(dict(part="bay_battery_module (alt)", parent="pod", share_inside=i, min_clearance_mm=d * 1000,
                     ok=i > 0.999 and d >= SKIN))
    # Propeller disc vs the wing trailing edge (the swept wing's trailing edge runs back outboard)
    xm = POD_X0 + POD_LEN
    disc = Pos(xm + 0.055, 0, POD_Z) * Rot(0, 90, 0) * Cylinder(PROP_IN * 0.0254 / 2, 0.03)
    d_wing = disc.distance_to(wing_all)
    d_elev = min(disc.distance_to(parts["elevon_R"]), disc.distance_to(parts["elevon_L"]))
    print(f"Propeller disc ({PROP_IN:.0f} in, 30 mm thick) to the wing: {d_wing * 1000:.1f} mm; to the elevons: {d_elev * 1000:.1f} mm")
    rows.append(dict(part=f"propeller disc {PROP_IN:.0f} in", parent="wing (gap)", share_inside=0.0,
                     min_clearance_mm=d_wing * 1000, ok=d_wing >= 0.02))
    fit = pd.DataFrame(rows)
    fit.round(4).to_csv(OUT / "fit_check.csv", index=False)
    print(f"Fit check (part fully inside its parent's outer surface, with >= {SKIN * 1000:.1f} mm for the skin):")
    print(fit.round(3).to_string(index=False))

    # Balance from the CAD: masses from the v2 budget, positions from the CAD centroids
    m_prop = M_PROP                                 # motor + ESC + prop budget
    budget = {
        "wing structure (centre, outer panels, elevons)": (float(DP["m_wing"]),
            ["wing_centre", "wing_outer_R", "wing_outer_L", "elevon_R", "elevon_L"]),
        "winglets": (2 * 0.25 * 0.266 * 0.8 * 1.0, ["winglet_R", "winglet_L"]),
        "batteries (P50B 30 cells)": (30 * 0.071 / 0.88, ["battery_R", "battery_L"]),
        "rescue kit": (1.50, ["rescue_kit"]),
        "kit parachute": (0.124, ["kit_parachute"]),
        "kit container + release": (0.08, ["kit_container", "release_servo"]),
        "gimbal (thermal + RGB)": (0.60, ["gimbal_ball", "gimbal_mount"]),
        "avionics (FC, GPS, radio)": (0.35, ["flight_controller", "gps"]),
        "pod structure + door + skid": (0.30, ["pod", "bay_door", "belly_skid"]),
        "motor + ESC + prop": (m_prop, ["motor", "esc", "propeller", "spinner"]),
    }
    mrows = []
    for item, (m, names) in budget.items():
        vols = [(parts[n].volume, parts[n].center()) for n in names]
        vt = sum(v for v, _ in vols)
        x = sum(v * c.X for v, c in vols) / vt
        z = sum(v * c.Z for v, c in vols) / vt
        mrows.append(dict(item=item, mass_kg=m, x=x, z=z, source="CAD centroid (volume-weighted)"))
    m_sofar = sum(r["mass_kg"] for r in mrows)
    mtow = MTOW                                       # P50B kit configuration
    mrows.append(dict(item="misc (wiring, glue) 5%", mass_kg=mtow - m_sofar, x=0.35, z=-0.02,
                      source="not modelled; same assumption as milestone 2"))
    mc = pd.DataFrame(mrows)
    x_cg = (mc.mass_kg * mc.x).sum() / mc.mass_kg.sum()
    z_cg = (mc.mass_kg * mc.z).sum() / mc.mass_kg.sum()
    kit_rows = mc.item.str.startswith(("rescue kit", "kit parachute"))
    x_cg_empty = (mc[~kit_rows].mass_kg * mc[~kit_rows].x).sum() / mc[~kit_rows].mass_kg.sum()
    mc.round(4).to_csv(OUT / "mass_cg.csv", index=False)
    print("\nBalance from the CAD layout:")
    print(mc.round(3).to_string(index=False))
    mac = G.mac
    print(f"\nTotal {mc.mass_kg.sum():.2f} kg. CG x = {x_cg:.4f} m (design {X_CG:.4f} m, "
          f"difference {(x_cg - X_CG) * 1000:+.1f} mm = {(x_cg - X_CG) / mac * 100:+.1f}% MAC); z = {z_cg:.3f} m")
    print(f"After the kit drops: CG x = {x_cg_empty:.4f} m ({(x_cg_empty - x_cg) * 1000:+.1f} mm)")
    pd.Series(dict(x_cg_cad=x_cg, z_cg_cad=z_cg, x_cg_cad_kit_dropped=x_cg_empty, x_cg_design=X_CG,
                   mass=mc.mass_kg.sum())).to_csv(OUT / "cg_summary.csv", header=["value"])

    # Export
    for n, s in parts.items():
        export_stl(s, str(OUT / "stl" / f"{n}.stl"), tolerance=0.0004, angular_tolerance=0.15)
    export_stl(bay_battery, str(OUT / "stl" / "alt_bay_battery.stl"), tolerance=0.0004)
    for n, s in parts.items():
        s.label = n
    bay_battery.label = "alt_bay_battery"
    asm = Compound(children=list(parts.values()), label="RescueWing_v2")
    export_step(asm, str(OUT / f"rescue_wing_{VARIANT}.step"))
    export_step(bay_battery, str(OUT / "alt_bay_battery_module.step"))
    print(f"\nExported {len(parts)} parts: {OUT.relative_to(ROOT)}/rescue_wing_{VARIANT}.step and stl/")


if __name__ == "__main__":
    main()
