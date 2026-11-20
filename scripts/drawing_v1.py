"""Lodestar v1 general arrangement drawing (sheet LOD-GA-001), A3 landscape, scale 1:15.

Three orthographic views and an isometric of the CAD v4 exterior (out/cad_v4/lodestar_v4.step, built by
cad_v4.py), projected hidden-line as vectors by OpenCASCADE inside FreeCAD (fcad/fc_project.py), then placed at
scale and dimensioned in matplotlib in model units. Properties,
mass summary and title block read lodestar_v1.py and the analysis outputs, so the sheet cannot drift from the
frozen design.

Run:    ../.venv/Scripts/python.exe -X utf8 drawing_v1.py      (from scripts/)
Writes: docs/drawings/LOD-GA-001_general_arrangement.pdf and .png
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import lodestar_v1 as v1

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "drawings"
SCALE = 15.0                          # sheet mm per model mm = 1/SCALE
SHEET = (420.0, 297.0)                # A3 landscape, mm
STEP = ROOT / "out" / "cad_v4" / "lodestar_v4.step"
ISO_DIR = (0.9, 0.55, -0.42)          # isometric view direction, eye towards model
LW_VISIBLE = 0.85                     # pt, visible outlines
INK, THIN = "#111111", 0.35
ROW_H = 5.0                           # table row height, mm
EXTERIOR = ["wing_centre", "wing_outer_L", "wing_outer_R", "winglet_L", "winglet_R", "elevon_L", "elevon_R", "pod",
            "bay_door", "belly_skid", "gimbal_ball", "gimbal_lens", "gps_mast", "gps", "pitot", "propeller", "spinner",
            "motor"]
FONT = ["Segoe UI", "Arial", "DejaVu Sans"]   # ISO 3098-style plain sans; lettering 2.5 / 3.5 / 5 mm
plt.rcParams.update({"font.family": FONT, "font.size": 8, "lines.linewidth": THIN,
                     "text.color": INK, "axes.edgecolor": INK})

# view: camera direction (from camera to model), view-up, and how model axes map to sheet (u, v)
VIEWS = {
    "PLAN":  dict(direction=(0, 0, -1), up=(-1, 0, 0), u=lambda p: p[1], v=lambda p: -p[0]),
    "FRONT": dict(direction=(1, 0, 0), up=(0, 0, 1), u=lambda p: -p[1], v=lambda p: p[2]),
    "SIDE":  dict(direction=(0, 1, 0), up=(0, 0, 1), u=lambda p: p[0], v=lambda p: p[2]),
}


def project():
    """Hidden-line vector projection of the CAD v4 exterior, done by OpenCASCADE inside FreeCAD (fcad/fc_project.py).

    Returns {view: (lines, ext)} for the orthographic views, lines in sheet-view model coordinates (u, v) in metres,
    and the isometric lines turned so the model +z axis points up the sheet.
    """
    import json
    import os
    import subprocess
    import tempfile

    from fcad.runner import freecadcmd

    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        job = {"step": str(STEP), "out": str(Path(tmp) / "proj.json"), "parts": EXTERIOR, "deflection_mm": 0.05,
               "views": {k: [-c for c in vw["direction"]] for k, vw in VIEWS.items()} | {"ISO": [-c for c in ISO_DIR]}}
        (Path(tmp) / "job.json").write_text(json.dumps(job))
        env = dict(os.environ, FCAD_JOB=str(Path(tmp) / "job.json"))
        subprocess.run([str(freecadcmd()), str(Path(__file__).parent / "fcad" / "fc_project.py")], env=env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
        res = json.loads(Path(job["out"]).read_text())
    if "error" in res or res.get("missing_parts"):
        raise RuntimeError(res.get("error") or f"parts not in STEP: {res['missing_parts']}")
    out = {}
    for k in VIEWS:
        # TechDraw plane (x, y) in mm -> sheet view (u, v) in m: u = y, v = -x for all three views
        lines = [np.array([[q[1], -q[0]] for q in ln]) / 1000 for ln in res["views"][k]["visible"]]
        pts = np.vstack(lines)
        pad = 0.03
        ext = (pts[:, 0].min() - pad, pts[:, 0].max() + pad, pts[:, 1].min() - pad, pts[:, 1].max() + pad)
        out[k] = (lines, ext)
    ux, uy = res["views"]["ISO"]["up"]
    ang = math.atan2(ux, uy)                                  # rotate so "up" lands on +y
    rot = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
    out["ISO"] = [np.array(ln) @ rot.T / 1000 for ln in res["views"]["ISO"]["visible"]]
    return out


def place(fig, lines, ext, x0_mm, y0_mm, lw=LW_VISIBLE):
    """Draw a projected view on the sheet at 1:SCALE with its lower-left corner at (x0, y0) mm; return the axes."""
    umin, umax, vmin, vmax = ext
    w_mm, h_mm = (umax - umin) * 1000 / SCALE, (vmax - vmin) * 1000 / SCALE
    ax = fig.add_axes([x0_mm / SHEET[0], y0_mm / SHEET[1], w_mm / SHEET[0], h_mm / SHEET[1]])
    for ln in lines:
        ax.plot(ln[:, 0], ln[:, 1], color=INK, lw=lw, solid_capstyle="round", solid_joinstyle="round")
    ax.set_xlim(umin, umax)
    ax.set_ylim(vmin, vmax)
    ax.axis("off")
    ax.set_clip_on(False)
    return ax


def dim(ax, a, b, off, text, horizontal=True, fs=8.5):
    """Linear dimension between model points a and b (u, v), offset perpendicular by `off` metres."""
    (ua, va), (ub, vb) = a, b
    if horizontal:
        y = (va if off > 0 else va) + off
        ax.plot([ua, ua], [va, y + 0.008 * np.sign(off)], color=INK, lw=0.25, clip_on=False)
        ax.plot([ub, ub], [vb, y + 0.008 * np.sign(off)], color=INK, lw=0.25, clip_on=False)
        ax.annotate("", (ua, y), (ub, y), arrowprops=dict(arrowstyle="<|-|>", lw=0.35, color=INK,
                                                         shrinkA=0, shrinkB=0, mutation_scale=5), annotation_clip=False)
        ax.text((ua + ub) / 2, y + 0.006 * np.sign(off), text, ha="center",
                va="bottom" if off > 0 else "top", fontsize=fs, clip_on=False,
                bbox=dict(fc="white", ec="none", pad=0.4))
    else:
        x = ua + off
        ax.plot([ua, x + 0.008 * np.sign(off)], [va, va], color=INK, lw=0.25, clip_on=False)
        ax.plot([ub, x + 0.008 * np.sign(off)], [vb, vb], color=INK, lw=0.25, clip_on=False)
        ax.annotate("", (x, va), (x, vb), arrowprops=dict(arrowstyle="<|-|>", lw=0.35, color=INK,
                                                         shrinkA=0, shrinkB=0, mutation_scale=5), annotation_clip=False)
        ax.text(x + 0.006 * np.sign(off), (va + vb) / 2, text, ha="left" if off > 0 else "right", va="center",
                rotation=90, fontsize=fs, clip_on=False, bbox=dict(fc="white", ec="none", pad=0.4))


def label(ax, text, ext):
    umin, umax, vmin, vmax = ext
    ax.text((umin + umax) / 2, vmin - 0.035, text, ha="center", va="top", fontsize=10.5, fontweight="bold",
            clip_on=False)


def table(fig, x_mm, y_top_mm, w_mm, title, rows, col_frac=0.5, row_h=ROW_H, fs=8):
    ax = fig.add_axes([0, 0, 1, 1], facecolor="none")
    ax.set_xlim(0, SHEET[0])
    ax.set_ylim(0, SHEET[1])
    ax.axis("off")
    h = row_h * (len(rows) + 1)
    y0 = y_top_mm - h
    ax.add_patch(plt.Rectangle((x_mm, y0), w_mm, h, fill=False, lw=0.5, ec=INK))
    ax.add_patch(plt.Rectangle((x_mm, y_top_mm - row_h), w_mm, row_h, fc="#e9e9e9", lw=0.5, ec=INK))
    ax.text(x_mm + 1.5, y_top_mm - row_h / 2, title, va="center", fontsize=fs + 0.6, fontweight="bold")
    for i, (k, val) in enumerate(rows):
        y = y_top_mm - row_h * (i + 1.5)
        ax.text(x_mm + 1.5, y, k, va="center", fontsize=fs)
        ax.text(x_mm + w_mm * col_frac, y, val, va="center", fontsize=fs)
        ax.plot([x_mm, x_mm + w_mm], [y - row_h / 2, y - row_h / 2], lw=0.15, color="#999999")
    ax.plot([x_mm + w_mm * col_frac - 1.5] * 2, [y0, y_top_mm - row_h], lw=0.25, color=INK)
    return y0


def properties():
    g, m, c, o, s = v1.GEOMETRY, v1.MASS, v1.CONTROLS, v1.OPERATIONS, v1.STRUCTURE
    perf = pd.read_csv(ROOT / "out" / "performance_v1" / "endurance_summary.csv")
    e = perf[(perf.cell == "SiCore") & (perf.bay == "rescue kit")].set_index("case")
    eb = perf[(perf.cell == "SiCore") & (perf.bay != "rescue kit")].set_index("case")
    pv1 = pd.read_csv(ROOT / "out" / "wing_trade" / "performance_v1.csv")
    stall = float(pv1[pv1.cell.str.startswith("Amprius")].stall_eas_mps.iloc[0])
    dash = pd.read_csv(ROOT / "out" / "performance_v1" / "wind_dash.csv").set_index("aircraft")
    ar = g["span"] ** 2 / g["S_ref"]
    geo = [("Span", f"{g['span'] * 1000:.0f} mm"), ("Wing area (ref)", f"{g['S_ref']:.3f} m²"),
           ("Aspect ratio", f"{ar:.2f}"), ("MAC", f"{g['mac'] * 1000:.0f} mm"),
           ("Root / tip chord", f"{g['c_root'] * 1000:.0f} / {g['c_tip'] * 1000:.0f} mm"),
           ("Sweep (c/4) / washout", f"{g['sweep_qc_deg']:.0f}° / {g['washout_deg']:.0f}°"),
           ("Airfoil / winglet", f"{g['airfoil']} / {g['winglet_airfoil']}"),
           ("Winglet height", f"{g['winglet_h'] * 1000:.0f} mm, {g['winglet_sweep_deg']:.0f}° sweep"),
           ("Elevon hinge", f"{g['hinge_xc'] * 100:.0f}% chord"),
           ("Wing pieces", f"{g['pieces']}, tube-jointed")]
    mass = [("Take-off mass, SiCore", f"{m['sicore_loaded_kg']:.2f} kg (limit {m['mtow_limit_kg']:.0f} kg)"),
            ("After kit release", f"{m['sicore_released_kg']:.2f} kg"),
            ("Wing structure", f"{s['wing_mass_kg']:.2f} kg"),
            ("Rescue kit", f"{m['rescue_kit_kg']:.2f} kg + chute"),
            ("CG, loaded / released", f"{m['x_cg_sicore_loaded'] * 1000:.0f} / {m['x_cg_sicore_released'] * 1000:.0f} mm"),
            ("CG limits fwd / aft", f"{c['cg_fwd_m'] * 1000:.0f} / {c['cg_aft_m'] * 1000:.0f} mm"),
            ("Static margin", f"{m['static_margin_loaded_pct']:.1f} / {m['static_margin_released_pct']:.1f} % MAC")]
    perf_rows = [("Stall, CLmax 0.80", f"{stall:.1f} m/s EAS"),
                 ("Search speed", f"{o['search_speed_tas'][0]:.0f}-{o['search_speed_tas'][1]:.0f} m/s TAS"),
                 ("Max level speed", f"{dash.loc['Lodestar SiCore', 'v_max_mps']:.1f} m/s"),
                 ("Endurance, kit, 2,000 m", f"{e.loc['design case', 'nominal_h']:.2f} h (P10 {e.loc['design case', 'p10_h']:.2f})"),
                 ("Endurance, search only", f"{eb.loc['design case', 'nominal_h']:.1f} h (battery module)"),
                 ("Limit load factor", f"+{s['n_pos']:.1f} / {s['n_neg']:.1f} g, FoS {s['fos']}"),
                 ("VD", f"{s['vd_eas']:.0f} m/s EAS")]
    sysrows = [("Motor / prop", "T-Motor AT3520-550 / 15x10 folding"),
               ("Battery", "SiCore 6.3 Ah 6S5P, 680 Wh"),
               ("Search camera", "2 x Boson 640, 95° splayed nadir"),
               ("Gimbal", "SIYI ZT6 (640 thermal + RGB)"),
               ("Recovery", "belly landing, 70 mm EPP skid"),
               ("Safety", "parachute (F3322), ADS-B In, 2 GNSS")]
    return geo, mass, perf_rows, sysrows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    views = project()
    fig = plt.figure(figsize=(SHEET[0] / 25.4, SHEET[1] / 25.4))
    # sheet border and frame
    fr = fig.add_axes([0, 0, 1, 1], facecolor="none")
    fr.set_xlim(0, SHEET[0])
    fr.set_ylim(0, SHEET[1])
    fr.axis("off")
    fr.add_patch(plt.Rectangle((10, 10), SHEET[0] - 20, SHEET[1] - 20, fill=False, lw=0.9, ec=INK))
    for i in range(1, 8):
        x = 10 + i * (SHEET[0] - 20) / 8
        fr.plot([x, x], [10, 13], lw=0.4, color=INK)
        fr.plot([x, x], [SHEET[1] - 13, SHEET[1] - 10], lw=0.4, color=INK)
    for i in range(8):
        fr.text(10 + (i + 0.5) * (SHEET[0] - 20) / 8, SHEET[1] - 11.5, str(i + 1), ha="center", va="center", fontsize=7)

    g = v1.GEOMETRY
    # PLAN view, upper left
    lines, ext = views["PLAN"]
    ax = place(fig, lines, ext, 26, 128)
    label(ax, "PLAN VIEW", ext)
    half = g["span"] / 2
    x_le_tip = half * math.tan(math.radians(g["sweep_qc_deg"])) + g["c_root"] / 4 - g["c_tip"] / 4
    dim(ax, (-half, ext[3] - 0.03), (half, ext[3] - 0.03), 0.07, f"{g['span'] * 1000:.0f} SPAN")
    dim(ax, (half, -x_le_tip), (half, -x_le_tip - g["c_tip"]), 0.08, f"{g['c_tip'] * 1000:.0f} TIP", horizontal=False)
    pod_x0, pod_x1 = g["pod_x0"], g["pod_x0"] + g["pod_len"]
    dim(ax, (-0.12, -pod_x0), (-0.12, -pod_x1), -0.08, f"{g['pod_len'] * 1000:.0f} POD", horizontal=False)
    dim(ax, (0.0, 0.0), (0.0, -g["c_root"]), 0.30, f"{g['c_root'] * 1000:.0f} ROOT", horizontal=False)
    xmin, xmax = -views["PLAN"][1][3] + 0.03, -views["PLAN"][1][2] - 0.03   # model x from plan v = -x
    dim(ax, (-half, -xmin), (-half, -xmax), -0.10, f"{(xmax - xmin) * 1000:.0f} OVERALL", horizontal=False)
    ax.text(half * 0.52, -(half * 0.52 * math.tan(math.radians(g["sweep_qc_deg"])) + g["c_root"] / 4) + 0.10,
            f"{g['sweep_qc_deg']:.0f}° SWEEP (c/4)", fontsize=8.5, ha="center",
            bbox=dict(fc="white", ec="none", pad=0.6))
    # tip chord edges: the tip face is edge-on from above, which OpenCASCADE's HLR leaves out
    # and the leading edge where it meets the pod, which it also drops (straight taper: x_le = y / half * x_le_tip)
    for side in (-1, 1):
        ax.plot([side * half] * 2, [-x_le_tip, -x_le_tip - g["c_tip"]], color=INK, lw=LW_VISIBLE)
        ys = np.array([0.026, 0.16])
        ax.plot(side * ys, -ys / half * x_le_tip, color=INK, lw=LW_VISIBLE, solid_capstyle="butt")
    # quarter-chord line, chain line
    yq = np.array([-half, 0, half])
    xq = g["c_root"] / 4 + np.abs(yq) * math.tan(math.radians(g["sweep_qc_deg"]))
    ax.plot(yq, -xq, color=INK, lw=0.25, ls=(0, (8, 2, 1, 2)), clip_on=False)
    ax.plot([0, 0], [ext[2] + 0.01, ext[3] - 0.01], color=INK, lw=0.25, ls=(0, (8, 2, 1, 2)), clip_on=False)
    # CG and neutral point marks
    for xc, txt in ((v1.MASS["x_cg_sicore_loaded"], "CG"),):
        ax.plot(0, -xc, marker="o", ms=4, mfc="white", mec=INK, mew=0.6, clip_on=False)
        ax.plot([-0.012, 0.012], [-xc, -xc], color=INK, lw=0.5)
        ax.annotate(f"{txt} {xc * 1000:.0f} (SiCore, kit on)", (0.006, -xc), (0.16, -xc - 0.05), fontsize=8,
                    va="center", arrowprops=dict(arrowstyle="-", lw=0.4, color=INK, shrinkA=1, shrinkB=2),
                    bbox=dict(fc="white", ec="none", pad=0.6))

    # FRONT view, below plan
    lines, ext = views["FRONT"]
    ax = place(fig, lines, ext, 26, 62)
    label(ax, "FRONT VIEW", ext)
    zmin, zmax = ext[2] + 0.03, ext[3] - 0.03
    dim(ax, (half + 0.02, zmin), (half + 0.02, zmax), 0.06, f"{(zmax - zmin) * 1000:.0f}", horizontal=False)
    dim(ax, (-half, 0.02), (-half, g["winglet_h"]), -0.06, f"{g['winglet_h'] * 1000:.0f} WINGLET", horizontal=False)

    # SIDE view, upper right
    lines, ext = views["SIDE"]
    ax = place(fig, lines, ext, 262, 222)
    ax.text((ext[0] + ext[1]) / 2, ext[2] - 0.11, "SIDE VIEW (LEFT)", ha="center", va="top", fontsize=10.5,
            fontweight="bold", clip_on=False)
    dim(ax, (xmin, zmin - 0.01), (xmax, zmin - 0.01), -0.04, f"{(xmax - xmin) * 1000:.0f} OVERALL")
    dim(ax, (pod_x0, zmax + 0.01), (pod_x1, zmax + 0.01), 0.04, f"{g['pod_len'] * 1000:.0f} POD")

    # isometric, not to scale, lower right
    iso = views["ISO"]
    ip = np.vstack(iso)
    w_mm = 84.0
    h_mm = w_mm * np.ptp(ip[:, 1]) / np.ptp(ip[:, 0])
    axi = fig.add_axes([324 / SHEET[0], 47 / SHEET[1], w_mm / SHEET[0], h_mm / SHEET[1]])
    for ln in iso:
        axi.plot(ln[:, 0], ln[:, 1], color=INK, lw=0.7, solid_capstyle="round", solid_joinstyle="round")
    axi.set_xlim(ip[:, 0].min(), ip[:, 0].max())
    axi.set_ylim(ip[:, 1].min(), ip[:, 1].max())
    axi.axis("off")
    fr.text(252, 60, "ISOMETRIC" + chr(10) + "(NOT TO SCALE)", ha="left", va="top", fontsize=10.5, fontweight="bold")

    geo, mass, perf, sysrows = properties()
    y = table(fig, 250, 196, 75, "GEOMETRY", geo)
    table(fig, 330, 196, 80, "MASS AND BALANCE", mass)
    table(fig, 330, 196 - ROW_H * (len(mass) + 1) - 3, 80, "PERFORMANCE (SiCore)", perf)
    table(fig, 250, y - 3, 75, "SYSTEMS", sysrows, col_frac=0.29)

    # notes
    notes = ["NOTES",
             "1. Dimensions in mm. Datum: wing root leading edge; x aft, y right, z up.",
             "2. Geometry: scripts/lodestar_v1.py; CAD: out/cad_v4/lodestar_v4.step (FreeCAD).",
             "3. Performance at 2,000 m DA, +20 °C, kit carried; endurance from a 4,000-draw Monte Carlo.",
             "4. Elevons ±20°, 2:1 linkage, KST A12-S HV servos; mass balance 60 g each.",
             "5. Construction: EPS 20 core, ±45° glass 80 g/m² skin, pultruded carbon spar caps."]
    for i, n in enumerate(notes):
        fr.text(26, 40 - i * 4.8, n, fontsize=8.5, fontweight="bold" if i == 0 else "normal")

    # title block, lower right
    fr.add_patch(plt.Rectangle((250, 14), 160, 30, fill=False, lw=0.8, ec=INK))
    for yy in (24, 34):
        fr.plot([250, 410], [yy, yy], lw=0.4, color=INK)
    for xx in (330, 370):
        fr.plot([xx, xx], [14, 24], lw=0.4, color=INK)
    fr.text(254, 39, "LODESTAR v1  CIVIL SEARCH-AND-RESCUE FLYING WING", fontsize=11, fontweight="bold", va="center")
    fr.text(254, 29, "GENERAL ARRANGEMENT", fontsize=14, fontweight="bold", va="center")
    cells = [(254, "DWG NO.", "LOD-GA-001"), (334, "REV / DATE", f"{v1.VERSION} / {v1.FROZEN}"),
             (374, "SCALE / SHEET", f"1:{SCALE:g}  A3  1/1")]
    for xx, k, val in cells:
        fr.text(xx, 21.5, k, fontsize=7, va="center", color="#444444")
        fr.text(xx, 17.0, val, fontsize=9.5, va="center")
    fr.text(372, 29, "THIRD-ANGLE PROJECTION", fontsize=7.5, va="center")

    for ext_ in ("pdf", "png"):
        fig.savefig(OUT / f"LOD-GA-001_general_arrangement.{ext_}", dpi=300)
    print("wrote", OUT / "LOD-GA-001_general_arrangement.pdf")


if __name__ == "__main__":
    main()
