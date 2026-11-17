"""Can moving the wing battery packs restore the 6% static margin on the compact XPS wing?

The heavier wing (wing_trade.py) moved the CG aft and cut the level-trim static margin to ~4.4-4.6% MAC.
This sweeps the pack position forward (dx) and inboard (dy), recomputes CG and the AVL neutral point
for all four mass states (P50B/SiCore, kit on/off), and checks the shifted 21700 cells against the
compact joint sleeves and the wing skin in the CAD lofts.

Run:  uv run python -X utf8 scripts/battery_trim.py
Writes: out/wing_trade/battery_trim.csv
"""
from pathlib import Path
import sys

sys.argv[1:] = ["v3"]                       # cad_model reads its variant from argv at import
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

import structures as s  # noqa: E402
import wing_trade as wt  # noqa: E402

TARGET = 6.0                                # % MAC, design target
DX = np.arange(0, -0.041, -0.005)           # m, negative = forward
DY = (0.0, -0.025, -0.050)                  # m, negative = inboard


def margins(cad, tab, joint_masses, dx, dy):
    cad = cad.copy()
    cad.loc[cad.item.str.startswith("batteries"), "x"] += dx
    y0 = s.Y_BATT
    s.Y_BATT = (y0[0] + dy, y0[1] + dy)
    try:
        rows = []
        for cell in ("P50B", "SiCore"):
            for loaded in (True, False):
                mi, _ = wt.mass_properties(cad, tab, joint_masses, loaded, cell)
                _, level, _ = wt.solve_strips(1.0, s.VC, s.RHO0, round(mi["m"], 3), round(mi["x"], 5))
                rows.append(dict(cell=cell, kit=loaded, cg_m=mi["x"], elevon_trim_deg=level["elevon"],
                                 sm_pct=100 * (level["Xnp"] - mi["x"]) / s.GEO.mac))
        return rows
    finally:
        s.Y_BATT = y0


# Same layout and constants as cad_model.py (which needs the OCP CAD kernel; this check does not).
X_BATT, Y_BATT_CAD, CELL_R, CELL_L, CELL_DX, CELL_DY, SKIN = 0.452, 0.355, 0.0105, 0.070, 0.0215, 0.071, 0.0015
_UP, _LO = s.AF.upper_coordinates()[::-1], s.AF.lower_coordinates()


def surfaces(xc, chord, y=None):
    """Upper and lower airfoil surface heights (m) at x/c. Untwisted as cad_model.camber_z when y is None;
    otherwise as the CAD loft: 3 mm-of-chord trailing-edge opening and twist about the quarter chord
    (small-angle)."""
    u = np.interp(xc, _UP[:, 0], _UP[:, 1]) * chord
    l = np.interp(xc, _LO[:, 0], _LO[:, 1]) * chord
    if y is None:
        return u, l
    open_ = 0.003 / 2 * xc * chord
    rise = -(np.asarray(xc) - 0.25) * chord * np.sin(np.radians(s.GEO.twist(y)))
    return u + open_ + rise, l - open_ + rise


def segment_gap(p0, p1, q0, q1, n=200):
    """Closest distance between two line segments (dense sampling, mm-accurate here)."""
    t = np.linspace(0, 1, n)[:, None]
    a = p0 + t * (p1 - p0)
    b = q0 + t * (q1 - q0)
    return float(np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)))


def clearances(dx, dy):
    """Minimum gaps (mm) from the shifted cells to the compact sleeves (cylinders treated as capsules,
    which is conservative) and to the skin (airfoil surface minus the 1.5 mm skin allowance)."""
    G, j, yb = s.GEO, wt.COMPACT, s.Y_BREAK
    yc, xb = Y_BATT_CAD + dy, X_BATT + dx
    c0 = G.chord(yc)
    zu, zl = surfaces((xb - G.x_le(yc)) / c0, c0)
    zc = (zu + zl) / 2

    def axis_point(y):
        c = G.chord(y)
        u, l = surfaces(s.X_SPAR, c)
        return np.array([G.x_le(y) + s.X_SPAR * c, y, (u + l) / 2])

    a0, a1 = axis_point(yb - 0.30), axis_point(yb + 0.30)
    axis = (a1 - a0) / np.linalg.norm(a1 - a0)
    mid = (a0 + a1) / 2
    sleeves = [(mid, mid + d * axis * j.engagement) for d in (-1, 1)]
    sleeve_gap, skin_gap = np.inf, np.inf
    for i in range(-2, 3):
        for k in range(-1, 2):
            x, y = xb + i * CELL_DX, yc + k * CELL_DY
            p0, p1 = np.array([x, y - CELL_L / 2, zc]), np.array([x, y + CELL_L / 2, zc])
            for q0, q1 in sleeves:
                sleeve_gap = min(sleeve_gap, segment_gap(p0, p1, q0, q1) - CELL_R - j.sleeve / 2)
            for ys in (y - CELL_L / 2, y + CELL_L / 2):
                c = G.chord(ys)
                xs = x + CELL_R * np.cos(np.linspace(0, np.pi, 41))
                half = np.sqrt(np.maximum(CELL_R**2 - (xs - x) ** 2, 0))
                u, l = surfaces((xs - G.x_le(ys)) / c, c, ys)
                skin_gap = min(skin_gap, np.min(u - (zc + half)), np.min((zc - half) - l))
    return 1000 * sleeve_gap, 1000 * (skin_gap - SKIN)


def main():
    cad = pd.read_csv(s.ROOT / "out" / "cad_v3" / "mass_cg.csv")
    tab = pd.read_csv(wt.OUT / "compact_xps_sections.csv")
    joint_masses = tuple(wt.COMPACT.masses())
    out = []
    for dy in DY:
        for dx in DX:
            m = pd.DataFrame(margins(cad, tab, joint_masses, dx, dy))
            sg, kg = clearances(dx, dy)
            row = dict(dx_mm=1000 * dx, dy_mm=1000 * dy, min_sm_pct=m.sm_pct.min(), max_sm_pct=m.sm_pct.max(),
                       elevon_trim_deg_range=f"{m.elevon_trim_deg.min():.2f}..{m.elevon_trim_deg.max():.2f}",
                       sleeve_gap_mm=sg, skin_gap_after_allowance_mm=kg)
            out.append(row)
            print(", ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in row.items()), flush=True)
    df = pd.DataFrame(out)
    df.to_csv(wt.OUT / "battery_trim.csv", index=False)
    ok = df[(df.min_sm_pct >= TARGET) & (df.sleeve_gap_mm > 2) & (df.skin_gap_after_allowance_mm > 0)]
    print("\nPositions meeting 6% in all four states with >2 mm sleeve gap and skin clearance:")
    print(ok.to_string(index=False) if len(ok) else "none")


if __name__ == "__main__":
    main()
