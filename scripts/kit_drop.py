"""Milestone 1b: rescue-kit drop study.

Questions:
  1. How big and heavy is a parachute that lands a 1.5 kg kit at <= 6 m/s?
  2. How far does the kit drift in wind, and how close to the person can it land?

Key idea (computed air release point, CARP): in a steady wind the kit's motion
*through the air* does not depend on the wind. So
    landing point = release point + no-wind throw + wind x fall time.
The drone estimates the wind and releases upwind by that amount. Its aiming error is
    miss = (true wind - estimated wind) x fall time,
so a short fall (low release) means a small miss.

Run:  uv run python scripts/kit_drop.py
Writes: out/kit_drop/*.csv, *.png, run_log.txt
Parachute data are the same public sources as the shelved pod study
(../flying-wing-interceptor/research/2026-11-11-pod-trade-study.md).
"""
from dataclasses import dataclass
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "kit_drop"
OUT.mkdir(parents=True, exist_ok=True)
G = 9.81


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


sys.stdout = Tee(OUT / "run_log.txt")


def rho(h_m):
    T = 288.15 - 0.0065 * h_m
    return 1.225 * (T / 288.15) ** 4.2559


GROUND_ALT = 2000.0       # m, same mountain design case as sizing.py (ASSUMPTION)
KIT_KG = 1.5              # requirement
RELEASE_V = 15.0          # m/s, the sizing's search speed
KIT_CDA = 0.02            # m^2, the kit bag itself (ASSUMPTION)
CX = 1.7                  # opening force coefficient, Knacke flat circular (DATA via Nakka App. K)
FILL_CONST = 8.0          # fill distance in canopy diameters (ASSUMPTION)
RELEASE_HW_KG = 0.08      # servo release + container (ASSUMPTION)


@dataclass
class Canopy:
    name: str
    cd: float
    kg_per_m2: float


# DATA: Iris Ultra Cd about 2.12, 0.095 kg/m^2 (pod study calibration).
# Cruciform (cross) canopies: Cd about 0.6-0.85 in Knacke; low swing, cheap to sew. 0.75 used.
# Their mass per area is an ASSUMPTION for 1.1 oz ripstop nylon + lines (about 0.07 kg/m^2).
CANOPIES = [Canopy("Annular (Iris-class)", 2.12, 0.095), Canopy("Cruciform (sewn ripstop)", 0.75, 0.07)]


def size_chute(v_land, c: Canopy):
    m_chute = 0.1
    for _ in range(50):
        m = KIT_KG + m_chute
        cds = 2 * m * G / (rho(GROUND_ALT) * v_land**2)
        S = cds / c.cd
        new = c.kg_per_m2 * S + RELEASE_HW_KG
        if abs(new - m_chute) < 1e-6:
            break
        m_chute = new
    D = math.sqrt(4 * S / math.pi)
    q0 = 0.5 * rho(GROUND_ALT) * RELEASE_V**2
    f_open = CX * q0 * cds
    return dict(canopy=c.name, v_land=v_land, cds_m2=cds, diameter_m=D, chute_kg=m_chute,
                total_kg=KIT_KG + m_chute, open_N=f_open, open_g=f_open / ((KIT_KG + m_chute) * G))


def fall(h_release, cds, mass, D, dt=0.002):
    """Fall through still air from level flight at RELEASE_V. Canopy fills linearly."""
    x, z, vx, vz, t = 0.0, h_release, RELEASE_V, 0.0, 0.0
    t_fill = None
    while z > 0:
        V = math.hypot(vx, vz)
        if t_fill is None:
            t_fill = FILL_CONST * D / max(V, 1.0)
        frac = min(1.0, t / t_fill)
        drag = 0.5 * rho(GROUND_ALT + z) * V**2 * (KIT_CDA + frac * cds)
        vx += -drag * vx / (mass * V) * dt
        vz += (-drag * vz / (mass * V) - G) * dt
        x += vx * dt
        z += vz * dt
        t += dt
    return dict(t_fall=t, throw_m=x, v_impact=math.hypot(vx, vz))


if __name__ == "__main__":
    print("Rescue Wing kit drop study, 2026-11-11, claude-opus-5-5")
    print(f"Kit {KIT_KG} kg, released at {RELEASE_V} m/s, ground at {GROUND_ALT:.0f} m "
          f"(air density {rho(GROUND_ALT):.3f})\n")

    rows = [size_chute(v, c) for c in CANOPIES for v in np.arange(3.0, 8.01, 0.5)]
    sz = pd.DataFrame(rows)
    sz.round(3).to_csv(OUT / "chute_sizing.csv", index=False)
    print("Parachute for a 5 m/s landing (about like jumping off a 1.3 m wall):")
    pick_rows = sz[np.isclose(sz.v_land, 5.0)]
    print(pick_rows.round(2).to_string(index=False))
    pick = pick_rows[pick_rows.canopy.str.startswith("Cruciform")].iloc[0]
    print(f"\nPick: cruciform, {pick.diameter_m:.2f} m across (flat), {pick.chute_kg * 1000:.0f} g including release. "
          "Cheap, sewable, swings less than a round canopy. The annular is lighter but costs more.")

    # Fall time and throw for a range of release heights
    hs = np.arange(30, 151, 10)
    fr = pd.DataFrame([{**fall(h, pick.cds_m2, pick.total_kg, pick.diameter_m), "h_release": h} for h in hs])
    # Miss distance from wind-estimate error (2D normal, sigma per axis):
    # 50% of drops land within 1.177*sigma*t, 95% within 2.448*sigma*t (Rayleigh distribution).
    for sig in (0.5, 1.0, 2.0):
        fr[f"R50_m_sigma{sig}"] = 1.1774 * sig * fr.t_fall
        fr[f"R95_m_sigma{sig}"] = 2.4477 * sig * fr.t_fall
    for w in (5, 10):
        fr[f"drift_m_wind{w}"] = w * fr.t_fall
    fr.round(2).to_csv(OUT / "release_height.csv", index=False)
    print("\nRelease height vs fall time, drift and accuracy (wind-estimate error 1 m/s per axis):")
    print(fr[["h_release", "t_fall", "throw_m", "v_impact", "drift_m_wind5", "drift_m_wind10",
              "R50_m_sigma1.0", "R95_m_sigma1.0"]].round(1).to_string(index=False))

    # Monte Carlo check of the formula at 60 m, sigma 1 m/s, 8 m/s wind with 1.5 m/s gust variation
    rng = np.random.default_rng(1)
    f60 = fall(60, pick.cds_m2, pick.total_kg, pick.diameter_m)
    n = 20000
    w_true = np.column_stack([8 + rng.normal(0, 1.0, n), rng.normal(0, 1.0, n)])
    w_est = np.array([8.0, 0.0])
    miss = np.linalg.norm((w_true - w_est) * f60["t_fall"], axis=1)
    print(f"\nMonte Carlo check, 60 m release, 8 m/s wind, 1 m/s estimate error: "
          f"R50 {np.percentile(miss, 50):.1f} m, R95 {np.percentile(miss, 95):.1f} m "
          f"(formula {1.1774 * f60['t_fall']:.1f} / {2.4477 * f60['t_fall']:.1f} m)")

    # What release height meets 'within 25 m' 95% of the time?
    for sig in (0.5, 1.0, 2.0):
        okh = fr[fr[f"R95_m_sigma{sig}"] <= 25]
        h = okh.h_release.max() if len(okh) else None
        if h is not None:
            print(f"  wind error {sig} m/s: 95% within 25 m if released at or below {h} m")
        else:
            print(f"  wind error {sig} m/s: 95% within 25 m is not reachable from 30 m or higher")

    # ---------------------------------------------------------- charts
    BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})

    fig, ax = plt.subplots(figsize=(7, 4))
    for sig, col, mk in ((0.5, BLUE, "o"), (1.0, ORANGE, "s"), (2.0, AQUA, "^")):
        ax.plot(fr.h_release, fr[f"R95_m_sigma{sig}"], color=col, lw=2, marker=mk, ms=6,
                label=f"wind error {sig} m/s")
    ax.axhline(25, color=MUTED, lw=1, ls="--")
    ax.text(30, 27, "25 m goal", color=MUTED)
    ax.set_xlabel("Release height above the person (m)")
    ax.set_ylabel("95% of kits land within (m)")
    ax.set_title("Lower release = better aim", color=INK, loc="left")
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(0.0, 0.92))
    ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "accuracy_vs_height.png", dpi=150)

    fig, ax = plt.subplots(figsize=(7, 4))
    for c, col, mk in zip(CANOPIES, (BLUE, ORANGE), ("o", "s")):
        d = sz[sz.canopy == c.name]
        ax.plot(d.v_land, d.chute_kg * 1000, color=col, lw=2, marker=mk, ms=6, label=c.name)
    ax.set_xlabel("Landing speed (m/s)")
    ax.set_ylabel("Parachute + release mass (g)")
    ax.set_title("Parachute mass for a 1.5 kg kit", color=INK, loc="left")
    ax.legend(frameon=False)
    ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "chute_mass_vs_landing_speed.png", dpi=150)
    print(f"\nCharts and CSVs written to {OUT.relative_to(ROOT)}")
