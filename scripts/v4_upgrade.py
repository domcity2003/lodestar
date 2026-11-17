"""Design v4: a wide-angle search camera and the lighter 7 m/s parachute.

Changes tested against v3 (SiCore foam-core @ 20 m/s, 45% help within 3 h with 10 ms blur):
  1. Search camera. The SIYI ZT6 thermal is 640 px behind a 13 mm lens (12 um pixels, 32.9 deg).
     A FLIR Boson 640 core with the 8.7 mm lens sees 50 deg (OEM listing: $4,104, 35-71 g with lens),
     fixed looking straight down; the ZT6 stays for confirming finds and aiming the drop. Both fit the
     0.60 kg camera budget already in the sizing (ZT6: 197 g). 1280 px gimbals were rejected: ~1 kg and
     $10-14k (Viewpro 1280 series, Sky Eye-HT50).
     Detection: the same Johnson-criteria + blur model as v3_blur.py, now with the field of view as a
     variable, scaled to the same 55 m calibration at 32 deg. Track spacing grows with the no-blur
     sweep width of the new camera (coverage factor 1, as before).
  2. Kit parachute for 7 m/s landing (drop-accuracy round): 144 g instead of 204 g.

Robustness: detection threshold N50 = 1.0 (base) and 1.5 (stricter: bigger pixels hurt more).
Search speed re-swept, because blur matters less when pixels are bigger.

Run:  uv run python scripts/v4_upgrade.py
Writes: out/v4/upgrade.csv, upgrade_log.txt, camera.png
"""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import v3_blur as vb  # noqa: E402
import design_v2 as d2  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "v4"
OUT.mkdir(parents=True, exist_ok=True)

TAU = 0.010
CHUTE_7MS = 0.144
HFOV_V4 = 50.0
N_RUNS = 2000


def w_cam(vg, hfov, n50=1.0, h=ms.H_AGL, npx=640, tau=TAU, d=vb.D_CRIT):
    half = h * math.tan(math.radians(hfov / 2))
    x = np.linspace(-half, half, 401)
    th = np.arctan(x / h)
    gsd0 = 2 * half / npx
    gc = gsd0 / np.cos(th) ** 2
    ga = np.hypot(gsd0 / np.cos(th), vg * tau)
    r = d / (2 * np.sqrt(gc * ga)) / n50
    E = 1.51 + 0.24 * r
    P = r**E / (1 + r**E)
    return float(np.trapezoid(P, x))


def camera(hfov, n50=1.0):
    """(W(ground speed) function, track spacing) for a camera, on the v3 calibration (55 m at 32 deg, no blur)."""
    ref = w_cam(0.0, 32.0, n50)
    return (lambda gs: vb.W0 * w_cam(gs, hfov, n50) / ref), vb.W0 * w_cam(0.0, hfov, n50) / ref


def apply_v4(hfov=HFOV_V4, n50=1.0, chute=CHUTE_7MS):
    """Set the module-level design inputs for a v4 run (call inside each worker)."""
    import sizing as sz
    sz.A = replace(sz.A, m_kit_chute=chute)
    d2.M_KIT_TOTAL = sz.A.m_kit + chute
    ms.W_OF_GS, ms.TRACK = camera(hfov, n50)


V11_CDA_APPLIED = False


def apply_v11():
    """v4 inputs plus the v1.1 body drag increment (EPP skid, strobes, second GNSS antenna). Idempotent, so it is
    safe to call in every worker and from every model (wing_trade, propulsion_margins, metrics, mission sim)."""
    global V11_CDA_APPLIED
    apply_v4()
    if not V11_CDA_APPLIED:
        import lodestar_v1 as v1
        d2.CDA_BODY += v1.CDA_INCREMENT_V11_M2
        V11_CDA_APPLIED = True


def fleet(speeds_si=(20.0, 21.0, 22.0), speeds_p50=(18.0, 19.0, 20.0)):
    out = []
    for lab, cell, vs in (("SiCore", "Amprius SiCore 6.3 Ah", speeds_si), ("P50B", "Molicel P50B", speeds_p50)):
        for v in vs:
            ac = vb.v3m.lodestar_v3(f"Lodestar {lab} @ {v:.0f}", cell, "kit", "halfway", 3.0, 7.0)
            ac.v_cruise, ac.search_h = v, ac.energy / float(ac.power(v))
            out.append(ac)
    return out


def job(args):
    hfov, n50, chute = args
    apply_v4(hfov, n50, chute)
    fl = fleet()
    rng = np.random.default_rng(ms.RNG_SEED)
    frames = [ms.run_scenario(fl, sc, box, N_RUNS, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
    df = ms.add_help_time(pd.concat(frames, ignore_index=True))
    import drop_accuracy as da
    tab = pd.read_csv(ROOT / "out" / "drop" / "drop_sweep.csv")
    ph = da.p_hit_fn(tab, "7 m/s chute")
    rows = []
    for (sc, ac), g in df.groupby(["scenario", "aircraft"], sort=False):
        rows.append(dict(hfov=hfov, n50=n50, chute_kg=chute, track_m=ms.TRACK, scenario=sc[:1], aircraft=ac,
                         found=g.found.mean(), help_3h_perfect=(g.t_help <= 3 * 3600).mean(),
                         help_3h=da.rescore(g, ph, ms.HIKE, 3 * 3600).mean(),
                         mtow=next(a.mass for a in fl if a.name == ac)))
    return pd.DataFrame(rows)


def plot(res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "axes.edgecolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.6))
    ax = axs[0]
    fovs = np.arange(16, 96, 2)
    for n50, c, ls in ((1.0, "#2a78d6", "-"), (1.5, "#0d3f8a", "--")):
        ref = w_cam(0.0, 32.0, n50)
        ax.plot(fovs, [vb.W0 * w_cam(20.0, f, n50) / ref for f in fovs], color=c, lw=2, ls=ls,
                label=f"detection threshold N50 = {n50}")
    ax.axvline(32, color=MUTED, lw=1, ls=":")
    ax.text(33, ax.get_ylim()[0] + 2, "ZT6 (32 deg)", color=MUTED, fontsize=9)
    ax.axvline(50, color=MUTED, lw=1, ls=":")
    ax.text(51, ax.get_ylim()[0] + 2, "Boson 640, 8.7 mm (50 deg)", color=MUTED, fontsize=9)
    ax.set_xlabel("Thermal camera field of view (deg), 640 px, 120 m above ground")
    ax.set_ylabel("Sweep width at 20 m/s (m)")
    ax.set_title("A wider lens sees more ground; past ~60 deg the pixels get too coarse", loc="left",
                 fontsize=10, color=INK)
    ax.grid(axis="y", color=GRID)
    ax.legend(frameon=False, fontsize=9)
    ax = axs[1]
    a = res[(res.scenario == "A") & (res.aircraft.str.startswith("Lodestar SiCore")) & (res.chute_kg == CHUTE_7MS)]
    for n50, c, ls in ((1.0, "#2a78d6", "-"), (1.5, "#0d3f8a", "--")):
        b = a[a.n50 == n50].groupby("hfov").help_3h.max() * 100
        ax.plot(b.index, b.values, color=c, lw=2, ls=ls, marker="o", ms=6, label=f"N50 = {n50}")
        for x, y in b.items():
            ax.text(x, y + 0.8, f"{y:.0f}%", ha="center", fontsize=8.5, color=INK)
    ax.set_xlabel("Search camera field of view (deg)")
    ax.set_ylabel("Help within 3 h (%), lost-hiker scenario")
    ax.set_title("SiCore build, best speed, realistic drops", loc="left", fontsize=10, color=INK)
    ax.grid(axis="y", color=GRID)
    ax.legend(frameon=False, fontsize=9)
    ax.set_ylim(0, 70)
    fig.tight_layout()
    fig.savefig(OUT / "camera.png", dpi=150)


if __name__ == "__main__":
    if "--plot" in sys.argv:
        plot(pd.read_csv(OUT / "upgrade.csv"))
        sys.exit()
    sys.stdout = Tee(OUT / "upgrade_log.txt")
    print("Lodestar v4 upgrade: wide search camera + 7 m/s chute, 2026-11-12\n")
    for n50 in (1.0, 1.5):
        print(f"N50 {n50}: sweep width at 20 m/s / track spacing:",
              {f: (round(camera(f, n50)[0](20.0), 1), round(camera(f, n50)[1], 1)) for f in (32, 40, 50, 60)})
    cases = [(f, n, CHUTE_7MS) for n in (1.0, 1.5) for f in (32.0, 40.0, 50.0, 60.0)] + [(32.0, 1.0, 0.204)]
    with ProcessPoolExecutor() as ex:
        res = pd.concat(list(ex.map(job, cases)), ignore_index=True)
    res.round(4).to_csv(OUT / "upgrade.csv", index=False)
    with pd.option_context("display.width", 220):
        for sc in ("A", "B"):
            t = res[res.scenario == sc].pivot_table(index="aircraft", columns=["n50", "hfov", "chute_kg"],
                                                    values="help_3h", sort=False) * 100
            print(f"\nHelp within 3 h (%), realistic drops, scenario {sc} (columns: N50, field of view, chute kg):")
            print(t.round(1).to_string())
    print("\nTake-off mass (kg):", res.groupby(["aircraft", "chute_kg"]).mtow.first().round(3).to_dict())
    plot(res)
