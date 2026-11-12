"""Design v3: does motion blur change the best search speed?

v3_search_speed.py assumed the camera's sweep width does not depend on speed. A microbolometer
thermal camera does not take an instant snapshot: each pixel follows the scene with a thermal time
constant tau of 7-12 ms (FLIR support note; 8-12 ms in FLIR's R&D article). Over that time the
ground slides by (ground speed x tau), which smears a person along-track.

Detection model (Johnson criteria, target transfer probability function):
  - person seen from above, critical dimension d = 0.5 m (shoulders; crouched or partly covered), ASSUMPTION
  - camera: 640 px across a 32 deg field of view, 120 m above ground (same as milestone 4)
  - ground pixel at off-nadir angle t: cross-track gsd0 / cos^2 t, along-track gsd0 / cos t
  - blur adds in quadrature along-track: sqrt(gsd_along^2 + (Vg tau)^2)
  - cycles on target N = d / (2 sqrt(gsd_cross x gsd_along_blurred))
  - P(N) = r^E / (1 + r^E), r = N / N50, E = 1.51 + 0.24 r (Johnson / NVESD); N50 = 1.0 base, 0.75-1.5 tested
  - physical sweep width W(Vg) = integral of P across the swath. The milestone 4 calibration
    (W = 55 m with no blur) is kept and scaled by W(Vg) / W(0), so only the blur effect is new.
Blur uses the GROUND speed of each search leg, so downwind legs blur more than upwind legs.

Two operator policies:
  A "same tracks": track spacing stays at 55 m, so blur leaves gaps (lower detection per pass)
  B "tighter tracks": spacing shrinks to the blurred sweep width at the search airspeed

Run:  uv run python scripts/v3_blur.py
Writes: out/v3/blur_sweep.csv, blur_width.csv, blur.png, blur_log.txt
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import v3_mission as v3m  # noqa: E402
from stability import Tee  # noqa: E402

OUT = v3m.OUT
D_CRIT, HFOV, NPX = 0.5, 32.0, 640
W0, TRACK0 = ms.W_SWEEP, ms.TRACK


def w_phys(vg, tau, n50=1.0, d=D_CRIT, h=ms.H_AGL):
    half = h * math.tan(math.radians(HFOV / 2))
    x = np.linspace(-half, half, 401)
    th = np.arctan(x / h)
    gsd0 = 2 * half / NPX
    gc = gsd0 / np.cos(th) ** 2
    ga = np.hypot(gsd0 / np.cos(th), vg * tau)
    r = d / (2 * np.sqrt(gc * ga)) / n50
    E = 1.51 + 0.24 * r
    P = r**E / (1 + r**E)
    return float(np.trapezoid(P, x))


def w_of_gs(tau, n50=1.0):
    ref = w_phys(0.0, tau, n50)
    return lambda gs: W0 * w_phys(gs, tau, n50) / ref


DESIGNS = {
    "SiCore": ("Amprius SiCore 6.3 Ah", "kit", "laminar", 3.0, 7.0),
    "P50B": ("Molicel P50B", "kit", "laminar", 3.0, 7.0),
}
SPEEDS = (15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0)
N = 2000


def fleet_at(speeds):
    fleet, meta = [], {}
    for lab, args in DESIGNS.items():
        for v in speeds:
            ac = v3m.lodestar_v3(f"{lab} @ {v:.0f}", *args)
            ac.v_cruise = v
            ac.search_h = ac.energy / float(ac.power(v))
            fleet.append(ac)
            meta[ac.name] = (lab, v)
    return fleet, meta


def run(fleet, meta, **tags):
    rng = np.random.default_rng(ms.RNG_SEED)       # same targets, winds and draws every run
    frames = [ms.run_scenario(fleet, sc, box, N, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
    df = ms.add_help_time(pd.concat(frames, ignore_index=True))
    s = ms.summarise(df, fleet)
    s["design"] = s.aircraft.map(lambda n: meta[n][0])
    s["v_search"] = s.aircraft.map(lambda n: meta[n][1])
    for k, v in tags.items():
        s[k] = v
    return s


def plot(wd, avg):
    INK, GRID = "#1f1f1e", "#e4e3de"
    COLS = {"no blur (old model)": "#9a9890", "tau 7 ms, N50 1.0": "#86b6ef", "tau 10 ms, N50 1.0": "#2a78d6",
            "tau 12 ms, N50 1.0": "#0d3f8a", "tau 10 ms, N50 1.0, tighter tracks": "#1baf7a"}
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.6))
    ax = axs[0]
    for tau, col in ((7.0, "#86b6ef"), (10.0, "#2a78d6"), (12.0, "#0d3f8a")):
        s = wd[(wd.tau_ms == tau) & (wd.n50 == 1.0)]
        ax.plot(s.ground_speed, s.W_m, color=col, lw=2, label=f"{tau:.0f} ms time constant")
    ax.set_xlabel("Ground speed (m/s)")
    ax.set_ylabel("Effective sweep width (m)")
    ax.set_title("Blur narrows the strip the camera can search", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False)
    for ax, d in zip(axs[1:], DESIGNS):
        for c, col in COLS.items():
            s = avg[(avg.case == c) & (avg.design == d)].sort_values("v_search")
            ax.plot(s.v_search, s.p_help_3h * 100, color=col, lw=2, marker="o", ms=5, label=c)
        ax.set_xlabel("Search airspeed (m/s)")
        ax.set_ylabel("Help within 3 h, avg of 2 scenarios (%)")
        ax.set_title(f"{d} cells", loc="left", fontsize=10, color=INK)
        ax.set_ylim(20, 40)
    axs[1].legend(frameon=False, fontsize=8, loc="lower right")
    for ax in axs:
        ax.grid(color=GRID)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Blur costs 2-5 points of help within 3 h, but the best search speed doesn't move",
                 color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "blur.png", dpi=150)


if __name__ == "__main__" and "--plot" in sys.argv:
    r = pd.read_csv(OUT / "blur_sweep.csv")
    plot(pd.read_csv(OUT / "blur_width.csv"),
         r.groupby(["case", "policy", "design", "v_search"], sort=False).p_help_3h.mean().reset_index())
elif __name__ == "__main__":
    sys.stdout = Tee(OUT / "blur_log.txt")
    print("Lodestar v3: motion blur vs search speed, 2026-11-12, claude-opus-5-5\n")

    # 1. sweep width vs ground speed
    rows = []
    for tau in (0.007, 0.010, 0.012):
        for n50 in (0.75, 1.0, 1.5):
            f = w_of_gs(tau, n50)
            for vg in range(0, 31):
                rows.append(dict(tau_ms=tau * 1e3, n50=n50, ground_speed=vg, W_m=f(float(vg))))
    wd = pd.DataFrame(rows)
    wd.round(3).to_csv(OUT / "blur_width.csv", index=False)
    piv = wd[wd.ground_speed.isin([0, 10, 15, 18, 20, 22, 25])].pivot_table(
        index=["tau_ms", "n50"], columns="ground_speed", values="W_m")
    print("Effective sweep width (m) by ground speed (m/s); 55 m = no blur:")
    print(piv.round(1).to_string(), "\n")

    # 2. mission sweeps
    out = []
    ms.W_OF_GS = None
    fleet, meta = fleet_at(SPEEDS)
    out.append(run(fleet, meta, case="no blur (old model)", tau_ms=0.0, n50=1.0, policy="A same tracks"))
    for tau, n50 in ((0.007, 1.0), (0.010, 1.0), (0.012, 1.0), (0.010, 0.75), (0.010, 1.5)):
        ms.W_OF_GS = w_of_gs(tau, n50)
        out.append(run(fleet, meta, case=f"tau {tau*1e3:.0f} ms, N50 {n50}", tau_ms=tau * 1e3, n50=n50,
                       policy="A same tracks"))
        print(f"done: tau {tau*1e3:.0f} ms N50 {n50}", flush=True)
    # policy B at the base case: track spacing = blurred width at the search airspeed (no wind)
    ms.W_OF_GS = w_of_gs(0.010, 1.0)
    for v in SPEEDS:
        ms.TRACK = ms.W_OF_GS(v)
        f1, m1 = fleet_at((v,))
        out.append(run(f1, m1, case="tau 10 ms, N50 1.0, tighter tracks", tau_ms=10.0, n50=1.0,
                       policy="B tighter tracks"))
    ms.TRACK, ms.W_OF_GS = TRACK0, None
    res = pd.concat(out, ignore_index=True)
    res.round(4).to_csv(OUT / "blur_sweep.csv", index=False)

    # 3. summary: best speed per case
    res["p_help_mean"] = res.p_help_3h
    avg = res.groupby(["case", "policy", "design", "v_search"], sort=False).p_help_3h.mean().reset_index()
    print("\nHelp within 3 h, average of the two scenarios (%):")
    tab = avg.pivot_table(index=["design", "case"], columns="v_search", values="p_help_3h", sort=False) * 100
    with pd.option_context("display.width", 200):
        print(tab.round(1).to_string())
    print("\nBest speed per case:")
    for (d, c), g in avg.groupby(["design", "case"], sort=False):
        b = g.loc[g.p_help_3h.idxmax()]
        print(f"  {d:7s} {c:38s} best {b.v_search:.0f} m/s -> {b.p_help_3h:.1%}")

    plot(wd, avg)
