"""Drop-accuracy round: how often does the kit land close enough for the person to reach it?

Until now the mission sim assumed every dropped kit lands beside the person. Milestone 1b
(kit_drop.py) only looked at one error source, the wind estimate. This study builds the full error
budget for a 40 m drop and feeds the misses back into the headline "help within 3 h" score.

Error sources (each drop draws all of them):
  - target position from the thermal camera: 3 m per axis (fixed-wing thermal tests report
    1.6-4.2 m mean error at 50-120 m AGL, ISPRS 2015 / Remote Sensing 2025)
  - height above the person: from a terrain map (DEM, ~8 m error in mountains) or a laser
    rangefinder (~0.5 m)
  - mean-wind estimate (GPS + airspeed wind triangle): 0.4 m/s per axis
  - gusts during the fall: Ornstein-Uhlenbeck process, sigma_u = 2.5 u*, sigma_v = 1.9 u*,
    sigma_w = 1.25 u* (Panofsky/Beljaars neutral surface layer), length scales 100 / 50 / 20 m
  - wind shear below the release: log law with a true roughness z0 drawn 0.03-0.5 m; the drone
    assumes 0.1 m
  - release timing: 50 ms jitter x ground speed
  - canopy: fill distance 6-10 diameters; random swing/glide 0.3 m/s (2 s time scale)
The drone aims (computed air release point, CARP) with its own *estimates* and a no-gust model;
the miss is where the kit really lands minus where the person really is.

Two wind estimators:
  "mean": a 30 s average, so the gust at release is unknown;
  "live": a 2 s window, so the drone knows most of the current gust (noisier: 0.7 m/s).

Run:  uv run python scripts/drop_accuracy.py          (add --plot to redraw only)
Writes: out/drop/*.csv, drop_log.txt, drop.png
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "drop"
OUT.mkdir(parents=True, exist_ok=True)

G, KAPPA = 9.81, 0.4
GROUND_ALT = 2000.0
M_KIT = 1.704            # kit + cruciform chute + release (design_v2.M_KIT_TOTAL)
D_CHUTE = 1.50           # m, cruciform for 5 m/s (kit_drop.py)
CDA_BAG = 0.02
VA_RELEASE = 14.0        # m/s airspeed on the into-wind run-in
Z0_ASSUMED = 0.1
AIM_OFFSET = 5.0         # m: aim beside the person, not on them (1.7 kg at 5 m/s is ~20 J)
SIG_TARGET = 3.0
SIG_WIND_MEAN, SIG_WIND_LIVE = 0.4, 0.7
SIG_TIMING = 0.05
L_U, L_V, L_W = 100.0, 50.0, 20.0
GLIDE_SIG, GLIDE_T = 0.3, 2.0
DT = 0.01
HURT_SHARE, R_HURT, R_MOBILE = 0.3, 15.0, 50.0   # ASSUMPTION: 30% can only crawl ~15 m; others walk 50 m
H_FLIGHT = 120.0         # mission wind is drawn at search altitude


def rho(h):
    return 1.225 * ((288.15 - 0.0065 * h) / 288.15) ** 4.2559


def cds_for(v_land):
    return 2 * M_KIT * G / (rho(GROUND_ALT) * v_land**2)


def profile(z, z0):
    return np.log(np.maximum(z, 2 * z0) / z0)


def fall(n, h, wind_ref, z0, cds, fill_k, gust_sig, gust0, glide, rng):
    d_chute = D_CHUTE * math.sqrt(cds / cds_for(5.0))              # smaller canopy for a faster landing
    """Integrate n kits. wind_ref (n,2) is the mean wind at h; it scales with the log law below h.
    Gusts: OU per axis with sigma gust_sig (n,3) and start state gust0 (n,3). Returns landing (n,2)."""
    pos = np.zeros((n, 3))
    pos[:, 2] = h
    w0 = np.column_stack([wind_ref, np.zeros(n)])
    vel = w0 + np.array([-VA_RELEASE, 0.0, 0.0])           # flying upwind (-x) into a +x wind
    g = gust0.copy()
    gl = np.zeros((n, 2))
    t = 0.0
    land = np.full((n, 2), np.nan)
    alive = np.ones(n, bool)
    t_fill = fill_k * d_chute / VA_RELEASE
    ref = profile(h, z0)
    while alive.any():
        z = pos[:, 2]
        U = np.maximum(np.hypot(*wind_ref.T), 0.5)
        T = np.column_stack([L_U / U, L_V / U, L_W / U])
        if gust_sig is not None:
            g += -g / T * DT + gust_sig * np.sqrt(2 * DT / T) * rng.standard_normal((n, 3))
        if glide:
            gl += -gl / GLIDE_T * DT + GLIDE_SIG * math.sqrt(2 * DT / GLIDE_T) * rng.standard_normal((n, 2))
        air = w0 * (profile(z, z0) / ref)[:, None] + g
        air[:, :2] += gl * min(1.0, t / 2.0)
        vr = vel - air
        V = np.linalg.norm(vr, axis=1)
        frac = np.minimum(1.0, t / t_fill)
        k = 0.5 * rho(GROUND_ALT + z) * V * (CDA_BAG + frac * cds) / M_KIT
        vel += (-k[:, None] * vr + np.array([0, 0, -G])) * DT
        pos += vel * DT
        t += DT
        hit = alive & (pos[:, 2] <= 0)
        land[hit] = pos[hit, :2]
        alive &= ~hit
        if t > 120:
            break
    return land


def drop_cases(U_flight, cfg, rng):
    """Miss distance (m) for drops at mission winds U_flight (n,) under config cfg."""
    n = len(U_flight)
    h, cds = cfg["h"], cds_for(cfg["v_land"])
    z0 = np.exp(rng.uniform(np.log(0.03), np.log(0.5), n))
    U_h = U_flight * profile(h, z0) / profile(H_FLIGHT, z0)            # true mean wind at release height
    ustar = KAPPA * np.maximum(U_h, 0.5) / profile(h, z0)
    sig = np.column_stack([2.5 * ustar, 1.9 * ustar, 1.25 * ustar])
    sig[:, :2] = np.maximum(sig[:, :2], 0.3)
    gust0 = sig * rng.standard_normal((n, 3))
    wind_true = np.column_stack([U_h, np.zeros(n)])
    land_true = fall(n, h + cfg["sig_h"] * rng.standard_normal(n), wind_true, z0, cds,
                     rng.uniform(6, 10, n), sig, gust0, True, rng)
    if cfg["wind_est"] == "live":                                       # sees the current gust
        w_est = wind_true + gust0[:, :2] + SIG_WIND_LIVE * rng.standard_normal((n, 2))
    else:
        w_est = wind_true + SIG_WIND_MEAN * rng.standard_normal((n, 2))
    land_pred = fall(n, np.full(n, float(h)), w_est, np.full(n, Z0_ASSUMED), cds, np.full(n, 8.0),
                     None, np.zeros((n, 3)), False, rng)
    gs = VA_RELEASE - U_h
    err = land_true - land_pred
    err[:, 0] += SIG_TIMING * gs * rng.standard_normal(n)
    err += SIG_TARGET * rng.standard_normal((n, 2))
    err[:, 1] += AIM_OFFSET
    return np.hypot(*err.T)


def p_reach(miss):
    return HURT_SHARE * (miss <= R_HURT).mean() + (1 - HURT_SHARE) * (miss <= R_MOBILE).mean()


CONFIGS = {
    "as designed (40 m, map height, 30 s wind)": dict(h=40, v_land=5.0, sig_h=8.0, wind_est="mean"),
    "+ laser rangefinder": dict(h=40, v_land=5.0, sig_h=0.5, wind_est="mean"),
    "+ live (2 s) wind": dict(h=40, v_land=5.0, sig_h=8.0, wind_est="live"),
    "30 m release": dict(h=30, v_land=5.0, sig_h=8.0, wind_est="mean"),
    "7 m/s chute": dict(h=40, v_land=7.0, sig_h=8.0, wind_est="mean"),
    "7 m/s chute + rangefinder": dict(h=40, v_land=7.0, sig_h=0.5, wind_est="mean"),
}
WINDS = np.arange(0.0, 10.01, 1.0)
N_PER = 3000


def sweep():
    rng = np.random.default_rng(7)
    rows = []
    for name, cfg in CONFIGS.items():
        U = np.repeat(WINDS, N_PER)
        miss = drop_cases(U, cfg, rng)
        for w in WINDS:
            m = miss[U == w]
            rows.append(dict(config=name, wind=w, R50=np.percentile(m, 50), R95=np.percentile(m, 95),
                             p15=(m <= 15).mean(), p25=(m <= 25).mean(), p50=(m <= 50).mean(), p_reach=p_reach(m)))
    return pd.DataFrame(rows)


def p_hit_fn(tab, config):
    t = tab[tab.config == config]
    return lambda w: np.interp(w, t.wind.values, t.p_reach.values)


def rescore(df, phit, hike_speed, window_s):
    """Expected help-within-window when a kit only helps with probability phit(wind)."""
    hike = np.hypot(df.tx, df.ty) / hike_speed
    team = np.where(df.found, df.t_find + hike, np.inf)
    kit = df.t_kit.notna().values
    tk = df.t_kit.fillna(np.inf).values
    p = np.where(kit, phit(df.wind.values), 0.0)
    ok_kit, ok_team = tk <= window_s, team <= window_s
    return p * ok_kit + (1 - p) * ok_team * 1.0 + p * (~ok_kit) * ok_team


def headline_runs():
    """Scenario A/B runs for the two costed Lodestar builds with 10 ms blur (same draws as v3_blur_fair)."""
    import mission_sim as ms
    import v3_blur as vb
    fleet = []
    for lab, cell, v in (("SiCore foam-core", "Amprius SiCore 6.3 Ah", 20.0), ("P50B foam-core", "Molicel P50B", 18.0)):
        ac = vb.v3m.lodestar_v3(f"Lodestar {lab} @ {v:.0f}", cell, "kit", "halfway", 3.0, 7.0)
        ac.v_cruise, ac.search_h = v, ac.energy / float(ac.power(v))
        fleet.append(ac)
    ms.W_OF_GS = vb.w_of_gs(0.010)
    rng = np.random.default_rng(ms.RNG_SEED)
    frames = [ms.run_scenario(fleet, sc, box, vb.N, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
    ms.W_OF_GS = None
    return ms.add_help_time(pd.concat(frames, ignore_index=True)), ms


def plot(tab, head):
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "axes.edgecolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED})
    cols = ["#9a9890", "#86b6ef", "#eb6834", "#2a78d6", "#1baf7a", "#0d3f8a"]
    mk = ["o", "s", "^", "D", "v", "P"]
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
    ax = axs[0]
    for (name, t), c, m in zip(tab.groupby("config", sort=False), cols, mk):
        ax.plot(t.wind, t.R95, color=c, lw=2, marker=m, ms=6, label=name)
    ax.axhline(25, color=MUTED, lw=1, ls="--")
    ax.text(0.1, 27, "25 m goal", color=MUTED)
    ax.set_xlabel("Wind at search height (m/s)")
    ax.set_ylabel("95% of kits land within (m)")
    ax.set_title("A faster-falling chute halves the wind miss; a live wind reading does not help",
                 loc="left", fontsize=10, color=INK)
    ax.grid(axis="y", color=GRID)
    ax.legend(frameon=False, fontsize=8.5)
    ax = axs[1]
    h = head.copy()
    names = list(h.columns)
    x = np.arange(len(h.index))
    bw = 0.8 / len(names)
    for j, (nm, c) in enumerate(zip(names, ["#9a9890", "#eb6834", "#2a78d6"])):
        v = h[nm].values
        b = ax.bar(x + (j - (len(names) - 1) / 2) * bw, v, bw * 0.92, color=c, label=nm)
        for xi, vi in zip(x + (j - (len(names) - 1) / 2) * bw, v):
            ax.text(xi, vi + 0.6, f"{vi:.0f}%", ha="center", fontsize=8.5, color=INK)
    ax.set_xticks(x, h.index, fontsize=9)
    ax.set_ylabel("Help within 3 h (%)")
    ax.set_title("Realistic drops cost about 2 points; a 7 m/s chute wins most of it back",
                 loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=8.5, loc="upper right")
    ax.grid(axis="y", color=GRID)
    ax.set_ylim(0, max(52, h.values.max() + 6))
    fig.tight_layout()
    fig.savefig(OUT / "drop.png", dpi=150)


if __name__ == "__main__":
    if "--plot" in sys.argv:
        plot(pd.read_csv(OUT / "drop_sweep.csv"), pd.read_csv(OUT / "drop_headline.csv", index_col=0))
        sys.exit()
    sys.stdout = Tee(OUT / "drop_log.txt")
    print("Lodestar drop-accuracy round, 2026-11-12\n")
    print(f"Kit {M_KIT} kg, cruciform {D_CHUTE} m, run-in {VA_RELEASE} m/s airspeed, aim {AIM_OFFSET} m beside the person.")
    print(f"Reach: {HURT_SHARE:.0%} hurt (reach {R_HURT:.0f} m), rest mobile (reach {R_MOBILE:.0f} m). ASSUMPTION.\n")
    tab = sweep()
    tab.round(4).to_csv(OUT / "drop_sweep.csv", index=False)
    avg = tab.groupby("config", sort=False)[["R50", "R95", "p15", "p25", "p50", "p_reach"]].mean()
    print("Averaged over mission winds 0-10 m/s (uniform):")
    print(avg.round(3).to_string())
    print("\nR95 (m) by wind:")
    print(tab.pivot_table(index="wind", columns="config", values="R95", sort=False).round(0).to_string())

    df, ms = headline_runs()
    W = ms.HELP_WINDOW * 60
    res = {}
    for sc, g in df.groupby("scenario", sort=False):
        for ac, gg in g.groupby("aircraft", sort=False):
            r = {"perfect drops (old model)": (gg.t_help <= W).mean() * 100}
            for cname in ("as designed (40 m, map height, 30 s wind)", "7 m/s chute"):
                r[cname] = rescore(gg, p_hit_fn(tab, cname), ms.HIKE, W).mean() * 100
            res[(sc[:1], ac)] = r
    hr = pd.DataFrame(res).T
    print("\nHelp within 3 h (%), scenario A/B, 10 ms blur:")
    print(hr.round(1).to_string())

    # trailhead (search-pattern round, trails to 6 km, p_on 0.5)
    pr = pd.read_csv(ROOT / "out" / "search" / "patterns_runs.csv.gz")
    pr = pr[(pr.pattern == "hasty 6") & (pr.p_on == 0.5) & pr.aircraft.str.startswith("Lodestar")]
    trail = {}
    for ac, gg in pr.groupby("aircraft", sort=False):
        r = {"perfect drops (old model)": (gg.t_help <= W).mean() * 100}
        for cname in ("as designed (40 m, map height, 30 s wind)", "7 m/s chute"):
            r[cname] = rescore(gg, p_hit_fn(tab, cname), ms.HIKE, W).mean() * 100
            r[cname + " | 1 h"] = rescore(gg, p_hit_fn(tab, cname), ms.HIKE, 3600).mean() * 100
        r["perfect | 1 h"] = (gg.t_help <= 3600).mean() * 100
        trail[ac] = r
    tr = pd.DataFrame(trail).T
    print("\nTrailhead, trails to 6 km first (%):")
    print(tr.round(1).to_string())
    pd.concat([hr.assign(case="scenario"), tr.assign(case="trailhead")]).round(2).to_csv(OUT / "drop_rescore.csv")

    a = hr.xs("A", level=0)
    head = pd.DataFrame({
        "perfect drops (old model)": a["perfect drops (old model)"].values,
        "realistic, as designed": a["as designed (40 m, map height, 30 s wind)"].values,
        "realistic, 7 m/s chute": a["7 m/s chute"].values},
        index=[n.replace("Lodestar ", "").replace(" @ ", "\n@ ") + " m/s" for n in a.index])
    head.to_csv(OUT / "drop_headline.csv")
    plot(tab, head)
    print(f"\nWrote {OUT.relative_to(ROOT)}")
