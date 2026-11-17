"""Detection model check: per-pass probability of detection (Pd) from the terrain replay's camera and
canopy model, evaluated at the conditions of published thermal-drone trials (gap item G5).

Reads terrain_sim's detection functions and constants without changing them. No mission runs.

Model (terrain_sim.camera_probability, canopy_clear):
  - in-frame Johnson-type pass probability, odds = N^(1.51 + 0.24 N) x CAMERA_ODDS_SCALE,
    N = 0.5 m / (2 sqrt(gsd_across x gsd_along_blurred)) cycles on the person;
  - CAMERA_ODDS_SCALE is fitted so the 32 deg camera at 120 m with no blur has sweep width 0.8 x footprint (55 m);
  - canopy gap probability [1 - c (1 - porosity)]^(1 / cos view angle), one hiding draw per person.

Comparisons:
  - Doull et al. 2021 (Sensors 21, 4074): manual and automated thermal detection of standing or walking
    people, Zenmuse XT 336 (6.8 mm), 50 m hover, four canopy classes. Logit coefficients from their Tables 2 and 4.
  - Schedl et al. 2020 (Nature Machine Intelligence): single thermal frames under dense forest, 640 px
    50.8 deg camera at 30-35 m, best single-image AP 24.8% (precision 25.1%, recall 50.4%).

Run:  .venv/Scripts/python.exe -X utf8 scripts/detection_check.py
Writes: out/detection_check/pd_table.csv, canopy_fit.csv, calibration_cases.csv, detection_check_log.txt,
        pd_vs_canopy.png
"""
from pathlib import Path
import math
import os
import sys

if "LODESTAR_POROSITY" in os.environ:
    sys.exit("Unset LODESTAR_POROSITY: this check sets porosity explicitly.")

import numpy as np
import pandas as pd
from scipy.optimize import brentq, minimize_scalar
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import terrain_sim as t  # noqa: E402
import v4_upgrade as v4u  # noqa: E402
from stability import Tee  # noqa: E402

OUT = t.ROOT / "out" / "detection_check"
OUT.mkdir(parents=True, exist_ok=True)

SCALE0 = t.CAMERA_ODDS_SCALE
H = t.H_SEARCH
D = t.vb.D_CRIT


# ---------------------------------------------------------------- model, parameterised copies
def p_cam(x, vg, height, hfov, npx=640, tau=v4u.TAU, scale=SCALE0):
    """terrain_sim.camera_probability with pixel count, time constant and odds scale exposed."""
    tan = math.tan(math.radians(hfov / 2))
    half = height * tan
    if height <= 0 or x > half:
        return 0.0
    cosine = height / math.hypot(height, x)
    gsd = 2 * half / npx
    across = gsd / cosine**2
    along = math.hypot(gsd / cosine, vg * tau)
    n = D / (2 * math.sqrt(across * along))
    odds = n ** (1.51 + 0.24 * n) * scale if n < 100 else math.inf
    return 1 - 1 / (1 + odds)


def gap(cover, cosine, porosity):
    """terrain_sim.canopy_clear with porosity exposed."""
    return (1.0 - cover * (1.0 - porosity)) ** (1.0 / max(cosine, 0.05))


def frame_mean(vg, hfov, cover=0.0, porosity=0.2, height=H, npx=640, tau=v4u.TAU, scale=SCALE0, n=801):
    """Per-pass Pd averaged over a person placed uniformly across the swath (= sweep width / swath)."""
    x = np.linspace(0, height * math.tan(math.radians(hfov / 2)), n)
    p = [p_cam(xi, vg, height, hfov, npx, tau, scale) * gap(cover, height / math.hypot(height, xi), porosity)
         for xi in x]
    return float(np.mean(p))


def nadir(vg, hfov, cover=0.0, porosity=0.2, height=H, npx=640, tau=v4u.TAU, scale=SCALE0):
    return p_cam(0.0, vg, height, hfov, npx, tau, scale) * gap(cover, 1.0, porosity)


def n50_equivalent(vg, hfov, scale, height=H, npx=640):
    """Johnson N50 that gives the same nadir Pd with the standard TTPF, P = r^E / (1 + r^E), r = N / N50."""
    tan = math.tan(math.radians(hfov / 2))
    gsd = 2 * height * tan / npx
    n = D / (2 * math.sqrt(gsd * math.hypot(gsd, vg * v4u.TAU)))
    target = p_cam(0.0, vg, height, hfov, npx, scale=scale)

    def f(n50):
        r = n / n50
        e = 1.51 + 0.24 * r
        return r**e / (1 + r**e) - target
    return brentq(f, 0.05, 20.0)


def check_against_terrain_sim():
    tan = t.camera_tangent(type("A", (), {"name": "Lodestar SiCore"})())
    for x in (0.0, 20.0, 50.0):
        for vg in (0.0, 18.0):
            a = t.camera_probability(x, vg, H, tan)
            b = p_cam(x, vg, H, t.CAMERAS["Lodestar"][0])
            assert abs(a - b) < 1e-12, (x, vg, a, b)
    assert abs(t.canopy_clear(0.6, 0.9) - gap(0.6, 0.9, t.CROWN_POROSITY)) < 1e-12
    assert t.CROWN_POROSITY == 0.2


# ---------------------------------------------------------------- published data
# Doull et al. 2021, manual TIR GLM (Table 2) and automated TIR GLM (Table 4), logit scale.
# Reference: open canopy (0-25%), 45 deg oblique, dawn. Classes: low 25-50, medium 50-75, high 75-100% cover.
DOULL = {
    "manual": dict(intercept=2.0822, nadir=0.383, dusk=-0.292, low=-0.837, medium=-1.576, high=-2.448),
    "automated": dict(intercept=1.564, nadir=0.234, dusk=0.0, low=0.0303, medium=-0.929, high=-1.514),
}
CLASS_COVER = {"open": 0.125, "low": 0.375, "medium": 0.625, "high": 0.875}   # class midpoints
# Zenmuse XT 336, 6.8 mm lens, 17 um pixels: horizontal FOV from the focal plane width
XT_HFOV = 2 * math.degrees(math.atan(336 * 17e-6 / 2 / 6.8e-3))


def doull_p(mode, cls, nadir_view=True, dusk=False):
    c = DOULL[mode]
    z = c["intercept"] + (c["nadir"] if nadir_view else 0.0) + (c["dusk"] if dusk else 0.0)
    z += c.get(cls, 0.0) if cls != "open" else 0.0
    return 1 / (1 + math.exp(-z))


def fit_porosity(ratios):
    """Porosity whose nadir gap-fraction ratios gap(c)/gap(c_open) best match the published Pd ratios."""
    covers = [CLASS_COVER[k] for k in ("low", "medium", "high")]

    def sse(p):
        g0 = gap(CLASS_COVER["open"], 1.0, p)
        return sum((gap(c, 1.0, p) / g0 - r) ** 2 for c, r in zip(covers, ratios))
    return minimize_scalar(sse, bounds=(0.0, 1.0), method="bounded").x


def fit_scale(p_target, vg, hfov, height, npx):
    return brentq(lambda s: p_cam(0.0, vg, height, hfov, npx, scale=s) - p_target, 1e-3, 1e3)


# ---------------------------------------------------------------- main
if __name__ == "__main__":
    sys.stdout = Tee(OUT / "detection_check_log.txt")
    print("Detection model check, 2026-11-15\n")
    check_against_terrain_sim()
    L_HFOV, O_HFOV = t.CAMERAS["Lodestar"][0], t.CAMERAS["other"][0]
    print(f"terrain_sim reproduced: CAMERA_ODDS_SCALE = {SCALE0:.4f}, CROWN_POROSITY = {t.CROWN_POROSITY}, "
          f"tau = {v4u.TAU*1e3:.0f} ms, d = {D} m, Lodestar {L_HFOV:.0f} deg, others {O_HFOV:.0f} deg, 640 px, {H:.0f} m")
    for hf in (L_HFOV, O_HFOV):
        print(f"  {hf:.0f} deg camera: centre GSD {2*H*math.tan(math.radians(hf/2))/640*100:.1f} cm, "
              f"swath {2*H*math.tan(math.radians(hf/2)):.1f} m")

    # 1. Model Pd table at the replay's conditions
    rows = []
    for cam, hf in (("Lodestar 50 deg", L_HFOV), ("competitor 32 deg", O_HFOV)):
        for vg in (0.0, 16.5, 18.0, 20.0):
            for cover in (0.0, 0.125, 0.375, 0.625, 0.875, 1.0):
                for por in (0.10, 0.20, 0.35, 0.50):
                    rows.append(dict(camera=cam, ground_speed=vg, cover=cover, porosity=por,
                                     pd_frame_mean=frame_mean(vg, hf, cover, por),
                                     pd_nadir=nadir(vg, hf, cover, por)))
    pd_tab = pd.DataFrame(rows)
    pd_tab.round(4).to_csv(OUT / "pd_table.csv", index=False)
    sel = pd_tab[(pd_tab.porosity == 0.20) & pd_tab.ground_speed.isin([18.0, 20.0])]
    print("\nModel per-pass Pd, porosity 0.20, 120 m, tau 10 ms (frame mean / nadir):")
    print(sel.pivot_table(index=["camera", "cover"], columns="ground_speed",
                          values=["pd_frame_mean", "pd_nadir"]).round(3).to_string())

    print("\nBlur sensitivity, Lodestar 50 deg, open ground, frame-mean Pd:")
    for tau in (0.0, 0.007, 0.010, 0.012):
        x = np.linspace(0, H * math.tan(math.radians(L_HFOV / 2)), 801)
        vals = [np.mean([p_cam(xi, vg, H, L_HFOV, tau=tau) for xi in x]) for vg in (18.0, 20.0)]
        print(f"  tau {tau*1e3:4.0f} ms: 18 m/s {vals[0]:.3f}, 20 m/s {vals[1]:.3f}")

    # 2. Same-condition comparison with Doull et al. 2021
    print(f"\nDoull et al. 2021 geometry: Zenmuse XT 336, HFOV {XT_HFOV:.1f} deg, 50 m hover, "
          f"GSD {2*50*math.tan(math.radians(XT_HFOV/2))/336*100:.1f} cm")
    fit_rows = []
    for mode in ("manual", "automated"):
        pub = {k: doull_p(mode, k) for k in CLASS_COVER}
        ratios = [pub[k] / pub["open"] for k in ("low", "medium", "high")]
        p_fit = fit_porosity(ratios)
        for k, c in CLASS_COVER.items():
            fit_rows.append(dict(
                source=f"Doull 2021 {mode}", canopy_class=k, cover_mid=c, pd_published_nadir_dawn=pub[k],
                pd_published_oblique_dusk=doull_p(mode, k, nadir_view=False, dusk=True),
                ratio_published=pub[k] / pub["open"],
                model_pd_p020=nadir(0.0, XT_HFOV, c, 0.20, 50.0, 336),
                model_ratio_p010=gap(c, 1, 0.10) / gap(0.125, 1, 0.10),
                model_ratio_p020=gap(c, 1, 0.20) / gap(0.125, 1, 0.20),
                model_ratio_p035=gap(c, 1, 0.35) / gap(0.125, 1, 0.35),
                model_ratio_p050=gap(c, 1, 0.50) / gap(0.125, 1, 0.50),
                porosity_fit=p_fit))
    fit = pd.DataFrame(fit_rows)
    fit.round(4).to_csv(OUT / "canopy_fit.csv", index=False)
    with pd.option_context("display.width", 220):
        print(fit.round(3).to_string(index=False))
    p_cam_doull = p_cam(0.0, 0.0, 50.0, XT_HFOV, 336)
    print(f"Model camera-only Pd at Doull geometry (nadir, no canopy): {p_cam_doull:.3f}")

    # 3. Same-condition comparison with Schedl et al. 2020 single frames (dense forest)
    print("\nSchedl et al. 2020 single frames: 640 px (cropped 512), 50.8 deg, 35 m, slow flight (4 m/s assumed)")
    p_aos = p_cam(0.0, 4.0, 35.0, 50.82, 512)
    print(f"  model camera-only Pd {p_aos:.3f}")
    for c in (0.80, 0.90, 0.95):
        print("  cover {:.2f}: ".format(c) + ", ".join(
            f"p={p:.2f} -> {p_aos*gap(c, 1, p):.2f}" for p in (0.10, 0.20, 0.35, 0.50)))
    print(f"  porosity giving recall 0.504 at cover 0.90: {1 - (1 - 0.504/p_aos)/0.90:.2f}; "
          f"giving 0.248 (AP) at cover 0.90: {1 - (1 - 0.248/p_aos)/0.90:.2f}")

    # 4. Calibration cases for the odds scale
    targets = {"manual nadir dawn": doull_p("manual", "open"), "automated nadir": doull_p("automated", "open"),
               "manual oblique dusk": doull_p("manual", "open", nadir_view=False, dusk=True)}
    print("\nOdds scale that reproduces Doull open-canopy Pd with the camera term alone (open class 0-25% cover "
          "treated as clear, which makes the fitted scale slightly low):")
    for k, v in targets.items():
        print(f"  {k}: published {v:.3f} -> scale {fit_scale(v, 0.0, XT_HFOV, 50.0, 336):.2f}")
    cases = []
    for label, scale, por in (("low (current baseline)", SCALE0, 0.20), ("nominal", 1.5, 0.35),
                              ("high", 3.0, 0.50), ("stress (below evidence)", SCALE0, 0.10)):
        def w32(scale=scale):
            x = np.linspace(-H * math.tan(math.radians(8)), H * math.tan(math.radians(8)), 801)
            return float(np.mean([p_cam(abs(xi), 0.0, H, 32.0, scale=scale) for xi in x]))
        row = dict(case=label, camera_odds_scale=scale, crown_porosity=por,
                   sweep_factor_32deg_noblur=w32(),
                   n50_equiv_lodestar_18=n50_equivalent(18.0, L_HFOV, scale),
                   pd_doull_geometry=p_cam(0.0, 0.0, 50.0, XT_HFOV, 336, scale=scale))
        for cam, hf in (("lodestar", L_HFOV), ("other", O_HFOV)):
            v = 18.0 if cam == "lodestar" else 16.5
            for cname, c in (("open", 0.0), ("light", 0.375), ("heavy", 0.875)):
                row[f"pd_{cam}_{cname}"] = frame_mean(v, hf, c, por, scale=scale)
        row["sweep_lodestar_18_m"] = row["pd_lodestar_open"] * 2 * H * math.tan(math.radians(L_HFOV / 2))
        row["ratio_lodestar_to_other_open"] = row["pd_lodestar_open"] / row["pd_other_open"]
        cases.append(row)
    cal = pd.DataFrame(cases)
    cal.round(4).to_csv(OUT / "calibration_cases.csv", index=False)
    with pd.option_context("display.width", 260, "display.max_columns", 30):
        print("\nCalibration cases (Lodestar 18 m/s, competitor 16.5 m/s, frame-mean Pd):")
        print(cal.round(3).T.to_string())

    # 5. Figure: Pd vs canopy cover
    plt.rcParams.update({"font.family": "serif", "font.size": 9, "axes.linewidth": 0.5,
                         "xtick.direction": "in", "ytick.direction": "in", "xtick.top": True, "ytick.right": True})
    fig, ax = plt.subplots(figsize=(4.6, 3.2))
    cc = np.linspace(0, 1, 41)
    greys = {0.10: "#bbbbbb", 0.20: "#111111", 0.35: "#666666", 0.50: "#999999"}
    base = p_cam(0.0, 0.0, 50.0, XT_HFOV, 336)
    for por, col in greys.items():
        ax.plot(cc, [base * gap(c, 1, por) for c in cc], color=col, lw=1.2 if por == 0.20 else 0.8)
        ax.text(1.01, base * gap(1.0, 1, por), f"p = {por:.2f}", fontsize=7, va="center", color=col)
    for mode, mk in (("manual", "o"), ("automated", "s")):
        ax.plot(list(CLASS_COVER.values()), [doull_p(mode, k) for k in CLASS_COVER], mk, ms=4, mfc="none",
                color="#1f4e79", label=f"Doull et al. 2021, {mode}")
    ax.set_xlabel("Canopy cover at the person, c [-]")
    ax.set_ylabel("Single-look Pd [-]")
    ax.set_xlim(0, 1.15)
    ax.set_ylim(0, 1)
    ax.legend(frameon=False, fontsize=7, loc="lower left")
    ax.set_title("Single-look Pd vs. canopy cover c, by crown porosity p", fontsize=9, loc="left")
    fig.text(0.01, 0.01, "Model at Doull geometry: 336 px, 45.6 deg, 50 m, hover, nadir. Field: nadir, dawn, class midpoints.",
             fontsize=6)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(OUT / "pd_vs_canopy.png", dpi=200)
    print(f"\nWrote {OUT.relative_to(t.ROOT)}")
