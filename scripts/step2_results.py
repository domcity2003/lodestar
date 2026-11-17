"""Step 2 headline tables for Lodestar v1.1 (P1, P2, P5 of research/2026-11-15-step2-plan.md) and the held-out
confirmation of the improvement trades.

Inputs (fleet_flyable.py runs, route pursuit_1, both patterns):
  held-out confirmation, seed 20261114: v1 and v1.1
  headline, seed 20261115 (no decision used it): v1.1 at three detection calibrations, plus 80% competitor endurance

P1  P(help within 1 h / 3 h) per aircraft; Lodestar SiCore minus each competitor, paired by hiker run, 95% bootstrap.
P2  time to first contact (kit on the ground, or ground team arrives after a find), among people contacted within
    3 h: median and P90; plus the share contacted within 1 h. Re-scored from runs.csv.gz with terrain_sim.score.
P5  (amended in the plan: Pd >= 0.5 under canopy is not reachable at the low calibration) area covered per sortie
    = straight search length of the planned route x no-blur sweep width of the camera, calm-wind route.

Run:    ../.venv/Scripts/python.exe -X utf8 step2_results.py      (from scripts/)
Writes: out/step2_results/ (p1_help.csv, p1_paired.csv, p2_contact.csv, p5_area.csv, confirm_v1_vs_v11.csv,
        run_log.txt)
"""
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

import terrain_sim as t
from wing_mission import N

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "out" / "wing_trade"
OUT = ROOT / "out" / "step2_results"
BASE = "flyable_fleet_pursuit_1_"
HEADLINE = {"low (0.735, 0.20)": "v1.1_seed_20261115",
            "nominal (1.5, 0.35)": "v1.1_seed_20261115_porosity_0.35_odds_1.5",
            "high (3.0, 0.50)": "v1.1_seed_20261115_porosity_0.5_odds_3",
            "low, competitors at 80% endurance": "v1.1_endurance_0.8_seed_20261115"}
CONFIRM = {"v1": "v1_seed_20261114", "v1.1": "v1.1_seed_20261114"}
LOD = "Lodestar SiCore"
MODE = "pursuit_1"


def load(tag):
    r = pd.read_csv(RUNS / (BASE + tag) / "runs.csv.gz")
    return r[r["mode"] == MODE]


def boot(d, rng, n=4000):
    s = d[rng.integers(0, len(d), (n, len(d)))].mean(axis=1)
    return np.quantile(s, [0.025, 0.975])


def p1(runs, label, rng):
    help_ = (runs.groupby(["pattern", "aircraft", "site"])[["help_1h", "help_3h"]].mean()
             .groupby(level=["pattern", "aircraft"]).mean() * 100).reset_index().assign(calibration=label)
    rows = []
    for pattern, g in runs.groupby("pattern"):
        for metric in ("help_1h", "help_3h"):
            s = g.pivot_table(index=["site", "run"], columns="aircraft", values=metric, aggfunc="first").astype(float)
            for ac in s.columns.drop(LOD):
                d = (s[LOD] - s[ac]).groupby("run").mean().to_numpy()
                lo, hi = boot(d, rng)
                rows.append(dict(calibration=label, pattern=pattern, metric=metric, competitor=ac,
                                 lodestar_minus_pp=100 * d.mean(), low_95_pp=100 * lo, high_95_pp=100 * hi))
    return help_, pd.DataFrame(rows)


def p2(runs, seed, label):
    """Re-score to get t_help (the saved runs keep t_find and t_kit; t_team comes from the cached hikers)."""
    chute = pd.read_csv(ROOT / "out" / "drop" / "drop_sweep.csv").query("config == '7 m/s chute'")
    rows = []
    for (site, ac, pattern), g in runs.groupby(["site", "aircraft", "pattern"]):
        hk = t.prep(site, N, seed)["hikers"]
        sc = t.score(g[["run", "found", "t_find", "t_kit"]].reset_index(drop=True), hk,
                     lambda w: np.interp(w, chute.wind, chute.p_reach))
        rows.append(sc[["run", "t_help"]].assign(site=site, aircraft=ac, pattern=pattern))
    th = pd.concat(rows)
    out = []
    for ac, g in th.groupby("aircraft"):
        ok = g.t_help[g.t_help <= 3 * 3600] / 60
        out.append(dict(calibration=label, aircraft=ac, contacted_3h_pct=100 * len(ok) / len(g),
                        contacted_1h_pct=100 * (g.t_help <= 3600).mean(),
                        median_min=ok.median(), p90_min=ok.quantile(0.9)))
    return pd.DataFrame(out)


def p5():
    def width(tan_half, pixel_tan):
        x = np.linspace(0.0, t.H_SEARCH * tan_half, 400)
        return 2 * float(np.trapezoid([t.camera_probability(xi, 0.0, t.H_SEARCH, tan_half, pixel_tan) for xi in x], x))
    meta = json.loads((RUNS / (BASE + HEADLINE["low (0.735, 0.20)"]) / "meta.json").read_text())
    hfov = meta["lodestar_camera"][0]
    import lodestar_v1 as v1
    w_lod = width(math.tan(math.radians(hfov / 2)), v1.CAMERA_V11["pixel_tan"])
    w_oth = width(math.tan(math.radians(t.CAMERAS["other"][0] / 2)), None)
    routes = pd.read_csv(RUNS / (BASE + HEADLINE["low (0.735, 0.20)"]) / "routes.csv")
    calm = routes[routes.wind_bound_mps == routes.groupby(["site", "pattern", "aircraft"]).wind_bound_mps
                  .transform("min")]
    km = calm.groupby("aircraft").straight_search_km.mean()
    out = pd.DataFrame(dict(aircraft=km.index, straight_search_km=km.values))
    out["sweep_width_m"] = np.where(out.aircraft.str.startswith("Lodestar"), w_lod, w_oth)
    out["area_km2_per_sortie"] = out.straight_search_km * out.sweep_width_m / 1000
    return out


def confirm(rng):
    a, b = load(CONFIRM["v1"]), load(CONFIRM["v1.1"])
    rows = []
    for (pattern, metric) in [(p, m) for p in ("trails 3", "trails 6") for m in ("help_1h", "help_3h")]:
        x = a[(a.aircraft == LOD) & (a.pattern == pattern)].set_index(["site", "run"])[metric].astype(float)
        y = b[(b.aircraft == LOD) & (b.pattern == pattern)].set_index(["site", "run"])[metric].astype(float)
        d = (y - x).groupby("run").mean().to_numpy()
        lo, hi = boot(d, rng)
        rows.append(dict(pattern=pattern, metric=metric, v1_pct=100 * x.groupby("site").mean().mean(),
                         v11_pct=100 * y.groupby("site").mean().mean(), delta_pp=100 * d.mean(),
                         low_95_pp=100 * lo, high_95_pp=100 * hi))
    return pd.DataFrame(rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20261116)
    log = []
    helps, paired, contact = [], [], []
    for label, tag in HEADLINE.items():
        runs = load(tag)
        h, p = p1(runs, label, rng)
        helps.append(h)
        paired.append(p)
        if "80%" not in label:
            contact.append(p2(runs, 20261115, label))
    helps, paired, contact = pd.concat(helps), pd.concat(paired), pd.concat(contact)
    helps.to_csv(OUT / "p1_help.csv", index=False)
    paired.to_csv(OUT / "p1_paired.csv", index=False)
    contact.to_csv(OUT / "p2_contact.csv", index=False)
    area = p5()
    area.to_csv(OUT / "p5_area.csv", index=False)
    conf = confirm(rng)
    conf.to_csv(OUT / "confirm_v1_vs_v11.csv", index=False)
    with pd.option_context("display.width", 220, "display.max_rows", 200):
        log += ["P1 help (percent)", helps.round(2).to_string(index=False), "",
                "P1 Lodestar SiCore minus competitor (pp, paired)", paired.round(2).to_string(index=False), "",
                "P2 contact", contact.round(1).to_string(index=False), "",
                "P5 area per sortie", area.round(2).to_string(index=False), "",
                "Held-out confirmation, seed 20261114 (v1.1 minus v1, Lodestar SiCore)", conf.round(2).to_string(index=False)]
    text = "\n".join(log)
    (OUT / "run_log.txt").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
