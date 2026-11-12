"""Search-pattern round: which flight pattern gets help to a real lost hiker soonest?

Milestone 4 hid the hiker uniformly in a box and flew a back-and-forth ("lawnmower") sweep. Real
lost people are not uniform. ISRID (International Search & Rescue Incident Database), as tabulated
in SAREnv (Drones 2025, 9, 628, Tables 1-2), temperate mountains, n = 568:
  distance from the initial planning point (IPP, last known position):
      25% 1.1 km, 50% 3.1 km, 75% 5.8 km, 95% 18.3 km  -> log-normal fit (SAREnv also fits log-normal)
  find location, temperate: road 13%, linear (trails) 25%, drainage 12% -> 50% on linear features

Scenario C ("trailhead"): the command post and launch point are at the IPP (a trailhead), so the
aircraft launches there and the ground team walks from there.
  - N_FEAT linear features (trails, roads, drainages) run outward from the IPP at random bearings;
    their positions are known from the map, as in real search planning
  - P_ON of hikers are on one of them (lateral offset ~ N(0, 30 m)); the rest anywhere, at a
    uniform bearing. Distance from the IPP is log-normal for both. ASSUMPTION: P_ON = 0.5 base (ISRID);
    0.3 tested, since minor trails and drainages the aircraft doesn't fly also hold people

Patterns (each aircraft flies at its own search speed, detection and blur as in v3_blur.py):
  box      lawnmower over a 4 x 4 km box centred on the IPP (the milestone 4 way of searching)
  spiral   expanding square from the IPP outwards (IAMSAR pattern): highest density first
  hasty R  fly out along each linear feature to R km and back in along the next ("star"),
           then the expanding square. R swept: 2, 4, 6, 9 km

Run:  uv run python scripts/search_patterns.py [N]   (then --plot to redraw the chart)
Writes: out/search/patterns.csv, patterns_log.txt, patterns.png
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import v3_mission as v3m  # noqa: E402
import v3_blur as vb  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "search"
OUT.mkdir(parents=True, exist_ok=True)

# ISRID temperate mountains quartiles (km) -> log-normal
Q = {0.25: 1.1, 0.50: 3.1, 0.75: 5.8, 0.95: 18.3}
MU = math.log(3.1)
SIGMA = (math.log(5.8) - math.log(1.1)) / (2 * 0.6745)   # from the quartiles: 1.23
N_FEAT = 4
LAT_SD = 30.0
MAX_LEGS = 6000
TRACK_SP = 55.0           # track spacing, as milestone 4 (the blur round showed tighter tracks don't pay)


# ---------------------------------------------------------------- patterns (lists of (p0, p1) legs)
def expanding_square(sp=TRACK_SP, r_max=25000.0, start=(0.0, 0.0)):
    legs, (x, y), k, dirs = [], start, 1, [(1, 0), (0, 1), (-1, 0), (0, -1)]
    i = 0
    while k * sp < 2 * r_max:
        for _ in range(2):
            dx, dy = dirs[i % 4]
            nx, ny = x + dx * k * sp, y + dy * k * sp
            legs.append(((x, y), (nx, ny)))
            x, y = nx, ny
            i += 1
        k += 1
    return legs


def lawnmower_box(half=2000.0, sp=TRACK_SP):
    ys = np.arange(-half + sp / 2, half, sp)
    legs = []
    for k, y in enumerate(ys):
        a, b = ((-half, y), (half, y)) if k % 2 == 0 else ((half, y), (-half, y))
        if legs:
            legs.append((legs[-1][1], a))          # short repositioning hop
        legs.append((a, b))
    return legs


def hasty_then_spiral(bearings, R):
    """Out along feature 1, hop to feature 2's end, in along feature 2, out along 3, hop, in along 4 ..."""
    legs, pos = [], (0.0, 0.0)
    for j, th in enumerate(bearings):
        end = (R * math.cos(th), R * math.sin(th))
        if j % 2 == 0:
            legs.append((pos, end))
            pos = end
        else:
            legs.append((pos, end))
            legs.append((end, (0.0, 0.0)))
            pos = (0.0, 0.0)
    if pos != (0.0, 0.0):
        legs.append((pos, (0.0, 0.0)))
    return legs + expanding_square()


# ---------------------------------------------------------------- generic path flier
def heading(p0, p1):
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def fly_path(ac, legs, target, wind, draws):
    """Same physics, reserve, find/drop/home logic as mission_sim.fly_mission, for any list of legs."""
    budget = ac.energy * (1 - ms.RESERVE)
    E = t = 0.0
    kit_on = ac.kit
    pos, hdg = (0.0, 0.0), None
    Wf = ms.W_OF_GS
    for k, (p0, p1) in enumerate(legs[:MAX_LEGS]):
        L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if L < 1.0:
            continue
        h = heading(p0, p1)
        if hdg is None:
            et = tt = 0.0
        else:
            d = abs((h - hdg + math.pi) % (2 * math.pi) - math.pi)
            r = ac.v_cruise**2 / (ms.G * math.tan(ms.BANK))
            tt = r * d / ac.v_cruise
            et = float(ac.power(ac.v_cruise, kit_on)) * ms.TURN_POWER * tt / 3600
        tl, el, gs = ms.leg_cost(ac, p0, p1, wind, kit_on, Va=ac.v_cruise)
        _, e_home, _ = ms.leg_cost(ac, p1, (0.0, 0.0), wind, kit_on)
        if not np.isfinite(tl) or E + et + el + e_home > ac.energy * (1 - ms.RESERVE):
            break
        t, E = t + tt, E + et
        ux, uy = (p1[0] - p0[0]) / L, (p1[1] - p0[1]) / L
        s = (target[0] - p0[0]) * ux + (target[1] - p0[1]) * uy
        if 0.0 <= s <= L:
            x = abs((target[0] - p0[0]) * uy - (target[1] - p0[1]) * ux)
            W = ms.W_SWEEP if Wf is None else Wf(gs)
            p = 1 - math.exp(-W**2 / (4 * math.pi * max(x * x, 1e-6)))
            if draws[k] < p:
                found_t = t + s / gs
                E += el * s / L
                here = (p0[0] + ux * s, p0[1] + uy * s)
                kit_t, t_here = None, found_t
                if ac.kit:
                    p_loiter = float(ac.power(ac.v_cruise, True))
                    e_drop = p_loiter * ms.DROP_TIME / 3600 + ac.mass * ms.G * ms.CLIMB_BACK / ms.sz.A.eta_max / 3600
                    _, e_home2, _ = ms.leg_cost(ac, here, (0.0, 0.0), wind, False)
                    if E + e_drop + e_home2 <= budget:
                        kit_t = found_t + ms.DROP_TIME + ms.KIT_FALL
                        E += e_drop
                        t_here = found_t + ms.DROP_TIME
                        kit_on = False
                th, eh, _ = ms.leg_cost(ac, here, (0.0, 0.0), wind, kit_on)
                return dict(found=True, t_find=found_t, t_kit=kit_t, t_land=t_here + th, E_used=E + eh)
        t, E = t + tl, E + el
        pos, hdg = p1, h
    th, eh, _ = ms.leg_cost(ac, pos, (0.0, 0.0), wind, kit_on)
    return dict(found=False, t_find=None, t_kit=None, t_land=t + th, E_used=E + eh)


def draw_world(n, rng, p_on):
    bear = rng.uniform(0, 2 * math.pi, (n, N_FEAT))
    on = rng.uniform(0, 1, n) < p_on
    feat = rng.integers(0, N_FEAT, n)
    r = np.exp(rng.normal(MU, SIGMA, n)) * 1000.0
    th_free = rng.uniform(0, 2 * math.pi, n)
    lat = rng.normal(0, LAT_SD, n)
    th = np.where(on, bear[np.arange(n), feat], th_free)
    tx = r * np.cos(th) - np.where(on, lat * np.sin(th), 0.0)
    ty = r * np.sin(th) + np.where(on, lat * np.cos(th), 0.0)
    ws = rng.uniform(0, 10.0, n)
    wd = rng.uniform(0, 2 * math.pi, n)
    draws = rng.uniform(0, 1, (n, MAX_LEGS))
    return dict(bear=bear, on=on, tx=tx, ty=ty, r=r, ws=ws, wd=wd, draws=draws)


def run(fleet, patterns, n, p_on, seed=ms.RNG_SEED):
    rng = np.random.default_rng(seed)
    w = draw_world(n, rng, p_on)
    rows = []
    box = lawnmower_box()
    spiral = expanding_square()
    for pname in patterns:
        for ac in fleet:
            for i in range(n):
                if pname == "box":
                    legs = box
                elif pname == "spiral":
                    legs = spiral
                else:
                    legs = hasty_then_spiral(w["bear"][i], float(pname.split()[1]) * 1000.0)
                wind = (w["ws"][i] * math.cos(w["wd"][i]), w["ws"][i] * math.sin(w["wd"][i]))
                res = fly_path(ac, legs, (w["tx"][i], w["ty"][i]), wind, w["draws"][i])
                rows.append(dict(pattern=pname, aircraft=ac.name, run=i, on_feature=bool(w["on"][i]),
                                 dist_km=w["r"][i] / 1000, tx=w["tx"][i], ty=w["ty"][i], wind=w["ws"][i], **res))
    df = pd.DataFrame(rows)
    hike = np.hypot(df.tx, df.ty) / ms.HIKE
    df["t_help"] = np.where(df.t_kit.notna(), df.t_kit, np.where(df.found, df.t_find + hike, np.nan))
    df["help_3h"] = df.t_help <= ms.HELP_WINDOW * 60
    df["team_only_3h"] = hike <= ms.HELP_WINDOW * 60     # a ground team that walked straight there
    return df


def build_fleet():
    fleet = []
    for lab, cell, v in (("Lodestar SiCore", "Amprius SiCore 6.3 Ah", 20.0), ("Lodestar P50B", "Molicel P50B", 18.0)):
        ac = v3m.lodestar_v3(lab, cell, "kit", "halfway", 3.0, 7.0)
        ac.v_cruise, ac.search_h = v, ac.energy / float(ac.power(v))
        fleet.append(ac)
    return fleet + list(ms.COMPETITORS) + [ms.FRONTIER]


def job(args):
    pattern, p_on, n = args
    ms.W_OF_GS = vb.w_of_gs(0.010)
    df = run(build_fleet(), [pattern], n, p_on)
    df["p_on"] = p_on
    return df


PATTERNS = ["box", "spiral", "hasty 2", "hasty 4", "hasty 6", "hasty 9"]

if __name__ == "__main__" and "--plot" not in sys.argv:
    from concurrent.futures import ProcessPoolExecutor
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    sys.stdout = Tee(OUT / "patterns_log.txt")
    print(f"Lodestar search-pattern round, 2026-11-12. N = {N} hikers per case.\n")
    print(f"ISRID temperate mountains: log-normal median 3.1 km, sigma {SIGMA:.2f}; "
          f"check: 25% {math.exp(MU - 0.6745 * SIGMA):.1f} km, 75% {math.exp(MU + 0.6745 * SIGMA):.1f} km, "
          f"95% {math.exp(MU + 1.645 * SIGMA):.1f} km (ISRID 1.1 / 5.8 / 18.3)\n")
    patterns = PATTERNS
    with ProcessPoolExecutor() as ex:
        out = list(ex.map(job, [(p, p_on, N) for p_on in (0.5, 0.3) for p in patterns]))
    df = pd.concat(out, ignore_index=True)
    df.to_csv(OUT / "patterns_runs.csv.gz", index=False)
    s = (df.groupby(["p_on", "pattern", "aircraft"], sort=False)
           .agg(p_find=("found", "mean"), p_help_3h=("help_3h", "mean"),
                median_help_min=("t_help", lambda x: x.median() / 60),
                team_alone_3h=("team_only_3h", "mean"))
           .reset_index())
    s.round(4).to_csv(OUT / "patterns.csv", index=False)
    with pd.option_context("display.width", 220):
        for p_on in (0.5, 0.3):
            tab = s[s.p_on == p_on].pivot_table(index="aircraft", columns="pattern", values="p_help_3h", sort=False) * 100
            print(f"\nHelp within 3 h (%), {p_on:.0%} of hikers on the flown linear features:")
            print(tab[patterns].round(1).to_string())
    print(f"\nA ground team walking straight to the hiker from the trailhead would arrive within 3 h in "
          f"{df.team_only_3h.mean():.0%} of cases (it doesn't know where to walk; upper bound).")


def plot():
    """Two panels: help within 3 h by pattern, and help-on-scene time curves for the hasty search."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = pd.read_csv(OUT / "patterns_runs.csv.gz")
    d = d[d.p_on == 0.5]
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    who = {"Lodestar SiCore": ("#2a78d6", "Lodestar (SiCore, 20 m/s)"),
           "DeltaQuad Evo GOV": ("#6b6a64", "DeltaQuad Evo GOV (4.25 h)"),
           "Quantum Trinity Pro": ("#b4b2a9", "Quantum Trinity Pro (1.5 h)")}
    labels = {"box": "4x4 km box\nlawnmower", "spiral": "expanding\nsquare", "hasty 2": "trails to\n2 km first",
              "hasty 4": "trails to\n4 km first", "hasty 6": "trails to\n6 km first", "hasty 9": "trails to\n9 km first"}
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8), gridspec_kw=dict(width_ratios=[1.25, 1]))
    ax = axs[0]
    x = np.arange(len(PATTERNS))
    for j, (a, (c, lab)) in enumerate(who.items()):
        y = [d[(d.pattern == p) & (d.aircraft == a)].help_3h.mean() * 100 for p in PATTERNS]
        ax.bar(x + (j - 1) * 0.26, y, 0.24, color=c, label=lab)
    ax.set_xticks(x, [labels[p] for p in PATTERNS], fontsize=9)
    ax.set_ylabel("Help on scene within 3 h (%)")
    ax.set_title("Flying the trails first lifts every aircraft", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    ax = axs[1]
    tt = np.arange(0, 181, 2)
    for a, (c, lab) in who.items():
        g = d[(d.pattern == "hasty 6") & (d.aircraft == a)]
        ax.plot(tt, [(g.t_help <= t * 60).mean() * 100 for t in tt], color=c, lw=2, label=lab)
    ax.axvline(60, color=GRID, lw=1)
    ax.set_xlabel("Minutes after launch")
    ax.set_ylabel("Help on scene (%)")
    ax.set_title("Trails to 6 km first: help arrives about 40 minutes sooner", loc="left", fontsize=10, color=INK)
    ax.set_xticks(range(0, 181, 30))
    for ax in axs:
        ax.grid(color=GRID, axis="y")
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Lost hiker, launch at the trailhead (ISRID temperate mountains, half on trails, roads or drainages)",
                 color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "patterns.png", dpi=150)


if __name__ == "__main__" and "--plot" in sys.argv:
    plot()
