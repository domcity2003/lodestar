"""Milestone 6: cost vs design trades. For each option, the unit cost (cost_model.py) and the
mission-sim score (help within 3 h, lost-hiker scenario; same missions as milestone 4), so the
question "is it worth the money?" has a number: dollars per percentage point of missions helped.

Options:
  wing build: foam-core (halfway wing drag, no moulds) vs moulded (laminar-quality skins, moulds)
  cells:      Molicel P50B ($8.25) vs Amprius SiCore (no public price; $23 assumed, $10-35 range)
  search speed: 18 m/s (P50B best) and 20 m/s (top of the search requirement)

Run:  uv run python scripts/cost_trades.py
Writes: out/cost/trades.csv, trades.png, trades_log.txt
"""
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import v3_mission as v3m  # noqa: E402
import cost_model as cm  # noqa: E402
from stability import Tee  # noqa: E402

OUT = cm.OUT
SICORE_PRICE = {"low": 10.0, "mid": 22.7, "high": 35.0}

if __name__ == "__main__":
    sys.stdout = Tee(OUT / "trades_log.txt")
    print("Rescue Wing milestone 6: cost vs design trades, 2026-11-12\n")
    configs = []
    for cell, cname in (("Molicel P50B", "P50B"), ("Amprius SiCore 6.3 Ah", "SiCore")):
        for build, finish in (("foam-core", "halfway"), ("moulded", "laminar")):
            for v in (18.0, 20.0):
                configs.append((cname, cell, build, finish, v))
    fleet, meta = [], {}
    for cname, cell, build, finish, v in configs:
        name = f"{cname}, {build}, {v:.0f} m/s"
        ac = v3m.rescue_wing_v3(name, cell, "kit", finish, 3.0, 7.0)
        ac.v_cruise = v
        ac.search_h = ac.energy / float(ac.power(v))
        fleet.append(ac)
        meta[name] = dict(cells=cname, build=build, v_search=v, published_h=ac.endurance_h)
    rng = np.random.default_rng(ms.RNG_SEED)
    N = 2000
    frames = [ms.run_scenario(fleet, sc, box, N, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
    df = ms.add_help_time(pd.concat(frames, ignore_index=True))
    summ = ms.summarise(df, fleet)

    rows = []
    for _, r in summ.iterrows():
        m = meta[r.aircraft]
        for n in (3, 10, 30):
            price = cm.unit_cost(m["build"], n, cell_price=(SICORE_PRICE["mid"] if m["cells"] == "SiCore" else None))["total"]
            rows.append(dict(scenario=r.scenario[:1], option=r.aircraft, fleet=n, unit_cost=price,
                             p_help_3h=r.p_help_3h, p_find=r.p_find, **m))
    t = pd.DataFrame(rows)
    ref = t[(t.cells == "P50B") & (t.build == "foam-core") & (t.v_search == 18.0)].set_index(["scenario", "fleet"])
    t["d_cost"] = t.apply(lambda x: x.unit_cost - ref.loc[(x.scenario, x.fleet)].unit_cost, axis=1)
    t["d_points"] = t.apply(lambda x: (x.p_help_3h - ref.loc[(x.scenario, x.fleet)].p_help_3h) * 100, axis=1)
    t["usd_per_point"] = np.where(t.d_points > 0.2, t.d_cost / t.d_points, np.nan)
    t.round(3).to_csv(OUT / "trades.csv", index=False)
    with pd.option_context("display.width", 220):
        print("Lost hiker (A) and wide area (B); reference = P50B, foam-core wing, 18 m/s:")
        print(t[t.fleet == 10][["scenario", "option", "published_h", "unit_cost", "p_find", "p_help_3h", "d_cost", "d_points", "usd_per_point"]]
              .round(3).to_string(index=False))
        print("\nUnit cost by fleet size (moulds shared over more aircraft):")
        print(t[t.scenario == "A"].pivot_table(index="option", columns="fleet", values="unit_cost").round(0).to_string())
    print("\nSiCore price range check (foam-core, 10 built): unit cost "
          + ", ".join(f"${cm.unit_cost('foam-core', 10, cell_price=p)['total']:,.0f} at ${p:.0f}/cell" for p in SICORE_PRICE.values()))

    # chart: score vs unit cost (10 built), scenario A
    BLUE, LIGHT, ORANGE, INK, MUTED, GRID = "#2a78d6", "#86b6ef", "#eb6834", "#1f1f1e", "#6b6a64", "#e4e3de"
    a = t[(t.scenario == "A") & (t.fleet == 10)]
    fig, ax = plt.subplots(figsize=(8, 4.6))
    for (cells, build), g in a.groupby(["cells", "build"]):
        col = BLUE if cells == "P50B" else ORANGE
        mk = "o" if build == "foam-core" else "s"
        ax.plot(g.unit_cost, g.p_help_3h * 100, mk, color=col, ms=9, mec="#fcfcfb", mew=1.5, label=f"{cells}, {build} wing")
        for _, r in g.iterrows():
            ax.annotate(f"{r.v_search:.0f} m/s", (r.unit_cost, r.p_help_3h * 100), xytext=(8, -3), textcoords="offset points",
                        color=MUTED, fontsize=8)
    ax.set_xlabel("Unit cost, USD (10 aircraft built, paid labour)")
    ax.set_ylabel("Help on scene within 3 h (%), lost hiker")
    ax.grid(color=GRID)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, fontsize=9, loc="lower right")
    ax.set_title("What each upgrade buys: mission score vs unit cost", color=INK, loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "trades.png", dpi=150)
    print("\nWritten: out/cost/trades.csv, trades.png")
