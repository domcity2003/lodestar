"""Design v3: what search speed maximises "help within 3 h"?

Milestone 4 searched at the best-range speed (most km^2 per Wh). That maximises the total area per
sortie, but the headline score is the chance that help reaches the person within 3 h, and inside
a 3 h window, area per HOUR matters more than area per Wh. This sweeps the search airspeed for the
v3 aircraft (same missions, winds and targets as milestone 4).

Run:  uv run python scripts/v3_search_speed.py
Writes: out/v3/search_speed.csv, search_speed.png, search_speed_log.txt
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
from stability import Tee  # noqa: E402

OUT = v3m.OUT

if __name__ == "__main__":
    sys.stdout = Tee(OUT / "search_speed_log.txt")
    print("Lodestar v3: search-speed sweep, 2026-11-12\n")
    base = {
        "v3 (P50B, kit)": ("Molicel P50B", "kit", "laminar", 3.0, 7.0),
        "v3+ (SiCore, kit)": ("Amprius SiCore 6.3 Ah", "kit", "laminar", 3.0, 7.0),
        "v3 option 3.4 m (P50B, kit)": ("Molicel P50B", "kit", "laminar", 3.4, 9.0),
    }
    fleet, labels = [], {}
    for lab, args in base.items():
        ref = v3m.lodestar_v3(lab, *args)
        print(f"{lab}: best-range search speed {ref.v_cruise:.1f} m/s")
        for v in (None, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0):
            ac = v3m.lodestar_v3(f"{lab} @ {'best range' if v is None else f'{v:.0f} m/s'}", *args)
            if v is not None:
                ac.v_cruise = v
                ac.search_h = ac.energy / float(ac.power(v))
            fleet.append(ac)
            labels[ac.name] = (lab, ac.v_cruise)
    rng = np.random.default_rng(ms.RNG_SEED)
    N = 2000
    frames = [ms.run_scenario(fleet, sc, box, N, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
    df = ms.add_help_time(pd.concat(frames, ignore_index=True))
    summ = ms.summarise(df, fleet)
    summ["design"] = summ.aircraft.map(lambda n: labels[n][0])
    summ["v_search"] = summ.aircraft.map(lambda n: labels[n][1])
    summ.round(3).to_csv(OUT / "search_speed.csv", index=False)
    with pd.option_context("display.width", 220):
        print(summ[["scenario", "aircraft", "v_search", "p_find", "p_kit_delivered", "median_help_min", "p_help_3h"]]
              .round(3).to_string(index=False))

    BLUE, LIGHT, AQUA, INK, MUTED, GRID = "#2a78d6", "#86b6ef", "#1baf7a", "#1f1f1e", "#6b6a64", "#e4e3de"
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4), sharey=False)
    for ax, sc in zip(axs, ms.SCENARIOS):
        for (lab, col) in zip(base, (BLUE, LIGHT, AQUA)):
            s = summ[(summ.scenario == sc) & (summ.design == lab)].sort_values("v_search")
            ax.plot(s.v_search, s.p_help_3h * 100, color=col, lw=2, marker="o", ms=6, label=lab)
        ax.set_xlabel("Search airspeed (m/s)")
        ax.set_ylabel("Help on scene within 3 h (%)")
        ax.set_title(sc, color=INK, loc="left", fontsize=10)
        ax.grid(color=GRID)
        ax.spines[["top", "right"]].set_visible(False)
    axs[0].legend(frameon=False)
    fig.suptitle("Searching faster than the best-range speed gets help there sooner", color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "search_speed.png", dpi=150)
