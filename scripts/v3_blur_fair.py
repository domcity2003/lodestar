"""Design v3: headline comparison with motion blur applied to EVERY aircraft (fair test).

Competitors search slower (14-16.5 m/s), so blur costs them less; this checks the gap still holds.
Lodestar builds as in cost_trades.py (foam-core = "halfway" skins, moulded = laminar).
Same blur model as v3_blur.py: 10 ms base, 7 and 12 ms bounds, N50 = 1.0.

Run:  uv run python scripts/v3_blur_fair.py
Writes: out/v3/blur_fair.csv, blur_fair_log.txt
"""
from pathlib import Path
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import mission_sim as ms  # noqa: E402
import v3_blur as vb  # noqa: E402
from stability import Tee  # noqa: E402

if __name__ == "__main__":
    sys.stdout = Tee(vb.OUT / "blur_fair_log.txt")
    print("Lodestar v3: fair blur comparison, 2026-11-12\n")
    fleet = []
    for lab, cell, finish, v in (("SiCore foam-core", "Amprius SiCore 6.3 Ah", "halfway", 20.0),
                                 ("SiCore foam-core", "Amprius SiCore 6.3 Ah", "halfway", 21.0),
                                 ("P50B foam-core", "Molicel P50B", "halfway", 18.0),
                                 ("SiCore moulded", "Amprius SiCore 6.3 Ah", "laminar", 20.0),
                                 ("P50B moulded", "Molicel P50B", "laminar", 18.0)):
        ac = vb.v3m.lodestar_v3(f"Lodestar {lab} @ {v:.0f}", cell, "kit", finish, 3.0, 7.0)
        ac.v_cruise, ac.search_h = v, ac.energy / float(ac.power(v))
        fleet.append(ac)
    fleet += list(ms.COMPETITORS) + [ms.FRONTIER]
    out = []
    for name, f in (("no blur", None), ("blur 7 ms", vb.w_of_gs(0.007)), ("blur 10 ms", vb.w_of_gs(0.010)),
                    ("blur 12 ms", vb.w_of_gs(0.012))):
        ms.W_OF_GS = f
        rng = np.random.default_rng(ms.RNG_SEED)
        frames = [ms.run_scenario(fleet, sc, box, vb.N, 10.0, rng) for sc, box in ms.SCENARIOS.items()]
        s = ms.summarise(ms.add_help_time(pd.concat(frames, ignore_index=True)), fleet)
        s["case"] = name
        out.append(s)
    ms.W_OF_GS = None
    res = pd.concat(out, ignore_index=True)
    res.round(4).to_csv(vb.OUT / "blur_fair.csv", index=False)
    tab = res.pivot_table(index=["scenario", "aircraft"], columns="case", values="p_help_3h", sort=False) * 100
    with pd.option_context("display.width", 200):
        print("Help within 3 h (%):")
        print(tab[["no blur", "blur 7 ms", "blur 10 ms", "blur 12 ms"]].round(1).to_string())
