"""Design v3, lever 3: wing finish, wing area and span, with the matched v3 propulsion.

Uses the milestone 1 sizing model (NeuralFoil MH45 + induced + pod drag) with:
  - the v3 propulsion: 63.6% battery-to-thrust efficiency at search, 0.34 kg (v3_propulsion.py)
  - the real P50B pack: 30 cells, 540 Wh, 2.42 kg (design v2)
  - 2,000 m, 10% landing reserve (the design case), kit carried the whole flight
Wing finish: "halfway" (v2 basis) or "laminar" (moulded, sanded skins hold laminar flow as
NeuralFoil predicts; milestone 3 range).
Wing structure: 1.0 kg/m^2 at 3.0 m span, scaled by (span / 3.0) for spar bending, ASSUMPTION.

Constraints: stall <= 12 m/s, fly no slower than 1.25 x stall, reach 22 m/s, span sets the
pieces (three pieces must fit in a car: 3.0 m = 0.9 m centre + 2 x 1.05 m outer panels).

Run:  uv run python scripts/v3_planform.py
Writes: out/v3/planform_sweep.csv, planform_log.txt
"""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import sizing as sz  # noqa: E402
import design_v2 as d2  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "v3"
PICK = pd.read_csv(OUT / "propulsion_pick.csv", index_col=0)["value"]
ETA_V3 = float(PICK["eta_sys [search, kit on]"])
M_PROP_V3 = float(PICK["mass_kg"])


def run(b, AR, finish, eta, m_prop):
    d2.at_altitude(2000.0)
    wh, m_b = d2.pack("Molicel P50B", d2.N_WING)
    pf = {"laminar": d2.PF_LAMINAR, "halfway": d2.PF_MID}[finish]
    a = replace(sz.A, profile_factor=pf, cda_body=d2.CDA_BODY, usable_frac=0.85 * 0.90,
                batt_wh_per_kg=wh / m_b, eta_cruise=eta, wing_kg_per_m2=1.0 * b / 3.0,
                prop_kw_per_kg=1e9, m_centerbody=sz.A.m_centerbody + m_prop - 0.05)
    return sz.evaluate(b, AR, m_b, a)


if __name__ == "__main__":
    sys.stdout = Tee(OUT / "planform_log.txt")
    print("Lodestar v3, wing finish, area and span, 2026-11-12, claude-opus-5-5")
    print(f"Propulsion: v2 assumption 55%, 0.28 kg | v3 matched {ETA_V3:.1%}, {M_PROP_V3:.2f} kg\n")
    rows = []
    for b in (3.0, 3.2, 3.4, 3.6):
        for AR in np.arange(5.5, 10.01, 0.5):
            for finish in ("halfway", "laminar"):
                for prop, eta, mp in (("v2 assumed", 0.55, 0.28), ("v3 matched", ETA_V3, M_PROP_V3)):
                    r = run(b, AR, finish, eta, mp)
                    if r:
                        rows.append(dict(finish=finish, propulsion=prop, **r))
    df = pd.DataFrame(rows)
    df["ok"] = (df.v_stall <= 12.0)
    df.round(4).to_csv(OUT / "planform_sweep.csv", index=False)

    base = df[(df.span_m == 3.0) & np.isclose(df.AR, 7.0) & (df.finish == "halfway") & (df.propulsion == "v2 assumed")].iloc[0]
    print(f"v2 baseline (3.0 m, AR 7, halfway finish, 55% propulsion): {base.endurance_h:.2f} h, stall {base.v_stall:.1f} m/s, "
          f"{base.p_search_w:.0f} W at {base.v_search:.1f} m/s\n")
    cols = ["span_m", "AR", "S_m2", "mtow", "v_stall", "v_search", "CL_search", "LD_search", "p_search_w", "endurance_h"]
    for finish in ("halfway", "laminar"):
        d = df[(df.finish == finish) & (df.propulsion == "v3 matched") & df.ok]
        best = d.loc[d.groupby("span_m").endurance_h.idxmax()]
        print(f"v3 propulsion, {finish} finish: best aspect ratio at each span (stall <= 12 m/s):")
        print(best[cols].round(3).to_string(index=False))
        at3 = d[(d.span_m == 3.0)].sort_values("AR")
        print(f"  at 3.0 m span, by aspect ratio:")
        print(at3[cols].round(3).to_string(index=False))
        print()
