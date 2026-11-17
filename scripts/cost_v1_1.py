"""P8 (Step 2): cost for Lodestar v1.1, per aircraft, per flight hour and per expected person helped.

Starts from the milestone 6 cost model (cost_model.py, v3 parts list, prices found 2026-11-12) and applies the
v1.1 parts changes below. The v3 outputs in out/cost/ are left as archived.

Per expected help: cost of one sortie / P(help within 3 h) from the headline mission runs (seed 20261115, the
pursuit_1 route, both patterns averaged). Competitors use the same ownership model as cost_model.py (price over
a 400 h life + 10% a year maintenance), their published endurance as the sortie length (capped at 3 h) and no kit.
Only competitors with a published price are included.

Run:    ../.venv/Scripts/python.exe -X utf8 cost_v1_1.py      (from scripts/)
Writes: out/cost_v1.1/ (parts_changes.csv, unit_cost.csv, per_hour.csv, per_help.csv, run_log.txt)
"""
from pathlib import Path

import pandas as pd

import cost_model as cm
from cost_trades import SICORE_PRICE
import mission_sim as ms

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "cost_v1.1"
RUNS = ROOT / "out" / "wing_trade"
HEADLINE = {"low": "flyable_fleet_pursuit_1_v1.1_seed_20261115",
            "nominal": "flyable_fleet_pursuit_1_v1.1_seed_20261115_porosity_0.35_odds_1.5",
            "high": "flyable_fleet_pursuit_1_v1.1_seed_20261115_porosity_0.5_odds_3"}
BOSON = ("FLIR Boson 640, 9.2 mm 50 deg lens: OEMCameras $4,354 list ($4,136 at qty 5, to $6,400 by grade), "
         "https://www.oemcameras.com/?p=4037")

REMOVE = ["Elevon servos: KST X10 Pro"]
ADD = [
    cm.Item("avionics", "Elevon servos: KST A12-S HV (v1.1, 1.35 N m)", 2, 90, 70, 120,
            "ESTIMATE (KST A-series HV retail)"),
    cm.Item("payload", "Nadir thermal: FLIR Boson 640 50 deg core + mount (v1)", 1, 4454, 4236, 6500, BOSON + " + $100 mount"),
    cm.Item("payload", "Twin camera: second Boson 640 50 deg core + splayed mount (v1.1)", 1, 4454, 4236, 6500,
            BOSON + " + $100 mount"),
    cm.Item("safety", "Aircraft parachute: fixed-wing bundle + launcher + independent trigger (RC1)", 1, 1500, 800, 2500,
            "ESTIMATE; anchor Fruity Chutes fixed-wing bundle 12.7 kg from $788 (10 kg size by quote), "
            "https://shop.fruitychutes.com/collections/fixed-wing-bundle"),
    cm.Item("safety", "ADS-B In: uAvionix pingRX Pro (RC8)", 1, 349, 319, 420,
            "uAvionix launch price $349, https://uavionix.com/press/introducing-pingrx-pro/"),
    cm.Item("safety", "Anti-collision strobes, 3 sm (RC8)", 2, 40, 25, 60, "ESTIMATE"),
    cm.Item("safety", "Second GNSS + antenna (RC10)", 1, 120, 60, 200, "ESTIMATE"),
    cm.Item("safety", "Backup avionics supply (2S) + locking connectors (RC2)", 1, 90, 60, 140, "ESTIMATE"),
    cm.Item("safety", "BMS, heater 50 C cut-out, liner (RC3)", 1, 60, 40, 100, "ESTIMATE"),
    cm.Item("power", "8 AWG pack leads (increment)", 1, 20, 10, 30, "ESTIMATE"),
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    log = ["Lodestar v1.1 cost (Step 2, P8)"]
    v3_common = list(cm.COMMON)
    cm.COMMON[:] = [i for i in v3_common if not i.name.startswith(tuple(REMOVE))] + ADD
    changes = pd.DataFrame([dict(change="removed", group=i.group, item=i.name, qty=i.qty, unit_usd=i.unit,
                                 low_usd=i.low, high_usd=i.high, source=i.source)
                            for i in v3_common if i.name.startswith(tuple(REMOVE))]
                           + [dict(change="added", group=i.group, item=i.name, qty=i.qty, unit_usd=i.unit,
                                   low_usd=i.low, high_usd=i.high, source=i.source) for i in ADD])
    changes.to_csv(OUT / "parts_changes.csv", index=False)

    rows = []
    for which, price_key in (("low", "low"), ("unit", "mid"), ("high", "high")):
        for lab, rate in (("paid build", cm.LABOUR_RATE), ("volunteer build", 0.0)):
            u = cm.unit_cost("foam-core", 10, rate, which=which, cell_price=SICORE_PRICE[price_key])
            rows.append(dict(case=which, labour=lab, **u))
    uc = pd.DataFrame(rows)
    uc.to_csv(OUT / "unit_cost.csv", index=False)
    mid = uc[(uc.case == "unit") & (uc.labour == "paid build")].iloc[0]
    log.append(f"Unit cost, foam-core wing, 10 built, SiCore cells at ${SICORE_PRICE['mid']}/cell, paid build: "
               f"${mid.total:,.0f} (range ${uc[(uc.case == 'low') & (uc.labour == 'paid build')].total.iloc[0]:,.0f}"
               f"-${uc[(uc.case == 'high') & (uc.labour == 'paid build')].total.iloc[0]:,.0f}); parts ${mid.parts:,.0f}")

    battery_cost = 30 * SICORE_PRICE["mid"] + 2 * 30
    kit = sum(p for _, p in cm.KIT_CONTENTS["FRS radio"])
    ph = pd.DataFrame([dict(crew_type=c, **cm.per_hour(mid.total, battery_cost, kit, r))
                       for c, r in (("volunteer crew", 0.0), ("paid crew (2 x $50/h)", 100.0))])
    ph.to_csv(OUT / "per_hour.csv", index=False)
    log.append(f"Per flight hour (volunteer crew): ${ph.total.iloc[0]:,.1f}")

    hours = {c.name: min(c.endurance_h, cm.HOURS_PER_SORTIE) for c in ms.COMPETITORS}
    price = {"AgEagle eBee X": ("senseFly eBee VISION (NDAA)", cm.COMPETITOR_PRICE["senseFly eBee VISION (NDAA)"]),
             "Quantum Trinity Pro": ("Quantum Systems Trinity Pro (base)", cm.COMPETITOR_PRICE["Quantum Systems Trinity Pro (base)"])}
    out = []
    for cal, d in HEADLINE.items():
        s = pd.read_csv(RUNS / d / "summary.csv").query("mode == 'pursuit_1'").groupby("aircraft").help_3h.mean()
        per_sortie = ph.total.iloc[0] * cm.HOURS_PER_SORTIE
        out.append(dict(calibration=cal, option="Lodestar v1.1 SiCore (10 built, volunteer crew)", sortie_h=3.0,
                        per_hour=ph.total.iloc[0], per_sortie=per_sortie, p_help_3h=s["Lodestar SiCore"],
                        per_expected_help=per_sortie / s["Lodestar SiCore"]))
        for sim_name, (label, p) in price.items():
            rate = p / cm.AIRFRAME_LIFE_H + cm.MAINT_FRAC * p / (cm.SORTIES_PER_YEAR * cm.HOURS_PER_SORTIE)
            out.append(dict(calibration=cal, option=f"{label} (sim: {sim_name})", sortie_h=hours[sim_name],
                            per_hour=rate, per_sortie=rate * hours[sim_name], p_help_3h=s[sim_name],
                            per_expected_help=rate * hours[sim_name] / s[sim_name]))
    ph_ = pd.DataFrame(out)
    ph_.to_csv(OUT / "per_help.csv", index=False)
    with pd.option_context("display.width", 200):
        log += ["", changes[["change", "item", "qty", "unit_usd"]].to_string(index=False), "",
                uc.round(0).to_string(index=False), "", ph.round(2).to_string(index=False), "",
                ph_.round(3).to_string(index=False)]
    text = "\n".join(log)
    (OUT / "run_log.txt").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
