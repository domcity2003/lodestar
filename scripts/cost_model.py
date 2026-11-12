"""Milestone 6: cost model. Parts list, unit cost, cost per flight hour and per rescue, and the
cost-vs-design trades (wing build method, cells, kit radio) against the mission-sim score.

Prices: US retail listings found 2026-11-12 (sources in the parts list and the research note).
Where no price is published (composite moulds, labour hours, Amprius SiCore cells), the number is
marked ESTIMATE and given a low/high range; the sensitivity table shows what each one moves.

Run:  uv run python scripts/cost_model.py
Writes: out/cost/*.csv, *.png, run_log.txt
"""
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "cost"
OUT.mkdir(parents=True, exist_ok=True)


@dataclass
class Item:
    group: str
    name: str
    qty: float
    unit: float          # USD each (mid)
    low: float
    high: float
    source: str


LABOUR_RATE = 45.0       # USD/h, composite technician, ESTIMATE (volunteer build = 0)

# ---------------------------------------------------------------- parts list (one aircraft, P50B, design v3)
COMMON = [
    # propulsion
    Item("propulsion", "Motor: T-Motor AT3520-550", 1, 80, 80, 122, "Altitude Hobbies $79.99; others to $122"),
    Item("propulsion", "Prop: Aeronaut CAM carbon 15x10 folding blades", 1, 25, 22, 30, "Buchmann CHF 25.65; ModelFlight AU$21.99"),
    Item("propulsion", "Prop hub + spinner (40 mm)", 1, 35, 25, 45, "ESTIMATE (Aeronaut hub/spinner sets)"),
    Item("propulsion", "ESC, 60 A class", 1, 90, 60, 130, "ESTIMATE (Hobbywing/T-Motor 60 A fixed-wing ESCs)"),
    # power
    Item("power", "Cells: Molicel P50B (30, 6S5P in the wing roots)", 30, 8.25, 7.50, 11.99, "18650 Battery Store: $8.25 at 200+, $11.99 single"),
    Item("power", "Pack build: nickel, BMS, leads, XT connectors (2 packs)", 2, 30, 20, 45, "ESTIMATE"),
    # avionics
    Item("avionics", "Autopilot: Holybro Pixhawk 6X + M10 GPS set", 1, 550, 450, 750, "Fruugo $549 (RRP $749)"),
    Item("avionics", "Airspeed sensor + pitot", 1, 60, 45, 90, "ESTIMATE (Matek/Holybro digital airspeed)"),
    Item("avionics", "Video, telemetry + RC link: SIYI HM30 (air + ground)", 1, 380, 245, 450, "SIYI store $284-$382 by bundle"),
    Item("avionics", "Elevon servos: KST X10 Pro", 2, 55, 48, 65, "aifitlab $54.99, Aloft $65"),
    Item("avionics", "Kit release servo", 1, 30, 20, 40, "ESTIMATE"),
    Item("avionics", "Wiring, connectors, power module", 1, 80, 50, 120, "ESTIMATE"),
    # payload
    Item("payload", "Gimbal: SIYI ZT6 (640x512 thermal + 4K, 3-axis)", 1, 2940, 2575, 3700, "Druav $2,940; Irlock $3,700; Alibaba $2,575 (10+)"),
    # kit system (aircraft side)
    Item("kit system", "Kit parachute: 1.5 m cross chute (ripstop, lines, swivel)", 1, 60, 35, 90, "ESTIMATE (204 g chute from milestone 1b)"),
    Item("kit system", "Kit container + bay door hardware", 1, 40, 25, 60, "ESTIMATE"),
]
BUILD = {
    # wing build methods: materials, labour hours per airframe, one-off tooling
    "foam-core": dict(materials=450, mat_lo=350, mat_hi=600, hours=45, h_lo=35, h_hi=60, tooling=600, tool_lo=400, tool_hi=1000,
                      note="hot-wire foam cores, vacuum-bagged glass/carbon skins, sanded; no moulds; 'halfway' wing drag"),
    "moulded": dict(materials=600, mat_lo=450, mat_hi=800, hours=30, h_lo=22, h_hi=40, tooling=12000, tool_lo=7000, tool_hi=20000,
                    note="CNC-cut plugs, moulds for 3 wing pieces + pod + winglets; smooth laminar-quality skins"),
}
# Tooling ESTIMATE anchor: a composite sailplane maker quotes about EUR 6,000 for CNC work and production moulds for one
# sailplane (RC Soaring Digest, 2021); Lodestar has a 3-piece wing plus a pod and winglets, so about 2x.

GROUND = [  # shared per team, not per aircraft
    Item("ground", "Laptop / rugged tablet for ground control", 1, 700, 400, 1500, "ESTIMATE (many teams already own one)"),
    Item("ground", "Bungee launcher", 1, 250, 150, 400, "ESTIMATE"),
    Item("ground", "Li-ion charger (6S, 300 W)", 1, 200, 120, 300, "ESTIMATE"),
]
KIT_CONTENTS = {  # consumable per drop
    "FRS radio": [("FRS/GMRS handheld radio", 35), ("0.5 L water", 3), ("foil blanket", 5), ("LED light", 15), ("first-aid basics", 25)],
    "inReach": [("Garmin inReach Mini 2 satellite messenger", 300), ("0.5 L water", 3), ("foil blanket", 5), ("LED light", 15), ("first-aid basics", 25)],
}

# ---------------------------------------------------------------- operating assumptions
SORTIES_PER_YEAR = 60       # ESTIMATE: about 25 call-outs + 35 training/standby flights for a county SAR team
HOURS_PER_SORTIE = 3.0
AIRFRAME_LIFE_H = 400       # ESTIMATE: belly-landed composite airframe; avionics and gimbal assumed to last as long
CELL_CYCLES = 600           # P50B: maker claims ~1,400 cycles (gentle cycling); independent test ~900 to 80%; deep drone cycles: 600 used
MAINT_FRAC = 0.10           # repairs and spares per year, fraction of aircraft cost, ESTIMATE
DROPS_PER_YEAR = 10         # kits dropped (finds) per year, ESTIMATE

# Mission-sim results (scenario A, lost hiker, 3-h help window; out/v3/search_speed.csv and cost_missions below)
HELI_RATE = {"sheriff helicopter (low)": 1200, "sheriff helicopter (LAPD audit)": 2916, "sheriff helicopter (Sonoma Henry 1)": 4050}
CAP_RATE = {"Civil Air Patrol Cessna 182 (dry rate)": 93, "Civil Air Patrol national blended": 170}
COMPETITOR_PRICE = {"senseFly eBee VISION (NDAA)": 59717, "Quantum Systems Trinity Pro (base)": 21260}


def line_cost(items, which="unit"):
    return sum(i.qty * getattr(i, which) for i in items)


def unit_cost(build, fleet_n, labour_rate=LABOUR_RATE, which="unit", cell_price=None):
    b = BUILD[build]
    parts = line_cost(COMMON, which)
    if cell_price is not None:
        parts += 30 * (cell_price - {"unit": 8.25, "low": 7.5, "high": 11.99}[which])
    k = {"unit": "", "low": "_lo", "high": "_hi"}[which]
    mat = b["materials" if not k else ("mat" + k)]
    hrs = b["hours" if not k else ("h" + k)]
    tool = b["tooling" if not k else ("tool" + k)]
    airframe = mat + hrs * labour_rate
    return dict(parts=parts, airframe=airframe, tooling_share=tool / fleet_n, total=parts + airframe + tool / fleet_n)


def per_hour(unit_total, battery_cost, kit_cost, crew_rate=0.0):
    """Aircraft cost per flight hour (like a CAP 'dry' rate), plus optional paid crew."""
    h_year = SORTIES_PER_YEAR * HOURS_PER_SORTIE
    depreciation = (unit_total - battery_cost) / AIRFRAME_LIFE_H
    battery = battery_cost / CELL_CYCLES / HOURS_PER_SORTIE
    maint = MAINT_FRAC * unit_total / h_year
    kits = DROPS_PER_YEAR * kit_cost / h_year
    return dict(depreciation=depreciation, battery=battery, maintenance=maint, kits=kits, crew=crew_rate,
                total=depreciation + battery + maint + kits + crew_rate)


if __name__ == "__main__":
    sys.stdout = Tee(OUT / "run_log.txt")
    print("Lodestar milestone 6: cost model, 2026-11-12, claude-opus-5-5\n")

    # ------------------------------------------------------------ parts list
    rows = [dict(group=i.group, item=i.name, qty=i.qty, unit_usd=i.unit, line_usd=i.qty * i.unit,
                 low_usd=i.qty * i.low, high_usd=i.qty * i.high, source=i.source) for i in COMMON]
    for build, b in BUILD.items():
        rows.append(dict(group=f"airframe ({build})", item=f"Materials: {b['note']}", qty=1, unit_usd=b["materials"],
                         line_usd=b["materials"], low_usd=b["mat_lo"], high_usd=b["mat_hi"], source="ESTIMATE"))
        rows.append(dict(group=f"airframe ({build})", item=f"Labour: {b['hours']} h at ${LABOUR_RATE:.0f}/h", qty=b["hours"],
                         unit_usd=LABOUR_RATE, line_usd=b["hours"] * LABOUR_RATE, low_usd=b["h_lo"] * LABOUR_RATE,
                         high_usd=b["h_hi"] * LABOUR_RATE, source="ESTIMATE"))
        rows.append(dict(group=f"airframe ({build})", item="Tooling (one-off, shared across the fleet)", qty=1,
                         unit_usd=b["tooling"], line_usd=b["tooling"], low_usd=b["tool_lo"], high_usd=b["tool_hi"],
                         source="ESTIMATE; anchor: ~EUR 6,000 CNC + moulds for one RC sailplane"))
    for i in GROUND:
        rows.append(dict(group=i.group, item=i.name, qty=i.qty, unit_usd=i.unit, line_usd=i.qty * i.unit,
                         low_usd=i.qty * i.low, high_usd=i.qty * i.high, source=i.source))
    bom = pd.DataFrame(rows)
    bom.round(2).to_csv(OUT / "parts_list.csv", index=False)
    by_group = bom[~bom.group.str.startswith(("airframe", "ground"))].groupby("group", sort=False)[["line_usd", "low_usd", "high_usd"]].sum()
    print("Parts (same for both wing builds), USD:")
    print(by_group.round(0).to_string())
    print(f"  subtotal {line_cost(COMMON):,.0f} (range {line_cost(COMMON, 'low'):,.0f}-{line_cost(COMMON, 'high'):,.0f})\n")

    # ------------------------------------------------------------ unit cost by build and fleet size
    urows = []
    for build in BUILD:
        for n in (1, 3, 10, 30, 100):
            for lab, rate in (("paid build", LABOUR_RATE), ("volunteer build", 0.0)):
                u = unit_cost(build, n, rate)
                urows.append(dict(build=build, fleet=n, labour=lab, **u))
    uc = pd.DataFrame(urows)
    uc.round(0).to_csv(OUT / "unit_cost.csv", index=False)
    print("Unit cost per aircraft (USD) by wing build and number built (paid labour):")
    piv = uc[uc.labour == "paid build"].pivot(index="fleet", columns="build", values="total")
    print(piv.round(0).to_string())
    print(f"  Requirement: <= $8,000 in parts. Parts + airframe materials: foam-core "
          f"{line_cost(COMMON) + BUILD['foam-core']['materials']:,.0f}, moulded {line_cost(COMMON) + BUILD['moulded']['materials']:,.0f}\n")

    # ------------------------------------------------------------ cost per flight hour (P50B, foam-core and moulded at 10 built)
    battery_cost = 30 * 8.25 + 2 * 30
    kit_frs = sum(p for _, p in KIT_CONTENTS["FRS radio"])
    kit_irc = sum(p for _, p in KIT_CONTENTS["inReach"])
    hrows = []
    for build in BUILD:
        u = unit_cost(build, 10)["total"]
        for kit_name, kc in (("FRS radio", kit_frs), ("inReach", kit_irc)):
            for crew_lab, crew in (("volunteer crew", 0.0), ("paid crew (2 x $50/h)", 100.0)):
                h = per_hour(u, battery_cost, kc, crew)
                hrows.append(dict(build=build, kit=kit_name, crew_type=crew_lab, unit_cost=u, **h))
    ph = pd.DataFrame(hrows)
    ph.round(2).to_csv(OUT / "cost_per_hour.csv", index=False)
    print(f"Cost per flight hour ({SORTIES_PER_YEAR} sorties x {HOURS_PER_SORTIE:.0f} h a year, {AIRFRAME_LIFE_H} h airframe life, "
          f"{DROPS_PER_YEAR} kits dropped a year; 10 aircraft built):")
    print(ph[["build", "kit", "crew_type", "unit_cost", "depreciation", "battery", "maintenance", "kits", "crew", "total"]]
          .round(1).to_string(index=False))

    # ------------------------------------------------------------ vs alternatives, per 3-h sortie and per "help within 3 h"
    sim = pd.read_csv(ROOT / "out" / "v3" / "search_speed.csv")
    simA = sim[sim.scenario.str.startswith("A")]
    # foam-core wing ("halfway" drag) at 18 m/s, from cost_trades.py (run it first); else the v3 moulded result
    tr = OUT / "trades.csv"
    if tr.exists():
        t_ = pd.read_csv(tr)
        p_v3 = float(t_[(t_.scenario == "A") & (t_.option == "P50B, foam-core, 18 m/s")].p_help_3h.iloc[0])
    else:
        p_v3 = float(simA[simA.aircraft == "v3 (P50B, kit) @ 18 m/s"].p_help_3h.iloc[0])
    comp = pd.read_csv(ROOT / "out" / "v3" / "mission_summary.csv")
    compA = comp[comp.scenario.str.startswith("A")].set_index("aircraft")
    base = ph[(ph.build == "foam-core") & (ph.kit == "FRS radio") & (ph.crew_type == "volunteer crew")].iloc[0]
    alt = [dict(option="Lodestar v3 (foam-core, 10 built, volunteer crew)", per_hour=base.total,
                p_help_3h=p_v3, note="kit dropped; mission sim")]
    for k, life_price in COMPETITOR_PRICE.items():
        # same ownership model as Lodestar: price over 400 h life + 10% a year maintenance; no kit
        rate = life_price / AIRFRAME_LIFE_H + MAINT_FRAC * life_price / (SORTIES_PER_YEAR * HOURS_PER_SORTIE)
        # mission-sim proxy: eBee VISION flies ~90 min like the eBee X in the sim
        p = float(compA.loc["Quantum Trinity Pro" if "Trinity" in k else "AgEagle eBee X"].p_help_3h)
        alt.append(dict(option=k, per_hour=rate, p_help_3h=p, note="same ownership model; finds only, team walks in"))
    for k, r in {**CAP_RATE, **HELI_RATE}.items():
        alt.append(dict(option=k, per_hour=r, p_help_3h=np.nan, note="published hourly rate"))
    alt = pd.DataFrame(alt)
    alt["per_3h_sortie"] = alt.per_hour * HOURS_PER_SORTIE
    alt["per_help_within_3h"] = alt.per_3h_sortie / alt.p_help_3h
    alt.round(1).to_csv(OUT / "vs_alternatives.csv", index=False)
    print("\nCost per 3-h sortie vs alternatives (aircraft cost only unless stated):")
    print(alt.round(1).to_string(index=False))

    # ------------------------------------------------------------ design trades: cost vs score
    print("\nSensitivity of the unit cost (foam-core, 10 built, paid labour) to each estimate, USD:")
    srows = []
    mid = unit_cost("foam-core", 10)["total"]
    srows.append(dict(change="mid case", total=mid))
    srows.append(dict(change="all parts at low prices", total=unit_cost("foam-core", 10, which="low")["total"]))
    srows.append(dict(change="all parts at high prices", total=unit_cost("foam-core", 10, which="high")["total"]))
    srows.append(dict(change="gimbal at $3,700 (Irlock)", total=mid + 3700 - 2940))
    srows.append(dict(change="volunteer build (no labour cost)", total=unit_cost("foam-core", 10, 0.0)["total"]))
    srows.append(dict(change="SiCore cells at $23 each (assumed $1,000/kWh small-volume)", total=mid + 30 * (22.7 - 8.25)))
    sdf = pd.DataFrame(srows)
    sdf["delta"] = sdf.total - mid
    sdf.round(0).to_csv(OUT / "sensitivity.csv", index=False)
    print(sdf.round(0).to_string(index=False))

    # ------------------------------------------------------------ chart: where the money goes
    BLUE, LIGHT, ORANGE, GREY, INK, MUTED, GRID, BG = "#2a78d6", "#86b6ef", "#eb6834", "#b4b2a9", "#1f1f1e", "#6b6a64", "#e4e3de", "#fcfcfb"
    groups = by_group.line_usd.to_dict()
    groups["airframe (foam-core, 10 built)"] = unit_cost("foam-core", 10)["airframe"] + unit_cost("foam-core", 10)["tooling_share"]
    g = pd.Series(groups).sort_values()
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.barh(g.index, g.values, color=[ORANGE if "payload" in k else BLUE for k in g.index], height=0.6)
    for yi, v in enumerate(g.values):
        ax.text(v + 40, yi, f"${v:,.0f}", va="center", color=INK, fontsize=9)
    ax.set_xlabel("USD per aircraft")
    ax.set_xlim(0, g.max() * 1.18)
    ax.grid(axis="x", color=GRID)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_title(f"Unit cost about ${mid:,.0f}: the thermal gimbal is the biggest item",
                 color=INK, loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "unit_cost_breakdown.png", dpi=150)

    # chart: cost per 3-h sortie vs alternatives (log scale)
    a = alt.sort_values("per_3h_sortie")
    fig, ax = plt.subplots(figsize=(9, 4.0))
    ax.barh(a.option, a.per_3h_sortie, color=[BLUE if o.startswith("Lodestar") else GREY for o in a.option], height=0.6)
    for yi, v in enumerate(a.per_3h_sortie):
        ax.text(v * 1.08, yi, f"${v:,.0f}", va="center", color=INK, fontsize=9)
    ax.set_xscale("log")
    ax.set_xlim(a.per_3h_sortie.min() / 1.5, a.per_3h_sortie.max() * 4)
    ax.set_xlabel("Aircraft cost of a 3-hour search sortie, USD (log scale)")
    ax.grid(axis="x", color=GRID)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_title("A 3-hour search: tens of dollars vs thousands for a helicopter", color=INK, loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "cost_per_sortie.png", dpi=150)
    print("\nWritten: out/cost/parts_list.csv, unit_cost.csv, cost_per_hour.csv, vs_alternatives.csv, sensitivity.csv, charts")
