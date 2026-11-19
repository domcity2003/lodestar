"""README figures for the v1.1 release, drawn from saved result tables (no simulation is rerun).

Writes docs/figures/*.png:
  fig1_help_by_aircraft.png   help within 1 h and 3 h, each aircraft, nominal with the low-high calibration range
  fig2_found_vs_time.png      share of lost hikers found vs minutes after launch, nominal calibration
  fig3_area_and_cost.png      area searched per sortie and cost per expected person helped
  fig4_mass_budget.png        predicted take-off mass by group against the 10 kg limit
  fig5_endurance.png          endurance Monte Carlo with the rescue kit aboard, against the 3 h requirement

Inputs: out/step2_results, out/wing_trade (v1.1 headline runs, seed 20261115), out/cost_v1.1,
out/parts_list, out/performance_v1.
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out"
FIG = ROOT / "docs" / "figures"
NOMINAL_RUN = OUT / "wing_trade" / "flyable_fleet_pursuit_1_v1.1_seed_20261115_porosity_0.35_odds_1.5"

INK, MUTED, GRID, BG = "#1f1f1e", "#6b6a64", "#e4e3de", "#ffffff"
LODESTAR, OTHER, RANGE = "#2a78d6", "#8d8b84", "#c9c7c0"
ORDER = ["Lodestar SiCore", "DeltaQuad Evo-LE (press)", "DeltaQuad Evo GOV", "C-Astral Bramor ppX",
         "Quantum Trinity Pro", "AgEagle eBee X"]
LABEL = {"Lodestar SiCore": "Lodestar v1.1", "DeltaQuad Evo-LE (press)": "DeltaQuad Evo-LE (press)"}

plt.rcParams.update({
    "font.size": 10, "font.family": "DejaVu Sans", "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "axes.titlecolor": INK, "savefig.dpi": 150,
})


def colour(ac):
    return LODESTAR if ac.startswith("Lodestar") else OTHER


def title(fig, text, sub=None):
    fig.text(0.012, 0.975, text, ha="left", va="top", fontsize=12, color=INK, weight="bold")
    if sub:
        fig.text(0.012, 0.915, sub, ha="left", va="top", fontsize=9, color=MUTED)


def fig1_help():
    d = pd.read_csv(OUT / "step2_results" / "p1_help.csv")
    d["cal"] = d.calibration.str.split().str[0]
    m = d.groupby(["aircraft", "cal"])[["help_1h", "help_3h"]].mean()  # mean of the 3- and 6-trail patterns
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.1), sharey=True)
    y = np.arange(len(ORDER))[::-1]
    for ax, col, ttl in ((axs[0], "help_1h", "Within 1 hour"), (axs[1], "help_3h", "Within 3 hours")):
        for yi, ac in zip(y, ORDER):
            lo, nom, hi = (m.loc[(ac, c), col] for c in ("low", "nominal", "high"))
            c = colour(ac)
            ax.plot([lo, hi], [yi, yi], color=c if c == LODESTAR else RANGE, lw=5, solid_capstyle="round", alpha=0.35 if c == LODESTAR else 1)
            ax.plot(nom, yi, "o", ms=9, color=c, mec="white", mew=1.5, zorder=3)
            ax.text(hi + 0.6, yi, f"{nom:.1f}", va="center", fontsize=9, color=INK,
                    weight="bold" if c == LODESTAR else "normal")
        ax.set_title(ttl, loc="left", fontsize=10.5)
        ax.set_xlabel("Lost hikers helped [%]")
        ax.grid(axis="x", color=GRID)
        ax.set_xlim(0, 26)
    axs[0].set_yticks(y, [LABEL.get(a, a) for a in ORDER])
    for t in axs[0].get_yticklabels():
        if t.get_text().startswith("Lodestar"):
            t.set_color(LODESTAR)
            t.set_weight("bold")
    title(fig, "Figure 1. Lost hikers helped within 1 h and 3 h of launch, by aircraft",
          "3,500 simulated searches on seven US trail areas, seed 20261115. Dot: nominal detection calibration "
          "(value shown); bar: low to high calibration.")
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(FIG / "fig1_help_by_aircraft.png")
    plt.close(fig)


def fig2_found():
    r = pd.read_csv(NOMINAL_RUN / "runs.csv.gz", usecols=["aircraft", "pattern", "mode", "found", "t_find"])
    r = r[r["mode"] == "pursuit_1"]
    t = np.linspace(0, 240, 241)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ends = []
    for ac in ORDER[::-1]:
        s = r[r.aircraft == ac]
        curves = []
        for _, g in s.groupby("pattern"):
            tf = np.sort(g.t_find[g.found].to_numpy() / 60)
            curves.append(np.searchsorted(tf, t, side="right") / len(g) * 100)
        v = np.mean(curves, axis=0)
        lead = ac.startswith("Lodestar")
        ax.plot(t, v, color=colour(ac), lw=2.6 if lead else 1.6, alpha=1 if lead else 0.85, zorder=3 if lead else 2)
        ends.append([v[-1], LABEL.get(ac, ac), lead])
    ends.sort()
    for i in range(1, len(ends)):  # keep end labels at least 1.1 points apart
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 1.1)
    for yv, lab, lead in ends:
        ax.text(243, yv, lab, va="center", fontsize=9, color=LODESTAR if lead else INK, weight="bold" if lead else "normal")
    for x in (60, 180):
        ax.axvline(x, color=MUTED, lw=0.8, ls=":")
    ax.set_xlim(0, 240)
    ax.set_xticks(range(0, 241, 30))
    ax.set_ylim(0, None)
    ax.set_xlabel("Time after launch [min]")
    ax.set_ylabel("Lost hikers found [%]")
    ax.grid(color=GRID)
    title(fig, "Figure 2. Share of lost hikers found vs. time after launch, by aircraft",
          "Nominal detection calibration, mean of the 3- and 6-trail patterns, seed 20261115. Lodestar drops its "
          "kit on finding; the others wait for a ground team.")
    fig.tight_layout(rect=(0, 0, 0.83, 0.86))
    fig.savefig(FIG / "fig2_found_vs_time.png")
    plt.close(fig)


def fig3_area_cost():
    a = pd.read_csv(OUT / "step2_results" / "p5_area.csv").set_index("aircraft")
    c = pd.read_csv(OUT / "cost_v1.1" / "per_help.csv")
    c["ac"] = c.option.str.extract(r"sim: ([^)]+)")[0].fillna("Lodestar SiCore")
    cn = c[c.calibration == "nominal"].set_index("ac")
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.9), gridspec_kw={"width_ratios": [1.35, 1]})
    y = np.arange(len(ORDER))[::-1]
    ax = axs[0]
    for yi, ac in zip(y, ORDER):
        v = a.loc[ac, "area_km2_per_sortie"]
        ax.barh(yi, v, 0.62, color=colour(ac))
        ax.text(v + 0.4, yi, f"{v:.1f}", va="center", fontsize=9, color=INK)
    ax.set_yticks(y, [LABEL.get(x, x) for x in ORDER])
    ax.set_xlabel("Area covered per sortie [km²]")
    ax.set_title("Search area per sortie (track length × sweep width)", loc="left", fontsize=10.5)
    ax.grid(axis="x", color=GRID)
    ax = axs[1]
    names = [x for x in ORDER if x in cn.index]
    y2 = np.arange(len(names))[::-1]
    for yi, ac in zip(y2, names):
        v = cn.loc[ac, "per_expected_help"]
        ax.barh(yi, v, 0.62, color=colour(ac))
        ax.text(v + 60, yi, f"${v:,.0f}", va="center", fontsize=9, color=INK)
    ax.set_yticks(y2, [LABEL.get(x, x) for x in names])
    ax.set_xlabel("USD per expected person helped")
    ax.set_title("Cost per expected person helped within 3 h", loc="left", fontsize=10.5)
    ax.set_xlim(0, cn.per_expected_help.max() * 1.22)
    ax.grid(axis="x", color=GRID)
    for t in [*axs[0].get_yticklabels(), *axs[1].get_yticklabels()]:
        if t.get_text().startswith("Lodestar"):
            t.set_color(LODESTAR)
            t.set_weight("bold")
    title(fig, "Figure 3. Search area per sortie and cost per expected person helped, by aircraft",
          "Area: straight search track on the real-terrain routes. Cost: nominal calibration, owned fleet of 10, "
          "volunteer crew; only aircraft with published prices.")
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(FIG / "fig3_area_and_cost.png")
    plt.close(fig)


def fig4_mass():
    m = pd.read_csv(OUT / "parts_list" / "mass_lines.csv")
    names = {"structure": "Structure", "battery": "Battery", "controlled": "Rescue kit (dropped)",
             "electronics": "Avionics and cameras", "mechanism": "Mechanisms and parachute", "harness": "Harness",
             "propulsion": "Propulsion", "thermal": "Thermal"}
    g = m.groupby("category")[["basic_kg", "mga_kg"]].sum().sort_values("basic_kg")
    basic, mga = g.basic_kg.sum(), g.mga_kg.sum()
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 3.9), gridspec_kw={"width_ratios": [1.6, 1]})
    y = np.arange(len(g))
    ax.barh(y, g.basic_kg, 0.62, color=LODESTAR, label="basic mass")
    ax.barh(y, g.mga_kg, 0.62, left=g.basic_kg, color="white", edgecolor=LODESTAR, hatch="////", lw=0.8,
            label="growth allowance")
    for yi, (b, a) in zip(y, zip(g.basic_kg, g.mga_kg)):
        ax.text(b + a + 0.04, yi, f"{b:.2f} + {a:.2f}", va="center", fontsize=8.5, color=INK)
    ax.set_yticks(y, [names[c] for c in g.index])
    ax.set_xlabel("Mass [kg]")
    ax.set_xlim(0, 3.6)
    ax.set_title("By group", loc="left", fontsize=10.5)
    ax.grid(axis="x", color=GRID)
    ax.legend(frameon=False, fontsize=8.5, loc="lower right")
    ax2.bar(0, basic, 0.5, color=LODESTAR)
    ax2.bar(0, mga, 0.5, bottom=basic, color="white", edgecolor=LODESTAR, hatch="////", lw=0.8)
    ax2.text(0, basic / 2, f"basic\n{basic:.2f} kg", ha="center", va="center", color="white", fontsize=9.5)
    ax2.axhline(10, color="#c0392b", lw=1.6)
    ax2.text(0.3, 10.2, f"10 kg limit (margin {10 - basic - mga:.2f} kg)", fontsize=9, color="#c0392b")
    ax2.text(0.3, basic + mga / 2 - 0.35, f"predicted {basic + mga:.2f} kg\nincl. growth allowance {mga:.2f} kg", va="center", fontsize=9, color=INK)
    ax2.set_xlim(-0.4, 1.9)
    ax2.set_ylim(0, 11)
    ax2.set_xticks([])
    ax2.spines["bottom"].set_visible(False)
    ax2.set_ylabel("Take-off mass [kg]")
    ax2.set_title("Total against the limit", loc="left", fontsize=10.5)
    title(fig, "Figure 4. Take-off mass by group, basic mass plus growth allowance",
          "Parts list v1.1 (docs/parts_list.md). Growth allowance by maturity: measured 0-3%, calculated 10-15%, "
          "estimated 25-60%.")
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(FIG / "fig4_mass_budget.png")
    plt.close(fig)


def fig5_endurance():
    e = pd.read_csv(OUT / "performance_v1" / "endurance_mc.csv.gz")
    s = pd.read_csv(OUT / "performance_v1" / "endurance_summary.csv")
    fig, ax = plt.subplots(figsize=(10, 3.9))
    bins = np.arange(2.3, 4.8, 0.05)
    for case, col, lab in (("design case", LODESTAR, "Design case: 2,000 m, 14-20 m/s, 10% reserve"),
                           ("published style", OTHER, "Published-figure style: sea level, min-power speed, no reserve")):
        h = e[(e.cell == "SiCore") & (e.bay == "rescue kit") & (e.case == case)].hours
        ax.hist(h, bins=bins, color=col, alpha=0.85, edgecolor="white", lw=0.6, label=lab)
        row = s[(s.cell == "SiCore") & (s.bay == "rescue kit") & (s.case == case)].iloc[0]
        ax.axvline(row.p50_h, color=col, lw=1.2, ls="--")
        ax.text(row.p50_h + 0.02, 405, f"median\n{row.p50_h:.2f} h", color=col, fontsize=9, va="bottom")
    ax.axvline(3.0, color="#c0392b", lw=1.6)
    ax.text(2.99, 230, "3 h requirement ", color="#c0392b", fontsize=9, ha="right")
    ax.set_xlabel("Endurance with the rescue kit aboard [h]")
    ax.set_ylabel("Monte Carlo draws")
    ax.legend(frameon=False, loc="upper left", fontsize=9)
    ax.set_ylim(0, 600)
    ax.set_xlim(2.3, 4.8)
    ax.grid(axis="y", color=GRID)
    title(fig, "Figure 5. Endurance with the rescue kit aboard, SiCore cells, by flight condition",
          "4,000 draws per case over cell capacity, profile drag, span efficiency, propulsion efficiency "
          "and mass growth. Dashed: median.")
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    fig.savefig(FIG / "fig5_endurance.png")
    plt.close(fig)


if __name__ == "__main__":
    FIG.mkdir(parents=True, exist_ok=True)
    for f in (fig1_help, fig2_found, fig3_area_cost, fig4_mass, fig5_endurance):
        f()
        print("wrote", f.__name__)
