"""README figures for the v1.1 release, drawn from saved result tables (no simulation is rerun).

Writes docs/figures/*.png:
  fig1_help_by_aircraft.png   help within 1 h and 3 h, each aircraft, nominal with the low-high calibration range
  fig2_found_vs_time.png      share of lost hikers found vs minutes after launch, nominal calibration
  fig3_search_areas.png       the seven search areas: terrain, trails, roads, hiker draws, Lodestar's first leg
  fig4_area_and_cost.png      area searched per sortie and cost per expected person helped
  fig5_mass_budget.png        predicted take-off mass by group against the 10 kg limit
  fig6_endurance.png          endurance Monte Carlo with the rescue kit aboard, against the 3 h requirement
  fig7_power_curves.png       thrust power required and available at the two envelope corners (propulsion_margins model)
  fig8_kit_release.png        6-DOF kit release traces (re-flown with sixdof_v1.py, same model and seed)

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

FONT = ["Segoe UI", "Arial", "DejaVu Sans"]
INK, MUTED, GRID, BG = "#1f1f1e", "#6b6a64", "#e4e3de", "#ffffff"
LODESTAR, OTHER, RANGE = "#2a78d6", "#8d8b84", "#c9c7c0"
ORDER = ["Lodestar SiCore", "DeltaQuad Evo-LE (press)", "DeltaQuad Evo GOV", "C-Astral Bramor ppX",
         "Quantum Trinity Pro", "AgEagle eBee X"]
LABEL = {"Lodestar SiCore": "Lodestar v1.1", "DeltaQuad Evo-LE (press)": "DeltaQuad Evo-LE (press)"}

plt.rcParams.update({
    "font.size": 10, "font.family": FONT, "mathtext.default": "regular", "axes.spines.top": False, "axes.spines.right": False,
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
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 0.85)
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
    title(fig, "Figure 4. Search area per sortie and cost per expected person helped, by aircraft",
          "Area: straight search track on the real-terrain routes. Cost: nominal calibration, owned fleet of 10, "
          "volunteer crew; only aircraft with published prices.")
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(FIG / "fig4_area_and_cost.png")
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
    title(fig, "Figure 5. Take-off mass by group, basic mass plus growth allowance",
          "Parts list v1.1 (docs/parts_list.md). Growth allowance by maturity: measured 0-3%, calculated 10-15%, "
          "estimated 25-60%.")
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(FIG / "fig5_mass_budget.png")
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
    ax.text(2.99, 330, "3 h requirement ", color="#c0392b", fontsize=9, ha="right")
    ax.set_xlabel("Endurance with the rescue kit aboard [h]")
    ax.set_ylabel("Monte Carlo draws")
    fig.legend(*ax.get_legend_handles_labels(), loc="upper left", bbox_to_anchor=(0.012, 0.86), ncol=2,
               frameon=False, fontsize=9)
    ax.set_ylim(0, 480)
    ax.set_xlim(2.3, 4.8)
    ax.grid(axis="y", color=GRID)
    title(fig, "Figure 6. Endurance with the rescue kit aboard, SiCore cells, by flight condition",
          "4,000 draws per case over cell capacity, profile drag, span efficiency, propulsion efficiency "
          "and mass growth. Dashed: median.")
    fig.tight_layout(rect=(0, 0, 1, 0.78))
    fig.savefig(FIG / "fig6_endurance.png")
    plt.close(fig)


def fig6_power():
    import propulsion_margins as pm
    cell_name = "Amprius SiCore 6.3 Ah"
    cell = pm.CELLS[cell_name]
    m, _ = pm.baseline(cell_name)
    cases = [("Sea level, +20 °C", pm.air(0.0, 20.0), 20.0, LODESTAR),
             ("4,400 m density altitude, −10 °C", pm.air(4400.0, -10.0), 0.0, "#c2410c")]
    fig, ax = plt.subplots(figsize=(10, 4.4))
    for label, ai, t_cell, col in cases:
        vs = np.arange(max(12.0, pm.v_stall(m, ai)), 32.01, 0.5)
        pav = np.array([(pm.available(V, ai, cell, t_cell, pm.SOC_END, False) or {"T": np.nan})["T"] * V for V in vs])
        dv = np.array([float(pm.drag(V, m, ai)) * V for V in vs])
        ax.plot(vs, pav, color=col, lw=2, ls=(0, (5, 2.5)))
        ax.plot(vs, dv, color=col, lw=2)
        ax.text(vs[0] - 0.3, pav[0], f"{label}\navailable", color=col, fontsize=9, ha="right", va="center")
        k = np.searchsorted(vs, 26.0)
        above = col == LODESTAR
        ax.annotate(f"{label}, required", (vs[k], dv[k]), (-14 if above else 14, 22 if above else -24),
                    textcoords="offset points", ha="right" if above else "left", va="center", fontsize=9, color=col,
                    arrowprops=dict(arrowstyle="-", lw=0.6, color=col))
    ax.axvline(22, color=MUTED, lw=0.9, ls=":")
    ax.text(22.2, 12, "22 m/s requirement", color=MUTED, fontsize=9, va="bottom")
    ax.set_xlim(5.5, 32.5)
    ax.set_ylim(0, 420)
    ax.set_xticks(range(12, 33, 2))
    ax.set_xlabel("True airspeed [m/s]")
    ax.set_ylabel("Thrust power [W]")
    ax.grid(color=GRID)
    title(fig, "Figure 7. Thrust power required and available vs. airspeed, at the two envelope corners",
          "SiCore cells at the end of the usable charge, kit aboard. Solid: power required; dashed: continuous "
          "power available.")
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    fig.savefig(FIG / "fig7_power_curves.png")
    plt.close(fig)


def fig7_kit_release():
    """Re-flies the three kit-release traces of sixdof_v1.py (same model, trims and seed) for a restyled plot."""
    import sixdof_v1 as sx
    sx.load(rebuild=False)
    traces = {}
    tr = sx.calm_trim(True, sx.V_DROP, sx.ALT)
    for frozen in (False, True):
        sim = sx.Sim(sx.ALT, sx.H_REL, sx.V_DROP, alpha_deg=tr["alpha_deg"])
        sim.set_trim(tr)
        sim.step_until(sim.t() + 10.0)
        sim.log.clear()
        t0 = sim.t()
        sim.step_until(t0 + 5.0 - sx.DT)
        if frozen:
            sim.autopilot = lambda: None
        sim.step_until(t0 + 35.0, release_at=t0 + 5.0)
        d = pd.DataFrame(sim.log)
        d["t"] -= t0 + 5.0
        traces[f"Calm, {sx.V_DROP:.0f} m/s, autopilot on" if not frozen else
               f"Calm, {sx.V_DROP:.0f} m/s, controls frozen at release"] = d
    trt = sx.calm_trim(True, sx.V_DROP_TURB, sx.ALT)
    rho = sx.isa_air(sx.ALT)["rho"]
    _, d, _ = sx.turb_run("moderate", sx.V_DROP_TURB, sx.H_REL, True, sx.SEED_BASE + 500, trt, rho,
                          t_settle=10.0, t_run=25.0, release=True)
    traces[f"Moderate turbulence, {sx.V_DROP_TURB:.0f} m/s, autopilot on"] = d
    cols = [LODESTAR, "#c2410c", OTHER]
    fig, axs = plt.subplots(2, 2, figsize=(11, 5.6), sharex=True)
    panels = (("h_agl", "Height above ground [m]"), ("theta", "Pitch [deg]"), ("V", "True airspeed [m/s]"),
              ("elevon", "Elevon [deg]"))
    for ax, (k, yl) in zip(axs.flat, panels):
        for (name, d), c in zip(traces.items(), cols):
            ax.plot(d.t, d[k], color=c, lw=1.6 if c != OTHER else 1.0, label=name, zorder=3 if c != OTHER else 2)
        ax.axvline(0, color=MUTED, lw=0.8, ls=":")
        ax.set_ylabel(yl)
        ax.grid(color=GRID)
        ax.set_xlim(-5, 25)
    for ax in axs[1]:
        ax.set_xlabel("Time after release [s]")
    fig.legend(*axs[0, 0].get_legend_handles_labels(), loc="upper left", bbox_to_anchor=(0.012, 0.885), ncol=3,
               frameon=False, fontsize=9)
    title(fig, "Figure 8. Height, pitch, airspeed and elevon vs. time through the kit release, by control mode",
          f"JSBSim 6-DOF, release of {sx.KIT_KG:.2f} kg at {sx.H_REL:.0f} m above ground, {sx.ALT:,.0f} m; "
          "kit door open 1 s before to 2 s after release.")
    fig.tight_layout(rect=(0, 0, 1, 0.8))
    fig.savefig(FIG / "fig8_kit_release.png")
    plt.close(fig)


def fig8_terrain(n=400):
    """The seven search areas: terrain, trails, roads, ISRID hiker draws and Lodestar's first leg (terrain_sim.py)."""
    import math

    import search_patterns as sp
    import terrain_sim as ts

    fig, axs = plt.subplots(2, 4, figsize=(15, 8.4))
    for ax, key in zip(axs.ravel(), ts.SITES):
        dem, lines, meta = ts.load(key)
        P = ts.prep(key, n, ts.ms.RNG_SEED)
        gy, gx = np.gradient(dem, ts.CELL)
        az, alt = math.radians(315), math.radians(45)
        slope = np.arctan(np.hypot(gx, gy))
        aspect = np.arctan2(-gx, gy)
        hs = np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)
        h = ts.HALF / 1000
        ext = [-h, h, -h, h]
        ax.imshow(dem, cmap="gist_earth", extent=ext, alpha=0.45, vmin=dem.min() - 600, vmax=dem.max() + 300)
        ax.imshow(hs, cmap="gray", extent=ext, alpha=0.35)
        for ln in lines:
            x, y = np.asarray(ln["x"]) / 1000, np.asarray(ln["y"]) / 1000
            if ln["kind"] == "trail":
                ax.plot(x, y, color="#7a3b10", lw=0.45)
            elif ln["kind"] == "road":
                ax.plot(x, y, color="#333333", lw=0.7)
        spc = ts.CAMERAS["Lodestar"][1]
        legs = P["pats"][("trails 6", spc)]["legs"]
        k = len(legs) - len(sp.expanding_square(spc))
        for a, b in legs[:k]:
            ax.plot([a[0] / 1000, b[0] / 1000], [a[1] / 1000, b[1] / 1000], color=LODESTAR, lw=1.2)
        hk = P["hikers"]
        hk = hk[hk.on_map]
        ax.scatter(hk.tx / 1000, hk.ty / 1000, s=4, color="#e8590c", lw=0, alpha=0.85)
        ax.plot(0, 0, marker="*", ms=13, color="#ffd400", mec=INK, mew=0.8)
        ax.set_xlim(-h, h)
        ax.set_ylim(-h, h)
        ax.set_title(f"{meta['name']}\n{meta['elev_min']:,.0f}–{meta['elev_max']:,.0f} m", fontsize=10, loc="left")
        ax.set_xticks([-10, 0, 10])
        ax.set_yticks([-10, 0, 10])
        ax.tick_params(labelsize=8.5)
        for sp_ in ("top", "right"):
            ax.spines[sp_].set_visible(True)
    ax = axs.ravel()[-1]
    ax.axis("off")
    keys = [(LODESTAR, "Lodestar's first leg: trails and roads to 6 km", dict(lw=2.2)),
            ("#7a3b10", "Mapped trails (OpenStreetMap)", dict(lw=1)),
            ("#333333", "Roads", dict(lw=1)),
            ("#e8590c", "Simulated lost hikers (ISRID)", dict(marker="o", lw=0, ms=5)),
            ("#ffd400", "Trailhead: launch and ground team", dict(marker="*", lw=0, ms=13, mec=INK))]
    for i, (c, lab, kw) in enumerate(keys):
        xs = [0.10] if "marker" in kw else [0.04, 0.16]          # one marker per key, not one per end
        ax.plot(xs, [0.86 - i * 0.12] * len(xs), color=c, **kw)
        ax.text(0.21, 0.86 - i * 0.12, lab, va="center", fontsize=10, color=INK)
    ax.text(0.04, 0.12, "30 × 30 km around each trailhead, km on the axes.\nElevation: AWS terrain tiles (~30 m).",
            fontsize=9, color=MUTED)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    title(fig, "Figure 3. The seven search areas: elevation, trails, roads and simulated lost-hiker positions")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(FIG / "fig3_search_areas.png", dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    FIG.mkdir(parents=True, exist_ok=True)
    for f in (fig1_help, fig2_found, fig3_area_cost, fig4_mass, fig5_endurance, fig6_power, fig7_kit_release,
              fig8_terrain):
        f()
        print("wrote", f.__name__)
