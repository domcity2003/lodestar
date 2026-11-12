"""Milestone 4: search-and-rescue mission simulation, Rescue Wing vs real competitors.

Every aircraft flies the same missions with the same thermal camera, in the same random winds,
with the same targets (common random numbers), so the comparison is fair.

Mission: launch at base, fly to the search box, fly a parallel-track ("lawnmower") search,
and come home with 10% of the published-flight energy left. If the camera spots the person,
Rescue Wing descends, makes an into-wind run and drops the 1.7 kg survival kit by parachute,
then flies home 1.7 kg lighter. Competitors (none can drop a kit) report the find and go home.

Models:
- Wind triangle: each leg flown at constant airspeed; ground speed and crab from the wind.
  Transit and return legs pick the airspeed that minimises energy per km over the ground.
- Rescue Wing power: the milestone 1 drag model (NeuralFoil MH45 + induced + pod) with the
  milestone 3 pod drag and the "halfway" wing drag, at the actual mass (so the kit drop helps).
- Competitors: published cruise speed and flight time give their cruise power; a standard
  propeller-aircraft power curve scales it with airspeed, with parasite = induced drag at the
  published cruise speed (i.e. makers' "optimal cruise" is their best-range speed).
- Every aircraft searches at its best-range speed (most area per Wh); Rescue Wing's is computed. Their published times are best case (calm, sea level,
  lighter mapping cameras), so this favours them.
- Detection: thermal camera, 32 deg field of view at 120 m above ground (milestone 1). Effective sweep
  width W = 55 m (80% of the 69 m footprint, for canopy and clutter); track spacing = W.
  Each pass detects with Koopman's inverse-cube lateral-range law, p(x) = 1 - exp(-W^2 / (4 pi x^2)),
  the standard search-theory model (integral of p over x equals W).

Run:  uv run python scripts/mission_sim.py
Writes: out/mission/*.csv, *.png, run_log.txt
"""
from dataclasses import dataclass
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import sizing as sz  # noqa: E402
import design_v2 as d2  # noqa: E402
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "mission"
OUT.mkdir(parents=True, exist_ok=True)
G = 9.81
RNG_SEED = 20261111

# ---------------------------------------------------------------- shared sensor and rules
H_AGL = 120.0
SWATH = 2 * H_AGL * math.tan(math.radians(32 / 2))
W_SWEEP = 0.8 * SWATH            # effective sweep width, m
TRACK = W_SWEEP                  # track spacing (coverage factor 1, standard first search)
RESERVE = 0.10                   # land with 10% of published-flight energy
BANK = math.radians(30)          # turns between search legs
TURN_POWER = 1.15                # power factor in a 30 deg banked turn (induced drag x 1.33)
DROP_TIME = 180.0                # s: descend to 40 m, confirm on camera, into-wind run-in, release
KIT_FALL = 40.0 / 5.0            # s: 40 m release height at 5 m/s (kit_drop.py)
CLIMB_BACK = 80.0                # m climbed back to transit altitude after the drop
HIKE = 3.0 / 3.6                 # m/s: ground team cross-country with gear (3 km/h), ASSUMPTION; straight line from base
HELP_WINDOW = 180.0              # min: score = chance that help (kit or team) reaches the person within 3 h of launch


@dataclass
class Aircraft:
    name: str
    v_cruise: float
    v_min: float
    v_max: float
    energy: float                # published-flight energy (Wh for Rescue Wing, cruise-power-hours otherwise)
    p_cruise: float = 1.0        # competitors: normalised so energy = published hours
    kit: bool = False
    rw_cfg: dict | None = None   # Rescue Wing power-model inputs
    mass: float = 0.0
    kit_mass: float = 0.0
    note: str = ""

    def power(self, V, kit_on=True):
        V = np.asarray(V, float)
        if self.rw_cfg is None:
            r = V / self.v_cruise
            return self.p_cruise * (0.5 * r**3 + 0.5 / r)   # parasite = induced at cruise: cruise is the best-range speed
        tab = getattr(self, "_tab", None)
        if tab is not None:  # cached curve (the NeuralFoil interpolation is slow inside the Monte Carlo)
            return np.interp(V, tab[0], tab[1] if kit_on else tab[2])
        m = self.mass - (0 if kit_on else self.kit_mass)
        c = self.rw_cfg
        q = 0.5 * c["rho"] * V**2
        CL = m * G / (q * c["S"])
        Re = c["rho"] * V * c["chord"] / c["mu"]
        CD = sz.cd_profile(np.minimum(CL, 1.2), Re) * c["pf"] + CL**2 / (math.pi * c["e"] * c["AR"]) + c["cda"] / c["S"]
        return q * c["S"] * CD * V / c["eta"] + c["p_av"]


def rescue_wing(name, cell, config, alt=0.0, drag="halfway"):
    pf = {"laminar": d2.PF_LAMINAR, "halfway": d2.PF_MID, "turbulent": d2.PF_TURB}[drag]
    r = d2.endurance(cell, config, pf, alt, 0.85)
    a = asb.Atmosphere(altitude=alt)
    rho, mu = float(a.density()), float(a.dynamic_viscosity())
    S = 3.0**2 / 7.0
    cfg = dict(rho=rho, mu=mu, S=S, chord=3.0 / 7.0, AR=7.0, pf=pf, e=sz.A.e_oswald, cda=d2.CDA_BODY,
               eta=sz.A.eta_cruise, p_av=sz.A.p_avionics_w)
    v_stall = math.sqrt(2 * r["mtow"] * G / (rho * S * sz.A.cl_max_3d))
    ac = Aircraft(name, 15.0, 1.25 * v_stall, 22.0, r["batt_wh"] * 0.85, kit=(config == "kit"), rw_cfg=cfg,
                  mass=r["mtow"], kit_mass=d2.M_KIT_TOTAL if config == "kit" else 0.0,
                  note=f"{cell}, bay: {'rescue kit' if config == 'kit' else 'battery module'}, {drag} wing drag")
    vt = np.linspace(0.9 * ac.v_min, 30.0, 500)
    tab = (vt, ac.power(vt, True), ac.power(vt, False))
    ac._tab = tab
    vg = np.linspace(ac.v_min, ac.v_max, 200)
    pv = ac.power(vg)
    ac.endurance_h = ac.energy / float(pv.min())                  # published-style max flight time (min-power speed)
    ac.v_cruise = float(vg[np.argmin(pv / vg)])                   # search at best range = most area per Wh
    ac.search_h = ac.energy / float(ac.power(ac.v_cruise))
    return ac


def competitor(name, v_cruise, hours, v_max, note):
    ac = Aircraft(name, v_cruise, 0.8 * v_cruise, v_max, hours, note=note)
    ac.endurance_h = ac.search_h = hours
    return ac


COMPETITORS = [
    competitor("DeltaQuad Evo GOV", 16.54, 4.25, 23.0, "VTOL, 10 kg MTOW, dual battery, 1 kg payload; docs.deltaquad.com"),
    competitor("C-Astral Bramor ppX", 16.0, 3.5, 22.0, "catapult/parachute, 4.7 kg, 0.5 kg payload, no thermal listed; c-astral.com"),
    competitor("Quantum Trinity Pro", 17.0, 1.5, 22.0, "VTOL, 5.75 kg; 90 min (59 min by default under export rules); QS techsheet"),
    competitor("AgEagle eBee X", 14.0, 1.5, 30.0, "hand launch, 1.6 kg, 90 min with endurance battery; cruise speed assumed 14 m/s"),
]
FRONTIER = competitor("DeltaQuad Evo-LE (press)", 16.5, 8.0, 23.0,
                      "13 kg MTOW, solid-state cells, launched Sept 2026; 8 h is press-reported; cruise speed assumed")


# ---------------------------------------------------------------- flight mechanics
def ground_speed(Va, w, d):
    """Ground speed along unit track d at airspeed Va in wind w (2-vectors). nan if it can't hold the track."""
    along = w[0] * d[0] + w[1] * d[1]
    cross = w[0] * d[1] - w[1] * d[0]
    s = np.asarray(Va, float) ** 2 - cross**2
    return np.where(s > 0, along + np.sqrt(np.maximum(s, 0)), np.nan)


def leg_cost(ac, p0, p1, w, kit_on=True, Va=None):
    """Time and energy to fly p0 -> p1. Va None = best airspeed for energy per km over the ground."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 0.0, 0.0, 0.0
    d = (dx / L, dy / L)
    if Va is None:
        vg = np.linspace(ac.v_min, ac.v_max, 60)
        gs = ground_speed(vg, w, d)
        cost = np.where(gs > 1.0, ac.power(vg, kit_on) / gs, np.inf)
        i = int(np.argmin(cost))
        if not np.isfinite(cost[i]):
            return np.inf, np.inf, 0.0
        Va, gsv = vg[i], gs[i]
    else:
        gsv = float(ground_speed(Va, w, d))
        if not gsv > 1.0:
            return np.inf, np.inf, 0.0
    t = L / gsv
    return t, float(ac.power(Va, kit_on)) * t / 3600, gsv


def turn_cost(ac, Va, kit_on=True):
    r = Va**2 / (G * math.tan(BANK))
    path = math.pi * r + max(0.0, 2 * r - TRACK)      # semicircle, plus a bulb when the turn is wider than the spacing
    t = path / Va
    return t, float(ac.power(Va, kit_on)) * TURN_POWER * t / 3600


def fly_mission(ac, box, target, wind, draws):
    """One sortie. box = (x0, x1, y0, y1) in m, base at the origin. Returns a result dict."""
    x0, x1, y0, y1 = box
    budget = ac.energy * (1 - RESERVE)
    E, t, kit_on = 0.0, 0.0, ac.kit
    ys = np.arange(y0 + TRACK / 2, y1, TRACK)
    pos = (x0, ys[0])
    dt, dE, _ = leg_cost(ac, (0.0, 0.0), pos, wind, kit_on)
    t, E = t + dt, E + dE
    found_t = kit_t = None
    searched = 0.0
    for k, y in enumerate(ys):
        start = (x0, y) if k % 2 == 0 else (x1, y)
        end = (x1, y) if k % 2 == 0 else (x0, y)
        if k > 0:
            tt, te = turn_cost(ac, ac.v_cruise, kit_on)
            # reposition from the previous leg end (same x, next track) is inside the turn
            t_turn, e_turn = tt, te
        else:
            t_turn, e_turn = 0.0, 0.0
        tl, el, gs = leg_cost(ac, start, end, wind, kit_on, Va=ac.v_cruise)
        _, e_home, _ = leg_cost(ac, end, (0.0, 0.0), wind, kit_on)
        if not np.isfinite(tl) or E + e_turn + el + e_home > budget:
            break                                          # not enough energy for this leg + the trip home
        t, E = t + t_turn, E + e_turn
        # abeam the target on this leg?
        x_abeam = abs(target[0] - start[0])
        p = 1 - math.exp(-W_SWEEP**2 / (4 * math.pi * max((target[1] - y) ** 2, 1e-6)))
        if draws[k] < p:
            found_t = t + x_abeam / gs
            t_here = found_t
            E += el * x_abeam / (x1 - x0)
            searched += TRACK * x_abeam
            here = (target[0], y)
            if ac.kit:
                p_loiter = float(ac.power(ac.v_cruise, True))
                e_drop = p_loiter * DROP_TIME / 3600 + ac.mass * G * CLIMB_BACK / sz.A.eta_max / 3600
                _, e_home2, _ = leg_cost(ac, here, (0.0, 0.0), wind, False)
                if E + e_drop + e_home2 <= budget:        # only drop if the reserve stays intact
                    kit_t = found_t + DROP_TIME + KIT_FALL
                    E += e_drop
                    t_here = found_t + DROP_TIME
                    kit_on = False
            th, eh, _ = leg_cost(ac, here, (0.0, 0.0), wind, kit_on)
            return dict(found=True, t_find=found_t, t_kit=kit_t, t_land=t_here + th, E_used=E + eh,
                        searched_km2=searched / 1e6)
        t, E = t + tl, E + el
        searched += TRACK * (x1 - x0)
        pos = end
    th, eh, _ = leg_cost(ac, pos, (0.0, 0.0), wind, kit_on)
    return dict(found=False, t_find=None, t_kit=None, t_land=t + th, E_used=E + eh, searched_km2=searched / 1e6)


def run_scenario(fleet, name, box, n, wind_max, rng):
    """n missions with random target position, wind speed and direction, and detection draws, shared by all aircraft."""
    x0, x1, y0, y1 = box
    tx = rng.uniform(x0, x1, n)
    ty = rng.uniform(y0, y1, n)
    ws = rng.uniform(0, wind_max, n)
    wd = rng.uniform(0, 2 * math.pi, n)
    n_legs = int(math.ceil((y1 - y0) / TRACK)) + 1
    draws = rng.uniform(0, 1, (n, n_legs))
    rows = []
    for ac in fleet:
        for i in range(n):
            w = (ws[i] * math.cos(wd[i]), ws[i] * math.sin(wd[i]))
            r = fly_mission(ac, box, (tx[i], ty[i]), w, draws[i])
            rows.append(dict(scenario=name, aircraft=ac.name, run=i, wind=ws[i], tx=tx[i], ty=ty[i], **r))
    return pd.DataFrame(rows)


def add_help_time(df):
    """First help on scene: the kit if one was dropped, else a ground team sent at the moment of the find."""
    hike = np.hypot(df.tx, df.ty) / HIKE
    df["t_help"] = np.where(df.t_kit.notna(), df.t_kit, np.where(df.found, df.t_find + hike, np.nan))
    return df


def summarise(df, fleet):
    out = []
    for (sc, name), g in df.groupby(["scenario", "aircraft"], sort=False):
        ac = next(a for a in fleet if a.name == name)
        f = g[g.found]
        k = g[g.t_kit.notna()]
        out.append(dict(
            scenario=sc, aircraft=name, published_h=round(ac.endurance_h, 2),
            p_find=len(f) / len(g),
            median_find_min=f.t_find.median() / 60 if len(f) else np.nan,
            p_kit_delivered=len(k) / len(g),
            median_kit_min=k.t_kit.median() / 60 if len(k) else np.nan,
            area_km2_if_not_found=g[~g.found].searched_km2.median() if (~g.found).any() else g.searched_km2.max(),
            p_find_high_wind=g[g.wind > 7].found.mean(),
            median_help_min=g.t_help.median() / 60 if g.t_help.notna().any() else np.nan,
            p_help_3h=(g.t_help <= HELP_WINDOW * 60).mean(),
        ))
    return pd.DataFrame(out)


SCENARIOS = {
    "A: lost hiker, 16 km2 box 5 km out": (3000.0, 7000.0, -2000.0, 2000.0),
    "B: wide area, 36 km2 box 10 km out": (7000.0, 13000.0, -3000.0, 3000.0),
}

if __name__ == "__main__":
    sys.stdout = Tee(OUT / "run_log.txt")
    print("Rescue Wing milestone 4: mission simulation, 2026-11-11\n")
    print(f"Sensor: 32 deg thermal at {H_AGL:.0f} m: footprint {SWATH:.0f} m, sweep width {W_SWEEP:.0f} m, "
          f"track spacing {TRACK:.0f} m. Reserve {RESERVE:.0%}. Wind 0-10 m/s, random direction. Sea level.\n")

    fleet = [
        rescue_wing("Rescue Wing v2 (P50B, kit)", "Molicel P50B", "kit"),
        rescue_wing("Rescue Wing v2+ (SiCore, kit)", "Amprius SiCore 6.3 Ah", "kit"),
        rescue_wing("Rescue Wing v2+ search-only", "Amprius SiCore 6.3 Ah", "search"),
        *COMPETITORS, FRONTIER,
    ]
    print("Fleet (published-style flight time = calm, sea level, to the maker's empty point):")
    for ac in fleet:
        print(f"  {ac.name:32s} max {ac.endurance_h:4.2f} h | searches at {ac.v_cruise:4.1f} m/s for {ac.search_h:4.2f} h   {ac.note}")
    pd.DataFrame([dict(aircraft=a.name, published_h=a.endurance_h, v_cruise=a.v_cruise, v_max=a.v_max,
                       drops_kit=a.kit, note=a.note) for a in fleet]).round(3).to_csv(OUT / "fleet.csv", index=False)

    rng = np.random.default_rng(RNG_SEED)
    N = 2000
    frames = [run_scenario(fleet, sc, box, N, 10.0, rng) for sc, box in SCENARIOS.items()]
    df = pd.concat(frames, ignore_index=True)
    df = add_help_time(df)
    df.to_csv(OUT / "runs.csv", index=False)
    summ = summarise(df, fleet)
    summ.round(3).to_csv(OUT / "summary.csv", index=False)
    for sc in SCENARIOS:
        print(f"\n{sc}  ({N} missions each, same targets and winds for every aircraft)")
        s = summ[summ.scenario == sc].drop(columns="scenario")
        with pd.option_context("display.width", 220):
            print(s.round(2).to_string(index=False))

    # Sampling error and a robustness check: Rescue Wing at 2000 m with a fully turbulent wing
    p = summ.p_find.clip(0.01, 0.99)
    print(f"\nMonte Carlo standard error on P(find): at most {np.sqrt(0.25 / N):.3f}")
    worst = [rescue_wing("Rescue Wing v2 (P50B, kit) 2000 m turbulent", "Molicel P50B", "kit", 2000.0, "turbulent"),
             rescue_wing("Rescue Wing v2+ (SiCore, kit) 2000 m turbulent", "Amprius SiCore 6.3 Ah", "kit", 2000.0, "turbulent")]
    rng2 = np.random.default_rng(RNG_SEED)
    dfw = run_scenario(worst, "A: lost hiker, 16 km2 box 5 km out", SCENARIOS["A: lost hiker, 16 km2 box 5 km out"], N, 10.0, rng2)
    sw = summarise(add_help_time(dfw), worst)
    print("\nWorst case (2000 m, fully turbulent wing), scenario A:")
    print(sw.drop(columns="scenario").round(2).to_string(index=False))
    sw.round(3).to_csv(OUT / "summary_worst_case.csv", index=False)

    # ------------------------------------------------------------ charts
    BLUE, MUTEDBAR = "#2a78d6", "#b4b2a9"
    INK, MUTED, GRID, BG = "#1f1f1e", "#6b6a64", "#e4e3de", "#fcfcfb"
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": BG, "axes.facecolor": BG})

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
    for ax, sc in zip(axes, SCENARIOS):
        s = summ[summ.scenario == sc].iloc[::-1]
        cols = [BLUE if n.startswith("Rescue") else MUTEDBAR for n in s.aircraft]
        ax.barh(s.aircraft, s.p_find * 100, color=cols, height=0.62)
        for yi, (pf, pk) in enumerate(zip(s.p_find, s.p_kit_delivered)):
            lab = f"{pf:.0%}" + (f"  (kit dropped {pk:.0%})" if pk > 0 else "")
            ax.text(pf * 100 + 1.5, yi, lab, va="center", color=INK, fontsize=9)
        ax.set_xlim(0, 125)
        ax.set_xticks([0, 25, 50, 75, 100])
        ax.grid(axis="x", color=GRID)
        ax.set_title(sc, color=INK, loc="left", fontsize=10)
        ax.set_xlabel("Chance of finding the person in one sortie (%)")
    fig.suptitle("Same camera, same winds, same missing person: Rescue Wing (blue) vs fielded drones (grey)",
                 color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "p_find.png", dpi=160)

    # Time to help: share of ALL missions where help is with the person by time t (scenario A)
    sc = list(SCENARIOS)[0]
    fig, ax = plt.subplots(figsize=(9.5, 4.8))
    tgrid = np.linspace(0, 480, 481)
    series = [("Rescue Wing v2+ (SiCore, kit)", "Rescue Wing v2+: survival kit dropped", BLUE),
              ("Rescue Wing v2 (P50B, kit)", "Rescue Wing v2: survival kit dropped", "#86b6ef"),
              ("DeltaQuad Evo GOV", "Evo GOV finds, ground team walks in", "#eb6834"),
              ("DeltaQuad Evo-LE (press)", "Evo-LE finds, ground team walks in", "#eda100"),
              ("C-Astral Bramor ppX", "Bramor ppX finds, ground team walks in", "#1baf7a")]
    for name, lab, c in series:
        g = df[(df.scenario == sc) & (df.aircraft == name)]
        v = g.t_help.dropna().values / 60
        ax.plot(tgrid, [(v <= t).sum() / len(g) * 100 for t in tgrid], color=c, lw=2, label=lab)
    ax.axvline(HELP_WINDOW, color=MUTED, lw=1, ls="--")
    ax.text(HELP_WINDOW + 4, 92, "3 h", color=MUTED)
    ax.set_xlabel("Minutes after launch")
    ax.set_ylabel("Missions where help has reached the person (%)")
    ax.set_xlim(0, 480)
    ax.set_ylim(0, 100)
    ax.grid(color=GRID)
    ax.legend(frameon=False, loc="upper left", fontsize=9)
    ax.set_title("Scenario A: how fast help reaches the missing person (one sortie, ground team at 3 km/h)",
                 color=INK, loc="left")
    fig.tight_layout()
    fig.savefig(OUT / "time_to_help.png", dpi=160)
    print("\nCharts: out/mission/p_find.png, time_to_help.png")
