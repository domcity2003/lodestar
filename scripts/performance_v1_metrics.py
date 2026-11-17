"""Step 2 spec-sheet performance metrics for the frozen v1 design (P3, P4, P6, P7 of the Step 2 plan).

P3  endurance with the rescue kit: design case (2,000 m, best speed 14-20 m/s, 10% reserve on top of the
    85% depth of discharge) and published style (sea level, min-power speed, 85% depth, no reserve), with a
    parameter Monte Carlo for P10 / P50 / P90.
P4  search-only endurance with the 18-cell battery module in the kit bay, same two conventions.
P6  wind penetration: max level speed on continuous power and groundspeed into a 12 m/s headwind
    (Lodestar from out/propulsion_margins/margins.csv, worst corner; competitors from published v_max).
P7  dash: time to reach a point 5 km away in still air and into a 10 m/s headwind, at max level speed.

The power model is the same one the mission simulation uses (v3_mission.lodestar_v3 with the v1 structure
mass from fleet_flyable.lodestar_structure_kg). The Monte Carlo perturbs that model's inputs:
- cell capacity: normal, sigma 3% (cell-to-cell and lot spread; clipped at +-2 sigma);
- profile drag (parasite power): uniform 0.85-1.20 (the finish range between the laminar and turbulent cases);
- span efficiency e: uniform 0.70-0.85 (sizing assumes 0.80);
- propulsion system efficiency: uniform 0.95-1.03 times the matched-prop table;
- take-off mass growth: triangular 0 / +4% / +10% (the 10% is the AIAA-style allowance in mass_growth.py).

Run:    .venv/Scripts/python.exe -X utf8 scripts/performance_v1_metrics.py
Writes: out/performance_v1/ (endurance_mc.csv, endurance_summary.csv, wind_dash.csv, log.txt)
"""
import math
from pathlib import Path

import numpy as np
import pandas as pd

import terrain_sim as t
from fleet_flyable import lodestar_structure_kg

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "performance_v1"
G = 9.80665
N_MC = 4000
CELLS = {"SiCore": "Amprius SiCore 6.3 Ah", "P50B": "Molicel P50B"}
SEED = 20261115


def build(cell, config, alt, mass):
    t.v4u.apply_v11()
    return t.vb.v3m.lodestar_v3(f"Lodestar {cell}", CELLS[cell], config, "halfway", 3.0, 7.0, alt=alt,
                                wing_mass_kg=mass)


def split(ac, V, kit_on=True):
    """Battery power at V split into thrust-parasite, thrust-induced, propulsion efficiency and avionics."""
    c = ac.rw_cfg
    m = ac.mass - (0 if kit_on else ac.kit_mass)
    q = 0.5 * c["rho"] * V**2
    p_ind = (m * G)**2 / (q * c["S"] * math.pi * c["e"] * c["AR"]) * V          # thrust power, induced
    tab = ac._tab
    p_thrust = np.asarray(ac.power.__func__(_no_tab(ac), V, kit_on), float)     # thrust power (eta 1, no avionics)
    p_batt = np.interp(V, tab[0], tab[1] if kit_on else tab[2])
    eta = p_thrust / (p_batt - t.ms.sz.A.p_avionics_w)
    return p_thrust - p_ind, p_ind, eta


def _no_tab(ac):
    clone = type(ac)(ac.name, ac.v_cruise, ac.v_min, ac.v_max, ac.energy, kit=ac.kit, rw_cfg=ac.rw_cfg,
                     mass=ac.mass, kit_mass=ac.kit_mass)
    return clone


def endurance(ac, energy_frac, v_lo, v_hi, draws=None, kit_on=True):
    """Hours = usable energy / min battery power over [v_lo, v_hi]; draws perturb the model (None = nominal)."""
    V = np.linspace(max(v_lo, ac.v_min), v_hi, 60)
    par, ind, eta = split(ac, V, kit_on)
    p_av = t.ms.sz.A.p_avionics_w
    if draws is None:
        p = (par + ind) / eta + p_av
        i = int(np.argmin(p))
        return ac.energy * energy_frac / p[i], V[i]
    k_par, e_fac, k_eta, k_m, k_cap = (draws[k][:, None] for k in ("k_par", "e_fac", "k_eta", "k_m", "k_cap"))
    # Mass growth raises induced power as m^2; the stall-limited minimum speed rises as sqrt(m).
    p = (par * k_par + ind * k_m**2 / e_fac) / (eta * k_eta) + p_av
    p = np.where(V[None, :] >= ac.v_min * np.sqrt(k_m), p, np.inf)
    return ac.energy * energy_frac * k_cap[:, 0] / p.min(axis=1), V[np.argmin(p, axis=1)]


def draw(n, rng):
    return dict(
        k_cap=np.clip(rng.normal(1.0, 0.03, n), 0.94, 1.06),
        k_par=rng.uniform(0.85, 1.20, n),
        e_fac=rng.uniform(0.70, 0.85, n) / t.ms.sz.A.e_oswald,
        k_eta=rng.uniform(0.95, 1.03, n),
        k_m=1 + rng.triangular(0.0, 0.04, 0.10, n),   # up to the 10% growth allowance (mass_growth.py)
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    mass = lodestar_structure_kg("v1.1")
    rng = np.random.default_rng(SEED)
    d = draw(N_MC, rng)
    rows, mc = [], []
    log = [f"v1 structure mass passed to the power model: {mass:.4f} kg", f"Monte Carlo: {N_MC} draws, seed {SEED}"]
    for cell in CELLS:
        for config, label in (("kit", "rescue kit"), ("search", "battery module (search only)")):
            for case, alt, frac, lo, hi in (("design case", 2000.0, 0.90, 14.0, 20.0),
                                            ("published style", 0.0, 1.0, 0.0, 22.0)):
                ac = build(cell, config, alt, mass)
                nominal, v_best = endurance(ac, frac, lo, hi)
                hours, v_mc = endurance(ac, frac, lo, hi, d)
                p10, p50, p90 = np.percentile(hours, [10, 50, 90])
                rows.append(dict(cell=cell, bay=label, case=case, altitude_m=alt, takeoff_kg=ac.mass,
                                 battery_wh=ac.energy / 0.85, nominal_h=nominal, best_speed_mps=v_best,
                                 p10_h=p10, p50_h=p50, p90_h=p90, p_below_3h=float((hours < 3.0).mean())))
                mc.append(pd.DataFrame(dict(cell=cell, bay=label, case=case, hours=hours, speed_mps=v_mc,
                                            **{k: v for k, v in d.items()})))
    summary = pd.DataFrame(rows)
    summary.to_csv(OUT / "endurance_summary.csv", index=False)
    pd.concat(mc, ignore_index=True).to_csv(OUT / "endurance_mc.csv.gz", index=False)

    # P6 / P7: wind penetration and dash.
    margins = pd.read_csv(ROOT / "out" / "propulsion_margins" / "margins.csv")
    vmax = margins[margins.requirement == "Max level speed, continuous power"].set_index("pack").value
    fleet = [(f"Lodestar {k}", float(vmax[v]), "propulsion_margins worst corner (DA 0 m, -10 C, kit on)")
             for k, v in CELLS.items()]
    fleet += [(c.name, c.v_max, "published-style v_max (mission_sim.COMPETITORS)")
              for c in [*t.ms.COMPETITORS, t.ms.FRONTIER]]
    wind = pd.DataFrame([dict(aircraft=n, v_max_mps=v, basis=b,
                              groundspeed_into_12_mps=v - 12.0,
                              dash_5km_still_min=5000 / v / 60,
                              dash_5km_into_10_min=5000 / (v - 10.0) / 60) for n, v, b in fleet])
    wind.to_csv(OUT / "wind_dash.csv", index=False)

    with pd.option_context("display.width", 200, "display.max_columns", 20):
        log += ["", summary.round(3).to_string(index=False), "", wind.round(2).to_string(index=False)]
    text = "\n".join(log)
    (OUT / "log.txt").write_text(text, encoding="utf-8")
    print(text)

    # Self-checks.
    failed = 0

    def check(name, ok, detail=""):
        nonlocal failed
        failed += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    si = summary.query("cell == 'SiCore' and bay == 'rescue kit' and case == 'design case'").iloc[0]
    ref = pd.read_csv(ROOT / "out" / "wing_trade" / "performance_v1.csv")
    check("nominal design-case endurance within the MC P10-P90 band",
          si.p10_h <= si.nominal_h <= si.p90_h, f"{si.p10_h:.2f} <= {si.nominal_h:.2f} <= {si.p90_h:.2f}")
    import lodestar_v1 as v1
    # The power model's sizing mass and the CAD-based frozen mass come from different bookkeeping; they must
    # agree within 25 g. wing_trade's performance_v1.csv now uses the same model call (apply_v4 + V11 masses).
    check("power-model take-off mass matches the frozen v1 mass within 25 g",
          abs(si.takeoff_kg - v1.MASS["sicore_loaded_kg"]) < 0.025,
          f"{si.takeoff_kg:.4f} kg vs {v1.MASS['sicore_loaded_kg']} kg")
    check("power-model mass matches wing_trade performance_v1.csv within 2 g",
          abs(ref.query("cell.str.startswith('Amprius')").takeoff_mass_kg.iloc[0] - si.takeoff_kg) < 0.002)
    mod = summary.query("cell == 'SiCore' and bay != 'rescue kit' and case == 'design case'").iloc[0]
    check("battery module outlasts the kit configuration", mod.nominal_h > si.nominal_h,
          f"{mod.nominal_h:.2f} h vs {si.nominal_h:.2f} h")
    print(f"\n{failed} failed")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
