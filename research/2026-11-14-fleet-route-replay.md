# Fleet comparison on continuous, wind-adaptive routes

2026-11-14

## Purpose

The continuous-route replay (`2026-11-13-continuous-route-replay.md`) moved Lodestar onto flyable, wind-adaptive routes, but the competitors still used the sharp-corner model. This replay puts all seven aircraft on the same route model, so the comparison is like for like.

## Method

`scripts/fleet_flyable.py` runs every aircraft through `flyable_mission.py`'s route builder and replay. Each aircraft flies the trails-first pattern at its own camera's track spacing. Fillets and turnarounds are sized to its own route airspeed and the scenario's wind band (2, 4, 6, 8 or 10 m/s). The sharp-corner replay is kept as a control.

Lodestar is the compact-XPS build. The competitors keep the published-endurance power curve from `mission_sim.py`. Three additions let them fly the same route model:

- **Turn power.** Induced power is half of cruise power at cruise speed and scales as 1/V. Bank multiplies it by 1 + tan²φ, the same rule Lodestar uses.
- **Climb energy.** Competitors use Lodestar's ratio of climb energy to cruise power (`terrain_sim.climb_k`), unchanged from the archived run.
- **Turn speed floor.** The published-style minimum speed (0.8 × cruise) is treated as a sea-level value. It is raised for density and the 30° bank limit, the same convention as Lodestar's 1.25 × stall minimum.

Competitors carry no rescue kit. The edits to `flyable_mission.py` do not change Lodestar's path.

**Verification** (`scripts/verify_fleet_flyable.py`):
- The sharp-corner competitor results reproduce the archived terrain run, run for run, at full endurance and at 80% endurance.
- Lodestar's results reproduce the compact-XPS flyable replay run for run.
- Each run has 98,000 rows: 7 sites × 2 patterns × 7 aircraft × 500 hikers × 2 route modes.
- The 33 terrain checks and 8 route checks pass.

## Results

**Table 1.** Modeled probability of help within 1 h / 3 h [%] by aircraft and trails-first radius, continuous wind-adaptive routes, seven-site mean.

| Aircraft | Trails 3 km | Trails 6 km | Trails 6 km, competitors at 80% endurance |
|---|---:|---:|---:|
| Lodestar SiCore (compact XPS) | 2.88 / 6.43 | 3.28 / 5.40 | 3.28 / 5.40 |
| Lodestar P50B (compact XPS) | 2.91 / 5.23 | 3.28 / 4.74 | 3.28 / 4.74 |
| DeltaQuad Evo GOV | 2.80 / 7.31 | 3.09 / 6.11 | 3.09 / 5.26 |
| DeltaQuad Evo-LE (press-reported) | 2.74 / 7.97 | 3.09 / 6.77 | 3.09 / 6.77 |
| C-Astral Bramor ppX | 2.80 / 6.06 | 3.17 / 5.54 | 3.17 / 4.74 |
| Quantum Trinity Pro | 2.57 / 2.71 | 2.83 / 2.94 | 2.31 / 2.31 |
| AgEagle eBee X | 2.83 / 3.14 | 3.14 / 3.49 | 2.71 / 2.71 |

Conditions: NLCD 2021 canopy, crown porosity 0.20, 500 matched hikers and winds per site, seven US mountain sites, 7 m/s kit chute for Lodestar only.

**Table 2.** Paired difference in 3-hour help, aircraft minus Lodestar SiCore [percentage points], with 95% interval, continuous wind-adaptive routes.

| Aircraft | Trails 3 km | Trails 6 km | Trails 6 km, competitors at 80% |
|---|---:|---:|---:|
| DeltaQuad Evo GOV | +0.89 (0.20 to 1.57) | +0.71 (0.17 to 1.26) | −0.14 (−0.63 to 0.37) |
| DeltaQuad Evo-LE (press) | +1.54 (0.86 to 2.26) | +1.37 (0.77 to 2.00) | +1.37 (0.77 to 2.00) |
| C-Astral Bramor ppX | −0.37 (−0.97 to 0.23) | +0.14 (−0.43 to 0.69) | −0.66 (−1.20 to −0.14) |

Intervals resample hiker-run IDs across the seven fixed sites (4,000 draws).

## Findings

1. **Rounded turns cost every aircraft, not just Lodestar.** Each fleet member loses about half its 3-hour score against the sharp-corner control, so the earlier 11% figures were optimistic for all of them.
2. **On equal routes, the Evo GOV leads compact-XPS SiCore by 0.7–0.9 points at 3 hours**, and the interval excludes zero. Under the sharp model the gap was 0.35–0.40 points (compact XPS). Lodestar flies 18.7 m/s, faster than the Evo GOV's 17.3 m/s, so its turns are wider (109 m versus 97 m at Bear Lake). That costs it more camera-active distance.
3. **At one hour, every aircraft except the Quantum Trinity Pro is within 0.2 points of SiCore**, and none of those gaps is resolved. Lodestar leads the Trinity by 0.3–0.5 points at one hour (the 6 km interval excludes zero). Early help is set by how far the aircraft reaches in the first hour and by Lodestar's kit, not by endurance.
4. **If competitors deliver 80% of their published endurance, SiCore and the Evo GOV are tied** (−0.14 points, interval spans zero). The Evo-LE does not change, because its 8 h battery outlasts the mapped route. Its lead rests on a press-reported endurance figure.
5. **The original-mass SiCore result is not rerun here.** The heavy compact-XPS wing is the baseline.

## Implications

- Lodestar's case at equal routes is now "competitive at one hour, slightly behind the Evo GOV at three hours, at roughly a fifth to a half of the price." It is no longer "ahead."
- The largest lever is route shape. Longer camera passes with fewer turns (STATUS item 1) help the faster aircraft most, so they should narrow the gap. Run that experiment fleet-wide.
- Lightening the wing raises the 3-hour score through endurance, but endurance is not the binding limit at one hour.

## Limits

- Competitor turn and climb power are scaled from Lodestar's rules, because no drag polars are published.
- The competitors' minimum speeds are assumed.
- The remaining limits in the continuous-route note (steady wind, no roll-rate or gust model, synthetic detection calibration) apply to every aircraft.

Outputs: `out/wing_trade/flyable_fleet/` and `out/wing_trade/flyable_fleet_endurance_0.8/` (`summary.csv`, `by_site.csv`, `paired_vs_sicore.csv`, `routes.csv`, `runs.csv.gz`).
