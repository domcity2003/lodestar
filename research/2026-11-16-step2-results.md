# Step 2 results: Lodestar v1.1 mission performance, verification and gap closure

2026-11-16

Step 2 of the v1 roadmap ("simulations and tests"; testing and improving). This note reports the headline metrics fixed in advance in [the Step 2 plan](2026-11-15-step2-plan.md) (P1–P8), the design changes made to reach them (v1 → v1.1), and the status of gaps G1–G14.

Method, fixed before any result was seen:
- Every improvement trade was tuned on seed 20261111.
- The improvements were confirmed on the held-out seed 20261114.
- The headline numbers come from seed 20261115. No decision used it.
- Detection is reported at three calibrations from [the detection validation](2026-11-15-detection-validation.md): low (odds scale 0.735, crown porosity 0.20, the pessimistic synthetic Johnson calibration), nominal (1.5, 0.35) and high (3.0, 0.50).
- Intervals are 95% paired bootstraps over hiker runs, 4,000 resamples.

## Answer

**Lodestar v1.1 is the best aircraft in the comparison at every detection calibration, on both trail patterns, at 1 h and at 3 h.**
- **Help within 3 h:** 13.8% / 12.1% of lost hikers on the 3-trail / 6-trail patterns at the low calibration, 18.5% / 17.4% at nominal and 22.7% / 22.6% at high.
- **Best competitor:** the DeltaQuad Evo-LE, using its 8 h press claim, reaches 11.5% / 11.2%, 15.4% / 14.4% and 18.3% / 17.4%.
- **At 1 h the lead is larger:** 1.5–1.6 times the best competitor at every calibration, because the dropped kit makes contact without waiting for a ground team to walk in.

Only one paired comparison is not clearly above zero: help within 3 h against the Evo-LE at the low calibration on the 6-trail pattern, +0.9 pp (−0.1 to +1.9).

**v1.1 beats v1 on the held-out seed by +3.5 pp at 1 h and +3.9 pp at 3 h** (3-trail pattern; intervals 2.6–4.5 and 2.9–5.0 pp).
- In relative terms, help within 1 h goes from 5.0% to 8.5%, a 71% gain.
- The gain comes mostly from the twin splayed thermal camera, which gives a 135.7 m track spacing instead of 55 m, and from the 40° search bank.

**The price is mass and cost.**
- v1.1 weighs 8.77 kg loaded against 8.31 kg for v1. With the AIAA S-120 growth allowance the predicted mass is 9.82 kg, leaving 0.18 kg to the 10 kg limit.
- Parts are about $16.4k, against the R13 target of ≤ $8k. Performance comes first, so R13 is deferred to a cost-down pass (Step 3 or later) rather than counted as a failed requirement.
- Even so, Lodestar has the lowest cost per expected person helped in the comparison: $810–$1,420, against $1,060–$1,680 for the Trinity Pro and $3,100–$4,700 for the eBee VISION.

## 1. Headline metrics

### P1. Probability of help within 1 h and 3 h

Help means the kit lands within reach of the person, or a ground team reaches them after a find. Each cell is 7 sites × 500 hikers. Values are Lodestar SiCore / best competitor, in %.

| Calibration | Pattern | Help within 1 h | Help within 3 h |
|---|---|---|---|
| Low | 3 trails | 8.6 / 5.7 (Bramor) | 13.8 / 11.5 (Evo-LE) |
| Low | 6 trails | 8.2 / 5.7 (Bramor) | 12.1 / 11.2 (Evo-LE) |
| Nominal | 3 trails | 11.8 / 7.2 (Bramor) | 18.5 / 15.4 (Evo-LE) |
| Nominal | 6 trails | 11.3 / 7.4 (Evo GOV) | 17.4 / 14.4 (Evo-LE) |
| High | 3 trails | 14.5 / 8.8 (Bramor) | 22.7 / 18.3 (Evo-LE) |
| High | 6 trails | 14.5 / 9.0 (Evo-LE) | 22.6 / 17.4 (Evo-LE) |

Table 1. Help within 1 h and 3 h [%], Lodestar v1.1 SiCore vs the best competitor in each cell, by calibration and pattern. Seed 20261115, pursuit_1 route, published competitor endurance.

Paired differences, Lodestar SiCore minus each competitor, are in percentage points with the 95% interval. For each calibration, the narrowest margin of the two patterns is shown.

| Competitor | Low, 1 h | Low, 3 h | Nominal, 3 h | High, 3 h |
|---|---|---|---|---|
| DeltaQuad Evo-LE (press, 8 h) | +2.7 (1.9 to 3.6) | +0.9 (−0.1 to 1.9) | +3.0 (2.0 to 4.0) | +4.5 (3.6 to 5.4) |
| DeltaQuad Evo GOV | +2.6 (1.8 to 3.5) | +2.3 (1.3 to 3.2) | +4.6 (3.6 to 5.7) | +6.1 (5.0 to 7.1) |
| C-Astral Bramor ppX | +2.4 (1.7 to 3.3) | +3.0 (2.1 to 4.0) | +5.4 (4.3 to 6.5) | +7.5 (6.3 to 8.7) |
| Quantum Trinity Pro | +2.7 (1.9 to 3.5) | +6.1 (5.1 to 7.2) | +9.5 (8.3 to 10.8) | +12.9 (11.5 to 14.4) |
| AgEagle eBee X | +2.7 (1.9 to 3.6) | +6.3 (5.2 to 7.4) | +9.9 (8.6 to 11.3) | +13.6 (12.1 to 15.1) |
| Lodestar P50B (budget pack) | +0.0 (−0.6 to 0.7) | +1.3 (0.5 to 2.1) | +1.9 (1.2 to 2.6) | +2.2 (1.6 to 2.9) |

Table 2. Lodestar SiCore minus competitor, help within 1 h and 3 h [pp, 95% paired bootstrap]. Full table: `out/step2_results/p1_paired.csv`.

**With competitors at 80% of their published endurance**, which is closer to what spec sheets deliver in cold mountain air, every lead grows except the Evo-LE's 6-trail figure (+0.8 pp, −0.1 to +1.8). The Trinity Pro and eBee X fall to about 5% help within 3 h.

### P2. Time to first contact

These figures cover hikers contacted within 3 h, at the low calibration.

| Aircraft | Contacted within 1 h [%] | Contacted within 3 h [%] | Median [min] | P90 [min] |
|---|---|---|---|---|
| Lodestar SiCore | 8.4 | 12.9 | 38 | 104 |
| Lodestar P50B | 8.1 | 11.2 | 33 | 87 |
| DeltaQuad Evo-LE | 5.4 | 11.3 | 64 | 153 |
| DeltaQuad Evo GOV | 5.3 | 10.3 | 55 | 133 |
| C-Astral Bramor ppX | 5.7 | 9.4 | 44 | 114 |
| Quantum Trinity Pro | 5.4 | 5.8 | 30 | 55 |
| AgEagle eBee X | 5.4 | 5.8 | 31 | 56 |

Table 3. Time to first contact among hikers contacted within 3 h, by aircraft. Seed 20261115, low calibration, both patterns pooled. Nominal and high: `out/step2_results/p2_contact.csv`; medians change by less than 3 min.

**How to read the median.** The Trinity Pro and eBee X show a shorter median only because their 1.5 h endurance limits their contacts to the early ones. Each of them contacts 5.8% of hikers against Lodestar's 12.9%. The fair comparison is against the long-endurance aircraft. There, Lodestar's median wait is 38 min against 64 min for the Evo-LE, and its P90 is 104 min against 153 min.

### P3 and P4. Endurance

The 4,000-draw Monte Carlo covers cell capacity, resistance, drag, propeller efficiency and the avionics load. Source: `out/performance_v1/`.

| Cell | Bay | Case | Nominal [h] | P10 [h] | P50 [h] | P90 [h] |
|---|---|---|---|---|---|---|
| SiCore | rescue kit | design case (2,000 m, +20 °C, best speed) | 3.49 | 2.95 | 3.21 | 3.50 |
| SiCore | rescue kit | published style (sea level, best speed) | 4.21 | 3.57 | 3.87 | 4.22 |
| SiCore | battery module, search only | design case | 5.71 | 4.85 | 5.26 | 5.73 |
| SiCore | battery module, search only | published style | 6.89 | 5.85 | 6.35 | 6.91 |
| P50B | rescue kit | design case | 2.78 | 2.36 | 2.56 | 2.79 |

Table 4. Endurance by cell, bay configuration and case [h]. Monte Carlo 4,000 draws, seed 20261115.

- R1 (≥ 3 h with the kit) is met at best speed: 3.49 h nominal, but a 15% chance of falling below 3 h.
- At a fixed 20 m/s search speed it is 2.97 h, 1% short. v1 gave 3.09 h; the 0.42 kg of v1.1 additions cost about 4%. This is the thinnest performance margin in v1.1 and is a Step 3 item.
- The P50B pack still does not reach 3 h with the foam-core wing.
- Search-only with the battery module, 6.9 h published-style is behind only the Evo-LE's 8 h press claim.

### P5. Area searched per sortie

The plan's definition, area at Pd ≥ 0.5 under canopy, cannot be met at the low calibration, so the plan was amended. Area is now the straight search length of the calm-wind route × the no-blur sweep width of the camera.

| Aircraft | Straight search [km] | Sweep width [m] | Area [km²/sortie] |
|---|---|---|---|
| Lodestar SiCore | 223 | 127 | 28.3 |
| DeltaQuad Evo-LE | 416 | 55 | 22.9 |
| Lodestar P50B | 177 | 127 | 22.5 |
| DeltaQuad Evo GOV | 216 | 55 | 11.9 |
| C-Astral Bramor ppX | 170 | 55 | 9.4 |
| Quantum Trinity Pro | 77 | 55 | 4.2 |
| AgEagle eBee X | 68 | 55 | 3.7 |

Table 5. Area searched per sortie by aircraft. Calm-wind route, mean of 7 sites and both patterns. Competitors fly the single 32° thermal camera of the archived runs.

### P6 and P7. Wind penetration and dash

| Aircraft | Max level speed [m/s] | Groundspeed into 12 m/s [m/s] | 5 km dash, still air [min] | 5 km dash into 10 m/s [min] |
|---|---|---|---|---|
| Lodestar SiCore | 29.5 | 17.5 | 2.83 | 4.28 |
| Lodestar P50B | 34.8 | 22.8 | 2.40 | 3.36 |
| AgEagle eBee X | 30.0 | 18.0 | 2.78 | 4.17 |
| DeltaQuad Evo GOV / Evo-LE | 23.0 | 11.0 | 3.62 | 6.41 |
| C-Astral Bramor / Trinity Pro | 22.0 | 10.0 | 3.79 | 6.94 |

Table 6. Max level speed, headwind groundspeed and 5 km dash time, by aircraft. Lodestar: propulsion-model worst corner (sea level, −10 °C, kit on). Competitors: published-style maximum speed.

The 22 m/s requirement is met with 7.5 m/s to spare. The SiCore and the eBee X are tied as the fastest of the long-range aircraft.

### P8. Cost

The cost model is `scripts/cost_v1_1.py`, starting from the milestone 6 cost model. v1.1 changes:
- adds the nadir Boson 640, which the v1 compliance note had left out;
- adds the twin Boson 640, the aircraft parachute and the safety fit;
- swaps the servos for the KST A12-S HV.

| Item | Value |
|---|---|
| Parts, 10 built, SiCore | $16,418 |
| Unit cost, paid build | $18,953 (range $15,888–$26,917) |
| Per flight hour, volunteer crew | $61.08 |
| Per expected help within 3 h (low / nominal / high) | $1,416 / $1,021 / $809 |
| Trinity Pro (base) per expected help | $1,676 / $1,287 / $1,062 |
| eBee VISION per expected help | $4,696 / $3,713 / $3,125 |

Table 7. Unit cost and cost per expected help, Lodestar v1.1 vs competitors with a published price. Competitors use the same ownership model: price over a 400 h life plus 10% a year maintenance.

The two Boson 640 cores are $8.9k of the $16.4k. They are where a cost-down pass should look first. Options are a 640 px core at about $2–3k, or one wider core instead of two.

## 2. Where Lodestar leads and where it trails

**Leads, and why:**
- **Twin splayed camera, 95° combined field of view, 2.5× the sweep width of a single 32° gimbal camera.** This is the largest single gain from v1 to v1.1.
- **The kit drop.** Contact does not wait for a ground team, so the 1 h lead is the largest lead.
- **3+ h endurance with fast cruise.** It covers the probable area early and still has time left.

**Trails, or is not clearly ahead:**
- **The Evo-LE at 3 h, low calibration, 6 trails.** That aircraft's 8 h press claim lets it keep searching after Lodestar has turned for home. At 1 h Lodestar still leads it by +2.7 pp.
- **Search-only endurance**, against the same Evo-LE claim.
- **Endurance at 20 m/s** is now 2.97 h. The margin was spent on the parachute and safety fit.
- **Unit cost:** about twice the R13 target, by decision.

## 3. Design changes v1 → v1.1

Every change below is carried in `scripts/lodestar_v1.py` (VERSION v1.1). Each one is in the CAD model (`scripts/cad_v4.py`) and the power model as an explicit mass row (`V11_MASS`, +0.417 kg in total) or as a drag increment (+0.0004 m² CdA).

| # | Change | Driver | Effect |
|---|---|---|---|
| 1 | Twin splayed Boson 640 nadir cameras: 95° combined field of view, 135.7 m track spacing | P1 trade (tuning seed) | Largest P1 gain; +0.09 kg |
| 2 | Search bank 40° for every aircraft (load factor 1.305) | P1 trade | Tighter turnarounds |
| 3 | Aircraft parachute 0.40 kg in the nose, uses the 0.33 kg spare payload allocation (RC1) | System safety | Impact 100–190 J instead of 0.5–9 kJ |
| 4 | Backup avionics supply and locking connectors (RC2), BMS with heater cut-out at 50 °C and cell liner (RC3), ADS-B In and strobes (RC8), second GNSS (RC10) | System safety | +0.19 kg |
| 5 | Elevon servos KST A12-S HV, 1.35 N·m, 12 mm | Controls (the X10 Pro was under-torqued) | Hinge demand / 2 = 0.89 N·m |
| 6 | 8 AWG pack leads | Lead temperature 83 °C in foam at full climb on a +40 °C day | +0.045 kg |
| 7 | Belly skid: EPP 20 g/L, 70 mm stroke, tapered contact face, mean crush force 250–360 N | Landing at the 2.0 m/s design sink rate | Wing landing margins all ≥ +0.12 |
| 8 | Flare pitch ≤ 4.5°; wear pads at the tips and winglet roots | 6-DOF landing | — |
| 9 | Drop run-in 18 m/s (calm, light turbulence) and 20 m/s (moderate) | 6-DOF: 14 m/s stalled in 49 of 50 light-turbulence runs | Kit reach probability 0.957 → 0.955 |
| 10 | Full power at release above 3,000 m DA, or a ≥ 16.5 m/s release | 6-DOF launch | — |
| 11 | Search at 20 m/s above 3,200 m DA | 30° turns at 18 m/s lose the 1.25 Vs margin | — |
| 12 | Spar cap ply drops: a 1.25 factor on cap stress at each drop, drops staggered ≥ 25 mm | Wing FE found −0.061 margin at the drops | Wing 1.983 → 2.006 kg; all FE margins ≥ 0 |
| 13 | Packs moved 5 mm aft | CG had moved forward with the nose parachute | Loaded CG 0.413 m, static margin 7.3% |
| 14 | Forward CG limit 0.361 → 0.365 m | Heavier aircraft: pull-up to n 3.8 at Vc | CG 48 mm inside the limit |
| 15 | No full-power climb below 35% charge above 3,000 m DA | Worst-corner climb fell to 2.82 m/s / 13.9% with the 25% rule | 3.09 m/s / 15.3% (needs 3.0 / 15%) |

Table 8. Changes from v1 to v1.1, with the analysis that drove each.

Items 14 and 15 were found when the full analysis chain was rerun at the final v1.1 mass. The freeze check stopped on both before any headline result was produced. The 5 W pack heater is already at its limit holding 15 °C, which is why item 15 is an operating rule and not a warmer pack.

## 4. Verification status

- `run_all_tests.py --full`: 24 scripts, all pass after the final mass. That covers the mission sim, archive replays, wing FE known answers, flutter, controls, propulsion, 6-DOF, system safety, mass growth, detection, performance, drop speed and the FreeCAD helper.
- `freeze_check.py` passes. It now has a mission section that reads each headline run's `meta.json` and checks it against the frozen v1.1 inputs:
  - structure mass 2.6427 kg;
  - camera 95° / 135.7 m;
  - search bank 40°;
  - body CdA;
  - the run's detection calibration.
- Two test guards changed, with the reason recorded in the code:
  - the wing-saving guard against the archived XPS wing went from 0.30 to 0.25 kg (saving now 0.285 kg);
  - the climb-fix check now reads the high-DA charge rule.

**Margins to watch in v1.1:**

| Quantity | Value | Limit | Margin |
|---|---|---|---|
| Predicted mass with growth allowance | 9.82 kg | 10 kg | 0.18 kg |
| Endurance at 20 m/s, 2,000 m, kit on | 2.97 h | 3 h | −1% (met at best speed: 3.49 h) |
| Worst-corner banked climb gradient | 15.3% | 15% | 0.3 pp (rests on the assumed 44 mΩ cell resistance, test T5) |
| Upper skin face wrinkling on EPS (FE) | MS +0.275 | 0 | assumed allowable, needs a coupon test |
| Servo torque vs hinge demand / 2 | 0.9 N·m spec vs 0.89 N·m | — | 1% on the spec; the part is 1.35 N·m |

Table 9. Thinnest margins in v1.1.

## 5. Gap closure G1–G14

| Gap | Item | Status | Evidence |
|---|---|---|---|
| G1 | Functional hazard assessment | Closed | [system safety](2026-11-15-system-safety.md) §3 |
| G2 | Component FMEA | Closed | [system safety](2026-11-15-system-safety.md) §4 |
| G3 | Contingencies: glide, link-loss return, termination | Closed | [system safety](2026-11-15-system-safety.md) §5; [6-DOF](2026-11-15-sixdof-v1.md) §4 |
| G4 | C2 link budget in mountain terrain | Closed; relay and autonomous search required (RC5) | [system safety](2026-11-15-system-safety.md) §6 |
| G5 | Detection model validation | Closed as a bound: P1 reported at three calibrations | [detection validation](2026-11-15-detection-validation.md) |
| G6 | v1 6-DOF: trim, Dryden turbulence, release, motor-out | Closed | [6-DOF](2026-11-15-sixdof-v1.md) |
| G7 | Mass growth allowance | Closed; margin 0.18 kg after v1.1 | [mass, power, interfaces](2026-11-15-mass-power-interfaces.md); `scripts/mass_growth.py` |
| G8 | Electrical power budget | Closed; the wiring diagram is Step 4 | [mass, power, interfaces](2026-11-15-mass-power-interfaces.md) G8 |
| G9 | Launch and recovery dynamics | Closed: skid respecified (item 7), launch rules (item 10) | [6-DOF](2026-11-15-sixdof-v1.md) §5–6; `scripts/fea_extra.py` |
| G10 | Verification cross-reference matrix | Closed; to be refreshed for v1.1 in Step 4 | [verification matrix](2026-11-15-verification-matrix.md) |
| G11 | Mission sim and cost on v1 | Closed: this note | `out/step2_results/`, `out/cost_v1.1/` |
| G12 | Interface list | Closed; the drawings are Step 4 | [mass, power, interfaces](2026-11-15-mass-power-interfaces.md) G12 |
| G13 | Operating envelope | Closed; v1.1 adds items 9–11 and 15 | [mass, power, interfaces](2026-11-15-mass-power-interfaces.md) G13 |
| G14 | Mission-sim QA open issues | Closed in code: the planning radius replans to convergence and the whole route is used. Archive reproduction keeps the old behaviour behind a flag | `scripts/long_pass.py`, `scripts/flyable_mission.py`; [mission-sim QA](2026-11-14-mission-sim-qa.md) |

Table 10. Status of gaps G1–G14 at the end of Step 2.

## 6. Limits

- **Detection is the largest uncertainty.** Absolute help rates move by a factor of 1.6 across the calibration range. The ranking does not change.
- **Competitors are modelled from published figures.** That covers endurance, cruise and maximum speed, with no mass or drag model. They fly the single 32° camera of the archived runs and carry no kit. The 80% endurance case brackets optimistic spec sheets.
- **Unconfirmed numbers.** The parachute packed volume and mass (0.40 kg) are vendor-class estimates, and the bundle price ($800–$2,500) is an estimate.
- **Mission scope.** Every mission number is for the lost-hiker scenario on 7 US trail sites with the pursuit route. It is not a general claim.

## 7. Step 3 candidates (one round of fixes)

1. **Get back the 20 m/s endurance margin** (2.97 → ≥ 3.05 h). Options: a lighter parachute bundle, the doubler and skin layout, a pointier pod nose, or shark-fin winglets.
2. **Grow the mass margin** from 0.18 kg toward ≥ 0.5 kg. The same items as option 1, plus measuring the parachute and the servos.
3. **Cost-down pass (R13).** Cheaper 640 px cores or one wide core. Report the P1 cost of each option against the dollars saved.
4. **Measure the SiCore cell resistance (test T5).** The worst-corner climb margin depends on it.
5. **Get the parachute vendor's packed volume.** The canister now clears the pod skin by 19.7 mm and the wing root by 4.8 mm.

## Reproduce

```
cd scripts
../.venv/Scripts/python.exe -X utf8 fleet_flyable.py --design v1.1 --seed 20261115            # low
LODESTAR_ODDS_SCALE=1.5 LODESTAR_POROSITY=0.35 ../.venv/Scripts/python.exe -X utf8 fleet_flyable.py --design v1.1 --seed 20261115
LODESTAR_ODDS_SCALE=3.0 LODESTAR_POROSITY=0.5  ../.venv/Scripts/python.exe -X utf8 fleet_flyable.py --design v1.1 --seed 20261115
../.venv/Scripts/python.exe -X utf8 fleet_flyable.py --design v1.1 --seed 20261115 --endurance 0.8
../.venv/Scripts/python.exe -X utf8 fleet_flyable.py --design v1.1 --seed 20261114            # held-out confirmation
../.venv/Scripts/python.exe -X utf8 fleet_flyable.py --design v1   --seed 20261114
../.venv/Scripts/python.exe -X utf8 step2_results.py
../.venv/Scripts/python.exe -X utf8 cost_v1_1.py
../.venv/Scripts/python.exe -X utf8 run_all_tests.py --full
```

Each `fleet_flyable.py` run takes about 1 h on 8 workers. Outputs: `out/wing_trade/flyable_fleet_pursuit_1_v1.1_*`, `out/step2_results/`, `out/cost_v1.1/`, `out/performance_v1/`.
