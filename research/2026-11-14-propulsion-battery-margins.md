# Propulsion and battery margins by altitude, temperature and pack

2026-11-14

## Answer first

**The motor, prop and ESC have enough power everywhere in the envelope.** The weak link is the SiCore pack, which can't deliver climb power when it is cold and nearly empty.

- **Max level speed passes with a wide margin.** At the worst corner the aircraft makes 29.5 m/s (SiCore) or 35.0 m/s (P50B) on continuous power, against the 22 m/s requirement. Into a 12 m/s headwind, ground speed stays at 17.5 m/s or more.
- **SiCore fails climb at the end of the usable charge in the cold.** At 4,400 m density altitude and −10 °C, with the kit on, the cells hit the loaded-voltage floor at 1.74 m/s, against 3 m/s required. The 30° banked climb gradient is 8.4% against the 15% the route planner assumes. The pack can't supply the 585 W a 3 m/s climb needs without sagging to 2.29 V per cell, below the 2.5 V cutoff. P50B passes both everywhere: 3.76 m/s and 19.6% at its worst.
- **Five other items fail or have no margin:**
  1. **Full throttle on a fresh pack overloads the motor.** It would draw 66 A (SiCore) or 86 A (P50B), against the motor's 50 A / 180 s rating and the 60 A ESC. **A 45 A current limit is mandatory.** Every requirement fits under it: the 3 m/s climb needs 40.6 A at worst.
  2. **A 14 m/s bungee release at 4,400 m is right at stall (1.00 Vs).** It needs 15.4 m/s for a 1.1 Vs margin.
  3. **Endurance (best speed, kit carried):** SiCore pre-warmed passes everywhere, 3.22 h worst. Cold-soaked at 4,400 m and −10 °C it gives 2.87 h. P50B meets 3 h only from 0 °C to +40 °C at or below 2,000 m (worst pre-warmed 2.57 h, cold-soaked 2.29 h).
  4. **A 10 km return into a 12 m/s headwind needs 51 Wh.** That's 2.2× the still-air return. P50B's 10% reserve (45.9 Wh) doesn't cover it.
  5. **The modelled winding temperature reaches 96 °C** in a 3 m/s climb at +40 °C. That's under the assumed 120 °C limit, but the thermal resistance is assumed.
- **Physical fixes:**
  - Keep the SiCore cells at about 15 °C or warmer: 20 mm of foam around the packs and a ~5 W thermostat pad.
  - Forbid full-power climbs below 25% charge, using a wind-aware RTH energy trigger. With both, the worst-case climb is 3.21 m/s and the banked gradient 16.5%, so both pass.
  - Set a 45 A ESC current limit (no prop change; a 15x8 would cost about 4% cruise power).
  - Use a launcher rated for at least 16 m/s release above 3,000 m density altitude.

Code: `scripts/propulsion_margins.py` (writes `out/propulsion_margins/`), checks in `scripts/test_propulsion_margins.py` (28 known-answer checks, all pass). Baseline (v1 working baseline, 2026-11-14): compact_eps20_light wing 1.881 kg, SiCore 8.03 kg / P50B 7.99 kg loaded, kit release 1.624 kg, "halfway" skin finish, CAM 15x10 + T-Motor AT3520-550, 6S5P.

## 1. Method

- **Envelope.** Density altitude 0, 2,000 and 4,400 m (air density), each at ambient −10, 0, +20 and +40 °C. The actual temperature sets viscosity, speed of sound, cell and motor temperatures. A hot-and-high case is added: 3,000 m pressure altitude at ISA+20 (15.5 °C) is 0.845 kg/m³, or 3,690 m density altitude. The grid is a bounding set; some combinations are not real days.
- **Power required.** The v3 drag model: NeuralFoil MH45, induced drag with e = 0.80, and the pod. Load factor 1/cos 30° in the banked climb.
- **Power available.** The same chain run at full throttle on an RPM grid, limited by whichever of these comes first:
  - motor voltage ≤ 0.80 × bus voltage;
  - motor current ≤ 50 A and power ≤ 1,100 W;
  - ESC current ≤ 60 A;
  - cell current ≤ datasheet continuous (pulse for go-around);
  - loaded cell voltage ≥ 2.8 V;
  - prop speed ≤ 9,000 rpm and tip Mach ≤ 0.6;
  - steady winding temperature ≤ 120 °C on continuous cases.
- **The 0.80 full-throttle factor** is fitted to T-Motor's own 100%-throttle bench rows. The ideal Kv model over-predicts full-throttle RPM by about 25%; the fitted model is within 2% of both bench rows.
- **Prop.** UIUC wind-tunnel CT and CP for the Aeronaut CAM 15x10, as in v3. Below the lowest tested advance ratio, the model blends to the [UIUC static run](https://m-selig.ae.illinois.edu/props/volume-3/data/ancf_15x10_static_0982od.txt), now saved in `data/uiuc_props/`. The 5% pusher installation loss is applied to power.
- **Battery.** 6S5P. Open-circuit voltage vs charge is a generic NMC curve shifted to each cell's nominal voltage. Resistance:
  - P50B: DC resistance from its datasheet.
  - SiCore: 2× the datasheet AC resistance; P50B's own DC/AC ratio is 1.97.
  - Temperature factor exp(2500 K × (1/T − 1/298 K)): ×2.2 at 0 °C, ×3.1 at −10 °C.
  - ×1.3 at 15% charge.
  - Pack self-heating flows out through 10 mm of EPS (k 0.035 W/m·K, 0.2 m²).
- **Worst-case state.** Power and climb checks use the end of the usable 85% (15% charge) at the cell temperature reached in flight, i.e. self-heated from ambient. Launch uses a full pack at ambient (cold-soaked). Endurance reuses the [cold-battery note](2026-11-12-cold-batteries.md) factors: 0.90 pre-warmed / 0.80 cold-soaked at −10 °C, 0.95 / 0.90 at 0 °C, 1.0 at +20 °C and above. The 10% reserve is held back from the usable energy.

Known-answer checks (`test_propulsion_margins.py`):
- ISA densities;
- the hot-and-high density altitude;
- T-Motor bench RPM within 3%;
- UIUC static coefficients exactly;
- CT/CP identical to `v3_propulsion.coeffs`;
- battery power at 20 m/s within 0.1% of the v3 chain (164.9 vs 165.1 W);
- the brief's 3.15 h design-case endurance (3.157 h);
- zero climb at max level speed;
- the brief's 12.32 m/s stall at 2,000 m.

## 2. Power required vs available

Table 1. Climb and speed capability vs. envelope corner, by pack and kit state (kit on unless noted).

*End of usable charge (15%), cells at in-flight temperature. Continuous limits; go-around uses 30 s limits. Search climb at 18 m/s. Banked climb at max(18 m/s, 1.25 × banked stall). Go-around at 1.3 Vs.*

| Pack | Corner | Stall Vs [m/s] | V max [m/s] | Climb at 18 m/s [m/s] | 30° banked climb gradient | Go-around gradient |
|---|---|---:|---:|---:|---:|---:|
| SiCore | 0 m, +20 °C | 11.18 | 33.0 | 3.93 | 22% | 27% |
| SiCore | 2,000 m, +20 °C | 12.34 | 33.9 | 3.56 | 19% | 22% |
| SiCore | 0 m, −10 °C | 11.18 | **29.5** | **1.93** | **10%** | 13% |
| SiCore | 4,400 m, 0 °C | 13.97 | 33.0 | **2.44** | **12%** | 14% |
| SiCore | 4,400 m, −10 °C | 13.97 | 32.0 | **1.74** | **8.4%** | 9.6% |
| SiCore | 4,400 m, −10 °C, kit off | 12.48 | 32.3 | **2.42** | **13%** | 15% |
| SiCore | hot-and-high | 13.46 | 34.1 | 3.09 | 16% | 18% |
| P50B | 2,000 m, +20 °C | 12.32 | 37.2 | 4.51 | 25% | 28% |
| P50B | 4,400 m, −10 °C | 13.94 | 36.8 | 3.76 | 20% | 21% |
| P50B | hot-and-high | 13.43 | 37.7 | 4.43 | 24% | 26% |

- The SiCore climb shortfall comes entirely from cell sag. At 4,400 m, +20 °C it climbs 2.99 m/s, effectively on the line. On cold corners the binding limit is the 2.8 V loaded floor.
- P50B is limited by the throttle (motor voltage) or the 50 A motor current, never by the cells.
- Figure 1 (`out/propulsion_margins/power_curves.png`) shows thrust power required and available for the SiCore pack. At 4,400 m, −10 °C, with cells at 0 °C and 15% charge, the continuous thrust power available is flat at about 200 W. At sea level and +20 °C it is about 390 W.

Table 2. Bungee launch climb-out vs. density altitude, kit on, 14 m/s release.

*Full pack, cold-soaked at ambient, 30 s limits. Level acceleration to 1.25 Vs, then climb to 50 m AGL.*

| Corner | Release / Vs | Release needed for 1.1 Vs [m/s] | Time to 50 m [s], SiCore / P50B | Climb gradient at 1.25 Vs | Static thrust [N] (T/W) |
|---|---:|---:|---:|---:|---:|
| 0 m, −10 °C | 1.25 | 12.3 | 12.7 / 12.7 | 0.23 | 32.9 (0.42) |
| 2,000 m, +20 °C | 1.13 | 13.6 | 12.1 / 12.1 | 0.24 | 33.4 (0.42) |
| hot-and-high | 1.04 | 14.8 | 11.7 / 11.6 | 0.26 | 33.4 (0.42) |
| 4,400 m, −10 °C | **1.00** | **15.4** | 15.7 / 11.5 | 0.20 | 26.8 (0.34) SiCore |

- Once the aircraft is flying, the climb-out is never thrust-limited: every case reaches 50 m in under 16 s and holds a climb gradient of 20% or more.
- The problem is the release speed. At 14 m/s the aircraft leaves the bungee with no stall margin above about 3,000 m density altitude.
- Static thrust of 33 N (0.42 of weight) is ample for a bungee launch and marginal for a hand launch, which isn't a requirement.

## 3. Motor, prop and ESC

**Published data check.** [T-Motor AT3520 KV550](https://store.tmotor.com/product/at3520-long-shaft-fixed-wing-motor.html):
- Ratings and constants: Kv 550, 31 mΩ, 1.1 A idle at 10 V, 218 g, 4–6S, 50 A and 1,100 W for 180 s. These match the v3 inputs. No continuous rating is published.
- T-Motor's matching prop is a 13x6.5, and its bench data stops at a 14x7: 44.3 A, 8,762 rpm, 4.1 kg thrust at 6S.
- The 15x10 is above the maker's tested load. Uncapped, on a fresh pack at sea level and +20 °C, it would draw:
  - SiCore: 66 A / 1,070 W at 7,490 rpm;
  - P50B: 86 A / 1,580 W at 8,410 rpm.
- **That fails the motor and the ESC, so a current limit is required.** All the power-available figures in this note assume the 50 A cap. A 45 A setting still covers every requirement (Table 3).

Table 3. Motor, prop and ESC operating point vs. requirement, worst corner over the envelope.

*End of usable charge. Winding temperature is steady state with 0.7 K/W winding-to-air (ASSUMED).*

| Case | Motor current [A] | Steady winding [°C] | RPM | Tip Mach | Prop efficiency |
|---|---:|---:|---:|---:|---:|
| Cruise at search speed | 12–14 | 30–60 | 4,250–4,920 | 0.24–0.30 | 0.77–0.80 |
| 22 m/s level | 18.0 | 55 (+40 °C) | 5,235 | 0.31 | 0.76 |
| 3 m/s climb at 18 m/s | **40.6** (0 m, +40 °C) | **96** (0 m, +40 °C) | 6,960 | 0.42 | 0.66–0.69 |
| 15% climb in 30° bank | 39.2 | 92 | 6,950 | 0.42 | 0.66–0.69 |
| Full power, any case | 50 (cap) | 128 short-term (go-around) | **8,355** | **0.495** | 0.61 (go-around) |

**Margins against each limit:**
- Motor current: 19% to the 50 A rating at the worst requirement point.
- ESC: 32% to 60 A.
- Prop RPM: 7% to 9,000 rpm. That figure is Aeronaut's rating for the CAM C 15x8 blade ([retailer listing](https://www.modelflight.com.au/aeronaut-cam-c-folding-blades-15x8-in-pair.html), seen in a search snippet; the page refused automated access). I assume it for the 15x10 too.
- Tip Mach: 0.495 at most, against 0.6.

**Prop efficiency.** It rises slightly with altitude, from 0.766 at sea level to 0.798 at 4,400 m (P50B cruise), because the prop runs at higher RPM nearer its best advance ratio.

**Data limits.** UIUC measured the 15x10 only up to 5,016 rpm in the tunnel and 5,947 rpm static. Climb (about 7,000 rpm) and full power (up to 8,355 rpm) reuse the 5,000 rpm curve. That is slightly conservative: static CT rises 1.3% from 4,940 to 5,947 rpm.

**Climb duration.** A 3 m/s climb to 500 m takes 170 s, inside the 180 s rating.

## 4. Batteries

**Datasheets:**
- [Molicel P50B](https://www.fsaeonline.com/content/Molicel-TDS--INR-21700-P50B-80122.pdf):
  - capacity 5.0 Ah typical, 3.6 V, 18.0 Wh;
  - 60 A continuous;
  - AC resistance 6.5 mΩ at 30% charge, DC 12.8 mΩ at 50% charge;
  - discharge −40 to 60 °C, 2.5 V cutoff, 71 g.
- [Amprius catalog](https://amprius.com/documents/Amprius_Product_Catalog.pdf), SA112, the SiCore 21700 that matches the "6.3 Ah" cell:
  - capacity 6.5 Ah typical, 6.35 Ah minimum, 3.46 V nominal;
  - energy 22.5 Wh typical, 21.97 Wh minimum;
  - **13 A (2C) continuous, 26 A (4C) for 30 s**;
  - AC resistance 22 mΩ;
  - discharge −20 to 60 °C, charge 0 to 45 °C, 2.5 V cutoff, 72 g, 315 Wh/kg.

**Two corrections to the repo's cell model.** These are not applied to the baseline energy, to keep comparability:
1. The continuous rating is 2C, not 3C.
2. Energy per cell is 22.5 Wh typical, not 22.68 (−0.8%), and the minimum is 3.1% lower again.

Table 4. Cell current, C-rate and loaded voltage vs. flight phase, by pack, worst corner.

*Per cell in 6S5P. Cruise is at mid charge; peak is the 3 m/s climb need at 15% charge.*

| Pack | Cruise current [A] (C) | Peak needed [A] (C) | Continuous rating [A] | Loaded V, cruise end, −10 °C | Loaded V, climb need, 4,400 m −10 °C |
|---|---:|---:|---:|---:|---:|
| SiCore | 1.5–1.7 (0.24–0.26C) | 8.6 (1.32C) | 13 | 3.14 | **2.29 (below 2.5 cutoff)** |
| P50B | 1.3–1.4 (0.26–0.28C) | 6.0 (1.20C) | 60 | 3.43 | 3.22 |

**Why SiCore sags.** C-rate isn't the problem: 1.3C against a 2C rating. Resistance is. The assumed SiCore DC resistance is 44 mΩ, against 12.8 mΩ for P50B. Cold (×3) and low charge (×1.3) push it to about 0.17 Ω per cell, and a 30 A pack current then drops about 1 V per cell. The SiCore cell's own I²R heat (about 7 W across the pack in cruise) keeps it near 0 °C at −10 °C ambient. P50B's (about 2 W) leaves it at −7 °C.

**Hot corner.** Pack temperature at the end of flight is at most 43 °C (SiCore, +40 °C ambient), 17 K inside the 60 °C discharge limit. A 3-minute full-power climb adds about 7 K, worked out by hand rather than in the script. Charging needs the cells at 0 °C or above (SA112), so field charging at −10 °C needs a warm box.

Table 5. Endurance vs. corner, by pack and battery state, kit carried, 10% reserve.

*Best speed = least power between max(14 m/s, 1.25 Vs) and 20 m/s, the requirement definition. Search speed = 20 m/s SiCore / 18 m/s P50B, the operating rule.*

| Corner | SiCore best speed (pre-warmed / cold-soaked) [h] | SiCore at 20 m/s [h] | P50B best speed (pre-warmed / cold-soaked) [h] | P50B at 18 m/s [h] |
|---|---:|---:|---:|---:|
| 2,000 m, +20 °C (design case) | 3.86 | 3.13 | 3.08 | 2.75 |
| 0 m, −10 °C | 3.83 / 3.40 | 2.73 / 2.43 | 3.05 / **2.71** | 2.45 / 2.18 |
| 2,000 m, −10 °C | 3.53 / 3.13 | 2.86 / 2.54 | **2.81 / 2.50** | 2.51 / 2.23 |
| 4,400 m, −10 °C | 3.22 / **2.87** | 2.94 / 2.62 | **2.57 / 2.29** | 2.53 / 2.25 |
| hot-and-high | 3.56 / 3.52 | 3.18 / 3.15 | **2.83 / 2.80** | 2.78 / 2.75 |
| 0 m, +40 °C | 4.17 | 2.94 | 3.32 | 2.64 |

**The 20 m/s search rule.** It costs 0.3–1.0 h against best speed. On cold days the SiCore aircraft falls below 3 h at 20 m/s even when pre-warmed (2.73–2.94 h), so the planner should drop to 18 m/s or less when the cold-weather endurance is the constraint.

Table 6. Pack heater power and endurance vs. insulation thickness, by pack.

*2,000 m, −10 °C, search speed, pack held at 15 °C (usable-energy factor 0.99). EPS k 0.035 W/m·K over 0.2 m². τ = pack cool-down time constant.*

| Pack | Insulation [mm] | τ [h] | Heater [W] | Endurance, heated [h] | Pre-warmed only [h] | Cold-soaked [h] |
|---|---:|---:|---:|---:|---:|---:|
| SiCore | 10 | 0.88 | 13.8 | 2.90 | 2.86 | 2.54 |
| SiCore | 20 | 1.75 | 5.5 | **3.04** | 2.86 | 2.54 |
| SiCore | 30 | 2.63 | 2.8 | 3.09 | 2.86 | 2.54 |
| P50B | 10 | 0.88 | 16.6 | 2.48 | 2.51 | 2.23 |
| P50B | 20 | 1.75 | 7.9 | 2.61 | 2.51 | 2.23 |

- **Correction to the cold-battery note.** It says "a few watts, about 1% of cruise" holds the pack near 15 °C. That is only true with about 30 mm of foam. With 10 mm it takes about 14 W (9% of cruise power), and the heater barely pays for itself.
- **Recommendation:** 20 mm of foam around the packs plus a 5 W thermostat pad. The foam alone doubles the cool-down time constant.

## 5. Return-to-home energy

Table 7. Return energy vs. headwind state, by pack, 10 km from home.

*Best return airspeed minimises battery Wh per ground metre. Battery at 25% charge. 12 m/s headwind.*

| Pack | Corner, kit | Best return airspeed [m/s] | Return time [min] | Return energy [Wh] | Still-air return [Wh] | Fixed 10% reserve [Wh] | Spare [Wh] |
|---|---|---:|---:|---:|---:|---:|---:|
| SiCore | 2,000 m +20 °C, on | 25.2 | 12.6 | 51.2 | 23.1 | 57.8 | +6.7 |
| SiCore | 4,400 m −10 °C, on | 27.0 | 11.1 | 45.3 | 22.1 | 52.1 | +6.7 |
| P50B | 2,000 m +20 °C, on | 25.1 | 12.7 | 51.1 | 23.2 | 45.9 | **−5.2** |
| P50B | 4,400 m −10 °C, on | 26.9 | 11.2 | 45.3 | 22.7 | 41.3 | **−4.0** |

**Reserve logic.** A fixed percentage is the wrong tool: the cost of getting home depends on distance and wind, and doubles from still air to a 12 m/s headwind. I'd separate two quantities:
- **Landing reserve:** keep the existing 10% of usable energy, never spent en route. It covers the approach, one go-around (a 30 s climb at about 600 W is 5 Wh) and loiter.
- **Bingo energy:** start home when the remaining energy, less the landing reserve, falls to 1.2 × E_return(d, wind + 3 m/s). E_return comes from the polar above, the 1.2 factor and +3 m/s cover wind-estimate error, and recomputing it continuously in flight is cheap.

**What that means at the edge.** At 10 km in a 12 m/s headwind the bingo is about 61 Wh, so the aircraft heads home with about 20% (SiCore) or 23% (P50B) of usable energy still on board. That costs about 25 minutes of search time against a still-air return. The same rule enforces the "no full-power climb below 25% charge" condition SiCore needs (Table 8).

## 6. Margins

Table 8. SiCore climb vs. fix, worst corner (4,400 m, −10 °C, kit on).

| Case | Cell temp [°C] | Charge | Climb at 18 m/s [m/s] | 30° banked gradient | Binding limit |
|---|---:|---:|---:|---:|---|
| Baseline | −0.2 | 15% | 1.74 | 8.4% | cell voltage floor |
| Pack held at 15 °C | 15 | 15% | 2.80 | 14.3% | throttle |
| Climbs only above 25% charge | −0.2 | 25% | 2.59 | 13.0% | throttle |
| **Both** | 15 | 25% | **3.21** | **16.5%** | throttle |
| SiCore DC resistance 30 mΩ instead of 44 | −0.2 | 15% | 2.68 | 13.6% | throttle |
| Floor at the 2.5 V cutoff (no margin) | −0.2 | 15% | 2.17 | 10.7% | throttle |
| P50B pack instead | −7.3 | 15% | 3.76 | 19.6% | throttle |

Table 9. Requirement or limit vs. worst-corner value, by pack, with margin and pass/fail.

*From `out/propulsion_margins/margins.csv`. The worst corner is chosen over all 13 corners and both kit states.*

| Requirement / limit | Limit | SiCore worst (corner) | Margin | P50B worst (corner) | Margin |
|---|---|---|---:|---|---:|
| Max level speed, continuous | ≥ 22 m/s | 29.5 (0 m −10 °C) | +7.5 | 35.0 (0 m −10 °C) | +13.0 |
| Ground speed into 12 m/s headwind | > 0 m/s | 17.5 | +17.5 | 23.0 | +23.0 |
| Climb at 18 m/s | ≥ 3 m/s (`sizing.A.climb_rate`) | **1.74 (4,400 m −10 °C)** | **−1.26 FAIL** | 3.76 | +0.76 |
| 30° banked climb gradient | ≥ 15% (`terrain_sim.CLIMB_GRAD`) | **8.4% (4,400 m −10 °C)** | **−6.6 pts FAIL** | 19.6% | +4.6 pts |
| Go-around gradient at 1.3 Vs | ≥ 3.3% ([CS-VLA 77](https://www.easa.europa.eu/sites/default/files/dfu/Easy%20Access%20Rules%20CS-VLA%20%28Amendment%201%29.pdf), 1:30) | 9.6% | +6.3 pts | 21% | +18 pts |
| Launch climb gradient at 1.25 Vs | ≥ 10% (CS-VLA 65, 1:10) | 19.7% | +9.7 pts | 27% | +17 pts |
| Bungee release / stall (14 m/s) | ≥ 1.10 (ASSUMED) | **1.00 (4,400 m)** | **−0.10 FAIL** | **1.00** | **−0.10 FAIL** |
| Time from release to 50 m | ≤ 30 s (ASSUMED) | 15.7 | +14.3 | 12.7 | +17.3 |
| Motor current needed | ≤ 50 A / 180 s | 40.6 (0 m +40 °C) | +9.4 (19%) | 40.5 | +9.5 |
| Motor current, uncapped full throttle | ≤ 50 A | **66** | **FAIL without limit** | **86** | **FAIL without limit** |
| Steady winding temperature needed | ≤ 120 °C (ASSUMED) | 96 | +24 K | 96 | +24 K |
| ESC current needed | ≤ 60 A | 40.6 | +19.4 (32%) | 40.5 | +19.5 |
| Prop RPM, any case | ≤ 9,000 (ASSUMED) | 7,790 | +13% | 8,355 | +7% |
| Prop tip Mach | ≤ 0.6 (ASSUMED) | 0.44 | +0.16 | 0.50 | +0.11 |
| Cell current needed | ≤ continuous rating | 8.6 A of 13 | +34% | 6.0 A of 60 | +90% |
| Loaded cell voltage needed | ≥ 2.8 V | **2.29 (4,400 m −10 °C)** | **−0.51 FAIL** | 3.22 | +0.42 |
| Pack temperature, hot | ≤ 60 °C | 43 | +17 K | 41 | +19 K |
| Pack temperature, cold | ≥ −20 / −40 °C | −0.2 | +20 K | −7.3 | +33 K |
| Endurance, design case (best speed) | ≥ 3 h | 3.86 | +0.86 | 3.08 | +0.08 |
| Endurance, worst corner, pre-warmed | ≥ 3 h | 3.22 (4,400 m −10 °C) | +0.22 | **2.57** | **−0.43 FAIL** |
| Endurance, worst corner, cold-soaked | ≥ 3 h | **2.87 (4,400 m −10 °C)** | **−0.13 FAIL** | **2.29** | **−0.71 FAIL** |
| 10 km return in 12 m/s headwind vs fixed 10% reserve | spare ≥ 0 | +6.7 Wh | pass | **−5.2 Wh** | **FAIL** |

**Failures and the physical fix for each:**

| Failure | Fix | Cost |
|---|---|---|
| SiCore climb and banked-climb gradient (cold, end of charge) | Hold the cells at 15 °C or above: 20 mm foam plus a 5 W thermostat pad. Forbid full-power climbs below 25% charge via the bingo rule. Measure SA112 DC resistance at −10 / 0 / 25 °C and 15–25% charge | About 5 W heater, about 50–100 g foam (estimated) |
| Uncapped full-throttle current | Set the ESC current limit to 45 A. Alternative: a 15x8 prop | No hardware cost; the 15x8 costs about 4% cruise power (v3 table) |
| Bungee release at high density altitude | Launcher rated at least 16 m/s release, or launch into at least 2 m/s wind, or launch below 90% of loaded mass above 3,000 m density altitude | Ground equipment |
| SiCore cold-soaked endurance at 4,400 m | Mandatory pack pre-warm (pre-warmed passes, 3.22 h). Insulation per above | Operations |
| P50B endurance off design | No propulsion fix. P50B is the fallback cell, not a 3 h pack outside +0 to +40 °C below about 2,000 m | Choose SiCore |
| P50B reserve vs headwind return | Bingo rule replaces the fixed reserve as the return budget | Software |

## 7. Assumptions and what would close them

**Assumptions:**
- SiCore DC resistance = 2 × the datasheet AC resistance.
- Resistance temperature law, B = 2,500 K.
- Generic NMC open-circuit voltage curve.
- 2.8 V loaded floor.
- 0.80 full-throttle voltage factor, fitted to only two bench rows.
- Motor thermal resistance 0.7 K/W and a 120 °C winding limit.
- 9,000 rpm prop limit, taken from the 15x8 blade.
- 60 A ESC.
- 10 mΩ wiring.
- 14 m/s bungee release.

**Tests that would close them:**
1. SA112 pulse tests (8 A, 30 s) at −10, 0 and 25 °C and 15 / 25 / 50% charge.
2. A 15x10 on AT3520-550 thrust stand at 6S up to 50 A, with a winding thermocouple and a wind-tunnel or car-top run at 18 m/s.
3. A chamber-soak of the insulated pack in the wing core.
4. Launcher exit-speed measurement at full mass.

Reproduce: `../.venv/Scripts/python.exe -X utf8 propulsion_margins.py` and `test_propulsion_margins.py` from `scripts/`. Outputs: `out/propulsion_margins/{performance, motor_points, battery, endurance, heater, rth, fixes_sicore_climb, margins}.csv`, `power_curves.png`, `margins_log.txt`.
