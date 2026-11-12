# Design v3: can Lodestar get better before the cost model?

2026-11-12 (US Central). Analysis by Claude Opus 5.5 (claude-opus-5-5).

**Code:**
- `scripts/v3_propulsion.py`: prop and motor matching
- `scripts/v3_airfoil.py`: airfoil screen
- `scripts/v3_planform.py`: wing finish, area and span
- `scripts/v3_mission.py`: AVL stability re-check plus the mission sim
- `scripts/v3_search_speed.py`: search-speed sweep
- `scripts/cad_model.py v3`: CAD fit and balance

**Data:** `data/uiuc_props/`, measured wind-tunnel data for 41 propellers from the UIUC Propeller Data Site.

**Outputs:** `out/v3/` and `out/cad_v3/`.

## Answer first
**Yes. Endurance in the design case goes from 3.0 h to 3.6 h (+18%), on the same airframe, cells and balance point.** In the lost-hiker scenario, help (the survival kit) now reaches the person within 3 h in **44%** of missions, against 35% for v2 and 17% for the best fielded drone.

Three changes do it:

| Change | What it is | Endurance, design case | Cost impact (for milestone 6) |
|---|---|---|---|
| v2 as reported | 55% propulsion efficiency assumed | 3.04 h | |
| Better estimate, no design change | v2's own 13 in prop with a properly matched motor: 62%, not 55% | 3.32 h (+0.28) | none |
| **1. Matched propulsion** | Aeronaut CAM 15x10 folding prop + T-Motor AT3520-550: 63.6% | 3.39 h (+0.07) | motor and prop swap, likely similar price |
| **2. Moulded, smooth wing skins** | Keeps laminar flow over the wing, as the airfoil analysis assumes | **3.60 h** (+0.22) | **moulds and labour: the main cost question** |
| **3. Search at 18 m/s, not 16.5** | An operating rule, not hardware: more area per hour inside the 3 h window | (−) | none |

I'd be honest about the biggest single gain (+0.28 h): it isn't a design change. The old 55% figure was a pessimistic placeholder from milestone 1. Real measured prop data shows a matched setup gets about 62%. The real design gains are the 15 in prop (small) and the smooth skins (bigger, but they cost money). That's the first thing to trade in the cost model.

**What I checked that didn't help:**
- **A different airfoil.** MH45 is already about the best thick-enough reflexed section. The few that beat it gain about 1% drag, or are too thin for the battery cells.
- **More wing area at 3.0 m span.** The 12 m/s stall limit already sets the area. A bigger wing flies slower and loses endurance.
- **A 3.4 m span** (would need 1.25 m outer panels). It adds 0.2 h of endurance but almost nothing to the score: 45% vs 44%. Not worth breaking the car-fit requirement.

With the SiCore cells (v3+), 4.5 h design case. Searching at 20 m/s, it gets help to the person within 3 h in **50%** of lost-hiker missions.

## 1. Where the power goes (v2, search, 2,000 m, kit on: 136 W from the battery)
| Loss | Battery watts | Share |
|---|---|---|
| Motor, ESC and prop losses (55% assumed) | 50 | 37% |
| Induced drag (cost of making lift) | 31 aero → 55 battery | 41% incl. its share of the losses |
| Wing profile drag | 24 aero → 43 battery | |
| Pod and gimbal drag | 7 aero → 12 battery | |
| Avionics, gimbal, radios | 25 | 18% |

Propulsion losses were the biggest single item. Induced drag is mostly set by the 3.0 m span limit.

## 2. Propulsion (`propulsion_all.csv`, `propulsion_map.png`)
**Method:**
- **Prop data:** measured thrust and power coefficients from the UIUC database. The CAM folding props come from Dantsker, Caccamo, Deters & Selig, AIAA 2020-2762; the APC Thin Electric props are a rigid reference.
- **Motor model:** the standard three-constant motor model, using makers' spec values for Kv, winding resistance and no-load current. Five motors: three Scorpion SII-30/40 and two T-Motor AT3520.
- **Losses:** 95% ESC efficiency, and a 5% installation loss for a pusher behind the pod.
- **Matching:** for each prop and motor pair, the model finds the RPM and current that give the thrust needed at search, kit-dropped, transit, top speed (22 m/s) and a 3 m/s climb. Pairs that can't reach top speed or climb on a sagging 6S battery are dropped.
- **Ranking:** by search power corrected for the propulsion mass, since heavier motors cost power.

**Results:**
- 125 pairs pass. The best folding props reach 62-64%, so pitch matters more than diameter.
- Only folding props are eligible, because a rigid prop would strike the ground in a belly landing.
- **Pick: CAM 15x10 + T-Motor AT3520-550.** That's 63.6% at search, at 3,880 rpm and 12.3 V. The propulsion mass is 0.34 kg (v2: 0.28 kg).
- **Why the lighter motor wins:** the Scorpion SII-4020 is 0.8% more efficient, but 70 g heavier. That net loses.

**CAD check (`out/cad_v3/`):**
- The 15 in disc clears the wing trailing edge by 40 mm. On a swept wing the trailing edge runs back toward the prop as you go outboard.
- Every part still fits inside its skin.
- The balance point lands 0.3 mm from design. The heavier motor cancels v2's 1.7 mm forward offset.

## 3. Airfoil (`airfoil_screen.csv`)
**Method:** 52 published reflexed and flying-wing sections (Hepperle MH, Eppler, EH, Selig, PW, HS, Drela AG), run through NeuralFoil at the root, mean and tip Reynolds numbers.

**Screens:**
- pitching moment no worse than MH45 − 0.01 (the wing must still trim without a tail);
- thickness at least 9.5% (the battery cells need it).

**Results:**
- Only MH60 and MH61 beat MH45, by 1.2-1.4% section drag. That's under 0.5 W.
- MH61 also has a lower tip CLmax and lower NeuralFoil confidence.
- Thinner sections save 3-5%, but they don't fit the cells.

**Decision:** keep MH45.

## 4. Wing finish, area and span (`planform_sweep.csv`, `endurance_steps.csv`)
**Wing finish:** "laminar" means the drag NeuralFoil predicts with free transition; "halfway" was v2's basis (milestone 3 range). Smooth moulded skins buy 0.22 h, the largest real design gain.

**Wing area:** at 3.0 m span, aspect ratio 7 (v2) is still the best the 12 m/s stall limit allows. Lower aspect ratios (more area) stall slower but lose 1-5% endurance.

**Span:**
- 3.4 m buys 0.2 h of endurance, even with a spar-weight penalty that scales with span.
- AVL says it is stable (`stability_v3.csv`).
- But see section 6: it doesn't move the help-within-3-h score.

## 5. Stability re-check (AVL, `stability_v3.csv`)
| Case | Static margin | Elevon trim | All handling criteria |
|---|---|---|---|
| Kit on, search | 5.9% | −0.3° | pass |
| Kit dropped, top speed | 5.0% | +1.3° | pass |
| Kit dropped, slow | 5.5% | −0.5° | pass |
| Kit on, top speed | 5.3% | +1.1° | pass |

That covers both short-period and phugoid (pitch) and Dutch roll, roll and spiral (lateral). The 3.4 m option also passes in all four cases.

## 6. Mission sim (`mission_summary.csv`, `search_speed.csv`)
**Setup:** same 2,000 missions, winds and targets as milestone 4. Power comes from the measured prop map at every airspeed.

| Aircraft (scenario A, lost hiker) | Flight time, published style | Find in one sortie | Help within 3 h |
|---|---|---|---|
| **Lodestar v3 (P50B), search at 18 m/s** | 4.3 h | 45% | **44%** |
| Lodestar v3 (P50B), best-range 16.5 m/s | 4.3 h | 45% | 41% |
| Lodestar v3+ (SiCore), 20 m/s | 5.4 h | 54% | **50%** |
| Lodestar v2 (P50B) | 3.7 h | 36% | 35% |
| DeltaQuad Evo GOV | 4.25 h | 51% | 17% |
| DeltaQuad Evo-LE (press-reported 8 h) | 8.0 h | 79% | 17% |

**The search-speed finding:**
- Milestone 4 searched at the best-range speed, which gives the most km² per Wh. Inside a 3-hour window, though, km² per hour matters more.
- For the P50B aircraft, "help within 3 h" peaks at **18 m/s**. Past that, it runs out of energy before the box is covered.
- The SiCore version has energy to spare, so it keeps gaining up to the 20 m/s top of the search-speed requirement.
- This lines up with milestone 5, which also said to fly the drop run at 18 m/s in turbulence. 18 m/s becomes the standard search and drop speed.

**Scenario B** (36 km², 10 km out): v3 at 18 m/s reaches 19% against 15% for v2 and 0% for every fielded drone. Their ground teams can't walk 10 km in 3 h.

## Caveats
1. **Motion blur at higher speed isn't modelled.** A microbolometer thermal camera smears the image over roughly its thermal time constant, typically around 10 ms, which is about 0.18 m at 18 m/s. That's 1.7 pixels at 10.8 cm per pixel. It would cut detection somewhat at 18-20 m/s. Flight tests should set the speed limit; 18 m/s is a sensible starting rule.
2. **Smooth skins are an assumption until tested.** Bugs, rain and paint chips trip the airflow from smooth (laminar) to rough (turbulent). Even then the halfway case gives 3.39 h, above v2's 3.04 h.
3. **Prop data.** It's from an isolated wind tunnel; I apply a 5% loss for the pusher installation. Below 3,000 rpm the lowest tested run is reused, which is slightly optimistic. The pick runs at 3,880 rpm, inside the tested range.
4. **Motor values are makers' spec-sheet figures, not measured.** Motor efficiency is about 86% at search.
5. **The 3.4 m option uses a crude spar-weight scaling.** It's dropped anyway.

## Sources
- UIUC Propeller Data Site, volumes 3-4: https://m-selig.ae.illinois.edu/props/propDB.html
- Dantsker, Caccamo, Deters & Selig, "Performance Testing of Aero-Naut CAM Folding Propellers", AIAA 2020-2762
- Scorpion SII-30/40 spec pages: https://www.scorpionsystem.com/catalog/aeroplane/motors_1/
- T-Motor AT3520 spec: https://store.tmotor.com/product/at3520-long-shaft-fixed-wing-motor.html
