# Milestone 4: design v2, mission simulation, and how Rescue Wing stacks up

2026-11-11 (US Central). Analysis by Claude Opus 5.5 (claude-opus-5-5).

**Code:**
- `scripts/design_v2.py`: cells, swappable bay, endurance matrix, AVL stability re-check
- `scripts/mission_sim.py`: Monte Carlo mission simulation against competitors

**Outputs:**
- `out/v2/`: `endurance_matrix.csv`, `stability_v2.csv`, `run_log.txt`
- `out/mission/`: `summary.csv`, `runs.csv`, `fleet.csv`, `p_find.png`, `time_to_help.png`, `run_log.txt`

## Answer first
**On the job it was designed for, getting help to a lost person fast, Rescue Wing beats every fielded drone in its class.** That includes a 13 kg aircraft claiming 8 hours.
- **What the simulation compares:** 2,000 simulated missions, with the same camera, the same winds and the same missing person for every aircraft.
- **The result:** Rescue Wing v2+ has its survival kit on the ground within 3 hours of launch in **39%** of missions. The best competitor manages **17%**.
- **Why the gap:** competitors can only find the person. A ground team still has to walk in. Rescue Wing drops water, warmth and a radio within about 3 minutes of the find.

**On pure search, it's competitive, not dominant.** Carrying the kit, it matches DeltaQuad's 10 kg Evo GOV within a few points, at 27% less weight.
- **Search-only mode:** with a battery module in the kit bay instead, it searches for 7.7 h (published-style).
- **Against the frontier:** that is close to the press-reported 8 h of the brand-new 13 kg Evo-LE, at 55% of the weight.
- **Where it falls short:** the Evo-LE still finds slightly more people per sortie (79% vs 74%).

## Who it's up against (published numbers)
| Aircraft | Type | Mass | Span | Flight time | Cruise | Can drop a kit? | Source |
|---|---|---|---|---|---|---|---|
| DeltaQuad Evo GOV | VTOL | 10 kg MTOW | 2.69 m | 4 h 15 min (dual battery, 1 kg payload) | 16.5 m/s | No | docs.deltaquad.com/gov/vehicle-specifications |
| DeltaQuad Evo-LE | VTOL | 13 kg MTOW | not listed | 8 h (press) | not listed | No | DeltaQuad press release / trade press, Sept 2026 |
| C-Astral Bramor ppX | catapult, parachute landing | 4.7 kg | 2.30 m | 3.5 h | 16 m/s | No (0.5 kg payload, no thermal listed) | c-astral.com |
| Quantum Systems Trinity Pro | VTOL | 5.75 kg | 2.39 m | 90 min (59 min default, export rules) | 17 m/s | No | Trinity Pro tech sheet, Oct 2024 |
| AgEagle eBee X | hand launch | 1.6 kg | 1.16 m | 90 min (endurance battery) | 11-30 m/s | No | eBee X brochure 2025 |

Published flight times are best case: calm air, sea level, and mostly light mapping cameras. I gave every competitor its published number as-is, which favours them.

## Design v2: more energy, same airframe
The airframe is validated through milestones 1-3, so v2 changes only what goes into the existing slots.

**1. Better cells, same 30 slots** (21700 format, 6S5P). Pack mass is the cells divided by 0.88, to cover nickel strip, the BMS and wiring.

| Cells | Cell | Pack | Wing pack | Status |
|---|---|---|---|---|
| v1 assumption | | 200 Wh/kg | 510 Wh, 2.55 kg | |
| **Molicel P50B** | 254 Wh/kg (18.0 Wh, 71 g) | 223 Wh/kg | 540 Wh, 2.42 kg | In volume production |
| **Amprius SiCore 6.3 Ah** | 315 Wh/kg | 277 Wh/kg | 680 Wh, 2.46 kg | Evaluation samples. Shipping to drone makers from a US pilot line; 3.6 V nominal assumed |

Discharge is gentle: about 21 A at full power across 5 parallel strings, so about 4 A per cell (under 1C). Both cells handle that easily.

**2. Mission-swappable kit bay.** The kit bay sits exactly on the balance point (that was the milestone 2 design rule), so whatever goes in it doesn't change the trim.
- An 18-cell battery module (6S3P) stands upright in the 220 × 130 × 120 mm bay, using 126 × 63 mm of its floor.
- It weighs 1.45 kg (P50B), less than the 1.70 kg kit plus parachute it replaces.
- One airframe, two missions: **search-and-deliver** or **long search**.

**Endurance (hours)**, `out/v2/endurance_matrix.csv`:

| Configuration | Published style: laminar | Published style: halfway | Published style: turbulent | Design case: laminar | Design case: halfway | Design case: turbulent |
|---|---|---|---|---|---|---|
| v1 cells, kit | 3.57 | 3.34 | 3.15 | 2.99 | 2.80 | 2.64 |
| **P50B, kit** | 3.84 | 3.60 | 3.38 | 3.24 | **3.04** | 2.86 |
| **SiCore, kit** | 4.82 | 4.52 | 4.25 | 4.06 | **3.81** | 3.59 |
| P50B, bay battery | 6.35 | 5.94 | 5.58 | 5.40 | 5.07 | 4.77 |
| **SiCore, bay battery** | 7.95 | 7.43 | 6.98 | 6.75 | **6.33** | 5.96 |

- **Published style:** sea level, calm air, battery to 85% depth of discharge. This is how makers quote it.
- **Design case:** 2,000 m altitude, with a 10% landing reserve (the actual requirement).
- **Laminar / halfway / turbulent:** the three wing-drag cases from milestone 3.

**The 3-hour requirement is back with margin.** P50B cells meet it in the design case unless the wing is fully turbulent. SiCore meets it in every case.

**Stability re-checked in AVL with the pod** (`out/v2/stability_v2.csv`). Eight cases:
- both cell types;
- kit on, kit dropped (fast and slow), and the battery module in the bay.

**All pass every handling criterion.** Static margin is 4.9-5.8%, the spiral is stable, and Dutch roll damping is 0.13-0.14. The balance point stays at x = 0.422 m and the packs sit at x = 0.46 m, as in milestone 3.

## The mission simulation
**Mission:** launch, fly to the search box, then fly a parallel-track ("lawnmower") search over it. Return home with 10% of the energy still in the battery.
- **On a find, Rescue Wing** descends to 40 m, confirms on camera, makes an into-wind run and drops the kit. That takes 3 min, plus 8 s for the kit to fall. It then flies home 1.7 kg lighter.
- **It only drops if it can still get home with the reserve intact.**
- **Competitors** report the find. A ground team then walks in from base at 3 km/h (cross-country with gear).

**Fairness rules:**
- **Same camera for everyone:** 640-pixel thermal, 32° field of view, 120 m above ground. That gives a 69 m footprint.
- **Detection model:** effective sweep width 55 m, with tracks 55 m apart. Each pass detects the person with Koopman's inverse-cube lateral-range law, the standard search-theory model.
- **Same 2,000 random missions for everyone** (common random numbers): target position, wind 0-10 m/s from any direction, and the detection dice.
- **Every aircraft searches at its best-range speed.** That is the speed that covers the most area per Wh. Rescue Wing's is computed: 15.5 m/s. Competitors' published "optimal cruise" is taken as theirs.
- **Competitor power vs airspeed** comes from a standard propeller-aircraft curve anchored at their published cruise speed and flight time.
- **Wind:** each leg uses the wind triangle (crab and ground speed). Transit legs pick the airspeed with the least energy per km over the ground. Turns are 30° bank, with a bulb turn when the turn is wider than the track spacing.

**Results** (`out/mission/summary.csv`; Monte Carlo standard error ≤ 1.1 points):

| Aircraft | A: finds | A: help within 3 h | A: median find → help on scene | B: finds | Area per sortie |
|---|---|---|---|---|---|
| **Rescue Wing v2+ (SiCore, kit)** | 47% | **39%** | 114 → **117 min** | 20% | 9.3 km² |
| **Rescue Wing v2 (P50B, kit)** | 36% | **35%** | 87 → **90 min** | 15% | 7.5 km² |
| **Rescue Wing v2+ search-only** | 74% | 15% | 176 → 276 min | 32% | 15.4 km² |
| DeltaQuad Evo GOV | 51% | 17% | 117 → 218 min | 21% | 10.6 km² |
| C-Astral Bramor ppX | 40% | 16% | 95 → 197 min | 16% | 8.4 km² |
| Quantum Trinity Pro | 16% | 15% | 37 → 142 min | 6% | 3.5 km² |
| AgEagle eBee X | 13% | 12% | 40 → 144 min | 4% | 2.6 km² |
| DeltaQuad Evo-LE (press numbers) | 79% | 17% | 169 → 270 min | 42% | 16.1 km² |

- **Scenario A:** lost hiker, 16 km² box centred 5 km out.
- **Scenario B:** wide area, 36 km² box centred 10 km out.
- **Area per sortie** is the area searched when the person isn't found (scenario A).

**Robustness:** I re-ran scenario A at 2,000 m with a fully turbulent wing. Rescue Wing v2+ still finds 44% and delivers the kit within 3 h in 41% (`out/mission/summary_worst_case.csv`). The thinner air makes it fly faster, which roughly offsets the shorter flight time.

**Charts:**
- `out/mission/p_find.png`: chance of finding the person per sortie, both scenarios.
- `out/mission/time_to_help.png`: how fast help reaches the person in scenario A. Rescue Wing's curve starts about 100 minutes before anyone else's.

## What this means
1. **The kit drop is the competitive edge, not endurance.** A drone that finds someone 5 km out still leaves them waiting about 100 minutes for a team on foot. Rescue Wing closes that gap in 3 minutes. For hypothermia or dehydration, that first hour or two is what matters.
2. **With SiCore cells, Rescue Wing searches the most area per sortie of any aircraft under 8 kg.**
   - **v2+ against Bramor ppX**, the closest flying-wing rival: 9.3 km² vs 8.4 km², while carrying the 1.7 kg kit plus the thermal camera (Bramor's maximum payload is 0.5 kg).
   - **The P50B version searches a little less than Bramor** (7.5 km²) under that load.
   - **The Evo aircraft search more**, but they are VTOL machines of 10-13 kg using solid-state or semi-solid-state cells.
3. **The swappable bay makes one airframe do two jobs.** Large-area searches can launch a search-only Rescue Wing first, then a kit carrier once the area narrows.
4. **Endurance is still the weak flank.**
   - **To beat Evo-LE outright on raw search**, it needs about 300+ Wh/kg packs. Amprius already offers 370-450 Wh/kg cells, but in pouch formats, which would mean re-laying-out the wing packs.
   - **A smooth moulded wing is worth 7%.** That's the gap between the halfway and laminar columns.

## Caveats
1. **Competitors' numbers are their published best case.**
   - Evo-LE's 8 h is press-reported, and its cruise speed is my assumption.
   - eBee X's best-range speed is assumed to be 14 m/s.
   - Trinity Pro is export-limited to 59 min by default.
2. **SiCore cells are evaluation-stage.** The production-ready v2 is the P50B version: 3.0 h design case, help within 3 h in 35% of missions, still 2× the best competitor.
3. **The search model is idealised.**
   - Uniform target location and one sortie only.
   - The detection law ignores speed and canopy differences between aircraft.
   - No relaunches.
   - Real SAR uses probability maps and multiple sorties, which helps every aircraft.
4. **The ground team assumption drives the help numbers.** At 3 km/h, cross-country with gear, the walk is about 100 min for 5 km. A team that's closer, or a helicopter, shrinks Rescue Wing's lead. A team on rough terrain grows it.
5. **This is an energy-and-kinematics simulation, not a 6-degree-of-freedom flight simulation.**
   - It doesn't model the autopilot or the aircraft's motion during the kit release.
   - AVL shows the aircraft trims and stays stable before and after the drop, since the kit sits on the balance point.
   - A JSBSim 6-DOF of the release transient moves to milestone 5.
6. **Endurance is a calculation, not a test.** The wing's surface finish is still the biggest unknown (±7%).
