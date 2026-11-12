# Search pattern: where to look first

12 November 2026 (US Central) · Claude Opus 5.5 (claude-opus-5-5)

## The answer

- **Fly the trails first, then spiral out.** In a realistic lost-hiker case, following the mapped trails, roads and drainages out to 6 km, then flying an expanding square from the last known point, raises Lodestar's help-within-3-h from 20% to 32%. The baseline is the milestone 4 back-and-forth sweep of a 4 × 4 km box. It's a procedure, so every aircraft gains.
- **The kit's value shows up as speed.** When the aircraft launches from the trailhead, a ground team often walks in within 3 hours anyway, so drones with 3.5 h or more converge near 30% on the 3-hour score.
- **Lodestar gets help on scene within 1 hour in 21% of searches**, against 8-11% for drones that only report a location.
- **Once the person is found, the median time to help is 38 minutes against 79** (DeltaQuad Evo GOV, same pattern).
- **Operating rule added:** trails first to 6 km, then the expanding square. A 9 km radius performs the same as 6 km with SiCore, and worse for shorter-endurance aircraft.

## Prior art

- **SAREnv** (Drones 2025, 9, 628) is an open benchmark for UAV search on 60 real-terrain scenarios. It tabulates ISRID (International Search & Rescue Incident Database) statistics:
  - **Distance from the initial planning point, temperate mountains** (n = 568): 25% at 1.1 km, 50% at 3.1 km, 75% at 5.8 km, 95% at 18.3 km.
  - **Fits:** SAREnv finds that a log-normal fits these distances best.
  - **Find locations, temperate** (n = 312): road 13%, linear features (trails) 25%, drainage 12%, structure 13%, field 14%, water 8%, woods 7%, others under 5%.
  - **Planners compared:** concentric circles, pizza zigzag, greedy and random exploration.
- **Lin & Goodrich (IROS 2009)** frame UAV wilderness search as covering the most probability within the flight time, and search higher-probability areas first.
- **Ground SAR practice:** a hasty search of trails and other linear features first, then area search. The expanding square is the standard IAMSAR pattern around a datum.

## Model (`scripts/search_patterns.py`)

**Scenario C, trailhead.** Launch point, command post and the ground team's start are all at the last known point.

**Linear features.** Four features run outward from the last known point at random bearings and are known from the map. P_ON = 50% of hikers lie on one of them, with a lateral offset that is normally distributed with a 30 m standard deviation. The rest are at a uniform bearing. As a sensitivity case, P_ON = 30%, since the aircraft can't fly every minor feature.

**Distance from the last known point.** Log-normal, with median 3.1 km and σ = 1.23 from the ISRID quartiles. The fit runs slightly wide: 25% at 1.4 km, 75% at 7.1 km, 95% at 23.5 km.

**Patterns:**
- **box:** the milestone 4 lawnmower over a 4 × 4 km box.
- **spiral:** an expanding square from the last known point.
- **hasty R:** a "star" out and back along the four features to R km, then the expanding square.

Track spacing is 55 m everywhere.

**Physics.** The same as the mission simulation:
- wind triangle, energy, reserve, and the drop or report logic;
- the inverse-cube detection law with 10 ms camera blur;
- turns costed by the heading change.

Every aircraft flies at its own search speed. 1,000 hikers per case, with common random numbers.

## Results

**Help within 3 h (%), P_ON = 50%:**

| Aircraft | box | spiral | trails 2 km | 4 km | 6 km | 9 km |
|---|---|---|---|---|---|---|
| Lodestar SiCore | 19.9 | 22.1 | 25.1 | 29.0 | 31.7 | 32.0 |
| Lodestar P50B | 15.4 | 20.5 | 24.6 | 28.6 | 30.8 | 30.8 |
| DeltaQuad Evo GOV | 13.4 | 18.7 | 23.5 | 27.4 | 29.6 | 28.6 |
| C-Astral Bramor ppX | 12.2 | 18.8 | 23.6 | 27.4 | 29.5 | 28.7 |
| Quantum Trinity Pro | 4.1 | 13.3 | 18.3 | 21.9 | 23.4 | 20.8 |
| AgEagle eBee X | 3.3 | 11.9 | 17.3 | 21.0 | 22.0 | 16.4 |

**With P_ON = 30%**, the trails-first gain is smaller (Lodestar box 20.6% → trails to 6 km 28.5%), but trails first is still best.

**Trails to 6 km, P_ON = 50%:**

| Aircraft | Found | Help ≤ 1 h | ≤ 2 h | ≤ 3 h | Median help after a find |
|---|---|---|---|---|---|
| Lodestar SiCore | 31.8% | 21.4% | 27.5% | 31.7% | 38 min |
| Lodestar P50B | 30.9% | 21.1% | 27.4% | 30.8% | 39 min |
| DeltaQuad Evo GOV | 33.0% | 10.3% | 23.5% | 29.6% | 79 min |
| C-Astral Bramor ppX | 31.7% | 10.0% | 23.6% | 29.5% | 79 min |
| Quantum Trinity Pro | 23.4% | 10.8% | 21.7% | 23.4% | 62 min |

Under 35% of hikers are found at all within the 3-hour window, because the ISRID distance spread is wide: a quarter of hikers are more than 6-7 km out. A ground team that knew where to walk would reach 81% within 3 h. That's an upper bound, since the team doesn't know where to go.

## What it means for the design

- **No airframe change.** The procedure, not the aircraft, is the big lever. Flight software should fly the trails-then-spiral plan, generated from map data such as OpenStreetMap trails and waterways.
- **Two ways to state the value:**
  - when the launch point is far from the search area (scenario A), the lead on help within 3 h is 3×: 45% vs 15%;
  - from the trailhead, help gets there about 40 minutes sooner.
- **Next ideas:**
  - use real terrain from SAREnv's open dataset in place of straight synthetic trails;
  - a smarter allocation than a fixed 6 km radius (for example greedy over a probability map, as SAREnv benchmarks).

## Limits

- Trails are straight rays; real ones curve and branch.
- The person doesn't move during the search.
- Canopy is folded into the 80% sweep-width factor.
- The fit is slightly wider than ISRID, which is mildly pessimistic for every aircraft.
