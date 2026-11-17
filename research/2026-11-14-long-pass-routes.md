# Longer camera passes on trail-following routes

2026-11-14

## Problem

On continuous wind-adaptive routes (`2026-11-13-continuous-route-replay.md`), the trails-first tour follows OSM trail vertices simplified at 25 m. At Bear Lake, the 6 km tour has 648 legs with a median heading change of 65°. For Lodestar at a 6 m/s wind band, 339 of the corners cannot take a tangent fillet and get a turnaround loop instead. The aircraft flies 266 km of arcs, with the camera off, against 119 km of straights. The energy envelope stops every route on every site before the tour ends.

## What did not work

Re-simplifying the tour did not remove the loops. I simplified it with Ramer–Douglas–Peucker at up to the full camera half-swath (56 m for Lodestar). I also dropped every loop-forcing corner that the same tolerance allowed. Lodestar still kept 150 loops at Bear Lake, because the loops sit on switchbacks and dead-end spurs, which are real trail shape.

## Method

`scripts/long_pass.py` flies the tour with a pure-pursuit tracker, the standard path-following autopilot law. It works like this:
- It steers toward the trail point one planning radius ahead of the nearest trail point, searched forward only.
- It never turns tighter than 1.3 × the route's ground radius for the wind band.
- The flown track is simplified at 10 m into legs, which the existing fillet builder (`turn_path.rounded_path`) rounds.
- The return leg and the expanding square after the tour are unchanged.
- Everything downstream is the continuous-route model, unchanged: route speed, terrain clearance, energy and reserve checks, camera, canopy and kit.

**Detection draws per pass.** The replay gives each source leg its own detection draw, so many short legs would add independent looks at one spot. The pursuit route therefore shares one draw per *pass*: a new pass starts after 90° of cumulative turning or at a gap. A re-scored copy of the baseline under the same rule (`rounded_passes`) isolates the effect of the convention. On the tuning draws, that rule changes the scores by 0.3 points or less. On the held-out draws, it changes them by up to 0.7 points, in either direction.

**Tuning and held-out test.** The lookahead was tuned on the existing draws (seed 20261111, 6 km route, SiCore and Evo GOV). 1.0 × radius beat 0.5 and 2.0 for both aircraft. The fleet was then evaluated once on fresh hikers, winds, detection draws and canopy hiding draws (seed 20261114): 7 sites × 2 patterns × 7 aircraft × 500 runs × 3 modes = 147,000 rows.

**Checks** (`scripts/test_long_pass.py`, 10 checks, all pass):
- simplification keeps spur tips;
- the tracker never exceeds its turn limit and stops at the end of the trail;
- pursuit legs are contiguous and need far fewer loops;
- pass grouping behaves as specified.

One bug was found and fixed before the final runs. The tracker could orbit the trail's end, unable to close within 10 m, until it hit its step cap. Both runs were then repeated. The tuning results matched to four decimals.

## Results

**Table 1.** Route geometry by aircraft and route type, seven-site, two-pattern, five-wind-band mean, held-out draws.

| Aircraft | Turnaround loops, baseline → pursuit | Straight (camera-on) km in planned route, baseline → pursuit | Trail within camera half-swath (pursuit) |
|---|---:|---:|---:|
| Lodestar SiCore | 282 → 10 | 144 → 255 | 93% |
| DeltaQuad Evo GOV | 269 → 11 | 151 → 247 | 88% |
| DeltaQuad Evo-LE (press) | 269 → 12 | 359 → 488 | 88% |
| C-Astral Bramor ppX | 264 → 11 | 116 → 196 | 89% |

**Table 2.** Modeled probability of help within 1 h / 3 h [%] by aircraft, route type and trails-first radius, held-out draws, seven-site mean.

| Aircraft | 3 km baseline | 3 km pursuit | 6 km baseline | 6 km pursuit |
|---|---:|---:|---:|---:|
| Lodestar SiCore | 4.11 / 7.60 | 5.05 / 9.66 | 3.74 / 6.43 | 4.88 / 8.37 |
| Lodestar P50B | 4.08 / 6.69 | 5.05 / 8.37 | 3.74 / 5.69 | 4.88 / 7.03 |
| DeltaQuad Evo GOV | 3.51 / 7.83 | 4.60 / 10.00 | 3.43 / 7.14 | 5.17 / 9.54 |
| DeltaQuad Evo-LE (press) | 3.51 / 8.54 | 4.77 / 10.66 | 3.43 / 7.89 | 5.14 / 10.80 |
| C-Astral Bramor ppX | 3.54 / 7.26 | 4.57 / 9.03 | 3.60 / 6.60 | 5.23 / 8.60 |
| Quantum Trinity Pro | 3.40 / 3.60 | 4.71 / 5.09 | 3.11 / 3.29 | 4.86 / 5.23 |
| AgEagle eBee X | 3.80 / 4.20 | 4.74 / 5.23 | 3.63 / 3.94 | 5.03 / 5.60 |

Baseline = `rounded_adaptive`, the continuous route with per-leg draws. Absolute values differ from the seed-20261111 tables because the hikers and winds are new.

**Table 3.** Paired difference in help, aircraft minus Lodestar SiCore [percentage points], with 95% interval, pursuit routes, held-out draws.

| Aircraft | 3 km, 1 h | 3 km, 3 h | 6 km, 1 h | 6 km, 3 h |
|---|---:|---:|---:|---:|
| DeltaQuad Evo GOV | −0.45 (−1.10 to 0.21) | +0.34 (−0.48 to 1.17) | +0.30 (−0.39 to 1.01) | +1.17 (0.18 to 2.17) |
| DeltaQuad Evo-LE (press) | −0.28 (−0.99 to 0.43) | +1.00 (0.17 to 1.89) | +0.27 (−0.39 to 0.92) | +2.43 (1.46 to 3.51) |
| C-Astral Bramor ppX | −0.48 (−1.16 to 0.26) | −0.63 (−1.43 to 0.15) | +0.35 (−0.33 to 1.06) | +0.23 (−0.77 to 1.21) |

## Findings

1. **Long passes are the largest gain found so far.** On held-out draws, pursuit routes raise 3-hour help for every aircraft, by 1.0–2.9 points. For Lodestar SiCore it rises from 7.60% to 9.66% (+2.06, 1.03 to 3.08) on the 3 km route, and from 6.43% to 8.37% (+1.94, 0.83 to 3.00) on the 6 km route. One-hour help rises 0.9–1.7 points for every aircraft.
2. **The route recovers about a third of what the switch to smooth turns lost**, under the same turn physics. Loops fall from about 280 to about 10 per route. Camera-on distance in the planned route rises by 36–116% depending on the aircraft (+78% for SiCore).
3. **The ranking barely moves.** Every aircraft gains about the same, because every aircraft was losing distance to loops.
   - 3 km route: Lodestar SiCore and the Evo GOV are tied at 3 hours (+0.34, interval spans zero). Lodestar is ahead at 1 hour by 0.45 points, but that lead is not resolved.
   - 6 km route: the Evo GOV leads by 1.17 points at 3 hours.
   - The Evo-LE leads at 3 hours on both routes, on its press-reported 8 h endurance.
4. **On the pursuit routes, the 3 km route scores higher than the 6 km route for SiCore, by 1.3 points at 3 hours.** Lodestar's best route is the tighter, near-trailhead tour.
5. **The per-leg draw convention is not neutral.** On held-out draws it moves baseline scores by up to 0.7 points per aircraft. In the 6 km baseline it accounts for most of the Evo GOV's lead (0.71 points per-leg versus 0.17 per-pass). Future comparisons should use per-pass draws.

## Limits

- The tracker plans in the horizontal plane. Climb and turn power, roll rate and gusts are still open (STATUS item 2).
- About 7–12% of the trail lies farther than the camera half-swath from the flown track, mostly at switchback and spur tips. The detection model scores this from actual geometry, but the trail-following pattern itself no longer guarantees every trail point is in frame.
- The 90° pass rule is a convention, not a calibrated correlation model for repeated looks.

Outputs:
- `out/wing_trade/long_pass_tune/`
- `out/wing_trade/long_pass_seed_20261114/`, containing `summary.csv`, `by_site.csv`, `paired.csv`, `paired_vs_sicore.csv`, `routes.csv` and `runs.csv.gz`.
