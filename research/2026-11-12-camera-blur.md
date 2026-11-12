# Camera motion blur vs search speed

12 November 2026 (US Central) · Claude Opus 5.5 (claude-opus-5-5)

## The answer

Counting thermal-camera blur lowers every score by 2-5 points of "help within 3 h", but it doesn't change any decision:

- **Search speeds stay the same:** 20 m/s with SiCore cells, 18 m/s with P50B cells. This held for every camera response time (7, 10, 12 ms) and detection threshold (N50 0.75, 1.0, 1.5) tested.
- **The lead over today's drones holds:** 45% vs 15% in the lost-hiker case, up from 50% vs 17% with no blur. Competitors fly slower, so blur costs them less, but they still rely on a team walking in.
- **Keep tracks 55 m apart.** Narrowing the tracks to fill the blur gaps scores worse than accepting the gaps.

The recommended build is unchanged: SiCore cells, foam-core wing, 20 m/s, about $7,760.

## Why blur matters

A microbolometer (uncooled thermal camera) pixel doesn't respond instantly. It follows the scene with a thermal time constant of about 7-12 ms ([FLIR support note](https://www.flir.com/en-gb/support/instruments2/what-is-the-response-time-for-flir-microbolometer-cameras-thermal-time-constant/); 8-12 ms in [FLIR's high-speed article](https://www.flir.com/discover/rd-science/high-speed-thermal-cameras--the-need-for-speed/)). In that time the ground slides by ground speed × time constant: 0.2 m at 20 m/s and 10 ms. That's about two pixels at 10.8 cm per pixel. The smear spreads a person's heat over more pixels and dims it.

## Model (`scripts/v3_blur.py`)

**Target and camera:**
- The target is a person seen from above, with a critical dimension of 0.5 m (shoulders; crouched or partly covered). This is an assumption.
- The camera has 640 pixels across a 32° field of view, flown at 120 m above ground. This is the same as milestone 4.

**Pixel size.** At an angle off straight-down, a pixel covers more ground: gsd0 / cos² across the track and gsd0 / cos along it. Blur adds to the along-track size in quadrature: √(gsd² + (Vg·τ)²).

**Detection chance.** The Johnson criteria with the target transfer probability function:
- N = d / (2√(gsd_cross · gsd_along)) cycles fall across the person;
- with r = N / N50, P = r^E / (1 + r^E) and E = 1.51 + 0.24 r;
- N50 is 1.0 in the base case, and 0.75-1.5 were tested.

**Sweep width.** Integrating P across the swath gives a physical sweep width. The milestone 4 calibration (55 m with no blur) is kept and scaled by W(Vg) / W(0), so only the blur effect is new.

**Wind.** Blur uses each search leg's ground speed, so downwind legs blur more. Hook: `mission_sim.W_OF_GS`, which is `None` by default, so older scripts reproduce their old numbers.

**Effective sweep width** (m) at N50 = 1.0:

| Time constant | 0 m/s | 15 m/s | 18 m/s | 20 m/s | 22 m/s |
|---|---|---|---|---|---|
| 7 ms | 55.1 | 51.2 | 50.0 | 49.2 | 48.4 |
| 10 ms | 55.1 | 48.6 | 46.9 | 45.8 | 44.7 |
| 12 ms | 55.1 | 46.9 | 44.9 | 43.7 | 42.6 |

## Results

These are scores for help within 3 h, as the average of the two scenarios, for the moulded wing (`out/v3/blur_sweep.csv`). The best speed is 21 m/s with SiCore and 18 m/s with P50B in every case.

| Case | SiCore @ 20 | SiCore @ 21 | P50B @ 18 |
|---|---|---|---|
| No blur (old model) | 35.7 | 37.0 | 31.5 |
| 7 ms | 33.5 | 34.5 | 30.0 |
| 10 ms | 32.1 | 33.0 | 28.9 |
| 12 ms | 31.2 | 31.9 | 28.1 |
| 10 ms, tighter tracks | 30.0 | 30.8 | 25.8 |

The **costed foam-core wing** carries less spare energy, so for it **20 m/s beats 21 m/s**: 45.1% vs 43.4% in the lost-hiker case at 10 ms. The rule stays at 20 m/s.

**Fair comparison, lost hiker, 10 ms blur on every aircraft** (`out/v3/blur_fair.csv`):

| Aircraft | No blur | 7 ms | 10 ms | 12 ms |
|---|---|---|---|---|
| Lodestar SiCore foam-core @ 20 | 50.0 | 47.0 | 45.1 | 44.0 |
| Lodestar P50B foam-core @ 18 | 40.2 | 38.4 | 37.0 | 36.1 |
| Lodestar P50B moulded @ 18 | 44.2 | 42.1 | 40.6 | 39.6 |
| DeltaQuad Evo GOV / Evo-LE | 16.6 | 15.8 | 15.4 | 14.8 |
| C-Astral Bramor ppX | 16.2 | 15.4 | 15.0 | 14.6 |
| Quantum Trinity Pro | 14.7 | 14.0 | 13.6 | 13.2 |
| AgEagle eBee X | 11.6 | 11.0 | 10.8 | 10.6 |

**Cost trades with blur on** (`scripts/cost_trades.py` now uses the 10 ms model):
- SiCore cells add 8.2 points for $434, about $53 per point (previously 9.7 points, about $45).
- Moulded skins add 0.3 points with SiCore cells, so they're still not worth the moulds.
- Cost per help-within-3-h mission is now:
  - **Lodestar** (P50B foam-core): $214;
  - **Trinity Pro:** $1,433;
  - **eBee VISION:** $5,092.

## Limits

- The Johnson criteria are a rule of thumb for a human observer. A detection algorithm, or an operator watching 30 frames a second, may do better, since a person stays in view for about 2.5 s per pass.
- The ZT6's time constant isn't published; 10 ms is the middle of the FLIR range.
- Image stabilisation, rolling or global readout, and thermal contrast (cold night vs hot afternoon) aren't modelled.
- **Flight test:** fly over a person lying in grass at 16, 18 and 20 m/s and count detections.
