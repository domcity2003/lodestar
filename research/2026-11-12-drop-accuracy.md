# Drop accuracy: does the kit land within reach?

12 November 2026 (US Central)

## The answer

- **The drop mostly works.** With the full error budget, 92% of kits land within the person's reach. The headline help-within-3-h figure drops from 45% to 43% (SiCore build, lost-hiker scenario A), and from 37% to 35.5% for the P50B build. The lead over today's drones (15%) stays near 3×.
- **The gusts during the fall set the miss, so the fix is to fall faster.** Switching to a 7 m/s parachute cuts the fall time by about a third. That chute is 1.1 m across instead of 1.5 m and about 60 g lighter.
  - 96% of kits then land within reach, and the score is back to 44%.
  - **Recommended design change.** The kit needs padding, since landing at 7 m/s is like a drop from 2.5 m.
- **Rangefinder.** A laser rangefinder in place of the terrain map is worth about 1 point. With the 7 m/s chute, it brings the 95% radius to 16 m.
- **Live wind reading.** A 2 s wind reading doesn't help. The gust measured at release has mostly changed by the time the kit lands, and the short reading is noisier (0.7 m/s against 0.4 m/s).
- **Trailhead scenario.** The effect is under 1 point (31.7% to 31.6%) and help within 1 h goes from 21.4% to 21.0%. A ground team is already walking and covers most misses.

## Prior art

- **Unguided parachute delivery accuracy.** Unguided parachute drops are accurate to "a few tens of meters", depending on release height (proceedings paper on scindeks-clanci.ceon.rs). Guided systems (JPADS) accept 150 m for 1 t loads. For guided airdrop, the wind estimate is the dominant error (Cacan, Georgia Tech CAMM).
- **UAV thermal geolocation.**
  - At 50-100 m AGL, using only the autopilot's GPS and IMU: 1.6 m mean error (ISPRS 2015).
  - Thermal hotspots at 60-120 m: under 4.2 m (Remote Sensing 2025).
  - With code-only GPS: 5-10 m (ISPRS 2016).
  - Value used: 3 m per axis.
- **Turbulence near the ground.** σu = 2.5 u* (Beljaars 1987). With the log law, this gives a turbulence intensity of about 0.17-0.23 at 40 m over rough ground. Standard Panofsky ratios are used for v and w.
- Milestone 1b (`kit_drop.py`) looked only at the wind-estimate error.

## Model (`scripts/drop_accuracy.py`)

**Kit and release.** A 1.704 kg kit under a cruciform canopy. The canopy fills over 6-10 diameters of travel. Release is from 40 m on a 14 m/s into-wind run-in.

**Aiming.** The drone aims with a computed air release point (CARP). The prediction uses its estimated wind, its assumed height, a log profile with z0 = 0.1 m, and no gusts.

**Error sources in the true fall:**
- **Gusts:** an Ornstein-Uhlenbeck process on each axis. Length scales are 100, 50 and 20 m; the gust state is random at release.
- **Wind shear:** the log law, with the true z0 drawn between 0.03 and 0.5 m.
- **Height above the person:** 8 m error from a terrain map, 0.5 m with a rangefinder.
- **Canopy swing:** 0.3 m/s, with a 2 s time scale.
- **Release timing:** 50 ms × ground speed.
- **Target fix:** 3 m on each axis.

**Aim point.** 5 m beside the person, so the kit doesn't hit them (about 20 J at 5 m/s).

**Wind.** The mission wind is drawn at search height (120 m) and scaled to the release height with the log law.

**Reach.** 30% of people are hurt and can reach 15 m; the rest can reach 50 m. Both are assumptions.

**Scoring.** For each simulated mission, the help time is the kit's landing time with probability p_reach(wind). Otherwise it's the ground team's arrival, since the team is sent at the find. Runs are 3,000 drops per wind step from 0 to 10 m/s, per option.

**Headline runs.** These reproduce v3_blur_fair, with 10 ms blur and the same random draws, so the perfect-drop figures (45.1% and 37.0%) match.

## Results

**Average over winds of 0-10 m/s:**

| Option | R50 (m) | R95 (m) | Within 25 m | Within reach |
|---|---|---|---|---|
| As designed (40 m, map height, 30 s wind) | 10.8 | 22.8 | 94.6% | 92.5% |
| + laser rangefinder | 9.7 | 19.9 | 97.5% | 94.5% |
| + live 2 s wind | 11.0 | 23.2 | 95.3% | 92.1% |
| 30 m release | 9.6 | 20.0 | 96.9% | 94.5% |
| **7 m/s chute** | 9.0 | 18.3 | 98.4% | **95.7%** |
| 7 m/s chute + rangefinder | 8.2 | 16.3 | 99.6% | 97.2% |

**95% radius in a 10 m/s wind:**
- 37 m as designed;
- 27 m with the 7 m/s chute;
- 22 m with the 7 m/s chute and a rangefinder.

The 30 m release is nearly as good as the faster chute but leaves little clearance over 20-25 m trees, so the faster chute is preferred.

**Help within 3 h (%), 10 ms blur:**

| Build | Perfect drops | As designed | 7 m/s chute |
|---|---|---|---|
| A, SiCore foam-core @ 20 | 45.1 | 43.2 | 44.0 |
| A, P50B foam-core @ 18 | 37.0 | 35.5 | 36.1 |
| B, SiCore foam-core @ 20 | 18.5 | 17.2 | 17.8 |
| B, P50B foam-core @ 18 | 15.4 | 14.3 | 14.7 |

## Limits

- **Trees and terrain.** Tree snag, slopes (a kit rolling downhill) and gullies aren't modelled. They're probably the larger real-world risks, and a flight test should drop into wooded slopes.
- **Turbulence statistics.** These are flat-ground surface-layer statistics; mountain terrain can be rougher.
- **Reach and injury.** The reach distances and the hurt share are assumptions. ISRID injury rates would firm them up.
- **Mission mass.** The mission sim still carries the 204 g chute mass. The 60 g saving isn't counted, so the result is slightly conservative.

## Sources

- Cacan, M. 2019, guided airdrop thesis (wind estimate as the dominant error): https://www.camm.gatech.edu/images/c/c8/Cacan_2019_Guided_Airdrop.pdf
- Unguided drop accuracy, "few tens of meters": https://scindeks-clanci.ceon.rs/data/pdf/proc-0018/2024/proc-00182400080A.pdf
- UAV thermal geolocation, 50-100 m: https://isprs-archives.copernicus.org/articles/XL-1-W5/333/2015/
- GPS/IMU-grade geolocation, 5-10 m: https://isprs-archives.copernicus.org/articles/XLI-B6/243/2016/
- Thermal hotspot geolocation under 4.2 m: https://doaj.org/article/d2fe181643844e30a5eabcc975761efc
- σu = 2.5 u* (Beljaars 1987), cited in https://www.frontiersin.org/journals/built-environment/articles/10.3389/fbuil.2025.1498984/pdf
