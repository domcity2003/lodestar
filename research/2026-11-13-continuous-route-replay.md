# Continuous search route replay

2026-11-13

## Route model

The previous simulation flew straight to each waypoint, then charged time and energy for an in-place heading change. A constant-speed aircraft at 18 m/s and 30° bank needs a 57.2 m still-air radius. Two 90° fillets need 114.4 m between successive corners. The box pattern's 67.5 m crossover cannot accommodate both. A 180° course reversal has no finite tangent fillet on its original centerline.

The new route builder trims straight legs only where each tangent fillet fits. It keeps the original corner and flies an explicitly modeled turnaround elsewhere. The turnaround tests same-turn outer tangents and tangent three-arc connections between the two headings at the corner. At an exact reversal the shortest connection has length `7πR/3`. Lines and arcs share position and heading at each junction. The route retains the original search-opportunity IDs and disables detections on the connectors. [LaValle, Dubins curves](https://msl.cs.illinois.edu/~lavalle/planning/node821.html)

The route uses one TAS per route and wind band, with an 18 m/s floor. It raises TAS to meet 1.25× the loaded banked stall speed at the highest altitude on the retained route and uses 30° as the bank limit. The original 30 m terrain grid is sampled at up to 15 m intervals along the actual path, and the altitude profile maintains the model's 120 m sampled clearance with the existing climb and descent gradients. The planner stops at the mapped terrain boundary or after the first piece that would exceed an optimistic outbound-distance bound. That bound assigns the entire usable battery to outbound flight, assumes the strongest allowed tailwind, omits turn drag and credits all net descent energy. The per-scenario reserve test uses actual ground speeds and the return energy estimate.

For a steady wind vector, let `u = sqrt(TAS² − crosswind²)` and `Vg = u + alongwind`. The required bank satisfies `tan(bank) = TAS × Vg² / (g × R × u)` on a circular ground track. The conservative radius `(TAS + wind_bound)² / (g × tan(bank_limit))` covers every heading within the prescribed wind band. At 18 m/s, a 10 m/s tailwind raises the ground radius to 138.5 m. The fixed policy builds for 10 m/s on every sortie. The adaptive policy picks the next enclosing bound from 2, 4, 6, 8 and 10 m/s using the scenario's known steady wind. It does not use the hiker position or the detection outcome. [FAA, Airplane Flying Handbook, chapter 7](https://www.faa.gov/sites/faa.gov/files/regulations_policies/handbooks_manuals/aviation/airplane_handbook/08_afh_ch7.pdf)

Flight time uses the actual wind triangle over each line and arc. Energy uses the site's existing propulsion table, sampled climb and descent, and the local induced-power increase implied by the bank. The model checks return energy at each piece endpoint with a direct terrain-profile return and a full-circle maneuver allowance. A detection retains the original canopy and viewing models, and the release retains the original chute performance and timing assumptions.

## Reproduction

```powershell
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
.\.venv\Scripts\python.exe -X utf8 scripts/test_terrain_sim.py
.\.venv\Scripts\python.exe -X utf8 scripts/test_turn_path.py
.\.venv\Scripts\python.exe -X utf8 scripts/flyable_mission.py --patterns 'trails 3' 'trails 6' --workers 7
.\.venv\Scripts\python.exe -X utf8 scripts/verify_flyable_replay.py
```

The replay uses 500 matched hiker and wind scenarios per site from the existing cache, across seven sites, two patterns, two battery builds and three route policies. The `out/wing_trade/flyable` tables retain individual outcomes, route geometry statistics and site averages. Paired intervals resample hiker-run IDs across the seven fixed sites. They express Monte Carlo variation conditional on this model and these sites.

## Results

Three-hour modeled probability of help, averaged equally across the seven sites:

| Battery | Trails-first radius | Original sharp corners | Rounded, fixed radius policy | Rounded, wind-adaptive policy |
|---|---:|---:|---:|---:|
| SiCore | 3 km | 11.49% | 4.69% | 6.43% |
| SiCore | 6 km | 11.08% | 4.66% | 5.40% |
| P50B | 3 km | 10.54% | 3.80% | 5.23% |
| P50B | 6 km | 10.03% | 3.89% | 4.74% |

SiCore's 6 km wind-adaptive result is 5.68 percentage points below the sharp-corner control (paired 95% interval −6.60 to −4.80 points). It gains 0.74 points over the fixed-radius policy (0.37 to 1.17). Mean flown distance on camera-active straights rises from 20.85 to 31.33 km with wind-adaptive routing. For SiCore at Bear Lake, the 6 km score is 12.00% sharp, 5.40% rounded fixed and 6.40% rounded adaptive. The Bear Lake plan uses 18.64–18.67 m/s and radii of 75–145 m across the five wind bands.

The sharp-to-rounded difference combines changes in geometry, speed, usable camera passes and reserve accounting. It does not isolate the effect of turn energy. The 3 km pattern with wind-adaptive routing gives the highest three-hour score of the rounded cases for each battery. For both batteries, the 6 km adaptive pattern scores higher at one hour (3.28% versus 2.88% SiCore, 2.91% P50B). These data support testing a route built around longer camera passes, with trail coverage and tight turns evaluated together. Competitors still use the historical sharp-turn model, so these results do not establish a fleet ranking.

## Remaining validation

Ground tracks are horizontal-curvature plans with a sampled vertical profile. Wind is steady and known by magnitude band, power during bank is an engineering estimate, and the geometry changes curvature at line–arc junctions. A future flight-dynamics pass needs roll-rate and coupled climb/turn power limits, gust and wind-estimation errors, and an explicit continuous return, orbit and release path. Sampled DEM clearance omits obstacles and has the resolution limits of the underlying grid. The synthetic probability scores depend on camera, canopy, rescue-time and drop assumptions that still need field calibration.
