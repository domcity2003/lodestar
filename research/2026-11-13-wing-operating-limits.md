# Compact wing stall, endurance and turn margins

2026-11-13

Keep compact XPS as the prototype baseline. The mixed EPS/XPS core saves 142 g of wing mass, but neither design meets the loaded stall target and the lighter core gains only 0.06 percentage points in the three-hour canopy replay. The existing global structural checks give similar tube stress, tip deflection and skin shear for both candidates. The local EPS wrinkling screen puts estimated face compression and estimated wrinkling resistance at roughly 11 MPa each. A bonded sandwich and foam-transition test would determine whether the EPS saves enough weight with the required skin support. [Wing redesign](2026-11-12-wing-redesign.md) · [NASA sandwich wrinkling review](https://ntrs.nasa.gov/citations/19990017863)

## Loaded stall target at 2,000 m

The screen uses the canopy replay configuration with the 7 m/s chute, both cell types at 18 m/s TAS, and the same 3D maximum lift coefficient assumption of 0.80. Older wing trade tables used the heavier chute and 20 m/s for SiCore. All speeds here are true airspeed.

| Wing | Pack | Loaded stall (m/s) | Stall after kit release (m/s) | Further aircraft mass to remove for 12 m/s (kg) | Required maximum lift coefficient at current mass |
|---|---|---:|---:|---:|---:|
| Compact XPS | SiCore | 12.63 | 11.32 | 0.81 | 0.886 |
| Compact mixed | SiCore | 12.51 | 11.21 | 0.66 | 0.870 |
| Compact XPS | P50B | 12.60 | 11.29 | 0.78 | 0.882 |
| Compact mixed | P50B | 12.49 | 11.18 | 0.63 | 0.866 |

At the modeled loaded SiCore mass, the XPS wing would also reach 12 m/s at about 1.42 m² if maximum lift coefficient and aircraft mass were held fixed, versus 1.29 m² today. That is an isolated sensitivity. An area change would affect mass, trim and drag and needs its own design iteration.

## Turn margin on the search routes

The 30° coordinated level-turn stall speed is the level stall speed divided by the square root of the cosine of bank. Applying the design margin of 1.25 times stall makes the 18 m/s turn-margin ceiling 3,163 m for XPS SiCore and 3,336 m for mixed-core SiCore under the standard atmosphere. At 4,400 m the 30° turn requires 19.20 m/s on XPS SiCore and 19.03 m/s on mixed-core SiCore to preserve that margin. [FAA Airplane Flying Handbook](https://www.faa.gov/sites/faa.gov/files/regulations_policies/handbooks_manuals/aviation/airplane_handbook/11_afh_ch10.pdf)

At Bear Lake the 6 km route plus expanding square has 2,128 planned heading changes. Of those, 797 exceed the XPS SiCore 18 m/s turn-margin ceiling and 612 exceed the mixed-core ceiling. The highest turn junction is 3,942 m, where XPS SiCore needs 18.74 m/s to retain its modeled margin at 30° bank. The count uses the sampled path over the entire planned route and includes legs a sortie may never reach. The canopy replay charges turn time and energy using a fixed 18 m/s and 30° bank, so its scores do not quantify whether those turns fit inside the available space and energy. The next replay needs local density for speed and bank selection, with the corresponding turn radius, time and energy.

## Level-search endurance at 2,000 m

With the kit carried throughout and the existing usable battery fraction and reserve, the compact XPS SiCore model gives 3.38 h warm, 3.04 h with the pre-warmed planning factor and 2.70 h with the cold-soaked planning factor. The mixed core gives 3.42, 3.08 and 2.73 h. The compact XPS P50B model gives 2.69, 2.42 and 2.15 h. These are steady-level values at 18 m/s with the planning battery factors. Terrain climbs, transit, maneuvering and low-temperature voltage sag can erode the small pre-warmed SiCore margin. [Cold battery assumptions](2026-11-12-cold-batteries.md)

Reproduce with `uv run python -X utf8 scripts/wing_operating_limits.py`. The script writes `out/wing_trade/operating_limits.csv` and `out/wing_trade/route_turn_screen.csv`.
