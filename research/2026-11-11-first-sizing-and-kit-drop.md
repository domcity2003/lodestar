# Milestones 0, 1 and 1b: prior art, first sizing, kit drop

2026-11-11 (US Central). Model and analysis by Claude Opus 5.5 (claude-opus-5-5).
Code: `scripts/sizing.py`, `scripts/kit_drop.py`. Outputs: `out/sizing/`, `out/kit_drop/` (CSVs, charts, `run_log.txt`).

## Answer first
**Rescue Wing v0.1:**
- **Airframe:** 3.0 m span flying wing, 1.29 m² of wing, about 7.4 kg at take-off.
- **Battery:** 2.55 kg Li-ion (510 Wh).
- **Endurance:** 3.0 h at 15 m/s, carrying a camera and a 1.5 kg rescue kit the whole time.
- **Search area:** about 7 km² per flight with a thermal camera at 120 m.
- **Kit drop:** the kit lands under a 1.5 m sewn cross-shaped parachute (204 g) at 5 m/s. Released 40 m above the person, 95% of kits land within 25 m.

Three findings drive the design:
1. **The 2.5 m span in requirements v1 can't reach 3 hours.** A 12 m/s stall speed in thin mountain air limits how heavy the wing can be for its size. A 2.5 m wing is then too small to carry enough battery: the best possible is 2.5 h. A **3.0 m span** (three pieces: a centre section plus two outer panels of about 1.1 m) gets there. The span limit is the constraint that sizes the aircraft.
2. **The design is robust.** Changing any single assumption within reason moves take-off mass between 6.6 and 9.2 kg, but every case still closes at 3.0 m span.
3. **Kit accuracy is set by how well the drone knows the wind, not by the parachute.** The aiming error equals the wind-estimate error × fall time. So the drop should be a low pass, about 40 m above the person, after the drone has circled to measure the wind.

## Prior art (benchmarks)
| System | Type | Endurance | Price | Note |
|---|---|---|---|---|
| EagleNXT (ex-senseFly) eBee VISION | hand-launched fixed wing, thermal + RGB | up to 90 min | about $44,900 (reseller) | Closest benchmark: backpack portable, FLIR Boson 640 |
| eBee X + Duet T | mapping wing, thermal option | up to 90 min (endurance battery) | n/a | Mapping, not live search |
| Autel Dragonfish | VTOL fixed wing | 158 min (maker claim) | n/a | Lists fire detection as a use |
| Draganfly Sentinel | VTOL fixed wing | up to 90 min | n/a | |
| Typical SAR quadcopter | multirotor | 30-45 min | | Most rescues so far are with these |

Demand: DJI has tracked drone-assisted rescues since 2017 (1,000+ people by its July 2023 count). In 2025-26, AI-assisted thermal drones found missing hikers in Spain, Italy, California, Oregon and Australia. **Rescue Wing's edge:** 2× the endurance of the eBee VISION, plus the ability to *deliver* something once the person is found.

## Method
**Sizing (`sizing.py`):** a sweep of 13,834 designs: span 1.6-3.4 m, aspect ratio 5-11, battery 0.5-5 kg. For each design:
- **Weight:** fixed items + wing (1.0 kg/m²) + battery + motor. The motor is sized for 22 m/s top speed or a 3 m/s climb, and its weight feeds back in until the numbers settle.
- **Drag:**
  - airfoil drag from **NeuralFoil** on the MH45, a reflexed flying-wing airfoil, at the actual Reynolds number;
  - induced drag at span efficiency 0.80, since washout costs some efficiency;
  - camera ball and pod drag (0.004 m²).
- **Power and endurance:** power = drag × speed / 0.55 + 25 W of electronics. Endurance is computed at the best allowed speed, at least 1.25 × stall speed and 14-20 m/s.
- **Design air:** 2,000 m altitude (density 1.005 kg/m³), the mountain case.

**Cross-check:** AeroSandbox **AeroBuildup** on a real 3D planform (taper 0.45, 25° sweep, 3° washout) gives L/D 19.5 against the sizing model's 19.0 (+3%). That implies 3.07 h instead of 3.00 h.

**Kit drop (`kit_drop.py`):** the parachute is sized for the landing speed (same handbook method and parachute data as the shelved pod study). A 2D simulation of the fall gives the fall time. In steady wind the fall through the air is the same whatever the wind, so the miss distance = wind-estimate error × fall time. A 20,000-run Monte Carlo matches that formula (R95 30.5 m vs 30.7 m).

## Results
### Requirement trade: best endurance reachable (h)
| Span cap | stall ≤ 12 m/s | stall ≤ 13 m/s | stall ≤ 14 m/s |
|---|---|---|---|
| 2.5 m | 2.54 | 3.04 | 3.39 |
| 2.8 m | 3.35 | 3.67 | 3.80 |
| **3.0 m** | **3.80** | 3.98 | 4.06 |
| 3.2 m | 4.06 | 4.20 | 4.30 |

Two ways to reach 3 h: allow a 3.0 m span, or accept a 13 m/s stall at 2.5 m. **I chose the 3.0 m span:** a slower stall makes hand launch and belly landing safer, and three pieces still fit in a car.

### Design point
| | |
|---|---|
| Span / wing area / aspect ratio | 3.0 m / 1.29 m² / 7.0 |
| Root / tip chord | 0.59 m / 0.27 m (taper 0.45, 25° sweep, 3° washout) |
| Take-off mass | 7.44 kg (battery 2.55, wing 1.29, motor system 0.28, fixed items 2.95, misc 0.37) |
| Stall / search / top speed | 11.9 / 15.0 / ≥ 22 m/s (at 2,000 m) |
| L/D at search speed | 19.0 (AeroBuildup 19.5) |
| Power at search / max | 130 W / 460 W |
| Battery | 510 Wh Li-ion, 77% usable (85% depth of discharge, then 10% landing reserve) |
| Endurance | 3.0 h |
| Thermal coverage | 32° lens at 120 m: 69 m swath, 10.8 cm per pixel (a person is about 4 pixels), about 3 km²/h, about 7 km² per flight |

### Sensitivity (whole sizing rerun with one change)
| Change | Take-off mass |
|---|---|
| Baseline | 7.44 kg |
| Wing 0.8 / 1.3 kg/m² | 6.86 / 8.71 kg |
| Battery 170 / 240 Wh/kg | 9.24 / 6.55 kg |
| Span efficiency 0.70 | 7.99 kg |
| Body drag × 2 | 8.10 kg |
| 3D CLmax 0.70 instead of 0.80 | 8.15 kg |
| Propulsion efficiency 0.50 | 8.04 kg |
| Sea level instead of 2,000 m | 6.68 kg |

Battery energy density matters most. Every case stays under the 10 kg take-off mass target.

### Kit drop
| | Cruciform (pick) | Annular (Iris-class) |
|---|---|---|
| Size for 5 m/s landing | 1.50 m across | 0.88 m |
| Mass incl. release | 204 g | 140 g |
| Opening load at 15 m/s | 256 N (15 g on the kit) | 246 N |

The 204 g result matches the 200 g the sizing assumed, so the two studies agree.

| Release height | Fall time | Drift in 10 m/s wind | 95% land within (1 m/s wind error) |
|---|---|---|---|
| 40 m | 8.5 s | 85 m | 21 m |
| 60 m | 12.6 s | 126 m | 31 m |
| 120 m | 24.6 s | 246 m | 60 m |

Without aiming upwind, the kit would drift by about the length of a football field. With aiming, accuracy depends on the wind estimate: 0.5 m/s error allows release from 90 m, 1 m/s needs 40 m, and 2 m/s can't hit the 25 m goal from 30 m or higher.

## Charts
- `out/sizing/endurance_vs_battery.png`: more battery means more endurance, with diminishing returns
- `out/sizing/power_vs_speed.png`: where the power goes (induced drag dominates at search speed)
- `out/sizing/mtow_vs_span.png`, `out/sizing/planform.png`
- `out/kit_drop/accuracy_vs_height.png`: lower release means better aim
- `out/kit_drop/chute_mass_vs_landing_speed.png`

## Caveats
1. **Wing structure at 1.0 kg/m² is the softest number.** It covers structure, servos, elevons and winglets but isn't calibrated to a real airframe. The sensitivity range is 6.9-8.7 kg.
2. **The 3D CLmax of 0.80 is assumed**, against NeuralFoil's 2D value of about 1.23. Milestone 2 (AVL) should check it, along with trim: with reflex, the elevons hold the nose up, which lowers usable lift.
3. **Not checked yet:** balance (loaded and empty), winglet sizing, or the trim drag of the elevons. These are milestone 2.
4. **Endurance assumes calm air and a constant 15 m/s.** Real search patterns in wind, plus climbs, will cut it. Milestone 4's mission sim will measure that.
5. **Kit drop:** uniform wind only. Low-level wind shear, trees and terrain will add error, and a 40 m pass over a person needs terrain awareness. FAA rules on dropping objects (Part 107.23(b): no hazard to people or property) apply.
6. **Detection:** about 4 pixels on a person is enough to detect someone in the open. Tree cover cuts detection a lot. Search coverage is an upper bound.

## Sources
- [EagleNXT eBee VISION](https://eaglenxt.com/products/drones/ebee-series/ebee-vision/); [DroneNerds listing (price)](https://www.dronenerds.com/products/sensefly-ebee-vision-us-ndaa-with-backpack); [eBee Vision datasheet](https://www.targikielce.pl/storage/file/core_files/2023/7/13/9b773b77d877453ab8b0208163fdaa8f/eBee-Vision.pdf)
- [AgEagle eBee X brochure](https://ageagle.com/wp-content/uploads/2025/05/eBee-X-2025-brochure-US-LETTER.pdf)
- [Autel Dragonfish](https://www.directindustry.com/prod/autel-robotics/product-182411-2723624.html); [Draganfly Sentinel spec sheet](https://draganfly.com/wp-content/uploads/2023/04/Draganfly-Sentinel-Specifications-Sheet.pdf); [US wildfire agencies' approved UAS list](https://ftp.wildfire.gov/public/incident_specific_data/n_rockies/IncidentAviationManagers/UAS/Attachment%202-%20Federally%20Approved%20UAS%20Aircraft%20and%20Specifications.pdf)
- [DJI rescue statistics](https://www.dji.com/media-center/announcements/dji-records-more-than-1000-people-rescued-by-drones-globally); [Fire and Rescue NSW, AI drone finds hikers (2026)](https://www.fire.nsw.gov.au/media/news/2026/20260625-fire-and-rescue-nsw-drone-team-locates-missing-hikers-using-ai-technology); [DroneXL, Oregon rescue](https://dronexl.co/2026/01/03/drone-rescue-lost-hiker-oregon); [Air Med & Rescue, missing-person search](https://www.airmedandrescue.com/latest/long-read/art-and-science-missing-person-searches)
- Parachute data and opening-force method: see `../flying-wing-interceptor/research/2026-11-11-pod-trade-study.md` (Knacke via Nakka Appendix K; Fruity Chutes Iris data)
