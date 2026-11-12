# Lodestar: project plan

Started 2026-11-11 (US Central).

A civil engineering design study. Virtual only, free tools, public data.

## The pitch
When a hiker goes missing, search teams can lose hours in the gap between "missing" and "found". Small drones with thermal cameras already find people (DJI counts 1,000+ people rescued with drones since 2017). But quadcopters fly for 30-45 minutes, and today's fixed-wing search drones manage about 90 minutes and cost around $45K (eBee VISION).

**Lodestar** is a small, B-2-style flying wing with three jobs:
1. **Search for longer.** Fly at least 3 hours, twice what today's fixed-wing search drones manage, scanning terrain with a thermal camera.
2. **Drop a survival kit.** Once the person is found, drop a 1.5 kg kit (radio, water, thermal blanket, light) by parachute close to them, so they can talk to rescuers and stay warm until a team arrives.
3. **Come home.** Recover by belly landing and fly again.

Headline numbers: **search area per flight** (km² per sortie) and **time to first contact** with the lost person.

## Scope check (done up front)
This is a civil search-and-rescue and wildfire-scouting aircraft. All design work is in scope: sizing, stability, CFD, CAD, simulation, renders and cost. The project stays civil: no military targeting or weapons features.

## Requirements v1
| Item | Value | Why |
|---|---|---|
| Endurance | ≥ 3 h, carrying the kit the whole time | 2× eBee VISION (90 min). Conservative: assumes the kit is never dropped |
| Search speed | 14-20 m/s | Slow enough for the thermal camera, fast enough to hold course in wind |
| Wind | fly against 12 m/s (about 27 mph) at max level speed ≥ 22 m/s | Mountain winds; must make progress home |
| Search altitude | 120 m (400 ft) above ground | US FAA Part 107 limit |
| Sensor | dual thermal + RGB gimbal, 0.6 kg budget | Class of FLIR Boson 640 dual-sensor gimbals |
| Rescue kit | 1.5 kg kit + its parachute, dropped from 60-120 m | Handheld radio, 0.5 L water, foil blanket, light, first-aid basics |
| Kit landing | ≤ 6 m/s, within about 25 m of the person | Gentle enough for a radio; close enough to reach on foot |
| Stall speed | ≤ 12 m/s | Hand or bungee launch, belly landing |
| Size | span ≤ 3.0 m in three pieces that fit in a car | v1 said 2.5 m; sizing showed no 2.5 m design reaches 3 h with a 12 m/s stall (see milestone 1) |
| Mass | MTOW ≤ 10 kg | Well under the FAA 25 kg small-drone limit |
| Shape | tailless swept flying wing with winglets | The look, plus low drag |
| Balance | stable with and without the kit | The balance point moves when 1.5 kg drops away |
| Cost | ≤ $8K in parts | A fraction of eBee VISION's about $45K |

## Current design (v2)
Same airframe as v0.1 (3.0 m span flying wing, 1.29 m² of wing, 0.25 m winglets, 3° washout, centre pod 0.82 m long, stall 11.8 m/s). Balance point and kit bay at x = 0.422 m; battery packs at x = 0.46 m.

**v2 changes:**
- **Real cells in the same 30 slots (6S5P, 21700 format):**
  - Molicel P50B (in production): 540 Wh, 7.3 kg, **3.0 h** in the design case (2,000 m, 10% reserve, halfway wing drag), 3.7 h published-style.
  - Amprius SiCore 6.3 Ah (evaluation samples): 680 Wh, **3.8 h**, 4.6 h published-style.
- **Mission-swappable kit bay:** the bay sits on the balance point, so it takes either the 1.7 kg rescue kit or an 18-cell battery module, with no retrim. Search-only: 5.1 h (P50B) or 6.3 h (SiCore) in the design case, 7.7 h published-style.

**Head to head:** in simulated missions, the survival kit reaches the person within 3 h in 39% of missions, against 17% for the best fielded competitor, whose ground team still has to walk in.

**Milestone 5:** the CAD forced the wing packs 8 mm forward (pack centre x = 0.452 m) and the flight controller 50 mm aft; the kit's parachute now sits on top of the kit. Kit release in JSBSim is a non-event in calm air (autopilot holds height within 0.7 m).

**Design v3 (2026-11-12):** same airframe and cells, three changes:
1. Matched propulsion: Aeronaut CAM 15x10 folding prop and T-Motor AT3520-550, 63.6% battery-to-thrust efficiency (v2 assumed 55%), from measured prop data.
2. Moulded, smooth wing skins.
3. Search and drop at 18 m/s.

Endurance in the design case is **3.6 h** (v2: 3.0 h); 4.5 h with SiCore cells. Help within 3 h in the lost-hiker scenario: **44%** (v2: 35%; best fielded drone: 17%); 50% with SiCore at 20 m/s. Airfoil (MH45), wing area and span stay: nothing better passed the checks, and a 3.4 m span doesn't move the score. The CAD still fits and balances (prop clears the wing by 40 mm, balance point 0.3 mm from design), and AVL stability passes in every case.

**After the cost model (milestone 6):** the recommended build is **SiCore cells, foam-core wing (no moulds), search at 20 m/s**. It costs about $7,760 per aircraft and gets help to the person within 3 h in 50% of lost-hiker missions. Moulded skins add nothing once the SiCore cells give spare energy. They only pay off with P50B cells and a fleet of 25 or more. If SiCore cells can't be bought (they are evaluation samples today), the fallback is P50B, foam-core, 18 m/s: $7,320 and 40%.

Details: `research/2026-11-12-cost-model.md`, `research/2026-11-12-v3-improvements.md`, `research/2026-11-11-cad-renders-6dof.md`, `research/2026-11-11-first-sizing-and-kit-drop.md`, `research/2026-11-11-stability-and-trim.md`, `research/2026-11-11-3d-model-and-aero.md`, `research/2026-11-11-v2-and-mission-sim.md`.

## The core engineering problems
1. **Endurance vs. weight.** More battery means more flight time but also more weight, more drag, a faster stall. There is a sweet spot; finding it is milestone 1.
2. **Stability without a tail**, both loaded and after the kit drops. Reflexed airfoil, sweep plus washout, winglets, elevons.
3. **Hitting the person with the kit.** The kit drifts with the wind while it falls, so the drone must release it upwind at the right point. This is the "computed air release point" problem that cargo aircraft solve.

## Milestones
| # | Milestone | Output | Tools |
|---|---|---|---|
| 0 | Prior art, requirements v1, tools | **Done 2026-11-11**: toolchain reused, smoke test 8/8 | web research |
| 1 | First sizing | **Done 2026-11-11**: 3.0 m span, 7.4 kg, 2.55 kg battery, 3.0 h at 15 m/s | AeroSandbox + NeuralFoil |
| 1b | Kit drop study | **Done 2026-11-11**: 1.5 m cross-shaped chute, 204 g; release at about 40 m puts 95% within 25 m | Python |
| 2 | Airfoil and stability, loaded and empty | **Done 2026-11-11**: stable and trimmed, 12.5-22 m/s, kit on and off; 0.25 m winglets, 6% static margin, kit bay on the balance point | AVL, AeroSandbox VLM |
| 3 | 3D model and aero | **Done 2026-11-11**: OpenVSP model with pod; pod moves the neutral point up to 15 mm forward, so the balance point moves to x = 0.422 m; winglets still pass; endurance 2.6-3.1 h depending on wing finish | OpenVSP, VSPAERO, AVL, AeroBuildup, Blender |
| 4 | Mission simulation + competitor benchmark | **Done 2026-11-11**: design v2 (better cells, swappable bay); 2,000-mission Monte Carlo against 5 real drones with the same camera and winds; help on scene within 3 h in 39% vs 17% | Python (energy and wind-triangle model; JSBSim 6-DOF still to do, moved to milestone 5) |
| 5 | Detailed CAD, renders, 6-DOF check | **Done 2026-11-11**: STEP assembly (31 parts), all parts fit with ≥1.5 mm skin clearance, balance point from the CAD within 1.7 mm of design; 5 Blender renders; JSBSim kit release: ≤0.7 m height change in calm air, no stalls in light turbulence; operating rules: fly the drop at 18 m/s in moderate turbulence, don't descend into sustained sink | build123d, Blender, JSBSim |
| 5b | Design v3: improvements before cost | **Done 2026-11-12**: matched folding prop + motor from measured data, smooth skins, 18 m/s search; 3.6 h design case (was 3.0 h); help within 3 h 44% (was 35%); airfoil, area and 3.4 m span checked and rejected | NeuralFoil, UIUC prop data, AVL, build123d, mission sim |
| 6 | Cost model | **Done 2026-11-12**: about $7,300 per aircraft ($4,800 parts + airframe), $26 per flight hour, a 3-h search about $80 vs $3,600+ for a sheriff helicopter; best value is SiCore cells + foam-core wing at 20 m/s (50% help within 3 h for +$430; 45% once camera blur is counted, row 7b) | Python |
| 7 | Write-up | **Done 2026-11-12**: one-page report with renders and charts, `out/report/index.html`, Project renamed from Rescue Wing to Lodestar the same day | HTML page |
| 7b | Camera blur | **Done 2026-11-12**: Johnson-criteria detection with thermal-camera blur (7-12 ms) on every aircraft. Scores drop 2-5 points; best speeds unchanged (20 m/s SiCore, 18 m/s P50B); lead holds, 45% vs 15% help within 3 h. Cost trades now include blur. `research/2026-11-12-camera-blur.md` | Python |
| 7c | Search pattern | **Done 2026-11-12**: ISRID lost-hiker statistics (via SAREnv) and a trailhead launch. Trails first to 6 km, then an expanding square: help within 3 h 20% -> 32%; help within 1 h 21% vs 8-11% for drones that only report. `research/2026-11-12-search-pattern.md` | Python |
| 7d | PDF | One tall-page PDF of the report with every image, `out/report/Lodestar_report.pdf` (`scripts/report_pdf.py`, headless Edge) | Edge |
| 7e | Drop accuracy | **Done 2026-11-12**: full error budget for the 40 m kit drop (camera fix, map height, wind estimate, gusts, shear, timing, canopy). 92% of kits land within reach; headline 45% -> 43%. A 7 m/s chute (1.1 m, 60 g lighter) wins back to 44%. Rangefinder +1 point; live wind reading doesn't help. `research/2026-11-12-drop-accuracy.md`, `scripts/drop_accuracy.py` | Python |

## Folder layout
- `PLAN.md`: this file
- `research/`: dated research notes
- `scripts/`: Python scripts (`smoke_test.py` checks every tool; `sizing.py`, `kit_drop.py`, `stability.py`, `vsp_model.py`, `pod_np_check.py`, `pod_avl_check.py`, `render_preview.py`, `design_v2.py`, `mission_sim.py`, `cad_model.py` (`v3` argument for design v3), `render_v2.py`, `sixdof_kit_release.py`, `v3_propulsion.py`, `v3_airfoil.py`, `v3_planform.py`, `v3_mission.py`, `v3_search_speed.py`, `cost_model.py`, `cost_trades.py`, `v3_blur.py`, `v3_blur_fair.py`, `search_patterns.py`, `report_pdf.py`; `avl_run.py` drives AVL; `blender_scene.py` and `blender_v2.py` run inside Blender)
- `out/cad/`: STEP assembly and per-part STL of design v2
- `out/`: generated results
- Tools are shared in `../aero-tools/` (OpenVSP, AVL, flow5, Blender)
- Python: `uv sync`, then `uv run python scripts/...` (Python 3.13)

## Related
`../flying-wing-interceptor/` (shelved): its parachute model is reused for the kit drop.
