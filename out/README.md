# Outputs

Every folder is written by one script in `scripts/` (named in its run log or in the note that cites it). Earlier
design rounds are kept because later scripts read them: v1 builds on the v2 sizing, the v3 propulsion and the
archived terrain runs, and `verify_*.py` replays them to check the code has not drifted.

Not in the repository (rebuilt by rerunning the script): FE solver working folders (`fea/ccx_*`), step 2 trade
caches, smoke and subset runs, and FreeCAD working files.

## v1 (revision v1.1): the released design

| Folder | Contents | Script |
|---|---|---|
| `report/` | Design report, HTML and PDF | `report_pdf.py` |
| `parts_list/` | Parts cost and mass line tables behind `docs/parts_list.md` | `parts_list_v1.py` |
| `cad_v4/` | FreeCAD assembly (STEP, STL), mass and CG, fit checks, renders | `cad_v4.py` |
| `wing_trade/` | Wing sizing (`summary_v1.csv`), aircraft mass balance, performance table, and every fleet mission run (`flyable_fleet_pursuit_1_v1.1_*`) | `wing_trade.py`, `fleet_flyable.py` |
| `step2_results/` | Headline tables P1, P2, P5 and the held-out v1 vs v1.1 confirmation | `step2_results.py` |
| `step2_trades*/` | Improvement trades on the tuning seed | `step2_trades.py` |
| `performance_v1/` | Endurance Monte Carlo, wind penetration and dash (P3, P4, P6, P7) | `performance_v1_metrics.py` |
| `cost_v1.1/` | Unit cost, cost per hour and per expected help (P8) | `cost_v1_1.py` |
| `fea/` | Wing FE margins, load cases, modes, landing and joint checks | `fea_wing.py`, `fea_extra.py` |
| `flutter_v2/` | p-k flutter cases and sweeps | `flutter_v2.py` |
| `controls/` | CG envelope, hinge moments, roll, lateral modes | `controls_sizing.py` |
| `propulsion_margins/` | Motor, prop and battery margins at every corner | `propulsion_margins.py` |
| `sixdof_v1/` | JSBSim trim, turbulence, kit release, motor-out, launch, landing | `sixdof_v1.py` |
| `safety/` | Glide reach, link budget, hazard tables | `system_safety.py` |
| `mass_growth/` | Mass line list with growth allowance | `mass_growth.py` |
| `drop_v1/` | Kit-drop error budget at the v1.1 run-in speeds | `drop_speed_v1.py` |
| `detection_check/` | Detection model consistency and calibration cases | `detection_check.py` |
| `aero_crosscheck/` | AVL and OpenVSP cross-check of the aero model | `aero_crosscheck.py` |
| `structures/` | Spanwise structural sizing | `structures.py` |
| `_v1.0_freeze/` | Snapshot of the v1.0 freeze outputs (2026-11-14/15), for comparison with v1.1 | — |

## Earlier design rounds (inputs to v1, kept for replay)

| Folder | Round |
|---|---|
| `sizing/`, `stability/`, `vsp/`, `avl_debug/`, `smoke/` | Milestones 0–3: first sizing, stability, OpenVSP model (2026-11-11) |
| `v2/`, `mission/`, `cad/`, `render/`, `jsbsim/` | Design v2: mission Monte Carlo, CAD, renders, kit release (2026-11-11) |
| `v3/`, `cad_v3/`, `cost/`, `search/`, `kit_drop/`, `drop/` | Design v3: propulsion, blur, search patterns, cost, drop accuracy (2026-11-12) |
| `v4/` | Wide-angle camera and 7 m/s chute upgrade (2026-11-12) |
| `terrain*/` | Real-terrain mission runs and their sensitivity cases (2026-11-13/14) |
