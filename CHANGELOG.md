# Changelog

Dates are US Central. Design revisions are frozen in `scripts/lodestar_v1.py` (`VERSION`).

## v1.1 (2026-11-16): v1 release

Step 2 (simulation, test and improvement). Details and evidence: [research/2026-11-16-step2-results.md](research/2026-11-16-step2-results.md).

Added
- Twin splayed FLIR Boson 640 nadir cameras: 95° combined view, 135.7 m track spacing.
- Search bank 40°.
- Aircraft parachute with an independent trigger.
- Backup avionics supply and locking connectors; BMS with a 50 °C heater cut-out; ADS-B In and strobes; second GNSS.
- KST A12-S HV elevon servos (1.35 N·m).
- 8 AWG pack leads.
- 70 mm EPP belly skid, specified by crush force; tip and winglet wear pads.
- Operating rules:
  - drop run-in 18–20 m/s;
  - full power at release above 3,000 m DA;
  - 20 m/s search above 3,200 m DA;
  - flare pitch ≤ 4.5°;
  - no full-power climb below 35% charge above 3,000 m DA.
- Mission results on held-out seeds at three detection calibrations, with paired bootstrap intervals; cost per expected help.
- Release documents:
  - drawing LOD-GA-001;
  - parts list;
  - v1.1 compliance note;
  - design report and PDF;
  - an output index.
- `freeze_check.py` mission section: each headline run's inputs are checked against the frozen design.

Changed
- Spar cap ply drops sized with a 1.25 stress factor and staggered ≥ 25 mm, because FE showed a −0.061 margin at abrupt drops. Wing 1.983 → 2.006 kg.
- Battery packs moved 5 mm aft.
- Forward CG limit 0.361 → 0.365 m.
- Take-off mass 8.31 → 8.76 kg (SiCore, kit on). Static margin 7.3% loaded, 7.7% released.
- Parts cost corrected: v1.0's $7,760 left out the nadir Boson. v1.1 is $16.9k including airframe materials. The $8k target (R13) is deferred.
- `report_pdf.py` uses headless Chrome when it is installed.

Results (nominal calibration, seed 20261115)
- Help within 3 h: 18.0%, against 14.9% for the best competitor.
- Help within 1 h: 11.6%, against 7.3%.
- On the held-out seed 20261114, v1.1 beats v1 by +3.5 pp at 1 h and +3.9 pp at 3 h.

## v1.0 (2026-11-14/15): v1 freeze

- One frozen parameter file and CAD v4, built headless in FreeCAD.
- Wing FE in CalculiX and p-k flutter.
- Controls sizing, propulsion and battery margins, a JSBSim 6-DOF model, system safety, mass growth and the verification matrix.
- `freeze_check.py` and `run_all_tests.py`.
- Notes: `research/2026-11-14-*` and `research/2026-11-15-*`.

## Design study rounds (2026-11-11 to 2026-11-13)

Milestones 0–9 in [PLAN.md](PLAN.md):
- sizing, stability, the OpenVSP model and the kit-drop study;
- design v2 and v3: propulsion, camera blur, search patterns, cost and drop accuracy;
- wing structure and real-terrain mission replays.
