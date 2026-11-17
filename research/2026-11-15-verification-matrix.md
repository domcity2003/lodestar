# Verification cross-reference matrix, Lodestar v1

2026-11-15

Step 2 gap G10 ([Step 2 plan](2026-11-15-step2-plan.md)). This note promotes the evidence column of the [compliance note](2026-11-14-v1-requirements-compliance.md) to a standalone matrix. Each requirement gets a verification method, the evidence chain (script → output → note), a status and the physical test that will close it.

## Answer

The matrix has **55 rows**:
- 13 top-level requirements (R1–R13);
- 21 engineering requirements (E1–E21: the compliance note's Table 2, plus three new ones from the [mass, power and interfaces note](2026-11-15-mass-power-interfaces.md));
- 15 hardware fixes (F1–F15);
- 6 operating limits (L1–L6).

**Table 1.** Row count vs. status, by requirement group.

| Group | Verified by analysis | Needs test | Open | Total |
|---|---:|---:|---:|---:|
| Top-level (R) | 11 | 1 | 1 | 13 |
| Engineering (E) | 9 | 11 | 1 | 21 |
| Hardware fixes (F) | 6 | 7 | 2 | 15 |
| Operating limits (L) | 3 | 2 | 1 | 6 |
| **Total** | **29** | **21** | **5** | **55** |

*Status key:*
- *Verified by analysis: the analysis closes the requirement at this design stage, and any listed test is confirmation.*
- *Needs test: the analysis rests on an assumed value (an allowable, a resistance, a stiffness, a part's performance) that only a test can replace.*
- *Open: no design or no analysis exists yet.*

**The five open rows:**
- **R13, cost:** the radio choice and the elevon servo are unsettled, and the cost margin is only 3%.
- **E10 and F4, servo:** no selected servo meets 0.9 N·m within 13 mm.
- **F2 and L2, energy rules:** the 25%-charge climb rule and the bingo rule are autopilot software, which is out of scope for v1.

**Where the open tests cluster.** Three tests between them appear against 14 of the 21 "needs test" rows, so running them first retires most of the test risk:
- **T7**, the wing proof load and ground vibration test: E2, E4, F7, F8, F9.
- **T5**, the SA112 pulse tests: E11, E12, F1.
- **T9**, a new bench test of the power system at 45 A: E13, E17, E20, E21, F3, F6.

## Methods

The methods follow the usual four. The definitions below are paraphrased, not quoted, from the [NASA Systems Engineering Handbook, NASA/SP-2016-6105 Rev 2](https://www.nasa.gov/reference/systems-engineering-handbook/), §5.3.

| Code | Method | Meaning here |
|---|---|---|
| A | Analysis | A model or calculation in a repo script, with its outputs checked by a test script or `freeze_check.py` |
| I | Inspection | Checking a drawing, the CAD model, a parameter file or the built hardware for a stated feature or dimension |
| D | Demonstration | Operating the system to show a function works, without measuring margins (e.g. a planner setting, a release sequence) |
| T | Test | Measuring a quantity under controlled conditions against a limit |

## Physical tests

T1–T8 come from the compliance note. The proposed tests (T9–T13 and FT1–FT6) are added in this note for rows no existing test closes.

**Table 2.** Planned physical test by ID, with what it measures.

| ID | Test | Measures | Status |
|---|---|---|---|
| T1 | Sandwich-beam wrinkling coupon, glass on EPS 20 | Face wrinkling allowable, compression and combined with shear | Planned (compliance note) |
| T2 | XPS-to-EPS transition coupon at the servo bay | Transition strength | Planned |
| T3 | Weighed 100 × 100 mm skin coupon | Skin areal mass | Planned |
| T4 | Static thrust, 15 × 10 on AT3520 at 45 A | Thrust, current, winding temperature | Planned |
| T5 | SA112 pulse tests at −10 / 0 / 25 °C, 15 / 25 / 50% charge | DC resistance, loaded voltage | Planned |
| T6 | Kit drop at 7 m/s with the packed radio and water | Kit survival, descent rate | Planned |
| T7 | Wing proof load to 3.8 g + ground vibration test; joint stiffness; hinge stiffness and free play | Strength, modes, stiffness, free play | Planned |
| T8 | Hang test for CG after assembly, both bay configurations; weighing | CG, mass | Planned |
| T9 | Power system bench at 45 A: ESC limit, battery-side current, lead and connector temperature in a foam mock-up, BEC load, regulator redundancy, compass field vs. current (`compassmot`) | Electrical rows | **Proposed here** |
| T10 | Servo bench: torque vs. deflection through the 2:1 linkage, stiffness, free play, rate, current, fail-safe hold | Servo spec, F4, F8 inputs | **Proposed here** |
| T11 | Launcher exit speed at full mass | Release speed ≥ 16 m/s | **Proposed here** (propulsion note, test 4) |
| T12 | Chamber soak of an insulated pack in a wing-core mock-up at −10 °C with the 5 W pad | Pack temperature, heater duty | **Proposed here** (propulsion note, test 3) |
| T13 | EPP skid crush or drop test at 1.5 m/s sink with the pod mass | Crush stroke, peak load | **Proposed here** |
| FT1 | First flights: trim, elevon at search speed, static margin by trim slope | R12, E8, L6 | **Proposed here** |
| FT2 | Handling: roll 0–60°, gust response, turns at 30° | E9, R2, L1 | **Proposed here** |
| FT3 | Performance: max level speed, climb, glide | R3, E11, E12 | **Proposed here** |
| FT4 | Endurance at 20 m/s with the kit carried | R1 | **Proposed here** |
| FT5 | Airspeed calibration (`ARSPD_AUTOCAL`) | E16, F5 | **Proposed here** |
| FT6 | In-flight kit drop at 40 m AGL | R6, R7 | **Proposed here** |

## Matrix

Script paths are under `scripts/`; outputs under `out/`; notes under `research/`. "Notes" names the research note by its topic.

**Table 3.** Top-level requirement vs. verification method, evidence and status.

| ID | Requirement | Method | Evidence (script → output → note) | v1 value | Status | Test |
|---|---|---|---|---|---|---|
| R1 | Endurance ≥ 3 h, kit carried | A, T | `wing_trade.py` → `wing_trade/performance_v1.csv`; `propulsion_margins.py` → `propulsion_margins/margins.csv`, `endurance.csv`; `performance_v1_metrics.py` → `performance_v1/endurance_summary.csv` → compliance, propulsion notes | 3.09 h at 20 m/s, 2,000 m; best-speed P50 3.52 h, P(< 3 h) 0.2% | Verified by analysis (with F1, L5) | T5, FT4 |
| R2 | Search speed 14–20 m/s | A | `controls_sizing.py` → `controls/gusts.csv`; `lodestar_v1.OPERATIONS` → controls note | 18–20 m/s | Verified by analysis | FT2 |
| R3 | Penetrate 12 m/s wind; V max ≥ 22 m/s | A | `propulsion_margins.py` → `margins.csv`; `performance_v1/wind_dash.csv` | 29.5 m/s worst corner | Verified by analysis | T4, FT3 |
| R4 | Search at 120 m AGL | A (inspection of the planner), D | `terrain_sim.py`, `mission_sim.py` → mission-sim QA note; [14 CFR 107.51](https://www.law.cornell.edu/cfr/text/14/107.51) | Terrain-following at 120 m | Verified by analysis | FT2 |
| R5 | Dual thermal + RGB gimbal, 0.6 kg | I, A | `cad_v4.py` → `cad_v4/mass_cg.csv`; `mass_growth.py` → `mass_growth/lines.csv` | 0.6 kg carried; ZT6 is 0.197 kg | Verified by analysis | T8 (weigh) |
| R6 | 1.5 kg kit + chute; release height (deviation D2: 40 m) | A | `drop_accuracy.py` → `drop/`; `sixdof_kit_release.py`; `controls_sizing.py` → `controls/kit_release.csv` → drop-accuracy note | 92% within reach at 40 m | Verified by analysis (as deviated) | T6, FT6 |
| R7 | Kit lands ≤ 6 m/s (deviation D1: 7 m/s) | A, T | `drop_accuracy.py` → `drop/` → drop-accuracy note | 7 m/s chute | **Needs test**: radio and water must survive 7 m/s | T6 |
| R8 | Stall ≤ 12 m/s | A | `wing_trade.py` → `performance_v1.csv`; `aero_crosscheck.py` → aero cross-check note | 11.4 m/s EAS at CLmax 0.80 | Verified by analysis (EAS) | FT1 |
| R9 | Span ≤ 3.0 m, three pieces | I | `lodestar_v1.GEOMETRY`; `cad_v4.py` → `cad_v4/lodestar_v4.step` | 3.0 m, 3 pieces | Verified by analysis | T8 (assembly) |
| R10 | MTOW ≤ 10 kg | A | `wing_trade.py` → `aircraft_mass_balance_v1.csv`; `mass_growth.py` → `mass_growth/summary.csv` → mass-growth note | 8.31 kg basic; 9.15 kg predicted with growth (0.85 kg margin) | Verified by analysis | T3, T8 |
| R11 | Tailless swept wing with winglets | I | `lodestar_v1.GEOMETRY`; CAD v4 | 25° sweep, 0.25 m winglets | Verified by analysis | — |
| R12 | Stable with and without the kit | A | `controls_sizing.py` → `controls/mass_states.csv`, `lateral_modes.csv`; `freeze_check.py` → controls note | SM 7.07% / 7.44%; Level 1 lateral modes | Verified by analysis | T8, FT1 |
| R13 | Parts cost ≤ $8K | A | `cost_model.py` → `out/cost/` → cost-model note | $7,760 (3% margin) | **Open**: the costed radio (HM30) differs from the radios in the safety work (RFD900x + pMDDL2450), and the costed servo does not meet the spec | — |

**Table 4.** Engineering requirement vs. verification method, evidence and status.

| ID | Requirement | Method | Evidence (script → output → note) | v1 value | Status | Test |
|---|---|---|---|---|---|---|
| E1 | Ultimate = 1.5 × limit, n +3.8 / −1.5, VD 30 m/s EAS (beam) | A | `wing_trade.py` → `wing_trade/summary_v1.csv`, `v1_sections.csv` → wing-weight note | Tube 149 MPa vs 500 MPa assumed | Verified by analysis | T7 |
| E2 | FE: every MS ≥ 0 at ultimate (flight, landing, launch, ground) | A | `fea_wing.py`, `fea_extra.py` → `fea/margins.csv`; `test_fea_wing.py` → FEA note | Lowest +0.01 (bare saddle), +0.12 wrinkling | **Needs test**: allowables are assumed | T1, T2, T7 |
| E3 | Belly landing at 1.5 m/s sink | A | `fea_extra.py` → `fea/landing_cases.csv` → FEA note | MS ≥ +0.7 with a 40 mm skid | Verified by analysis | T13 |
| E4 | No flutter below 1.15 VD (34.5 m/s EAS) | A | `flutter_v2.py` → `flutter_v2/summary.csv`; `test_flutter_v2.py`, `verify_flutter.py` → flutter note | 49.4 m/s EAS | **Needs test**: joint and hinge stiffness, free play and modes are assumed | T7, T10 |
| E5 | No divergence or reversal in the envelope | A | `flutter_v2.py` → `flutter_v2/static_aeroelastic.csv`, `roll_effectiveness.csv` | Reversal about 75 m/s EAS | Verified by analysis | T7 |
| E6 | Wing ≤ 2.00 kg incl. joints (deviation D3) | A, T | `wing_trade.py` → `summary_v1.csv`; `mass_growth.py` → wing-weight, mass-growth notes | 1.983 kg; 2.20 kg predicted with growth | **Needs test**: 17 g of room | T3, T8 |
| E7 | Every mass state ± 4 mm inside 0.361–0.422 m | A | `controls_sizing.py` → `controls/summary.csv`; `freeze_check.py`; `mass_growth.py` → `mass_growth/summary.csv` | 0.4113–0.4134 m; aft-biased growth overruns by 0.35 mm (9 g of ballast) | Verified by analysis | T8 |
| E8 | Trim inside elevon travel, 12.3–30 m/s, SL and 4,400 m | A | `controls_sizing.py` → `controls/trim_sweep.csv`, `pullup.csv` | −3.2° to +1.6°; 3.8 g pull −5.7° | Verified by analysis | FT1 |
| E9 | Roll 60° in ≤ 1.3 s (MIL-F-8785C Class I Level 1) | A | `controls_sizing.py` → `controls/roll_performance.csv` | 0.37–0.53 s | Verified by analysis | FT2 |
| E10 | Servo ≥ 0.9 N·m, ≤ 0.12 s/60°, ≤ 13 mm | A, I | `controls_sizing.py` → `controls/hinge_moments.csv` → controls note | Design hinge moment 0.89 N·m; no selected part meets it | **Open** | T10 |
| E11 | Climb ≥ 3 m/s at 18 m/s, worst corner | A | `propulsion_margins.py` → `fixes_sicore_climb.csv`; `test_propulsion_margins.py` | 3.03 m/s with F1 + F2 (1% margin) | **Needs test**: SiCore DC resistance assumed | T5, FT3 |
| E12 | Climb gradient ≥ 15% in a 30° bank | A | same | 15.3% with F1 + F2 | **Needs test** | T5, FT3 |
| E13 | Motor and ESC current inside rating | A | `propulsion_margins.py` → `margins.csv` | 41.85 A needed vs 45 A limit | **Needs test**: motor model fitted to two bench rows | T4, T9 |
| E14 | Launch release ≥ 1.1 × Vs | A | `propulsion_margins.py` → `performance.csv` | 0.98 × Vs at 14 m/s, 4,400 m; needs 15.7 m/s | **Needs test** (with L3) | T11 |
| E15 | 10 km return into 12 m/s with the landing reserve intact | A | `propulsion_margins.py` → `rth.csv`, `margins.csv` | SiCore +6.1 Wh | Verified by analysis (bingo rule L2 open) | FT3 |
| E16 | Pitot within 2% at search speed | A | `controls_sizing.py` → controls note §7 | 1.4% low at 230 mm | **Needs test** | FT5 |
| E17 | Compass interference acceptable | A | controls note §7 (straight-wire model) | 0.2–0.6 µT twisted, against 21–36 µT single | **Needs test** | T9 |
| E18 | Every part inside its parent, ≥ 1.5 mm skin allowance, no interference | I, A | `cad_v4.py` → `cad_v4/fit_check.csv`, `joint_gaps.csv` → CAD v4 note | Cell to sleeve 5.3 mm; prop to wing 40.3 mm | Verified by analysis | T8 (assembly) |
| E19 | MTOW ≤ 10 kg with mass growth allowance (new, G7) | A | `mass_growth.py` → `mass_growth/summary.csv` → mass-growth note | 9.15 kg predicted; 0.85 kg margin | Verified by analysis | T8 |
| E20 | Non-propulsion power ≤ 25 W allowance in search (new, G8) | A | `propulsion_margins.py` chain + datasheets → mass-growth note, Table 6 | 15.6 W warm, 20.6 W with the heater | **Needs test**: FC, servo and Remote ID power estimated | T9, T10 |
| E21 | Pack leads ≤ 75 °C next to EPS at 39.2 A (new, G8) | A | Mass-growth note, Table 8 (adiabatic bound) | 10 AWG ≈ 83 °C if buried; 8 AWG ≈ 57 °C | **Needs test** | T9 |

**Table 5.** Hardware fix vs. verification method, evidence and status.

| ID | Fix | Method | Evidence | Status | Test |
|---|---|---|---|---|---|
| F1 | 20 mm EPS + 5 W thermostat pad; cells ≥ 15 °C | A, T | `propulsion_margins.py` → `heater.csv`; `lodestar_v1.PROPULSION` | **Needs test** | T12, T5 |
| F2 | No full-power climb below 25% charge | D | `lodestar_v1.OPERATIONS`; autopilot logic not written | **Open** (software, v2) | HIL, FT3 |
| F3 | 45 A ESC current limit; ESC with RPM telemetry | I, T | `margins.csv`; `freeze_check.py` (limit ≥ need) | **Needs test** | T4, T9 |
| F4 | 2:1 linkage; servo ≥ 0.9 N·m, ≤ 0.12 s/60°, ≤ 13 mm | A, I, T | `controls/hinge_moments.csv`, controls note Table 6 | **Open** (no compliant servo) | T10 |
| F5 | Pitot ports ≥ 230 mm ahead of the nose, 6° nose-down | A, I | Controls note §7 | Verified by analysis | FT5 |
| F6 | Twisted-pair power leads | I, T | Controls note §7 | **Needs test** | T9 |
| F7 | 60 g static mass balance per elevon at y = 0.87 m | A, I | `flutter_v2.py` → `flutter_v2/elevon_sweep.csv`; `cad_v4/mass_cg.csv` row; `freeze_check.py` | **Needs test** (balance on the built surface) | T7 |
| F8 | Elevon free play ≤ 0.15°; non-back-drivable, holds on signal loss | A, T | `flutter_v2/freeplay.csv` | **Needs test** | T7, T10 |
| F9 | Joint bending stiffness ≥ 1,000 N·m/rad | A, T | `flutter_v2/joint_sweep.csv` | **Needs test** | T7 |
| F10 | EPP belly skid, ≥ 40 mm crush stroke | A, T | `fea/landing_cases.csv` | **Needs test** (crush curve of the actual EPP) | T13 |
| F11 | Centre-section upper-skin doubler | A, I | `fea/margins.csv`; `lodestar_v1.STRUCTURE` | Verified by analysis | T7 |
| F12 | Sleeve ribs; sleeves wrapped onto web and caps | A, I | `fea/joint_checks.csv` | Verified by analysis | T7 |
| F13 | Hoop plies on the joiner at the sleeve mouths | A, I | `fea/joint_checks.csv` (MS +0.19) | Verified by analysis | T7 |
| F14 | G10 saddle plates at the pod pads | A, I | `fea/margins.csv` (bare +0.01) and hand calculation | Verified by analysis | T7 |
| F15 | Structural battery-bay lid; winglet overlap tapes | A, I | `fea/margins.csv` (+7.1, +10.3) | Verified by analysis | T7 |

**Table 6.** Operating limit vs. verification method, evidence and status.

| ID | Limit | Method | Evidence | Status | Test |
|---|---|---|---|---|---|
| L1 | Search and drop ≥ 18 m/s in moderate turbulence | A | `controls_sizing.py` → `controls/gusts.csv`; milestone 5 6-DOF (the v1 6-DOF rerun is gap G6) | Verified by analysis | FT2 |
| L2 | Wind-aware bingo rule | A, D | `propulsion_margins.py` → `rth.csv` | **Open** (autopilot software) | HIL |
| L3 | Launcher ≥ 16 m/s release above 3,000 m DA; packs pre-warmed | A, T | `propulsion_margins/performance.csv` | **Needs test** | T11 |
| L4 | Launch within 30° of the wind | A | `controls/crosswind.csv` | Verified by analysis | FT1 |
| L5 | Below 0 °C, search ≤ 18 m/s | A | `propulsion_margins/endurance.csv` | Verified by analysis | FT4 (cold day) |
| L6 | CG build tolerance ± 4 mm | T | `freeze_check.py`; `mass_growth/summary.csv` | **Needs test** | T8 |

## Gaps this matrix exposes

1. **Requirements with no physical test before first flight:** R4, R11 and E15. They are closed by analysis and inspection, which is acceptable at this stage.
2. **Software-dependent rows (F2, L2, part of D3 in the interface list).** v1 scopes the autopilot out, so these stay open until autopilot selection and HIL in v2. The FHA (G1) should rate their hazards on the assumption they are absent.
3. **Rows whose margin is 3% or less:**
   - E6, the wing mass: 17 g;
   - E11 and E12, the climb fixes: 1% and 2%;
   - E7 under aft-biased growth: −0.35 mm;
   - R13, the cost: 3%.

   These are the first candidates for Step 3 design changes if a test comes in low.
4. **Evidence that is a note rather than a checked script output:** E16, E17 and F5 (pitot and compass, controls note §7), and E20–E21 (hand arithmetic). A small test file would make them reproducible. They are not checked by `freeze_check.py` today.

## Reproduce

- Status counts: Tables 3–6 of this note.
- Evidence scripts:
  - `scripts/run_all_tests.py` runs the test and verify scripts;
  - `scripts/freeze_check.py` checks the frozen parameters;
  - `scripts/mass_growth.py` checks the growth allowance.
