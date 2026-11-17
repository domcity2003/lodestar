# Lodestar v1: requirements compliance and certification basis

2026-11-14, updated 2026-11-15 after the wing FE verification

> Superseded for revision v1.1 (the released v1) by [the v1.1 compliance note](2026-11-16-v1.1-requirements-compliance.md). This note records the v1.0 freeze.

## Answer

Lodestar v1 meets **11 of 13** top-level requirements by analysis, with the operating limits listed below. It carries **three accepted deviations**:
- the kit lands at 7 m/s against the ≤ 6 m/s target;
- the kit is released at 40 m AGL against the original 60–120 m band;
- the wing budget is reset from 1.29 kg to 2.00 kg.

Two late analyses changed the hardware:
- **Flutter:** each elevon needs a 60 g mass balance, so the packs moved forward to hold the static margin.
- **Wing FE model:** the beam sizing was unconservative on the EPS skin. The wing now has a principal-stress cap sizing, a centre-section upper-skin doubler and a 40 mm crushable belly skid. Six construction details are mandatory.

Every structural margin is now positive, and the wing is 1.983 kg.

The **P50B fallback pack does not reach 3 h** with the foam-core wing, so SiCore is the baseline pack. Every number below comes from a script that a test or `freeze_check.py` keeps in step with the frozen parameter file `scripts/lodestar_v1.py`.

Status key:
- **Met:** shown by analysis with margin.
- **Met, limited:** met inside a stated operating limit or with a stated fix.
- **Deviation:** the requirement is changed, with the reason recorded.
- **Open:** needs a test before the claim can be made.

## 1. Top-level requirements

**Table 1.** Requirement vs. v1 value, by requirement (PLAN.md Requirements v1).

| # | Requirement | v1 value | Status | Evidence |
|---|---|---|---|---|
| R1 | Endurance ≥ 3 h, kit carried throughout | SiCore: 3.09 h at 20 m/s, 2,000 m (conservative wing-drag model); 3.68 h at best speed, +20 °C. P50B: 2.70 h / 2.94 h | **Met, limited** (SiCore). Below 0 °C, packs are held at 15 °C or above and the planner drops to ≤ 18 m/s. P50B is not compliant with the foam-core wing | `out/wing_trade/performance_v1.csv`; `out/propulsion_margins/margins.csv`, `heater.csv` |
| R2 | Search speed 14–20 m/s | 18–20 m/s; 18 m/s minimum in moderate turbulence | **Met** | controls note §gusts; `lodestar_v1.OPERATIONS` |
| R3 | Penetrate a 12 m/s wind; max level speed ≥ 22 m/s | 29.5 m/s worst corner (SiCore), 35.0 m/s (P50B), with a 45 A current limit | **Met** | `out/propulsion_margins/margins.csv` |
| R4 | Search altitude 120 m AGL | Part 107 400 ft limit observed; terrain-following route planner | **Met** | mission-sim QA note; [14 CFR 107.51](https://www.law.cornell.edu/cfr/text/14/107.51) |
| R5 | Sensor: dual thermal + RGB gimbal, 0.6 kg | Carried in the mass balance at 0.6 kg | **Met** | `out/wing_trade/aircraft_mass_balance_v1.csv` |
| R6 | 1.5 kg kit + chute, dropped from 60–120 m | 1.50 kg kit + 0.124 kg chute, released at **40 m** AGL | **Deviation D2** | `research/2026-11-12-drop-accuracy.md` |
| R7 | Kit lands ≤ 6 m/s, within about 25 m | **7 m/s** chute; 92% of kits land within reach | **Deviation D1** | `research/2026-11-12-drop-accuracy.md` |
| R8 | Stall ≤ 12 m/s | 11.4 m/s EAS at the sizing CLmax of 0.80 (12.60 m/s TAS at 2,000 m). At the cross-checked CLmax of 1.01–1.20 it is about 9.3–10.2 m/s EAS | **Met** (EAS). 5% over if read as TAS at 2,000 m with the conservative CLmax | `out/wing_trade/performance_v1.csv`; `research/2026-11-14-aero-crosscheck.md` |
| R9 | Span ≤ 3.0 m, three pieces, fits in a car | 3.0 m; centre section plus two outer panels; 20 × 16 mm carbon joiner | **Met** | `lodestar_v1.GEOMETRY`; `out/cad_v4/` |
| R10 | MTOW ≤ 10 kg | 8.31 kg SiCore loaded, including the pack thermal kit, elevon balance and skin doubler | **Met** (17% margin) | `out/wing_trade/aircraft_mass_balance_v1.csv` |
| R11 | Tailless swept flying wing with winglets | 25° sweep, 0.25 m winglets | **Met** | `lodestar_v1.GEOMETRY` |
| R12 | Stable with and without the kit | Static margin 7.1% loaded, 7.4% released (target ≥ 6%); all lateral modes Level 1; spiral stable | **Met** | `out/wing_trade/aircraft_mass_balance_v1.csv`; `out/controls/` |
| R13 | Parts cost ≤ $8K | About $7,760 as first stated, but that figure left out the nadir Boson 640 (v1 was about $12.4k). v1.1 (twin camera + safety fit): $16,418 | **Deferred** (2026-11-16 decision: performance first, cost-down pass later). Lowest cost per expected help in the comparison: $810–$1,420 | `out/cost_v1.1/`; `research/2026-11-16-step2-results.md` P8 |

**Reading R8.** A stall-speed requirement for launch and landing is an equivalent-airspeed requirement, because the wing stalls at the same EAS at any altitude. The 12.6 m/s figure is the true airspeed at 2,000 m. On that basis R8 is met even with the conservative CLmax. If a reviewer insists on TAS at the design altitude, R8 is met at the cross-checked CLmax and 5% over at 0.80. No wing enlargement is planned. The launcher is sized for the true-airspeed case instead (L3 below).

## 2. Derived and engineering requirements

**Table 2.** Engineering requirement vs. v1 value, by discipline.

| Discipline | Requirement | v1 value | Status | Evidence |
|---|---|---|---|---|
| Structure | Ultimate load = 1.5 × limit, n = +3.8 / −1.5, at VD 30 m/s EAS | Caps sized on the skin's minimum principal stress (compression + torsion shear), with a 1.10 FE correlation factor inboard; centre-section skin doubler; joiner tube 149 MPa against a 500 MPa assumed allowable | **Met** by beam analysis | `out/wing_trade/summary_v1.csv` |
| Structure | FE model: every margin of safety ≥ 0 at ultimate, for flight, landing, launch and ground cases | Lowest design margin +0.01 (wing-pod saddle, bare-EPS case; the G10 plates take it to about +10), then +0.12 (upper skin wrinkling, y = 0.71 m) and +0.19 (tube ovalization, needs hoop plies). Six details are required (F10–F15) | **Met** with the construction details | `out/fea/margins.csv`; `research/2026-11-15-wing-fea.md` |
| Structure | Belly landing at 1.5 m/s sink | A 10 mm crush stroke on hard ground fails the root skin. With the 40 mm EPP skid, every landing margin is +0.7 or better | **Met** with the skid (F10) | same |
| Aeroelastic | No flutter below 1.15 × VD (34.5 m/s EAS) | p-k method with FE-matched stiffness, final v1 wing. Servo engaged, 60 g balance: 49.4 m/s EAS (body-freedom flutter, 3–5 Hz). Free-floating elevon with the 60 g balance: ≥ 54.2 m/s. Free-floating elevon with no balance: 29.2 m/s, which **fails**. Joint at 1,000 N·m/rad: ≥ 38.1 m/s | **Met** with the elevon balance, free play ≤ 0.15°, a fail-safe non-back-drivable servo, and joint bending stiffness ≥ 1,000 N·m/rad (F7–F9) | `out/flutter_v2/`; `research/2026-11-14-flutter-verification.md` |
| Aeroelastic | No divergence or control reversal inside the envelope | No divergence; reversal about 75 m/s EAS; roll effectiveness 0.81 at VD | **Met** | same |
| Mass | Wing ≤ 2.00 kg including joints | 1.983 kg | **Deviation D3** (budget reset) | `research/2026-11-14-wing-weight.md` |
| CG | Every mass state, ± 4 mm build tolerance, inside 0.361–0.422 m | 0.4113–0.4134 m (+ 4 mm = 0.4174 m) | **Met** (4.6 mm margin at the aft limit) | `freeze_check.py`; `out/controls/summary.csv` |
| Controls | Trim within elevon travel, 12.3–30 m/s, sea level and 4,400 m | −2.5° to +1.5°; a 3.8 g pull needs −5.7° (±20° available) | **Met** | `out/controls/trim_sweep.csv`, `pullup.csv` |
| Controls | Roll to 60° within 1.3 s (MIL-F-8785C Class I Level 1) | 0.37–0.53 s | **Met** | `out/controls/roll_performance.csv` |
| Controls | Servo torque and rate | Design hinge moment 0.89 N·m; servo ≥ 0.9 N·m and ≤ 0.12 s/60° through a 2:1 linkage | **Met** with the specified servo | `out/controls/hinge_moments.csv` |
| Propulsion | Climb ≥ 3 m/s at 18 m/s, worst corner (4,400 m, −10 °C, 15% charge) | SiCore 1.66 m/s as-is; **3.03 m/s** with the pack heater plus the 25%-charge climb rule (1% margin) | **Met, limited** (fixes F1, F2) | `out/propulsion_margins/fixes_sicore_climb.csv` |
| Propulsion | Climb gradient ≥ 15% in a 30° bank | SiCore 7.8% as-is; **15.3%** with F1 + F2 (2% margin) | **Met, limited** | same |
| Propulsion | Motor and ESC current inside rating | Unlimited full throttle draws 66–86 A; the worst requirement needs 41.9 A | **Met** with a 45 A ESC limit (F3) | `out/propulsion_margins/margins.csv` |
| Launch | Release ≥ 1.1 × stall | 0.98 × stall at 14 m/s release, 4,400 m; 1.1 × needs 15.7 m/s | **Met, limited**: launcher rated ≥ 16 m/s above 3,000 m density altitude (L3) | `out/propulsion_margins/margins.csv` |
| Energy | 10 km return into a 12 m/s headwind with the landing reserve intact | SiCore +6.1 Wh spare; P50B −5.8 Wh against the fixed 10% | **Met, limited**: wind-aware bingo rule replaces the fixed reserve (L2) | `out/propulsion_margins/rth.csv` |
| Air data | Pitot reads within 2% at search speeds | Reads 4% low at the current position | **Met** with the pitot moved ≥ 230 mm ahead of the pod nose, tilted 6° down (F5) | controls note |
| Navigation | Compass interference | Single power leads put 21–36 µT at the compass | **Met** with twisted pairs (F6) | controls note |
| Packaging | Every part inside its parent with ≥ 1.5 mm skin allowance; no interference | All pass at the re-trimmed pack position: cell to sleeve 5.3 mm, cell to skin 2.7 mm after the allowance; the prop clears the wing by 40 mm | **Met** | `out/cad_v4/fit_check.csv` |

## 3. Deviations

| ID | Requirement | Change | Reason |
|---|---|---|---|
| D1 | Kit landing ≤ 6 m/s | 7 m/s | The smaller 7 m/s chute (1.1 m, 60 g lighter) drifts less, so more kits land within reach. The radio and water bottle must be packed for a 7 m/s impact. The drop test must confirm it (test T6). |
| D2 | Release from 60–120 m | 40 m AGL on a 14 m/s into-wind run | Lower release cuts wind drift; 92% of kits land within reach at 40 m. The requirement's 60–120 m band came before the drop-accuracy study. 40 m is inside the Part 107 400 ft ceiling. |
| D3 | Wing ≤ 1.29 kg | ≤ 2.00 kg including joints | The 1.29 kg figure predates the three-piece joint (0.34 kg) and the finish. The FE verification then added 0.10 kg: principal-stress caps and the centre-section doubler. At 15.4 g/dm², the v1 wing is inside the 10.5–16 g/dm² band of published comparable builds. |

## 4. Fixes and operating limits that the compliance depends on

These are carried in `lodestar_v1.STRUCTURE`, `PROPULSION`, `OPERATIONS` and `CONTROLS`, and checked by `freeze_check.py` where a number exists to check.

**Hardware changes.**
- **F1** 20 mm EPS around each wing pack plus a 5 W thermostat pad, holding the cells at 15 °C or above. About 0.10 kg, carried in the mass balance.
- **F2** A no-full-power-climb rule below 25% charge, enforced by the bingo rule (L2).
- **F3** A 45 A ESC current limit; the ESC must report RPM for the autopilot's vibration filter.
- **F4** A 2:1 elevon linkage; a servo of at least 0.9 N·m and 0.12 s/60°, at most 13 mm thick.
- **F5** The pitot extended to ≥ 230 mm ahead of the pod nose and tilted 6° nose-down.
- **F6** Twisted-pair power leads.
- **F7** A 60 g static mass balance on each elevon, at the servo station (y = 0.87 m) on a 40 mm arm ahead of the hinge, through a slot in the rear web. The added 120 g sits aft of the CG, so the wing packs moved a further 15 mm forward and 25 mm inboard (x = 0.412 m, y = 0.305 m) to hold the static margin. CAD v4, the loads and every downstream check were rerun at that layout.
- **F8** Total elevon free play ≤ 0.15°, counting the servo dead band. The servo holds position on signal loss and does not back-drive.
- **F9** Wing joint bending stiffness ≥ 1,000 N·m/rad, measured on the assembled wing (target ≥ 2,000). At 500 N·m/rad, body-freedom flutter falls below VD.
- **F10** An EPP belly skid under the pod with at least 40 mm of crush stroke.
- **F11** One extra ±45 glass ply on the centre-section upper skin (y = 0–0.45 m), dropped off over 0.45–0.50 m.
- **F12** Sleeve ends seated on two 1.5 mm ply/G10 ribs each, and sleeves bonded through a glass wrap onto the spar web and caps, not into foam alone.
- **F13** Hoop plies on the joiner tube at the sleeve mouths (ovalization margin +0.19).
- **F14** Two 60 × 40 mm G10 saddle plates where the wing sits on the pod.
- **F15** A structural lower lid on each battery bay (M3 nylon screws in G10 inserts at 30 mm pitch), and two 25 mm glass overlap tapes at each winglet joint.

**Operating limits.**
- **L1** Search at 18 m/s minimum in moderate turbulence: a 7.6 m/s gust uses up the stall margin at 15 m/s.
- **L2** Head home when the remaining energy, less the 10% landing reserve, falls to 1.2 × the return energy for the current distance and wind + 3 m/s.
- **L3** Above 3,000 m density altitude, use a launcher rated for 16 m/s release or more, and pre-warm the packs before launch.
- **L4** Launch within 30° of the wind: there is no rudder, so crosswind is handled by crabbing.
- **L5** Below 0 °C, plan the search at ≤ 18 m/s. At 20 m/s the cold-day SiCore endurance falls to 2.7–2.9 h.
- **L6** Hold the CG build tolerance to ± 4 mm, verified by a hang test after assembly.

## 5. Certification basis (tailored, awareness level)

Lodestar is a civil small UAS of about 8 kg, so it falls under the FAA Part 107 framework, not a type certificate. A defense-style program would still show a tailored airworthiness case. The matrix below maps the v1 evidence to the two references a reviewer would expect.
- **MIL-HDBK-516C** is the US airworthiness criteria handbook for manned and unmanned aircraft. It is guidance, tailored per program ([EverySpec](https://everyspec.com/MIL-HDBK/MIL-HDBK-0500-0599/MIL-HDBK-516C_52120/); [tailoring method for UAS](https://vtechworks.lib.vt.edu/handle/10919/98018)).
- **ASTM F3298-24** covers the design and construction of lightweight UAS ([ASTM](https://store.astm.org/f3298-24.html)).

The section names below are summaries of the topics, not quotations of either standard.

**Table 3.** v1 evidence by airworthiness area (tailored; reference only).

| Area | MIL-HDBK-516C topic | ASTM F3298 topic | v1 evidence | Gap before flight |
|---|---|---|---|---|
| Structures | Structural loads, strength, aeroelasticity | Structure | Beam sizing at FOS 1.5; CalculiX shell/solid FE with a 45-row margins table; V-g screen and p-k second method, including the elevon | Coupon tests T1–T3; wing proof load to limit; ground vibration test |
| Flight technology | Stability, control, performance | Performance and handling | AVL and four cross-check solvers; controls sizing; 6-DOF kit release; propulsion margins | Flight test of the trim, roll and stall points |
| Propulsion | Propulsion installation and limits | Propulsion | Motor, prop and battery margins over 13 corners | Static thrust and battery pulse tests T4, T5 |
| Electrical | Power generation and distribution | Electrical | Current limit, heater, compass interference | Bench test of the power system at 45 A |
| Mass properties | Weight and balance | Weight and balance | Spanwise mass model plus CAD v4 with mesh-integrated mass properties | Hang test for CG, ± 4 mm |
| Payload release | Stores separation | Payloads | Drop-accuracy Monte Carlo; 6-DOF release transient | Drop test T6 |
| Flight controls and software | Flight control system, software | Avionics, software | Out of scope for v1 (physical implications only) | Autopilot selection and HIL testing in v2 |
| Lost link and flight termination | Flight termination | Command and control | Out of scope for v1 | Lost-link behaviour and a terminal parachute (ASTM F3322) for BVLOS, v2 |

**Regulatory items that touch the design** (from [the regulatory sources note](2026-11-14-regulatory-sources.md)):
- **Remote ID** is required. v1 carries mass and power for a broadcast module with an FAA-accepted declaration of compliance (ASTM F3586).
- **The kit drop** must not create an undue hazard ([14 CFR 107.23](https://www.law.cornell.edu/cfr/text/14/107.23)). The drop envelope (D1, D2) and a no-drop-over-people rule document this.
- **BVLOS** needs a waiver today, because Part 108 is not final. The proposed rule's lost-link, detect-and-avoid and flight-recorder items are v2 work.
- **Parts sourcing.** Foreign-made drone components are now on the FCC Covered List, so a Blue UAS-listed or US-made parts list is the strongest design signal for defense-adjacent use. The thermal camera, radio and cells are the likely hard items. Sourcing is reviewed with the parts list in Step 4.

## 6. Tests the claims depend on

| ID | Test | Claim it closes |
|---|---|---|
| T1 | Sandwich-beam wrinkling coupon, glass on EPS 20, at the production cure pressure (≥ 14.4 MPa), in compression and in combined compression and shear | Wing mass and strength |
| T2 | XPS-to-EPS foam-transition coupon at the servo bay | Wing strength |
| T3 | Weighed 100 × 100 mm skin coupon | Wing mass |
| T4 | Static thrust of the 15 × 10 prop on the AT3520 at 45 A | Climb, launch |
| T5 | SA112 pulse tests (8 A, 30 s) at −10 / 0 / 25 °C and 15 / 25 / 50% charge | Climb fix F1/F2, endurance |
| T6 | Kit drop at 7 m/s with the packed radio and water | D1 |
| T7 | Wing proof load to limit (3.8 g) and a ground vibration test: clamped first bending near 13 Hz and torsion near 49 Hz (FE); free-free symmetric bending about 18–20 Hz and antisymmetric about 25–27 Hz; joint bending stiffness; elevon hinge stiffness and free play | Structure, flutter, F8, F9 |
| T8 | Hang test for CG after assembly | R12, L6 |

## Reproduce

From `scripts/`:
- `../.venv/Scripts/python.exe -X utf8 freeze_check.py` checks every frozen number against its source.
- `../.venv/Scripts/python.exe -X utf8 run_all_tests.py` runs every test and verify script (`--full` adds the replays, the tool-chain check and the FreeCAD helper).
