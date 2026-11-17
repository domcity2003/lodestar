# Mass growth, power budget, interfaces and operating envelope

2026-11-15

Step 2 gaps G7, G8, G12 and G13 ([Step 2 plan](2026-11-15-step2-plan.md)). The verification matrix (G10) is in [its own note](2026-11-15-verification-matrix.md).

## Answer

- **Mass (G7).** With a standard growth allowance on every line, Lodestar v1 is predicted at **9.15 kg** loaded (SiCore). That is 8.31 kg of basic mass plus a 0.83 kg allowance (10.0% of basic). The margin to the 10 kg MTOW requirement is **0.85 kg (8.5% of the limit)**. R10 still holds, but the 17% margin quoted so far is a margin on basic mass, and half of it is growth that has not happened yet.
- **CG under growth.** If every line grows where it sits, the CG moves only 0.1 mm and the static margin stays at **7.0%**. If only the lines aft of the CG grow, the CG moves 5.0 mm aft and the static margin falls to **5.96% loaded and 5.94% released**, just under the 6% target and still above the 5% aft-limit basis. Loaded, the CG plus the 4 mm build tolerance then overruns the aft limit by 0.35 mm. Nine grams of nose ballast fixes that. The aft limit, not the 10 kg limit, is the mass property with least room: only **0.17 kg** more can go at the motor station before the toleranced CG reaches it.
- **The 0.6 kg gimbal line is really nose ballast.** The costed gimbal (SIYI ZT6) weighs 0.197 kg. If the budget drops to that, the CG moves 16 mm aft, out of the envelope, and the static margin falls to 3.5%. About 0.29 kg has to stay in the nose, as ballast or as a forward pack shift, whichever gimbal is chosen.
- **Power (G8).** The non-propulsion loads add up to **about 16 W in warm search and 21 W with the 5 W pack heater on**. The performance model carries a flat 25 W, so it is conservative for most of the flight. Short phases reach 30–37 W: the turn and kit drop with the gimbal slewing, and the return at full radio power. The battery peak is **39 A at full power** under the 45 A motor-current limit, and **7.9 A in search at 20 m/s**.
- **Wiring (G8).** The pack-to-ESC run needs **10 AWG kept off the EPS, or 8 AWG if it runs through the foam**. In a 3 m/s climb on a +40 °C day, 10 AWG warms about 43 K with no cooling. That puts the conductor near 83 °C, above the roughly 75 °C long-term limit of EPS.
- **Interfaces (G12).** 34 interfaces are listed. Six have no design yet:
  - the wing-to-pod fastening;
  - the elevon servo (no part meets the spec);
  - the launcher shoe;
  - the servo connector and the antenna connector at the wing joint;
  - the paralleling of the bay battery module with the wing packs.
- **Operating envelope (G13).** One page, in Table 12. Two limits are new:
  - Above about **3,200 m density altitude**, 30° turns at 18 m/s lose the 1.25 × stall margin, so search at 20 m/s there or limit the bank.
  - Precipitation and icing are **not cleared** at all. The aircraft has no heated pitot and no ingress rating.

Script: `scripts/mass_growth.py`, which writes `out/mass_growth/`. Its 16 self-checks all pass. The basic mass matches `lodestar_v1.MASS` within 1 g in all four states, and the basic CG matches the mass balance within 0.1 mm. The power budget is hand arithmetic on rows of `out/propulsion_margins/` and on the propulsion chain in `scripts/propulsion_margins.py`. Every input is listed in the tables.

## Inconsistencies found between files

These need a decision. None of them was changed here.

1. **Take-off mass: 8.365 kg in the performance model, 8.315 kg in the frozen mass balance.**
   - Where each figure is used:
     - `out/wing_trade/performance_v1.csv` and every `out/propulsion_margins/` result use 8.365 kg.
     - `lodestar_v1.MASS` and `aircraft_mass_balance_v1.csv` use 8.315 kg.
   - Why they differ: the performance model is `sizing.evaluate` with the v1 wing mass. It rebuilds the mass from its own bookkeeping, and the 50.5 g difference is exactly three items:
     - misc is taken as 5% of MTOW, 0.418 kg, against a fixed 0.257 kg in the CAD table (+161 g);
     - the winglets are omitted, because the wing mass from `structures.wing_mass` excludes them (−106 g);
     - the chute and container are taken as 0.200 kg against 0.204 kg (−4 g).
   - Effect: the performance results are 50 g conservative, about 0.6% on power.
   - A third figure exists. `out/performance_v1/endurance_summary.csv`, from the parallel Step 2 performance work, uses **8.306 kg**, 9 g under the frozen value.
   - Recommendation: one mass source for every performance script.
2. **The "misc (wiring, glue) 5%" row is not 5%.** It is 0.257 kg, which is 3.1% of the v1 mass. `cad_v4.py` computes it as 7.364 kg (the v3 placeholder MTOW) minus the old budget items. It was never re-derived for v1, although the wiring grew: the twisted pairs, the heater leads, and the 10 AWG leads below, which add about 0.04 kg over 12 AWG. The growth allowance below gives this line the harness allowance (60%) for that reason.
3. **The ESC current figure differs across three places.**
   - The comment on `lodestar_v1.PROPULSION.esc_current_limit_a` and the text of the propulsion note say the worst requirement needs **40.6 A**.
   - The current `out/propulsion_margins/margins.csv`, rerun at the v1 mass, says **41.85 A**. The compliance note says 41.9 A.
   - The 45 A limit still covers 41.85 A, and `freeze_check.py` reads the CSV, so the check is right. The comment and the note are stale.
4. **The elevon servo example does not meet its own spec.**
   - The controls note gives the KST X12-508 as "around 10 kg·cm at 8.4 V". Its listings give **6.2 kg·cm (0.61 N·m) at 8.4 V** ([BuddyRC listing](https://buddyrc.com/products/kst-x12-508-digital-servo-hv-8-4v-0-07s-6-2kg-cm-102oz-in)), below the 0.9 N·m requirement.
   - The cost model prices a KST X10 Pro, which is the 10 mm class the controls note calls too weak.
   - No compliant servo has been selected yet. That leaves F4 and the 30 g servo mass open.
5. **Radios.**
   - The cost model and avionics mass assume one SIYI HM30 unit for video, telemetry and RC.
   - The parallel Step 2 safety work (`out/safety/radio_budget.csv`) models an RFD900x for command and control plus a Microhard pMDDL2450 for video. That is two radios, with different mass, power and cost.
   - The power budget below uses the HM30 because it is the costed part. The choice has to be made once, in the parts list.
6. **The gimbal budget is 0.6 kg; the costed gimbal is 0.197 kg** ([SIYI ZT6 spec](https://siyi.biz/en/product/tri-axis-multi-sensor-gimbal-pod/zt6/spec/)). See G7: the difference is acting as nose ballast.
7. **Stale numbers in older notes** (the CSVs and `lodestar_v1.py` agree with each other):
   - **CAD v4 note:** its late update gives 8.215 kg, CG 0.4144 m and 6.83% static margin. The frozen values are 8.315 kg, 0.4134 m and 7.07%, after the FE doubler and caps added 0.10 kg.
   - **Flight-control note:**
     - CG envelope 0.357–0.422 m, against 0.361–0.422 m in `out/controls/summary.csv` and `CONTROLS`;
     - masses 7.993/6.369 kg;
     - packs at x 0.427, y 0.330 m.
   - **Propulsion note:** its prose tables are from the 8.03 kg baseline. The CSV now gives:
     - climb 1.66 m/s against the prose's 1.74;
     - design endurance 3.68 h against 3.86 h;
     - return-to-home spare 6.1 Wh against 6.7 Wh.
8. **Step 2 Monte Carlo mass growth is narrower than this allowance.** `performance_v1_metrics.py` draws mass growth from a triangular 0 / +2% / +6% distribution. The allowance here is +10.0% of basic. The Step 2 endurance bands should widen the upper tail to about +10%, or state why not.

## G7. Mass properties with a growth allowance

### Method

The standard is ANSI/AIAA S-120A-2015, *Mass Properties Control for Space Systems* ([ANSI store](https://webstore.ansi.org/standards/aiaa/ansiaiaa120a2015)). Its terms:
- **Basic mass** is the current best estimate, with no margin.
- **Mass growth allowance (MGA)** is the expected growth from design immaturity.
- **Predicted mass** is basic mass plus MGA.
- **Margin** is the limit minus predicted mass.

S-120A Table 1 sets the allowance by hardware category and design maturity. The table is sold, not published, so I could not quote it. The schedule used here is built from openly published sources:
- **Estimated maturity:** the AIAA S-120-2006 allowance for the "Estimated" class, as reproduced by [MacKenzie and Burgess, *Satellite Mass Growth*, ICEAA 2016](https://www.iceaaonline.com/wp-content/uploads/2016/10/SS01-ppt-MacKenzie-Sattelite-Mass-Growth.pdf):
  - structures and mechanisms 25%;
  - electronics under 5 kg 30%;
  - batteries 25%;
  - harness 60%;
  - propulsion 25%;
  - thermal 25%.

  The same deck reports that measured unit-level growth on real programs ran **above** these allowances, so they are not generous.
- **Calculated maturity:** about 0.4–0.5 × the Estimated value. This is **an assumption**. S-120 steps the allowance down through layout, pre-release and released drawings, but I have not seen its exact values.
- **Measured or datasheet:** 3%. That is the order of the existing-hardware (A5) allowance in [NASA MSFC's 2013 SAWE briefing](https://ntrs.nasa.gov/api/citations/20130014267/downloads/20130014267.pdf). A part weighed on the aircraft carries only measurement uncertainty.
- **Controlled lines:** 0%.
  - The rescue kit is packed and weighed to its 1.50 kg requirement before each flight; a heavier kit does not fly.
  - The unused part of the gimbal budget is an allocation, not hardware.

Maturity definitions used:
- **Estimated:** a budget or parametric figure.
- **Calculated:** from the structural model, CAD volumes and densities, or a requirement such as the balance moment.
- **Measured (datasheet):** a manufacturer's published mass for the selected part. No part has been weighed yet.

**Table 1.** MGA schedule [% of basic mass] by hardware category and design maturity.

| Category | Estimated | Calculated (assumed) | Measured / datasheet |
|---|---:|---:|---:|
| Structure | 25 | 10 | 3 |
| Mechanism | 25 | 10 | 3 |
| Electronics (< 5 kg) | 30 | 15 | 3 |
| Battery | 25 | 10 | 3 |
| Harness | 60 | 30 | 3 |
| Propulsion | 25 | 10 | 3 |
| Thermal | 25 | 10 | 3 |

*Estimated column from AIAA S-120-2006 via MacKenzie and Burgess (2016). Calculated column assumed. Measured from NASA MSFC (2013). Written to `out/mass_growth/schedule.csv`.*

### Mass lines

The basic mass is the frozen v1 balance. It is the CAD v4 table (`out/cad_v4/mass_cg_sicore.csv`) with the CAD budget wing replaced by the spanwise wing model, 1.983 kg (`out/wing_trade/summary_v1.csv`), as `wing_trade.mass_properties` does. Four lines are split where part of the mass has a better basis:
- **wing:** shell, joint hardware, joint allowance and servos;
- **pack:** datasheet cells and the pack build;
- **gimbal:** the ZT6 datasheet mass and the unused budget;
- **motor + ESC + prop:** the datasheet motor and the rest.

The wing lines sit at the wing's effective x of 0.484 m. That value is solved from the loaded mass-balance CG, and it reproduces the released CG within 0.01 mm.

**Table 2.** Basic mass, maturity, MGA and predicted mass by mass line, SiCore, kit on.

| Line | Basic [kg] | x [m] | Maturity | Category | MGA [%] | MGA [kg] | Predicted [kg] |
|---|---:|---:|---|---|---:|---:|---:|
| Wing shell, core, caps, elevons (spanwise model) | 1.583 | 0.484 | Calculated | Structure | 10 | 0.158 | 1.742 |
| Wing joints: tubes, sleeves, ribs | 0.249 | 0.484 | Calculated | Structure | 10 | 0.025 | 0.274 |
| Wing joint allowance, 45 g per break | 0.090 | 0.484 | Estimated | Structure | 25 | 0.023 | 0.113 |
| Elevon servos (2 × 30 g) | 0.060 | 0.484 | Estimated | Mechanism | 25 | 0.015 | 0.075 |
| Winglets (2) | 0.106 | 0.947 | Estimated | Structure | 25 | 0.027 | 0.133 |
| Cells, SiCore SA112 × 30 (72 g) | 2.160 | 0.412 | Measured (datasheet) | Battery | 3 | 0.065 | 2.225 |
| Pack build: nickel, BMS, leads, wrap | 0.295 | 0.412 | Estimated | Battery | 25 | 0.074 | 0.368 |
| Pack thermal kit (foam + heater) | 0.100 | 0.412 | Estimated | Thermal | 25 | 0.025 | 0.125 |
| Elevon mass balance (2 × 60 g) | 0.120 | 0.715 | Calculated | Structure | 10 | 0.012 | 0.132 |
| Rescue kit | 1.500 | 0.422 | Measured (weighed to spec) | Controlled | 0 | 0 | 1.500 |
| Kit parachute (7 m/s) | 0.124 | 0.422 | Calculated | Mechanism | 10 | 0.012 | 0.136 |
| Kit container + release servo | 0.080 | 0.415 | Estimated | Mechanism | 25 | 0.020 | 0.100 |
| Gimbal, SIYI ZT6 with damper (197 g) | 0.197 | 0.070 | Measured (datasheet) | Electronics | 3 | 0.006 | 0.203 |
| Gimbal budget not yet used (R5) | 0.403 | 0.070 | Allocation | Controlled | 0 | 0 | 0.403 |
| Avionics: FC, GPS, radio, airspeed, Remote ID | 0.350 | 0.234 | Estimated | Electronics | 30 | 0.105 | 0.455 |
| Pod structure, door, skid | 0.300 | 0.267 | Estimated | Structure | 25 | 0.075 | 0.375 |
| Motor, T-Motor AT3520-550 (218 g) | 0.218 | 0.666 | Measured (datasheet) | Propulsion | 3 | 0.007 | 0.225 |
| ESC, prop blades, hub, spinner | 0.122 | 0.666 | Estimated | Propulsion | 25 | 0.031 | 0.153 |
| Misc: wiring, connectors, glue | 0.257 | 0.350 | Estimated | Harness | 60 | 0.154 | 0.412 |
| **Total** | **8.315** | 0.4134 | | | **10.0** | **0.833** | **9.147** |

*SiCore, kit on. Datasheets: [Amprius SA112 catalog](https://amprius.com/documents/Amprius_Product_Catalog.pdf); [SIYI ZT6](https://siyi.biz/en/product/tri-axis-multi-sensor-gimbal-pod/zt6/spec/); [T-Motor AT3520](https://store.tmotor.com/product/at3520-long-shaft-fixed-wing-motor.html). `out/mass_growth/lines.csv`; P50B in `lines_p50b.csv`.*

**By maturity:**
- Measured or datasheet: 4.08 kg basic, 49% of the aircraft, but only 0.08 kg of the allowance.
- Calculated: 2.08 kg, carrying 0.21 kg of allowance.
- Estimated: 2.16 kg, carrying 0.55 kg of allowance.

Two-thirds of the predicted growth sits in estimated lines that cost little to firm up. The misc wiring line (0.15 kg), the avionics (0.11 kg), the pod (0.08 kg) and the pack build (0.07 kg) are the ones to weigh first.

**The wing has no growth room inside its own budget.** Its four lines are predicted at 2.20 kg against the 2.00 kg wing budget of deviation D3, which leaves 17 g today. The weighed skin coupon (T3) and the first wing are the checks.

### Margin and CG

**Table 3.** Mass, margin to 10 kg, CG and static margin vs. growth case, by kit state, SiCore.

| Kit | Case | Mass [kg] | Margin to 10 kg [kg] | CG x [m] | ΔCG [mm] | Static margin [% MAC] | Aft limit − CG [mm] | Inside envelope with ± 4 mm | Nose ballast to recover [kg] |
|---|---|---:|---:|---:|---:|---:|---:|---|---:|
| on | Basic | 8.315 | 1.685 | 0.4134 | 0 | 7.07 | 8.6 | yes | 0 |
| on | Predicted, each line grows in place | **9.147** | **0.853** | 0.4135 | +0.1 | **7.04** | 8.5 | yes | 0 |
| on | Predicted, only lines aft of the CG grow | 8.644 | 1.356 | 0.4184 | +5.0 | **5.96** | 3.6 | **no (0.35 mm over)** | 0.009 |
| on | Predicted, only lines forward of the CG grow | 8.819 | 1.181 | 0.4086 | −4.7 | 8.12 | 13.4 | yes | 0 |
| on | Predicted, all MGA lumped at the motor station | 9.147 | 0.853 | 0.4363 | +23.0 | 1.96 | −14.3 | no | 0.48 |
| on | Predicted, gimbal at ZT6 mass (allocation removed) | 8.744 | 1.256 | 0.4293 | +15.9 | 3.52 | −7.3 | no | 0.28 |
| off | Basic | 6.691 | 3.309 | 0.4113 | 0 | 7.29 | 10.7 | yes | 0 |
| off | Predicted, each line grows in place | 7.511 | 2.489 | 0.4116 | +0.3 | 7.22 | 10.4 | yes | 0 |
| off | Predicted, only lines aft of the CG grow | 7.171 | 2.829 | 0.4174 | +6.1 | **5.94** | 4.6 | yes | 0 |
| off | Predicted, all MGA lumped at the motor station | 7.511 | 2.489 | 0.4391 | +27.8 | 1.11 | −17.1 | no | 0.45 |

*CG envelope 0.361–0.422 m (`lodestar_v1.CONTROLS`), build tolerance ± 4 mm. Static margin from the AVL neutral point of each mass state at Vc 22 m/s, sea level, the lowest-margin condition (`out/controls/mass_states.csv`: 0.4451 m kit on, 0.4441 m kit off). The neutral point is held fixed under growth; a heavier aircraft flies at a higher CL, which moves it slightly aft, so this is conservative. Ballast sits at the gimbal station, x = 0.070 m. P50B results are within 0.04 kg and 0.03% MAC (`out/mass_growth/summary.csv`).*

**What it means:**
- **Mass.** R10 is met with growth: 9.15 kg predicted, 0.85 kg margin. Even the bounding lumped case with its 0.48 kg of ballast comes to 9.63 kg, still under 10 kg.
- **Static margin.** It stays at or above 6% when growth lands where the hardware is (7.04% / 7.22%). Aft-biased growth takes it to 5.96% / 5.94%. That is below the 6% target, still above the 5% that sets the aft limit, and recovered by 9 g of nose ballast.
- **Aft-limit room.** The aft limit is tighter than the 10 kg limit. From the predicted mass, 0.17 kg at the motor station (x = 0.666 m) brings the toleranced CG to the aft limit. Mass added aft of x ≈ 0.6 m (ESC, motor, elevon hardware, winglet antennas) needs a matching nose mass.
- **Gimbal choice.** If a 0.2 kg gimbal replaces the 0.6 kg budget, about 0.28 kg must stay forward:
  - as ballast at the nose;
  - as a heavier sensor;
  - or as a pack shift of about 0.03 m forward, which the 5.3 mm cell-to-sleeve gap may not allow.

  Weigh this with the next CAD iteration. The basic mass does not change here, because 0.6 kg is the requirement value.

Reproduce: `.venv/Scripts/python.exe -X utf8 scripts/mass_growth.py` from the repo root. It writes `lines.csv`, `lines_p50b.csv`, `summary.csv`, `schedule.csv` and `run_log.txt` to `out/mass_growth/`, and exits with code 1 if a self-check fails.

## G8. Electrical power budget

### Bus and rails

The bus voltages follow from the pack layout: 30 cells as 6S5P, 15 cells per wing root. **Assumed here:** each wing pack is 3S5P, and the two are joined in series inside the centre section. Both packs sit at y = ±0.305 m, inboard of the y = ±0.45 m wing break, so no main power crosses the wing joint. Only servo, heater and antenna leads cross it.

**Table 4.** Bus and rail voltage vs. state, by rail.

| Rail | Source | Voltage range [V] | Feeds |
|---|---|---|---|
| Main bus, SiCore 6S | Wing packs, series | 25.2 full; 20.8 nominal (3.46 V/cell); 16.8 at the 2.8 V loaded floor; 15.0 at the 2.5 V cutoff | ESC, gimbal, video/C2 air unit, heater, both regulators |
| Main bus, P50B 6S | Wing packs, series | 25.2 / 21.6 / 16.8 / 15.0 | same |
| Avionics 5.3 V | Power module with current and voltage sensing | 4.9–5.5 at the FC (max 6) | Flight controller, GPS/compass, airspeed sensor, Remote ID |
| Servo 8.2 V HV | Switching BEC | 7.4–8.4 | Two elevon servos, kit release servo |

*Pixhawk 6X power input 4.9–5.5 V, max 6 V, peripheral port limiters 1.5 A + 1.5 A ([Holybro spec](https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification)). The SIYI ZT6 takes 11–25.2 V ([SIYI](https://siyi.biz/en/product/tri-axis-multi-sensor-gimbal-pod/zt6/spec/)), so it runs from the bus directly. The HM30 air unit takes 12.6–25.2 V, but early production lots take 3S–4S only ([SIYI HM30 spec](https://www.siyi.biz/en/product/image-digital-link/hm30/spec/)), so buy the current revision or use SIYI's 4S–18S power module.*

### Loads

**Table 5.** Electrical load vs. state, by component.

| Load | Average [W] | Peak [W] | Basis |
|---|---:|---:|---|
| Gimbal, SIYI ZT6 (thermal + 4K) | 6.5 | 15 | Datasheet, [SIYI](https://siyi.biz/en/product/tri-axis-multi-sensor-gimbal-pod/zt6/spec/) |
| Video, telemetry and RC air unit, SIYI HM30 | 2.8 | 12 | Datasheet, [SIYI](https://www.siyi.biz/en/product/image-digital-link/hm30/spec/) |
| Flight controller, Pixhawk 6X | 2.5 | 3.0 | **Estimate**; Holybro publishes no board consumption |
| GPS + compass (u-blox M10 class) | 0.5 | 0.7 | **Estimate** |
| Airspeed sensor (MS4525DO/DLVR class) | 0.05 | 0.05 | **Estimate** |
| Remote ID (Holybro, ESP32-C3, 20 dBm) | 0.5 | 1.5 | **Estimate**; Holybro gives 5 V supply and 100 mW radiated, not consumption ([Holybro](https://docs.holybro.com/radio/remote-id/overview-and-spec)) |
| Elevon servo, each (≥ 0.9 N·m HV, 2:1 linkage) | 0.2 hold / 1.0 search / 2.0 manoeuvre | 21 (stall, about 2.5 A at 8.4 V) | **Estimate**. The design hinge moment is 0.89 N·m (`out/controls/hinge_moments.csv`). No compliant servo is selected, so no datasheet current exists |
| Kit release servo | 0.1 | 5 (about 1 s) | **Estimate** |
| Pack heater pads | 5.0 | 5.0 | `lodestar_v1.PROPULSION.heater_w`; `out/propulsion_margins/heater.csv` gives 5.3 W at −10 °C with 20 mm EPS |
| Regulator losses | 10% of the 5.3 V and 8.2 V rail loads | | **Assumed** 90% conversion |

*The propulsion chain in `scripts/propulsion_margins.py` (P_batt = P_elec / 0.95 + 25 W) carries all non-propulsion loads as a flat `sizing.A.p_avionics_w` = 25 W. Table 6 replaces that flat figure with the components above and takes the propulsion power as P_batt − 25 W.*

**Table 6.** Battery power and current vs. flight phase, by load.

| Phase | Typical duration | Propulsion [W] | 5.3 V rail [W] | 8.2 V servo rail [W] | Gimbal [W] | Radio [W] | Heater [W] | Non-propulsion total [W] | Battery total [W] | Battery current [A] |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Preflight (on the ground) | 10 min | 0 | 3.9 | 0.6 | 6.5 | 2.8 | 5.0 | 18.8 | 19 | 0.9 |
| Launch and climb-out at full power | 30 s | 667 | 3.9 | 4.6 | 6.5 | 2.8 | 5.0 | 22.8 | 690 | **39.1** |
| Climb, 3 m/s at 18 m/s | 170 s to 500 m | 558 | 3.9 | 2.3 | 6.5 | 2.8 | 5.0 | 20.6 | 578 | 29.3 |
| Search cruise, 20 m/s | about 3 h | 145 | 3.9 | 2.3 | 6.5 | 2.8 | 5.0 | **20.6** | **166** | **7.9** |
| Search cruise, 18 m/s | about 3 h | 130 | 3.9 | 2.3 | 6.5 | 2.8 | 5.0 | 20.6 | 150 | 7.1 |
| Turn, 30° bank at 20 m/s | 10–20 s each | 160 | 3.9 | 4.6 | 15.0 | 2.8 | 5.0 | 31.3 | 191 | 9.1 |
| Kit drop run at 18 m/s and release | 1–2 min | 130 | 3.9 | 10.0 | 15.0 | 2.8 | 5.0 | 36.7 | 167 | 7.9 |
| Return, 25.2 m/s into a 12 m/s headwind | 12.6 min for 10 km | 221 | 3.9 | 2.3 | 6.5 | 12.0 | 5.0 | 29.8 | 251 | 12.0 |
| Landing approach and flare (gliding) | 1–2 min | 0 | 3.9 | 4.6 | 6.5 | 2.8 | 5.0 | 22.8 | 23 | 1.1 |

*Design corner: 2,000 m density altitude, +20 °C, SiCore, 8.365 kg kit on (the performance model mass), mid charge, cells at 25 °C. Propulsion power comes from `propulsion_margins.required` on the same chain that wrote `out/propulsion_margins/`. The climb row matches `motor_points.csv` (582.6 W including the 25 W allowance). The launch row is full power with the 45 A motor-current limit, worst over all 13 corners (SiCore, 25% charge, bus 17.65 V). The return speed and power are from `rth.csv`. The heater is shown on in every phase as the cold-day case; on a warm day subtract 5 W. The radio peak (12 W) is assumed at the edge of range on the return. Battery current = total / bus voltage from the chain.*

**What it means:**
- **The 25 W allowance is adequate.** Search, 85% of the flight, uses 15.6 W of non-propulsion power when warm and 20.6 W with the heater. Over a 3 h sortie that is 13–28 Wh less than the model assumes, 2–4% of usable energy, so the endurance figures are slightly conservative.
- **Exceedances are short.** The kit-drop run (37 W), the turns (31 W) and the return at full radio power (30 W) exceed 25 W only briefly. They cost under 2 Wh per sortie.
- **Peak non-propulsion power is about 90 W**, if every load peaks at once: both elevon servos stalled, the gimbal slewing, the radio at maximum and the heater on. That is 13% of the full-power battery draw and never lasts long.

### Regulators and wiring

**Table 7.** Regulator rating vs. load, by rail.

| Rail | Continuous load [A] | Peak load [A] | Specified rating | Margin |
|---|---:|---:|---|---|
| 5.3 V avionics | 0.67 | 0.99 (devices); 3.0 (port limiters) | ≥ 3 A continuous power module, 6S input, with current sense | 4.5× continuous; covers the FC's own port limiters |
| 8.2 V servo | 0.3 search; 0.5 manoeuvre | 5.7 (both elevons stalled + release) | ≥ 6 A continuous, ≥ 10 A peak HV BEC, 6S input | 1.75× on the all-stall peak |
| Redundancy | | | Feed the Pixhawk's second power input from the servo BEC through its own 5.3 V regulator | One regulator failure must not cut the autopilot (FMEA, G2) |

**Table 8.** Conductor temperature rise and voltage drop vs. wire gauge, pack-to-ESC run.

| Gauge [AWG] | Resistance [mΩ/m] | Loss at 39.2 A [W/m] | Rise, 180 s at 39.2 A, adiabatic [K] | Rise, climb profile, adiabatic [K] | Drop over 1.3 m at 39.2 A [V] | Copper mass [g/m] |
|---:|---:|---:|---:|---:|---:|---:|
| 8 | 2.06 | 3.2 | 22 | **17** | 0.11 | 75 |
| 10 | 3.28 | 5.0 | 55 | **43** | 0.17 | 47 |
| 12 | 5.21 | 8.0 | 139 | 108 | 0.27 | 30 |
| 14 | 8.29 | 12.7 | 351 | 273 | 0.42 | 19 |

*Copper resistance at 20 °C ([AWG table](https://en.wikipedia.org/wiki/American_wire_gauge)), with a 1.1 factor for the copper temperature coefficient over the rise. Copper only, no heat loss to the jacket or air: an upper bound. Climb profile: 31.5 A for 170 s (the 3 m/s climb at 0 m, +40 °C, `motor_points.csv`), then 30 s at 39.2 A. The 39.2 A is the battery-side peak with the 45 A motor limit (scratch run of `propulsion_margins.available` with the limit set to 45 A). The 1.3 m run is pack to pack across the centre section plus both leads to the ESC at x = 0.565 m.*

**The wire check against the 45 A ESC limit:**
- **Battery current.** The 45 A limit is on the motor side. The most the battery side ever sees is **39.2 A**: full power, SiCore at 25% charge, bus 17.65 V. That is 7.8 A per cell, against SA112's 13 A continuous.
- **Phase leads.** The ESC-to-motor leads carry about 45 × √(2/3) ≈ 37 A RMS per phase at the limit. They are about 0.1 m long, in the ventilated tail of the pod, so 12 AWG is adequate.
- **Battery leads: 10 AWG is the minimum.** It drops 0.17 V (1% of the 16.8 V loaded floor), and with the connectors stays inside the 10 mΩ `R_WIRE` the propulsion model assumes.
- **Heat is the constraint, not voltage drop.** On a +40 °C day a 10 AWG lead can reach about 83 °C in a full climb. That is above the long-term service limit of EPS, 74–75 °C ([Sika Sarnatherm EPS](https://usa.sika.com/dms/getdocument.get/8c3a73e9-e135-46bc-a3b1-6a47e0fc5496/sarnatherm_eps.pdf); [Carpenter, typical properties](https://carpenter.com/wp-content/uploads/2022/04/TypicalPropertiesC578.pdf)). So:
  - **Either** run 10 AWG silicone leads in a vented channel or a PTFE sleeve with stand-offs, not touching the EPS;
  - **or** use 8 AWG, which rises about 17 K and peaks near 57 °C, if the leads are buried in foam.

  8 AWG adds about 37 g over the 1.3 m run, inside the misc line's allowance.
- **Twisted pairs (F6).** Run + and − together over the whole run.
- **Connectors.** Rate the pack connectors for at least 40 A continuous; the 90-class anti-spark type is the usual choice.
- **Servo leads.** 20 AWG on the roughly 1.5 m run out to y = 0.87 m. It drops 0.25 V at a 2.5 A stall (3%); 22 AWG would drop 0.40 V.
- **Heater leads.** 24 AWG is enough for about 0.25 A.
- **Heater supply.** The heater must be thermostat- or PWM-controlled from the bus. A plain resistive pad would vary 2.25× in power between a full (25.2 V) and an empty (16.8 V) pack.

**Deferred to Step 4:** the full wiring and power-distribution diagram, the connector list and harness masses.

## G12. Interface list (ICD-lite)

Owner side is the item responsible for meeting the interface. "Open" means the interface has no design yet.

**Table 9.** Mechanical interfaces by pair, with owner, parameters and verification.

| ID | Interface | Owner side | Type | Key parameters | Verification |
|---|---|---|---|---|---|
| M1 | Centre section ↔ outer panel (joint), y = ±0.45 m | Wing | Plug-in spar joint | 20 × 16 mm carbon tube, 24 mm OD sleeves, 160 mm engagement per side. Tube 149 MPa ultimate against a 500 MPa assumed allowable. Aft key force ≤ 68.9 N. Joint bending stiffness ≥ 1,000 N·m/rad (target 2,000). Sleeve ribs (F12), hoop plies (F13) | Analysis (`out/wing_trade/v1_joint.csv`, `out/fea/joint_checks.csv`); T7 stiffness; inspection |
| M2 | Wing centre section ↔ pod (saddle) | Pod | Bolted saddle, **fasteners open** | 2 × 60 × 40 mm G10 plates over the EPS (F14). Bare-EPS bearing MS +0.01, about +10 with plates. Carries the 10 g bungee launch and the belly landing | Analysis (FE landing cases); fastener design open; proof load in T7 |
| M3 | Pod ↔ belly skid | Pod | Bonded EPP block | ≥ 40 mm crush stroke (F10) at z = −0.111 m, x = 0.27 m; every landing MS ≥ +0.7 at 1.5 m/s sink | Analysis (`out/fea/landing_cases.csv`); EPP crush coupon (proposed T13) |
| M4 | Pod ↔ kit bay module | Pod | Bay with door | Bay x 0.31–0.53 m, centred on the design CG x = 0.422 m. Door at z = −0.100 m. Takes the kit (1.50 kg + 0.124 kg chute) or the 18-cell battery module (`alt_bay_battery_module.step`, 436 cm³) without retrim | CAD fit check (`out/cad_v4/fit_check.csv`); T8 hang test in both configurations |
| M5 | Kit container ↔ release | Kit system | Servo-actuated latch | Container + release 0.08 kg stays on board. Release servo at x = 0.317 m. Release at 40 m AGL on a 14 m/s into-wind run (D2), or 18 m/s in moderate turbulence (L1). ΔCG at release −2.1 mm x, +9.1 mm z; re-trim 0.7° | Analysis (`out/controls/kit_release.csv`, `out/drop/`); T6 drop test |
| M6 | Pod nose ↔ gimbal | Pod | Damped mount, ball below the pod | Mount x = 0.070 m, z = −0.100 m. ZT6 197 g with its damper board (budget 0.6 kg; see G7 on ballast). Lens at z = −0.141 m, so retract or protect it for the belly landing | CAD fit; landing clearance check open |
| M7 | Pod tail ↔ motor and prop | Propulsion | Firewall mount, pusher | AT3520 at x = 0.67 m, CAM 15 × 10 folding prop. Prop disc clears the wing by 40.3 mm. Soft mount if vibration exceeds 15 m/s² | CAD fit; T4 thrust stand; vibration logs in flight test |
| M8 | Pod ↔ ESC | Propulsion | Internal mount with cooling ducts | ESC at x = 0.565 m, 31 mm clearance. Inlet and outlet for several watts at 45 A | CAD fit; bench test at 45 A (proposed T9) |
| M9 | Wing root ↔ battery packs | Wing | Bay in the core, structural lid | Packs at x = 0.412 m, y = ±0.305 m, 15 cells each, axes spanwise. Cell-to-sleeve 5.3 mm; cell-to-skin 2.7 mm after allowance. 20 mm EPS + 5 W pad (F1). Lid with M3 nylon screws in G10 inserts at 30 mm pitch (F15). Bay walls carry 10 g launch inertia (MS +0.65) | CAD fit; FE; chamber soak (proposed T12) |
| M10 | Outer panel ↔ elevon | Wing | Hinge at 75% chord, y = 0.45 m to the tip | Free play ≤ 0.15° (F8). Hinge stiffness ≈ 136 N·m/rad at 2:1. 60 g mass balance 40 mm ahead of the hinge at y = 0.87 m (F7). Travel ±20° with mechanical stops beyond it | T7 (hinge stiffness, free play); static balance check |
| M11 | Outer panel ↔ elevon servo | Wing | Servo bay at y = 0.87 m, 60% chord | ≤ 13 mm thick servo in 20–25 mm of available depth. ≥ 0.9 N·m, ≤ 0.12 s/60°, non-back-drivable, holds on signal loss. 2:1 linkage | **Servo not selected** (inconsistency 4); bench torque and stiffness (proposed T10) |
| M12 | Outer panel ↔ winglet | Wing | Bonded butt joint with tapes | 0.25 m NACA0008 winglet. Two 25 mm glass overlap tapes (F15; MS +10.3). Houses an RC or telemetry antenna | FE (`out/fea/margins.csv`); inspection |
| M13 | Pod nose ↔ pitot | Pod | Push-fit, breakaway | Static ports ≥ 230 mm ahead of the nose (x ≤ −0.40 m), 6° nose-down. Sensor in the nose with short tubes (F5) | Calibration flight (`ARSPD_AUTOCAL`) |
| M14 | Pod top ↔ GPS mast and Remote ID | Avionics | Mast + module under a glass skin | GPS on a 60 mm mast at x = 0.12 m, top z ≈ +0.105 m. Remote ID ≥ 100 mm aft of the GPS. No carbon over antennas | Inspection; compass calibration |
| M15 | Aircraft ↔ launcher | Ground equipment | Bungee or catapult shoe, **open** | Release ≥ 16 m/s above 3,000 m density altitude (L3), 14 m/s minimum below. Launch within 30° of the wind (L4). 10 g launch inertia case (FE). Prop stopped until clear of the rail (`TKOFF_THR_DELAY`) | Launcher exit-speed measurement at full mass (proposed T11) |

**Table 10.** Electrical interfaces by pair, with owner, parameters and verification.

| ID | Interface | Owner side | Type | Key parameters | Verification |
|---|---|---|---|---|---|
| E1 | Left pack ↔ right pack (series link) | Power | 10 AWG (or 8 AWG) twisted run in the centre section | 2 × 3S5P in series = 6S5P (assumed). Up to 39.2 A. Stays inboard of the wing joint | Bench test at 45 A (proposed T9) |
| E2 | Packs ↔ ESC | Power | Main leads + pack connector | 15.0–25.2 V. ≤ 39.2 A battery side. Rated ≥ 40 A continuous. Twisted pairs (F6). Kept off EPS, or 8 AWG | T9; compass `compassmot` check |
| E3 | ESC ↔ motor | Propulsion | 3 phase leads, 12 AWG | Motor current limit 45 A (F3). About 37 A RMS per phase. Short, in the tail | T4; T9 |
| E4 | Main bus ↔ 5.3 V power module ↔ FC | Avionics | Power module with V/I sense | FC input 4.9–5.5 V (max 6). ≥ 3 A. Second feed from the servo-rail regulator | T9; FC voltage log |
| E5 | Main bus ↔ 8.2 V BEC ↔ servos | Avionics | HV BEC | 7.4–8.4 V. ≥ 6 A continuous / 10 A peak. **Elevon leads cross the wing joint: a keyed multi-pin connector is needed at y = ±0.45 m (open)** | T9; T10 |
| E6 | Main bus ↔ gimbal | Payload | Direct, fused | 11–25.2 V. 6.5 W average, 15 W peak | Bench power-up across 15–25.2 V |
| E7 | Main bus ↔ video and C2 air unit | Avionics | Direct (current revision) or 4S–18S module | 12.6–25.2 V. 2.8 W average, 12 W peak. Radio choice open (inconsistency 5) | Bench; range test |
| E8 | Main bus ↔ pack heater pads | Power | Thermostat or PWM switch | 5 W regulated, about 0.25 A. Leads cross no joint (packs are in the centre section). Pre-warm from ground power before launch | Chamber soak (proposed T12) |
| E9 | FC ↔ Remote ID | Avionics | 5 V from TELEM or CAN | About 0.5 W (estimated) | Inspection; FAA declaration-of-compliance check |
| E10 | Bay battery module ↔ main bus (search-only configuration) | Power | Parallel 6S3P, **open** | Must sit at the same state of charge and voltage as the wing packs, or connect through ideal diodes. Mismatched 6S packs in parallel drive large balancing currents | Design open; bench test |
| E11 | Winglet antennas ↔ air unit | Avionics | Coax through the wing | **Coax crosses the wing joint (connector at y = ±0.45 m, open)**. Keep clear of carbon caps and the joint tube | Range test |

**Table 11.** Data interfaces by pair, with owner, parameters and verification.

| ID | Interface | Owner side | Protocol | Key parameters | Verification |
|---|---|---|---|---|---|
| D1 | FC ↔ ESC | Avionics | Bidirectional DShot or PWM + telemetry wire | RPM telemetry for the harmonic notch (`INS_HNTCH_MODE = 3`) | Bench; vibration log |
| D2 | FC ↔ elevon servos | Avionics | PWM, `SERVO1/2_FUNCTION` 77/78 | `MIXING_GAIN` 0.5; travel ±20° at the surface from `SERVOn_MIN/MAX` | Ground check of throws |
| D3 | FC ↔ kit release servo | Avionics | PWM with a two-step arm and release | No-drop-over-people rule ([14 CFR 107.23](https://www.law.cornell.edu/cfr/text/14/107.23)) | HIL (out of scope); T6 |
| D4 | FC ↔ GPS / compass | Avionics | DroneCAN or serial + I²C | `GPS1_POS_X = −0.295`, `GPS1_POS_Z ≈ −0.125` | Compass calibration |
| D5 | FC ↔ airspeed sensor | Avionics | I²C | `ARSPD_RATIO` calibration | Calibration flight |
| D6 | FC ↔ C2 link | Avionics | MAVLink serial | Lost-link behaviour (Part 108 item, v2) | Range test; link budget (G4) |
| D7 | Gimbal ↔ video link | Payload | Ethernet video; SDK or MAVLink control | 640 × 512 thermal + 4K | Bench |
| D8 | FC ↔ Remote ID | Avionics | DroneCAN or MAVLink | Position from the FC ([ArduPilot Remote ID](https://ardupilot.org/plane/docs/common-remoteid.html)) | Declaration-of-compliance check |

**Count:** 34 interfaces (15 mechanical, 11 electrical, 8 data). **Open** design items:
- M2 fasteners;
- M11 servo selection;
- M15 launcher shoe;
- E5 and E11 joint connectors;
- E10 bay-module paralleling.

## G13. Operating envelope

**Table 12.** Operating limit vs. parameter, with the governing case and its source.

| Parameter | Limit | Governing case | Source |
|---|---|---|---|
| **Mean wind** | ≤ 12 m/s | Max level speed 29.5 m/s worst corner gives 17.5 m/s groundspeed home; best return airspeed 25.2 m/s | R3; `out/propulsion_margins/margins.csv`, `rth.csv` |
| **Gusts, structural** | CS-VLA 15.2 m/s at Vc, 7.6 m/s at VD | Wing sized to these at FOS 1.5 | `scripts/structures.py`; `out/fea/margins.csv` |
| **Gusts, operational** | A 7.6 m/s sharp-edged gust stalls the wing at every search speed (peak 1.6–3.6 g, inside +3.8 g) | No control saturation; recovery by the autopilot | Controls note, Table 9 (`out/controls/gusts.csv`) |
| **Turbulence** | Moderate (Dryden, W20 = 30 kt) at most; search and drop at ≥ 18 m/s TAS | 3σ lift jump equals the stall margin at 15 m/s kit on | L1; `lodestar_v1.OPERATIONS.turbulence_min_tas` |
| **Launch crosswind** | Launch within 30° of the wind; cross component ≤ 6 m/s at 12 m/s wind | No rudder; sideslip off the rail | L4; `out/controls/crosswind.csv` |
| **Ambient temperature** | −10 to +40 °C analysed. Packs held ≥ 15 °C (20 mm EPS + 5 W pad) | SiCore climb sag when cold; pack 43 °C at +40 °C ambient | F1; `out/propulsion_margins/` |
| **Cold-day search speed** | Below 0 °C, plan the search at ≤ 18 m/s | 20 m/s endurance falls to 2.7–2.9 h when cold | L5 |
| **Cell limits** | Discharge −20 to +60 °C; charge 0 to +45 °C (SA112); field charging below 0 °C needs a warm box | Datasheet | [Amprius catalog](https://amprius.com/documents/Amprius_Product_Catalog.pdf) |
| **Density altitude** | 0 to 4,400 m (design 2,000 m). Hot-and-high 3,000 m ISA+20 = 3,691 m DA checked | Climb and launch margins | `lodestar_v1.OPERATIONS` |
| **Launch above 3,000 m DA** | Launcher rated ≥ 16 m/s release; packs pre-warmed | 14 m/s release is 0.98 × Vs at 4,400 m | L3 |
| **Search turns above about 3,200 m DA** | **New:** at 18 m/s, a 30° bank loses the 1.25 × stall margin above about 3,200 m DA. Search at 20 m/s (holds to 4,400 m) or limit the bank to 20° | Loaded stall 11.41 m/s EAS; 30° bank × 1.075 | This note (`out/wing_trade/operating_limits.csv` gives 3,163 m for the older, heavier wing) |
| **Precipitation and icing** | **Not cleared: no flight in precipitation, in cloud, or in visible moisture at or below +3 °C** | No heated pitot, open EPS core edges, no ingress rating for the avionics; icing never analysed | **Open**; cloud clearance per [14 CFR 107.51](https://www.law.cornell.edu/cfr/text/14/107.51) |
| **Height** | ≤ 120 m (400 ft) AGL, terrain-following | Part 107 | R4 |
| **Groundspeed** | ≤ 44.7 m/s (87 kt) | Part 107 | [14 CFR 107.51](https://www.law.cornell.edu/cfr/text/14/107.51) |
| **Battery state of charge** | 85% usable depth of discharge; 10% of usable held as the landing reserve; no full-power climb below 25% charge; head home at the bingo energy (1.2 × E_return at wind + 3 m/s) | SiCore sag; headwind return | F2, L2; `lodestar_v1.OPERATIONS` |
| **Pack current** | Motor ≤ 45 A (ESC limit); battery ≤ 39.2 A; ≤ 7.8 A per cell (SA112 13 A continuous, 26 A for 30 s) | 15 × 10 prop overloads the motor uncapped | F3; this note |
| **CG** | 0.361–0.422 m (3.6–17.9% MAC); built to 0.4134 ± 0.004 m kit on, 0.4113 kit off; static margin ≥ 6% target, ≥ 5% limit | Aft limit = 5% MAC on the most forward neutral point (Vc) | L6; `lodestar_v1.CONTROLS`, `MASS`; G7 above |
| **Mass** | ≤ 10 kg; 8.315 kg basic, 9.15 kg predicted with growth | R10 | G7 above |
| **Load factor** | +3.8 / −1.5 limit; bank ≤ 30° in routine flight | Structure FOS 1.5 | `lodestar_v1.STRUCTURE`, `OPERATIONS` |

**Table 13.** Characteristic speed vs. mass state, with source.

| Speed | Kit on (8.31 kg) | Kit off (6.69 kg) | Source |
|---|---|---|---|
| Stall, 1 g, CLmax 0.80 (EAS) | 11.4 m/s | 10.2 m/s | `out/propulsion_margins/performance.csv` (8.365 kg model mass) |
| Stall TAS at 2,000 m / 4,400 m | 12.6 / 14.3 m/s | 11.3 / 12.8 m/s | same |
| Stall at the cross-checked CLmax 1.01–1.20 (EAS) | 9.3–10.2 m/s | — | `research/2026-11-14-aero-crosscheck.md` |
| Minimum operating (1.25 × Vs) | 14.3 m/s EAS | 12.8 m/s EAS | `lodestar_v1.OPERATIONS.stall_margin` |
| Search | 18–20 m/s TAS (18 in moderate turbulence, ≤ 18 below 0 °C, 20 above 3,200 m DA when turning at 30°) | same | R2, L1, L5, this note |
| Kit release | 14 m/s into wind at 40 m AGL (18 m/s in moderate turbulence) | — | D2, L1 |
| Best return into 12 m/s wind | 25.2 m/s TAS | 24.4 m/s TAS | `rth.csv` |
| Cruise structural (Vc) | 22 m/s EAS | 22 m/s EAS | `lodestar_v1.STRUCTURE` |
| Max level, continuous power, worst corner | 29.5 m/s (SiCore), 35.0 m/s (P50B) | 29.7 m/s (SiCore) | `margins.csv`, `performance.csv` |
| Dive (VD) | 30 m/s EAS | 30 m/s EAS | `lodestar_v1.STRUCTURE` |
| Flutter clearance (1.15 VD) / predicted flutter | ≥ 34.5 / 49.4 m/s EAS | same | flutter note |
| Autopilot `AIRSPEED_MIN` / `AIRSPEED_MAX` | ≈ 14 / 22 m/s EAS | | Controls note |

*CLmax 0.80 is the conservative sizing value. Stall speeds come from the performance model mass (8.365 kg), so they are about 0.3% high against the frozen 8.315 kg.*

## Reproduce

- `.venv/Scripts/python.exe -X utf8 scripts/mass_growth.py` regenerates G7 (`out/mass_growth/`).
- Tables 6 and 8 are arithmetic on `out/propulsion_margins/motor_points.csv`, `rth.csv` and calls to `propulsion_margins.required` / `available` (motor current limit set to 45 A for the peak). The inputs are in the captions.
