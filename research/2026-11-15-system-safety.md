# Lodestar v1 system safety: hazard assessment, FMEA, contingencies and C2 link

2026-11-15

Step 2 items G1 (functional hazard assessment), G2 (component FMEA), G3 (contingency physical side) and G4 (command-and-control link budget). Computations: `scripts/system_safety.py` (writes `out/safety/`); checks: `scripts/test_system_safety.py`.

## Answer

**v1 is safe enough to fly within visual line of sight over empty terrain, but not yet for the mission it is designed for.** The mission is a beyond-visual-line-of-sight (BVLOS) search along busy trails with SAR helicopters in the same airspace. Three findings drive that, and each needs a change.

1. **The route deliberately flies over people, and v1 has no way to make a failure gentle.**
   - The trails-first route keeps the aircraft within 50 m of a mapped trail or road for **46% (3 km) and 68% (6 km)** of the flown track, up to 92% at Yosemite.
   - With no aircraft parachute, a failure ends in an impact of **0.5 kJ** (tumbling), **1.25 kJ** (controlled glide) or **9 kJ** (dive from 120 m). The Part 107 Category 3 over-people limit is 34 J.
   - Flight over people therefore needs a [14 CFR 107.39](https://www.law.cornell.edu/cfr/text/14/107.39) waiver. A terminal parachute (ASTM F3322) is the main design change. It cuts the impact to **100–190 J** but costs about 0.4 kg and about 7% endurance, which takes the design-case R1 endurance from 3.09 h to about 2.9 h.
2. **A motor-out reaches the launch point from only about half of the route.**
   - At the planning L/D of 15 in still air, the aircraft can glide back to the launch point from **64% of the 3 km route and 50% of the 6 km route** (mean of 7 sites).
   - Counting open landing areas away from people as well, the figures are **82% and 72%**. They fall to **65% and 50%** in a 10 m/s headwind.
   - The Great Smokies and the White Mountains have almost no open ground. There, the rest of the route ends in a forced landing in trees.
3. **The C2 link has gaps of up to about 20 minutes on the 6 km routes.**
   - The 900 MHz telemetry link holds for **99% of the 3 km route and 89% of the 6 km route** from a 10 m mast at the launch point.
   - The longest outage at search speed is **4.9 min (3 km)** and **20.8 min (6 km, Sequoia)**.
   - A relay carried to a high point brings the 6 km figure to 94%. At Yosemite, no relay site reachable from a trail sees the ground station.
   - So the aircraft must keep searching autonomously during an outage, with the bingo rule computed on board. Returning home on every link loss would abort most 6 km sorties.

**The link-loss return is affordable.**
- Returning from the farthest route point, over the ridges in between, into a 10 m/s headwind, takes **10–69 Wh**.
- The bingo threshold there (1.2 × the return energy plus the 10% landing reserve) is **12–24% of usable energy**.
- The existing rule L2 covers it, provided it runs on the aircraft and not on the ground station.

Using MIL-STD-882E severity and probability scaled to an 8.3 kg civil UAS:
- **Before the new changes**, five hazards are High:
  - loss of control;
  - a mid-air collision with a SAR helicopter;
  - people under the route;
  - a single-point avionics power loss;
  - an airspeed failure.
- The other six are Serious.
- **With the ten required changes** in Section 7, every hazard is Medium except mid-air collision. That one stays Serious and needs written acceptance for each operation.

## 1. Method and scales

The method follows [MIL-STD-882E](https://everyspec.com/MIL-STD/MIL-STD-0800-0899/MIL-STD-882E_41682/), tasks 202 (preliminary hazard analysis) and 205 (system hazard analysis). 882E lets each program set its own probability and acceptance scales. The scales below are tailored for one civil SAR team flying a single 8.3 kg aircraft for a few hundred hours a year. The probability bands are engineering judgement for hobby-to-light-commercial components: no Lodestar part has field data. The [Part 108 proposal](https://www.flyusi.org/guides/get-108-ready?hsLang=en-us) asks for at least 150 h of reliability flying, which is the evidence that would replace them.

**Table 1.** Severity categories, by consequence (882E categories, civil UAS wording).

| Cat. | 882E name | Lodestar consequence |
|---|---|---|
| 1 | Catastrophic | Death or serious injury to a third party, the lost person or the crew; mid-air collision with a manned aircraft; wildfire ignition |
| 2 | Critical | Possible minor injury; uncontrolled landing near a trail or road; property damage; a fly-away outside the operating area |
| 3 | Marginal | Loss or heavy damage of the aircraft in an unpopulated area; the mission is aborted |
| 4 | Negligible | Mission degraded (time, coverage, video); no damage |

**Table 2.** Probability levels, by rate per flight hour (tailored).

| Level | 882E name | Rate per flight hour | Meaning for one team (~300 h/yr) |
|---|---|---|---|
| A | Frequent | ≥ 10⁻² | Several times a year |
| B | Probable | 10⁻³ – 10⁻² | About once a year |
| C | Occasional | 10⁻⁴ – 10⁻³ | Once in a few years |
| D | Remote | 10⁻⁵ – 10⁻⁴ | Possible in the life of a small fleet |
| E | Improbable | < 10⁻⁵ | Not expected in the fleet's life |
| F | Eliminated | 0 | Hazard removed by design |

**Table 3.** Risk matrix, severity × probability (882E Table III).

| Probability | 1 Catastrophic | 2 Critical | 3 Marginal | 4 Negligible |
|---|---|---|---|---|
| A Frequent | High | High | Serious | Medium |
| B Probable | High | High | Serious | Medium |
| C Occasional | High | Serious | Medium | Low |
| D Remote | Serious | Medium | Medium | Low |
| E Improbable | Medium | Medium | Medium | Low |
| F Eliminated | Eliminated | Eliminated | Eliminated | Eliminated |

**Risk acceptance (tailored).** 882E assigns acceptance authority by risk level; for a civil team it maps as follows:
- **High:** not acceptable. A design or operations change is required.
- **Serious:** accepted only in writing, for each operation, by the team's accountable manager or chief pilot, and only with the mitigations in place.
- **Medium:** accepted by the program owner when the mitigation is verified.
- **Low:** accepted.

**The flag rule for "REQUIRED".** A row is marked REQUIRED when, with v1 as frozen (its F1–F15 and L1–L6 only), the risk is High, or Serious with no existing mitigation. The residual risk shown assumes the proposed change is made.

## 2. Ground and air risk: SORA cross-check

The JARUS Specific Operations Risk Assessment (SORA 2.5) is the European and international method for this class of operation. It is not FAA law, but FAA BVLOS waivers ask for the same evidence. Sources: the [JARUS publications page](https://jarus-rpas.org/publications/) (SORA 2.5 main body, Table 2 intrinsic GRC and the SAIL table) and the [Norwegian CAA SORA 2.5 calculator](https://training.caa.no/SORA-2.5-calculator).

**Table 4.** SORA 2.5 classes for Lodestar, by operating area.

| Item | Value | Reason |
|---|---|---|
| iGRC column | 3 m / 35 m/s | Span 3.0 m; maximum level speed 34.7 m/s (propulsion-margins note). Any speed increase moves it to the 8 m / 75 m/s column, one class worse |
| iGRC, remote wilderness (< 5 people/km²) | 3 | Off-trail search areas |
| iGRC, lightly populated (< 50/km²) | 4 | Busy trail corridors in season. This is the honest planning value for a trails-first route |
| iGRC, sparsely populated (< 500/km²) | 5 | Valley floors near trailheads (Yosemite Valley, Bear Lake corridor on a summer weekend) |
| M2 (impact energy reduced) | −1 (medium robustness) with an ASTM F3322 parachute | Not available on v1 |
| Initial ARC | ARC-b | Below 500 ft AGL, uncontrolled airspace, rural |
| Local ARC at a SAR incident | treated as ARC-c | Helicopters work at 0–500 ft over the same search area. Strategic mitigation (an incident TFR under [14 CFR 91.137](https://www.law.cornell.edu/cfr/text/14/91.137), managed by the incident air-operations branch) is what brings it back to ARC-b |
| SAIL, v1 as frozen, trails at GRC 4 | III (ARC-b) to IV (ARC-c) | SAIL table |
| SAIL with a parachute (GRC 3) and an incident-managed airspace block (ARC-b) | II | The target for a waiver application |

**The SORA reading agrees with the 882E reading.** Ground risk is driven by the trail overflight, and the parachute is the mitigation that moves the class. Air risk is driven by the helicopters at the incident, and procedural segregation is the mitigation that moves it. Detect-and-avoid (DAA) equipment alone does not move it at SAIL III–IV.

## 3. G1: Functional hazard assessment

Initial risk is for v1 as frozen; residual risk assumes the changes in the last column, with the IDs from Section 7.

**Table 5.** Functional hazards: effect, initial and residual 882E risk, by function.

| ID | Function: failure condition | Worst credible effect | Initial (S/P, risk) | Existing mitigation | Added mitigation | Residual |
|---|---|---|---|---|---|---|
| H1 | Thrust: total loss in flight (motor, ESC, power) | Forced glide. The launch point is reachable from 50–64% of the route in still air (Table 10); otherwise the aircraft lands in trees or on steep ground, possibly beside a trail | 2C **Serious** | F3 current limit; ESC RPM telemetry | RC9 glide to a preloaded landing area away from trails; RC7 track offset | 2D Medium |
| H2 | Control: loss of control (servo hard-over, linkage, autopilot, structural) | Uncontrolled descent within about 100 m of the track; impact 0.5–9 kJ; the route is near a trail 46–68% of the time | 1C **High** | F4, F7, F8, F9, F10–F15; tests T1–T7 | RC1 terminal parachute (100–190 J); RC2 avionics power; RC7 | 2D Medium |
| H3 | C2 link: loss of command link | No pilot intervention for up to about 21 min on 6 km routes (Table 13). Alone this degrades the mission; it is a contributor to H8 | 3A **Serious** | None (no lost-link behaviour defined in v1) | RC5 autonomous continuation, on-board bingo, pre-flight link map, 10 m mast and relay | 4A Medium |
| H4 | Navigation: loss or corruption of position (GNSS outage, jamming, compass error) | Drift or fly-away outside the search area, possibly over a road or into manned traffic | 1D **Serious** | F6 twisted pairs (compass) | RC10 dual GNSS, geofence with termination (needs RC1) | 1E Medium |
| H5 | Kit release: release over people, or chute failure | 1.62 kg falling from 120 m (inadvertent release, about 600 J or more), or from 40 m with a failed chute | 1D **Serious** | Release only at 40 m AGL on the run-in; aim 5 m beside the person (drop study) | RC6 two-stage arm/fire interlock and release inhibits; static-line chute packing check | 1E Medium |
| H6 | Energy storage: thermal runaway, overheating, cold | In-flight fire (loss of aircraft); after a crash in dry forest, wildfire. Cold: climb shortfall (F1/F2 note) | 1D **Serious** | F1 insulation and heater, F2 climb rule, L5 | RC3 heater lockout and independent cut-out, cell monitoring, battery-bay liner, crash-site fire watch | 1E Medium |
| H7 | Structure: wing, joint or pod failure in flight or launch | As H2 | 1D **Serious** | Every FE margin ≥ 0 with F10–F15; flutter clearance with F7–F9; tests T1–T7 | RC1 (consequence); joint retention check before each flight | 2E Medium |
| H8 | Airspace: mid-air with a manned aircraft (SAR helicopter, air ambulance, GA) | Catastrophic for the helicopter crew | 1C **High** | Part 107 right-of-way rule ([107.37](https://www.law.cornell.edu/cfr/text/14/107.37)); Remote ID | RC8 incident-airspace integration, altitude block, ADS-B In with autonomous hold or descend, strobes; RC5 keeps the aircraft predictable during outages | 1D Serious (per-operation acceptance) |
| H9 | Ground risk: people under the route (normal flight) | An 8.3 kg aircraft overflying hikers; any failure above becomes Catastrophic there | 1C **High** | None in v1 | RC7 107.39 waiver, trail-track offset, drop rules; RC1 | 2D Medium |
| H10 | Power distribution: loss of avionics power | Autopilot and servo brown-out, then H2. One connector or regulator is a single point of failure | 1C **High** | None stated | RC2 independent avionics backup supply, locking connectors | 1E Medium |
| H11 | Air data: wrong airspeed (blocked or iced pitot) | Stall or overspeed, then H2 | 1C **High** | F5 position only | RC4 airspeed-failure detection with synthetic-airspeed fallback; icing limit L7 | 1E Medium |

## 4. G2: Component FMEA

S/P are the 882E severity and probability (Tables 1–2). "Initial" is v1 as frozen; "Residual" is with the stated mitigation. The F and L references are fixes and limits from the [compliance note](2026-11-14-v1-requirements-compliance.md); RC references are the required changes in Section 7.

**Table 6.** Component failure modes, effects, detection, mitigation and risk: propulsion and energy.

| # | Item | Failure mode | Local effect | System effect | Detection | Mitigation | Initial | Residual | Flag |
|---|---|---|---|---|---|---|---|---|---|
| 1 | Motor (AT3520 class) | Bearing seizure, winding short, overheating | Loss or reduction of thrust | H1 forced glide | ESC RPM vs throttle; current; motor temperature model | F3 45 A limit (worst need 41.9 A against a 50 A / 180 s rating); RC9 | 2C Serious | 2D Medium | |
| 2 | Folding prop and mount | Blade loss, hinge failure, imbalance | Vibration at the pod tail | Possible IMU aliasing, then H2; motor-out | IMU vibration level; ESC RPM harmonic notch (F3) | Preflight blade and hinge check; cut motor on vibration above threshold, then RC9 | 2C Serious | 2D Medium | |
| 3 | ESC (60 A) | Thermal shutdown, desync, stuck at full throttle | Loss of thrust, or uncommanded full power | H1, or overspeed and climb (controllable) | RPM telemetry, current, temperature | F3 limit; cooling inlet and outlet (controls note); desync detection | 2C Serious | 2D Medium | |
| 4 | Packs, 6S5P SiCore | Internal short, then thermal runaway | Cell venting and fire | H6: loss of aircraft; wildfire after a crash | Cell-voltage spread, pack temperature | RC3: BMS with cell-voltage and temperature monitoring, cell fuse links, mica or ceramic-paper bay liner (about 30 g), fire watch at any crash site | 1D Serious | 1E Medium | **REQUIRED** (RC3) |
| 5 | Packs | Capacity or resistance worse than modelled, cold soak | Early voltage sag | Bingo misjudged, forced landing short of home | Coulomb count against the voltage model | F1, F2, L2, L5; pulse test T5 | 2C Serious | 2D Medium | |
| 6 | Pack connectors and harness (packs are in the two wing roots, joined across the centre) | Disconnect from launch shock or vibration; chafe | Loss of the whole bus | H10: motor **and** avionics lost, then H2 | Bus voltage (too late) | RC2: locking connectors with a retention clip; separate avionics backup supply | 1C **High** | 1E Medium | **REQUIRED** (RC2) |
| 7 | Pack heater, 5 W pad (F1) | Stuck on | Pack heating. Insulation resistance 0.020 m / (0.035 W/m·K × 0.20 m²) ≈ 2.9 K/W, so the stuck heater adds about 14 K and cruise self-heating (about 4 W) about 11 K. On a +40 °C day the cells reach about 65 °C, over the SA112's 60 °C limit | H6 | Pack temperature sensor | RC3: ambient lockout above +10 °C, independent 50 °C thermal cut-out in series with the pad | 1D Serious | 1E Medium | **REQUIRED** (RC3) |
| 8 | Pack heater | Stuck off, or thermostat failure | Cold pack | Climb shortfall at the worst corner (1.66 m/s vs 3 m/s required) | Pack temperature | F2 climb rule via L2; pre-warm (L3) | 3C Medium | 3D Medium | |
| 9 | Power distribution (power module, servo BEC, avionics BEC) | Regulator failure, brown-out on a servo stall current | Autopilot reset or servo power loss | H10, then H2 | Autopilot voltage monitor (too late for a reset) | RC2: dual-input avionics supply (main bus plus a backup 2S cell, about 50 g, 15 min); servo BEC sized for two stalled servos | 1C **High** | 1E Medium | **REQUIRED** (RC2) |

**Table 7.** Component failure modes, effects, detection, mitigation and risk: structure, controls, release and recovery.

| # | Item | Failure mode | Local effect | System effect | Detection | Mitigation | Initial | Residual | Flag |
|---|---|---|---|---|---|---|---|---|---|
| 10 | Elevon servo (≥ 0.9 N·m, 2:1 linkage) | Gear strip, motor burn-out, hard-over | One elevon free, frozen or at full travel | Two surfaces only and no redundancy. A frozen surface leaves degraded control; a hard-over is likely loss of control (H2) | Servo current and position feedback (if fitted); attitude error | F4 torque margin; F8 holds position on signal loss and does not back-drive. RC1 limits the consequence. Option for v2: split each elevon into two surfaces with two servos | 1C **High** | 2D Medium | **REQUIRED** (RC1) |
| 11 | Linkage and hinge | Clevis disconnect, free play growth, hinge tape peel | Free-floating elevon | With F7, a free-floating elevon is flutter-free to ≥ 54.2 m/s EAS; without the balance, flutter at 29.2 m/s (inside VD). Loss of one surface: as row 10 | Preflight free-play check (≤ 0.15°, F8); attitude error | F7, F8; clevises with locking safety clips; per-flight check | 1D Serious | 1E Medium | |
| 12 | Elevon mass balance (F7, 60 g) | Detaches | Surface unbalanced | Flutter only if the linkage also fails (double fault) | Preflight check | Bonded and mechanically keyed balance | 1E Medium | 1E Medium | |
| 13 | Wing joint (20 × 16 mm tube, sleeves) | Retention loss, tube crack, stiffness below 1,000 N·m/rad | Panel separation, or body-freedom flutter below VD | H7, then H2 | GVT and stiffness test (T7); preflight latch check | F9, F12, F13; two independent retention latches per joint | 1D Serious | 1E Medium | |
| 14 | Wing structure, pod saddle, skid | Overload in a gust or a hard landing; crushed foam | Skin wrinkling or saddle failure | H7 | Inspection after each landing | F10, F11, F14, F15; FE margins ≥ 0 | 1D Serious | 1E Medium | |
| 15 | Kit release mechanism | Inadvertent release from servo glitch, EMI or software | Kit falls from 120 m | H5 (over a trail) | None until it happens | RC6: two-stage arm and fire; release inhibited unless in drop mode, at 35–45 m AGL, within 30 m of the target; latch needs powered travel to open, holds on signal loss | 1D Serious | 1E Medium | **REQUIRED** (RC6) |
| 16 | Kit release mechanism | Fails to release | Kit stays on board | Mission degraded (no kit); the ground team still goes | Release confirmation switch | Retry, then return with the kit (endurance already counts it) | 4C Low | 4C Low | |
| 17 | Kit parachute (7 m/s, 1.1 m) | Fails to open or entangles | Kit falls from 40 m at up to about 25 m/s (about 500 J) | H5 near the person | Visible on the video | Static-line deployment; packing check; 5 m aim offset; drop only when other people are clear of the R95 circle (18–27 m) | 1D Serious | 1E Medium | |
| 18 | Aircraft parachute | **Not fitted on v1** | — | No flight termination short of a dive or spin; every H2/H4 outcome is an uncontrolled impact | — | RC1: ASTM F3322 parachute with an independent trigger (its own processor and battery; it fires on geofence breach, attitude limits or a manual command) | 1C **High** | 2D Medium | **REQUIRED** (RC1) |
| 19 | Belly skid (F10) | Missing or crushed from a previous landing | Root skin and cap over-stressed on landing | Aircraft damage (Marginal) | Preflight check | F10; replace after a hard landing | 3C Medium | 3D Medium | |
| 20 | Bungee launch | Release below 1.1 × stall at high density altitude, or crosswind | Stall at launch next to the crew | Crew injury | Launch speed log | L3, L4; crew stands clear of the launch line | 2C Serious | 2D Medium | |

**Table 8.** Component failure modes, effects, detection, mitigation and risk: avionics, sensors and links.

| # | Item | Failure mode | Local effect | System effect | Detection | Mitigation | Initial | Residual | Flag |
|---|---|---|---|---|---|---|---|---|---|
| 21 | Autopilot and IMU | IMU fault, vibration aliasing, processor lock-up | Wrong attitude estimate or no control output | H2 | Redundant-IMU consistency check, watchdog | Triple-IMU flight controller with isolation; ESC-RPM notch (F3); RC1 independent termination | 1D Serious | 2E Medium | |
| 22 | GNSS | Outage in a canyon, multipath off cliffs, jamming | Position error or loss | H4 | EKF innovation checks; satellite count | RC10: dual GNSS (two constellations, separate antennas); dead reckoning on airspeed and heading; geofence with termination (needs RC1) | 1D Serious | 1E Medium | **REQUIRED** (RC10) |
| 23 | Compass | Interference from the power leads | Heading error | Navigation drift | EKF yaw innovation | F6 twisted pairs; compassmot calibration | 2D Medium | 2E Medium | |
| 24 | Pitot and airspeed sensor (combined with airspeed failure) | Blocked by an insect, water or ice; broken on landing; sensor drift | Airspeed reads low or high | Stall (reads high) or overspeed (reads low), then H2 (H11) | Airspeed against ground speed and wind estimate; EKF airspeed innovation | F5 position; RC4: automatic airspeed-failure detection with fallback to synthetic airspeed or throttle-pitch tables; preflight pitot test; L7 icing limit (no heated probe) | 1C **High** | 1E Medium | **REQUIRED** (RC4) |
| 25 | C2 radio (900 MHz) | Terrain outage (certain on 6 km routes), radio failure | No command or telemetry | H3; contributes to H8 | Link-quality monitor | RC5 lost-link behaviour, 10 m mast, relay, pre-flight link map | 3A Serious | 4A Medium | **REQUIRED** (RC5) |
| 26 | Video link (2.4 GHz) | Terrain outage (65–100% availability, Table 12), interference | No live video | Detections are not seen live; operator awareness is lost | Link monitor | Record on board; on-board hotspot detection, geotagged and sent when the link returns; relay (RC5) | 4A Medium | 4B Medium | |
| 27 | Camera gimbal | Stuck or runaway | Camera off nadir | Missed detections (mission) | Gimbal attitude feedback | Lock gimbal at nadir on fault | 4C Low | 4C Low | |
| 28 | Remote ID module | No broadcast | Non-compliant flight | Must land under Part 89 | Self-test before takeoff | Takeoff inhibit if Remote ID fails | 4C Low | 4D Low | |

## 5. G3: Contingency physical side

### 5.1 Glide polar

The polar comes from the drag model the propulsion margins use: NeuralFoil MH45 profile drag times the finish factor, induced drag and the pod (`propulsion_margins.drag`). The folded prop is treated as drag-free. That model gives a best L/D of 17.8. The aero cross-check put the realistic range at **15–19**, so the reach maps use a **planning polar scaled to L/D 15**.

**Table 9.** Best glide and ground glide ratio vs. headwind, kit on (8.315 kg), by altitude.

| Density altitude [m] | Stall [m/s] | Model L/D | Best-glide speed [m/s] | Sink [m/s] | Ground glide ratio, planning polar, 0 / 5 / 10 m/s headwind | Reach from 120 m AGL, flat ground, 0 / 5 / 10 m/s [km] |
|---:|---:|---:|---:|---:|---|---|
| 0 | 11.4 | 18.0 | 15.7 | 0.87 | 15.0 / 10.5 / 6.9 | 1.80 / 1.26 / 0.82 |
| 2,000 | 12.6 | 17.8 | 17.3 | 0.98 | 15.0 / 10.9 / 7.4 | 1.80 / 1.30 / 0.89 |
| 4,400 | 14.2 | 17.5 | 19.6 | 1.12 | 15.0 / 11.3 / 8.1 | 1.80 / 1.36 / 0.97 |

*Speed-to-fly into the headwind rises to 22–24 m/s at 10 m/s. After the kit drop (6.69 kg), the ground glide ratio into 10 m/s falls by about 0.7. Source: `out/safety/glide_polar.csv`.*

**Cross-check pending.** The 6-DOF glide polar from the v1 JSBSim rebuild (`out/sixdof_v1/glide_polar.csv`) did not exist when this ran. `system_safety.py` prints its best L/D next to ours automatically when it appears. If it comes in below 15, the reach fractions below are optimistic.

### 5.2 Motor-out reach on the 7 sites

**Model.**
- **Route:** the cached trails-first routes from `terrain_sim.prep` (Lodestar camera spacing). They are resampled every 60 m and cut at 175 km, about the distance flown in one sortie at 18 m/s before the bingo point. The route includes the expanding square that follows the trail tour.
- **Glide test:** every second sample (120 m) tries a straight-line glide at the terrain-following search altitude. The glide must keep **30 m** above the terrain (tree tops) all the way, and arrive 30 m above the destination.
- **Destinations:** first the launch point, then up to 12 of the nearest open landing areas (listed below) inside glide range.
- **Wind:** the headwind is applied on every heading (worst case).
- **Simplifications:** the path is straight, with no turning around a ridge, and the Earth is flat.

**Landing areas** are 90 × 90 m blocks of 30 m cells that:
- have tree canopy under 10% (NLCD);
- have a slope under 12°;
- are not a flat water surface;
- lie at least 150 m from any mapped trail or road.

They make up 0–1.6% of each map. The aircraft may be damaged there, but no third party is at risk.

**Table 10.** Fraction of the route from which a motor-out glide reaches the launch point, or the launch point or an open landing area, vs. headwind, by site and pattern (planning L/D 15).

| Site | Pattern | Launch point, 0 / 5 / 10 m/s | Launch or landing area, 0 / 5 / 10 m/s | Route within 50 m of a trail or road |
|---|---|---|---|---|
| Yosemite: Happy Isles | 3 km | 0.69 / 0.61 / 0.57 | 0.69 / 0.61 / 0.57 | 0.69 |
| | 6 km | 0.29 / 0.21 / 0.17 | 0.29 / 0.21 / 0.17 | 0.92 |
| Sequoia: Lodgepole | 3 km | 0.94 / 0.90 / 0.82 | 0.97 / 0.90 / 0.82 | 0.41 |
| | 6 km | 0.69 / 0.55 / 0.47 | 0.92 / 0.63 / 0.47 | 0.67 |
| Rocky Mtn: Bear Lake | 3 km | 0.73 / 0.66 / 0.54 | 0.79 / 0.74 / 0.54 | 0.44 |
| | 6 km | 0.48 / 0.45 / 0.38 | 0.66 / 0.60 / 0.47 | 0.74 |
| Olympic: Hurricane Ridge | 3 km | 0.16 / 0.10 / 0.08 | 0.92 / 0.85 / 0.60 | 0.25 |
| | 6 km | 0.18 / 0.11 / 0.08 | 0.89 / 0.82 / 0.58 | 0.36 |
| Mt Rainier: Paradise | 3 km | 0.55 / 0.48 / 0.40 | 0.98 / 0.97 / 0.94 | 0.60 |
| | 6 km | 0.55 / 0.48 / 0.40 | 0.94 / 0.93 / 0.89 | 0.75 |
| Great Smokies: Newfound Gap | 3 km | 0.50 / 0.41 / 0.28 | 0.50 / 0.41 / 0.28 | 0.34 |
| | 6 km | 0.50 / 0.35 / 0.24 | 0.51 / 0.35 / 0.24 | 0.49 |
| White Mtns: Pinkham Notch | 3 km | 0.91 / 0.86 / 0.77 | 0.91 / 0.86 / 0.77 | 0.51 |
| | 6 km | 0.82 / 0.77 / 0.69 | 0.84 / 0.78 / 0.69 | 0.82 |
| **Mean of 7** | 3 km | **0.64 / 0.58 / 0.49** | **0.82 / 0.76 / 0.65** | **0.46** |
| | 6 km | **0.50 / 0.42 / 0.35** | **0.72 / 0.62 / 0.50** | **0.68** |

*Planning polar L/D 15, kit on, 30 m terrain clearance, straight glide, headwind on every heading. Route: 175 km of the trails-first pattern with the expanding square. With the unscaled model polar (L/D 17.8) the means rise by 2–5 points. Over the trail part of the route alone, the launch-point fractions are 0.61 / 0.52 / 0.42 (3 km) and 0.47 / 0.37 / 0.30 (6 km). Source: `out/safety/glide_reach.csv`.*

**What this means.**
- A motor-out ends in a recovered aircraft about half the time. When it doesn't, it usually ends in a damaged aircraft in open ground or in a tree landing.
- The terrain sets the result more than the polar does:
  - **Hurricane Ridge** sits on a ridge road. The launch point is rarely reachable, but subalpine meadows are everywhere.
  - **The Smokies and the Whites** are closed forest with almost no open ground, so the fallback is the trees.
  - **At Yosemite**, the 6 km tour climbs the valley walls; from the rim, the floor is out of reach.
- The near-trail column is the ground-risk exposure. A forced landing chosen by the autopilot can avoid the trail, which is change RC9. A loss of control cannot avoid it, which is why RC1 is needed.

### 5.3 Link-loss return energy

The rule (L2, `lodestar_v1.OPERATIONS`): head home when the remaining energy, less the 10% landing reserve, falls to 1.2 × E_return(d, wind + 3 m/s). Here E_return comes from the propulsion model's best-return airspeed into the wind, plus the energy to climb over the ridges on the straight line home (`terrain_sim.return_climb`), at an assumed motor efficiency of 0.6.

**Table 11.** Return energy and bingo threshold from the farthest route point vs. headwind, by site and pattern (SiCore, kit on, 578 Wh usable).

| Site | Pattern | Farthest point [km] | Climb on the way home [m] | E_return, 0 / 10 m/s headwind [Wh] | Bingo at 10 m/s [Wh] | Bingo [% usable] |
|---|---|---:|---:|---|---:|---:|
| Yosemite | 3 km | 2.8 | 71 | 10 / 19 | 80 | 13.9 |
| | 6 km | 4.8 | 1,103 | 55 / 69 | 141 | 24.4 |
| Sequoia | 3 km | 2.7 | 109 | 11 / 19 | 80 | 13.9 |
| | 6 km | 5.2 | 257 | 24 / 38 | 104 | 18.0 |
| Rocky Mtn | 3 km | 2.4 | 159 | 13 / 19 | 81 | 13.9 |
| | 6 km | 4.7 | 112 | 17 / 29 | 93 | 16.1 |
| Olympic | 3 km | 2.8 | 210 | 16 / 24 | 86 | 14.9 |
| | 6 km | 5.6 | 389 | 30 / 46 | 113 | 19.6 |
| Rainier | 3 km | 2.2 | 231 | 15 / 21 | 83 | 14.4 |
| | 6 km | 4.9 | 0 | 14 / 28 | 91 | 15.7 |
| Smokies | 3 km | 2.5 | 213 | 15 / 22 | 85 | 14.6 |
| | 6 km | 4.9 | 309 | 25 / 40 | 105 | 18.2 |
| Whites | 3 km | 2.8 | 129 | 13 / 21 | 84 | 14.5 |
| | 6 km | 5.4 | 375 | 29 / 46 | 113 | 19.6 |

*Wind margin of +3 m/s on top of the stated headwind, as in L2. Return airspeed 19–27 m/s; return time 2–7 min. Bingo includes the 57.8 Wh landing reserve. Source: `out/safety/link_loss_energy.csv`.*

**The return is cheap compared with the battery.** The farthest points are only 2–6 km out, so even the Yosemite case (1.1 km of climbing over the valley wall) leaves three-quarters of the energy for searching. The requirement this creates is about where the bingo is computed, not about energy. The bingo must be evaluated **on the aircraft**, with its own wind estimate, so that it still works during a 20-minute outage. That is part of RC5.

### 5.4 Flight termination

**v1 carries no aircraft parachute and no independent flight-termination system.** The only way to end a flight short of landing is to cut the motor and either glide, spin or dive. The consequence is that every "terminate" outcome is an uncontrolled impact. Table 12 gives the energies.

**Table 12.** Impact speed and energy vs. termination mode, by mass state (2,000 m, ISA + 15 °C).

| Mode | Kit on: speed [m/s] / energy [J] | Kit released: speed [m/s] / energy [J] | × Part 107 Cat. 3 limit (33.9 J), kit on |
|---|---|---|---|
| Controlled glide, belly landing at best-glide speed | 17.3 / 1,252 | 15.6 / 810 | 37 |
| Tumbling or flat-spin descent (CD ≈ 1.0 on the planform) | 11.2 / 525 | 10.1 / 340 | 15 |
| Steep dive from 120 m AGL (CD0 ≈ 0.02) | 46.6 / 9,039 | 45.1 / 6,812 | 267 |
| Terminal parachute, 6.7 m/s (4.5 m², 2.4 m flat diameter) | 6.7 / 187 | 6.7 / 150 | 5.5 |
| Terminal parachute, 5.0 m/s (8.1 m², 3.2 m flat diameter) | 5.0 / 104 | 5.0 / 84 | 3.1 |

*Parachute area from CD 0.8. The Category 2 and 3 limits (11 and 25 ft-lb) are from [14 CFR 107.120](https://www.law.cornell.edu/cfr/text/14/107.120) and [107.130](https://www.law.cornell.edu/cfr/text/14/107.130). Source: `out/safety/termination.csv`.*

**Even with a parachute, Lodestar cannot meet any Part 107 over-people category.** The parachute changes the outcome from likely fatal (kilojoules) to likely survivable (about 100–190 J, a heavy blow). That is the SORA M2 credit and the basis of a 107.39 waiver.

The parachute has a cost:
- **Mass:** about 0.4 kg for a 2.4–3.2 m canopy, lines and an ejector (estimate: canopy fabric at 40–60 g/m²).
- **Endurance:** at constant lift coefficient, endurance scales roughly with m^−1.5, so 8.3 kg → 8.7 kg costs about 7%. The design-case endurance falls from **3.09 h to about 2.9 h**, below R1. The best-speed 3.68 h figure still passes.
- **R1 needs a decision:** either relax R1 for the BVLOS configuration, or find the mass elsewhere (the 1.7 kg margin to 10 kg is not the issue; the energy is).

## 6. G4: C2 link budget in mountain terrain

### 6.1 Radios and free-space budget

**Table 13.** Radio parameters and free-space margin vs. distance, by link.

| Link | Radio | Band | Tx power | Sensitivity | Margin at 5 / 10 km [dB] | Free-space range at a 10 dB fade margin |
|---|---|---|---|---|---|---|
| C2 telemetry | RFDesign RFD900x ([spec](https://docs.freeflysystems.com/products/products/alta-x/untitled-5/uxv-navtab-w-rfd900-legacy/uxv-rfd900-radio-tech-specs)) | 902–928 MHz | 30 dBm (1 W) | −105 dBm at 64 kbps | 34.3 / 28.3 | 82 km |
| Video | Microhard pMDDL2450 ([brochure](https://www.microhardcorp.com/brochures/pMDDL2450.Brochure.Rev.1.3.0.pdf)) | 2.4 GHz | 30 dBm | −98 dBm, QPSK 1/2, 8 MHz, 2×2 MIMO (5.9 Mbps) | 18.8 / 12.8 | 13.8 km |

*Antennas: 2 dBi on the aircraft (winglet, per the controls note) and a 5 dBi omni at the ground station. 1 dB cable loss at each end. EIRP is inside the [47 CFR 15.247](https://www.law.cornell.edu/cfr/text/47/15.247) limit of 1 W conducted with up to 6 dBi of antenna gain. The 10 dB fade margin is an assumed planning value for multipath off rock walls. The RFD900x sensitivity is quoted from −121 dBm (lowest rates) down to −98 dBm (200 kbps) across datasheet revisions. −105 dBm at 64 kbps is the conservative MAVLink-rate figure.*

Free space is not the constraint: the search radius is 3–6 km, and both radios have margin there. **Terrain is the constraint.**

### 6.2 Terrain masking

The model takes a straight path from the station to each route sample (60 m spacing) over the 30 m DEM:
- **Earth bulge** with k = 4/3.
- **Diffraction** at the worst obstruction, as an [ITU-R P.526](https://www.itu.int/rec/R-REC-P.526) single knife edge, J(v).
- **Link rule:** a sample has link when the margin after free-space loss and diffraction is at least 10 dB.

A single knife edge is optimistic behind several rounded ridges. Table 15 therefore also gives the pessimistic bound, in which geometric line of sight is required. The truth lies between the two.

**Stations:**
- **Ground station** at the launch point, on a 2 m or a 10 m mast.
- **Optional relay** (2 m mast). It goes on one of the 40 highest points within 5 km of launch and within 100 m of a trail or road (so a team can carry it there), sampled every 300 m. The chosen point must have line of sight and the fade margin to the 10 m station, and adds the most 900 MHz coverage on the 6 km route.

**Table 14.** Fraction of the route with C2 and video link vs. station, by site and pattern.

| Site | Pattern | C2 900 MHz: 2 m / 10 m / 10 m + relay | Video 2.4 GHz: 2 m / 10 m / 10 m + relay |
|---|---|---|---|
| Yosemite | 3 km | 0.93 / 0.94 / 0.94 | 0.90 / 0.91 / 0.91 |
| | 6 km | 0.75 / 0.75 / 0.75 | 0.72 / 0.72 / 0.72 |
| Sequoia | 3 km | 1.00 / 1.00 / 1.00 | 0.78 / 0.80 / 1.00 |
| | 6 km | 0.75 / 0.76 / 0.90 | 0.65 / 0.65 / 0.84 |
| Rocky Mtn | 3 km | 1.00 / 1.00 / 1.00 | 0.92 / 0.94 / 1.00 |
| | 6 km | 0.86 / 0.87 / 0.97 | 0.64 / 0.80 / 0.92 |
| Olympic | 3 km | 1.00 / 1.00 / 1.00 | 0.84 / 0.95 / 0.98 |
| | 6 km | 0.98 / 0.98 / 1.00 | 0.86 / 0.92 / 0.99 |
| Rainier | 3 km | 1.00 / 1.00 / 1.00 | 0.92 / 0.94 / 1.00 |
| | 6 km | 0.97 / 0.98 / 1.00 | 0.86 / 0.87 / 1.00 |
| Smokies | 3 km | 1.00 / 1.00 / 1.00 | 0.95 / 0.97 / 0.97 |
| | 6 km | 0.94 / 0.95 / 0.99 | 0.83 / 0.86 / 0.94 |
| Whites | 3 km | 1.00 / 1.00 / 1.00 | 0.99 / 0.99 / 1.00 |
| | 6 km | 0.94 / 0.94 / 0.99 | 0.85 / 0.86 / 0.94 |
| **Mean of 7** | 3 km | **0.99 / 0.99 / 0.99** | **0.90 / 0.93 / 0.98** |
| | 6 km | **0.89 / 0.89 / 0.94** | **0.77 / 0.81 / 0.91** |

*Knife-edge diffraction credit, 10 dB fade margin, aircraft at the terrain-following altitude (120 m AGL). Relay sites: 2.1–4.8 km from launch and 92–1,425 m above it. At Yosemite, no candidate sees the station, so the relay adds nothing there. Source: `out/safety/link_budget.csv`.*

**Table 15.** Longest continuous outage at 18 m/s vs. station, by site and pattern [min] (knife-edge model; line-of-sight-only bound in brackets).

| Site | Pattern | C2: 2 m | C2: 10 m | C2: 10 m + relay | Video: 10 m + relay |
|---|---|---|---|---|---|
| Yosemite | 3 km | 4.9 (5.3) | 4.9 (5.3) | 4.9 (5.3) | 5.3 |
| | 6 km | 15.3 (15.5) | 15.3 (15.5) | 15.3 (15.5) | 15.4 |
| Sequoia | 3 km | 0.3 (4.3) | 0.1 (4.1) | 0.0 (0.3) | 0.0 |
| | 6 km | 20.9 (22.1) | 20.8 (21.9) | 6.9 (16.2) | 16.1 |
| Rocky Mtn | 3 km | 0.0 (2.9) | 0.0 (2.3) | 0.0 (0.1) | 0.1 |
| | 6 km | 15.4 (28.5) | 8.7 (17.4) | 3.8 (3.9) | 3.9 |
| Olympic | 3 km | 0.0 (4.1) | 0.0 (2.0) | 0.0 (2.0) | 0.8 |
| | 6 km | 3.5 (6.2) | 3.3 (5.9) | 0.0 (0.8) | 0.4 |
| Rainier | 3 km | 0.0 (3.1) | 0.0 (2.2) | 0.0 (0.1) | 0.0 |
| | 6 km | 2.3 (8.9) | 2.2 (8.3) | 0.0 (0.0) | 0.0 |
| Smokies | 3 km | 0.1 (4.8) | 0.0 (1.2) | 0.0 (1.0) | 0.9 |
| | 6 km | 2.6 (14.5) | 2.6 (12.0) | 0.8 (2.8) | 2.8 |
| Whites | 3 km | 0.2 (2.1) | 0.2 (0.8) | 0.0 (0.8) | 0.4 |
| | 6 km | 2.6 (3.5) | 2.5 (3.2) | 0.5 (2.1) | 2.1 |

*Route sampled every 60 m along the flown path, at 18 m/s. Geometric line of sight from the launch point covers 83% (2 m mast) and 90% (10 m mast) of the 3 km routes, and 72% and 78% of the 6 km routes. 60% first-Fresnel-zone clearance is a few points lower again. On flat ground, a 2 m mast loses 60% clearance near the mast beyond about 8 km (test check).*

**What this implies.**
- **The 3 km routes are a good link case.** C2 holds for 93–100% of the route from a 2 m mast, and the worst outage is about 5 min (Yosemite).
- **The 6 km routes have long shadows behind ridges and valley walls.** The 900 MHz link drops for up to 21 min (Sequoia) or 15 min (Yosemite, Rocky Mountain on a 2 m mast). With line of sight required, Rocky Mountain on a 2 m mast reaches 28 min.
- **A 10 m mast helps the line-of-sight fraction** (72% → 78% on 6 km routes) but not the worst shadows.
- **A carried relay helps most where a high trail point sees both sides** (Rocky Mountain, Rainier, Olympic, Smokies, Whites). But the relay sites are 0.1–1.4 km of climbing from the trailhead, which is a team task of an hour or more, not a deployment step.
- **The video link is worse at 2.4 GHz** (Fresnel zone and diffraction loss), so a live-video search covers only 72–100% of the route even with the relay.

**Consequences for the design:**
- **Autonomous search with return-on-loss is required, but "return immediately on loss" is the wrong behaviour.** On the 6 km routes it would abort nearly every sortie in its first shadow. The aircraft must keep flying the pre-loaded route through predicted outages, with the bingo, geofence and kit-release inhibits computed on board. It should return home only when an outage lasts longer than predicted for that leg (plus a margin), or when an on-board health check fails.
- **Detection cannot depend on live video.** The aircraft must record and detect on board, and send geotagged hits when the link returns. Otherwise the search is blind for 10–30% of the 6 km route.
- **The pre-flight plan should include this link map** (the functions in `system_safety.py` already compute it for any route), so the pilot knows where and for how long the aircraft will be out of contact.

## 7. Required design and operations changes

These are proposals; `lodestar_v1.py` is not changed. They are in priority order.

**Table 16.** Required changes, by hazard closed, with mass and performance impact.

| ID | Change | Type | Closes | Impact |
|---|---|---|---|---|
| RC1 | Terminal aircraft parachute to ASTM F3322 ([store listing](https://store.astm.org/sub-uascmp.html)), 6.7 m/s or slower, with an independent trigger (its own processor and battery: geofence breach, attitude or spin detection, manual command) | Design | H2, H4, H7, H9; FMEA 10, 18, 21 | About +0.4 kg; endurance about −7%, so R1 at the design case drops to about 2.9 h (needs a decision); SORA M2 −1 |
| RC2 | Independent avionics supply: dual-input power module fed from the main bus and a backup 2S pack (about 50 g, about 15 min of avionics and servos); locking pack connectors with a retention clip; servo BEC sized for two stalled servos | Design | H10; FMEA 6, 9 | About +70 g |
| RC3 | Battery safety: BMS with cell-voltage and temperature monitoring and cell fuse links; heater locked out above +10 °C ambient, with an independent 50 °C cut-out in series; mica or ceramic-paper liner in each bay; fire watch at any crash site in dry fuel; the hot-day pack temperature with the F1 insulation to be checked under G13 | Design and operations | H6; FMEA 4, 7 | About +50 g |
| RC4 | Airspeed-failure detection with automatic fallback (synthetic airspeed or throttle-pitch tables); preflight pitot test; new limit L7: no flight in visible moisture at or below +3 °C OAT (no heated probe) | Design (configuration) and operations | H11; FMEA 24 | None |
| RC5 | Lost-link behaviour: continue the pre-loaded route through predicted outages, with bingo (L2), geofence and release inhibits computed on board; return home if an outage outlasts its prediction by 2 min, or immediately if there is no prediction. Pre-flight terrain link map for the planned route; 10 m mast at the ground station; relay carried to a high point when the map shows long shadows; on-board recording and hotspot detection | Design (software requirement) and operations | H3; FMEA 25, 26 | None (radio relay kit on the ground) |
| RC6 | Kit release interlock: two-stage arm and fire; release inhibited unless in drop mode, at 35–45 m AGL and within 30 m of the target; a latch that needs powered travel to open and holds on signal loss | Design | H5; FMEA 15 | Negligible |
| RC7 | People under the route: apply for a [107.39](https://www.law.cornell.edu/cfr/text/14/107.39) waiver (or fly under a public COA) using this FHA as the safety case. In the planner, offset trail tracks laterally by about 40 m so that hikers on the trail are not overflown; the trail stays inside the 112 m-wide footprint of the 50° camera at 120 m. Drop only when nobody else is inside the R95 circle | Operations, planner | H9 | Detection change from the offset to be measured in Step 3 |
| RC8 | Airspace at the incident: fly only inside an altitude and area block agreed with the incident air-operations branch (TFR under [91.137](https://www.law.cornell.edu/cfr/text/14/91.137)); ADS-B In on the aircraft and the ground station, with an autonomous hold or descend on traffic inside a set range (the aircraft must not transmit ADS-B Out under Part 107, [14 CFR 107.53](https://www.law.cornell.edu/cfr/text/14/107.53)); anti-collision strobes visible to 3 sm ([107.29](https://www.law.cornell.edu/cfr/text/14/107.29)) | Operations and design | H8 | About +30 g |
| RC9 | Motor-out logic: glide to the best reachable destination from a pre-loaded table (the launch point first, then the open landing areas away from trails that this analysis finds), not straight home | Design (software requirement) | H1; FMEA 1–3 | None |
| RC10 | Dual GNSS (two constellations, separate antennas), dead reckoning on airspeed and heading, and a geofence that triggers RC1 | Design | H4; FMEA 22 | About +40 g |

**Combined effect.**
- The changes add about **0.6 kg** (8.3 → about 8.9 kg), still under 10 kg. Most of that is the parachute.
- R1 endurance must then be re-run: by the m^−1.5 scaling it falls to **about 2.8 h** at the design case.
- This is the main design conflict this note exposes. A mission-capable BVLOS configuration and the 3 h endurance requirement no longer fit together at the current battery. Step 3 should weigh:
  - a larger pack;
  - the bay battery module;
  - a 2.8 h requirement for the BVLOS configuration.

**Table 17.** Hazard risk before and after the required changes.

| ID | Hazard | v1 as frozen | With RC1–RC10 |
|---|---|---|---|
| H1 | Loss of thrust | Serious | Medium |
| H2 | Loss of control | High | Medium |
| H3 | Loss of C2 link | Serious | Medium |
| H4 | Loss of navigation | Serious | Medium |
| H5 | Kit misrelease | Serious | Medium |
| H6 | Energy storage | Serious | Medium |
| H7 | Structural failure | Serious | Medium |
| H8 | Mid-air with manned aircraft | High | **Serious** (accepted per operation, with incident airspace integration) |
| H9 | People under the route | High | Medium |
| H10 | Avionics power loss | High | Medium |
| H11 | Airspeed failure | High | Medium |

*Severity and probability scales from Tables 1–3.*

## 8. Limits of this analysis

- **Probability levels are judgement.** There is no Lodestar reliability data. The 150 h reliability flying that Part 108 proposes is what would set them.
- **The glide reach** uses straight-line glides, a worst-case headwind on every heading and a polar that the 6-DOF model has not yet confirmed. Turning around a ridge, or using a tailwind, would only increase reach.
- **Landing areas** come from 30 m canopy and slope data. They do not see boulders, snow, power lines or campers.
- **The link model** uses one dominant knife edge and no vegetation loss. Wet conifer canopy at the aircraft's antenna height is not counted, but the aircraft flies 120 m above the canopy, so the dominant loss is at the ridges. Real outages could be longer than the knife-edge numbers; the line-of-sight-only bound is the pessimistic case.
- **The relay site** is chosen to maximise coverage, not for how easily people can reach it. Some of the chosen points are summits.
- **The parachute mass and endurance penalty** are estimates. They need a product selection (ASTM F3322-listed) and a re-run of the performance model.
- **Population density on trails** is not measured. The SORA classes in Table 4 are argued from typical use, not from a population map.

## Reproduce

From `scripts/`:
- `../.venv/Scripts/python.exe -X utf8 system_safety.py` runs everything in about 2 minutes and writes `out/safety/`: `glide_polar.csv`, `glide_reach.csv`, `link_budget.csv`, `link_loss_energy.csv`, `termination.csv`, `radio_budget.csv` and `safety_log.txt`.
- `../.venv/Scripts/python.exe -X utf8 test_system_safety.py` runs the known-answer checks:
  - free-space loss;
  - Fresnel radius;
  - knife-edge loss;
  - line of sight over flat ground and a ridge, and Earth bulge;
  - glide distance equal to height × L/D;
  - speed-to-fly;
  - impact energies;
  - landing-area filters;
  - route resampling.
