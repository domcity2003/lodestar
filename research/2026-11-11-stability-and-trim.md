# Milestone 2: stability and trim, kit on and kit off

2026-11-11 (US Central).
Code: `scripts/stability.py` (study), `scripts/avl_run.py` (AVL driver). Outputs: `out/stability/` (CSVs, charts, `run_log.txt`, final AVL input files in `out/stability/avl/`).

## Answer first
**The Rescue Wing is stable and trimmable from near-stall to top speed, with the kit on and after it drops.** All 8 cases tested (12.5, 15, 18 and 22 m/s, each with and without the kit) meet the standard handling criteria for small aircraft.

The design choices that get there:
| Choice | Value | Why |
|---|---|---|
| Balance point | x = 0.437 m from the root leading edge (21.4% MAC), **6% MAC ahead of the neutral point** | Stable, while the elevons stay within about ±2° of neutral when trimmed |
| Kit bay | **centred on the balance point** | Dropping 1.7 kg then barely moves the balance: stability margin 5.8% → 5.1% |
| Battery | two packs in the wing roots, at x = 0.49 m, 0.2 m either side of centre | The heavy item that is moved to set the balance |
| Washout | 3° (tips twisted nose-down) | Trim close to neutral elevon |
| Winglets | **0.25 m tall**, swept 35° | The smallest size that keeps Dutch roll damped at top speed |
| Elevons | outer 70% of the span, 25% of the chord | Trim deflection −1.8° to +1.3° across the whole speed range |

**What changed from v0.1:** take-off mass is 7.55 kg (+0.11 kg for winglets and the real chute weight). With the sizing's cautious drag assumption, endurance is 2.97 h (2 minutes short of 3 h). AVL's drag-due-to-lift is lower than assumed, so the true figure is likely 3.0-3.3 h. Milestone 3's CFD should settle this.

## Plain-language background
- **Neutral point:** the balance position where the aircraft doesn't care whether its nose goes up or down. Put the balance point ahead of it and the aircraft weathervanes back to level flight on its own.
- **Static margin:** how far ahead, as a percentage of the average chord (MAC = 0.449 m). Too little and it's twitchy; too much and the elevons must hold the nose up all the time, which costs lift and adds drag.
- **Why flying wings are hard:** a normal plane has a tail far behind the wing to do this job. A flying wing has to manage with sweep, twist and a reflexed airfoil (its trailing edge turns slightly up), and its margin for error is small. Moving the 1.7 kg kit 10 cm the wrong way would leave the aircraft unstable after the drop (see below).
- **Flight modes** are the aircraft's natural wobbles after a bump:
  - **short period:** a quick nose bob;
  - **phugoid:** a slow swap between height and speed;
  - **Dutch roll:** a combined wag of yaw and roll;
  - **roll:** how fast the aircraft settles into a bank;
  - **spiral:** a slow tendency to tighten or straighten out of a turn.

  Each needs enough damping, the way a car needs shock absorbers.

## Method
1. **Geometry from milestone 1:** 3.0 m span, root chord 0.591 m, tip chord 0.266 m, 25° quarter-chord sweep, MH45 airfoil, elevons from 0.45 m out to the tip.
2. **Mass model:** every item placed by position:
   - wing structure spread along the span, each strip at 40% of its chord;
   - the camera gimbal under the nose and the motor at the tail (it's a pusher);
   - kit and battery positioned to set the balance point.

   This gives the inertias needed for the flight modes.
3. **AVL 3.52**, driven directly with keystrokes. AeroSandbox's AVL wrapper can't trim the elevons, so I wrote a small driver. Each case:
   - is level flight at the given speed, with lift equal to weight;
   - has the elevons trimmed to zero pitching moment and the ailerons to zero roll;
   - reports the stability derivatives, the neutral point, spanwise lift, and all 8 eigenvalues.

   The airfoil and body drag from the sizing (CD = 0.0121) goes into AVL, so the phugoid damping is realistic.
4. **Sweeps:** static margin 2-15%, washout 0-6°, winglet height 0-30 cm, speed 12.5-22 m/s with the kit on and off, and the kit bay moved ±10 cm.
5. **Criteria:** MIL-F-8785C, Class I (small aircraft), Category B (cruise and search flying), Level 1. These were written for piloted aircraft. An autopilot can fly worse, but meeting them makes the autopilot's job easy and a manual takeover safe.
   | Mode | Requirement |
   |---|---|
   | Short period damping | 0.30-2.0 |
   | Phugoid damping | ≥ 0.04 |
   | Dutch roll | damping ≥ 0.08, damping × frequency ≥ 0.15, frequency ≥ 0.4 rad/s |
   | Roll time constant | ≤ 1.4 s |
   | Spiral | if unstable, time to double ≥ 20 s |

## Results
### Neutral point: two independent tools agree
| Tool | Neutral point (no winglets) |
|---|---|
| AVL | 0.462 m |
| AeroSandbox vortex lattice | 0.461 m |

They differ by 0.1% of MAC. Adding 0.20 m winglets moves the neutral point to 0.464 m (27.4% MAC).

### Static margin trade (search speed, kit on)
| Static margin | Trim elevon | Short period damping | Phugoid damping | Endurance (AVL drag) |
|---|---|---|---|---|
| 2% | +1.6° | 0.81 | 0.065 | 3.29 h |
| **6%** | **−0.4°** | **0.62** | **0.056** | **3.28 h** |
| 10% | −2.5° | 0.55 | 0.068 | 3.26 h |
| 15% | −5.0° | 0.50 | 0.080 | 3.19 h |

Every option is stable. 6% is the middle ground: a margin for balance errors without paying for constant up-elevon.

### Washout
More washout trims the wing with less up-elevon (−1.9° at 0° washout → +1.0° at 6°) and slightly unloads the outboard wing, at a small endurance cost. **3°** puts the trim elevon at about 0° at search speed. The effect is small either way, so washout is a fine-tuning knob, not a driver.

### Winglets
| Height | Yaw stability Cnb | Dutch roll damping, search | Dutch roll damping, 22 m/s loaded | Meets criteria |
|---|---|---|---|---|
| 0 | 0.006 | 0.092 | 0.045 | no |
| 0.15 m | 0.012 | 0.093 | 0.065 | no |
| 0.20 m | 0.017 | 0.100 | 0.078 | no (just short) |
| **0.25 m** | **0.023** | **0.110** | **0.093** | **yes** |
| 0.30 m | 0.031 | 0.120 | 0.108 | yes |

Taller winglets damp the Dutch roll but turn the spiral mode mildly unstable. Even at 0.30 m it takes 45 s to double, well past the 20 s limit and easy for an autopilot.

### Final configuration across the speed range
| Kit | Speed | Static margin | Trim elevon | Short period damping | Phugoid damping | Dutch roll damping | Roll time constant | Spiral time to double |
|---|---|---|---|---|---|---|---|---|
| on | 12.5 m/s | 6.4% | −1.8° | 0.60 | 0.067 | 0.129 | 0.09 s | 340 s |
| on | 15 | 5.8% | −0.4° | 0.62 | 0.056 | 0.110 | 0.07 s | 58 s |
| on | 22 | 5.1% | +1.1° | 0.65 | 0.061 | 0.093 | 0.05 s | 78 s |
| off | 12.5 | 5.4% | −0.6° | 0.70 | 0.047 | 0.119 | 0.08 s | 58 s |
| off | 15 | 5.1% | +0.3° | 0.71 | 0.047 | 0.108 | 0.07 s | 46 s |
| off | 22 | 4.8% | +1.3° | 0.72 | 0.071 | 0.101 | 0.05 s | 97 s |

(18 m/s cases are in `final_speed_sweep.csv`; all pass.)

### The kit bay must sit on the balance point
| Kit bay vs balance point | Static margin before → after drop |
|---|---|
| 10 cm forward | 5.8% → **−1.4% (unstable)** |
| 5 cm forward | 5.8% → 1.8% |
| **on it** | **5.8% → 5.1%** |
| 5 cm aft | 5.8% → 8.4% |
| 10 cm aft | 5.8% → 11.7% |

This is the most important layout rule from this milestone. A kit bay forward of the balance point is dangerous; one slightly aft is safe. **Rule for CAD: kit bay centred on, or up to 5 cm behind, x = 0.437 m.**

### Was a 3D CLmax of 0.80 right? Yes, with margin
When trimmed to CL 0.8 (the stall speed in the sizing), the most loaded part of the wing reaches 69% of its 2D maximum lift (NeuralFoil MH45 at its local Reynolds number). That part is about 0.8-0.9 m out from the centreline. Extrapolated, the first strip reaches its maximum at a trimmed CL of about 1.17.

Vortex-lattice methods over-predict maximum lift, so real stall comes earlier. A conservative reading is CLmax ≈ 1.0, so **0.80 has about 25% margin.** Stall should also start mid-wing rather than at the tips, which keeps roll control while stalling.

### Cross-checks
| Quantity | AVL | Textbook approximation |
|---|---|---|
| Phugoid frequency | 0.76 rad/s | 0.93 (Lanchester) |
| Dutch roll frequency | 3.26 rad/s | 2.77 (yaw stiffness only) |
| Roll time constant | 0.070 s | 0.059 s |
| Neutral point | 0.462 m | 0.461 m (AeroSandbox VLM) |

The approximations ignore coupling, so a 15-20% gap is expected; same order of magnitude and right trends.

### Control authority
- **Roll:** 15° of aileron gives a steady roll rate pb/2V of 0.22, about 125°/s. That is 3× what small drones need, so roll throw can be limited to about ±5°, leaving most of the elevon travel for pitch.
- **Adverse yaw:** small but present. The yaw moment is 6% of the roll moment and swings the nose away from the turn. With no rudder, the autopilot handles it with a little coordinated pitch and bank; split drag-rudders on the winglets are the fix if flight sim shows sloppy turns.

## Charts
- `out/stability/static_margin_trade.png`: trim elevon and endurance vs static margin
- `out/stability/winglet_trade.png`: yaw stability and Dutch roll damping vs winglet height
- `out/stability/spanwise_lift_at_stall.png`: local lift vs local maximum at CL 0.8
- `out/stability/mass_layout.png`: where the heavy parts sit relative to the balance and neutral points

## Caveats
1. **AVL is inviscid** (no skin friction or flow separation). Its span efficiency e ≈ 1.03 is too optimistic for a real wing; 0.85-0.90 is typical once viscous effects are included. Endurance with the cautious sizing value (0.80) is 2.97 h; at 0.90 it is 3.12 h.
2. **The centre pod (camera, kit bay) isn't in the AVL model.** A pod ahead of the wing moves the neutral point forward, typically by a few % of MAC. Milestone 3 (OpenVSP with the pod) must re-check this, and the 6% margin is partly there to absorb it.
3. **Battery position is tight.** At x = 0.49 m and y = ±0.2 m, the packs sit at about 70% of the local chord, where the MH45 is about 27 mm thick. 21700 cells (21 mm) fit lying spanwise, but only just. The alternative is to move some battery into the pod. Check in CAD.
4. **Mass positions are estimates** until the CAD exists. The method re-solves the battery position for any layout, so this is cheap to redo.
5. **Elevon effectiveness from a vortex lattice** over-predicts at larger deflections. Trim needs only ±2°, so this barely matters here.
6. **The criteria are for piloted aircraft.** They make a good bar to clear, but the real check is closed-loop flight with an autopilot in turbulence: milestone 4.

## What feeds the next milestones
- Geometry: add 0.25 m winglets (35° sweep, taper 0.6), 3° washout.
- CAD layout rules: kit bay on x = 0.437 m (or up to 5 cm aft); battery packs centred at x ≈ 0.49 m; camera at the front of the pod; motor at the tail.
- Milestone 3 must re-check the neutral point with the pod modelled.
