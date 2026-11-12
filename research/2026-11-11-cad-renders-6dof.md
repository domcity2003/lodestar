# Milestone 5: detailed CAD, renders, and a 6-DOF check of the kit release

2026-11-11 (US Central). Analysis by Claude Opus 5.5 (claude-opus-5-5).

**Code:**
- `scripts/cad_model.py`: build123d CAD with the internal layout, plus fit, overlap and balance checks
- `scripts/render_v2.py` and `scripts/blender_v2.py`: Blender Cycles renders
- `scripts/sixdof_kit_release.py`: JSBSim 6-DOF model built from AVL, with an autopilot and turbulence Monte Carlo

**Outputs:**
- `out/cad/`: `rescue_wing_v2.step` (assembly), `alt_bay_battery_module.step`, `stl/` (31 parts), `fit_check.csv`, `mass_cg.csv`
- `out/render/v2_*.png`: hero, in flight, cutaway, exploded, kit drop
- `out/jsbsim/`: aircraft XML, `model_check.csv`, `release_results.csv`, `turbulence_summary.csv`, charts

## Answer first
**The kit drop is a non-event for the aircraft.** In calm air, the autopilot holds height through the release to within 0.7 m, and pitch moves by 2°.

**Why it's so gentle:**
- The kit sits on the balance point. In the CAD, dropping it moves the balance point by 0.5 mm.
- The only real effect is that the aircraft becomes 23% lighter in an instant.

**Even with the autopilot off,** with the controls frozen at the moment of release, it just climbs gently. It rises 9 m over about 10 s, never comes close to a stall, and the slow pitch-and-speed swing (phugoid) damps out on its own.

**The real limit is turbulence, not the release:**
- **Light turbulence** (inside the mission's wind envelope): it holds within a few metres and stays well away from a stall.
- **Moderate turbulence**, a 15 m/s wind near the ground: the wing stalls briefly in about half the runs at 15 m/s. Flying the drop run at 18 m/s removes that.
- **Long mountain downdrafts** can push it into the ground from 40 m, because they sink faster than it can climb.

**That gives two operating rules:**
- In moderate turbulence, fly the drop run at 18 m/s.
- Don't descend to 40 m when the aircraft is measuring sustained sink of more than about 2 m/s.

**The CAD closes:**
- **Fit:** every part sits inside the skin with at least 1.5 mm to spare.
- **Overlaps:** no internal parts clash.
- **Balance:** the balance point computed from the CAD lands 1.7 mm from the design target.

## 1. Detailed CAD (build123d)
**Wing:**
- **Three car-portable pieces:** a 0.9 m centre section and two 1.05 m outer panels, joined at the elevon break with 16 mm carbon tube joiners.
- **Shape:** MH45 airfoil, 25° quarter-chord sweep, 3° washout.
- **Control surfaces:** elevons at 75% chord.
- **Winglets:** NACA 0008, 0.25 m tall.

**Centre pod:** the milestone 3 shape, with a bay door cut into its belly.

**Inside:**
- **Wing-root packs:** 2 × 15 Molicel P50B cells.
- **Kit bay:** a container holding the 1.5 kg kit, with the parachute pack on top and a release servo.
- **Electronics:** flight controller, GPS mast, ESC, and two thin-wing elevon servos.
- **Camera:** gimbal ball with its lens.
- **Other parts:** pitot tube, pusher motor and a 13 in propeller, belly skid.
- **Alternative bay load:** the 18-cell battery module, as its own STEP file.

**Checks run on the CAD itself** (`fit_check.csv`):

| Part | Inside its skin | Clearance to the outer surface |
|---|---|---|
| Wing-root battery packs | 100% | 2.0 mm |
| Kit container | 100% | 5.5 mm |
| Elevon servos | 100% | 3.2 mm |
| Spar joiners | 100% | 13.2 mm |
| Flight controller / ESC | 100% | 41 / 31 mm |
| Bay battery module (alternative) | 100% | 23.5 mm |

Overlap check across all 22 internal parts: none, apart from parts bolted together by design (the gimbal ball on its mount, the prop on its spinner).

**Three layout changes the CAD forced:**
1. **The wing packs moved 8 mm forward** (pack centre x = 0.452 m, was 0.46 m). At the old spot, the rearmost cells had only 1.2 mm of skin. I first tried moving the packs inboard, but that made it worse: on a swept wing, inboard puts them further back along the airfoil, where it's thinner.
2. **The flight controller moved back 50 mm** to make up for the packs moving forward.
3. **The kit bay was re-packed.** The parachute now sits on top of the kit instead of in front of it, so the kit's own centre of mass is on the balance point. The container has rounded edges, because the pod's rounded corners left only 0.7 mm at its front-bottom corner.

**Balance from the CAD** (`mass_cg.csv`): masses from the v2 budget, positions from the CAD part centroids.
- Total 7.30 kg. Balance point at x = 0.4203 m, against the 0.4220 m design. That is 1.7 mm forward, 0.4% of the mean chord: slightly more stable, and a few grams of ballast fixes it.
- After the kit drops, the balance point moves 0.5 mm.

**Reloading the STEP file:** 60 solids, 3.02 × 1.40 × 0.41 m. It opens in FreeCAD or any CAD package.

## 2. Renders (Blender Cycles, 128 samples)
| Render | What it shows |
|---|---|
| `v2_hero.png` | Studio 3/4 view, search-and-rescue colours (white, orange winglets) |
| `v2_inflight.png` | Banking over a sky, from below: gimbal ball, orange bay door |
| `v2_cutaway.png` | See-through skins: wing-root cells (blue), spar joiners, kit bay, flight controller, gimbal |
| `v2_exploded.png` | The three-piece wing pulled apart, internals lifted out |
| `v2_kitdrop.png` | Bay door open, kit falling under its 1.5 m cross parachute |

## 3. 6-DOF kit release (JSBSim)
**The model:**
- **Aero:** AVL (with the pod) gives every stability and control derivative at the release point: 15 m/s, 2,000 m, kit on, balance point x = 0.422 m. That is the aerodynamic model JSBSim flies.
- **Kit:** a JSBSim point mass whose weight drops to zero at release.
- **Bay door:** a drag force (CdA 0.012 m², my assumption) at the door position, from 1 s before to 2 s after the release.
- **Servos:** 20/s lag, 6 rad/s rate limit, ±20° travel.
- **Autopilot (50 Hz):** holds altitude through pitch, airspeed through thrust, and keeps the wings level. It's a basic ArduPilot-class setup, with airspeed gain scaling.

**Check against AVL** (`model_check.csv`), the same aircraft flown open-loop in JSBSim:

| Mode | AVL period | JSBSim period | Difference |
|---|---|---|---|
| Phugoid | 8.61 s | 8.82 s | +2.4% |
| Dutch roll | 2.20 s | 2.05 s | −6.8% |

The JSBSim model flies like the AVL aircraft that passed every handling criterion in milestones 2-4.

**Release at 40 m above the ground, calm air** (`release_results.csv`):

| Case | Max climb | Max sink | Pitch change | Peak CL (max 0.80) | Min airspeed |
|---|---|---|---|---|---|
| Autopilot, 15 m/s | 0.7 m | 0.1 m | 2.1° | 0.51 | 14.6 m/s |
| Autopilot, 13 m/s (slower, more accurate drop) | 1.0 m | 0.1 m | 3.0° | 0.68 | 12.7 m/s |
| Autopilot, double bay-door drag | 0.7 m | 0.1 m | 2.2° | 0.52 | 14.4 m/s |
| Controls frozen at release | 8.9 m | 0 m | 8.1° | 0.61 | 11.2 m/s |

With the controls frozen, the minimum airspeed of 11.2 m/s is still above the kit-off stall speed of about 10.4 m/s.

**Release in turbulence: 100 runs per case** (`turbulence_summary.csv`, `release_turbulence.png`).
- **Turbulence model:** MIL-F-8785C Dryden, low-altitude form. Vertical gusts are about 0.1 × the wind at 20 ft, with a scale length equal to the height.
- **Light:** 15 kt wind at 20 ft. That is inside the mission's 0-10 m/s winds.
- **Moderate:** 30 kt (15 m/s), as fast as the aircraft flies. That is outside its envelope.
- **Stall flag:** lift above CLmax for more than 0.25 s. Shorter gust spikes don't stall a real wing, because of dynamic-stall delay.

| Case | Ground hits | Stalls | Lowest height (clean runs) | 95th pct height loss | 95th pct pitch change | 95th pct load |
|---|---|---|---|---|---|---|
| Light turbulence, 15 m/s | 0 | 1 (before the release) | 36.0 m | 3.4 m | 10° | 1.6 g |
| Moderate turbulence, 15 m/s | 0 | **57** (47 before the release) | 31.9 m | 7.7 m | 17° | 2.2 g |
| **Moderate turbulence, drop flown at 18 m/s** | 0 | **1** (before the release) | 31.7 m | 5.7 m | 14° | 2.4 g |
| Stress test: long downdrafts, 15 m/s | **10** | 13 | 7.9 m | 12.5 m | 18° | 1.6 g |

How to read it:
- **Most stalls aren't caused by the release.** 47 of the 57 moderate-turbulence stalls began before the kit dropped. At 15 m/s and CL 0.5, a 3σ gust (about 4.5 m/s) is enough to stall the wing. Flying 3 m/s faster lowers CL to 0.35, which takes the stalls from 57 to 1.
- **The stress test** is a stand-in for mountain sink. JSBSim's high-altitude gust model applied at 40 m gives downdrafts of 6 m/s that last more than 10 s. That is about twice what Rescue Wing can out-climb at full power, so 10 of 100 runs hit the ground. A 7 kg aircraft can't fix this; it's an operating rule. Measure sink before descending, and don't drop in it.

## What this means
1. **The release is not a risk.** The design rule from milestone 2 (kit bay on the balance point) does what it promised. The CAD confirms the kit sits within 0.5 mm of it, and JSBSim shows the release is barely noticeable.
2. **Turbulence near the ground sets the operating limits, not the kit.** For a slow, lightly loaded wing, a 1.5 m/s vertical gust at 15 m/s is a 6° jump in angle of attack. Two rules:
   - In moderate turbulence, fly the drop run at 18 m/s instead of 15. It costs a little accuracy, which the release-point calculation absorbs.
   - Don't start the 40 m descent when the aircraft measures sustained sink of more than about 2 m/s. In mountain downdrafts, release from higher or wait.
3. **Two things I found and fixed along the way:**
   - **Autopilot gains:** they need airspeed scaling, as ArduPilot does. Without it, gusts that push the speed past 20 m/s drove the pitch loop into an oscillation.
   - **JSBSim turbulence:** JSBSim picks its turbulence model by height above sea level, not above the ground. Over 2,000 m terrain, it uses the high-altitude model at 40 m above the ground. Anyone simulating low flights over high terrain in JSBSim should know this.

## Caveats
1. **The aero model is linear around 15 m/s.** It can't stall, so I flag a stall whenever lift stays above CLmax (0.80) for more than 0.25 s. What happens after a real stall isn't modelled.
2. **The bay-door drag is an assumption.** Doubling it barely changes the result.
3. **The autopilot is a simple one I tuned.** ArduPilot's TECS height-and-speed controller would hold height better; this is a conservative stand-in.
4. **The CAD is a layout model, not a structural design.** Skins, ribs and spar sizing come later. Clearances assume 1.5 mm of skin and foam.
5. **One mass discrepancy.** The JSBSim and AVL runs use the milestone 2 mass model, 7.42 kg with the kit. The CAD budget is 7.30 kg. The 1.6% difference is too small to change any conclusion.
