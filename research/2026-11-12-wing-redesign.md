# Lodestar removable wing joint

2026-11-12, US Central.

## Design decision

Use the **20 × 16 mm carbon tube, 24 mm sleeves, and XPS core** as the next prototype baseline. The compact tube saves 109 g relative to the 24 × 18 mm tube with the same core. It cuts nominal peak tube bending stress from **303 to 149 MPa** in the revised load envelope and from **563 to 277 MPa** in the retained sensitivity cases, relative to the original tube. The stress comparison uses the same assumed 500 MPa flexural allowable for all candidates. The 24 mm tube offers more nominal bending margin at a weight penalty. Sleeve and attachment loads are similar because the wing loads and engagement lengths are similar.

The baseline wing comes to **2.29 kg** with sleeves and reinforcement allowance, about **0.20 kg** heavier than the like-for-like legacy wing and **1.01 kg** above the original budget. The loaded CAD-derived mass is **8.37 kg with P50B** and **8.40 kg with SiCore**. The level-trim static margin is about **4.6% of mean aerodynamic chord** loaded and **4.4–4.5%** after release. The clamped-wing bending–torsion screen gives 106 m/s TAS at sea level and 119 m/s TAS at 4,400 m. Its high result makes the unresolved joint compliance and free-flight modes more important than small differences between these numbers.

| Wing candidate | Mass with joints (kg) | Peak tube stress (MPa) | Peak tube stress in sensitivities (MPa) |
|---|---:|---:|---:|
| Original 16 mm tube, XPS | 2.092 | 303 | 563 |
| 20 mm tube, XPS | 2.291 | 149 | 277 |
| 20 mm tube, mixed core | 2.149 | 154 | 278 |
| 24 mm tube, XPS | 2.401 | 75 | 139 |
| 24 mm tube, mixed core | 2.259 | 77 | 139 |

For the compact XPS case, the joint-cut load table supplies paired bending, shear and torsion from each load case. The governing envelope estimates peak sleeve line demand of about 20 N/mm and an aft-key reaction of about 66 N. Use paired loads from the table for a representative joint proof fixture, then measure rotation and free play as well as strength. Apply the retained full-command sensitivity in a separate fixture case before reducing the joiner further.

| Compact XPS joint fixture case | Ultimate shear (N) | Ultimate bending (N·m) | Ultimate torque about spar (N·m) |
|---|---:|---:|---:|
| Positive dive gust | +148.8 | +69.3 | +5.3 |
| Negative dive gust | −89.1 | −43.1 | −5.0 |
| Peak torque in roll | +46.1 | +20.7 | +13.9 |
| Fixed aileron without angular relief, sensitivity | +229.9 | +128.5 | −13.9 |

At the existing 2,000 m search speeds with 10% battery reserve, the compact XPS model gives **3.07 h for SiCore at 20 m/s** and **2.67 h for P50B at 18 m/s**. The separate sizing power model includes its own miscellaneous mass fraction. Its loaded weights are approximately 50–55 g above the corresponding CAD masses. The predicted loaded stall is about 12.7 m/s true at 2,000 m and 11.5 m/s equivalent. Clarify the convention and altitude for the 12 m/s requirement before comparing compliance.

## Restoring the 6% static margin

*Added 2026-11-12 late evening.* Moving each wing battery pack **25 mm forward and 25 mm inboard** brings the level-trim static margin back to **6.27–6.46% of mean aerodynamic chord** in all four mass states (P50B or SiCore, kit on or released), from 4.43–4.66%. Forward alone needs 25 mm, which puts the outboard front cells into the joint sleeve. The inboard shift moves them clear. At the chosen position the minimum cell-to-sleeve gap is 7.3 mm. The minimum cell-to-skin gap after the 1.5 mm skin allowance is 2.3 mm, versus 0.7 mm today, because the airfoil is thicker further forward. The pack's inner edge moves to 224 mm from the centreline, well outboard of the 80 mm pod half-width. Level-trim elevon deflection falls by about 0.3°. The packs stay inboard of the joint, so the joint loads are unchanged. Root bending relief from the pack mass falls by roughly 1–2%.

The clearance check (`scripts/battery_trim.py`) treats cells and sleeves as cylinders and the skin as the twisted airfoil. That is because the CAD kernel was blocked by Windows Application Control during this run. On the unmoved layout it reproduces the CAD fit check within 1 mm for the sleeve gap (19.4 vs 20.4 mm, conservative) and 0.2 mm for the skin gap. Rebuild the CAD at the new pack position and rerun `wing_trade.py` before treating these numbers as final. Output: `out/wing_trade/battery_trim.csv`.

## Stall requirement: convention and compliance

*Added 2026-11-12 late evening.* The 12 m/s limit in PLAN.md comes from milestone 1. That milestone sized the wing for a 12 m/s stall "in thin mountain air" at 2,000 m, so it is a **true airspeed at 2,000 m**. True airspeed is also the quantity that matters for a hand throw and a belly landing. On that reading the compact XPS aircraft **misses the requirement**: 12.7 m/s loaded, about 6% over. Stall speed scales with the square root of mass at fixed maximum lift coefficient (0.80 in sizing):

| Case | Mass (kg) | Stall, TAS at 2,000 m (m/s) |
|---|---:|---:|
| Loaded, compact XPS | 8.40 | 12.70 |
| Kit released, which is the usual landing state | 6.78 | 11.41 |
| Loaded, illustrative hollowed-XPS core (−0.35 kg) | 8.05 | 12.43 |
| Loaded, full EPS core (−0.61 kg, wrinkling untested) | 7.79 | 12.23 |
| Loaded mass needed for 12.0 m/s | 7.50 | 12.00 |

Landing after a drop meets the limit with margin. The miss is at launch, and only for hand launch. A bungee launch reaches 13–15 m/s routinely, so the miss is not a problem there. No core option on the table recovers 12.0 m/s alone. The choices are a bungee-only launch rule, or a requirement of about 12.5 m/s loaded.

## Search mission effect

### Departure and drop energy

2026-11-12, US Central.

The terrain model now includes departure time and climb energy, accounts for wind when converting cruise and avionics power to energy per metre, and integrates energy to the point of detection along the sampled altitude profile. The previous model allocated energy in proportion to distance along the whole leg, which could miss an early climb. The drop model now reserves a terrain-aware return from the assumed 40 m release height using the lighter aircraft mass. It screens bare-earth clearance at the detection position and uses the 7 m/s chute's nominal fall time.

The departure estimate assumes a still-air climb at the search gradient and accounts for transit time if the pattern starts away from home. The release position is a proxy for the actual upwind approach. Obstacles, flight path during the approach and turns, and available climb power still need separate checks. The synthetic suite exercises ridge clearance, the initial climb, energy partway through a leg, and a visible target whose release proxy fails the terrain check.

The replay used the same 500 scenarios at each of seven sites across six wing mass cases and two batteries. Compact XPS SiCore went from **10.88% to 10.08%** help within one hour and **19.05% to 18.69%** within three hours. P50B went from **10.62% to 9.91%** and **17.17% to 16.97%**. Compact mixed core adds about 0.17 points to SiCore at three hours. The earlier DeltaQuad comparison predates the departure and energy changes and needs a matching fleet rerun. Current outputs are in `out/wing_trade/terrain_summary.csv`; immediate prior outputs have the `terrain_pre_energy_` prefix.

### Earlier comparisons

The original wing-mass comparison below used the earlier camera and return-energy models. The revised seven-site fleet replay includes a bounded camera footprint, bare-earth line of sight, and terrain-aware return energy. At the original mass budget, with the same trails-6 route, SiCore scores **10.88% at one hour and 19.76% at three hours** against the DeltaQuad Evo GOV's **8.94% and 19.83%**. The SiCore minus DeltaQuad difference is +1.94 points at one hour and −0.07 points at three hours. Approximate 95% Monte Carlo intervals are +1.23 to +2.65 points and −0.98 to +0.85 points, respectively. These use the 500 paired run IDs, averaged across the seven fixed sites before estimating the standard error because the sites reuse the seed. They quantify sampling variation within the simulation. In the same original-mass SiCore scenarios, the kit adds 1.28 points at one hour and 0.16 points at three hours over sending a team from the same detections. Figures are from `out/terrain/terrain_runs.csv.gz`.

The updated wing-mass replay preserves those original-mass scores. The compact XPS wing gives **19.05% for SiCore** and **17.17% for P50B** at three hours, down 0.71 and 1.08 points respectively from the corrected original-mass runs. One-hour scores hold at 10.88% and 10.62%. At three hours the compact mixed core adds about 0.06 points on SiCore, so the modeled benefit remains small relative to the unresolved foam and bond behavior. See `out/wing_trade/terrain_summary.csv` and `out/wing_trade/terrain_by_site.csv`. The previous outputs are saved with the `prev_` prefix.

In the paired terrain replay, SiCore help within three hours fell from **23.53% to 22.59%** (−0.94 percentage points) with the compact XPS wing. P50B fell from **21.56% to 20.37%** (−1.20 points). SiCore's one-hour score remained **12.95%**. Against the legacy wing with its structure and tube actually counted, the compact XPS penalty was only 0.11 points on SiCore and 0.17 on P50B. The mixed core gained 0.11 points on SiCore compared with compact XPS. That small modeled gain provides little reason to accept the unmeasured skin support and cure risks. These comparisons use the same 3,500 simulated hikers and camera/drop assumptions. They isolate wing mass and its effect on power and climb energy. The paired baseline uses each aircraft's own mass for climb energy. The earlier terrain run used SiCore's climb coefficient for both batteries, which changes the P50B baseline by one success at the Olympic site.

## Load cases

The original 16 × 13 mm carbon tube was screened at the estimated 7.40 kg takeoff mass with the wing held stationary. The new trade includes each joint's mass, sizes the spar against the revised loads, and repeats the loads with the resulting mass, spanwise inertia, and longitudinal balance. It covers P50B and SiCore packs, kit carried and kit released, and sea level and 4,400 m. The 22 and 30 m/s cruise and dive cases are treated as equivalent airspeeds. Thus the 30 m/s dive case is 37.5 m/s true at 4,400 m in this model.

At 2.53 g and dive dynamic pressure, AVL holds a 15° differential elevon command and trims pitch at five nondimensional roll rates from −0.10 to +0.10, including reversal. The sectional inertia force includes angular acceleration from AVL's net body-axis rolling moment divided by the distributed CAD mass model's roll inertia. Integrated strip lift independently verifies the AVL roll sign. A stationary full command without angular relief and the uncapped gusts remain sensitivity cases. The roll calculation uses quasi-steady aerodynamics, rigid aircraft inertia, approximate component positions, and assumed control deflection and rates. It omits the actuator, combined pitch/roll travel stops, transient yaw coupling and nonlinear section stall. [MIT AVL primer](https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf)

## Joint concept

The main carbon tube carries transverse shear and bending through independent sleeves in the centre and outboard panels. A separate aft key at the rear web carries torque, and positive retention must keep the panels engaged. The sleeve envelope occupies the original foam spar web. The sleeve therefore needs local supports and cap/web load transfer on both sides of each break. The trade budgets 45 g per break for those supports, wraps, aft key, retention and adhesive, and credits zero mass removal where the sleeve displaces foam. This is a provisional mass allowance. Tube bending assumes 500 MPa; sleeve line load and aft-key force are design and fixture inputs. Bearing, ovalization, sleeve splitting, load transfer, pull-out and fatigue require their own checks. The STEP files show nominal cylinders at their aircraft coordinates. Sleeve fit, tolerances and the rest of the joint remain to be detailed. [FAA composite structure guidance](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC20-107B.pdf)

The 24 × 18 mm tube and 28 mm sleeve leave at least 8.89 mm to the aerodynamic skin and 18.41 mm to a bare CAD battery cell at 160 mm engagement per panel. After the 1.5 mm skin allowance, minimum skin clearance is 7.39 mm. The 20 × 16 mm tube and 24 mm sleeve add about 2 mm of clearance at the same engagement. The packaging check uses the existing ruled lofts and cell solids. Battery holders, wiring, sleeve wraps and the aft key still need full assembly clearance checks. Both STEP files were reimported and their three solid volumes matched the specified cylinder dimensions.

## Outer core

The mixed core uses nominal 16 kg/m³ EPS outboard of 0.75 m, with XPS retained around the servo and winglet attachments and through the joint region. The modeled spar web and skin schedule stay the same. This reduces outboard mass and changes roll inertia, torsional stiffness and CG. Low-density foam can compromise skin support and crush or distort during vacuum cure. The current beam and section model screens global response and spar/skin shear. It cannot establish local skin wrinkling, foam shear and skin-core bond allowables. Before adopting EPS, weigh and test the actual bonded skin/core stack and one representative transition with production cure pressure. [EPS product data](https://www.thefoamfactory.com/tech/PolystyreneTech.html), [NASA sandwich wrinkling guidance](https://ntrs.nasa.gov/citations/19990017863), [WEST SYSTEM vacuum bagging](https://www.westsystem.com/instruction/epoxy-basics/vacuum-bagging/)

At the first EPS station, the modeled peak spar cap strain implies roughly 11 MPa of longitudinal skin compression at the same height. A preliminary wrinkling estimate of about 11 MPa using the published sandwich scaling law, an illustrative coefficient of 0.3 and the assumed EPS moduli leaves little room for uncertainty. The skin also carries torsion. These are separate peak demands and an uncalibrated wrinkling estimate. They justify retaining XPS for the baseline and testing EPS as a separate weight-reduction candidate. [NASA foam-core analysis](https://ntrs.nasa.gov/citations/20210011234)

## Reproduce

- `uv run python -X utf8 scripts/wing_trade.py` sizes the five candidates and writes the spanwise loads, joint demands, mass and CG, clamped-wing flutter and search endurance to `out/wing_trade/`.
- `uv run python -X utf8 scripts/wing_trade.py --fit-only` runs the solid clearance checks and exports the two nominal joints.
- `uv run python -X utf8 scripts/wing_mission.py` compares the same 500 hikers at each of the seven terrain sites using the existing trails-first pattern and detection draws. Use the existing terrain data. Each Lodestar variant uses its own mass for climb energy.

The clamped-wing flutter screen includes the joint masses. The beam assumes continuous spar stiffness through the break and holds the P50B pack distribution fixed for flutter comparisons. Joint rotational compliance and free-flight pitch/elevon modes remain open.

The numerical check in `scripts/verify_flutter.py` reproduced the Goland benchmark at 137.23 m/s and 70.09 rad/s. For compact XPS it refined the mesh from 30 to 60 elements, increased the retained modes from six to ten, and extended the reduced-frequency sweep so all ten tracked modes started below the project screen. All ten had negative computed damping parameter `g` through the 36 m/s EAS screen in the model's sign convention. The first crossings moved from 105.74 to 105.68 m/s TAS at sea level and from 119.21 to 119.23 m/s TAS at 4,400 m. See `out/wing_trade/flutter_verification.csv`.
