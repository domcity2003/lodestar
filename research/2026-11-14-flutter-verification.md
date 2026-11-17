# Flutter, divergence and elevon flutter by method

2026-11-14

## Update, 2026-11-15: rerun on the final v1 wing

This rerun uses the final v1 wing:
- stiffer inboard caps and the centre-section skin doubler from the [FE verification](2026-11-15-wing-fea.md);
- packs at x = 0.412 m, y = 0.305 m;
- 8.31 kg loaded;
- FE frequencies of 13.3 Hz first bending and 49.4 Hz first torsion.

**Table U1.** Flutter speed by case [m/s EAS], FE-matched stiffness, minimum of sea level and 4,400 m (`out/flutter_v2/fe_stiffness_sensitivity.csv`, `all_cases.csv`). Requirement: 1.15 VD = 34.5 m/s EAS.

| Case | Flutter, m/s EAS | Margin |
|---|---:|---:|
| Servo engaged (136 N·m/rad), joint 4,170 N·m/rad, 60 g balance | 49.4 | 1.43 |
| Free-floating elevon, 60 g balance at the servo station | 54.2 | 1.57 |
| Free-floating elevon, no balance | 29.2 | **0.85** |
| Servo engaged, joint swept down to 1,000 N·m/rad | 38.1 | 1.10 |
| Free-free symmetric, AVL-matched pitch, kit on | 56.8 | 1.65 |

The conclusions stand. Without the 60 g balance, a free-floating elevon flutters inside the envelope. With it, every case clears. The joint must be at least 1,000 N·m/rad. `freeze_check.py` checks the calibrated rows of this table against 1.15 VD.

## Plain answer

With the servo engaged, the v1 wing clears the flutter screen with a modest margin. The lowest mode is **body-freedom flutter**, where the short period couples with first wing bending. Divergence and reversal are far away.

The **elevon is the red flag.** Wherever it can float, its flutter speed falls inside the flight envelope. That happens inside the free-play band, or if the servo fails or back-drives. The fix is to statically mass-balance the elevon, with about 60 g on a 40 mm forward arm at the servo station. Doing that, keeping free play at or below 0.15° and making servo failure safe clears every case.

The joint must also be confirmed stiffer than about 1,000 N·m/rad, with a target of 2,000 N·m/rad or more.

Screens:

- **1.2 V_D = 36 m/s EAS**, the project screen: 36.0 m/s TAS at sea level, 45.0 m/s TAS at 4,400 m.
- **1.15 V_D = 34.5 m/s EAS**: 34.5 and 43.1 m/s TAS.
- V_D = 30 m/s EAS, which is 37.5 m/s TAS at 4,400 m.

Table 1. Lowest instability speed vs. mode, v1 wing, kit on, stiffness matched to the shell FE modes, with units

| Mode | Sea level, m/s TAS (= EAS) | 4,400 m, m/s TAS | 4,400 m, m/s EAS | Margin vs. 36 / 34.5 m/s EAS |
|---|---:|---:|---:|---:|
| **Body-freedom flutter**, servo 136 N·m/rad, joint at tube-only stiffness 4,170 N·m/rad, AVL-matched pitch (**governing, servo engaged**) | **45.7** | **57.8** | **46.3** | **1.27 / 1.33** |
| same, joint 2,000 N·m/rad | 41.9 | 54.0 | 43.2 | 1.16 / 1.21 |
| same, joint 1,000 N·m/rad | 37.1 | 47.3 | 37.9 | 1.03 / 1.07 |
| same, joint 4,170 N·m/rad, 60 g elevon balance | 46.9 | 60.2 | 48.2 | 1.30 / 1.36 |
| Antisymmetric flutter, servo 136 N·m/rad, joint 4,170 N·m/rad | 94.5 | 108.7 | 87.0 | 2.63 / 2.74 |
| **Free-floating elevon, symmetric** (inside the free-play band, or servo failed) | **29.6** | 29.6 | **23.7** | **0.66 / 0.69** |
| same, AVL-matched pitch | 24.4 | 24.7 | 19.8 | 0.54 / 0.57 |
| Free-floating elevon, antisymmetric | 33.6 | 35.0 | 28.0 | 0.78 / 0.81 |
| Free-floating elevon with **60 g balance at the servo station, 40 mm arm**, lowest of the symmetric, antisymmetric and AVL-pitch cases | 52.1 | 63.3 | 50.7 | 1.41 / 1.47 |
| Divergence (swept, clamped) | none | none | none | – |
| Aileron reversal (clamped, beam stiffness) | 93.1 | 116.3 | 93.1 | 2.59 / 2.70 |

**Why the stiffness is "FE-matched".** The shell/solid FE model (`out/fea/modes.csv`, clamped, P50B) gives first torsion at **46.6 Hz**, against 57.3 Hz for the section-table beam. First bending is 12.3 Hz against 12.6 Hz. Table 1 therefore scales the beam's GJ by 0.661 and EI by 0.954, which reproduces the FE frequencies: 12.3, 37.6 and 47.9 Hz against FE 12.3, 36.2 and 46.6 Hz. The beam-stiffness results are in Tables 3–4. They are 5–20% higher for the wing modes and much higher for elevon flutter, which couples with torsion.

**Physical requirements**

- **Elevon hinge stiffness.** Servo + linkage stiffness at the hinge must be **at least 10 N·m/rad**. At 10 N·m/rad the lowest case is 48.1 m/s EAS. At 7 N·m/rad it is 31.5 m/s, and at 5 N·m/rad 26.7 m/s. Use **20 N·m/rad or more** as the design minimum. The sized 136 N·m/rad (servo through a 2:1 linkage) meets it about 7 times over, and above about 14 N·m/rad the elevon stops setting the flutter speed.
- **Free play: no more than 0.15° total at the hinge, as sized, and balance the elevon.**
  - A dead band lets the elevon float for small motions. With the describing function, onset of limit-cycle oscillation (LCO) is the free-floating flutter speed, whatever the size of the band. Unbalanced and FE-matched, that is **24–30 m/s EAS**: inside the envelope, near Vc (22 m/s) and below V_D.
  - The LCO amplitude scales with free play: about 1.2 times the half free play, so **±0.09° for 0.15° total** with 136 N·m/rad. The amplitude is small, but a continuous buzz wears the linkage and the free play grows. AC 23.629-1A's tab rule (1–2% of the chord at the trailing edge) would allow 0.6–1.1° here. That rule does not cover a flutter-critical primary surface.
  - With the 60 g balance, the free-floating flutter speed is at least 50.7 m/s EAS, so 0.15° of free play causes no LCO below the screen.
  - Count the servo's electronic dead band as part of the free play.
- **Mass balance: needed.** About **60 g per elevon**, concentrated at the servo station (y ≈ 0.87 m) on a **40 mm** arm ahead of the hinge: a horn or balance arm inside the wing cove. That is 2.4 g·m, essentially full static balance of the 2.23 g·m elevon static moment. The added mass is 120 g per aircraft.
  - 40 g gives 42.7 m/s EAS at worst, a margin of 1.19.
  - 80 g over-balances: a low-frequency symmetric instability appears at 35.9 m/s.
  - Balance spread along the elevon nose on a 15 mm arm, or placed at the tip, does not work. 40 g spread along the nose gives 28.2 m/s EAS at 4,400 m.
  - Also make servo failure safe: hold position on signal loss, with a metal-gear servo that does not back-drive. With a floating elevon the aircraft also has a stick-free pitch instability (a positive real root at all speeds).
- **Joint bending rotational stiffness at least 1,000 N·m/rad** to clear 36 m/s EAS (37.1 m/s at sea level). Target **at least 2,000 N·m/rad measured**. The tube-only estimate of 4,170 N·m/rad omits sleeve clearance and bond compliance. Joint torsion springs of 2,000 and 500 N·m/rad, added to the tube bending spring, lower body-freedom flutter from 79.1 to 71.7 and 60.2 m/s (beam stiffness, strip pitch, sea level). Keep the key's rotational stiffness at 2,000 N·m/rad or more.

**Ground vibration test (GVT) that would confirm this.**

Setup: hang the aircraft from soft bungees. Per AC 23.629-1A, the rigid-body bounce on the suspension must be below half the first elastic frequency, so below about 8 Hz. Excite with a small shaker or an instrumented hammer, with accelerometers at the tips, the joints, the pod and both elevon trailing edges.

Measure:

- Free-free symmetric first bending, predicted at **17.9 Hz** (FE-matched; 18.3 Hz on beam stiffness).
- Antisymmetric first bending, predicted at **25.1 Hz** (26.0 Hz on beam stiffness).
- First torsion near **45–48 Hz**.
- Elevon rotation with the servo powered (about 90–105 Hz) and unpowered.

Add static tests:

- Moment–rotation across each joint (bending and torsion).
- Load–deflection at each elevon trailing edge, giving hinge stiffness and free play, with the servo powered.
- A bifilar or knife-edge swing of each elevon, giving mass, CG and inertia about the hinge, before and after balancing.

If first bending or torsion comes out more than 10% below prediction, or the joint is below 2,000 N·m/rad, rerun `scripts/flutter_v2.py` with the measured values before flight.

## Method

Two independent methods now agree where they overlap.

| | Method 1 (structures.flutter) | Method 2 (scripts/flutter_v2.py) |
|---|---|---|
| Solver | V-g (k-method) sweep | p-k (Hassig) with a full root scan in frequency at each airspeed, so aerodynamic roots such as the short period are kept |
| Beam | straight, projected span | along the swept elastic axis (24.4°); strip pitch α = φ cos Λ − w′ sin Λ (bending–torsion coupling from sweep), normal-section GJ = GJ_streamwise · cos Λ |
| Root | clamped | clamped, or free-free half model with rigid heave and pitch (symmetric) or roll (antisymmetric) of the body items |
| Aerodynamics | Theodorsen strips, 2π | Theodorsen strips with flap terms (NACA Report 496), circulatory part scaled by AVL's section lift slope (4.3 root, 4.9 mid-span, falling at the tip, vs. 2π cos Λ = 5.7) |
| Joint | continuous spar | duplicated node with bending and torsion springs at y = 0.45 m |
| Elevon | locked | hinge-rotation field (75–100% chord, y = 0.45–1.5 m) with elevon torsion stiffness, servo spring at y = 0.87 m, inertia coupling and optional balance mass |

**Pitch correction.** The strip aerodynamics leave out the pod and winglets, and the 3D downwash near the root. The strip model's rigid short period at 20 m/s is therefore 2.92 Hz (ζ 0.68), against **2.09 Hz (ζ 0.65) from AVL** for the same mass distribution (Iyy 0.242 vs. 0.237 kg·m²). A quasi-steady correction of the rigid-body pitch moment, ΔCmα = +0.203 and ΔCmq = +1.34 on S and MAC, matches AVL exactly (`short_period_check.csv`). The correction lowers body-freedom flutter by about 30%. The AVL-matched cases govern because AVL models the whole aircraft.

**Mass.** 7.96 kg with P50B packs and kit on, CG x = 0.4135 m; 6.33 kg with the kit released. The v1 section table and CAD v4 positions are used, with packs at y = 0.259–0.401 m and x = 0.427 m.

**Elevon per side**, from the EPS 20 core, one-ply ±45 glass skins and hinge tape:

- mass 60.6 g
- static moment 2.23 g·m aft of the hinge (CG 36.7 mm aft)
- inertia about the hinge 142 kg·mm²
- torsion GJ about 4.4 N·m² at y = 1 m

## Verification

Table 2. Flutter speed and frequency vs. reference, by test case, with units

| Case | Method 2 | Reference |
|---|---:|---:|
| Typical section HP-1 (μ 20, a −0.2, x_α 0.1, r² 0.24, σ 0.4), U_F/(bω_θ) | 2.184 | 2.165 ([AeroBeams/Hodges & Pierce](https://luizpancini.github.io/AeroBeams.jl/dev/literate/typicalSectionFlutterAndDivergence); the reference uses Peters' finite-state aerodynamics) |
| same, ω_F/ω_θ | 0.649 | 0.6545 |
| same, divergence U_D/(bω_θ) | 2.8284 | r√(μ/(1+2a)) = 2.8284 |
| Goland wing, sea level | 137.26 m/s, 70.09 rad/s | 137.1 m/s, 69.1 rad/s ([comparison](https://www.mdpi.com/2076-3417/16/11/5687)) |
| Lodestar v1, straight clamped beam, 2π (method 1's assumptions) | 108.25 m/s, 36.0 Hz | 109.21 m/s, 36.1 Hz (V-g) |
| Lodestar compact_eps20_light, same | 105.64 m/s | 106.52 m/s (V-g) |
| Retained modes 10 / 16 / 24, free-free symmetric | 83.10 / 83.07 / 83.06 m/s | – |
| 30 vs. 60 elements | 83.07 / 83.25 m/s | – |
| Clamped modes, beam / FE-matched beam / shell FE | 12.6, 40.5, 57.3 / 12.3, 37.6, 47.9 / 12.3, 36.2, 46.6 Hz | `out/fea/modes.csv` |

`scripts/test_flutter_v2.py` also checks two more things. The flap lift slope must equal thin-airfoil theory and the apparent-mass matrix must be symmetric. Rigid heave, pitch and roll of the swept free-free model, with joint springs and the elevon, must store no strain energy. All tests pass.

The p-k crossing matches the V-g crossing within 0.9% on the same model. Method 2 integrates the strips at Gauss points and keeps 16 modes. Method 1 lumps the strips at nodes and keeps 6.

## Results by physics added (beam stiffness)

Table 3. First flutter crossing vs. model, v1, m/s

| Model | 0 m TAS | 4,400 m TAS | 4,400 m EAS | Frequency, Hz |
|---|---:|---:|---:|---:|
| Clamped straight, 2π (method 1 assumptions) | 108.3 | 120.2 | 96.2 | 36.0 |
| Clamped straight, AVL lift slope | 134.4 | 149.0 | 119.2 | 31.9 |
| Clamped swept, η = cos Λ | 97.5 | 112.4 | 90.0 | 37.1 |
| Clamped swept, AVL lift slope | 111.9 | 128.7 | 102.9 | 33.1 |
| Clamped swept, FE-matched EI/GJ | 90.0 | 103.1 | 82.5 | 27.6 |
| Free-free symmetric, kit on, strip pitch | 83.1 | 95.6 | 76.5 | 10.7 |
| Free-free symmetric, kit released, strip pitch | 88.3 | 98.6 | 78.9 | 12.9 |
| Free-free antisymmetric, kit on | 127.2 | 139.5 | 111.7 | 30.6 |
| Free-free symmetric, AVL-matched pitch, kit on | 57.2 | 71.7 | 57.4 | 5.2 |
| same, kit released | 62.3 | 77.4 | 61.9 | 6.9 |
| same, joint 4,170 N·m/rad | 50.5 | 64.6 | 51.7 | 3.7 |
| same, joint 1,000 N·m/rad | 39.2 | 49.4 | 39.5 | 1.8 |
| same, joint 500 N·m/rad | 29.6 | 35.7 | 28.6 | 1.1 |
| same, FE-matched EI/GJ, rigid joint | 52.6 | 65.2 | 52.2 | 5.5 |

Elevon locked unless stated. Rigid joint unless stated. P50B packs.

What each step shows:

- **Sweep.** Bending of the aft-swept wing washes the tip out. With the AVL lift slope, the swept clamped wing flutters at about the same speed as the unswept 2π model, at 112 vs. 108 m/s.
- **Free-free.** Body-freedom flutter appears. The short-period root rises with speed and merges with first bending (Figure 1). This is the classic flying-wing mode, as on the X-56 ([NASA AePW-3 flight-test group](https://nescacademy.nasa.gov/workshops/AePW3/public/wg/flighttest); [NASA X-56 flight test](https://ntrs.nasa.gov/citations/20220012337)). It halves the clamped flutter speed once the rigid pitch dynamics match AVL.
- **Kit release.** Dropping the 1.6 kg kit raises the body-freedom speed by about 5 m/s.
- **Static margin.** The free-free model also shows a real-root static instability at 70–125 m/s. Wing bending reduces the effective static margin of the swept wing. It comes down to V_D only for a joint near 500 N·m/rad.

The earlier wing, compact_eps20_light, gives nearly the same results (`all_cases.csv`, group eps20):

| Case, sea level | compact_eps20_light | v1 |
|---|---:|---:|
| Clamped swept | 110.6 m/s | 111.9 m/s |
| Free-free symmetric, strip pitch | 84.4 m/s | 83.1 m/s |
| Free-free antisymmetric | 122.3 m/s | 127.2 m/s |

## Elevon flutter

Table 4. Flutter speed vs. hinge stiffness, by symmetry, altitude and wing stiffness, m/s EAS

| Hinge stiffness, N·m/rad | Sym 0 m (beam / FE) | Sym 4,400 m (beam / FE) | Anti 0 m (beam / FE) | Anti 4,400 m (beam / FE) |
|---:|---:|---:|---:|---:|
| 0 (free) | 53.5 / **29.6** | 48.1 / **23.7** | 43.4 / 33.6 | 36.4 / **28.0** |
| 2 | 51.7 / 30.5 | 44.9 / 23.8 | 44.7 / 35.0 | 36.0 / 27.5 |
| 5 | 51.8 / 35.1 | 43.1 / 26.7 | 52.0 / 52.9 | 38.5 / 34.0 |
| 7 | – / 45.5 | – / 31.5 | – / 84.9 | – / 83.4 |
| 10 | 60.3 / 68.7 | 49.1 / 48.1 | 104.9 / 86.4 | 76.4 / 85.3 |
| 14 | – / 69.1 | – / 65.8 | – / 89.2 | – / 86.2 |
| 20 | 79.9 / 69.5 | 76.5 / 66.1 | 109.4 / 91.2 | 106.8 / 86.7 |
| 136 (sized) | 82.2 / 71.3 | 77.4 / 67.1 | 115.6 / 98.7 | 107.7 / 89.3 |

Free-free model, strip pitch aerodynamics, rigid joint. With 136 N·m/rad and above about 14 N·m/rad, the result is body-freedom or antisymmetric wing flutter, not elevon flutter.

Below about 7 N·m/rad the instability is wing-torsion–elevon flutter at 44–56 Hz. The torsion mode couples with the elevon rotation, which lags behind it. The classic remedy is mass balance or a stiff, play-free actuator.

Moving the servo from y = 0.87 m to 1.08 m (60% of the elevon span from the inboard end) changes little at 20 and 136 N·m/rad: 78–79 m/s symmetric and 110–115 m/s antisymmetric at sea level, beam stiffness. At 2–5 N·m/rad it changes the result by up to ±17 m/s.

Table 5. Free-floating elevon flutter speed vs. balance mass at the servo station (40 mm arm), FE-matched stiffness, m/s EAS

| Balance per elevon | Sym 0 m | Anti 0 m | Sym 4,400 m | Anti 4,400 m |
|---:|---:|---:|---:|---:|
| 0 | 29.6 | 33.6 | 23.7 | 28.0 |
| 20 g | 36.1 | 37.4 | 32.3 | 33.5 |
| 40 g | 49.3 | 45.6 | 46.4 | 42.7 |
| 60 g | 59.6 | 52.5 | 57.9 | 50.7 |
| 80 g | 35.9 (low-frequency) | 58.0 | 41.4 (low-frequency) | 55.6 |
| 40 g spread along the elevon nose, 15 mm arm | 30.2 | 32.5 | 25.4 | 28.2 |

With AVL-matched pitch, a free elevon with 60 g balance flutters at 52.1 m/s EAS at sea level and 52.8 m/s EAS at 4,400 m. Beam-stiffness balance results are in Figure 2b.

**Free play.** A dead band of total width 2d on the servo spring is modelled with a describing function (`freeplay.csv`, `deadband_ratio`):

- Inside the band the elevon is free. For larger motion the effective stiffness rises toward 136 N·m/rad.
- LCO can start once the airspeed passes the flutter speed at any stiffness below 136 N·m/rad. Unbalanced and FE-matched, that is the free-floating 24–30 m/s EAS.
- The amplitude is set where the effective stiffness equals the critical stiffness, 5–10 N·m/rad: A ≈ 1.1–1.2 d, so ±0.08–0.09° for 0.15° total.

AC 23.629-1A sets the damping and margin reference: g = 0.03 at 1.2 V_D. If damping falls steeply, g = 0 should be no lower than 1.2 V_D. AC 23.629-1A §12 also notes that strip-theory control-surface coefficients can overpredict flutter speeds compared with wind-tunnel tests. That is a further reason to balance rather than rely on the analysed margin.

Sources: [AC 23.629-1A text](https://rosap.ntl.bts.gov/view/dot/71218); [AC 23.629-1B, current](https://www.faa.gov/regulations_policies/advisory_circulars/index.cfm/go/document.information/documentID/22454); [aileron freeplay LCO wind-tunnel study](https://library.kab.ac.ug/Record/doaj-art-3948c02d95404303bac81dbb1c96cabb/Description); [freeplay review, ICAS 2024](https://www.icas.org/icas_archive/icas2024/data/papers/icas2024_0922_paper.pdf).

## Divergence and reversal

These come from the clamped half wing with steady strips and the AVL lift slope (`static_aeroelastic.csv`, `roll_effectiveness.csv`, beam stiffness).

- **Divergence.** The aft-swept wing has no positive divergence pressure: bending washes the tip out, so the swept wing has no divergence. The same beam without the sweep coupling would diverge at 194 m/s EAS, 6.5 V_D.
- **Reversal.** Aileron reversal is at **93 m/s EAS**: 93.1 m/s TAS at sea level and 116.3 m/s TAS at 4,400 m, 3.1 V_D.
- **Roll effectiveness.** 0.81 at V_D and 0.74 at 36 m/s EAS, against 0.92 for the unswept beam. Sweep washout costs about 11% of roll power at V_D.
- **FE torsion effect.** With FE-matched torsion (GJ × 0.66), reversal moves lower, roughly in proportion to GJ, to about 75 m/s EAS, still above 2 V_D. This estimate was not rerun.

## Limits of this analysis

- **Aerodynamics.** Strip theory has no 3D unsteady wake, winglet or pod unsteady aerodynamics, or spanwise-flow terms. The rigid-body correction is quasi-steady. Gravity and the phugoid are left out.
- **Model.** The model is linear and small-amplitude. The free-play result is a describing-function estimate, not a time simulation.
- **Stiffness and materials.** The FE-matched case scales whole-span GJ and EI uniformly to match the FE frequencies. EPS and glass properties are assumed. The joint stiffness is an estimate that leaves out clearance.
- **Speed grid.** Crossings come from linear interpolation on a 2 m/s grid below 60 m/s and a 5 m/s grid above.

## Figures

- Figure 1 (`out/flutter_v2/fig1_vg_vf_free_free_sym.png`): damping g and frequency vs. true airspeed, by p-k root. Free-free symmetric, strip pitch, beam stiffness, kit on, elevon locked, rigid joint, sea level and 4,400 m. Dashed line: 36 m/s EAS.
- Figure 2 (`out/flutter_v2/fig2_elevon_and_joint.png`):
  - 2a. Flutter speed vs. hinge stiffness.
  - 2b. Free-floating elevon flutter speed vs. balance mass and location, sea level.
  - 2c. Flutter speed vs. joint bending stiffness.

  All three use beam stiffness and strip pitch. The FE-matched and AVL-matched values are in Tables 1, 4 and 5.

## Reproduce

From `scripts/`:

- `../.venv/Scripts/python.exe -X utf8 test_flutter_v2.py` runs the checks, a few minutes.
- `../.venv/Scripts/python.exe -X utf8 flutter_v2.py` runs the study: 193 beam-stiffness cases, then the FE-matched cases when `out/fea/modes.csv` exists, in parallel. It takes about 15 minutes on 16 cores and needs AVL in `../aero-tools/avl/`.
- `flutter_v2.py --fe` reruns only the FE-matched cases (`fe_stiffness_sensitivity.csv`).
- Outputs in `out/flutter_v2/`:
  - `summary.csv`, `all_cases.csv`
  - `joint_sweep.csv`, `elevon_sweep.csv`, `freeplay.csv`
  - `static_aeroelastic.csv`, `short_period_check.csv`
  - `modes_*.csv`, `pk_roots_*.csv`
  - figures and logs
