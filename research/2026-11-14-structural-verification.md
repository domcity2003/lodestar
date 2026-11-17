# Wing structural verification: load cases, finite elements and margins

2026-11-14

## Answer

**No, not yet.** The 1.881 kg v1 wing (EPS 20 core, XPS only at the servo bay and y > 1.40 m) does **not** hold every case with a positive margin. Two problems need design changes, and both are cheap:

1. **Upper skin wrinkling near the root (flight).** The sizing in `structures.size_wing` checks only the spanwise skin stress (utilisation 0.995). The finite-element model shows the same skin also carries 9–12 MPa of torsion shear. The combined principal compression is **19.4 MPa ultimate** at y = 0.06 m, against the 14.4 MPa Hoff allowable for glass on EPS 20, so **MS = −0.26**. The skin is over the allowable from the root to about y = 0.5 m. Using the ±45 face's higher stiffness along the principal direction raises the allowable to 16.4 MPa, which still gives MS = −0.15. Fixes:
   - EPS of about 30 kg/m³ in the centre section, which needs E ≈ 6.3 MPa and G ≈ 3.9 MPa (+0.18 kg);
   - XPS in the centre section, MS +0.43 (+0.23 kg);
   - or a coupon test showing that EPS 20 is stiffer than the low-end values assumed (E·G ≥ 2.5 × 10¹³ Pa²).
2. **Belly landing.** The flight envelope does not cover landing. When the pod stops on hard ground, the packs and outer wing keep moving down. At 1.5 m/s sink with 10 mm of crush, the pulse is 13 ms with an 18 g peak, and the ultimate root bending reaches **−263 N·m**. That is 3.5 times the flight negative envelope of −74.5 N·m. The lower caps and lower skin fail: MS −0.28 for the caps on the web and −0.64 for skin wrinkling. A **30 mm crushable belly** (6 g peak) brings it to −97 N·m, and the lower skin then has MS +0.32. The pod saddle also needs a ply or G10 bearing plate.

Every other element has a positive margin. Three items are flagged as design rules rather than load problems:
- the sleeve ends and sleeve bond must bear on hard points (ply/G10 ribs and glass wraps), not on EPS;
- the winglet needs overlap tapes rather than a foam butt joint;
- the battery bay must keep its foam, or have a screwed structural lid on the lower surface. An unsupported skin over the 107 mm bay buckles at 27 kPa.

Almost every positive margin rests on an assumed allowable, so the test list at the end is the real gate.

**Inputs and sensitivity.** The analysis uses the v1 design files of 2026-11-14 17:16 (`v1_sections.csv`, `v1_loads.csv`, `v1_joint.csv`, `aircraft_mass_balance_v1.csv`, `cad_v4/mass_cg.csv`). Copies are kept in `out/fea/inputs_v1_20261114_1716/`, and the scripts read from there.

A later layout change moves the packs to x = 0.412 m, y = 0.305 m and adds a 60 g elevon balance and a 0.10 kg thermal kit. That change shifts loads by a few percent, and every stress-based margin scales as (1 + MS)/(1 + Δ) − 1. A +3% load change gives:
- upper skin on EPS: −0.26 → −0.28;
- outer-panel skin at y ≈ 0.66 m: +0.03 → 0.00;
- tube ovalization: +0.21 → +0.18 in flight (the landing value moves with pack mass and position, not with the flight loads);
- every other margin stays above +0.3.

The negative margins and the fixes do not depend on that change. The two near-zero items are already on the test list.

The FE model agrees with the beam model where both make the same assumptions:
- bending stiffness EI: +2% (median, y = 0.2–1.2 m);
- equivalent streamwise torsional stiffness GJ: +4%;
- tip deflection: −2.4%;
- spanwise skin stress: −5 to +13%.

The beam model **overstates cap stress by 15–18%** because it ignores the skins' share of bending. It **understates skin shear outboard** because it leaves out the bending shear flow in the skins.

First bending is **12.33 Hz** in the FE model versus 14.84 Hz in the flutter beam. Laying the same beam along the 25° swept spar gives 12.81 Hz, so about four fifths of the gap is the beam's straight projected-span idealisation. The first torsion mode is at **46.6 Hz**, versus 60.1 Hz in the beam. It is pitch of the tip section, which rocks the raised winglet mass fore and aft. Frequencies are in `out/fea/modes.csv`.

## Model

The model covers one half-wing, swept and tapered, with the MH45 section, from the centreline to the tip. Washout is omitted. The model is written as a CalculiX 2.22 deck by `scripts/fea_wing.py`. Medium mesh: 8,970 nodes, 11,280 elements.

- **Skins:** S4 shells over the full chord, one ply of ±45 glass, orthotropic (E = 11 GPa, G₁₂ = 9 GPa, ν = 0.5), 0.19 kg/m² including finish. The material is transversely isotropic about the spar axis, so one orientation serves both skins and webs. The test suite checks that orientation and the local stress output.
- **Spar caps:** the 12 mm cap strip of the skin (13.2 mm streamwise) is a smeared glass + carbon shell with the cap thickness from `v1_sections.csv`. Its nodes sit at the cap centroid, so the lever arm matches the beam model's h − 2t_s − t_cap. The skin stress over the cap is extrapolated to the outer surface by h/h_eff.
- **Webs:** the spar web at 25% chord is a shell, Rohacell 51 block plus two glass plies, 12.16 mm. The rear web at 72% chord is 3 mm XPS plus two plies.
- **Core:** C3D8I/C3D6 solids filling the section, sharing nodes with the skins: EPS 20 (E 4, G 2.5 MPa), XPS (E 12, G 6 MPa). The core is modelled as solids, not as an elastic foundation, for three reasons:
  - it supports the skins against local panel modes, which would otherwise appear near 1 Hz in the frequency run;
  - it adds about 7% of GJ;
  - it gives core stresses directly.
  
  Wrinkling itself (wavelength about 3 mm) is below the mesh scale, so it is checked with Hoff's formula using the FE skin stresses.
- **Boundary conditions:** clamped at the centreline, as in the beam model. The tip section is a rigid winglet-root rib carrying the winglet mass at h/√3 above the tip.
- **Elevon:** continuous with the wing (servo-held), as in the beam section integrals. The joint is continuous (rigid joint, as in the beam).
- **Mass:** FE structural half-wing 0.736 kg versus 0.741 kg in the section table (−0.6%). Packs (P50B 1.21 kg per side, or SiCore 1.23 kg) are spread over every node of their v1 footprint, x = 0.427 m and y = 0.224–0.436 m. Lumping a pack on a few foam nodes gave spurious mass-on-foam modes. Joint masses, servo, winglet and hinge line are point masses.
- **Loads:** each case's limit shear and torque about the spar from `v1_loads.csv`, multiplied by 1.5. In each 25 mm interval a force at the spar line and a force at the rear-web line reproduce V, M_b and T at every station within 0.6% of the case maximum. The forces are spread over the web nodes.

## 1. Load cases

**Table 1.** Limit load factor, speed, mass and governed element, by load-case family (160 cases: P50B and SiCore, kit on and off, sea level and 4,400 m, right and left wing for roll).

| Case | n limit | V EAS (TAS max) [m/s] | Mass [kg] | Root M_b ult [N·m] | Governs |
|---|---:|---:|---:|---:|---|
| Manoeuvre + | 3.80 | 30 (37.5) | 6.33–7.99 | 51–82 | nothing (gusts exceed it) |
| Gust + at Vc (stall-capped) | 5.98–7.55 | 22 (27.5) | 6.33–7.99 | 86–120 | +M_b root to tip, torque, web shear, joint tube and sleeve, tip deflection |
| Gust + at VD | 5.49–6.13 | 30 (37.5) | 6.33–7.99 | 80–118 | web shear at one inboard station |
| Manoeuvre − | −1.50 | 30 (37.5) | 6.33–7.99 | −17 to −30 | nothing |
| Gust − at Vc (stall-capped) | −2.67 to −3.37 | 22 (27.5) | 6.33–7.99 | −43 to −56 | nothing |
| Gust − at VD | −3.49 to −4.13 | 30 (37.5) | 6.33–7.99 | −52 to −75 | −M_b root to break |
| Roll, p̄ = −0.10 to +0.10 incl. reversal, 15° differential | 2.53 | 30 (37.5) | 6.33–7.99 | 31–61 | outboard torque and shear (p̄ −0.10), aft key and break torque (p̄ +0.10) |
| Gust ± at Vc uncapped (sensitivity) | −6.52 to 8.52 | 22 | 6.33–7.99 | −118 to 140 | reported only |
| Fixed aileron, no angular relief (sensitivity) | 2.53 | 30 | 6.33–7.99 | −142 to 234 | joint tube 273 MPa, reported only |

Speeds are equivalent airspeeds, so the 4,400 m cases have the same q and slightly lower capped n (6.00 vs 6.19 for P50B, kit on). They never govern. Kit on governs bending (7.96–7.99 kg at n ≈ 6.2). Kit off governs torque (n = 7.51 at 6.37 kg). The cell choice changes loads by less than 0.1%. Per-case data are in `out/fea/load_cases.csv`; family summaries in `load_case_families.csv`.

**Table 2.** Ultimate loads in the cases outside the flight envelope, by case. FOS 1.5 on all.

| Case | Basis | Result |
|---|---|---|
| Belly landing, sudden stop | Clamped flutter beam (v1 packs), modal superposition, half-sine base deceleration. Sink 1.5 m/s ≈ 0.51(W/S)^¼ = 1.35 m/s ([CS-VLA 473](https://www.easa.europa.eu/sites/default/files/dfu/Easy%20Access%20Rules%20CS-VLA%20%28Amendment%201%29.pdf)), with 2.13 m/s (the CS-VLA floor) as a sensitivity. Crush 10 mm (hard ground, Kevlar skid) or 30 mm (crushable belly). Lift = weight at touchdown. | 10 mm: root −263, y 0.125 −189, break −81 N·m (18 g equivalent on the wing). 30 mm: root −97, −79, −45 N·m. 2.13 m/s, 10 mm: root −459 N·m. Root shear 606 N per side. |
| Belly landing, 10 g quasi-static | For comparison | root −141 N·m |
| Bungee/catapult launch | 10 g limit, longitudinal | in-plane root moment 154 N·m, aft key 192 N, attachment shear 649 N, pack bearing 42 kPa |
| Kit release (1.62 kg) | Step unload, dynamic factor 2 | Δn = +0.51, so n = 1.5 at most. Does not govern. |
| Ground handling | Assembled aircraft lifted at both tips, 1 g | root +73 N·m, inside the flight envelope (120) |
| Winglet side load, max sideslip | Lateral gust at VD gives β = 14.3°. Winglet stalls: NeuralFoil NACA0008 CL_max = 0.89 at Re 4.3 × 10⁵. | side force 39 N, winglet root moment 4.5 N·m, wing tip M_b +4.0 / T +1.9 N·m (outboard of y ≈ 1.2 m this exceeds flight M_b, but minimum caps there have MS > 4) |
| Elevon hinge | Elevon normal-force coefficient 1.0 at VD (upper bound) | 82 N per elevon, 0.074 N/mm hinge line, hinge moment 3.2 N·m |

The landing estimate assumes the pod stops rigidly. A softer pod and skid only lengthen the pulse, which the crush distance already represents. The beam's first bending frequency is 20% above the FE value, and impulsive moment scales with frequency, so the beam numbers are on the high side.

## 2. Finite elements vs the beam model

**Table 3.** FE and beam-model ultimate stresses vs span, by case (medium mesh). Case `pos`: P50B, 0 m, kit on, gust + at Vc. Case `neg`: SiCore, 0 m, kit on, gust − at VD. Case `tor`: SiCore, kit off, gust + at Vc.

| Case | y [m] | Compressed cap FE / beam [MPa] | Skin σ₁₁ comp. FE / beam [MPa] | Skin principal comp. FE [MPa] | Skin shear FE / beam [MPa] | Web q FE / beam [N/mm] |
|---|---:|---:|---:|---:|---:|---:|
| pos | 0.09 | 111 / 131 | 13.3 / 11.8 | 19.0 | 9.3 / 9.2 | 1.6 / 2.3 |
| pos | 0.29 | 120 / 144 | 12.3 / 13.0 | 16.8 | 9.5 / 7.9 | 0.9 / 2.0 |
| pos | 0.59 | 100 / 119 | 10.1 / 10.6 | 12.8 | 7.8 / 5.0 | 1.7 / 2.9 |
| pos | 0.99 | 37 / 45 | 3.6 / 4.0 | 6.9 | 5.0 / 3.0 | 0.8 / 1.5 |
| neg | 0.29 | 72 / 88 | 7.4 / 7.9 | – | 5.7 / 4.9 | 0.6 / 1.2 |
| tor | 0.29 | 98 / 119 | 10.1 / 10.7 | – | 10.9 / 8.5 | 0.5 / 1.3 |

**Table 4.** Stiffness and deflection, FE vs beam model.

| Quantity | FE | Beam (structures.py) | FE / beam |
|---|---:|---:|---:|
| EI from unit tip moment, median y 0.2–1.2 m | – | – | 1.02 (0.96–1.20) |
| Streamwise-equivalent GJ = T/(dθ/dy)/cos Λ, median y 0.2–1.2 m | – | – | 1.04 (0.95–1.06) |
| Tip deflection at limit, case pos [mm] | 29.2 | 29.9 | 0.976 |
| Tip deflection at limit, case neg [mm] | −18.2 | −18.6 | 0.976 |

Why they differ:
- **Cap stress, FE 15–18% lower:** the beam puts the whole moment in the caps (σ = M/A·h_eff), while its EI counts the skins.
- **Web shear, FE lower:** the nose and skins share the vertical shear.
- **Skin shear, FE higher outboard (up to 1.6×):** the FE includes the bending shear flow in the skins as well as torsion.
- **Skin principal compression:** this is the quantity the beam never forms (§3).
- **EI and GJ:** the near-root and tip outliers are clamp and rigid-rib effects. Station values are in `out/fea/stiffness.csv`, `fe_stations.csv` and `fe_vs_beam.csv`.

**Table 5.** Natural frequencies [Hz] by mode, FE vs flutter beam, half wing clamped at the centreline, P50B packs (SiCore within 0.3%). The v1.1 column is the later layout: packs at x 0.412 m, y 0.305 m; 60 g balance per elevon at (0.715, 0.87); +50 g per side thermal kit.

| Mode | FE, v1 | FE, v1.1 | Beam, projected span (flutter screen), v1 | Beam, swept length, v1 | Shape (FE, v1) |
|---|---:|---:|---:|---:|---|
| Bending 1 | 12.33 | 12.06 | 14.84 | 12.81 | tip twist/bend 0.09 |
| Bending 2 | 36.25 | 36.31 | 46.01 | 40.54 | one node |
| Torsion 1 | 46.62 | 46.17 | 60.07 | 53.73 | tip pitch with fore-aft sway of the raised winglet mass |
| Mode 4 | 68.79, torsion 2 | 64.78, bending 3 (coupled) | 88.70 (torsion 2) | 78.52 | twist dominant, one bending node |
| Mode 5 | 69.83, bending 3 (coupled) | 68.28, torsion 2 | 185.9 (bending 3) | 164.1 | w and twist of similar size (twist/bend 0.83) |
| Torsion 3 | 111.3 | 109.3 | 122.5 | 109.4 | |

Modes 4 and 5 are a close pair (1 Hz apart in v1, 3.5 Hz in v1.1). Sweep couples the second torsion with a third bending shape that the straight beam places far higher, near 165–186 Hz. The aft elevon balance swaps their order. The flutter work should use the FE frequencies and the shapes in `out/fea/modes_shapes.csv`, or at least the swept-length beam.

**Table 6.** Mesh convergence, case pos (ultimate) and frequencies.

CONVERGENCE_TABLE

## 3. Margins of safety

MS = allowable / applied − 1, with applied = ultimate (limit × 1.5). Allowables are design values that are already knocked down: for the carbon, about 0.6 × datasheet, as in `structures.py`. No further factor is applied. FE stresses are applied to the whole envelope by scaling the beam envelope by the FE/beam ratio of the governing FE case at each station, taking the larger of that and the direct FE maximum. Rows within 0.06 m of the clamp and the rigid tip rib are excluded.

**Table 7.** Margin of safety by element and failure mode, ultimate. Full data with sources: `out/fea/margins.csv`.

| Element | Failure mode | Governing case | Applied | Allowable | MS | Status |
|---|---|---|---:|---:|---:|---|
| Upper skin on EPS | wrinkling, min principal | gust + at Vc env., y 0.06 m | 19.4 | 14.4 MPa | **−0.26** | NEGATIVE |
| Upper skin on EPS | wrinkling, spanwise stress only (beam criterion) | same | 13.9 | 14.4 MPa | +0.03 | comparison only |
| Spar cap, lower | compression on web, belly landing 10 mm | landing, y 0 | 261 | 187 MPa | **−0.28** | NEGATIVE |
| Lower skin on EPS | wrinkling, belly landing 10 mm | landing, y 0 | 40.3 | 14.4 MPa | **−0.64** | NEGATIVE |
| Lower skin on EPS | wrinkling, belly landing 30 mm crush | landing, y 0.125 | 10.9 | 14.4 MPa | +0.32 | needs test |
| Wing-pod saddle | bearing on EPS, landing | landing | 253 | 70 kPa | **−0.72** | NEGATIVE: needs ply/G10 saddle plate |
| Sleeve end support | bearing into EPS | landing | 1.39 | 0.07 MPa | −0.95 | design rule: hard point |
| Sleeve end support | two 1.5 mm ply/G10 ribs | landing | 9.3 | 25 MPa | +1.69 | OK-assumed |
| Sleeve bond | into EPS only | landing | 0.195 | 0.10 MPa | −0.49 | design rule: glass wrap |
| Sleeve bond | glass wrap onto web/caps | landing | 0.195 | 0.64 MPa | +2.28 | OK-assumed |
| Battery bay skin | no core, no lid: panel buckling | gust + at Vc, y 0.33 | 15.6 | 0.027 MPa | −1.00 | design rule: keep foam or structural lid |
| Battery bay lid | M3 screws at 30 mm, q = T/2A | env. T | 18.9 | 150 N | +6.93 | OK-assumed |
| Winglet attachment | foam butt joint only | side load at VD | 0.32 | 0.20 MPa | −0.37 | design rule: overlap tapes |
| Winglet attachment | 2 × 25 mm glass overlap tapes | side load at VD | 0.056 | 0.64 MPa | +10.3 | OK-assumed |
| Joint tube 20 × 16 | ovalization at sleeve mouth | landing 10 mm (flight: 124 MPa, MS +0.21) | 144 | 150 MPa | +0.04 | needs test |
| Lower skin on EPS | wrinkling, flight | gust − at VD, y 0.06 | 10.7 | 14.4 MPa | +0.34 | needs test |
| Spar cap, upper | wrinkling on Rohacell web | gust + at Vc, y 0.14 | 131 | 187 MPa | +0.42 | OK-assumed |
| Bay walls | pack inertia on EPS, launch | launch | 42 | 70 kPa | +0.65 | OK-assumed |
| Winglet root skin | wrinkling on XPS | side load | 17.9 | 27.8 MPa | +0.55 | OK-assumed |
| Servo bay cut-out | wrinkling at edge, Kt 3 | gust − at VD | 16.6 | 27.8 MPa | +0.67 | needs transition coupon |
| Ground handling | lifted at both tips | 1 g × 1.5 | 72.8 | 120 N·m (flight) | +0.65 | covered by flight |
| Joint tube | bending, fixed-aileron sensitivity | roll sensitivity | 273 | 500 MPa | +0.83 | needs test |
| Spar cap, lower | compression on web, flight | gust − at VD | 79.8 | 187 MPa | +1.34 | OK-assumed |
| Joint tube | bending, belly landing | landing | 175 | 500 MPa | +1.86 | needs test |
| Aft-skin region | wrinkling, in-plane launch bending | launch | 6.85 | 14.4 MPa | +1.10 | needs test |
| Upper skin on XPS | wrinkling | gust + at Vc, y 0.84 | 9.3 | 27.8 MPa | +1.98 | OK-assumed |
| Attachment bolts | 2 × M5 nylon, tension, pod hangs, n 7.55 | gust + at Vc | 197 | 600 N | +2.04 | OK-assumed |
| Joint tube | bending, flight | gust + at Vc | 147 | 500 MPa | +2.40 | needs test |
| Spar web | shear | env. V, y 0.46 | 3.2 | 11.1 N/mm | +2.47 | OK-assumed |
| Aft key | 4 mm pin shear / 2 mm G10 bearing | launch, in-plane | 15.3 / 24 | 60 / 150 MPa | +2.92 / +5.24 | OK-assumed |
| Skins | shear | gust + at Vc, y 0.44 | 12.6 | 60 MPa | +3.75 | OK-assumed |
| Spar cap, upper | fibre compression | gust + at Vc | 131 | 700 MPa | +4.33 | OK-assumed |
| Attachment dowels | 2 × 6 mm, launch shear | launch | 324 | 1,696 N | +4.23 | OK-assumed |
| EPS core | shear | FE, y 0.34 | 16.6 | 100 kPa | +5.03 | needs test |
| Rear web | shear strain | FE torsion | 0.00086 | 0.0067 | +6.75 | OK-assumed |
| Spar caps | tension | gust + at Vc | 131 | 1,000 MPa | +6.66 | OK-assumed |
| EPS core | compression | FE | 5.1 | 70 kPa | +12.8 | OK-assumed |
| Elevon hinge | line load | C_N 1.0 at VD | 0.074 | 2 N/mm | +26 | needs test |
| Sleeve | bearing | landing | 1.7 | 200 MPa | +118 | OK-assumed |
| Deflection | tip at limit load | gust + at Vc | 30 mm | 75 mm (5% semispan, proposed criterion) | +1.51 | OK |

**Minimum margin by location:**
- centre section, y 0.06–0.5 m: upper skin wrinkling in flight, −0.26, and lower cap/skin in a hard belly landing, −0.64;
- joint at y = 0.45 m: tube ovalization, +0.04 (landing) and +0.21 (flight);
- outer panel, y 0.5–1.4 m: upper skin wrinkling, +0.03 at y ≈ 0.66 m on the principal stress (14.0 MPa);
- servo bay: +0.67;
- winglet root: +0.55, provided overlap tapes are used.

Structural concepts assumed for these checks are not yet designed:
- wing-pod attachment: 2 × M5 nylon bolts, 2 × 6 mm carbon dowels and a saddle;
- aft key: 4 mm carbon pin in a 2 mm G10 plate;
- sleeve-end ply ribs;
- a lower-surface battery lid.

## Joint, local

Paired M, V and T come from `v1_joint.csv`. A rigid tube with end contacts gives the mouth reaction R = V + M/L (L = 160 mm): 577 N in flight, 734 N in landing, 1,018 N in the fixed-aileron sensitivity.

- **Ring bending:** the conservative check uses a diametral line load over a 20 mm contact length ([Roark's formulas](https://www.mheducation.com/highered/product/roark-s-formulas-stress-strain-young-budynas/M9780071742474.html), ring case). The result needs a **roll-wrapped tube with hoop plies**. A pultruded UD tube has roughly 40–60 MPa of transverse strength, which would fail.
- **Sleeve bearing:** trivial.
- **Sleeve support:** the load must reach the caps through ribs and wraps that are already in the 45 g allowance. Epoxy into EPS fails, at 1.2–1.4 MPa against 0.07.
- **Aft key:** the 10 g launch governs, because the in-plane moment at the break is taken as a tube–key couple (192 N). The flight torque gives 68 N. Results are in `out/fea/joint_checks.csv`.

## Verification

`scripts/test_fea_wing.py`, 5 of 5 passing:
- cantilever box of S4 shells against Euler–Bernoulli: tip deflection +0.44%, flange stress −0.00%;
- orthotropic ±45 glass box under tip torque against Bredt: twist −0.01%, wall shear −0.01%. This also confirms the orientation and the local-axis stress output;
- first bending frequency against Euler–Bernoulli: −0.53%;
- load reconstruction against the beam model's V, M_b and T: 0.63% worst;
- FE wing mass against the section table: −0.84% (coarse mesh; −0.6% medium).

## Tests that the margins rest on

Coupon and element tests, in order of consequence:

1. **Glass/EPS 20 sandwich wrinkling, in edgewise compression, and in compression with shear** (a ±45 skin panel under combined load, or a sandwich beam plus a picture-frame shear coupon), at production cure pressure. Also flatwise E_c and G_c of the EPS 20 batch. This decides between EPS 20, EPS 30 and XPS for y < 0.5 m. [NASA sandwich wrinkling review](https://ntrs.nasa.gov/citations/19990017863)
2. **Pultruded cap compression on the Rohacell web:** a short sandwich column for cap wrinkling or crippling against the 187 MPa allowable. Also cap tension and compression coupons.
3. **Joint element test:**
   - tube in sleeve with ribs and wraps, under the paired loads of Table 1 (gust + at Vc and the fixed-aileron case) to ultimate;
   - measure rotation and free play;
   - a separate transverse ring-crush test of the tube (ovalization, MS +0.04);
   - tube 4-point bending to verify the 500 MPa allowable.
4. **Belly-landing drop test** of the centre section with a dummy pack mass, on the chosen belly and skid, at 1.5 m/s. Measure the peak deceleration; the 10 mm vs 30 mm crush case decides it.
5. **XPS to EPS transition coupon** at the servo bay, with the cut-out and hatch.
6. **Winglet joint** with overlap tapes, under the 39 N side load to ultimate.
7. **Battery lid** (or closed pocket): torsion test of a bay-length wing segment.
8. **Skin shear coupon** (±45 tension, ASTM D3518-style) for the 60 MPa allowable.
9. **Elevon hinge peel/shear coupon.**
10. **Ground vibration test** of the assembled wing (12.3 / 36 / 47 Hz expected) to correlate the modes before the flutter clearance. [FAA AC 20-107B](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_20-107B_with_change_1.pdf)

## Reproduce

From `scripts/` with the repository venv. CalculiX 2.22 from the FreeCAD 1.1.4 bundle is used read-only; set `CCX_EXE` to override the path. Inputs are read from `out/fea/inputs_v1_20261114_1716/`; set `FEA_INPUTS` to a folder with the same file names to rerun on regenerated v1 files.
- `../.venv/Scripts/python.exe -X utf8 fea_wing.py`: frequencies (`out/fea/modes.csv`, `modes_shapes.csv`), then statics, load matrix, margins.
- `../.venv/Scripts/python.exe -X utf8 fea_extra.py`: statics, load matrix, non-flight cases, joint checks and margins only.
- `../.venv/Scripts/python.exe -X utf8 fea_extra.py --convergence`: the three-mesh check (`out/fea/convergence.csv`).
- `../.venv/Scripts/python.exe -X utf8 test_fea_wing.py`: known-answer tests.
