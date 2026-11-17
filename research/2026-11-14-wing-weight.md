# Wing mass reduction by core material, with a skin-wrinkling limit

2026-11-14

## Answer

The v1 wing is now **1.881 kg** with joints, down from 2.291 kg (−0.41 kg, −18%). The change: the solid XPS core (33 kg/m³) becomes EPS (20 kg/m³) everywhere except the servo bay and the winglet root. The spar caps were resized under a new skin-wrinkling limit, so the lighter core does not let the skin buckle into the foam. The loaded aircraft drops from 8.40 to 7.99 kg (SiCore), and loaded stall at 2,000 m drops from 12.63 to 12.34 m/s TAS at the sizing CLmax of 0.80.

The 1.29 kg budget is not met. That budget came from milestone-1 sizing, before the three-piece joint (0.34 kg), its local supports, and the surface finish were counted. On a like-for-like basis (budget plus the joint), the wing is 0.25 kg over, and it is **14.6 g/dm²**. Comparable published builds run 10.5 g/dm² (a measured foam-core CFRP thesis wing) to about 16 g/dm² (estimate), per [the competitor construction survey](2026-11-14-competitor-wing-construction.md). Lodestar now sits inside that band. The v1 wing budget is reset to **1.90 kg including joints**, and the excess is recorded in the requirements table.

## What was holding the wing back

**Table 1.** Wing mass by component [kg], compact-XPS wing (both sides).

| Core | Skins + finish | Spar caps | Spar web | Rear web, hinge | Servos | Joints | Total |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.159 | 0.495 | 0.071 | 0.114 | 0.053 | 0.060 | 0.339 | 2.291 |

The core alone was half the wing. The skins sit at minimum gauge: torsion shear uses 17% of their allowable, so a thinner skin would be set by handling damage, not load. The spar caps are already cheap (71 g). That left the core as the lever.

## Method

1. **Wrinkling limit added to spar sizing** (`structures.size_wing`).
   - The skin carries the same strain as the caps, scaled out to the surface. The skin's compressive stress must stay below the Hoff wrinkling stress k·(E_f·E_c·G_c)^(1/3), with k = 0.3 as a knocked-down design value for hand laminates ([NASA sandwich wrinkling review](https://ntrs.nasa.gov/citations/19990017863)).
   - The limit is applied at ultimate load, FOS 1.5. Where it binds, the caps get thicker until the skin strain falls enough.
   - It does not bind on XPS (utilization 0.60). Re-sizing the archived XPS wing gives identical caps, so every earlier XPS result stands.
2. **Core candidates** were run through the full converged mass–load loop in `wing_trade.py`. The loop covers both packs, kit on and off, sea level and 4,400 m, gust and roll cases, joint checks and the clamped flutter screen.
3. **EPS 20 properties are assumptions** at the low end of EN 13163 EPS 100 data: E = 4 MPa, G = 2.5 MPa. Glass on EPS 20 gives an allowable wrinkling stress of 14.4 MPa.

**Table 2.** Wing mass and margins by core layout, compact 20 mm joint, converged loads.

| Design | Core | Wing kg | Skin wrinkling utilization | Tube stress MPa (vs 500 assumed) | Limit tip deflection mm | Clamped flutter, sea level m/s TAS |
|---|---|---:|---:|---:|---:|---:|
| compact_xps | XPS everywhere | 2.291 | 0.60 | 149 | 33.4 | 105.7 |
| compact_eps20 | EPS 20; XPS at joint ±0.20 m, servo bay, winglet root | 2.042 | 0.99 | 153 | 32.2 | 106.8 |
| **compact_eps20_light (v1)** | EPS 20; XPS at servo bay and winglet root only | **1.881** | **0.98** | **152** | **30.3** | **106.5** |

Conditions: 1 ply ±45 glass 80 g/m² skin, pultruded carbon caps, Rohacell 51 spar web, 20 × 16 mm tube, 24 mm sleeves, 45 g allowance per break.

- **EPS through the joint.** The sleeve's local support blocks, wraps and aft key are already carried in the 45 g-per-break joint allowance. The foam between them therefore needs only to support the skin, the same job as everywhere else.
- **Carbon skin was dropped without a full run.** A stiffer skin takes more stress at the same strain, while the wrinkling allowable grows only as E_f^(1/3), so carbon makes wrinkling worse on EPS.
- **The earlier EPS 16 mixed core** had wrinkling demand equal to its allowable. EPS 20 was chosen for some headroom on handling and cure pressure.

## Update, 2026-11-15: FE verification raised the wing to 1.983 kg

The shell/solid FE model (`scripts/fea_wing.py`, CalculiX) showed the beam sizing was unconservative on the EPS skin. The beam check compared only spanwise compression with the Hoff wrinkling stress. The FE showed that torsion shear of 9–12 MPa, combined with that compression, gives a minimum principal stress up to 19.4 MPa against the 14.4 MPa allowable (margin −0.26). The torsion shear peaks across the battery-bay span. A hard belly landing (10 mm crush stroke) also failed the root skin and lower cap.

**Table 3.** Change by step, v1 wing [kg].

| Step | Wing kg | FE minimum flight skin margin |
|---|---:|---:|
| v1 as first frozen (spanwise wrinkling check) | 1.881 | −0.26 |
| Caps sized on principal stress (compression + torsion shear) | 1.907 | −0.09 |
| + FE correlation factor 1.10 inboard of y = 0.30 m | 1.923 | −0.09 |
| + doubler y = 0.10–0.36 m (hard ply edges) | 1.955 | −0.15 (stress peaks at the doubler ends) |
| **+ doubler over the whole centre section, y = 0–0.45 m, ply drop-off over 0.45–0.50 m (frozen)** | **1.983** | **+0.12** |

What changed in the design:
- **Sizing criterion.** `structures.size_wing` now limits the minimum principal stress of each compressed face: spanwise compression plus torsion shear. Inboard of y = 0.30 m it carries a 1.10 FE correlation factor.
- **Upper-skin doubler.** One extra ±45 glass ply (80 g/m²) on the centre-section upper skin halves the torsion shear there. The centre section is its own piece, so the doubler ends at the panel joint, with a 50 mm drop-off.
- **Belly skid.** An EPP skid with at least 40 mm of crush stroke. With 30 mm the root lower skin is still 3% short. With the skid, every landing margin is +0.7 or better.

The wing is 1.983 kg, or 15.4 g/dm². That is still inside the 10.5–16 g/dm² band of comparable builds. The v1 wing budget is reset again, to **2.00 kg including joints**, and the requirements table records the change.

## Where the remaining mass could come from (v2 candidates)

- **A molded hollow shell** (spar, ribs and sandwich skins, as the DeltaQuad and Bramor use) is about 1.7–1.85 kg by hand estimate. It needs a 3 mm sandwich core to keep the skin panels from buckling at the root strain. The saving over 1.88 kg is small for the tooling cost.
- **A two-piece wing** (one root joint inside the pod instead of two outboard) could save about 0.12 kg, but it breaks the three-piece car-portability requirement.
- **Weigh a 100 × 100 mm coupon of the actual layup.** The 0.03 kg/m² finish and the resin take-up are assumptions.

## Tests this choice depends on

1. A flatwise-compression and sandwich-beam wrinkling coupon, glass on EPS 20, at the production cure pressure. It must show at least 14.4 MPa face wrinkling stress, or the caps grow (cheap: about 3 g per 10% shortfall).
2. A foam-transition coupon at the servo bay, XPS to EPS.
3. A weighed skin coupon to confirm the areal masses.

## Reproduce

From `scripts/`:
- `../.venv/Scripts/python.exe -X utf8 wing_trade.py --designs compact_eps20_light _eps20_light` writes `out/wing_trade/*eps20_light*`.
- `../.venv/Scripts/python.exe -X utf8 test_wing_construction.py` runs 12 checks, all passing. They cover the hand value of the Hoff stress, XPS caps unchanged, the limit binding on EPS, and the baseline outputs.
