# Wing construction and areal mass of comparable UAVs

2026-11-14

## Plain answer

Nobody publishes wing-only mass for the commercial flying wings, so the range below is a mix of one measured data point and estimates.

- Your budget of 1.29 kg is about 14 g/dm² if the wing is ~0.9 m². Your current 2.29 kg is about 25 g/dm². (My arithmetic.)
- The best measured wing-structure data point I found is a 4.5 m UAV wing thesis: [1.59 kg structure on 1.518 m²](https://fenix.tecnico.ulisboa.pt/downloadFile/1126518382177895/MarioRamos_ExtendedAbstract.pdf), about 10.5 g/dm² (my division). It uses a foam core, a 0.15 mm CFRP skin and two pultruded carbon spar caps.
- A realistic target for a 0.8–1.0 m², 6–8 kg wing is **~12–16 g/dm²** (estimate). The method that gets there: foam core (or EPP) with thin vacuum-bagged glass/carbon skins, plus carbon spar caps. The skins are sized by the actual load, with no heavy molded sandwich except near the root.
- Competitors with this layout (carbon/Kevlar skins over a light core) are in the same weight class, but their wing masses are not published.

## Table 1: Comparable aircraft

| Aircraft | Span m | Wing area m² | MTOW kg | Empty kg | Construction | Source |
|---|---|---|---|---|---|---|
| DeltaQuad Evo (VTOL wing) | 2.69 | 0.84 | 10 | 4.8 (6.8 with battery) | Composite; DeltaQuad lists fiberglass, carbon and Kevlar | [specs](https://docs.deltaquad.com/evo/vehicle-specifications), [search summary of materials](https://docs.deltaquad.com/gov/vehicle-specifications) |
| DeltaQuad Evo Tactical | n/a | n/a | 11 | 5.8 | Same family | [specs](https://docs.deltaquad.com/tac/vehicle-specifications) |
| DeltaQuad Evo-LE | n/a | n/a | 13 | n/a | Same family | [gadgetreview](https://www.gadgetreview.com/deltaquad-evo-le-claims-8-hour-endurance-and-298-mile-range) |
| C-Astral Bramor ppX | 2.30 | n/a | 4.7 (takeoff) | n/a | Kevlar/carbon/Vectran composite | [C-Astral](https://www.c-astral.com/en/unmanned-systems/bramor-ppx) |
| Quantum Trinity Pro | 2.39 | n/a | 5.75 | n/a | Not found | [Frontier Precision](https://shop.frontierprecision.com/product/quantum-systems-4260574665982-trinity-pro-evtol-fixed-wing-mapping-drone/) |
| eBee X | 1.16 | n/a | ~1.6 | n/a | EPP foam, with carbon reinforcement on older models; the 2026 brochure lists EPP plus a Curv polypropylene composite | [senseFly](https://www.sensefly.com/drone/ebee-x-fixed-wing-drone/), [Advexure](https://advexure.com/products/ebee-x), [brochure](https://www.thefuture3d.com/spec-sheets/ageagle/AgEagle_eBee_X__Brochure_2026_EN.pdf) |
| Skywalker X8 | 2.12 | ~0.80 | ~3.2 (manual) | 0.88 (manual, dry kit) | Molded EPO foam | [Unmanned Tech Shop](https://www.unmannedtechshop.co.uk/products/skywalker-x8-kit-2120mm-epo-fpv-wing), [RobotShop](https://www.robotshop.com/products/sonicmodell-skywalker-x8-2122-flying-wing-drone-fpv-complete-kit) |
| CU Super RAAVEN | 4.5 | n/a | n/a | ~5.4 (12 lb "weight range") | Skinned EPP | [CU IRISS](https://colorado.edu/iriss/content/our-capabilities/super-raaven) |
| Ritewing Mini Drak | 1.0 | ~0.35 (550 in²) | n/a | 1.25 ready to fly | Molded EPP, two carbon rods | [Mini Drak review](https://fpv-shopping-list-2.pages.dev/mini-drak.html) |
| X-UAV Mini Talon (V-tail, not a wing) | 1.30 | n/a | n/a | 0.564 kit | EPO; wings and tail 244 g, spars 102 g | [Mapping Mozzie](https://mapping-mozzie.readthedocs.io/en/latest/parts/airframe/) |

The Evo is the closest competitor: 0.84 m², 2.69 m span, 4.8 kg empty with a 10 kg MTOW. Its whole airframe mass is a ceiling for how heavy its wing can be.

## Table 2: Areal mass by construction method

Most of these are aircraft-level numbers or my own estimates. Few published figures are wing-only.

| Method | Areal mass | Basis | Source |
|---|---|---|---|
| Foam core, thin CFRP skin, carbon caps (4.5 m UAV) | ~10.5 g/dm² structure | Measured in a thesis; my division | [Ramos thesis](https://fenix.tecnico.ulisboa.pt/downloadFile/1126518382177895/MarioRamos_ExtendedAbstract.pdf) |
| Molded hollow carbon F5J sailplane | 17–21 g/dm² **whole-aircraft** wing loading | Manufacturer claim; the wing is only part of this | [Carbonita](https://www.soaringusa.com/Carbonita-F5J-4m.html), [Simitri Light](https://www.soaringusa.com/F5J-LMR-ALES-Electrics/Simitri-Light-F5J.html), [Kappa 3.5](https://www.soaringusa.com/Kappa-3.5.html) |
| Balsa/carbon built-up F5J | 25.5–27 g/dm² **whole-aircraft** | Manufacturer claim | [Element 3.5E](https://www.modelairplanenews.com/element-3-5e-f5j-ales-electric-sailplane-arf/) |
| Balsa-sheeted foam | not found as g/dm² | Wing examples such as 280 g for a 1.9 m span, no area given | [Model Aviation](https://library.modelaviation.com/article/lightweight-wing-skinning-200807) |
| Molded EPO/EPP | not found | Only whole-kit masses exist | see Table 1 |
| Rough sandwich estimate | 1.0–1.5 kg/m² | My estimate from ply and core thicknesses, using [carbon ply ~1600 kg/m³, core 29–60 kg/m³](https://link.springer.com/article/10.1007/s40430-024-04785-2) | estimate |

The F5J numbers are total mass divided by area, so the wing alone is lower. The same source set notes that [balsa-sheeted foam is usually a little heavier than built-up balsa](https://groups.google.com/g/rec.models.rc.air/c/AffR7hn6-aI).

## Spar notes

- A carbon tube of 16 mm OD with a 1 mm wall weighs about [70 g/m](https://www.readymaderc.com/products/details/86721-strix-nesting-pair-1m-3k-carbon-fiber-spar-tubes-16-14mm-14-12mm). A 30 mm OD, 1.6 mm wall tube weighs about [165 g/m](https://gator-rc.com/gator-30mm-carbon-fiber-wing-tube-and-sleeve-gc1003?amp=1).
- Spars on the Mini Talon kit are 102 g of a 564 g airframe. That is a heavy share for a small plane, not a target.
- Pultruded cap strips are used in the thesis wing above (two caps in a foam D-box). I found no published per-meter mass for strips.
- A tube is easiest and strong in bending but sits at one depth. Cap strips on the wing surface give more bending stiffness per gram where the depth is large, which suits a thick flying-wing root. (My engineering judgment, not sourced.)
- Pattern to copy for the 6–8 kg class: two caps or a tube across the wing, thin skins carrying torsion, a light core to hold the skin shape.

## Caveats

- Measured: DeltaQuad Evo and Bramor ppX specs are manufacturer sheets. The 1.59 kg thesis mass is a published value.
- Estimated: every g/dm² figure for the Evo, Trinity and others. No one publishes their wing mass.
- The Trinity Pro tech sheet PDF would not parse, so empty mass, area and materials are missing.
- The F5J wing loadings include fuselage, tail and gear. Do not read them as wing-only mass.
- No balsa-sheeted, EPO or EPP areal mass numbers were found. A weighed sample (cut a 100 x 100 mm panel of your layup and weigh it) will beat any of these.
- Did not read the full Avramov NARCON paper (403 on fetch) or the Lisbon thesis body, only abstracts and snippets.
