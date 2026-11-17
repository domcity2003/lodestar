# Terrain replay with tree canopy

2026-11-13

The baseline replay covers seven sites, 500 simulated hikers per site, seven aircraft and four search patterns (98,000 cases). Lodestar flies at 18 m/s with the 50° camera, while the competitor models use the inherited 32° camera. The full-fleet replay uses the 1.29 kg Lodestar wing budget. The separate 6 km trails-first wing replay uses the same hikers, routes and detection draws at each design's mass.

Tree cover is the 2021 USFS canopy fraction in the 30 m cell at the simulated person. For canopy cover \(c\), crown porosity \(p\), and view angle \(\theta\) from vertical, the gap probability is \([1-c(1-p)]^{1/\cos\theta}\), with a lower bound of 0.05 on the cosine. A shared uniform draw represents the person's location under cover across all passes and aircraft at that site. The crown porosity values 0.10, 0.20 and 0.35 are assumptions. The map gives coverage, and field data would be needed to calibrate thermal visibility. [USFS 2021 NLCD metadata](https://www.mrlc.gov/downloads/sciweb1/shared/mrlc/metadata/nlcd_tcc_conus_2021_v2021-4.tif.html)

| Aircraft and wing | Route | Help within 1 h | Help within 3 h |
|---|---|---:|---:|
| SiCore, 1.29 kg budget | Trails to 3 km | 6.13% | 12.11% |
| DeltaQuad Evo GOV | Trails to 3 km | 5.49% | 11.89% |
| SiCore, 1.29 kg budget | Trails to 6 km | 5.95% | 11.45% |
| SiCore, 2.29 kg compact XPS | Trails to 6 km | 5.95% | 11.08% |
| P50B, 2.29 kg compact XPS | Trails to 6 km | 5.95% | 10.03% |
| DeltaQuad Evo GOV | Trails to 6 km | 5.46% | 11.43% |

On the 6 km route the original-mass SiCore advantage over Evo GOV at one hour was 0.50 percentage points (paired Monte Carlo 95% interval +0.05 to +0.95). At three hours the difference was +0.03 points (−0.59 to +0.64). These t intervals cluster each simulation index across sites because the sites reuse random inputs. They quantify sampling variation over the fixed sites and assumptions. The kit adds 0.35 points at one hour and 0.03 at three hours relative to sending a ground team from those same SiCore detections.

The compact XPS SiCore replay fell from 10.08% to 5.95% at one hour and from 18.69% to 11.08% at three hours versus the preceding bare-earth replay, which also flew SiCore at 20 m/s. P50B was at 18 m/s in both replays and fell from 9.91% to 5.95% and 16.97% to 10.03%.

The mixed-core candidate saves 142 g relative to compact XPS. In the canopy wing replay its three-hour SiCore score rises from 11.08% to 11.14%, equivalent to about two modeled cases across the 3,500 hikers. The canopy replay gives little reason to trade core durability for that mass saving alone. Stall margin and build and test results still matter to the core decision.

## Crown porosity sensitivity

Each run repeats all 98,000 cases, with matching site, aircraft, route and hiker IDs. Every detection in the denser-canopy run also occurred in the corresponding more-open run.

| Crown porosity | SiCore, 1 h | Evo GOV, 1 h | SiCore, 3 h | Evo GOV, 3 h |
|---:|---:|---:|---:|---:|
| 0.10 | 5.50% | 4.94% | 10.51% | 10.34% |
| 0.20 | 5.95% | 5.46% | 11.45% | 11.43% |
| 0.35 | 6.72% | 6.26% | 12.91% | 12.77% |

These rows use the 6 km trails-first route and the original 1.29 kg Lodestar wing budget. Moving from 0.10 to 0.35 adds 2.39 percentage points within three hours for SiCore and 2.43 for Evo GOV, while the aircraft remain within 0.17 points of each other at three hours. The 3 km route scored higher for both aircraft across all three canopy settings. Both routes were evaluated on the same input sample.

## Competitor endurance assumptions

The default starts from published maximum endurance, uses the existing square-root density correction, and applies a 10% mission reserve in the simulator. The 0.80 case applies a separate 20% reduction in the competitor energy budget as a sensitivity to operating conditions, with the same aircraft and terrain inputs. The 20% value is a scenario assumption. DeltaQuad's 255-minute Evo GOV figure already assumes 90% battery use, low wind and a low-altitude VTOL cycle, so the simulator's reserve acts on the published usable budget. Bramor gives up to 3.5 hours in calm standard conditions, AgEagle gives up to 90 minutes with an endurance battery, and Quantum lists 90+ minutes for Trinity Pro. The Evo-LE's up-to-eight-hour figure depends on payload, conditions and mission profile. [DeltaQuad Evo GOV](https://docs.deltaquad.com/gov/vehicle-specifications) · [C-Astral Bramor ppX](https://www.c-astral.com/en/unmanned-systems/bramor-ppx/faq) · [AgEagle eBee X](https://ageagle.com/wp-content/uploads/2022/06/AgEagle-eBee-X-EN-2022-2.pdf) · [Quantum Trinity Pro](https://quantum-systems.com/wp-content/uploads/2023/12/QS_TPro_TechSheet_A4_BS3mm_231211_Screen.pdf) · [DeltaQuad Evo-LE](https://www.deltaquad.com/newsroom/press-releases/deltaquad-launches-evo-le-with-up-to-eight-hours-of-flight-endurance)

The 49,000-case endurance replay reproduced every Lodestar row on both routes. No competitor gained a detection or an earlier help outcome. Evo GOV's one-hour scores held and its three-hour score fell from 11.89% to 10.77% on the 3 km route and from 11.43% to 10.49% on the 6 km route. The original-mass SiCore's three-hour difference on the 6 km route became +0.97 points (paired 95% interval +0.30 to +1.64). The compact XPS SiCore's existing 11.08% score gives a numerical difference of +0.60 points in this scenario. Evo-LE still scores 11.71% within three hours on the same route. The direction of the compact wing's comparison with Evo GOV depends on the endurance assumption.

```powershell
$env:LODESTAR_POROSITY = '0.10'
uv run python -X utf8 scripts/terrain_sim.py 500
$env:LODESTAR_POROSITY = '0.35'
uv run python -X utf8 scripts/terrain_sim.py 500
Remove-Item Env:LODESTAR_POROSITY
uv run python -X utf8 scripts/terrain_sim.py 500 --pattern 'trails 3' --pattern 'trails 6' --competitor-endurance 0.8
```
