# Detection model validation against field data (gap G5)

2026-11-15

## The answer

The replay's person-detection model is **pessimistic**. It is not optimistic anywhere I could test it.

- **Open ground:** at the same camera geometry as the one field study with usable numbers ([Doull et al. 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8232034/)), the model gives a single-look Pd of 0.75. The field values are 0.86 (automated detector, or a person reviewing oblique dusk images) and 0.92 (a person reviewing nadir dawn images). The model is low by 0.11 to 0.18 in Pd, which means its detection odds are 2 to 4 times too low.
- **Woodland canopy:** the model loses too much Pd under trees. At 75-100% cover it keeps 33% of its open-ground Pd. The field data keep 55% (manual) and 67% (automated). At the current crown porosity of 0.20 the model gives 0.22 where the field gives 0.50 to 0.57.
- **Dense temperate forest:** the model is within the published range. Single thermal frames under dense forest at 35 m gave an average precision of 24.8% and a recall of 50.4% at 25% precision ([Schedl et al. 2020](https://ar5iv.labs.arxiv.org/html/2009.08835)). The model gives 0.26 at 90% cover.
- **Least supported parameter: crown porosity (0.20).** It has no source. The only canopy-class field study fits a porosity of 0.49 (manual review) to 0.63 (automated). The implied N50, set by the inherited "sweep width = 0.8 × footprint" rule, is the next weakest. Blur (10 ms) is the best supported: the manufacturer's range of 7-12 ms moves Lodestar's open-ground Pd by only ±0.03.
- **The current baseline is the case least favourable to Lodestar.** Lodestar's 50° camera has coarser pixels (17.5 cm) than the competitors' 32° class (10.8 cm), so it sits lower on the detection curve. Lodestar's per-pass Pd is 0.72 of a competitor's at the current setting. That rises to 0.82 at the nominal calibration and 0.89 at the high one. Raising detection helps every aircraft, but it helps Lodestar most.

The calibration range for the mission runs is in Table 3. The **current settings are the low case**. The nominal case is an odds scale of 1.5 with crown porosity 0.35. The high case is an odds scale of 3.0 with porosity 0.50.

## How the model computes Pd per pass

The model lives in `scripts/terrain_sim.py` (`camera_probability`, `canopy_clear`, `CAMERA_ODDS_SCALE`, `CROWN_POROSITY`).

1. **Pixel size.** The camera has 640 pixels across its field of view at 120 m above the person: 50° for Lodestar (Boson 640, 8.7 mm) and 32° for the competitors. The centre ground sample distance is 17.5 cm for Lodestar and 10.8 cm for the competitors. Off-nadir pixels grow by 1/cos² across track and 1/cos along track.
2. **Blur.** Motion blur adds ground speed × 10 ms in quadrature to the along-track pixel.
3. **Cycles on the person.** N = 0.5 m / (2 √(across × along)). The 0.5 m critical dimension (shoulders, crouched or partly covered) is an assumption from `v3_blur.py`.
4. **Pass probability.** odds = N^(1.51 + 0.24 N) × CAMERA_ODDS_SCALE, and Pd = odds / (1 + odds). This is the Johnson target transfer function with the N50 folded into a scale factor.
5. **Calibration.** CAMERA_ODDS_SCALE = 0.735 is fitted so that the 32° camera at 120 m, with no blur, sees a sweep width of 55 m. That is 0.8 of its 68.8 m footprint, which is the milestone 4 rule in `mission_sim.W_SWEEP`. At Lodestar's nadir at 18 m/s this matches a standard Johnson N50 of 1.19 cycles. The usual detection criterion is 1.0 ± 0.25 cycles across the minimum dimension, or 0.75 across the geometric mean ([NVESD table, Leachtenauer exhibit](https://www.docketalarm.com/cases/PTAB/IPR2015-00436/Inter_Partes_Review_of_U.S._Pat._8599001/10-14-2015-Patent_Owner/Exhibit-2012-Ex_2012___Leachtenauer/); [Johnson's criteria](https://en.wikipedia.org/wiki/Johnson%27s_criteria)).
6. **Canopy.** Detection also needs a gap: a per-person uniform draw below [1 − c(1 − porosity)]^(1/cos θ). Here c is the NLCD 2021 tree-canopy fraction of the 30 m cell, and porosity is 0.20. The draw is shared by every pass, so repeat passes never see a hidden person.
7. **Detection.** A pass detects the person if the camera draw, the canopy draw and bare-earth line of sight all succeed. The detection draw is fresh on each leg.

`scripts/detection_check.py` reproduces `camera_probability` and `canopy_clear` exactly (it asserts equality) and evaluates them at the replay's conditions and at the published trial conditions.

## Model Pd at the replay's conditions

**Table 1.** Model single-pass Pd vs. canopy cover c, by camera and ground speed. Pd is averaged across the swath.

| Camera | Ground speed [m/s] | Open (c = 0) | Light (c = 0.375) | Heavy (c = 0.875) | Full (c = 1.0) |
|---|---:|---:|---:|---:|---:|
| Lodestar, 50°, 17.5 cm | 18 | 0.48 | 0.33 | 0.14 | 0.09 |
| Lodestar, 50°, 17.5 cm | 20 | 0.47 | 0.33 | 0.14 | 0.09 |
| Competitor, 32°, 10.8 cm | 18 | 0.65 | 0.46 | 0.19 | 0.13 |
| Competitor, 32°, 10.8 cm | 20 | 0.64 | 0.44 | 0.19 | 0.12 |

*Conditions: 120 m above the person, 640 px, 10 ms time constant, CAMERA_ODDS_SCALE 0.735, crown porosity 0.20. Light and heavy are the midpoints of the field study's 25-50% and 75-100% canopy classes. Source: `out/detection_check/pd_table.csv`.*

Lodestar's sweep width at 18 m/s is 54 m, inside a 112 m swath. The plan's metric P5 asks for "area searched at Pd ≥ 0.5 under canopy". Under the current calibration Lodestar never reaches Pd 0.5 on a single pass, even in the open. That metric only becomes measurable under the nominal or high cases.

## Published evidence

**Table 2.** Published person-detection results from aerial thermal and RGB imagery, with the conditions that set comparability.

| Source | Sensor and geometry | Detector | Cover | Result | Comparable to per-pass Pd? |
|---|---|---|---|---|---|
| [Doull et al. 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8232034/), miombo woodland, Tanzania | Zenmuse XT 336, 6.8 mm (45.6°), 50 m hover (two flights at 70 m); GSD about 12.5 cm | 5 people reviewing stills; YOLO-type detector | Open 0-25%, low 25-50%, medium 50-75%, high 75-100% | Overall thermal Pd 0.69 (manual) and 0.73 (automated); RGB 0.40 and 0.14. From the published logit model, nadir and dawn: open 0.92, low 0.84, medium 0.71, high 0.50 (manual); 0.86, 0.86, 0.71, 0.57 (automated) | Yes, single look. GSD is close to Lodestar's; there is no motion blur |
| [Hambrecht et al. 2019](https://www.sciencedirect.com/science/article/abs/pii/S0006320718315726), same group, miombo | Thermal and RGB drones | People reviewing images | Canopy density classes | Thermal beats RGB; canopy density, distance from the image centreline and the analyst are significant for thermal; altitude is significant only for RGB | Direction only (numbers not retrieved) |
| [Schedl et al. 2020](https://ar5iv.labs.arxiv.org/html/2009.08835), Austria | 640×512 thermal, 50.8°, 30-35 m; GSD about 6.5 cm | Trained detector | Dense conifer, broadleaf and mixed forest | Single frames: AP 24.8% (precision 25.1%, recall 50.4%). Integrated airborne optical sectioning (AOS) images: AP 92.2%, 53 of 57 people found | Yes for single frames (dense forest) |
| [Schedl et al. 2021, Science Robotics](https://www.aaas.org/news/rescue-drone-sees-through-thick-forest-track-people) | AOS on an autonomous drone | Onboard classifier | Conifer, broadleaf, mixed | 38 of 42 hidden people found in free flight; 86% on predefined paths | No: multi-frame integration, low altitude |
| [Kurmi et al. 2022](https://pmc.ncbi.nlm.nih.gov/articles/PMC8907346) | FLIR Vue Pro 512 px, 43°, 30-37 m, 4-10 m/s | Classifier over 30-frame integrals | Open field, conifer, broadleaf | Under forest, single-image confidence could not separate true from false detections (ratio 0.04-0.83); combining frames raised it to 4-94 | Qualitative support for low single-look Pd under dense forest |
| [SAVIOUR 2024 trial, Rogaland, Norway](https://internationaljournalofdisasterriskmanagement.com/index.php/Vol1/article/view/234) | 6 helicopter and 6 drone crews, 48 sorties, 251 subjects | Operational crews | Low, medium and high vegetation | Mean probability of detection per sortie above 83%; high-density forest 73.3% (drone) and 71.4% (helicopter) | No: cumulative over a whole sortie, altitude not stated |
| [Büttner et al. 2026](https://pmc.ncbi.nlm.nih.gov/articles/PMC13338175/) | High-resolution thermal, 50-120 m standoff, alpine | Detectors vs. people | Mixed trees and rock | Best mAP50 of 0.55 on a 640×512 benchmark where people average about 10 px tall; human reviewers generally beat the model | Context: algorithms trail people at low pixel counts |
| HERIDAL, SARD and WiSARD datasets ([survey](https://arxiv.org/html/2309.09518v2)) | RGB, or RGB and thermal | Detector benchmarks | Mixed | Detection benchmarks only; no per-pass probability or sweep width | No |

No published thermal-drone sweep-width study at 100-120 m was found. Koester's sweep-width method for land search ([USCG/NSARC guide](https://www.dco.uscg.mil/Portals/9/CG-5R/nsarc/LSA%20-%20Simple%20Design%20Volume%20II%20Appendixes%205-20-06.pdf); [DTIC ADA511593](https://apps.dtic.mil/sti/pdfs/ADA511593.pdf)) defines the experiment a drone trial would need. The work found applies it to ground searchers, not to drones. So the comparison below uses single-look detection at matched geometry rather than sweep width.

## Same-condition comparison

**Table 3a.** Single-look Pd vs. canopy class: model at the field study's geometry vs. Doull et al. 2021.

| Canopy class (midpoint c) | Field, manual, nadir, dawn | Field, automated, nadir | Field, manual, oblique, dusk | Model, porosity 0.20 | Model / field, manual nadir |
|---|---:|---:|---:|---:|---:|
| Open (0.125) | 0.92 | 0.86 | 0.86 | 0.67 | 0.73 |
| Low (0.375) | 0.84 | 0.86 | 0.72 | 0.52 | 0.62 |
| Medium (0.625) | 0.71 | 0.71 | 0.55 | 0.37 | 0.52 |
| High (0.875) | 0.50 | 0.57 | 0.34 | 0.22 | 0.44 |

*Conditions: model at 336 px, 45.6°, 50 m, hover (no blur), nadir, current odds scale 0.735. The model's camera-only Pd is 0.745. Field values come from the logit coefficients in Doull et al. 2021, Tables 2 and 4 (reference: open canopy, 45° view, dawn). Source: `out/detection_check/canopy_fit.csv`.*

**Table 3b.** Pd under canopy as a fraction of open-ground Pd: field ratios vs. model ratios, by crown porosity.

| Class | Field, manual | Field, automated | Model p = 0.10 | p = 0.20 (current) | p = 0.35 | p = 0.50 |
|---|---:|---:|---:|---:|---:|---:|
| Low | 0.91 | 1.00 | 0.75 | 0.78 | 0.82 | 0.87 |
| Medium | 0.77 | 0.82 | 0.49 | 0.56 | 0.65 | 0.73 |
| High | 0.55 | 0.67 | 0.24 | 0.33 | 0.47 | 0.60 |

*Least-squares porosity fit to the three ratios: 0.49 (manual) and 0.63 (automated).*

Figure 1 (`out/detection_check/pd_vs_canopy.png`) plots the same comparison: the model lines for porosity 0.10-0.50 fall below the field points at every cover class.

**Figure 1.** Single-look Pd vs. canopy cover c, by crown porosity p, with the Doull et al. 2021 manual and automated points. *Model at the field geometry (336 px, 45.6°, 50 m, hover, nadir) with odds scale 0.735.*

**Dense forest check.** At the Schedl et al. 2020 geometry (640 px cropped to 512, 50.8°, 35 m, 4 m/s assumed), the model's camera-only Pd is 0.94. At 90% cover it gives 0.18, 0.26, 0.39 and 0.51 for porosity 0.10, 0.20, 0.35 and 0.50. The published single-frame AP of 0.25 corresponds to porosity 0.18. The recall of 0.50, reached only at 25% precision, corresponds to 0.49. For dense temperate forest, the current 0.20 is therefore a fair central value. The miombo woodland data (open-crowned, semi-deciduous) support about 0.5.

## Verdict

1. **Open ground: pessimistic by 0.11-0.18 in Pd** at matched geometry (0.75 vs. 0.86-0.92). Part of the gap is fair: the field subjects were mostly standing or walking (pose was not significant in that study), the images were stills reviewed without time pressure, and dawn gives high thermal contrast. A real-time operator or onboard detector over warm afternoon ground will do worse. The model's 0.75 is still below even the field study's least favourable open-ground case (oblique view at dusk, 0.86).
2. **Woodland canopy: pessimistic by a factor of 2 to 2.6 under heavy cover.** Porosity 0.20 removes too much signal for open-crowned woodland.
3. **Dense temperate forest: within range.** Porosity 0.20 matches the single-frame AP under dense forest.
4. **Structure: repeat passes are pessimistic.** One hiding draw per person caps cumulative Pd at the gap fraction. That is 0.30 at 87.5% cover with porosity 0.20. Operational crews reached 73% per sortie in high-density forest (SAVIOUR 2024), and AOS shows that different view angles see through different gaps. This matters little for single-pass trail routes. It matters more for the expanding square, where the same ground is seen from several angles.
5. **Least supported parameter: CROWN_POROSITY.** It has no source, the one quantitative field study fits a value 2.5-3 times higher, and it applies uniformly to forest types that differ a lot. Second is the implied N50 (CAMERA_ODDS_SCALE), which rests on the unsourced "0.8 × footprint" sweep-width rule and the 0.5 m critical dimension. Blur is the best supported.

## Calibration range for the mission runs

**Table 4.** Detection calibration cases: parameter values and the resulting single-pass Pd, by camera and canopy.

| Case | CAMERA_ODDS_SCALE | CROWN_POROSITY | Basis | Implied N50 at Lodestar nadir, 18 m/s | 32° sweep factor (no blur) | Pd at Doull geometry | Lodestar open / light / heavy | Competitor open / light / heavy | Lodestar ÷ competitor, open |
|---|---:|---:|---|---:|---:|---:|---|---|---:|
| Stress (below evidence) | 0.735 | 0.10 | Existing sensitivity run | 1.19 | 0.81 | 0.75 | 0.48 / 0.32 / 0.10 | 0.67 / 0.44 / 0.14 | 0.72 |
| **Low (current baseline)** | **0.735** | **0.20** | Inherited 0.8 × footprint rule; dense-forest single-frame AP (Schedl 2020) | 1.19 | 0.81 | 0.75 | 0.48 / 0.33 / 0.14 | 0.67 / 0.47 / 0.20 | 0.72 |
| **Nominal** | **1.5** | **0.35** | Odds scale reproduces the field Pd for an automated detector, or a person reviewing oblique dusk images (0.86); porosity midway between the dense-forest and woodland fits | 0.81 | 0.90 | 0.86 | 0.66 / 0.49 / 0.28 | 0.80 / 0.61 / 0.34 | 0.82 |
| **High** | **3.0** | **0.50** | Odds scale reproduces a person reviewing nadir dawn images (0.92); porosity from the manual woodland fit (0.49) | 0.59 | 0.94 | 0.92 | 0.79 / 0.64 / 0.44 | 0.89 / 0.72 / 0.50 | 0.89 |

*Conditions: 120 m, 640 px, 10 ms time constant. Lodestar 50° at 18 m/s ground speed; competitor 32° at 16.5 m/s. Light c = 0.375 and heavy c = 0.875. Pd is averaged across the swath. Source: `out/detection_check/calibration_cases.csv`.*

- **Pd cap:** none is needed. The high case tops out at 0.79 (Lodestar) and 0.89 (competitor) in the open, below the 0.92 field value. If a cap is wanted for a future camera, 0.92 is the highest single-look value with a source.
- **Blur:** keep 10 ms. If a third axis is wanted, use 7 and 12 ms. These move Lodestar's open-ground Pd at 18 m/s from 0.48 to 0.52 and 0.46.
- **Track spacing:** keep `CAMERAS` spacing (67.5 m Lodestar, 55 m competitors) fixed across the cases, so that only detection changes.

### How to set them

- **Crown porosity (terrain replay):** the environment variable is already supported, and the output goes to `out/terrain_porosity_<value>/`:
  ```powershell
  $env:LODESTAR_POROSITY = '0.50'
  .venv/Scripts/python.exe -X utf8 scripts/terrain_sim.py 500 --pattern 'trails 3' --pattern 'trails 6'
  Remove-Item Env:LODESTAR_POROSITY
  ```
  The 0.10 and 0.35 runs already exist (`out/terrain_porosity_0.1`, `_0.35`). Only 0.50 is new.
- **Odds scale (terrain replay):** there is no CLI flag or environment hook. `camera_probability` reads the module global `terrain_sim.CAMERA_ODDS_SCALE` at call time. Workers are spawned processes on Windows and re-import the module, so the value has to be set inside each worker, not in the parent. Two ways to do that:
  - In a driver that runs `t.fly` in its own worker function (the `spacing_check.py` pattern), put `t.CAMERA_ODDS_SCALE = 1.5` as the first line of the worker.
  - For `terrain_sim.py` itself, whoever owns the headline runs can add a two-line hook mirroring the porosity one (not made here). For example: `CAMERA_ODDS_SCALE = float(os.environ.get("LODESTAR_ODDS_SCALE", 0)) or camera_calibration()`, plus an output redirect to `out/terrain_odds_<value>/`.
- **Flat-world mission sim (`mission_sim.py`, `v3_blur.py`, `v4_upgrade.py`):** the same cases are the sweep-width factor in `W_SWEEP = factor × SWATH`: 0.80 (low), 0.895 (nominal) and 0.944 (high). Keep `TRACK` at its current value. Note that `v3_blur.W0` and `terrain_sim.CAMERA_ODDS_SCALE` are derived from `W_SWEEP` at import time, so a change has to happen before those modules are imported.

## How much the headline depends on it

- **Porosity (measured).** On the 6 km trails route, 1.29 kg wing, SiCore help within 3 h was 10.51%, 11.45% and 12.91% at porosity 0.10, 0.20 and 0.35. Evo GOV was 10.34%, 11.43% and 12.77% (`research/2026-11-13-canopy-replay.md`). The steps are roughly linear at about 1 point per 0.1 of porosity. So 0.50 should land near 14.3% for SiCore; that is an extrapolation, not a run. Lodestar and the Evo GOV moved together to within 0.17 points, so porosity shifts the absolute P1 but not the comparison.
- **Odds scale (not run).** This parameter does change the comparison. Lodestar's open-ground per-pass Pd rises by 36% (nominal) and 64% (high). The competitors rise by 20% and 33%, because their finer pixels already sit higher on the detection curve. On single-pass trail routes the share of hikers found scales roughly with per-pass Pd. So the current baseline understates Lodestar relative to the 32° competitors, and P1 should be reported at the low, nominal and high cases rather than at the baseline alone.
- **What to run in Step 2:** the four cases in Table 4 on the headline routes and seeds, with track spacing held fixed. Report P1 as low / nominal / high. Report P5 (Pd ≥ 0.5 under canopy) only for the nominal and high cases, because the baseline never reaches 0.5.

## Limits

- Only one study (Doull et al. 2021) gives single-look Pd by canopy class with thermal at a GSD close to Lodestar's. It is one site, standing or walking subjects, stills reviewed at leisure, and no motion. The nominal and high cases lean on it.
- Field canopy classes are local cover over the subject. The model uses NLCD fraction in a 30 m cell. The two are related but not the same quantity.
- The AOS figures are for a trained detector on single frames at a detection threshold chosen for AP. They are not an operator watching video.
- No published drone sweep-width (lateral range) experiment at 100-120 m was found. A flight test over people lying in grass and under canopy at 18 and 20 m/s, scored with Koester's sweep-width method, would replace these cases with a measurement. It remains in the test plan.

## Files

- `scripts/detection_check.py`: reproduces the replay's detection functions and evaluates the tables above. It runs in about 6 s.
- `out/detection_check/`: `pd_table.csv`, `canopy_fit.csv`, `calibration_cases.csv`, `detection_check_log.txt`, `pd_vs_canopy.png`.

```powershell
.venv/Scripts/python.exe -X utf8 scripts/detection_check.py
```
