# What makes a design repo look professional (and not AI-made)

Studied 2026-11-13 (shallow clones in `/home/user/refs/`, outside this repo):
AeroSandbox, Feather-RC-Glider, solar-seaplane-preliminary-sizing (all Peter Sharpe, MIT), RocketPy, SciencePlots.

## What they do

1. **One dimensioned engineering drawing is the front page.** Feather's `CAD/feather.png` is a
   single sheet: three-view with dimensions, leader-line callouts, an "Overall Properties" block,
   mass breakdown table, control scheme, flight-dynamics numbers (phugoid, Dutch roll, spiral),
   detail views. Monospace type, black on white, no colour. It looks like a title block, not a slide.
2. **The README is a design narrative, not a feature list.** It states the decision, the
   alternatives (hand-launch vs discus-launch), and why each was rejected, with the numbers.
   First-person, plain sentences, no emoji bullets, no "key features" boilerplate.
3. **Real artifacts next to the claims.** Native CAD files, `.xfl`/`.xml` analysis files, build
   photos, historical weights, the first-flight video link. The study is shown to have become metal.
4. **The scripts are the study.** `design_opt.py`, `span_sweep.py`, `glide_polar.py` at repo root,
   one command each, and the figures are what those scripts write into `figures/`.
5. **Few, quiet badges** (AeroSandbox: build, PyPI, license). RocketPy's long badge wall is the
   weaker look. Citation file (`CITATION.cff`) and a changelog read as serious.
6. **Plot style is a single shared file.** SciencePlots' base style: inward ticks on all four
   sides, minor ticks, 0.5 pt axes, no legend frame, serif font and LaTeX maths, a small fixed
   figure size (3.5 x 2.6 in) so text matches a paper's body text, and a muted fixed colour cycle.

## Where Lodestar differs now

- Plots (`out/report/img/trades.png`) are default-matplotlib: DejaVu Sans, boxed legend, bright
  blue/orange, grid on, no shared style. Reads as "generated".
- Renders (`v2_exploded.jpg`) are small objects floating in a large grey void with soft stock
  shadows, and primary-colour boxes for parts. Reads as a placeholder scene.
- No dimensioned drawing sheet at all. No mass-budget figure. The report is an HTML page of
  headline numbers rather than a design packet.
- Results are scattered across many dated notes with no single "point design" table.

## Plan for the detailed-design pass

1. `style/lodestar.mplstyle` used by every script (SciencePlots-like rules, one sober palette,
   consistent figure widths, direct labels instead of boxed legends).
2. A dimensioned three-view sheet generated from the STEP/AVL geometry, with a properties block,
   mass budget, CG envelope, stability derivatives (build123d or Blender freestyle + matplotlib).
3. Mass-budget and CG-travel figures (kit on / kit off).
4. Re-render hero shots: fill the frame, real ground plane, mid-grey materials, one accent.
5. README rewrite as design narrative: decision, rejected alternatives with numbers, one command per study.
6. `CITATION.cff`, `CHANGELOG.md`, trimmed badges.

## Text rules (owner preference, 2026-11-13)

Technical, not editorial. Titles name the quantity, the independent variable and the grouping, with
symbols and units, e.g. "Mission score vs. unit cost, by cell type, wing construction, search speed".
Test conditions (wind model, Monte Carlo run count, mission, camera) go in a small caption line, not
in axis labels. Axis labels are "quantity [unit]". No rhetorical or question-style titles and no
"what X buys" phrasing in figures, README or report headings. Figures are numbered
("Figure 3. ...") with a plain description.
