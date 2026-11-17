# Mission simulation QA

2026-11-14

## Summary

This was a code-quality pass over the mission simulation. It did not run any new performance study. I reviewed `terrain_sim.py`, `flyable_mission.py`, `turn_path.py`, `long_pass.py` and `fleet_flyable.py`. I looked for unit and sign errors, density conventions, reserve accounting, seed hygiene, time-window boundaries and exceptions that were caught or ignored silently.

**What was checked and held up:**
- **Physics formulas:** the wind-triangle and bank formulas, the ground-radius bound, induced power in a bank, and the climb energy (Wh per metre).
- **Speed convention:** turn speeds are true airspeed (TAS). The density scaling is right for Lodestar and for the competitors.
- **Reserve:** 10% of usable battery energy, checked at every piece end. The return trip and a full-circle turn allowance are included.
- **Kit release:** the kit drops only after a find, at least 40 m above the person, and 185.7 s later. This holds on all four archives that include a kit.
- **Help windows:** the 1 h and 3 h windows include their end points and are measured from launch.
- **Seeds:** the tuning seed (20261111) and the held-out seed (20261114) produce disjoint hikers, winds and canopy draws.

None of these needed a change.

**What was broken and fixed.** None of these fixes changes an archived number.
1. **NaN slipped past the flight-envelope guard** (`flyable_mission.Flight`). If the crosswind exceeded the airspeed, ground speed became NaN. The old test `(gs <= 1).any()` treated NaN as passing. The replay would then have run with NaN energy and never hit the reserve. The guard now fails on NaN, and a non-finite return-home energy also raises. This cannot happen with today's 10 m/s wind cap and speeds of 14 m/s or more, so no archived run is affected.
2. **The pursuit tracker could truncate the trail silently** (`long_pass.pursue`). When it hit its step cap, the rest of the trail was dropped without a message. It now raises. I scanned every archived site, both trail patterns, speeds of 14 and 18.8 m/s and the 2 and 10 m/s wind bands. None of them reaches the cap.
3. **`smoke_test.py` always exited 0**, even when a tool failed. It now exits 1 and prints the counts. Today it reports one failure, build123d. The CAD kernel (OCP) is blocked by Windows Application Control. This is a machine policy, not a code bug.

**New default route.** `long_pass.route_flight(..., mode=DEFAULT_ROUTE_MODE)` is now the one place that builds a continuous route. The default mode, `pursuit_1`, is the pursuit tracker at lookahead 1.0 with one detection draw per pass.
- `fleet_flyable.py --route` selects the route mode. The default is `pursuit_1`. `rounded_adaptive` reproduces the archived run, and only that mode writes to the archived `flyable_fleet*` folder names.
- New options:
  - `--seed` for the held-out run;
  - `--sites`, `--patterns` and `--hikers` for smoke runs, which write to a `_subset` folder.
- `long_pass.py` now names `TUNING_SEED`, `HELD_OUT_SEED` and `DEFAULT_LOOKAHEAD`.
- `flyable_mission.Flight.profile()` exposes the time and energy profile, so tests can check it. `replay()` uses the same code.
- A run-for-run replay (`verify_route_modes.py`) confirms that the refactor changed no archived number.

**Smoke run of the new default** (`fleet_flyable.py --sites rocky_bear_lake --patterns "trails 3" --hikers 20`): all seven aircraft built and replayed pursuit routes in about 4 minutes. Output is in `out/wing_trade/flyable_fleet_pursuit_1_subset/`. Twenty hikers are far too few to read as a result.

**Test counts:** see Table 1. `run_all_tests.py` runs the fast set by default. `--full` adds the run-for-run replay and the tool-chain smoke test.

## Tests

**Table 1.** Test and verify scripts, what each checks, and result on 2026-11-14.

| Script | What it checks | Result |
|---|---|---|
| `test_terrain_sim.py` | Terrain-following profile, energy integration, return climb, line of sight, camera calibration, canopy, sharp-corner flight, competitor endurance | 33/33 pass |
| `test_turn_path.py` | Fillets, turnaround loops, tailwind and crosswind bank envelope, terrain under arcs, map edge, batched return climb, launch time and reserve on a straight | 8/8 pass |
| `test_long_pass.py` | Simplification, pursuit turn limit and trail-end stop, fewer loops, contiguous legs, pass grouping | 10/10 pass |
| `test_mission_invariants.py` (new) | Route modes, wind-band turn radius, bank in every wind direction, energy and time monotone, reserve, kit, 1 h / 3 h windows, units and density, envelope guards, seeds | 67/67 pass |
| `verify_flyable_replay.py` | Archive: 42,000 Lodestar flyable cases; sharp mode reproduces the terrain run site averages | pass |
| `verify_fleet_flyable.py` | Archive: 98,000 fleet rows; sharp competitors reproduce the terrain run and Lodestar reproduces the flyable replay, run for run | pass |
| `verify_fleet_flyable.py 0.8` | Same as above, at 80% competitor endurance | pass |
| `verify_long_pass.py` (new) | Archive: long-pass tuning (35,000 rows) and held-out (147,000 rows) counts and keys; matched winds; help ordering; held-out draws differ from tuning; kit invariants on four archives | 29/29 pass |
| `verify_route_modes.py` (new, `--full`) | Replays Rainier, trails 6, SiCore, tuning seed, through `route_flight`; all three modes match `long_pass_tune` and `rounded_adaptive` matches `flyable_fleet`, run for run | 2/2 pass |
| `smoke_test.py` (`--full`) | Tool chain: NeuralFoil, VLM, AVL, OpenVSP, JSBSim, build123d, FreeCAD, flow5 | 7/8: build123d blocked by Windows Application Control |

The fast set has 8 scripts and 150 checks, all passing. The full set has 10 scripts: 159 checks pass and 1 fails (build123d, an environment issue).

Run all: `cd scripts; ../.venv/Scripts/python.exe -X utf8 run_all_tests.py [--full]`. It exits nonzero on any failure.

New invariants in `test_mission_invariants.py`. Each runs for the default mode and both archived modes, at the 2 and 10 m/s wind bands:
- No arc is tighter than the wind-band ground radius.
- Bank stays at or below 30° for 36 wind directions at the band edge.
- Energy is never negative or decreasing, and time always increases.
- The descent energy never drops below the avionics idle floor.
- Pass IDs start at 0, never decrease and fit the draw array.
- **Reserve:** the searched distance stops at the last piece that keeps 10%. A hiker just beyond that point is missed, and one just inside it is found.
- **Kit:** the timing and height rules hold. The kit is withheld when the ground under the aircraft is too high, and competitors never drop one.
- **Help windows:** exactly 3600 s and 10,800 s count. 3600.5 s does not.
- **Units and density:** the still-air radius at 18 m/s is 57.2 m. Lodestar's power model uses the site density. The competitor turn floor is a sea-level value that rises with altitude, in TAS.
- **Seeds:** the two seeds give different draws, and every hiker has 6,000 detection draws.

## Open issues

1. **The pursuit planning radius uses the speed before convergence.** `route_flight` sizes the tracker radius from `max(v_cruise, min_turn_speed(site altitude))`. `build_route` can then raise the route speed for higher terrain, by up to 0.5 m/s at Bear Lake (18.37 to 18.78 m/s for SiCore). The tracker margin then falls from 1.3× to about 1.26× the true route radius. This is not a physics error, because fillets and loops still use the true radius. It is a small inconsistency, kept on purpose so the archived results reproduce. It could be fixed by iterating, which would require new runs.
2. **Straight-flight power uses the site density.** The cruise power table does not change with route altitude. Only turn induced power and the turn speed floor use the local density. On routes that climb several hundred metres above the trailhead, this slightly understates power. This is documented in the code but not quantified.
3. **The archived reproduction depends on `out/wing_trade/summary.csv`.** The compact-XPS wing mass is read from that file at run time. `wing_trade.py` is being edited concurrently. If it rewrites that file, `verify_route_modes.py` will fail, even though the route code is unchanged. Freezing the mass into the archive manifest would remove this dependency (Step 1, item 11).
4. **Return-trip cost differs between models.** The flyable model's return trip uses the route airspeed and kit-on power. The sharp-corner model picks the best-range speed. Both are conservative in different ways. Neither is wrong, but they are not identical.
5. **`verify_flyable_replay.py` rewrites a file.** It writes `out/wing_trade/flyable/paired_intervals.csv` each time it runs. The bootstrap uses a fixed seed, so the content is the same. It is still a side effect inside a check.
6. **The smoke test fails on build123d** until the Application Control policy allows the OCP DLL.
7. The modeling limits in the long-pass note are unchanged: the 90° pass rule, steady wind, and no roll-rate or gust model.
