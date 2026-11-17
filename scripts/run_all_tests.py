"""Run every Lodestar test and verify script and print one pass/fail table.

Default (fast set):
  mission sim: test_terrain_sim, test_turn_path, test_long_pass, test_mission_invariants,
    verify_flyable_replay, verify_fleet_flyable (1.0 and 0.8), verify_long_pass;
  design: test_wing_construction, verify_flutter, test_fea_wing, test_flutter_v2,
    test_controls_sizing, test_propulsion_margins, freeze_check.
--full adds the replays, the tool-chain check and the CAD helper:
  verify_route_modes (re-runs one site and matches the archives run for run, a few minutes),
  smoke_test (AVL, OpenVSP, JSBSim, build123d, FreeCAD, flow5), fcad/test_fcad (FreeCAD headless).

Run from scripts/:  ../.venv/Scripts/python.exe -X utf8 run_all_tests.py [--full] [-j 4]
Exit code is nonzero if any script fails.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
FAST = [
    ("test_terrain_sim.py", "terrain profile, camera, line of sight, canopy, sharp-corner flight"),
    ("test_turn_path.py", "fillets, turnarounds, wind/bank envelope, route terrain and reserve"),
    ("test_long_pass.py", "pursuit tracker turn limit and end, simplification, pass grouping"),
    ("test_mission_invariants.py", "route modes, turn radius, energy, reserve, kit, time windows, units, seeds"),
    ("verify_flyable_replay.py", "archive: Lodestar flyable replay vs sharp-corner terrain run"),
    ("verify_fleet_flyable.py", "archive: fleet replay vs terrain run and flyable replay"),
    ("verify_fleet_flyable.py 0.8", "archive: fleet replay at 80% competitor endurance"),
    ("verify_long_pass.py", "archive: long-pass runs, held-out seed, kit invariants"),
    ("test_wing_construction.py", "Hoff wrinkling hand value, XPS caps unchanged, v1 EPS wing outputs"),
    ("verify_flutter.py", "flutter screen vs Goland wing, mode/element convergence"),
    ("test_fea_wing.py", "wing FEA known answers: cantilever beam, plate, mesh convergence"),
    ("test_flutter_v2.py", "flutter v2 known answers: Goland, Theodorsen, control-surface mode"),
    ("test_controls_sizing.py", "CG envelope, hinge moments, roll, lateral modes vs v1 mass balance"),
    ("test_propulsion_margins.py", "prop/motor/battery model vs UIUC and datasheet values, margin table"),
    ("test_sixdof_v1.py", "v1 6-DOF model: mass, CG, inertia, trim, release, glide vs drag model"),
    ("test_system_safety.py", "link budget path loss, terrain line of sight, glide reach known answers"),
    ("mass_growth.py", "mass growth allowance: line totals, CG and static margin under growth"),
    ("detection_check.py", "detection model copies match terrain_sim; calibration cases"),
    ("performance_v1_metrics.py", "v1 endurance Monte Carlo, wind penetration and dash"),
    ("freeze_check.py", "every script and output agrees with lodestar_v1.py"),
]
FULL = [
    ("verify_route_modes.py", "replay: one site through route_flight matches archives run for run"),
    ("smoke_test.py", "tool chain runs on this machine"),
    ("drop_speed_v1.py", "kit-drop error budget at 14/18/20 m/s run-in; 14 m/s reproduces the archive"),
    ("fcad/test_fcad.py", "FreeCAD helper known answers: volumes, centroids, inertia, fit checks"),
]


def counts(out):
    """(passed, failed) checks from a script's output; None if it reports no per-check counts."""
    passed, failed = len(re.findall(r"^PASS\b", out, re.M)), len(re.findall(r"^FAIL\b", out, re.M))
    ran = re.search(r"^Ran (\d+) tests?", out, re.M)
    if ran:
        bad = re.search(r"FAILED \((?:failures=(\d+))?(?:, )?(?:errors=(\d+))?", out)
        nbad = sum(int(x) for x in bad.groups() if x) if bad else 0
        return int(ran.group(1)) - nbad, nbad
    smoke = re.search(r"^(\d+) OK, (\d+) FAIL", out, re.M)
    if smoke:
        return int(smoke.group(1)), int(smoke.group(2))
    return (passed, failed) if passed or failed else None


def run(entry):
    script, _ = entry
    env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", PYTHONIOENCODING="utf-8")
    start = time.time()
    proc = subprocess.run([sys.executable, "-X", "utf8", *script.split()], cwd=HERE, env=env,
                          capture_output=True, text=True, encoding="utf-8", errors="replace")
    return proc.returncode, proc.stdout + proc.stderr, time.time() - start


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--full", action="store_true", help="also run the replays and the tool-chain smoke test")
    parser.add_argument("-j", "--jobs", type=int, default=4, help="scripts run in parallel")
    parser.add_argument("-v", "--verbose", action="store_true", help="print each script's output")
    args = parser.parse_args()
    entries = FAST + (FULL if args.full else [])
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(run, entries))

    rows, total_pass, total_fail = [], 0, 0
    for (script, what), (code, out, seconds) in zip(entries, results):
        c = counts(out)
        if c is None:                       # verify scripts: one assertion block = one check
            c = (1, 0) if code == 0 else (0, 1)
        if code != 0 and c[1] == 0:         # crashed or exited nonzero without a FAIL line
            c = (c[0], 1)
        total_pass, total_fail = total_pass + c[0], total_fail + c[1]
        rows.append((script, "PASS" if code == 0 else "FAIL", c[0], c[1], seconds, what))
        if args.verbose or code != 0:
            print(f"\n===== {script} (exit {code})\n{out.rstrip()[-4000:]}")

    width = max(len(r[0]) for r in rows)
    print(f"\n{'script':<{width}}  result  pass  fail  time_s  checks")
    for script, result, p, f, s, what in rows:
        print(f"{script:<{width}}  {result:<6}  {p:>4}  {f:>4}  {s:>6.0f}  {what}")
    bad = sum(r[1] == "FAIL" for r in rows)
    print(f"\n{len(rows) - bad}/{len(rows)} scripts passed; {total_pass} checks passed, {total_fail} failed"
          + ("" if args.full else "  (fast set; --full adds the replays)"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
