"""Kit-drop accuracy at the v1 run-in speeds (Step 2, 6-DOF finding 1).

The drop-accuracy round (drop_accuracy.py, 2026-11-12) used a 14 m/s run-in. The v1 6-DOF study
(research/2026-11-15-sixdof-v1.md) found 14 m/s is only 1.11 x stall for the 8.3 kg aircraft and stalls in
light turbulence, so v1.1 flies the run-in at 18 m/s (calm or light air) and 20 m/s (moderate turbulence).
This reruns the same error budget at 14, 18 and 20 m/s without touching drop_accuracy.py's archived outputs.

Run:    .venv/Scripts/python.exe -X utf8 scripts/drop_speed_v1.py
Writes: out/drop_v1/drop_sweep_va<speed>.csv, summary.csv
"""
from pathlib import Path

import pandas as pd

import drop_accuracy as da

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "drop_v1"
SPEEDS = (14.0, 18.0, 20.0)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for va in SPEEDS:
        da.VA_RELEASE = va
        tab = da.sweep()
        tab.round(4).to_csv(OUT / f"drop_sweep_va{va:g}.csv", index=False)
        avg = tab.groupby("config", sort=False)[["R50", "R95", "p_reach"]].mean()
        for config, r in avg.iterrows():
            rows.append(dict(va_release_mps=va, config=config, **r.to_dict()))
        print(f"run-in {va:g} m/s\n{avg.round(3).to_string()}\n", flush=True)
    summary = pd.DataFrame(rows)
    summary.to_csv(OUT / "summary.csv", index=False)
    base = pd.read_csv(ROOT / "out" / "drop" / "drop_sweep.csv")
    again = pd.read_csv(OUT / "drop_sweep_va14.csv")
    same = (base.round(3).equals(again.round(3)))
    print(f"{'PASS' if same else 'FAIL'}  14 m/s rerun reproduces out/drop/drop_sweep.csv")
    raise SystemExit(0 if same else 1)


if __name__ == "__main__":
    main()
