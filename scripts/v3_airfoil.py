"""Design v3, lever 2: is there a better flying-wing airfoil than MH45?

A tailless wing needs a section with a near-zero (or slightly nose-up) pitching moment, so the
wing trims without a tail; that rules out ordinary cambered airfoils. Candidates are the published
reflexed and flying-wing sections in the UIUC database (Hepperle MH, Eppler, Horten-style EH,
Selig S5020, PW, HS and Drela AG/"ct" sections), screened with NeuralFoil:

  - pitching moment Cm (quarter chord) at cl 0.5 must be at least MH45's minus 0.01 (AVL re-check later)
  - thickness at least 9.5% (the wing-root cells needed MH45's 9.8% in the milestone 5 CAD)
  - drag at the root, mean and tip Reynolds numbers and cl 0.4-0.6 (search and transit)
  - 2D CLmax at the tip Reynolds number (the tips set the stall)

Run:  uv run python scripts/v3_airfoil.py
Writes: out/v3/airfoil_screen.csv, airfoil_log.txt
"""
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb

sys.path.insert(0, str(Path(__file__).parent))
from stability import Tee  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "v3"
OUT.mkdir(parents=True, exist_ok=True)

CANDIDATES = ["mh45", "mh46", "mh49", "mh60", "mh61", "mh62", "mh64", "mh70", "mh78", "mh80", "mh81",
              "e182", "e184", "e186", "e230", "e325", "e326", "e327", "e328", "e329", "e330", "e331", "e332",
              "e333", "e334", "e335", "e336", "e337", "e338", "e339", "eh1070", "eh1090", "eh1590", "eh2010",
              "eh2012", "eh3012", "PW51i", "PW75", "PW106", "PW1211", "S5020-2087", "hs510", "hs520", "hs522",
              "ag45c03", "ag46c03", "ag47c03", "ag44ct02r", "ag45ct02r", "ag46ct02r", "ag47ct02r", "ag455ct02r"]
# Reynolds numbers at 14.8 m/s, 2,000 m: root chord 0.59 m, mean chord 0.45 m, tip 0.27 m
RE = {"root": 5.1e5, "mean": 3.9e5, "tip": 2.3e5}
CLS = (0.4, 0.5, 0.6)
ALPHA = np.linspace(-4, 16, 81)


def polar(af, Re):
    a = af.get_aero_from_neuralfoil(alpha=ALPHA, Re=Re, mach=0.04, model_size="xxlarge")
    cl, cd, cm = (np.asarray(a[k]) for k in ("CL", "CD", "CM"))
    conf = np.asarray(a["analysis_confidence"])
    i = int(np.argmax(cl))
    return cl[: i + 1], cd[: i + 1], cm[: i + 1], float(cl[i]), conf[: i + 1]


if __name__ == "__main__":
    sys.stdout = Tee(OUT / "airfoil_log.txt")
    print("Lodestar v3, airfoil screen, 2026-11-12, claude-opus-5-5 (NeuralFoil xxlarge)\n")
    rows = []
    for name in CANDIDATES:
        try:
            af = asb.Airfoil(name).repanel(n_points_per_side=120)
        except Exception as e:  # noqa: BLE001
            print(f"  skip {name}: {e}")
            continue
        r = dict(airfoil=name, t_c=float(af.max_thickness()))
        for k, Re in RE.items():
            cl, cd, cm, clmax, conf = polar(af, Re)
            for c in CLS:
                if c < clmax - 0.05 and np.all(np.diff(cl) > -1e-3):
                    r[f"cd_{k}_{c}"] = float(np.interp(c, cl, cd))
                else:
                    r[f"cd_{k}_{c}"] = np.nan
            r[f"clmax_{k}"] = clmax
            r[f"cm_{k}"] = float(np.interp(0.5, cl, cm))
            r[f"conf_{k}"] = float(np.min(conf[cl > 0.3])) if np.any(cl > 0.3) else np.nan
        # mission-weighted drag: search (cl 0.5) at all three stations, chord-weighted, plus transit cl 0.4
        w = {"root": 0.25, "mean": 0.5, "tip": 0.25}
        r["cd_search"] = sum(w[k] * r[f"cd_{k}_0.5"] for k in w)
        r["cd_transit"] = sum(w[k] * r[f"cd_{k}_0.4"] for k in w)
        r["cd_slow"] = sum(w[k] * r[f"cd_{k}_0.6"] for k in w)
        rows.append(r)
    df = pd.DataFrame(rows)
    ref = df[df.airfoil == "mh45"].iloc[0]
    df["cm_ok"] = df.cm_mean >= ref.cm_mean - 0.01
    df["thick_ok"] = df.t_c >= 0.095
    df["d_cd_search_pct"] = (df.cd_search / ref.cd_search - 1) * 100
    df["d_clmax_tip"] = df.clmax_tip - ref.clmax_tip
    df.round(5).to_csv(OUT / "airfoil_screen.csv", index=False)
    cols = ["airfoil", "t_c", "cm_mean", "cd_search", "cd_transit", "cd_slow", "d_cd_search_pct", "clmax_tip",
            "conf_mean", "cm_ok", "thick_ok"]
    print(f"MH45 reference: t/c {ref.t_c:.3f}, Cm {ref.cm_mean:+.4f}, search cd {ref.cd_search:.5f}, tip CLmax {ref.clmax_tip:.2f}\n")
    print("All candidates, sorted by search drag:")
    with pd.option_context("display.width", 200, "display.max_rows", 100):
        print(df.sort_values("cd_search")[cols].round(4).to_string(index=False))
    ok = df[df.cm_ok & df.thick_ok].sort_values("cd_search")
    print("\nPass the pitching-moment and thickness screens, best first:")
    print(ok[cols].round(4).head(10).to_string(index=False))
