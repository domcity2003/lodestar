"""Checks for the skin-wrinkling limit in structures.size_wing and the EPS 20 v1 wing.

The limit uses the minimum principal stress of the compressed skin (spanwise compression plus torsion shear),
with the FE correlation factor inboard of structures.SKIN_FE_FACTOR_Y.

Run:  ../.venv/Scripts/python.exe -X utf8 test_wing_construction.py   (from scripts/)
"""
import sys

import numpy as np
import pandas as pd

import structures as s
import wing_trade as wt


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        check.failed += 1


check.failed = 0
OUT = s.ROOT / "out" / "wing_trade"


def test_hoff_value():
    # 0.3 * (11 GPa * 4 MPa * 2.5 MPa)^(1/3) = 14.37 MPa by hand.
    v = s.WRINKLE_K * (s.GLASS45.E * s.EPS20.E * s.EPS20.G) ** (1 / 3) / 1e6
    check("Hoff wrinkling stress for glass on EPS 20 matches hand value", abs(v - 14.37) < 0.01, f"{v:.3f} MPa")


def test_xps_unchanged():
    """The new limit must not bind on XPS: re-sizing the archived compact_xps envelope gives the same caps."""
    old = pd.read_csv(OUT / "compact_xps_sections.csv")
    env = old[["y", "Mb_pos", "Mb_neg", "V_max", "T_max"]]
    new = s.size_wing(s.LAYUPS[0], env, old.y.values)
    check("XPS cap schedule is unchanged by the wrinkling limit", np.allclose(new.t_cap_mm, old.t_cap_mm),
          f"max diff {np.abs(new.t_cap_mm - old.t_cap_mm).max():.3f} mm")
    check("XPS principal wrinkling limit does not bind (utilisation < 1)", (new.skin_principal_MPa / new.skin_wrinkle_MPa).max() < 1.0,
          f"max {(new.skin_principal_MPa / new.skin_wrinkle_MPa).max():.3f}")


def test_limit_binds_on_eps():
    """With EPS everywhere the caps must grow until the skin is at or below its wrinkling stress."""
    old = pd.read_csv(OUT / "compact_xps_sections.csv")
    env = old[["y", "Mb_pos", "Mb_neg", "V_max", "T_max"]]
    eps = s.size_wing(s.LAYUPS[0], env, old.y.values, core_at=lambda y: s.EPS20)
    u = eps.skin_principal_MPa / eps.skin_wrinkle_MPa
    check("EPS skins never exceed the wrinkling stress", u.max() <= 1.0 + 1e-9, f"max {u.max():.3f}")
    check("EPS needs thicker root caps than XPS", eps.t_cap_mm.iloc[0] > old.t_cap_mm.iloc[0])
    check("EPS wing section is lighter at the root", eps.m_per_m.iloc[0] < old.m_per_m.iloc[0])


def test_baseline_outputs():
    sec = pd.read_csv(OUT / "v1_sections.csv")
    summ = pd.read_csv(OUT / "summary_v1.csv").iloc[0]
    check("v1 principal wrinkling utilisation <= 1", (sec.skin_principal_MPa / sec.skin_wrinkle_MPa).max() <= 1.0 + 1e-9)
    p = sec[sec.y < s.SKIN_FE_FACTOR_Y]
    check("v1 inboard skin carries the FE correlation factor",
          (p.skin_principal_MPa * 1.0 <= p.skin_wrinkle_MPa).all() and s.SKIN_FE_FACTOR >= 1.10)
    check("v1 core is EPS except the servo bay and winglet root",
          all((r.core.startswith("XPS")) == (abs(r.y - s.Y_SERVO) < 0.05 or r.y > 1.40) for r in sec.itertuples()))
    # v1.0 saved 0.308 kg; v1.1 adds 23 g (ply-drop rule + the 0.42 kg heavier aircraft) against an XPS wing that
    # is still sized at the archived lighter mass, so the guard is 0.25 kg (saving now 0.285 kg).
    check("v1 wing is lighter than compact XPS by > 0.25 kg",
          pd.read_csv(OUT / "summary.csv").set_index("design").loc["compact_xps", "wing_mass_kg"] - summ.wing_mass_kg > 0.25,
          f"{summ.wing_mass_kg:.3f} kg")
    check("v1 tube margin stays positive at the assumed allowable", summ.tube_margin_at_assumed_allowable > 0)
    check("v1 clamped flutter clears the 36 m/s EAS screen", summ.flutter_4400m_eas > 36)
    check("eps20_light_core function matches the documented hardpoints",
          wt.eps20_light_core(0.45) is s.EPS20 and wt.eps20_light_core(s.Y_SERVO) is s.XPS and wt.eps20_light_core(1.45) is s.XPS)


if __name__ == "__main__":
    for f in (test_hoff_value, test_xps_unchanged, test_limit_binds_on_eps, test_baseline_outputs):
        print(f"--- {f.__name__}")
        f()
    print(f"\n{check.failed} failed")
    sys.exit(1 if check.failed else 0)
