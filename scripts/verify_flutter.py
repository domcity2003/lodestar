"""Verify the fixed-root flutter screen against Goland and a refined compact-XPS sweep."""
import math

import pandas as pd

import structures as s
import wing_trade as wt


def main():
    _, _, speed, frequency = s.goland()
    if abs(speed / 137.1 - 1) > 0.02 or abs(frequency / 69.12 - 1) > 0.03:
        raise AssertionError(f"Goland reference mismatch: {speed:.2f} m/s, {frequency:.2f} rad/s")
    print(f"Goland: {speed:.2f} m/s, {frequency:.2f} rad/s")

    sections = pd.read_csv(wt.OUT / "compact_xps_sections.csv")
    rows = []
    for altitude, density in ((0, s.RHO0), (4400, s.RHO_HI)):
        screen_tas = 1.2 * s.VD * math.sqrt(s.RHO0 / density)
        for elements, modes, kmax in ((30, 6, 3), (60, 8, 3), (60, 10, 24)):
            beam = s.wing_beam(sections, ne=elements, extra_masses=tuple(wt.COMPACT.masses()))
            _, branches, speed, frequency = s.flutter(*beam, density, n_modes=modes, kmax=kmax)
            initial = branches.loc[branches.step == 0]
            stable_through_screen = 0
            for mode in initial["mode"]:
                trace = branches.loc[branches["mode"] == mode].sort_values("step")
                first_above = trace.loc[trace.V >= screen_tas, "step"]
                if not first_above.empty:
                    end = int(first_above.iloc[0])
                    prefix = trace.loc[trace.step <= end]
                    stable_through_screen += int(len(prefix) == end + 1 and (prefix.g < 0).all())
            rows.append(dict(altitude_m=altitude, elements=elements, modes=modes, kmax=kmax,
                             screen_tas_mps=screen_tas, first_crossing_tas_mps=speed,
                             first_crossing_rad_s=frequency, initial_branches=len(initial),
                             initial_max_tas_mps=initial.V.max(), initial_max_g=initial.g.max(),
                             branches_stable_through_screen=stable_through_screen))
        coarse, fine = rows[-3], rows[-1]
        if not (fine["initial_branches"] == fine["modes"]
                and fine["initial_max_tas_mps"] < screen_tas
                and fine["branches_stable_through_screen"] == fine["modes"]):
            raise AssertionError(f"Incomplete initial mode coverage at {altitude} m: {fine}")
        if not (fine["first_crossing_tas_mps"] > screen_tas
                and abs(fine["first_crossing_tas_mps"] / coarse["first_crossing_tas_mps"] - 1) < 0.02):
            raise AssertionError(f"Flutter crossing outside the expected range at {altitude} m: {fine}")
    table = pd.DataFrame(rows)
    table.to_csv(wt.OUT / "flutter_verification.csv", index=False)
    print(table.round(3).to_string(index=False))


if __name__ == "__main__":
    main()
