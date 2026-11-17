"""Screen the two compact cores at the current terrain replay configuration."""
import math

import aerosandbox as asb
import pandas as pd
from scipy.optimize import brentq

import terrain_sim as terrain


OUT = terrain.ROOT / "out" / "wing_trade"
DESIGNS = ("compact_xps", "compact_mixed_core")
ALTITUDES = (2000, 4400)
PATTERNS = ("trails 3", "trails 6")
HIKERS = 500


def screen(aircraft, design, wing_mass, altitude):
    assumptions = terrain.ms.sz.A
    rho, area = aircraft.rw_cfg["rho"], aircraft.rw_cfg["S"]
    weight = aircraft.mass * terrain.ms.G
    stall = aircraft.v_min / assumptions.stall_margin
    bank = terrain.ms.BANK
    bank_factor = 1.0 / math.sqrt(math.cos(bank))
    speed = aircraft.v_cruise
    stall_target = assumptions.v_stall_max
    max_mass = 0.5 * rho * area * assumptions.cl_max_3d * stall_target**2 / terrain.ms.G
    warm_hours = aircraft.energy * (1 - terrain.ms.RESERVE) / float(aircraft.power(speed))
    # Bank angle at which the airspeed still preserves the project's stall margin.
    max_bank = math.acos((aircraft.v_min / speed)**2) if speed >= aircraft.v_min else math.nan
    turn_speed = aircraft.v_min * bank_factor
    turn_density_limit = rho * (turn_speed / speed)**2
    turn_ceiling = brentq(lambda h: float(asb.Atmosphere(altitude=h).density()) - turn_density_limit,
                          -1000, 10000)
    return dict(
        design=design, aircraft=aircraft.name, altitude_m=altitude, wing_mass_kg=wing_mass,
        loaded_mass_kg=aircraft.mass, clean_mass_kg=aircraft.mass - aircraft.kit_mass,
        stall_loaded_tas_mps=stall, stall_clean_tas_mps=stall * math.sqrt(1 - aircraft.kit_mass / aircraft.mass),
        max_mass_for_stall_target_kg=max_mass,
        mass_reduction_for_stall_target_kg=max(0.0, aircraft.mass - max_mass),
        cl_max_for_stall_target=2 * weight / (rho * area * stall_target**2),
        area_for_stall_target_m2=area * (stall / stall_target)**2,
        search_tas_mps=speed, turn_bank_deg=math.degrees(bank),
        turn_stall_tas_mps=stall * bank_factor,
        min_turn_tas_mps=turn_speed,
        max_bank_at_search_deg=math.degrees(max_bank),
        turn_margin_mps=speed - turn_speed,
        altitude_limit_for_turn_m=turn_ceiling,
        cl_max_for_turn_margin=assumptions.cl_max_3d * (turn_speed / speed)**2,
        warm_level_endurance_h=warm_hours,
        prewarmed_level_endurance_h=0.90 * warm_hours,
        cold_soaked_level_endurance_h=0.80 * warm_hours,
    )


def main():
    masses = pd.read_csv(OUT / "summary.csv").set_index("design").wing_mass_kg
    rows = []
    for design in DESIGNS:
        for altitude in ALTITUDES:
            fleet = terrain.site_fleet(altitude, wing_mass_kg=masses[design], include_competitors=False)
            rows.extend(screen(ac, design, masses[design], altitude) for ac in fleet)
    results = pd.DataFrame(rows)
    results.to_csv(OUT / "operating_limits.csv", index=False)
    columns = ["design", "aircraft", "altitude_m", "stall_loaded_tas_mps", "min_turn_tas_mps",
               "turn_margin_mps", "warm_level_endurance_h"]
    print(results[columns].round(3).to_string(index=False))

    # The replay charges turns between consecutive nontrivial legs. Sample the path
    # altitude at each junction. The route screen includes legs beyond the energy cutoff.
    turns = []
    spacing = terrain.CAMERAS["Lodestar"][1]
    reference_rows = results[results.altitude_m == ALTITUDES[0]]
    reference_density = float(asb.Atmosphere(altitude=ALTITUDES[0]).density())
    for site in terrain.SITES:
        prep = terrain.prep(site, HIKERS, terrain.ms.RNG_SEED)
        for pattern in PATTERNS:
            path = prep["pats"][(pattern, spacing)]
            headings = []
            altitudes = []
            for (start, end), profile in zip(path["legs"][:terrain.sp.MAX_LEGS], path["prof"]):
                if math.dist(start, end) >= 1.0:
                    headings.append(terrain.sp.heading(start, end))
                    altitudes.append(profile[1][0])
            actual_turns = [alt for h0, h1, alt in zip(headings, headings[1:], altitudes[1:])
                            if abs(math.remainder(h1 - h0, math.tau)) > 1e-6]
            highest_altitude = max(actual_turns)
            turn_density = float(asb.Atmosphere(altitude=highest_altitude).density())
            for row in reference_rows.itertuples():
                turns.append(dict(site=site, pattern=pattern, design=row.design, aircraft=row.aircraft,
                                  planned_turns=len(actual_turns), highest_turn_m=highest_altitude,
                                  min_tas_at_highest_turn_mps=row.min_turn_tas_mps
                                  * math.sqrt(reference_density / turn_density),
                                  above_margin_ceiling=sum(alt > row.altitude_limit_for_turn_m
                                                           for alt in actual_turns)))
    pd.DataFrame(turns).to_csv(OUT / "route_turn_screen.csv", index=False)


if __name__ == "__main__":
    main()
