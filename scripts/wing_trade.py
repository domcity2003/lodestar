"""Compare Lodestar wing joints and cores at converged aircraft mass.

Run with uv run python -X utf8 scripts/wing_trade.py. See the engineering note
for load assumptions, packaging checks and the limits of the joint model.
"""
from dataclasses import dataclass
from functools import lru_cache
import math
import sys

import numpy as np
import pandas as pd

import structures as s
from avl_run import MassItem, cg_and_inertia
from design_v2 import N_WING, pack


OUT = s.ROOT / "out" / "wing_trade"
EPS = s.Mat("16 kg/m³ EPS", 16, E=2.5e6, G=2e6)
SPAN = s.GEO.span
STATIONS = np.linspace(0, SPAN / 2, 61)
ROLL_N = 2 / 3 * s.N_MAN_POS
CARBON_RHO = s.CARBON_UD.rho
JOINT_ALLOW = 500e6  # unverified common flexural screening value
GAUSS_X, GAUSS_W = np.polynomial.legendre.leggauss(8)


@dataclass(frozen=True)
class Joint:
    outer: float
    inner: float
    sleeve: float
    engagement: float     # axial per panel
    allowance: float      # kg per break for local blocks, wraps, key, retention and adhesive

    @property
    def section_modulus(self):
        return math.pi * (self.outer**4 - self.inner**4) / (32 * self.outer)

    def masses(self):
        tube_area = math.pi / 4 * (self.outer**2 - self.inner**2)
        sleeve_area = math.pi / 4 * (self.sleeve**2 - self.outer**2)
        mass = 2 * self.engagement * (tube_area + sleeve_area) * CARBON_RHO + self.allowance
        # Each half is integrated independently so the break never bisects a mass lump.
        for side in (-1, 1):
            for node, weight in zip(GAUSS_X, GAUSS_W):
                y = s.Y_BREAK + side * self.engagement * math.cos(s.SWEEP) * (node + 1) / 2
                yield s.GEO.x_le(y) + s.X_SPAR * s.GEO.chord(y), y, mass * weight / 4


LEGACY = Joint(0.016, 0.013, 0.016, 0.30 / math.cos(s.SWEEP), 0.0)
SLEEVED = Joint(0.024, 0.018, 0.028, 0.160, 0.045)
COMPACT = Joint(0.020, 0.016, 0.024, 0.160, 0.045)


def mixed_core(y):
    # Dense core at joint, servo and winglet attachment zones.
    return EPS if y >= 0.75 and abs(y - s.Y_SERVO) >= 0.05 and y <= 1.40 else s.XPS


def eps20_core(y):
    # EPS 20 everywhere except foam hardpoints: XPS around the joint sleeves, the servo bay and the winglet root.
    hard = abs(y - s.Y_BREAK) <= 0.20 or abs(y - s.Y_SERVO) < 0.05 or y > 1.40
    return s.XPS if hard else s.EPS20


def eps20_light_core(y):
    # EPS 20 through the joint region too. The sleeves' local support blocks, wraps and key are carried in the
    # joint allowance (45 g per break), so only the servo bay and the winglet root keep dense foam.
    return s.XPS if abs(y - s.Y_SERVO) < 0.05 or y > 1.40 else s.EPS20


def mass_properties(cad, tab, joint_masses, loaded, cell):
    """Replace the CAD's budget wing with spanwise masses and the actual joint allowance."""
    items = []
    battery_mass = 2.4205 if cell == "P50B" else pack("Amprius SiCore 6.3 Ah", N_WING)[1]
    for row in cad.itertuples():
        if row.item.startswith("wing structure") or (not loaded and row.item in ("rescue kit", "kit parachute")):
            continue
        if row.item.startswith("batteries"):
            for side in (-1, 1):
                for y in np.linspace(*s.Y_BATT, 5):
                    items.append(MassItem(row.item, battery_mass / 10, row.x, side * y, row.z))
        elif row.item.startswith("elevon mass balance"):
            import lodestar_v1 as v1
            for side in (-1, 1):
                items.append(MassItem(row.item, row.mass_kg / 2, row.x, side * v1.ADDED_MASS["elevon_balance_y"], row.z))
        elif row.item == "winglets":
            for side in (-1, 1):
                items.append(MassItem("winglet", row.mass_kg / 2, row.x, side * SPAN / 2, row.z))
        else:
            items.append(MassItem(row.item, row.mass_kg, row.x, 0, row.z))

    ys = np.linspace(0, SPAN / 2, 151)
    mid = (ys[:-1] + ys[1:]) / 2
    masses = np.interp(mid, tab.y, tab.m_per_m) * np.diff(ys)
    x_coords = np.array([s.GEO.x_le(y) for y in mid]) + (
        np.interp(mid, tab.y, tab.xcg_c) * np.array([s.GEO.chord(y) for y in mid])
    )
    for side in (-1, 1):
        for x, y, mass in zip(x_coords, mid, masses):
            items.append(MassItem("wing section", mass, x, side * y))
        for x, y, mass in joint_masses:
            items.append(MassItem("joint", mass, x, side * y))
        servo_x = s.GEO.x_le(s.Y_SERVO) + 0.6 * s.GEO.chord(s.Y_SERVO)
        items.append(MassItem("servo", s.M_SERVO, servo_x, side * s.Y_SERVO))
    return cg_and_inertia(items), battery_mass / 2


@lru_cache(maxsize=512)
def solve_strips(n, speed, rho, mass, cg, aileron=None, rate=0):
    return s.strips_at(n, speed, rho, aileron, mass=mass, x_cg=cg, roll_rate=rate)


@lru_cache(maxsize=64)
def gust_cases(mass, rho, cg):
    return s.vn_cases(mass=mass, rho=rho, x_cg=cg)[0]


def load_cases(tab, joint_masses, cad):
    def section_at(y):
        c = s.GEO.chord(y)
        return (np.interp(y, tab.y, tab.m_per_m), np.interp(y, tab.y, tab.xcg_c) * c,
                np.interp(y, tab.y, tab.xsc_c) * c)

    cases = []
    aircraft = []
    for cell in ("P50B", "SiCore"):
        for loaded in (True, False):
            mi, battery = mass_properties(cad, tab, joint_masses, loaded, cell)
            # Rounded to below the mass/CG convergence tolerance to reuse identical aero solves.
            mass, cg = round(mi["m"], 3), round(mi["x"], 5)
            _, level, _ = solve_strips(1.0, s.VC, s.RHO0, mass, cg)
            aircraft.append(dict(cell=cell, kit=loaded, mass_kg=mi["m"], cg_m=mi["x"], Ixx_kg_m2=mi["Ixx"],
                                 level_static_margin_pct_mac=100 * (level["Xnp"] - mi["x"]) / s.GEO.mac))
            for altitude, rho in ((0, s.RHO0), (4400, s.RHO_HI)):
                vd = s.VD * math.sqrt(s.RHO0 / rho)
                prefix = f"{cell}, {altitude} m, {'kit on' if loaded else 'kit off'}"
                specs = [(r.case, r.n_limit, r.V, None, 0, False)
                         for r in gust_cases(mass, rho, cg).itertuples()]
                specs.extend((r.case + " uncapped", r.n_formula, r.V, None, 0, True)
                             for r in gust_cases(mass, rho, cg).itertuples() if r.n_limit != r.n_formula)
                # One command direction covers both roll directions on the symmetric aircraft.
                # Negative rate with a positive roll command represents a reversal.
                specs.extend((f"roll {rate:+.2f}", ROLL_N, vd, -15.0, rate, False)
                             for rate in (-0.10, -0.05, 0, 0.05, 0.10))
                specs.append(("fixed aileron, no angular relief", ROLL_N, vd, -15.0, 0, True))
                for label, n, speed, aileron, rate, sensitivity in specs:
                    strips, trim, q = solve_strips(n, speed, rho, mass, cg, aileron, rate)
                    angular = 0.0
                    if aileron is not None and not sensitivity:
                        # Check the stability/geometry convention with integrated strip lift.
                        # Winglet side force can contribute to total roll moment, so compare sign only.
                        strip_moment = (strips.y * strips.L).sum()
                        if abs(trim["Cltot"]) > 0.005 and strip_moment * trim["Cltot"] >= 0:
                            raise ValueError(f"Inconsistent roll sign at {prefix}, {label}.")
                        angular = q * s.S * SPAN * trim["Cltot"] / mi["Ixx"]
                    for side in ("R", "L") if aileron is not None else ("R",):
                        load = s.spar_loads(strips, n, section_at, STATIONS, side,
                                           roll_accel=angular, extra_masses=joint_masses,
                                           battery_mass_side=battery)
                        cases.append(load.assign(case=f"{prefix}, {label}, {side}", sensitivity=sensitivity,
                                                 roll_accel_rad_s2=angular, elevon_deg=trim["elevon"]))
    return pd.concat(cases, ignore_index=True), pd.DataFrame(aircraft)


def envelope(loads):
    g = loads.loc[~loads.sensitivity].groupby("y")
    return pd.DataFrame(dict(y=STATIONS, Mb_pos=s.FOS * g.Mb.max().clip(lower=0).to_numpy(),
                             Mb_neg=s.FOS * g.Mb.min().clip(upper=0).to_numpy(),
                             V_max=s.FOS * g.V.agg(lambda x: x.abs().max()).to_numpy(),
                             T_max=s.FOS * g.T.agg(lambda x: x.abs().max()).to_numpy()))


def joint_checks(loads, joint):
    cuts = loads.loc[np.isclose(loads.y, s.Y_BREAK)].copy()
    cuts["tube_bending_MPa"] = s.FOS * cuts.Mb.abs() / joint.section_modulus / 1e6
    cuts["tube_bending_utilization"] = cuts.tube_bending_MPa * 1e6 / JOINT_ALLOW
    # Peak bound for a linear radial line-load distribution on one sleeve.
    # A finite contact region or clearance can concentrate loads beyond this distribution.
    cuts["peak_sleeve_line_load_N_mm"] = s.FOS * (
        4 * cuts.V.abs() / joint.engagement + 6 * cuts.Mb.abs() / joint.engagement**2
    ) / 1e3
    if joint.sleeve == joint.outer:
        cuts["peak_sleeve_line_load_N_mm"] = np.nan
    # Torsion is assigned to a separate aft key rather than credited to tube friction.
    arm = (s.X_REAR - s.X_SPAR) * s.GEO.chord(s.Y_BREAK) * math.cos(s.SWEEP)
    cuts["aft_key_force_N"] = s.FOS * cuts.T_spar.abs() / arm
    return cuts


def check_packaging():
    """Check tube/sleeve envelopes against the existing ruled wing lofts and cells."""
    from build123d import Compound, Cylinder, Location, Plane, Shell, Unit, Vector, export_step, loft
    import cad_model as cad

    yb = s.Y_BREAK
    panels = {
        "inboard": loft([cad.wing_section(0), cad.wing_section(yb)], ruled=True),
        "outboard": loft([cad.wing_section(yb), cad.wing_section(SPAN / 2)], ruled=True),
    }
    cells = []
    yc = cad.Y_BATT
    zc, _ = cad.camber_z((cad.X_BATT - cad.G.x_le(yc)) / cad.G.chord(yc), cad.G.chord(yc))
    for i in range(-2, 3):
        for j in range(-1, 2):
            cells.append(cad.cell(cad.X_BATT + i * 0.0215, yc + j * 0.071, zc))

    def axis_point(y):
        c = cad.G.chord(y)
        return Vector(cad.G.x_le(y) + s.X_SPAR * c, y, cad.camber_z(s.X_SPAR, c)[0])

    axis = (axis_point(yb + 0.30) - axis_point(yb - 0.30)).normalized()
    centre = (axis_point(yb + 0.30) + axis_point(yb - 0.30)) * 0.5
    rows = []
    for joint in (SLEEVED, COMPACT):
        tube_loc = Location(Plane(origin=centre, z_dir=axis))
        tube = tube_loc * (Cylinder(joint.outer / 2, 2 * joint.engagement)
                           - Cylinder(joint.inner / 2, 2 * joint.engagement))
        tube.label = "Joiner tube"
        parts = [tube]
        for side, panel in panels.items():
            direction = 1 if side == "outboard" else -1
            loc = Location(Plane(origin=centre + axis * direction * joint.engagement / 2, z_dir=axis))
            envelope = loc * Cylinder(joint.sleeve / 2, joint.engagement)
            # Cut faces are panel interfaces, through which the tube is meant to pass.
            skin = Shell([face for face in panel.faces() if face.bounding_box().size.Y > 1e-6])
            skin_gap = envelope.distance_to(skin)
            cell_gap = min(envelope.distance_to(cell) for cell in cells)
            if skin_gap < cad.SKIN or cell_gap <= 0:
                raise ValueError(f"{joint} fails packaging clearance on the {side} side.")
            sleeve = envelope - loc * Cylinder(joint.outer / 2, joint.engagement)
            sleeve.label = f"{side.title()} sleeve"
            parts.append(sleeve)
            rows.append(dict(tube_od_mm=1000 * joint.outer, sleeve_od_mm=1000 * joint.sleeve, panel=side,
                             engagement_mm=1000 * joint.engagement, skin_gap_mm=1000 * skin_gap,
                             cell_gap_mm=1000 * cell_gap, gap_after_skin_allowance_mm=1000 * (skin_gap - cad.SKIN)))
        export_step(Compound(children=parts, label="Nominal right wing joint"),
                    OUT / f"joint_{round(1000 * joint.outer)}mm_nominal.step", unit=Unit.M)
    pd.DataFrame(rows).to_csv(OUT / "packaging.csv", index=False)


def solve_design(name, joint, core_at, cad, lay=None):
    print(f"Solving {name}", flush=True)
    masses = tuple(joint.masses())
    zero = np.zeros_like(STATIONS)
    env = pd.DataFrame(dict(y=STATIONS, Mb_pos=zero, Mb_neg=zero, V_max=zero, T_max=zero))
    lay = lay or s.LAYUPS[0]
    tab = s.size_wing(lay, env, STATIONS, core_at=core_at)
    for _ in range(12):
        loads, aircraft = load_cases(tab, masses, cad)
        env = envelope(loads)
        sized = s.size_wing(lay, env, STATIONS, core_at=core_at)
        if sized.t_cap_mm.equals(tab.t_cap_mm):
            break
        tab = sized
    else:
        raise RuntimeError(f"Mass/load iteration failed for {name}.")
    tab = sized
    joint_cases = joint_checks(loads, joint)
    joint_cases.to_csv(OUT / f"{name}_joint.csv", index=False)
    loads.to_csv(OUT / f"{name}_loads.csv", index=False)
    tab.merge(env, on="y").to_csv(OUT / f"{name}_sections.csv", index=False)
    deflections = [abs(s.tip_deflection(tab, case.Mb)[-1])
                   for _, case in loads.loc[~loads.sensitivity].groupby("case")]
    beam = s.wing_beam(tab, extra_masses=masses)
    flutter0, flutter_hi = [s.flutter(*beam, rho)[2] for rho in (s.RHO0, s.RHO_HI)]
    wing_mass = s.wing_mass(tab) + 2 * sum(m for _, _, m in masses)
    included = joint_cases.loc[~joint_cases.sensitivity]
    return dict(
        design=name, wing_mass_kg=wing_mass, joints_kg=2 * sum(m for _, _, m in masses),
        max_tip_limit_mm=1000 * max(deflections),
        max_tube_bending_MPa=included.tube_bending_MPa.max(),
        tube_margin_at_assumed_allowable=JOINT_ALLOW / (1e6 * included.tube_bending_MPa.max()) - 1,
        governing_tube_case=included.loc[included.tube_bending_MPa.idxmax(), "case"],
        max_tube_sensitivity_MPa=joint_cases.loc[joint_cases.sensitivity, "tube_bending_MPa"].max(),
        peak_sleeve_line_load_N_mm=included.peak_sleeve_line_load_N_mm.max() if joint.sleeve > joint.outer else np.nan,
        max_aft_key_force_N=included.aft_key_force_N.max(),
        max_skin_wrinkle_utilization=(tab.skin_principal_MPa / tab.skin_wrinkle_MPa).max(),
        cap_mass_kg=2 * np.trapezoid(2 * lay.cap_w * tab.t_cap_mm / 1e3 * s.CARBON_UD.rho, tab.y),
        max_skin_shear_utilization=(tab.tau_skin_MPa / tab.skin_allow_MPa).max(),
        max_web_shear_utilization=(tab.q_web_Nmm / tab.q_web_allow_Nmm).max(),
        flutter_sea_level_tas=flutter0,
        flutter_4400m_tas=flutter_hi,
        flutter_4400m_eas=flutter_hi * math.sqrt(s.RHO_HI / s.RHO0),
    ), aircraft.assign(design=name)


DESIGNS = {
    "legacy_xps": (LEGACY, None, None), "sleeved_xps": (SLEEVED, None, None),
    "sleeved_mixed_core": (SLEEVED, mixed_core, None), "compact_xps": (COMPACT, None, None),
    "compact_mixed_core": (COMPACT, mixed_core, None),
    "compact_eps20": (COMPACT, eps20_core, None),
    "compact_eps20_carbon": (COMPACT, eps20_core, 2),
    "compact_eps20_light": (COMPACT, eps20_light_core, None),
    # v1 freeze: same wing, solved with the CAD v4 layout (packs 25 mm forward and 25 mm inboard).
    "v1": (COMPACT, eps20_light_core, None),
}


def main(names=None, tag=""):
    OUT.mkdir(parents=True, exist_ok=True)
    summaries, aircraft, perf = [], [], []
    for name in names or list(DESIGNS)[:5]:
        joint, core_at, lay = DESIGNS[name]
        cad = pd.read_csv(s.ROOT / "out" / ("cad_v4" if name == "v1" else "cad_v3") / "mass_cg.csv")
        extra = 0.0
        if name == "v1":
            # Pack position from the frozen layout (CAD v4): y_batt +- 0.071 m span, cell axes spanwise.
            import lodestar_v1 as v1
            yb = v1.LAYOUT["y_batt"]
            s.Y_BATT, s.X_BATT_GLOBAL = (yb - 0.071, yb + 0.071), v1.LAYOUT["x_batt"]
            s.UPPER_DOUBLER_Y = v1.STRUCTURE["upper_skin_doubler_y"]
            s.UPPER_DOUBLER_TAPER = v1.STRUCTURE["upper_skin_doubler_taper_m"]
            s.CAP_DROP_FACTOR = v1.STRUCTURE.get("cap_ply_drop_factor", 1.0)
            # Added masses (pack thermal kit, elevon balance) ride in the CAD table; the performance
            # model takes them as extra structure mass.
            extra = cad[cad.source.str.startswith("lodestar_v1.ADDED_MASS")].mass_kg.sum()
            if hasattr(v1, "V11_MASS"):
                # v1.1: every Step 2 mass change, including the payload and parachute lines the CAD holds as
                # budget items (the sizing model keeps its 0.60 kg payload budget, so the delta goes here).
                extra += sum(m for m, *_ in v1.V11_MASS.values())
            import v4_upgrade
            v4_upgrade.apply_v11()  # 7 m/s kit chute, 50 deg camera, v1.1 skid/strobe drag (as the mission model)
        summary, mass_cases = solve_design(name, joint, core_at, cad, None if lay is None else s.LAYUPS[lay])
        summaries.append(summary)
        aircraft.append(mass_cases)
        perf.append(s.performance_sensitivity(summary["wing_mass_kg"] + extra)
                    .query("mass_case != 'original budget'").drop(columns="mass_case").assign(design=name))
        print(pd.Series(summary).to_string(), flush=True)
    pd.DataFrame(summaries).replace(math.inf, np.nan).to_csv(OUT / f"summary{tag}.csv", index=False)
    pd.concat(aircraft).to_csv(OUT / f"aircraft_mass_balance{tag}.csv", index=False)
    pd.concat(perf).to_csv(OUT / f"performance{tag}.csv", index=False)


if __name__ == "__main__":
    if sys.argv[1:] == ["--fit-only"]:
        # cad_model reads its variant from argv at import.
        sys.argv[1:] = ["v3"]
        OUT.mkdir(parents=True, exist_ok=True)
        check_packaging()
    elif sys.argv[1:2] == ["--designs"]:
        main(sys.argv[2].split(","), tag=sys.argv[3] if len(sys.argv) > 3 else "_selected")
    else:
        main()
