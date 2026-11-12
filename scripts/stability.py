"""Milestone 2: stability and trim of the Lodestar, with the kit on and after it drops.

Questions:
  1. Where must the balance point (CG) sit, and where do the heavy parts go to put it there?
  2. How much washout (tip twist) and how tall should the winglets be?
  3. Is it stable and trimmable from stall to top speed, both with the 1.7 kg kit and without it?
  4. Do the flight modes (short period, phugoid, Dutch roll, roll, spiral) meet handling criteria?
  5. Was the sizing's 3D CLmax of 0.80 right?

Tools: AVL 3.52 (vortex lattice: trim, derivatives, strip loads, eigenmodes), checked against
AeroSandbox's VLM for the neutral point and textbook approximations for the modes.

Run:  uv run python scripts/stability.py
Writes: out/stability/*.csv, *.png, run_log.txt, final AVL files in out/stability/avl/
"""
from dataclasses import replace
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
from avl_run import Geometry, MassItem, wing_mass_items, cg_and_inertia, run_trim  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "stability"
OUT.mkdir(parents=True, exist_ok=True)
G = 9.81


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()



# ---------------------------------------------------------------- design point from milestone 1
DP = pd.read_csv(ROOT / "out" / "sizing" / "design_point.csv", index_col=0)["value"]
ATMO = asb.Atmosphere(altitude=2000.0)
RHO = float(ATMO.density())
MU = float(ATMO.dynamic_viscosity())
V_SEARCH = float(DP["v_search"])
CL_SEARCH = float(DP["CL_search"])
LD_SIZING = float(DP["LD_search"])
E_SIZING, AR = 0.80, float(DP["AR"])
CD_NONINDUCED = CL_SEARCH / LD_SIZING - CL_SEARCH**2 / (math.pi * E_SIZING * AR)  # profile + body, from sizing
BATT_WH, USABLE, ETA, P_AV = float(DP["batt_wh"]), 0.85 * 0.90, 0.55, 25.0

M_WING = float(DP["m_wing"])
M_BATT = float(DP["m_batt"])
M_KIT = 1.50 + 0.204          # kit + parachute/release from kit_drop.py
M_MISC = float(DP["mtow"]) * 0.05

# Handling criteria: MIL-F-8785C, Class I (small aircraft), Category B (cruise, search), Level 1.
# Written for piloted aircraft; an autopilot can fly worse airframes, but meeting them means
# the autopilot has an easy job and a manual takeover is safe.
CRIT = dict(sp_zeta=(0.30, 2.0), ph_zeta_min=0.04, dr_zeta_min=0.08, dr_zw_min=0.15, dr_wn_min=0.4,
            roll_tau_max=1.4, spiral_t2_min=20.0)


def mass_items(g: Geometry, x_batt: float, x_kit: float, kit_on: bool = True, winglet_kg_per_m2=1.0):
    wl_area = g.winglet_h * g.chord(g.span / 2) * (1 + g.winglet_taper) / 2
    items = wing_mass_items(g, M_WING)
    if g.winglet_h > 0:
        xt = g.x_le(g.span / 2) + 0.4 * g.chord(g.span / 2)
        for s in (1, -1):
            items.append(MassItem("winglet", wl_area * winglet_kg_per_m2, xt + 0.05, s * g.span / 2, g.winglet_h / 2))
    items += [
        MassItem("gimbal (thermal + RGB)", 0.60, 0.10, 0, -0.08),   # belly, front of the pod
        MassItem("avionics", 0.35, 0.20, 0, -0.02),
        MassItem("centre pod structure", 0.30, 0.30, 0, -0.03),
        MassItem("misc (wiring, glue)", M_MISC, 0.35),
        MassItem("motor + prop (pusher)", float(DP["m_prop"]), float(DP["c_root"]) + 0.05),
        MassItem("battery L", M_BATT / 2, x_batt, 0.20, 0),          # Li-ion packs in the wing roots
        MassItem("battery R", M_BATT / 2, x_batt, -0.20, 0),
    ]
    if kit_on:
        items.append(MassItem("rescue kit + chute", M_KIT, x_kit, 0, -0.06))
    return items


def place_for_cg(g: Geometry, x_cg_target: float, kit_offset: float = 0.0):
    """Put the kit at the target CG (plus an optional offset) and solve for the battery x
    that lands the loaded CG on the target. Kit at the CG means dropping it barely moves the CG."""
    x_kit = x_cg_target + kit_offset
    others = mass_items(g, 0.0, x_kit)
    m_other = sum(i.m for i in others if not i.name.startswith("battery"))
    mx_other = sum(i.m * i.x for i in others if not i.name.startswith("battery"))
    x_batt = (x_cg_target * (m_other + M_BATT) - mx_other) / M_BATT
    return x_batt, x_kit


def classify_modes(eig):
    """AVL returns 8 roots: short period, phugoid, Dutch roll (3 complex pairs), roll and spiral (2 real).
    Sort complex pairs by natural frequency: highest = short period, lowest = phugoid, middle = Dutch roll."""
    eig = [e for e in eig if abs(e) > 1e-6]
    pairs = sorted([e for e in eig if e.imag > 1e-6], key=abs)
    reals = sorted([e.real for e in eig if abs(e.imag) <= 1e-6])
    out = {}
    if len(pairs) == 3 and len(reals) == 2:
        for name, e in zip(("phugoid", "dutch_roll", "short_period"), pairs):
            wn = abs(e)
            out[f"{name}_wn"] = wn
            out[f"{name}_zeta"] = -e.real / wn
        out["roll_tau"] = -1 / reals[0]           # most negative real root
        s = reals[1]
        out["spiral_root"] = s
        out["spiral_t2"] = math.log(2) / s if s > 0 else float("inf")   # time to double if unstable
        out["spiral_t_half"] = math.log(2) / -s if s < 0 else float("inf")
    else:
        out["mode_note"] = f"unexpected root structure: {len(pairs)} pairs, {len(reals)} reals"
    return out


def lon_ok(m):
    """Pitch modes: short period and phugoid."""
    return ("short_period_zeta" in m and CRIT["sp_zeta"][0] <= m["short_period_zeta"] <= CRIT["sp_zeta"][1]
            and m["phugoid_zeta"] >= CRIT["ph_zeta_min"])


def lat_ok(m):
    """Lateral modes: Dutch roll, roll, spiral. These are what the winglets change."""
    if "dutch_roll_zeta" not in m:
        return False
    return (m["dutch_roll_zeta"] >= CRIT["dr_zeta_min"]
            and m["dutch_roll_zeta"] * m["dutch_roll_wn"] >= CRIT["dr_zw_min"]
            and m["dutch_roll_wn"] >= CRIT["dr_wn_min"]
            and m["roll_tau"] <= CRIT["roll_tau_max"]
            and m["spiral_t2"] >= CRIT["spiral_t2_min"])


def analyse(g, x_cg, V=V_SEARCH, kit_on=True, kit_offset=0.0, modes=True, workdir=None):
    x_batt, x_kit = place_for_cg(g, x_cg, kit_offset)
    items = mass_items(g, x_batt, x_kit, kit_on)
    mi = cg_and_inertia(items)
    r = run_trim(g, items, V, RHO, modes=modes, workdir=workdir, cdp=CD_NONINDUCED)
    if not r.ok:
        return dict(ok=False)
    st = r.st
    CL = st["CLtot"]
    cdff = st.get("CDff", st["CDind"])
    e = st.get("e", float("nan"))
    CD = CD_NONINDUCED + cdff
    W = mi["m"] * G
    endurance = BATT_WH * USABLE / (W * V * CD / CL / ETA + P_AV)
    sm = (st["Xnp"] - mi["x"]) / g.mac
    res = dict(ok=True, V=V, kit_on=kit_on, mass=mi["m"], x_cg=mi["x"], x_np=st["Xnp"], static_margin=sm,
               x_batt=x_batt, x_kit=x_kit, alpha=st["Alpha"], CL=CL, elevon=st["elevon"], e=e, CDi=cdff,
               LD=CL / CD, endurance_h=endurance, Cma=st["Cma"], Cnb=st["Cnb"], Clb=st["Clb"],
               Clp=st["Clp"], Cnr=st["Cnr"], Cmq=st["Cmq"], Cm_de=st.get("Cmd01"), Cl_da=st.get("Cld02"),
               Cn_da=st.get("Cnd02"), Ixx=mi["Ixx"], Iyy=mi["Iyy"], Izz=mi["Izz"])
    if modes:
        res.update(classify_modes(r.eig))
        res["eig"] = r.eig
        res["pitch_ok"] = lon_ok(res)
        res["lateral_ok"] = lat_ok(res)
    res["strips"] = r.strips
    return res


def np_reference(g: Geometry):
    """Independent neutral point from AeroSandbox's own vortex lattice (no winglets, no elevons)."""
    af = asb.Airfoil(g.airfoil)
    xs = [asb.WingXSec(xyz_le=[g.x_le(y), y, g.z(y)], chord=g.chord(y), twist=g.twist(y), airfoil=af)
          for y in (0.0, g.y_break, g.span / 2)]
    wing = asb.Wing(name="w", symmetric=True, xsecs=xs)
    plane = asb.Airplane(wings=[wing], xyz_ref=[0.4, 0, 0], s_ref=g.S, c_ref=g.mac, b_ref=g.span)
    vlm = asb.VortexLatticeMethod(airplane=plane, op_point=asb.OperatingPoint(atmosphere=ATMO, velocity=V_SEARCH, alpha=4.0),
                                  spanwise_resolution=12, chordwise_resolution=8)
    return float(vlm.run_with_stability_derivatives(alpha=True, beta=False, p=False, q=False, r=False)["x_np"])


def local_clmax(chord):
    Re = RHO * V_SEARCH * chord / MU
    a = asb.Airfoil("mh45").get_aero_from_neuralfoil(alpha=np.linspace(0, 16, 65), Re=Re, mach=0.04, model_size="large")
    return float(np.max(a["CL"]))


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    sys.stdout = Tee(OUT / "run_log.txt")
    print("Lodestar stability and trim, 2026-11-11")
    g0 = Geometry(c_root=float(DP["c_root"]), c_tip=float(DP["c_tip"]))
    print(f"Wing {g0.span} m, S {g0.S:.3f} m2, MAC {g0.mac:.3f} m (MAC leading edge at x = {g0.x_mac_le:.3f} m)")
    print(f"Search: {V_SEARCH} m/s at 2,000 m (rho {RHO:.3f}), CL {CL_SEARCH:.3f}\n")

    # Step 1: neutral point, and an independent check
    base = analyse(g0, 0.40, modes=False)
    x_np = base["x_np"]
    x_np_asb = np_reference(replace(g0, winglet_h=0))
    x_np_avl_nowl = analyse(replace(g0, winglet_h=0), 0.40, modes=False)["x_np"]
    print("Step 1. Neutral point (the balance point at which the wing is neutrally stable in pitch)")
    print(f"  AVL with 0.20 m winglets: x_np = {x_np:.3f} m ({(x_np - g0.x_mac_le) / g0.mac * 100:.1f}% MAC)")
    print(f"  AVL, no winglets: {x_np_avl_nowl:.3f} m; AeroSandbox VLM, no winglets: {x_np_asb:.3f} m "
          f"(difference {(x_np_asb - x_np_avl_nowl) / g0.mac * 100:+.1f}% MAC)\n")

    # Step 2: static margin sweep (loaded, search speed)
    print("Step 2. How far ahead of the neutral point should the balance point sit? (washout 3 deg, winglets 0.20 m)")
    rows = []
    for sm in (0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15):
        r = analyse(g0, x_np - sm * g0.mac)
        rows.append({k: v for k, v in r.items() if k not in ("eig", "strips")})
    sm_df = pd.DataFrame(rows)
    sm_df.round(4).to_csv(OUT / "static_margin_sweep.csv", index=False)
    print(sm_df[["static_margin", "x_cg", "x_batt", "elevon", "alpha", "e", "LD", "endurance_h",
                 "short_period_zeta", "short_period_wn", "phugoid_zeta"]].round(3).to_string(index=False))
    SM = 0.06
    print(f"  -> choose a static margin of {SM:.0%} of MAC: stable, but not so nose-heavy that the "
          "elevons must hold a lot of up-trim (which costs lift and adds drag)\n")
    x_cg = x_np - SM * g0.mac

    # Step 3: washout sweep
    print("Step 3. Washout (tips twisted nose-down): trims the wing and keeps the tips from stalling first")
    rows = []
    for wo in (0, 1, 2, 3, 4, 5, 6):
        g = replace(g0, washout_deg=wo)
        xnp = analyse(g, 0.40, modes=False)["x_np"]
        r = analyse(g, xnp - SM * g.mac, modes=False)
        m_l = r["mass"]
        v08 = math.sqrt(2 * m_l * G / (RHO * g.S * 0.8))
        s08 = analyse(g, xnp - SM * g.mac, V=v08, modes=False)
        y, c, cl = s08["strips"]
        k_out = int(np.argmin(abs(y - 0.85 * g.span / 2)))  # outboard strip at 85% semispan, where the elevons are
        rows.append(dict(washout_deg=wo, x_np=xnp, elevon=r["elevon"], alpha=r["alpha"], e=r["e"], LD=r["LD"],
                         endurance_h=r["endurance_h"], elevon_at_CL08=s08["elevon"],
                         root_cl_ratio_CL08=float(cl[0] / local_clmax(c[0])),
                         outboard_cl_ratio_CL08=float(cl[k_out] / local_clmax(c[k_out]))))
    wo_df = pd.DataFrame(rows)
    wo_df.round(4).to_csv(OUT / "washout_sweep.csv", index=False)
    print(wo_df.round(3).to_string(index=False))

    # Step 4: winglet sweep (lateral-directional modes)
    print("\nStep 4. Winglet height: yaw stability (Cnb) and the lateral flight modes")
    rows = []
    for h in (0.0, 0.10, 0.15, 0.20, 0.25, 0.30):
        g = replace(g0, winglet_h=h)
        xnp = analyse(g, 0.40, modes=False)["x_np"]
        r = analyse(g, xnp - SM * g.mac)
        # Lateral modes get weakest at top speed: check it loaded and empty
        f = analyse(g, xnp - SM * g.mac, V=22.0, kit_on=False)
        fl = analyse(g, xnp - SM * g.mac, V=22.0, kit_on=True)
        rows.append({k: v for k, v in r.items() if k not in ("eig", "strips")}
                    | dict(winglet_h=h, fast_empty_dr_zeta=f["dutch_roll_zeta"], fast_empty_spiral_t2=f["spiral_t2"],
                           fast_loaded_dr_zeta=fl["dutch_roll_zeta"],
                           fast_lateral_ok=bool(f["lateral_ok"] and fl["lateral_ok"])))
    wl_df = pd.DataFrame(rows)
    wl_df.round(4).to_csv(OUT / "winglet_sweep.csv", index=False)
    cols = ["winglet_h", "Cnb", "Clb", "dutch_roll_wn", "dutch_roll_zeta", "roll_tau", "spiral_root", "spiral_t2",
            "e", "LD", "lateral_ok", "fast_loaded_dr_zeta", "fast_empty_dr_zeta", "fast_empty_spiral_t2", "fast_lateral_ok"]
    print(wl_df[[c for c in cols if c in wl_df]].round(3).to_string(index=False))
    passing = wl_df[(wl_df.lateral_ok == True) & (wl_df.fast_lateral_ok == True)]  # noqa: E712
    h_pick = float(passing.winglet_h.min()) if len(passing) else 0.20
    print(f"  -> winglet height {h_pick:.2f} m "
          f"({'smallest that meets every criterion' if len(passing) else 'no height meets every criterion; see notes'})\n")

    # Step 5: final configuration across speeds, kit on and off
    WO = 3.0
    gF = replace(g0, washout_deg=WO, winglet_h=h_pick)
    x_npF = analyse(gF, 0.40, modes=False)["x_np"]
    x_cgF = x_npF - SM * gF.mac
    print(f"Step 5. Final: washout {WO:.0f} deg, winglets {h_pick:.2f} m, balance point x = {x_cgF:.3f} m "
          f"({(x_cgF - gF.x_mac_le) / gF.mac * 100:.1f}% MAC)")
    rows = []
    v_stall_loaded = float(DP["v_stall"])
    for kit_on in (True, False):
        for V in (12.5, 15.0, 18.0, 22.0):
            r = analyse(gF, x_cgF, V=V, kit_on=kit_on, workdir=(OUT / "avl") if (kit_on and V == 15.0) else None)
            rows.append({k: v for k, v in r.items() if k not in ("eig", "strips")})
    sp_df = pd.DataFrame(rows)
    sp_df.round(4).to_csv(OUT / "final_speed_sweep.csv", index=False)
    print(sp_df[["kit_on", "V", "mass", "static_margin", "CL", "alpha", "elevon", "LD", "short_period_zeta",
                 "phugoid_zeta", "dutch_roll_zeta", "roll_tau", "spiral_t2", "pitch_ok", "lateral_ok"]].round(3).to_string(index=False))
    print(f"  battery packs at x = {sp_df.x_batt.iloc[0]:.3f} m, kit bay at x = {sp_df.x_kit.iloc[0]:.3f} m")

    # Kit-offset sensitivity: what if the kit bay can't sit exactly on the balance point?
    print("\n  If the kit bay sits off the balance point, the drop shifts it:")
    rows = []
    for off in (-0.10, -0.05, 0.0, 0.05, 0.10):
        on = analyse(gF, x_cgF, kit_on=True, kit_offset=off, modes=False)
        offr = analyse(gF, x_cgF, kit_on=False, kit_offset=off, modes=False)
        rows.append(dict(kit_offset_m=off, sm_loaded=on["static_margin"], sm_empty=offr["static_margin"],
                         elevon_loaded=on["elevon"], elevon_empty=offr["elevon"]))
        print(f"    kit {off * 100:+.0f} cm: static margin {on['static_margin']:.1%} -> {offr['static_margin']:.1%} after drop; "
              f"trim elevon {on['elevon']:+.1f} -> {offr['elevon']:+.1f} deg")
    pd.DataFrame(rows).round(4).to_csv(OUT / "kit_offset.csv", index=False)

    # Step 6: CLmax check at the stall-speed lift coefficient
    print("\nStep 6. Is a 3D CLmax of 0.80 realistic? Trim at increasing CL and compare each strip's lift with "
          "NeuralFoil's 2D maximum for that chord")
    rows = []
    m_loaded = sp_df.mass.iloc[0]
    for CLt in (0.6, 0.7, 0.8, 0.9, 1.0):
        V = math.sqrt(2 * m_loaded * G / (RHO * gF.S * CLt))
        r = analyse(gF, x_cgF, V=V, modes=False)
        y, c, cl = r["strips"]
        clmax = np.array([local_clmax(ci) for ci in c])
        ratio = cl / clmax
        i = int(np.argmax(ratio))
        k90 = int(np.argmin(abs(y - 0.9 * gF.span / 2)))
        rows.append(dict(CL_trim=r["CL"], V=V, elevon=r["elevon"], alpha=r["alpha"], worst_strip_y=float(y[i]),
                         worst_ratio=float(ratio[i]), root_ratio=float(ratio[0]), ratio_at_90pct_span=float(ratio[k90])))
        if abs(CLt - 0.8) < 1e-9:
            strips_08 = (y, cl, clmax)
    cl_df = pd.DataFrame(rows)
    cl_df.round(4).to_csv(OUT / "clmax_check.csv", index=False)
    print(cl_df.round(3).to_string(index=False))
    over = cl_df[cl_df.worst_ratio >= 1.0]
    first = over.CL_trim.min() if len(over) else None
    k = np.polyfit(cl_df.worst_ratio, cl_df.CL_trim, 1)
    print(f"  linear extrapolation: worst strip reaches its 2D maximum at trimmed CL about {np.polyval(k, 1.0):.2f}")
    print(f"  -> first strip reaches its 2D maximum at trimmed CL about {first:.2f}" if first
          else "  -> no strip reaches its 2D maximum up to CL 1.0")

    # Approximation cross-checks for the final search case
    fin = analyse(gF, x_cgF)
    q = 0.5 * RHO * V_SEARCH**2
    wn_ph = math.sqrt(2) * G / V_SEARCH
    wn_dr = math.sqrt(fin["Cnb"] * q * gF.S * gF.span / fin["Izz"])
    tau_roll = -fin["Ixx"] / (fin["Clp"] * q * gF.S * gF.span**2 / (2 * V_SEARCH))  # Lp = Clp q S b^2 / 2V
    ld_true = CL_SEARCH / (CD_NONINDUCED + CL_SEARCH**2 / (math.pi * 0.8 * AR))
    print(f"  phugoid damping: AVL {fin['phugoid_zeta']:.3f} vs Lanchester 1/(sqrt(2) L/D) = {1 / (math.sqrt(2) * fin['LD']):.3f}")
    print("\nCross-check of AVL's modes against textbook approximations (search case, kit on):")
    print(f"  phugoid wn: AVL {fin['phugoid_wn']:.3f} vs Lanchester sqrt(2) g/V = {wn_ph:.3f} rad/s")
    print(f"  Dutch roll wn: AVL {fin['dutch_roll_wn']:.2f} vs sqrt(Cnb q S b / Izz) = {wn_dr:.2f} rad/s")
    print(f"  roll time constant: AVL {fin['roll_tau']:.3f} vs -Ixx / Lp = {tau_roll:.3f} s")
    print(f"  inertias: Ixx {fin['Ixx']:.3f}, Iyy {fin['Iyy']:.3f}, Izz {fin['Izz']:.3f} kg m^2")
    pb2v = -fin["Cl_da"] * 15 / fin["Clp"]
    print(f"  roll authority: 15 deg aileron gives steady roll pb/2V = {pb2v:.3f} "
          f"(about {pb2v * 2 * V_SEARCH / gF.span * 57.3:.0f} deg/s; 0.05-0.07 is typical for small UAVs)")
    print(f"  adverse yaw: Cn per deg aileron {fin['Cn_da']:.5f} vs Cl per deg {fin['Cl_da']:.5f}")

    # ---------------------------------------------------------- charts
    BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
    INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                         "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})

    # Chart 1: static margin trade: trim elevon and endurance
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    axes[0].plot(sm_df.static_margin * 100, sm_df.elevon, color=BLUE, lw=2, marker="o", ms=6)
    axes[0].set_ylabel("Trim elevon (deg, negative = up)")
    axes[0].set_title("Trim elevon", color=INK, loc="left")
    axes[1].plot(sm_df.static_margin * 100, sm_df.endurance_h, color=BLUE, lw=2, marker="o", ms=6)
    axes[1].set_ylabel("Endurance (h)")
    axes[1].set_title("Endurance after trim", color=INK, loc="left")
    for ax in axes:
        ax.axvline(SM * 100, color=MUTED, lw=1, ls="--")
        ax.set_xlabel("Static margin (% MAC)")
        ax.grid(axis="y", color=GRID)
    fig.suptitle("More nose-heaviness = more stable, but more up-elevon and less endurance", color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "static_margin_trade.png", dpi=150)

    # Chart 2: winglets vs Dutch roll damping and spiral
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    axes[0].plot(wl_df.winglet_h * 100, wl_df.Cnb, color=BLUE, lw=2, marker="o", ms=6)
    axes[0].axhline(0, color=MUTED, lw=1)
    axes[0].set_ylabel("Cnb (per rad, positive = weathervanes into wind)")
    axes[0].set_title("Yaw stability", color=INK, loc="left")
    axes[1].plot(wl_df.winglet_h * 100, wl_df.dutch_roll_zeta, color=BLUE, lw=2, marker="o", ms=6,
                 label="search, 15 m/s, kit on")
    axes[1].plot(wl_df.winglet_h * 100, wl_df.fast_loaded_dr_zeta, color=ORANGE, lw=2, marker="s", ms=6,
                 label="top speed, 22 m/s, kit on (sets the size)")
    axes[1].legend(frameon=False, loc="upper left")
    axes[1].axhline(CRIT["dr_zeta_min"], color=MUTED, lw=1, ls="--")
    axes[1].text(20, CRIT["dr_zeta_min"] - 0.008, "minimum damping 0.08", color=MUTED)
    axes[1].set_ylabel("Dutch roll damping ratio")
    axes[1].set_title("Dutch roll damping", color=INK, loc="left")
    for ax in axes:
        ax.set_xlabel("Winglet height (cm)")
        ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "winglet_trade.png", dpi=150)

    # Chart 3: strip lift vs local max at CL 0.8
    y, cl, clmax = strips_08
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(y, cl, color=BLUE, lw=2, label="local lift coefficient (trimmed, CL 0.8)")
    ax.plot(y, clmax, color=ORANGE, lw=2, ls="--", label="local 2D maximum (NeuralFoil)")
    ax.set_xlabel("Distance from centreline (m)")
    ax.set_ylabel("Section lift coefficient")
    ax.set_title("Which part of the wing stalls first?", color=INK, loc="left")
    ax.legend(frameon=False, loc="lower left")
    ax.grid(axis="y", color=GRID)
    fig.tight_layout()
    fig.savefig(OUT / "spanwise_lift_at_stall.png", dpi=150)

    # Chart 4: side view of where the masses sit
    items = mass_items(gF, sp_df.x_batt.iloc[0], sp_df.x_kit.iloc[0])
    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot([0, gF.c_root], [0, 0], color=MUTED, lw=6, solid_capstyle="round", alpha=0.4)
    named = [i for i in items if i.name not in ("wing strip", "winglet", "battery R")]
    for i in named:
        ax.plot(i.x, 0, "o", ms=6 + 10 * math.sqrt(i.m), color=BLUE, alpha=0.8, mec="#fcfcfb", mew=2)
        ax.annotate(i.name.replace("battery L", "battery (2 packs)"), (i.x, 0), xytext=(0, 18 if named.index(i) % 2 else -26),
                    textcoords="offset points", ha="center", color=INK, fontsize=8)
    ax.axvline(x_cgF, color=ORANGE, lw=2)
    ax.text(x_cgF, 0.9, " balance point", color=INK, va="top")
    ax.axvline(x_npF, color=AQUA, lw=2, ls="--")
    ax.text(x_npF, 0.6, " neutral point", color=INK, va="top")
    ax.set_ylim(-1, 1)
    ax.set_yticks([])
    ax.set_xlabel("Distance aft of the root leading edge (m)")
    ax.set_title("Mass layout (root chord shown in grey)", color=INK, loc="left")
    ax.spines["left"].set_visible(False)
    fig.tight_layout()
    fig.savefig(OUT / "mass_layout.png", dpi=150)

    pd.Series({"washout_deg": WO, "winglet_h": h_pick, "static_margin": SM, "x_np": x_npF, "x_cg": x_cgF,
               "x_batt": sp_df.x_batt.iloc[0], "x_kit": sp_df.x_kit.iloc[0], "mac": gF.mac, "x_mac_le": gF.x_mac_le}
              ).to_csv(OUT / "final_config.csv", header=["value"])
    print(f"\nCharts and CSVs written to {OUT.relative_to(ROOT)}")
