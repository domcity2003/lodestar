"""Step 2 (G6, G9): 6-DOF JSBSim model of the frozen Lodestar v1, and the flight cases it answers.

Questions:
  1. Trim across the envelope (1.1 / 1.25 x stall to VD; kit on and off; sea level and 2,000 m):
     alpha, elevon, thrust and throttle; flag any trim outside the elevon pitch budget.
  2. MIL-F-8785C Dryden turbulence (light / moderate / severe) Monte Carlo at search speed and at the
     drop speed: bank and pitch excursions, lowest airspeed against stall, elevon saturation.
  3. Kit release at 40 m with the v1 masses, calm and moderate turbulence.
  4. Motor-out at search speed: glide transition, best glide speed, glide ratio, sink, a 30 deg banked
     turn in the glide; glide polar (V vs sink) kit on / off at 2,000 m -> out/sixdof_v1/glide_polar.csv.
  5. Bungee launch: 16 m/s release (OPERATIONS launch_release_min_tas), launcher energy, first 5 s at
     full power, still air and at the 30 deg crosswind limit; lowest airspeed against 1.1 x stall.
  6. Belly landing: touchdown vertical / horizontal speed at the approach speed with a flare, and the
     energy the 40 mm crushable skid has to absorb.

Model (out/sixdof_v1/aircraft/lodestar_v1/lodestar_v1.xml), all inputs read, none re-entered:
  mass      controls_sizing.aircraft_items(): out/cad_v4/mass_cg.csv items + the spanwise v1 wing sections,
            joints, servos and the lodestar_v1.ADDED_MASS rows (the item list that reproduces
            lodestar_v1.MASS). Kit-off mass, CG and inertia as the empty aircraft; the rescue kit and its
            parachute as two JSBSim point masses at their CAD positions, zeroed at release.
  lift, moments  AVL (controls_sizing.avl: v1 planform with pod and winglets, the same runs the controls
            note used) at ten trimmed CL values from -0.1 to 1.0, kit-on CG as moment reference. Tables in
            alpha of CL and Cm at zero elevon, elevon power, and every lateral derivative; roll and yaw
            are computed in stability axes and rotated to body axes. Linear lift: there is no stall
            model, so time spent above CLmax 0.80 (lodestar_v1 OPERATIONS) is counted as a stall.
  drag      the mission / propulsion-margins drag model (propulsion_margins.drag: NeuralFoil MH45 profile
            drag at the local Reynolds number x the "halfway" finish factor, induced drag with e = 0.80,
            pod + gimbal drag area), as a JSBSim table in CL and Reynolds number. So glide and endurance
            numbers here agree with the mission sim by construction.
  thrust    external force on the motor axis. Maximum thrust vs airspeed from propulsion_margins.available()
            (UIUC CAM 15x10 data, AT3520-550, SiCore 6S5P at 15 C): continuous limits in flight, 30 s
            limits for launch. Folding prop: zero thrust and zero drag with the motor off (ASSUMPTION).
  servos    first-order lag 20 /s, surface rate 250 deg/s (0.12 s/60 deg servo through the 2:1 linkage),
            +/-20 deg geometric travel per surface (lodestar_v1 CONTROLS). Elevon / aileron mixing is done
            in the autopilot with the plain-flap effectiveness of controls_sizing (effective vs geometric).
  autopilot 50 Hz, ArduPilot-class basic modes: airspeed -> thrust (PI), altitude -> pitch -> elevon
            (PID outer, PI + pitch-rate inner), bank hold -> aileron (P + roll-rate), airspeed gain
            scheduling (V_trim / V)^2. Glide and approach: pitch-for-airspeed. Flare: sink-rate hold.
  turbulence JSBSim MIL-F-8785C Dryden (turb-type 3). Below 1,000 ft the low-altitude form uses only the
            wind at 20 ft: light 15 kt, moderate 30 kt, severe 45 kt. JSBSim picks the form from the height
            above SEA LEVEL, so turbulence cases put the terrain at sea level and warm the air so the density
            equals the case altitude (the 2026-11-11 kit-release study did the same).

Run (from scripts/):  ../.venv/Scripts/python.exe -X utf8 sixdof_v1.py [N_MC]
Writes: out/sixdof_v1/ (aircraft XML, CSV results, figures, run_log.txt)
"""
from __future__ import annotations

from pathlib import Path
import math
import sys
import time

import numpy as np
import pandas as pd
import jsbsim
from scipy.optimize import fsolve, brentq

sys.path.insert(0, str(Path(__file__).parent))
import lodestar_v1 as v1  # noqa: E402
import controls_sizing as cs  # noqa: E402
import avl_run as av  # noqa: E402
import propulsion_margins as pm  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "sixdof_v1"
ACNAME = "lodestar_v1"
ACDIR = OUT / "aircraft" / ACNAME

FT, LB, IN = 0.3048, 4.4482216, 0.0254
G0 = 9.80665
DT = 1 / 200                       # s, JSBSim step
AP_EVERY = 4                       # autopilot at 50 Hz
CELL = pm.CELLS["Amprius SiCore 6.3 Ah"]
T_PACK = v1.PROPULSION["pack_min_temp_c"]      # packs held at >= 15 C
CL_MAX = v1.OPERATIONS["cl_max_3d"]
STALL_HOLD = 0.25                  # s above CLmax counted as a stall (as the 2026-11-11 study)
TRAVEL = v1.CONTROLS["elevon_travel_deg"][1]   # deg geometric per surface
ROLL_RESERVE = 5.0                 # deg kept for roll (controls_sizing ROLL_MIN)
PITCH_BUDGET = TRAVEL - ROLL_RESERVE           # deg geometric available for pitch trim
SURF_RATE = math.radians(60 / v1.CONTROLS["servo_max_s_per_60deg"] / v1.CONTROLS["linkage_ratio"])  # rad/s
SERVO_LAG = 20.0                   # 1/s
V_SEARCH = v1.OPERATIONS["search_speed_tas"][1]   # 20 m/s TAS, SiCore
V_DROP = v1.OPERATIONS["drop_runin_tas"]["light"]   # v1.1: 18 m/s (v1.0 14 m/s stalled 49/50 in light turbulence)
V_DROP_TURB = v1.OPERATIONS["drop_runin_tas"]["moderate"]   # v1.1: 20 m/s in moderate turbulence
H_SEARCH = 120.0                   # m AGL (requirement R4)
H_REL = v1.OPERATIONS["release_agl_m"]
ALT = v1.OPERATIONS["design_alt_m"]
VD_EAS = v1.STRUCTURE["vd_eas"]
KIT_KG = v1.MASS["rescue_kit_kg"] + v1.MASS["kit_chute_kg"]
TURB = {"light": 15.0, "moderate": 30.0, "severe": 45.0}       # kt at 20 ft, MIL-F-8785C low altitude
SEVERITY = {"light": 3, "moderate": 4, "severe": 6}             # JSBSim exceedance index (used above 2,000 ft only)
N_MC = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 50
SEED_BASE = 20261115


# ============================================================================ mass and AVL data
def mass_model():
    """Kit-on and kit-off item lists (SiCore) and their totals; kit point masses from the CAD table."""
    cad, src, dx = cs.cad_table()
    on = cs.aircraft_items(cad, True, "SiCore", dx=dx)
    off = cs.aircraft_items(cad, False, "SiCore", dx=dx)
    kit = [i for i in on if i.name in ("rescue kit", "kit parachute")]
    return dict(on=on, off=off, mi_on=av.cg_and_inertia(on), mi_off=av.cg_and_inertia(off), kit=kit, source=src)


CL_SWEEP = (-0.10, 0.05, 0.15, 0.25, 0.35, 0.45, 0.56, 0.68, 0.80, 1.00)
DERIVS = ("CLq", "Cmq", "CYb", "CYp", "CYr", "Clb", "Clp", "Clr", "Cnb", "Cnp", "Cnr")


def avl_sweep(mm, cache=True):
    path = OUT / "avl_sweep.csv"
    if cache and path.exists():
        return pd.read_csv(path)
    rows = []
    x_ref = mm["mi_on"]["x"]
    for cl in CL_SWEEP:
        st, _, _ = cs.avl(mm["on"], V_SEARCH, cs.RHO_2K, CL=cl, x_cg=x_ref)
        d2r = 180 / math.pi
        de = st["elevon"]
        r = dict(CL_trim=cl, alpha_deg=st["Alpha"], elevon_trim_deg=de, Xnp=st["Xnp"],
                 CLde=st["CLd01"] * d2r, Cmde=st["Cmd01"] * d2r,
                 CYda=st["CYd02"] * d2r, Clda=st["Cld02"] * d2r, Cnda=st["Cnd02"] * d2r,
                 CDind_avl=st["CDind"], e_avl=st["e"])
        r["CL0"] = st["CLtot"] - st["CLd01"] * de
        r["Cm0"] = st["Cmtot"] - st["Cmd01"] * de
        r.update({k: st[k] for k in DERIVS})
        rows.append(r)
    df = pd.DataFrame(rows)
    df.to_csv(path, index=False)
    return df


def avl_modes(mm, V=V_SEARCH):
    """AVL eigenvalues, kit on, 2,000 m: phugoid and Dutch-roll periods for the model check."""
    _, _, eig = cs.avl(mm["on"], V, cs.RHO_2K, modes=True)
    pairs = sorted([e for e in eig if e.imag > 1e-6], key=lambda e: abs(e))
    ph = pairs[0]
    dr = [e for e in pairs if 1.0 < e.imag < 6.0 and e.real > -2.0]
    return 2 * math.pi / ph.imag, (2 * math.pi / dr[0].imag if dr else float("nan")), eig


# ============================================================================ drag and thrust
def cd_model(cl, re):
    """propulsion_margins.drag() as a coefficient: profile (x halfway finish) + induced + pod/gimbal."""
    cl = np.asarray(cl, float)
    return (pm.sz.cd_profile(np.minimum(cl, 1.2), re) * pm.PF + cl**2 / (math.pi * pm.sz.A.e_oswald * pm.AR)
            + pm.d2.CDA_BODY / pm.S)


def isa_air(alt_m):
    """ISA air at a geometric altitude (density altitude = altitude), as propulsion_margins.air()."""
    import aerosandbox as asb
    t_c = float(asb.Atmosphere(altitude=alt_m).temperature()) - 273.15
    return pm.air(alt_m, t_c)


def thrust_table(ai, short, soc=0.5):
    vg = np.arange(0.0, 40.01, 1.0)
    tm = []
    for V in vg:
        a = pm.available(max(V, 0.5), ai, CELL, T_PACK, soc, short)
        tm.append(a["T"] if a else 0.0)
    return vg, np.array(tm)


# ============================================================================ JSBSim aircraft
def _table1(name, xs, ys):
    rows = "\n".join(f"            {x:.6f} {y:.6f}" for x, y in zip(xs, ys))
    return (f'<function name="{name}"><table><independentVar lookup="row">aero/alpha-rad</independentVar>'
            f"<tableData>\n{rows}\n          </tableData></table></function>")


def _ext(xs, ys, lo, hi):
    """Add linear extrapolation end points at alpha lo and hi (rad)."""
    s0 = (ys[1] - ys[0]) / (xs[1] - xs[0])
    s1 = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
    return np.r_[lo, xs, hi], np.r_[ys[0] + s0 * (lo - xs[0]), ys, ys[-1] + s1 * (hi - xs[-1])]


def write_aircraft(mm, sw):
    ACDIR.mkdir(parents=True, exist_ok=True)
    on, off = mm["mi_on"], mm["mi_off"]
    a = np.radians(sw.alpha_deg.values)
    lo, hi = math.radians(-20), math.radians(35)
    tabs = []
    for col in ("CL0", "Cm0") + tuple(c for c in sw.columns if c in DERIVS or c in ("CLde", "Cmde", "CYda", "Clda", "Cnda")):
        xs, ys = (a, sw[col].values)
        if col in ("CL0", "Cm0"):
            xs, ys = _ext(a, sw[col].values, lo, hi)
        tabs.append(_table1(f"aero/c/{col}", xs, ys))
    cls = np.r_[-0.6, np.arange(-0.3, 1.21, 0.05), 1.6, 2.5]
    res = np.array([6e4, 1e5, 1.5e5, 2e5, 3e5, 4e5, 6e5, 8e5, 1.2e6])
    cd = np.array([[float(cd_model(c, r)) for r in res] for c in cls])
    head = "          " + " ".join(f"{r:.0f}" for r in res)
    body = "\n".join("          " + f"{c:.3f} " + " ".join(f"{v:.6f}" for v in row) for c, row in zip(cls, cd))
    cd_tab = ('<function name="aero/cd"><table><independentVar lookup="row">aero/cl</independentVar>'
              '<independentVar lookup="column">aero/re-model</independentVar>'
              f"<tableData>\n{head}\n{body}\n        </tableData></table></function>")

    def P(p):
        return f"<property>{p}</property>"

    def prod(*xs):
        return "<product>" + "".join(x if x.startswith("<") else P(x) for x in xs) + "</product>"

    al = "aero/alpha-rad"
    funcs = [
        f'<function name="fcs/elevon-total-rad"><sum>{P("fcs/elevon-pos-rad")}{P("fcs/elevon-trim-rad")}</sum></function>',
        f'<function name="aero/re-model"><product>{P("velocities/vt-fps")}<value>{FT}</value>{P("fcs/re-per-mps")}</product></function>',
        f'<function name="aero/qhat">{prod("velocities/q-aero-rad_sec", "aero/ci2vel")}</function>',
        # stability-axis roll and yaw rates (AVL derivatives are stability-axis)
        f'<function name="aero/ps-hat"><product><sum>{prod("velocities/p-aero-rad_sec", f"<cos>{P(al)}</cos>")}'
        f'{prod("velocities/r-aero-rad_sec", f"<sin>{P(al)}</sin>")}</sum>{P("aero/bi2vel")}</product></function>',
        f'<function name="aero/rs-hat"><product><difference>{prod("velocities/r-aero-rad_sec", f"<cos>{P(al)}</cos>")}'
        f'{prod("velocities/p-aero-rad_sec", f"<sin>{P(al)}</sin>")}</difference>{P("aero/bi2vel")}</product></function>',
        *tabs,
        f'<function name="aero/cl"><sum>{P("aero/c/CL0")}{prod("aero/c/CLq", "aero/qhat")}{prod("aero/c/CLde", "fcs/elevon-total-rad")}</sum></function>',
        cd_tab,
        f'<function name="aero/cy"><sum>{prod("aero/c/CYb", "aero/beta-rad")}{prod("aero/c/CYp", "aero/ps-hat")}'
        f'{prod("aero/c/CYr", "aero/rs-hat")}{prod("aero/c/CYda", "fcs/aileron-pos-rad")}</sum></function>',
        f'<function name="aero/cl-s"><sum>{prod("aero/c/Clb", "aero/beta-rad")}{prod("aero/c/Clp", "aero/ps-hat")}'
        f'{prod("aero/c/Clr", "aero/rs-hat")}{prod("aero/c/Clda", "fcs/aileron-pos-rad")}</sum></function>',
        f'<function name="aero/cn-s"><sum>{prod("aero/c/Cnb", "aero/beta-rad")}{prod("aero/c/Cnp", "aero/ps-hat")}'
        f'{prod("aero/c/Cnr", "aero/rs-hat")}{prod("aero/c/Cnda", "fcs/aileron-pos-rad")}</sum></function>',
        f'<function name="aero/cm"><sum>{P("aero/c/Cm0")}{prod("aero/c/Cmq", "aero/qhat")}{prod("aero/c/Cmde", "fcs/elevon-total-rad")}</sum></function>',
        f'<function name="aero/croll"><difference>{prod("aero/cl-s", f"<cos>{P(al)}</cos>")}{prod("aero/cn-s", f"<sin>{P(al)}</sin>")}</difference></function>',
        f'<function name="aero/cyaw"><sum>{prod("aero/cl-s", f"<sin>{P(al)}</sin>")}{prod("aero/cn-s", f"<cos>{P(al)}</cos>")}</sum></function>',
    ]
    qs = ("aero/qbar-psf", "metrics/Sw-sqft")

    def axis(name, coef, extra=()):
        return f'<axis name="{name}"><function name="aero/force/{name}">{prod(*qs, *extra, coef)}</function></axis>'

    axes = [axis("LIFT", "aero/cl"), axis("DRAG", "aero/cd"), axis("SIDE", "aero/cy"),
            axis("ROLL", "aero/croll", ("metrics/bw-ft",)), axis("PITCH", "aero/cm", ("metrics/cbarw-ft",)),
            axis("YAW", "aero/cyaw", ("metrics/bw-ft",))]
    clip = math.radians(2 * cs.effective(TRAVEL))     # travel is clipped per surface in mix(); this only bounds the servo model

    def actuator(n):
        return (f'<actuator name="fcs/{n}-actuator"><input>fcs/{n}-cmd-rad</input><lag>{SERVO_LAG}</lag>'
                f"<rate_limit>{SURF_RATE:.4f}</rate_limit><clipto><min>{-clip:.4f}</min><max>{clip:.4f}</max></clipto>"
                f"<output>fcs/{n}-pos-rad</output></actuator>")

    def contact(name, x, y, z, k=4000.0, c=200.0):
        return (f'<contact type="STRUCTURE" name="{name}"><location unit="M"><x>{x}</x><y>{y}</y><z>{z}</z></location>'
                f"<static_friction>0.6</static_friction><dynamic_friction>0.5</dynamic_friction>"
                f'<spring_coeff unit="N/M">{k}</spring_coeff><damping_coeff unit="N/M/SEC">{c}</damping_coeff></contact>')

    pm_xml = "".join(
        f'<pointmass name="{k.name}"><weight unit="KG">{k.m:.5f}</weight>'
        f'<location unit="M"><x>{k.x:.5f}</x><y>{k.y:.5f}</y><z>{k.z:.5f}</z></location></pointmass>'
        for k in mm["kit"])
    xml = f"""<?xml version="1.0"?>
<!-- Lodestar v1 (lodestar_v1.py {v1.VERSION}), written by scripts/sixdof_v1.py, 2026-11-15. Do not edit by hand. -->
<fdm_config name="{ACNAME}" version="2.0" release="ALPHA">
  <fileheader><author>Lodestar</author><filecreationdate>2026-11-15</filecreationdate>
    <description>Lodestar v1 flying wing: AVL lift/moment tables in alpha, mission drag model, SiCore pack, kit as point masses</description></fileheader>
  <metrics>
    <wingarea unit="M2">{cs.SREF:.5f}</wingarea>
    <wingspan unit="M">{cs.B:.5f}</wingspan>
    <chord unit="M">{cs.MAC:.5f}</chord>
    <location name="AERORP" unit="M"><x>{on['x']:.5f}</x><y>0</y><z>{on['z']:.5f}</z></location>
    <location name="EYEPOINT" unit="M"><x>0.05</x><y>0</y><z>-0.11</z></location>
    <location name="VRP" unit="M"><x>0</x><y>0</y><z>0</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="KG*M2">{off['Ixx']:.5f}</ixx>
    <iyy unit="KG*M2">{off['Iyy']:.5f}</iyy>
    <izz unit="KG*M2">{off['Izz']:.5f}</izz>
    <ixz unit="KG*M2">{off['Ixz']:.5f}</ixz>
    <emptywt unit="KG">{off['m']:.5f}</emptywt>
    <location name="CG" unit="M"><x>{off['x']:.5f}</x><y>0</y><z>{off['z']:.5f}</z></location>
    {pm_xml}
  </mass_balance>
  <ground_reactions>
    {contact("SKID", 0.27, 0, -0.113)}
    {contact("NOSE", -0.17, 0, -0.06)}
    {contact("LEFT_TIP", 0.95, -1.5, 0.0)}
    {contact("RIGHT_TIP", 0.95, 1.5, 0.0)}
    {contact("TAIL", 0.75, 0, -0.06)}
  </ground_reactions>
  <propulsion/>
  <external_reactions>
    <force name="thrust" frame="BODY" unit="LBS">
      <location unit="M"><x>0.68</x><y>0</y><z>-0.035</z></location>
      <direction><x>1</x><y>0</y><z>0</z></direction>
    </force>
    <force name="door" frame="BODY" unit="LBS">
      <location unit="M"><x>0.4220</x><y>0</y><z>-0.11</z></location>
      <direction><x>-1</x><y>0</y><z>0</z></direction>
    </force>
  </external_reactions>
  <flight_control name="fcs">
    <property value="0">fcs/elevon-cmd-rad</property>
    <property value="0">fcs/aileron-cmd-rad</property>
    <property value="0">fcs/elevon-trim-rad</property>
    <property value="25000">fcs/re-per-mps</property>
    <channel name="servos">
      {actuator("elevon")}
      {actuator("aileron")}
    </channel>
  </flight_control>
  <aerodynamics>
    {chr(10).join("    " + f for f in funcs)}
    {chr(10).join("    " + x for x in axes)}
  </aerodynamics>
</fdm_config>
"""
    (ACDIR / f"{ACNAME}.xml").write_text(xml, encoding="utf-8")


# ============================================================================ elevon mixing
_GEO = np.linspace(0, 40, 801)
_EFF = np.array([cs.effective(d) for d in _GEO])


def to_geometric(eff_deg):
    return np.sign(eff_deg) * np.interp(abs(eff_deg), _EFF, _GEO)


def to_effective(geo_deg):
    return np.sign(geo_deg) * np.interp(abs(geo_deg), _GEO, _EFF)


def mix(de, da):
    """Effective elevon (pitch) and aileron (roll) commands, rad -> per-surface geometric clip at TRAVEL.
    Right surface = de + da, left = de - da (AVL sign convention, controls_sizing). Returns (de, da, saturated)."""
    r, l_ = math.degrees(de + da), math.degrees(de - da)
    gr, gl = to_geometric(r), to_geometric(l_)
    sat = abs(gr) >= TRAVEL or abs(gl) >= TRAVEL
    gr, gl = float(np.clip(gr, -TRAVEL, TRAVEL)), float(np.clip(gl, -TRAVEL, TRAVEL))
    r, l_ = math.radians(to_effective(gr)), math.radians(to_effective(gl))
    return (r + l_) / 2, (r - l_) / 2, sat, max(abs(gr), abs(gl))


# ============================================================================ simulation
class Sim:
    """One JSBSim run with the autopilot. ground_m: terrain elevation; rho_case: if given, the terrain is put
    at sea level and the air warmed so its density equals rho_case (needed for low-altitude turbulence)."""

    def __init__(self, ground_m, h_agl, V, *, kit=True, rho_case=None, turb=None, seed=1,
                 alpha_deg=5.0, gamma_deg=0.0, theta_deg=None, beta_deg=0.0, phi_deg=0.0, short_thrust=False, soc=0.5,
                 ai=None):
        f = self.f = jsbsim.FGFDMExec(str(OUT), None)
        f.set_debug_level(0)
        f.set_dt(DT)
        assert f.load_model(ACNAME)
        self.ground = 0.0 if rho_case is not None else ground_m
        if rho_case is not None:
            t_k = 101325.0 / (287.053 * rho_case)
            f["atmosphere/delta-T"] = (t_k - 288.15) * 1.8
        f["ic/terrain-elevation-ft"] = self.ground / FT
        f["ic/h-sl-ft"] = (self.ground + h_agl) / FT
        f["ic/vt-fps"] = V / FT
        f["ic/alpha-deg"] = alpha_deg
        f["ic/beta-deg"] = beta_deg
        f["ic/phi-deg"] = phi_deg
        if theta_deg is not None:
            f["ic/theta-deg"] = theta_deg
        else:
            f["ic/gamma-deg"] = gamma_deg
        f["ic/psi-true-deg"] = 0.0
        if not kit:
            for i in range(2):
                f[f"inertia/pointmass-weight-lbs[{i}]"] = 0.0
        if turb:
            f["atmosphere/turb-type"] = 3
            f["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = TURB[turb] * 1.68781
            f["atmosphere/turbulence/milspec/severity"] = SEVERITY[turb]
            f["simulation/randomseed"] = seed
        self.ai = ai if ai is not None else isa_air(ground_m + h_agl)
        f["fcs/re-per-mps"] = self.ai["rho"] * pm.CHORD / self.ai["mu"]
        self.vg_t, self.tmax = thrust_table(self.ai, short_thrust, soc)
        f.run_ic()
        self.rho = f["atmosphere/rho-slugs_ft3"] * 515.379
        self.mode = "alt"
        self.V_cmd, self.h_cmd, self.phi_cmd, self.th_fix, self.sink_cmd = V, self.ground + h_agl, 0.0, 0.0, 0.0
        self.T = self.T0 = 0.0
        self.th0 = math.radians(alpha_deg + gamma_deg)
        self.de0 = 0.0
        self.de = self.da = 0.0
        self.de_off = 0.0
        self.i_v = self.i_h = self.i_th = self.i_g = 0.0
        self.motor = True
        self.motor_delay = 0.0
        self.door = False
        self.door_cda = 0.012           # m2, open bay (2026-11-11 study ASSUMPTION)
        self.n = 0
        self.sat_steps = self.ap_steps = 0
        self.over_run = self.over_max = 0.0
        self.log = []
        self.stop = None

    # -- helpers
    def t(self):
        return self.f["simulation/sim-time-sec"]

    def V(self):
        return self.f["velocities/vt-fps"] * FT

    def tmax_at(self, V):
        return float(np.interp(V, self.vg_t, self.tmax))

    def set_trim(self, tr):
        self.T = self.T0 = tr["T"]
        self.th0 = math.radians(tr["theta_deg"])
        self.de = self.de0 = math.radians(tr["de_eff_deg"])
        self.set_elevon_trim(self.de0)

    def set_elevon_trim(self, de):
        """The servo model holds deflections about a trim offset, so a run starts at the trimmed elevon."""
        self.de_off = de
        self.f["fcs/elevon-trim-rad"] = de

    # -- autopilot
    def autopilot(self):
        f, dt = self.f, DT * AP_EVERY
        V = self.V()
        h = f["position/h-sl-ft"] * FT
        hdot = f["velocities/h-dot-fps"] * FT
        th, q = f["attitude/theta-rad"], f["velocities/q-rad_sec"]
        ph, p = f["attitude/phi-rad"], f["velocities/p-rad_sec"]
        sc = float(np.clip((self.V_cmd / max(V, 5.0)) ** 2, 0.4, 2.0))
        tmax = self.tmax_at(V) if self.motor else 0.0
        if self.mode == "alt":
            ev = self.V_cmd - V
            self.i_v = float(np.clip(self.i_v + ev * dt, -10, 10))
            self.T = float(np.clip(self.T0 + 2.0 * ev + 0.6 * self.i_v, 0, tmax))
            eh = self.h_cmd - h
            self.i_h = float(np.clip(self.i_h + eh * dt, -20, 20))
            th_cmd = self.th0 + 0.03 * eh + 0.004 * self.i_h - 0.04 * hdot
            th_cmd = float(np.clip(th_cmd, math.radians(-12), math.radians(15)))
        elif self.mode == "glide":          # motor off, pitch for airspeed
            self.T = 0.0
            ev = V - self.V_cmd
            self.i_g = float(np.clip(self.i_g + ev * dt, -20, 20))
            th_cmd = float(np.clip(self.th0 + 0.025 * ev + 0.006 * self.i_g, math.radians(-25), math.radians(15)))
        elif self.mode == "launch":         # full power, fixed pitch, pitch lowered only below the protect speed
            self.T = tmax if self.t() >= self.motor_delay else 0.0
            under = max(0.0, self.V_cmd - V)
            th_cmd = self.th_fix - 0.05 * under
        elif self.mode == "flare":          # motor off, hold sink rate, pitch limited
            self.T = 0.0
            es = (-hdot) - self.sink_cmd    # positive = sinking faster than wanted
            self.i_g = float(np.clip(self.i_g + es * dt, -5, 5))
            th_cmd = float(np.clip(self.th_flare0 + 0.08 * es + 0.03 * self.i_g, math.radians(-5), math.radians(14)))
        else:
            raise ValueError(self.mode)
        et = th - th_cmd
        self.i_th = float(np.clip(self.i_th + et * dt, -0.3, 0.3))
        de = self.de0 + sc * (1.0 * et + 0.8 * self.i_th + 0.25 * q)
        da = -math.copysign(1.0, CLDA_SIGN) * sc * (0.6 * (ph - self.phi_cmd) + 0.12 * p)
        self.de, self.da, sat, _ = mix(de, da)
        self.ap_steps += 1
        self.sat_steps += sat

    def step_until(self, t_end, release_at=None, door_window=(1.0, 2.0), until=None):
        f = self.f
        while f["simulation/sim-time-sec"] < t_end - 1e-9:
            t = f["simulation/sim-time-sec"]
            if release_at is not None:
                self.door = release_at - door_window[0] <= t <= release_at + door_window[1]
                if t >= release_at and f["inertia/pointmass-weight-lbs[0]"] > 0:
                    f["inertia/pointmass-weight-lbs[0]"] = 0.0
                    f["inertia/pointmass-weight-lbs[1]"] = 0.0
            if self.n % AP_EVERY == 0:
                self.autopilot()
            f["fcs/elevon-cmd-rad"] = self.de - self.de_off
            f["fcs/aileron-cmd-rad"] = self.da
            f["external_reactions/thrust/magnitude"] = self.T / LB
            f["external_reactions/door/magnitude"] = (f["aero/qbar-psf"] * 47.880 * self.door_cda / LB) if self.door else 0.0
            f.run()
            self.n += 1
            cl = f["aero/cl"]
            self.over_run = self.over_run + DT if cl > CL_MAX else 0.0
            self.over_max = max(self.over_max, self.over_run)
            if self.n % 4 == 0:
                self.record()
            if until is not None and until(self):
                self.stop = f["simulation/sim-time-sec"]
                self.record()
                break

    def record(self):
        f = self.f
        th = f["attitude/theta-rad"]
        h_cg = f["position/h-agl-ft"] * FT
        self.log.append(dict(
            t=f["simulation/sim-time-sec"], h_agl=h_cg, V=self.V(),
            h_skid=skid_height(self),
            theta=math.degrees(th), phi=math.degrees(f["attitude/phi-rad"]), psi=math.degrees(f["attitude/psi-rad"]),
            alpha=math.degrees(f["aero/alpha-rad"]), beta=math.degrees(f["aero/beta-rad"]), cl=f["aero/cl"],
            q=f["velocities/q-rad_sec"], nz=-f["accelerations/n-pilot-z-norm"],
            hdot=f["velocities/h-dot-fps"] * FT, vg=f["velocities/vg-fps"] * FT,
            elevon=math.degrees(f["fcs/elevon-total-rad"]), aileron=math.degrees(f["fcs/aileron-pos-rad"]),
            thrust=self.T, mass=f["inertia/weight-lbs"] * LB / G0, door=self.door))

    def mi_now(self):
        return (self.f["inertia/cg-x-in"] * IN, self.f["inertia/cg-z-in"] * IN)


CLDA_SIGN = -1.0       # overwritten from the AVL sweep in load()


def skid_height(s):
    """Height of the belly-skid contact point above the ground (pitch only; bank from the wing tip is checked
    separately). Lodestar axes x aft, z up: a nose-up pitch theta lowers points aft of the CG."""
    f = s.f
    th = f["attitude/theta-rad"]
    xc, zc = s.mi_now()
    dx, dz = 0.27 - xc, -0.113 - zc
    return f["position/h-agl-ft"] * FT + dz * math.cos(th) - dx * math.sin(th)


# ============================================================================ trim (JSBSim residuals)
def _residual(f, V, h, gamma, alpha, de, T):
    f["ic/vt-fps"] = V / FT
    f["ic/h-sl-ft"] = h / FT
    f["ic/alpha-deg"] = alpha
    f["ic/gamma-deg"] = gamma
    f["ic/beta-deg"] = 0.0
    f["ic/phi-deg"] = 0.0
    f["fcs/elevon-cmd-rad"] = 0.0
    f["fcs/elevon-trim-rad"] = math.radians(de)
    f["fcs/aileron-cmd-rad"] = 0.0
    f["external_reactions/thrust/magnitude"] = T / LB
    f.run_ic()
    return f


def trim_fdm(kit, ai, h_sl=None):
    f = jsbsim.FGFDMExec(str(OUT), None)
    f.set_debug_level(0)
    f.set_dt(DT)
    assert f.load_model(ACNAME)
    if not kit:
        for i in range(2):
            f[f"inertia/pointmass-weight-lbs[{i}]"] = 0.0
    f["fcs/re-per-mps"] = ai["rho"] * pm.CHORD / ai["mu"]
    f["ic/h-sl-ft"] = 1000.0 / FT
    f["ic/vt-fps"] = 20.0 / FT
    f.run_ic()                     # updates the mass properties (kit point masses)
    return f


def accel(f):
    return np.array([f["accelerations/udot-ft_sec2"] * FT, f["accelerations/wdot-ft_sec2"] * FT,
                     f["accelerations/qdot-rad_sec2"]])


def trim(f, V, h, *, gamma=0.0, T=None, guess=(5.0, 0.0, 3.0)):
    """Steady straight flight. T None: solve alpha, elevon, thrust at flight-path angle gamma.
    T given (0 = motor off): solve alpha, elevon, gamma. Returns dict incl. the JSBSim lift and weight."""
    def res(x):
        a, de, z = x
        if T is None:
            _residual(f, V, h, gamma, a, de, z)
        else:
            _residual(f, V, h, z, a, de, T)
        # one zero-length evaluation of the derivatives with the actuator at the commanded position
        return accel(f)
    x0 = np.array(guess if T is None else (guess[0], guess[1], -3.0))
    sol, info, ier, msg = fsolve(res, x0, full_output=True, xtol=1e-10)
    r = res(sol)
    a, de, z = sol
    out = dict(V=V, h=h, alpha_deg=a, de_eff_deg=de, de_geo_deg=float(to_geometric(de)),
               T=(z if T is None else T), gamma_deg=(gamma if T is None else z),
               resid=float(np.abs(r).max()), ok=bool(ier == 1 and np.abs(r).max() < 1e-3),
               CL=f["aero/cl"], CD=f["aero/cd"], qbar=f["aero/qbar-psf"] * 47.880,
               lift_N=f["aero/force/LIFT"] * LB, drag_N=f["aero/force/DRAG"] * LB,
               weight_N=f["inertia/weight-lbs"] * LB, mass=f["inertia/weight-lbs"] * LB / G0,
               x_cg=f["inertia/cg-x-in"] * IN, theta_deg=a + (gamma if T is None else z),
               g_local=f["accelerations/gravity-ft_sec2"] * FT)
    # force balance normal to the flight path: (L + T sin alpha) / (m g_eff cos gamma). JSBSim flies over a rotating
    # WGS84 Earth at latitude 0: effective gravity = local gravity - Omega^2 r (0.35% at the equator).
    r_e = 6378137.0 + h
    out["g_eff"] = out["g_local"] - 7.292115e-5**2 * r_e
    out["n_balance"] = ((out["lift_N"] + out["T"] * math.sin(math.radians(a)))
                        / (out["mass"] * out["g_eff"] * math.cos(math.radians(out["gamma_deg"]))))
    return out


def load(rebuild=True):
    """Build (or reuse) the model. Returns the mass model and AVL sweep."""
    global CLDA_SIGN
    OUT.mkdir(parents=True, exist_ok=True)
    mm = mass_model()
    sw = avl_sweep(mm, cache=not rebuild)
    CLDA_SIGN = float(np.sign(sw.Clda.median()))
    if rebuild or not (ACDIR / f"{ACNAME}.xml").exists():
        write_aircraft(mm, sw)
    return mm, sw


def v_stall(m, rho, n=1.0):
    return math.sqrt(2 * n * m * G0 / (rho * cs.SREF * CL_MAX))


# ============================================================================ analyses
class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


def oscillation_period(t, y):
    y = y - np.mean(y)
    zc = t[1:][(y[:-1] < 0) & (y[1:] >= 0)]
    return float(np.mean(np.diff(zc))) if len(zc) >= 3 else float("nan")


def model_check(mm):
    """Open-loop JSBSim periods against AVL eigenvalues (kit on, 20 m/s, 2,000 m)."""
    ph_avl, dr_avl, _ = avl_modes(mm)
    ai = isa_air(ALT)
    f = trim_fdm(True, ai)
    tr = trim(f, V_SEARCH, ALT + H_SEARCH)
    out = {}
    for mode, kick in (("phugoid", ("de", 2.0, 0.5, 60.0, "V")), ("Dutch roll", ("da", 3.0, 0.3, 8.0, "beta"))):
        s = Sim(ALT, H_SEARCH, V_SEARCH, alpha_deg=tr["alpha_deg"], gamma_deg=0.0)
        s.set_trim(tr)
        s.mode = None
        t0 = s.t()
        s.de, s.da = s.de0, 0.0
        _open_loop(s, t0 + 2.0)
        if kick[0] == "de":
            s.de += math.radians(kick[1])
        else:
            s.da += math.radians(kick[1])
        _open_loop(s, t0 + 2.0 + kick[2])
        s.de, s.da = s.de0, 0.0
        _open_loop(s, t0 + 2.0 + kick[2] + kick[3])
        d = pd.DataFrame(s.log)
        d = d[d.t > t0 + 2.0 + kick[2]]
        out[mode] = oscillation_period(d.t.values, d[kick[4]].values)
    return pd.DataFrame([dict(mode="phugoid", avl_period_s=ph_avl, jsbsim_period_s=out["phugoid"]),
                         dict(mode="Dutch roll", avl_period_s=dr_avl, jsbsim_period_s=out["Dutch roll"])]).assign(
        diff_pct=lambda d: (d.jsbsim_period_s / d.avl_period_s - 1) * 100)


def _open_loop(s, t_end):
    f = s.f
    while s.t() < t_end - 1e-9:
        f["fcs/elevon-cmd-rad"] = s.de - s.de_off
        f["fcs/aileron-cmd-rad"] = s.da
        f["external_reactions/thrust/magnitude"] = s.T / LB
        f.run()
        s.n += 1
        if s.n % 4 == 0:
            s.record()


# ---------------------------------------------------------------------------- 1. trim sweep
def trim_sweep():
    rows = []
    for alt in (0.0, ALT):
        ai = isa_air(alt)
        vg, tc = thrust_table(ai, short=False)
        for kit in (True, False):
            f = trim_fdm(kit, ai)
            m = f["inertia/weight-lbs"] * LB / G0
            vs = v_stall(m, ai["rho"])
            vd = VD_EAS * math.sqrt(1.225 / ai["rho"])
            speeds = sorted(set([round(1.1 * vs, 2), round(v1.OPERATIONS["stall_margin"] * vs, 2)]
                                + [float(v) for v in np.arange(14, vd, 2.0) if v > 1.25 * vs + 0.5] + [round(vd, 2)]))
            guess = (8.0, -1.0, 3.0)
            for V in speeds:
                tr = trim(f, V, alt + H_SEARCH, guess=guess)
                guess = (tr["alpha_deg"], tr["de_eff_deg"], tr["T"])
                tmax = float(np.interp(V, vg, tc))
                flag = []
                if abs(tr["de_geo_deg"]) > PITCH_BUDGET:
                    flag.append("elevon beyond pitch budget")
                if tr["T"] > tmax:
                    flag.append("thrust above continuous maximum (not reachable level)")
                if tr["CL"] > CL_MAX:
                    flag.append("CL above CLmax")
                rows.append(dict(alt_m=alt, kit=kit, mass_kg=m, V_tas=V, V_eas=V * math.sqrt(ai["rho"] / 1.225),
                                 V_over_Vs=V / vs, CL=tr["CL"], alpha_deg=tr["alpha_deg"], elevon_eff_deg=tr["de_eff_deg"],
                                 elevon_geo_deg=tr["de_geo_deg"], pitch_budget_used_pct=100 * abs(tr["de_geo_deg"]) / PITCH_BUDGET,
                                 thrust_N=tr["T"], thrust_max_cont_N=tmax, throttle_frac=tr["T"] / tmax if tmax > 0 else np.nan,
                                 lift_over_weight=tr["n_balance"], resid=tr["resid"],
                                 flag="; ".join(flag)))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------- 2. turbulence Monte Carlo
def calm_trim(kit, V, alt):
    ai = isa_air(alt)
    f = trim_fdm(kit, ai)
    return trim(f, V, alt + 50.0)


def turb_run(level, V, h_agl, kit, seed, tr, rho, t_settle=10.0, t_run=60.0, release=False):
    s = Sim(0.0, h_agl, V, kit=kit, rho_case=rho, turb=level, seed=seed, alpha_deg=tr["alpha_deg"], ai=isa_air(ALT))
    s.set_trim(tr)
    s.step_until(s.t() + t_settle)
    s.log.clear()
    s.sat_steps = s.ap_steps = 0
    s.over_max = 0.0
    t0 = s.t()
    s.step_until(t0 + t_run, release_at=(t0 + 5.0) if release else None)
    d = pd.DataFrame(s.log)
    d["t"] -= t0 + (5.0 if release else 0.0)
    m_end = d.mass.iloc[-1]
    vs_now = np.where(d.mass > m_end + 0.5, v_stall(d.mass.iloc[0], rho), v_stall(m_end, rho))
    return s, d, dict(seed=seed, phi_max_deg=d.phi.abs().max(), theta_dev_max_deg=(d.theta - tr["theta_deg"]).abs().max(),
                      V_min=d.V.min(), V_min_over_Vs=float((d.V / vs_now).min()), cl_max=d.cl.max(),
                      longest_over_clmax_s=s.over_max, stall=s.over_max > STALL_HOLD,
                      elevon_sat_frac=s.sat_steps / max(s.ap_steps, 1), h_dev_max_m=(d.h_agl - h_agl).abs().max(),
                      h_min_agl=d.h_agl.min(), nz_max=d.nz.max(), nz_min=d.nz.min())


def turbulence_mc(n):
    rho = isa_air(ALT)["rho"]
    cases = []
    for level in ("light", "moderate", "severe"):
        cases.append((f"search {V_SEARCH:.0f} m/s, kit on, {level}", level, V_SEARCH, H_SEARCH, True))
    for level in ("light", "moderate", "severe"):
        cases.append((f"drop run {V_DROP:.0f} m/s, kit on, {level}", level, V_DROP, H_REL, True))
    for level in ("moderate", "severe"):
        cases.append((f"drop run {V_DROP_TURB:.0f} m/s, kit on, {level}", level, V_DROP_TURB, H_REL, True))
    cases.append((f"drop run {V_DROP_TURB:.0f} m/s, kit on, light", "light", V_DROP_TURB, H_REL, True))
    cases.append((f"search {V_SEARCH:.0f} m/s, kit off, moderate", "moderate", V_SEARCH, H_SEARCH, False))
    trims = {}
    rows, allruns = [], []
    for name, level, V, h, kit in cases:
        key = (kit, V)
        if key not in trims:
            trims[key] = calm_trim(kit, V, ALT)
        out = []
        for k in range(n):
            _, _, m = turb_run(level, V, h, kit, SEED_BASE + k, trims[key], rho)
            out.append(m)
        d = pd.DataFrame(out)
        d.insert(0, "case", name)
        allruns.append(d)
        vs = v_stall(trims[key]["mass"], rho)
        rows.append(dict(case=name, level=level, V=V, kit=kit, runs=n, Vs_1g=vs,
                         phi_p50=d.phi_max_deg.median(), phi_p95=d.phi_max_deg.quantile(0.95),
                         theta_p50=d.theta_dev_max_deg.median(), theta_p95=d.theta_dev_max_deg.quantile(0.95),
                         V_min_over_Vs_p05=d.V_min_over_Vs.quantile(0.05), V_min_over_Vs_worst=d.V_min_over_Vs.min(),
                         runs_stalled=int(d.stall.sum()), runs_cl_over_max=int((d.cl_max > CL_MAX).sum()),
                         elevon_sat_frac_mean=d.elevon_sat_frac.mean(), elevon_sat_frac_p95=d.elevon_sat_frac.quantile(0.95),
                         h_dev_p95=d.h_dev_max_m.quantile(0.95), h_min_agl=d.h_min_agl.min(), nz_max_p95=d.nz_max.quantile(0.95)))
        print(f"  {name:<40} phi P95 {rows[-1]['phi_p95']:5.1f} deg, theta P95 {rows[-1]['theta_p95']:5.1f} deg, "
              f"V/Vs P5 {rows[-1]['V_min_over_Vs_p05']:.2f}, stalls {rows[-1]['runs_stalled']}/{n}, "
              f"elevon sat {100 * rows[-1]['elevon_sat_frac_mean']:.2f}%")
    return pd.DataFrame(rows), pd.concat(allruns, ignore_index=True)


# ---------------------------------------------------------------------------- 3. kit release
def release_metrics(d, h0):
    a = d[d.t >= 0]
    pre = d[(d.t < -1.2) & (d.t > -4)]
    th0 = pre.theta.mean()
    return dict(h_gain_max_m=(a.h_agl - h0).max(), h_loss_max_m=(h0 - a.h_agl).max(), h_min_agl=d[d.t >= -1].h_agl.min(),
                pitch_dev_max_deg=(a.theta - th0).abs().max(), theta_peak_deg=a.theta.max(), cl_max=d[d.t >= -1].cl.max(),
                V_min=d[d.t >= -1].V.min(), nz_max=a.nz.max(), nz_min=a.nz.min(),
                elevon_change_deg=a.elevon.iloc[-50:].mean() - pre.elevon.mean(),
                mass_before=pre.mass.iloc[-1], mass_after=a.mass.iloc[-1])


def kit_release(n):
    rho = isa_air(ALT)["rho"]
    rows, traces = [], {}
    for V in (V_DROP, V_DROP_TURB):
        tr = calm_trim(True, V, ALT)
        for frozen in (False, True):
            s = Sim(ALT, H_REL, V, alpha_deg=tr["alpha_deg"])
            s.set_trim(tr)
            s.step_until(s.t() + 10.0)
            s.log.clear()
            t0 = s.t()
            s.step_until(t0 + 5.0 - DT)
            if frozen:
                s.autopilot = lambda: None
            s.step_until(t0 + 35.0, release_at=t0 + 5.0)
            d = pd.DataFrame(s.log)
            d["t"] -= t0 + 5.0
            name = f"calm, {V:.0f} m/s, " + ("controls frozen at release" if frozen else "autopilot")
            traces[name] = d
            rows.append(dict(case=name, **release_metrics(d, H_REL)))
    for V in (V_DROP, V_DROP_TURB):
        tr = calm_trim(True, V, ALT)
        out = []
        for k in range(n):
            _, d, m = turb_run("moderate", V, H_REL, True, SEED_BASE + 500 + k, tr, rho, t_settle=10.0, t_run=25.0, release=True)
            out.append(dict(**release_metrics(d, H_REL), stall=m["stall"]))
            if k == 0:
                traces[f"moderate, {V:.0f} m/s, seed 1"] = d
        dd = pd.DataFrame(out)
        for qn, q in (("median", 0.5), ("P95", 0.95)):
            r = dd.drop(columns=["stall"]).quantile(q).to_dict()
            if qn == "P95":
                r["h_min_agl"] = dd.h_min_agl.quantile(0.05)
                r["V_min"] = dd.V_min.quantile(0.05)
            rows.append(dict(case=f"moderate turbulence, {V:.0f} m/s, {qn} of {n} (h_min, V_min: P5)", **r,
                             runs_stalled=int(dd.stall.sum())))
    return pd.DataFrame(rows), traces


# ---------------------------------------------------------------------------- 4. motor-out and glide polar
def glide_polar():
    rows = []
    for alt in (ALT, 0.0):
        ai = isa_air(alt)
        for kit in (True, False):
            f = trim_fdm(kit, ai)
            m = f["inertia/weight-lbs"] * LB / G0
            vs = v_stall(m, ai["rho"])
            vd = VD_EAS * math.sqrt(1.225 / ai["rho"])
            guess = (9.0, -1.5, -4.0)
            for V in np.r_[np.round(np.arange(math.ceil(1.05 * vs * 4) / 4, vd + 0.01, 0.5), 2)]:
                tr = trim(f, float(V), alt + 300.0, T=0.0, guess=guess)
                guess = (tr["alpha_deg"], tr["de_eff_deg"], tr["gamma_deg"])
                g = math.radians(tr["gamma_deg"])
                rows.append(dict(config="kit on" if kit else "kit off", alt_m=alt, mass_kg=m, V_tas=float(V),
                                 V_eas=float(V) * math.sqrt(ai["rho"] / 1.225), V_over_Vs=float(V) / vs, CL=tr["CL"], CD=tr["CD"],
                                 alpha_deg=tr["alpha_deg"], elevon_geo_deg=tr["de_geo_deg"], gamma_deg=tr["gamma_deg"],
                                 sink_mps=-float(V) * math.sin(g), glide_ratio=1 / math.tan(-g), CL_over_CD=tr["CL"] / tr["CD"],
                                 above_1p25_Vs=float(V) >= v1.OPERATIONS["stall_margin"] * vs, trim_ok=tr["ok"]))
    return pd.DataFrame(rows)


def best_points(gp):
    out = []
    for (cfg, alt), d in gp.groupby(["config", "alt_m"]):
        ok = d[d.above_1p25_Vs]
        bg = ok.loc[ok.glide_ratio.idxmax()]
        ms = ok.loc[ok.sink_mps.idxmin()]
        out.append(dict(config=cfg, alt_m=alt, mass_kg=d.mass_kg.iloc[0], V_best_glide=bg.V_tas, glide_ratio_max=bg.glide_ratio,
                        sink_at_best_glide=bg.sink_mps, V_min_sink=ms.V_tas, sink_min=ms.sink_mps,
                        glide_ratio_at_search=float(np.interp(V_SEARCH, d.V_tas, d.glide_ratio)),
                        sink_at_search=float(np.interp(V_SEARCH, d.V_tas, d.sink_mps)),
                        V_1p25_Vs=float(d.V_tas.iloc[0] / d.V_over_Vs.iloc[0] * v1.OPERATIONS["stall_margin"])))
    return pd.DataFrame(out)


def motor_out(bp):
    """Motor stops at search speed, 2,000 m, 300 m AGL. The autopilot pitches for best-glide speed, glides
    40 s wings level, then flies a 30 deg banked turn for 20 s, then rolls out."""
    rows, traces = [], {}
    for kit in (True, False):
        b = bp[(bp.config == ("kit on" if kit else "kit off")) & (bp.alt_m == ALT)].iloc[0]
        tr = calm_trim(kit, V_SEARCH, ALT)
        s = Sim(ALT, 300.0, V_SEARCH, kit=kit, alpha_deg=tr["alpha_deg"])
        s.set_trim(tr)
        s.step_until(s.t() + 5.0)
        s.log.clear()
        t0 = s.t()
        s.step_until(t0 + 2.0)
        ai = isa_air(ALT)
        fg = trim_fdm(kit, ai)
        tg = trim(fg, float(b.V_best_glide), ALT + 200, T=0.0, guess=(7, -1, -3))
        s.motor = False
        s.mode, s.V_cmd = "glide", float(b.V_best_glide)
        s.th0, s.de0 = math.radians(tg["theta_deg"]), math.radians(tg["de_eff_deg"])
        s.step_until(t0 + 42.0)
        s.phi_cmd = math.radians(v1.OPERATIONS["max_bank_deg"])
        s.step_until(t0 + 62.0)
        s.phi_cmd = 0.0
        s.step_until(t0 + 80.0)
        d = pd.DataFrame(s.log)
        d["t"] -= t0 + 2.0
        traces["kit on" if kit else "kit off"] = d
        steady = d[(d.t > 25) & (d.t < 40)]
        turn = d[(d.t > 45) & (d.t < 60)]
        sink = -steady.hdot.mean()
        gr = steady.vg.mean() / sink
        rows.append(dict(config="kit on" if kit else "kit off", V_cmd=b.V_best_glide, V_steady=steady.V.mean(),
                         sink_steady=sink, glide_ratio_sim=gr, glide_ratio_polar=b.glide_ratio_max,
                         theta_min_transition=d[d.t < 20].theta.min(), V_min_transition=d[d.t < 20].V.min(),
                         V_max_transition=d[d.t < 20].V.max(), elevon_steady_geo=float(to_geometric(steady.elevon.mean())),
                         bank_in_turn=turn.phi.mean(), sink_in_turn=-turn.hdot.mean(), V_in_turn=turn.V.mean(),
                         aileron_max_turn_entry=d[(d.t > 40) & (d.t < 45)].aileron.abs().max(),
                         elevon_max_geo=float(to_geometric(d.elevon.abs().max())), cl_max=d.cl.max(),
                         height_lost_80s=d.h_agl.iloc[0] - d.h_agl.iloc[-1]))
    return pd.DataFrame(rows), traces


# ---------------------------------------------------------------------------- 5. bungee launch
LAUNCH_DA = (3000.0, v1.OPERATIONS["max_alt_m"])
RAIL_DEG = 10.0         # launcher rail angle, ASSUMPTION (typical small-UAS bungee ramps 5-15 deg)
CRADLE_INC = 5.0        # wing incidence on the cradle (alpha at release), ASSUMPTION
H_LAUNCH = 1.5          # m, CG height at the end of the rail
TKOFF_PITCH = 15.0      # deg, ArduPilot TKOFF_LVL_PITCH default (pitch held after the throw)
LAUNCH_WIND = 12.0      # m/s, the wind the flight-control note uses for the 30 deg crosswind rule
MOTOR_DELAY = 1.0       # s, prop held stopped until clear of the launcher (ArduPilot TKOFF_THR_DELAY), ASSUMPTION


def launch_runs():
    rows, traces = [], {}
    xw = math.radians(v1.OPERATIONS["max_crosswind_launch_deg"])
    v_rel = v1.OPERATIONS["launch_release_min_tas"]
    cases = []
    for da in LAUNCH_DA:
        cases.append((f"DA {da:.0f} m, still air, pitch hold {TKOFF_PITCH:.0f} deg", da, None, TKOFF_PITCH, True))
    cases.append((f"DA {LAUNCH_DA[0]:.0f} m, still air, pitch hold {TKOFF_PITCH:.0f} deg, no speed protection",
                  LAUNCH_DA[0], None, TKOFF_PITCH, False))
    for W in (LAUNCH_WIND, 6.0):
        cases.append((f"DA {LAUNCH_DA[0]:.0f} m, {W:.0f} m/s wind 30 deg off the nose", LAUNCH_DA[0], W, TKOFF_PITCH, True))
    cases.append((f"DA {LAUNCH_DA[1]:.0f} m, {LAUNCH_WIND:.0f} m/s wind 30 deg off the nose", LAUNCH_DA[1], LAUNCH_WIND, TKOFF_PITCH, True))
    cases.append((f"DA {LAUNCH_DA[1]:.0f} m, still air, motor start delayed 0.5 s", LAUNCH_DA[1], None, TKOFF_PITCH, True))
    cases.append((f"DA {LAUNCH_DA[1]:.0f} m, still air, motor start delayed {MOTOR_DELAY:.0f} s", LAUNCH_DA[1], None, TKOFF_PITCH, True))
    cases.append((f"DA {LAUNCH_DA[1]:.0f} m, {LAUNCH_WIND:.0f} m/s wind 30 deg off the nose, motor start delayed {MOTOR_DELAY:.0f} s",
                  LAUNCH_DA[1], LAUNCH_WIND, TKOFF_PITCH, True))
    for name, da, W, pitch, protect in cases:
        ai = isa_air(da)
        m = None
        beta = 0.0
        vg_need = v_rel
        if W is not None:
            # Air arrives from 30 deg right of the rail. The rail gives a ground speed along the nose; the airspeed at
            # release is v_rel with sideslip beta = asin(W sin 30 / v_rel). In a steady uniform wind the air-relative
            # motion does not depend on the wind itself, so the run starts at v_rel and beta in still air.
            beta = math.degrees(math.asin(W * math.sin(xw) / v_rel))
            vg_need = math.sqrt(v_rel**2 - (W * math.sin(xw))**2) - W * math.cos(xw)
        s = Sim(da, H_LAUNCH, v_rel, kit=True, alpha_deg=CRADLE_INC, theta_deg=RAIL_DEG + CRADLE_INC,
                beta_deg=beta, short_thrust=True, soc=0.95)
        s.motor_delay = (0.5 if "delayed 0.5" in name else MOTOR_DELAY) if "delayed" in name else 0.0
        m = s.f["inertia/weight-lbs"] * LB / G0
        vs = v_stall(m, s.rho)
        tr = trim(trim_fdm(True, ai), 1.25 * vs, da + 50, gamma=0.0)
        s.de0 = s.de = math.radians(tr["de_eff_deg"])
        s.set_elevon_trim(s.de0)
        s.mode, s.th_fix = "launch", math.radians(pitch)
        s.V_cmd = 1.25 * vs if protect else 0.0
        s.record()
        s.step_until(5.0, until=lambda z: z.f["position/h-agl-ft"] * FT < 0.15)
        d = pd.DataFrame(s.log)
        traces[name] = d
        dpsi = (d.psi.iloc[-1] - d.psi.iloc[0] + 180) % 360 - 180
        rows.append(dict(case=name, da_m=da, mass_kg=m, Vs_1g=vs, V_release=v_rel, beta_release_deg=beta,
                         ground_speed_along_rail=vg_need, V_min=d.V.min(), V_min_over_Vs=d.V.min() / vs,
                         below_1p1_Vs=bool(d.V.min() < 1.1 * vs), t_V_min=d.t.iloc[int(d.V.values.argmin())],
                         h_min_agl=d.h_agl.min(), h_5s=d.h_agl.iloc[-1], V_5s=d.V.iloc[-1], theta_max=d.theta.max(),
                         phi_max_abs=d.phi.abs().max(), beta_5s=d.beta.iloc[-1], heading_change_deg=dpsi,
                         cl_max=d.cl.max(), longest_over_clmax_s=s.over_max, T_full=d.thrust.max(),
                         ground_hit=s.stop is not None,
                         ke_launcher_J=0.5 * m * vg_need**2))
    return pd.DataFrame(rows), traces


def launcher_energy():
    """Kinetic energy the launcher must give the aircraft (kit on, still air) and the stroke at the 10 g FEA
    launch limit; stored-energy range for an assumed launcher efficiency."""
    m = v1.MASS["sicore_loaded_kg"]
    V = v1.OPERATIONS["launch_release_min_tas"]
    ke = 0.5 * m * V**2
    rows = []
    for a_g in (10.0, 6.0, 4.0):
        rows.append(dict(mass_kg=m, V_release=V, ke_J=ke, accel_avg_g=a_g, stroke_m=V**2 / (2 * a_g * G0),
                         stored_J_eff_0p5=ke / 0.5, stored_J_eff_0p7=ke / 0.7))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------- 6. belly landing
FLARE_H = 3.0          # m skid height to start the flare (ArduPilot LAND_FLARE_ALT default 3 m)
FLARE_SINK = 0.25      # m/s, ArduPilot TECS_LAND_SINK default


def landing(kit, V_app_over_vs, flare=True, turb=None, seed=1, approach_h=40.0):
    rho = isa_air(ALT)["rho"]
    ai = isa_air(ALT)
    f = trim_fdm(kit, ai)
    m = f["inertia/weight-lbs"] * LB / G0
    vs = v_stall(m, rho)
    V_app = V_app_over_vs * vs
    tg = trim(f, V_app, ALT + 50, T=0.0, guess=(9, -2, -4))
    s = Sim(ALT, approach_h, V_app, kit=kit, alpha_deg=tg["alpha_deg"], gamma_deg=tg["gamma_deg"],
            rho_case=rho if turb else None, turb=turb, seed=seed, ai=ai)
    s.T = 0.0
    s.motor = False
    s.de0 = s.de = math.radians(tg["de_eff_deg"])
    s.set_elevon_trim(s.de0)
    s.mode, s.V_cmd, s.th0 = "glide", V_app, math.radians(tg["theta_deg"])
    s.step_until(120.0, until=lambda z: skid_height(z) < FLARE_H if flare else skid_height(z) < 0.0)
    if flare and skid_height(s) > 0:
        s.mode = "flare"
        s.th_flare0 = s.f["attitude/theta-rad"]
        s.sink_cmd = FLARE_SINK
        s.i_g = 0.0
        s.step_until(s.t() + 30.0, until=lambda z: skid_height(z) < 0.0)
    d = pd.DataFrame(s.log)
    last = d.iloc[-1]
    vz = -last.hdot
    return dict(kit=kit, V_app=V_app, V_app_over_vs=V_app_over_vs, flare=flare, turb=turb or "calm", seed=seed,
                mass_kg=m, touchdown_V=last.V, touchdown_vx_ground=last.vg, touchdown_sink=vz, theta_td=last.theta,
                alpha_td=last.alpha, phi_td=last.phi, cl_max=d.cl.max(), longest_over_clmax_s=s.over_max,
                V_min=d.V.min(), ke_vertical_J=0.5 * m * vz**2, ke_horizontal_J=0.5 * m * last.vg**2,
                flare_time_s=(d.t.iloc[-1] - d[d.h_skid < FLARE_H].t.iloc[0]) if flare and (d.h_skid < FLARE_H).any() else 0.0), d


# EPP (expanded polypropylene) compressive stress at 25% strain, JSP ARPRO values (ASTM D3575) as tabulated by
# Rapid Die Cut: 14.5 / 23.5 / 42 psi at 20 / 30 / 45 g/L (link in the research note).
EPP_GRADES = {"EPP 20 g/L": 14.5 * 6894.76, "EPP 30 g/L": 23.5 * 6894.76, "EPP 45 g/L": 42.0 * 6894.76}   # Pa
EPP_DENSIFY = 0.70     # usable strain before densification, ASSUMPTION (typical bead foams 0.6-0.75)
SKID_FOOT = (0.30, 0.025)   # m, CAD v4 belly skid footprint (length x width)


def skid_energy(landings):
    stroke = v1.STRUCTURE["belly_skid_crush_stroke_min_mm"] / 1e3
    rows = []
    for (label, m, vz) in landings:
        e = 0.5 * m * vz**2
        f_avg = e / stroke                          # force that uses exactly the full stroke
        a_half_sine_peak = math.pi * vz**2 / (4 * stroke) / G0     # FEA pulse shape (fea_extra.landing_dynamic)
        r = dict(case=label, mass_kg=m, sink_mps=vz, energy_J=e, f_avg_full_stroke_N=f_avg,
                 a_peak_half_sine_g=a_half_sine_peak, min_thickness_mm=1e3 * stroke / EPP_DENSIFY)
        for g, sig in EPP_GRADES.items():
            area_cad = SKID_FOOT[0] * SKID_FOOT[1]
            r[f"{g}: area for full stroke cm2"] = f_avg / sig * 1e4
            r[f"{g}: stroke used by CAD footprint mm"] = 1e3 * e / (sig * area_cad)
            r[f"{g}: peak g with CAD footprint"] = sig * area_cad / (m * G0)
            r[f"{g}: energy available CAD footprint J"] = sig * area_cad * stroke
        rows.append(r)
    return pd.DataFrame(rows)


def tip_clearance():
    """Height of the wing-tip trailing edge above the ground when the skid touches, vs pitch and bank.
    Lodestar axes x aft, z up; nose-up pitch lowers aft points, right-wing-down bank lowers the right tip."""
    x_te, y_tip = cs.GEO.x_le(cs.B / 2) + cs.GEO.chord(cs.B / 2), cs.B / 2
    rows = []
    for label, z_skid in (("CAD v4 skid (4 mm strip), bottom z -0.111 m", -0.111),
                          ("EPP skid, 40 mm stroke / 0.70 usable strain = 57 mm, bottom z -0.164 m", -0.107 - 0.057)):
        for th in (4.0, 6.5, 8.0):
            t = math.radians(th)

            def h(x, y, z, ph):
                return z * math.cos(t) * math.cos(ph) - (x - 0.27) * math.sin(t) - y * math.sin(ph) * math.cos(t)
            clear0 = h(x_te, y_tip, 0.0, 0.0) - h(0.27, 0.0, z_skid, 0.0)
            phi_max = math.degrees(brentq(lambda ph: h(x_te, y_tip, 0.0, ph) - h(0.27, 0.0, z_skid, ph), 0.0, 0.5)) if clear0 > 0 else 0.0
            rows.append(dict(skid=label, theta_deg=th, tip_te_clearance_wings_level_mm=1e3 * clear0, bank_for_tip_contact_deg=phi_max))
    return pd.DataFrame(rows)


# ============================================================================ figures
def figures(trims_df, turb_sum, rel_traces, gp, mo_traces, l_traces, ld_traces):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    style = ROOT / "style" / "lodestar.mplstyle"
    if style.exists():
        plt.style.use(str(style))
    INK, C1, C2, MUTED = "#1f1f1e", "#2a5d9f", "#c0504d", "#7a7f87"

    # Figure 1: glide polar
    fig, ax = plt.subplots(figsize=(6.0, 3.6))
    for (cfg, alt), d in gp.groupby(["config", "alt_m"]):
        if alt != ALT:
            continue
        col = C1 if cfg == "kit on" else C2
        ax.plot(d.V_tas, d.sink_mps, color=col, lw=1.5)
        dd = d[~d.above_1p25_Vs]
        ax.plot(dd.V_tas, dd.sink_mps, color=col, lw=3, alpha=0.3)
        k = d[d.above_1p25_Vs].glide_ratio.idxmax()
        ax.plot([d.V_tas[k]], [d.sink_mps[k]], "o", color=col, ms=4)
        ax.annotate(f"{cfg}: L/D {d.glide_ratio[k]:.1f} at {d.V_tas[k]:.1f} m/s", (d.V_tas[k], d.sink_mps[k]),
                    xytext=(6, -12 if cfg == "kit on" else 8), textcoords="offset points", fontsize=8, color=col)
    ax.invert_yaxis()
    ax.set_xlabel("True airspeed V [m/s]")
    ax.set_ylabel("Sink rate [m/s]")
    ax.set_title("Figure 1. Sink rate vs. true airspeed, motor off, by kit state, 2,000 m ISA", loc="left", fontsize=9)
    fig.text(0.01, 0.01, "JSBSim trim, folding prop (zero thrust/drag); thick segment: below 1.25 Vs", fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(OUT / "fig1_glide_polar.png", dpi=180)
    plt.close(fig)

    # Figure 2: kit release
    fig, axs = plt.subplots(2, 2, figsize=(7.5, 4.8), sharex=True)
    for name, col in ((f"calm, {V_DROP:.0f} m/s, autopilot", C1), (f"calm, {V_DROP:.0f} m/s, controls frozen at release", C2),
                      (f"moderate, {V_DROP_TURB:.0f} m/s, seed 1", MUTED)):
        d = rel_traces[name]
        for ax, k, yl in ((axs[0, 0], "h_agl", "Height above ground h [m]"), (axs[0, 1], "theta", r"Pitch $\theta$ [deg]"),
                          (axs[1, 0], "V", "True airspeed V [m/s]"), (axs[1, 1], "elevon", r"Elevon $\delta_e$ [deg]")):
            ax.plot(d.t, d[k], color=col, lw=1.2, label=name)
            ax.set_ylabel(yl, fontsize=8)
    for ax in axs.flat:
        ax.axvline(0, color=MUTED, lw=0.6, ls="--")
        ax.set_xlim(-5, 25)
    axs[1, 0].set_xlabel("Time after release t [s]")
    axs[1, 1].set_xlabel("Time after release t [s]")
    axs[0, 0].legend(frameon=False, fontsize=6)
    fig.suptitle("Figure 2. Height, pitch, airspeed and elevon vs. time through the kit release, by control mode",
                 x=0.01, ha="left", fontsize=9)
    fig.text(0.01, 0.005, f"Release of {KIT_KG:.3f} kg at 40 m AGL, 2,000 m; door open 1 s before to 2 s after", fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(OUT / "fig2_kit_release.png", dpi=180)
    plt.close(fig)

    # Figure 3: launch
    fig, axs = plt.subplots(1, 3, figsize=(8.5, 3.0))
    cols = [C1, C2, MUTED, "#5b8c5a", "#8e6bb0", "#d18b2c"]
    for (name, d), col in zip(l_traces.items(), cols):
        axs[0].plot(d.t, d.V, color=col, lw=1.1, label=name)
        axs[1].plot(d.t, d.h_agl, color=col, lw=1.1)
        axs[2].plot(d.t, d.theta, color=col, lw=1.1)
    for ax, yl in zip(axs, ("True airspeed V [m/s]", "Height above ground h [m]", r"Pitch $\theta$ [deg]")):
        ax.set_xlabel("Time after release t [s]")
        ax.set_ylabel(yl, fontsize=8)
    axs[0].legend(frameon=False, fontsize=5.5, loc="lower right")
    fig.suptitle("Figure 3. Airspeed, height and pitch vs. time after a 16 m/s bungee release, by density altitude and wind",
                 x=0.01, ha="left", fontsize=9)
    fig.text(0.01, 0.005, f"Kit on, full power (30 s limits), rail {RAIL_DEG:.0f} deg, cradle incidence {CRADLE_INC:.0f} deg",
             fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 0.92))
    fig.savefig(OUT / "fig3_launch.png", dpi=180)
    plt.close(fig)

    # Figure 4: landing
    fig, axs = plt.subplots(1, 3, figsize=(8.5, 3.0))
    for (name, d), col in zip(ld_traces.items(), cols):
        t = d.t - d.t.iloc[-1]
        axs[0].plot(t, d.h_skid, color=col, lw=1.1, label=name)
        axs[1].plot(t, -d.hdot, color=col, lw=1.1)
        axs[2].plot(t, d.V, color=col, lw=1.1)
    for ax, yl in zip(axs, ("Skid height [m]", "Sink rate [m/s]", "True airspeed V [m/s]")):
        ax.set_xlabel("Time to touchdown t [s]")
        ax.set_ylabel(yl, fontsize=8)
        ax.set_xlim(-8, 0.2)
    axs[0].set_ylim(-0.2, 8)
    axs[0].legend(frameon=False, fontsize=6)
    fig.suptitle("Figure 4. Skid height, sink rate and airspeed vs. time to touchdown, belly landing, by kit state and flare",
                 x=0.01, ha="left", fontsize=9)
    fig.text(0.01, 0.005, f"Motor off, 2,000 m, calm; flare at {FLARE_H:.0f} m to {FLARE_SINK} m/s sink", fontsize=7, color=MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 0.92))
    fig.savefig(OUT / "fig4_landing.png", dpi=180)
    plt.close(fig)


# ============================================================================ main
def main():
    t_start = time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    sys.stdout = Tee(OUT / "run_log.txt")
    print(f"Lodestar v1 6-DOF (JSBSim {jsbsim.__version__}), {v1.VERSION}, 2026-11-15; Monte Carlo runs per case {N_MC}\n")
    mm, sw = load(rebuild=True)
    on, off = mm["mi_on"], mm["mi_off"]
    print(f"Mass source: {mm['source']}")
    print(f"  kit on  {on['m']:.3f} kg, CG x {on['x']:.4f} z {on['z']:.4f} m, Ixx {on['Ixx']:.4f} Iyy {on['Iyy']:.4f} Izz {on['Izz']:.4f} kg m2")
    print(f"  kit off {off['m']:.3f} kg, CG x {off['x']:.4f} z {off['z']:.4f} m, Ixx {off['Ixx']:.4f} Iyy {off['Iyy']:.4f} Izz {off['Izz']:.4f} kg m2")
    print(f"  lodestar_v1: {v1.MASS['sicore_loaded_kg']} / {v1.MASS['sicore_released_kg']} kg, CG {v1.MASS['x_cg_sicore_loaded']} / {v1.MASS['x_cg_sicore_released']} m")
    print("\nAVL sweep (kit-on CG reference, trimmed):")
    print(sw[["CL_trim", "alpha_deg", "elevon_trim_deg", "CL0", "Cm0", "CLde", "Cmde", "Cmq", "Clb", "Cnb", "Clp", "Clda"]].round(4).to_string(index=False))
    pd.Series(dict(mass_on=on["m"], x_on=on["x"], z_on=on["z"], Ixx_on=on["Ixx"], Iyy_on=on["Iyy"], Izz_on=on["Izz"],
                   mass_off=off["m"], x_off=off["x"], z_off=off["z"], Ixx_off=off["Ixx"], Iyy_off=off["Iyy"], Izz_off=off["Izz"],
                   Ixz_off=off["Ixz"], kit_kg=sum(k.m for k in mm["kit"]))).to_csv(OUT / "model_mass.csv", header=["value"])

    chk = model_check(mm)
    chk.round(3).to_csv(OUT / "model_check.csv", index=False)
    print("\nModel check: JSBSim open-loop periods vs AVL eigenvalues (kit on, 20 m/s, 2,000 m)")
    print(chk.round(2).to_string(index=False))

    print("\n1. Trim across the envelope")
    ts = trim_sweep()
    ts.round(4).to_csv(OUT / "trim_envelope.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        print(ts[["alt_m", "kit", "V_tas", "V_over_Vs", "CL", "alpha_deg", "elevon_geo_deg", "pitch_budget_used_pct", "thrust_N",
                  "thrust_max_cont_N", "throttle_frac", "lift_over_weight", "flag"]].round(3).to_string(index=False))

    print(f"\n2. Dryden turbulence Monte Carlo ({N_MC} runs per case, 60 s each)")
    tsum, truns = turbulence_mc(N_MC)
    tsum.round(4).to_csv(OUT / "turbulence_summary.csv", index=False)
    truns.round(4).to_csv(OUT / "turbulence_runs.csv", index=False)

    print("\n3. Kit release at 40 m")
    rel, rel_tr = kit_release(max(10, N_MC // 2))
    rel.round(4).to_csv(OUT / "release_results.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        print(rel.round(2).to_string(index=False))

    print("\n4. Motor-out and glide polar")
    gp = glide_polar()
    gp.round(4).to_csv(OUT / "glide_polar.csv", index=False)
    bp = best_points(gp)
    bp.round(4).to_csv(OUT / "glide_best.csv", index=False)
    print(bp.round(3).to_string(index=False))
    mo, mo_tr = motor_out(bp)
    mo.round(4).to_csv(OUT / "motor_out.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        print(mo.round(3).to_string(index=False))

    print("\n5. Bungee launch")
    le = launcher_energy()
    le.round(3).to_csv(OUT / "launcher_energy.csv", index=False)
    print(le.round(2).to_string(index=False))
    lr, l_tr = launch_runs()
    lr.round(4).to_csv(OUT / "launch.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        print(lr.round(2).to_string(index=False))

    print("\n6. Belly landing")
    lrows, ld_tr = [], {}
    for kit in (False, True):
        for va in (1.2, 1.3):
            for flare in (True, False):
                r, d = landing(kit, va, flare)
                lrows.append(r)
                if va == 1.2:
                    ld_tr[f"kit {'on' if kit else 'off'}, 1.2 Vs, {'flare' if flare else 'no flare'}"] = d
    for k in range(max(10, N_MC // 2)):
        r, _ = landing(False, 1.2, True, turb="light", seed=SEED_BASE + 900 + k)
        lrows.append(r)
    lnd = pd.DataFrame(lrows)
    lnd.round(4).to_csv(OUT / "landing.csv", index=False)
    calm = lnd[lnd.turb == "calm"]
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        print(calm.round(3).to_string(index=False))
    lt = lnd[lnd.turb == "light"]
    print(f"  light turbulence, kit off, 1.2 Vs, flare, {len(lt)} runs: sink median {lt.touchdown_sink.median():.2f}, "
          f"P95 {lt.touchdown_sink.quantile(0.95):.2f}, max {lt.touchdown_sink.max():.2f} m/s; bank at touchdown P95 "
          f"{lt.phi_td.abs().quantile(0.95):.1f} deg")
    cases = []
    for _, r in calm.iterrows():
        cases.append((f"kit {'on' if r.kit else 'off'}, {r.V_app_over_vs:.1f} Vs, {'flare' if r.flare else 'no flare'}", r.mass_kg, r.touchdown_sink))
    cases.append(("light turbulence P95, kit off, 1.2 Vs, flare", lt.mass_kg.iloc[0], lt.touchdown_sink.quantile(0.95)))
    cases.append(("FEA design case (fea_extra), kit off", v1.MASS["sicore_released_kg"], 1.5))
    cases.append(("FEA design case (fea_extra), kit on", v1.MASS["sicore_loaded_kg"], 1.5))
    tc = tip_clearance()
    tc.round(3).to_csv(OUT / "tip_clearance.csv", index=False)
    print(tc.round(2).to_string(index=False))
    sk = skid_energy(cases)
    sk.round(3).to_csv(OUT / "skid_energy.csv", index=False)
    with pd.option_context("display.width", 300, "display.max_columns", 40):
        print(sk.round(2).T.to_string())

    figures(ts, tsum, rel_tr, gp, mo_tr, l_tr, ld_tr)
    print(f"\nRun time {(time.time() - t_start) / 60:.1f} min. Wrote out/sixdof_v1/")


if __name__ == "__main__":
    main()
