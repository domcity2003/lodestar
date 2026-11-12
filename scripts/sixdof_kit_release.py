"""Milestone 5c: 6-DOF simulation of the kit release in JSBSim.

The question: when 1.7 kg (23% of the aircraft's weight) drops out of the bay at 40 m above the ground,
does Lodestar stay controlled, and how much height does it gain or lose?

How:
  1. AVL (with the pod) gives the full set of stability and control derivatives at the release
     condition: v2 P50B layout, 15 m/s, 2,000 m altitude, kit on, balance point x = 0.422 m.
  2. This script writes a JSBSim aircraft from them (out/jsbsim/aircraft/lodestar/):
     - linear aero in all 6 axes, drag polar CDp + k CL^2 (both from the same AVL run)
     - kit = a JSBSim point mass at the bay; its weight goes to zero at release
     - bay-door drag as an external force at the door while it is open (1 s before to 2 s after)
     - pusher thrust as an external force on the thrust line
     - elevon and aileron servos: first-order lag (20/s) and 6 rad/s rate limit, +/-20 deg travel
  3. A simple autopilot at 50 Hz (altitude, airspeed, wings-level), like an ArduPilot-class
     fixed-wing autopilot in its basic modes.
  4. Checks the JSBSim model against AVL first (phugoid and Dutch roll periods from open-loop
     responses vs AVL's eigenvalues), then runs the release cases, then 100 runs in MIL-F-8785C
     moderate turbulence.

Run:  uv run python scripts/sixdof_kit_release.py
Writes: out/jsbsim/ (aircraft XML, csv results, charts, run_log.txt)
"""
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import jsbsim
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).parent))
import stability as st  # noqa: E402
import vsp_model as vm  # noqa: E402
import design_v2 as d2  # noqa: E402
from avl_run import run_trim, cg_and_inertia  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "jsbsim"
ACDIR = OUT / "aircraft" / "lodestar"
ACDIR.mkdir(parents=True, exist_ok=True)

FT, LB, KG2SLUG = 0.3048, 4.4482216, 1 / 14.5939
G0 = 9.80665
V_REL = 15.0               # m/s release speed (kit_drop.py)
H_GROUND = 2000.0          # m terrain elevation (design case altitude)
H_AGL = 40.0               # m release height (kit_drop.py: 95% within 25 m at about 40 m)
CDA_DOOR = 0.012           # m^2 open bay + door drag, ASSUMPTION (cavity CD ~0.5 on the 0.023 m^2 opening)
T_MAX = 20.0               # N, static-thrust-limited max at search speed (460 W x 0.6 / 15 m/s ~ 18 N)
DT = 1 / 200               # s, JSBSim step
AP_EVERY = 4               # autopilot at 50 Hz
N_MC = int(sys.argv[1]) if len(sys.argv) > 1 else 100   # turbulence runs
STALL_HOLD = 0.25          # s above CLmax counted as a stall (shorter gust spikes don't stall a real wing: dynamic-stall delay)
CL_MAX = 0.80              # 3D CLmax used in sizing (milestone 2 check: first strip reaches its 2D max ~0.9)

BLUE, ORANGE, GREY, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#8a9099", "#1f1f1e", "#6b6f76", "#e6e8eb"


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


# ---------------------------------------------------------------- 1. AVL derivatives at the release point
def avl_point():
    xr = [(0.0, 0.0)]
    for xl, w, h, rc in vm.POD_SECTIONS:
        xr.append((xl * vm.POD_LEN, math.sqrt((w * h - (4 - math.pi) * rc**2) / math.pi)))
    xr.append((vm.POD_LEN, 0.0))
    g = st.Geometry(c_root=vm.C_ROOT, c_tip=vm.C_TIP, washout_deg=vm.WASHOUT, winglet_h=0.25,
                    body_xr=tuple(xr), body_x0=vm.POD_X0, body_z0=vm.POD_Z)
    _, m_w = d2.pack("Molicel P50B", d2.N_WING)
    st.M_BATT, st.M_KIT = m_w, d2.M_KIT_TOTAL
    x_batt, x_kit = st.place_for_cg(g, 0.422)
    on = st.mass_items(g, x_batt, x_kit, kit_on=True)
    off = st.mass_items(g, x_batt, x_kit, kit_on=False)
    r = run_trim(g, on, V_REL, st.RHO, modes=True, cdp=st.CD_NONINDUCED)
    assert r.ok
    kit = [i for i in on if i.name.startswith("rescue kit")][0]
    return g, r, cg_and_inertia(on), cg_and_inertia(off), kit


def aero_coeffs(g, r):
    s = r.st
    a = math.radians(s["Alpha"])
    d2r = 180 / math.pi                                   # AVL control derivatives are per degree
    c = dict(
        CLa=s["CLa"], CLq=s["CLq"], CLde=s["CLd01"] * d2r,
        Cma=s["Cma"], Cmq=s["Cmq"], Cmde=s["Cmd01"] * d2r,
        CYb=s["CYb"], CYp=s["CYp"], CYr=s["CYr"], CYda=s["CYd02"] * d2r,
        Clb=s["Clb"], Clp=s["Clp"], Clr=s["Clr"], Clda=s["Cld02"] * d2r,
        Cnb=s["Cnb"], Cnp=s["Cnp"], Cnr=s["Cnr"], Cnda=s["Cnd02"] * d2r,
        CDp=s["CDvis"], k=s["CDind"] / s["CLtot"] ** 2,
    )
    de = math.radians(s["elevon"])
    c["CL0"] = s["CLtot"] - c["CLa"] * a - c["CLde"] * de
    c["Cm0"] = -(c["Cma"] * a + c["Cmde"] * de)
    c["alpha_trim_deg"], c["elevon_trim_deg"], c["CL_trim"] = s["Alpha"], s["elevon"], s["CLtot"]
    return c


# ---------------------------------------------------------------- 2. JSBSim aircraft
def lin_fn(name, terms, const=None):
    """<function> summing products. terms: list of (coef, [properties])."""
    parts = []
    if const is not None:
        parts.append(f"<value>{const:.6f}</value>")
    for coef, props in terms:
        inner = "".join(f"<property>{p}</property>" for p in props)
        parts.append(f"<product><value>{coef:.6f}</value>{inner}</product>")
    return f'<function name="{name}"><sum>{"".join(parts)}</sum></function>'


def axis_force(name, coef_prop, scale_props):
    inner = "".join(f"<property>{p}</property>" for p in scale_props)
    return (f'<axis name="{name}"><function name="aero/force/{name}"><product>'
            f"{inner}<property>{coef_prop}</property></product></function></axis>")


def write_aircraft(g, c, mi_on, mi_off, kit):
    a_ = "aero/alpha-rad"
    qh, ph, rh = "aero/qhat", "aero/phat", "aero/rhat"
    funcs = [
        '<function name="aero/qhat"><product><property>velocities/q-aero-rad_sec</property><property>aero/ci2vel</property></product></function>',
        '<function name="aero/phat"><product><property>velocities/p-aero-rad_sec</property><property>aero/bi2vel</property></product></function>',
        '<function name="aero/rhat"><product><property>velocities/r-aero-rad_sec</property><property>aero/bi2vel</property></product></function>',
        lin_fn("aero/cl", [(c["CLa"], [a_]), (c["CLq"], [qh]), (c["CLde"], ["fcs/elevon-pos-rad"])], c["CL0"]),
        f'<function name="aero/cd"><sum><value>{c["CDp"]:.6f}</value><product><value>{c["k"]:.6f}</value>'
        "<property>aero/cl</property><property>aero/cl</property></product></sum></function>",
        lin_fn("aero/cy", [(c["CYb"], ["aero/beta-rad"]), (c["CYp"], [ph]), (c["CYr"], [rh]), (c["CYda"], ["fcs/aileron-pos-rad"])]),
        lin_fn("aero/croll", [(c["Clb"], ["aero/beta-rad"]), (c["Clp"], [ph]), (c["Clr"], [rh]), (c["Clda"], ["fcs/aileron-pos-rad"])]),
        lin_fn("aero/cm", [(c["Cma"], [a_]), (c["Cmq"], [qh]), (c["Cmde"], ["fcs/elevon-pos-rad"])], c["Cm0"]),
        lin_fn("aero/cn", [(c["Cnb"], ["aero/beta-rad"]), (c["Cnp"], [ph]), (c["Cnr"], [rh]), (c["Cnda"], ["fcs/aileron-pos-rad"])]),
    ]
    qs = ["aero/qbar-psf", "metrics/Sw-sqft"]
    axes = [
        axis_force("LIFT", "aero/cl", qs), axis_force("DRAG", "aero/cd", qs), axis_force("SIDE", "aero/cy", qs),
        axis_force("ROLL", "aero/croll", qs + ["metrics/bw-ft"]),
        axis_force("PITCH", "aero/cm", qs + ["metrics/cbarw-ft"]),
        axis_force("YAW", "aero/cn", qs + ["metrics/bw-ft"]),
    ]

    def actuator(n):
        return (f'<actuator name="fcs/{n}-actuator"><input>fcs/{n}-cmd-rad</input><lag>20</lag>'
                f"<rate_limit>6.0</rate_limit><clipto><min>-0.35</min><max>0.35</max></clipto>"
                f"<output>fcs/{n}-pos-rad</output></actuator>")

    xml = f"""<?xml version="1.0"?>
<fdm_config name="lodestar" version="2.0" release="ALPHA">
  <fileheader><author>Lodestar design study</author><description>Lodestar v2, linear aero from AVL at {V_REL} m/s, 2026-11-11</description></fileheader>
  <metrics>
    <wingarea unit="M2">{g.S:.5f}</wingarea>
    <wingspan unit="M">{g.span:.5f}</wingspan>
    <chord unit="M">{g.mac:.5f}</chord>
    <location name="AERORP" unit="M"><x>{mi_on['x']:.5f}</x><y>0</y><z>{mi_on['z']:.5f}</z></location>
    <location name="EYEPOINT" unit="M"><x>0.05</x><y>0</y><z>-0.11</z></location>
    <location name="VRP" unit="M"><x>0</x><y>0</y><z>0</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="KG*M2">{mi_off['Ixx']:.5f}</ixx>
    <iyy unit="KG*M2">{mi_off['Iyy']:.5f}</iyy>
    <izz unit="KG*M2">{mi_off['Izz']:.5f}</izz>
    <ixz unit="KG*M2">0</ixz>
    <emptywt unit="KG">{mi_off['m']:.5f}</emptywt>
    <location name="CG" unit="M"><x>{mi_off['x']:.5f}</x><y>0</y><z>{mi_off['z']:.5f}</z></location>
    <pointmass name="rescue kit">
      <weight unit="KG">{kit.m:.5f}</weight>
      <location unit="M"><x>{kit.x:.5f}</x><y>0</y><z>{kit.z:.5f}</z></location>
    </pointmass>
  </mass_balance>
  <ground_reactions>
    <contact type="STRUCTURE" name="NOSE"><location unit="M"><x>0.0</x><y>0</y><z>-0.16</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction>
      <spring_coeff unit="N/M">3000</spring_coeff><damping_coeff unit="N/M/SEC">300</damping_coeff></contact>
    <contact type="STRUCTURE" name="LEFT_TIP"><location unit="M"><x>0.9</x><y>-1.5</y><z>0</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction>
      <spring_coeff unit="N/M">3000</spring_coeff><damping_coeff unit="N/M/SEC">300</damping_coeff></contact>
    <contact type="STRUCTURE" name="RIGHT_TIP"><location unit="M"><x>0.9</x><y>1.5</y><z>0</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction>
      <spring_coeff unit="N/M">3000</spring_coeff><damping_coeff unit="N/M/SEC">300</damping_coeff></contact>
  </ground_reactions>
  <propulsion/>
  <external_reactions>
    <force name="thrust" frame="BODY" unit="LBS">
      <location unit="M"><x>0.68</x><y>0</y><z>-0.035</z></location>
      <direction><x>1</x><y>0</y><z>0</z></direction>
    </force>
    <force name="door" frame="BODY" unit="LBS">
      <location unit="M"><x>{kit.x:.4f}</x><y>0</y><z>-0.11</z></location>
      <direction><x>-1</x><y>0</y><z>0</z></direction>
    </force>
  </external_reactions>
  <flight_control name="fcs">
    <property value="0">fcs/elevon-cmd-rad</property>
    <property value="0">fcs/aileron-cmd-rad</property>
    <channel name="servos">
      {actuator("elevon")}
      {actuator("aileron")}
    </channel>
  </flight_control>
  <aerodynamics>
    {''.join(funcs)}
    {''.join(axes)}
  </aerodynamics>
</fdm_config>
"""
    (ACDIR / "lodestar.xml").write_text(xml)


# ---------------------------------------------------------------- 3. simulation + autopilot
class Sim:
    def __init__(self, c, turb=None, seed=0, door_cda=CDA_DOOR, v=V_REL, sea_level_ground=False):
        """sea_level_ground: put the terrain at sea level and warm the air so its density equals 2,000 m.
        Needed for turbulence: JSBSim picks the MIL-F-8785C low/high-altitude turbulence model from the
        altitude above SEA LEVEL (checked: 40 m over 2,000 m terrain gives the same gusts as 2,040 m over
        sea-level terrain), so at 2,000 m terrain it would apply the high-altitude model at 40 m above
        the ground. The flight dynamics here depend only on density (no Mach, no engine model)."""
        self.f = jsbsim.FGFDMExec(str(OUT), None)
        self.f.set_debug_level(0)
        self.f.set_dt(DT)
        assert self.f.load_model("lodestar")
        f = self.f
        self.ground = 0.0 if sea_level_ground else H_GROUND
        if sea_level_ground:
            t_k = 101325.0 / (287.053 * st.RHO)               # temperature that gives 2,000 m density at sea level
            f["atmosphere/delta-T"] = (t_k - 288.15) * 1.8    # Rankine
        f["ic/terrain-elevation-ft"] = self.ground / FT
        f["ic/h-sl-ft"] = (self.ground + H_AGL) / FT
        f["ic/vt-fps"] = v / FT
        f["ic/alpha-deg"] = c["alpha_trim_deg"]
        f["ic/gamma-deg"] = 0.0
        f["ic/psi-true-deg"] = 0.0
        f["fcs/elevon-cmd-rad"] = math.radians(c["elevon_trim_deg"])
        if turb:
            f["atmosphere/turb-type"] = 3                       # MIL-F-8785C (Dryden)
            f["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = turb["w20_kt"] * 1.68781
            f["atmosphere/turbulence/milspec/severity"] = turb["severity"]
            try:
                f["simulation/randomseed"] = seed
            except Exception:
                pass
        f.run_ic()
        self.c, self.v_cmd, self.door_cda = c, v, door_cda
        self.h_cmd = self.ground + H_AGL
        self.rho = f["atmosphere/rho-slugs_ft3"] * 515.379
        self.h_min, self.cl_peak = 1e9, -1e9
        self.over_run = self.over_max = 0.0     # s: current / longest continuous time with CL above CLmax
        self.ap_on = True
        self.i_v = self.i_h = self.i_th = 0.0
        cl = c["CL_trim"] * (V_REL / v) ** 2
        self.T = 0.5 * st.RHO * v**2 * 1.2857 * (c["CDp"] + c["k"] * cl**2)
        self.de0 = math.radians(c["elevon_trim_deg"])
        self.de, self.da = self.de0, 0.0
        self.frozen = None
        self.door = False
        self.n = 0
        self.log = []

    def get(self, p):
        return self.f[p]

    def autopilot(self):
        f, dt = self.f, DT * AP_EVERY
        V = f["velocities/vt-fps"] * FT
        h = f["position/h-sl-ft"] * FT
        hdot = f["velocities/h-dot-fps"] * FT
        th, q = f["attitude/theta-rad"], f["velocities/q-rad_sec"]
        ph, p = f["attitude/phi-rad"], f["velocities/p-rad_sec"]
        # airspeed -> thrust (PI)
        ev = self.v_cmd - V
        self.i_v = np.clip(self.i_v + ev * dt, -10, 10)
        self.T = float(np.clip(self.T0 + 2.0 * ev + 0.6 * self.i_v, 0, T_MAX))
        # altitude -> pitch command -> elevon (PID outer, PI + rate damping inner)
        eh = self.h_cmd - h
        self.i_h = np.clip(self.i_h + eh * dt, -20, 20)
        th_cmd = float(np.clip(self.th0 + 0.03 * eh + 0.004 * self.i_h - 0.04 * hdot, math.radians(-12), math.radians(15)))
        et = th - th_cmd
        self.i_th = np.clip(self.i_th + et * dt, -0.3, 0.3)
        # Airspeed gain scaling (as ArduPilot does): control power grows with V^2, so scale the surface
        # commands by (V_trim / V)^2. Without it, gusts that push the speed to 20+ m/s drove the pitch loop
        # into a 3 Hz limit cycle against the servo lag and rate limit (found in the first turbulence runs).
        sc = float(np.clip((self.v_cmd / max(V, 5.0)) ** 2, 0.4, 2.0))
        self.de = float(self.de0 + sc * (1.0 * et + 0.8 * self.i_th + 0.25 * q))
        # wings level
        self.da = float(sc * (0.6 * ph + 0.12 * p))

    def step(self, t_end, release_at=None, door_window=(1.0, 2.0)):
        f = self.f
        while f["simulation/sim-time-sec"] < t_end:
            t = f["simulation/sim-time-sec"]
            if release_at is not None:
                self.door = release_at - door_window[0] <= t <= release_at + door_window[1]
                if t >= release_at and f["inertia/pointmass-weight-lbs[0]"] > 0:
                    f["inertia/pointmass-weight-lbs[0]"] = 0.0
            if self.n % AP_EVERY == 0 and self.ap_on:
                self.autopilot()
            f["fcs/elevon-cmd-rad"] = self.de
            f["fcs/aileron-cmd-rad"] = self.da
            f["external_reactions/thrust/magnitude"] = self.T / LB
            f["external_reactions/door/magnitude"] = (f["aero/qbar-psf"] * 47.880 * self.door_cda / LB) if self.door else 0.0
            f.run()
            self.n += 1
            self.h_min = min(self.h_min, f["position/h-agl-ft"] * FT)
            self.cl_peak = max(self.cl_peak, f["aero/cl"])
            self.over_run = self.over_run + DT if f["aero/cl"] > CL_MAX else 0.0
            self.over_max = max(self.over_max, self.over_run)
            if self.n % 4 == 0:
                self.log.append(dict(
                    t=t, h_agl=f["position/h-agl-ft"] * FT, V=f["velocities/vt-fps"] * FT,
                    theta=math.degrees(f["attitude/theta-rad"]), phi=math.degrees(f["attitude/phi-rad"]),
                    alpha=math.degrees(f["aero/alpha-rad"]), cl=f["aero/cl"], q=f["velocities/q-rad_sec"],
                    nz=-f["accelerations/n-pilot-z-norm"],
                    elevon=math.degrees(f["fcs/elevon-pos-rad"]), aileron=math.degrees(f["fcs/aileron-pos-rad"]),
                    thrust=self.T, mass=f["inertia/weight-lbs"] * LB / G0, x_cg=f["inertia/cg-x-in"] * 0.0254,
                    door=self.door, beta=math.degrees(f["aero/beta-rad"])))

    def settle(self, t=30.0, trim=None):
        """Fly level on autopilot until trimmed, then remember the trim. In turbulence, pass the calm-air
        trim (T0, th0, de0) so the autopilot's reference isn't a snapshot of a gust."""
        if trim:
            self.T0, self.th0, self.de0 = trim
            self.T, self.de = self.T0, self.de0
            self.step(t)
        else:
            self.T0, self.th0 = self.T, math.radians(self.c["alpha_trim_deg"])
            self.step(t)
            self.T0 = self.T
            self.th0 = self.f["attitude/theta-rad"]
            self.de0 = self.de
            self.i_v = self.i_h = self.i_th = 0.0
        self.log.clear()

    def trim(self):
        return self.T0, self.th0, self.de0


def metrics(df, t_rel):
    a = df[df.t >= t_rel]
    h0 = df[df.t < t_rel - 1.2].h_agl.iloc[-25:].mean()
    th0 = df[df.t < t_rel - 1.2].theta.iloc[-25:].mean()
    dev = (a.h_agl - h0).abs()
    outside = a[dev > 1.0]
    settle = (outside.t.max() - t_rel) if len(outside) else 0.0
    return dict(h_gain_max_m=(a.h_agl - h0).max(), h_loss_max_m=(h0 - a.h_agl).max(), min_h_agl_m=df[df.t >= t_rel - 1.0].h_agl.min(),
                pitch_dev_max_deg=(a.theta - th0).abs().max(), cl_max=df[df.t >= t_rel - 1.0].cl.max(),
                v_min=df[df.t >= t_rel - 1.0].V.min(), phi_max_deg=a.phi.abs().max(),
                elevon_range_deg=a.elevon.max() - a.elevon.min(), nz_max=a.nz.max(), nz_min=a.nz.min(),
                settle_s=settle)


def oscillation_period(t, y):
    y = y - np.mean(y)
    zc = t[1:][(y[:-1] < 0) & (y[1:] >= 0)]
    return float(np.mean(np.diff(zc))) if len(zc) >= 3 else float("nan")


def main():
    sys.stdout = Tee(OUT / "run_log.txt")
    print("Lodestar v2: 6-DOF kit release in JSBSim, 2026-11-11\n")
    g, r, mi_on, mi_off, kit = avl_point()
    c = aero_coeffs(g, r)
    write_aircraft(g, c, mi_on, mi_off, kit)
    pd.Series(c).to_csv(OUT / "aero_coeffs.csv", header=["value"])
    print(f"AVL point: {V_REL} m/s, {H_GROUND:.0f} m, mass {mi_on['m']:.2f} kg (kit on) / {mi_off['m']:.2f} kg (kit off), "
          f"CG x {mi_on['x']:.4f} / {mi_off['x']:.4f} m, alpha {c['alpha_trim_deg']:.2f} deg, elevon {c['elevon_trim_deg']:.2f} deg")
    print(f"  CLa {c['CLa']:.2f}/rad, Cma {c['Cma']:.3f}, Cmq {c['Cmq']:.2f}, Cmde {c['Cmde']:.3f}/rad, "
          f"Cnb {c['Cnb']:.4f}, Clb {c['Clb']:.4f}, CDp {c['CDp']:.4f}, k {c['k']:.4f}")

    # ---- check 1: open-loop modes vs AVL
    eig = r.eig
    pairs = sorted([e for e in eig if e.imag > 1e-6], key=abs)
    ph_avl, dr_avl = 2 * math.pi / pairs[0].imag, 2 * math.pi / pairs[1].imag
    s = Sim(c)
    s.settle(40)
    s.ap_on = False
    s.de += math.radians(2.0)
    s.step(s.f["simulation/sim-time-sec"] + 0.5)
    s.de -= math.radians(2.0)
    s.step(s.f["simulation/sim-time-sec"] + 40)
    d = pd.DataFrame(s.log)
    ph_js = oscillation_period(d.t.values, d.V.values)
    s = Sim(c)
    s.settle(40)
    s.ap_on = False
    s.da += math.radians(3.0)
    s.step(s.f["simulation/sim-time-sec"] + 0.3)
    s.da -= math.radians(3.0)
    s.step(s.f["simulation/sim-time-sec"] + 8)
    d = pd.DataFrame(s.log)
    dr_js = oscillation_period(d.t.values, d.beta.values)
    chk = pd.DataFrame([dict(mode="phugoid", avl_period_s=ph_avl, jsbsim_period_s=ph_js),
                        dict(mode="Dutch roll", avl_period_s=dr_avl, jsbsim_period_s=dr_js)])
    chk["diff_pct"] = (chk.jsbsim_period_s / chk.avl_period_s - 1) * 100
    chk.round(3).to_csv(OUT / "model_check.csv", index=False)
    print("\nCheck: JSBSim open-loop periods vs AVL eigenvalues (same aircraft, kit on)")
    print(chk.round(2).to_string(index=False))

    # ---- release cases
    T_REL = 5.0
    cases = [
        ("autopilot, calm, 15 m/s", dict(), True),
        ("autopilot, calm, 13 m/s", dict(v=13.0), True),
        ("autopilot, calm, door drag x2", dict(door_cda=2 * CDA_DOOR), True),
        ("controls frozen (autopilot off at release)", dict(), False),
    ]
    rows, traces = [], {}
    calm = Sim(c)
    calm.settle(40)
    calm_trim = calm.trim()
    print("")
    print(f"Calm trim at 15 m/s: thrust {calm_trim[0]:.2f} N, pitch {math.degrees(calm_trim[1]):.2f} deg, "
          f"elevon {math.degrees(calm_trim[2]):.2f} deg")
    for name, kw, ap in cases:
        s = Sim(c, **kw)
        s.settle(40)
        t0 = s.f["simulation/sim-time-sec"]
        if not ap:
            s.step(t0 + T_REL - 0.01)
            s.ap_on = False
            s.step(t0 + T_REL + 30, release_at=t0 + T_REL)
        else:
            s.step(t0 + T_REL + 30, release_at=t0 + T_REL)
        df = pd.DataFrame(s.log)
        df["t"] -= t0 + T_REL
        traces[name] = df
        rows.append(dict(case=name, **metrics(df, 0.0)))
    res = pd.DataFrame(rows)

    # ---- turbulence Monte Carlo (MIL-F-8785C Dryden, low-altitude form: sigma_w = 0.1 x wind at 20 ft,
    # scale length = height). Light = 15 kt at 20 ft (inside the mission's 0-10 m/s winds); moderate = 30 kt
    # (15 m/s: as fast as the aircraft flies, outside its envelope). Stress test: JSBSim's high-altitude model
    # applied at 40 m (terrain at 2,000 m), i.e. downdrafts lasting 10 s or more, a stand-in for mountain sink.
    trims = {V_REL: calm_trim}
    s18 = Sim(c, v=18.0)
    s18.settle(40)
    trims[18.0] = s18.trim()

    def run_mc(turb, sea_level_ground, v, n):
        out, tr = [], {}
        for seed in range(n):
            s = Sim(c, turb=turb, seed=seed + 1, sea_level_ground=sea_level_ground, v=v)
            s.settle(30, trim=trims[v])
            pre_hmin, pre_over = s.h_min, s.over_max
            t0 = s.f["simulation/sim-time-sec"]
            s.step(t0 + T_REL + 20, release_at=t0 + T_REL)
            df = pd.DataFrame(s.log)
            df["t"] -= t0 + T_REL
            m = metrics(df, 0.0)
            m["min_h_agl_whole_run_m"] = min(pre_hmin, s.h_min)
            m["longest_over_clmax_s"] = s.over_max
            m["ground_hit"] = m["min_h_agl_whole_run_m"] < 0.5
            m["stall"] = s.over_max > STALL_HOLD
            m["stall_before_release"] = pre_over > STALL_HOLD
            out.append(dict(seed=seed + 1, **m))
            if seed < 30:
                tr[seed] = df
        return pd.DataFrame(out), tr

    summ = {}
    for key, turb, slg, v, label in (
            ("light", dict(w20_kt=15.0, severity=3), True, 15.0, "light turbulence (15 kt), 15 m/s"),
            ("moderate", dict(w20_kt=30.0, severity=3), True, 15.0, "moderate turbulence (30 kt), 15 m/s"),
            ("moderate_18", dict(w20_kt=30.0, severity=3), True, 18.0, "moderate turbulence (30 kt), drop at 18 m/s"),
            ("stress", dict(w20_kt=30.0, severity=3), False, 15.0, "stress test: long downdrafts, 15 m/s")):
        mcd, tr = run_mc(turb, slg, v, N_MC)
        mcd.round(4).to_csv(OUT / f"turbulence_mc_{key}.csv", index=False)
        ok = mcd[~mcd.ground_hit & ~mcd.stall]
        cols = [k for k in mcd.columns if k not in ("seed", "ground_hit", "stall", "stall_before_release")]
        if len(ok):
            q = ok[cols].quantile([0.5, 0.95])
            rows.append(dict(case=f"{label}: median", **q.loc[0.5].to_dict()))
            rows.append(dict(case=f"{label}: 95th percentile", **q.loc[0.95].to_dict()))
        summ[key] = dict(case=label, runs=len(mcd), ground_hits=int(mcd.ground_hit.sum()), stalls=int(mcd.stall.sum()),
                         stalls_before_release=int(mcd.stall_before_release.sum()),
                         lowest_h_clean_runs_m=float(ok.min_h_agl_whole_run_m.min()) if len(ok) else float("nan"),
                         p95_h_loss_clean_m=float(ok.h_loss_max_m.quantile(0.95)) if len(ok) else float("nan"),
                         traces=tr)
        print("")
        print(f"Turbulence, {label}: {len(mcd)} runs: ground hits {summ[key]['ground_hits']}, stalls "
              f"(CL > CLmax for > {STALL_HOLD} s) {summ[key]['stalls']} ({summ[key]['stalls_before_release']} began before "
              f"the release); lowest height in the clean runs {summ[key]['lowest_h_clean_runs_m']:.1f} m")
    tsum = pd.DataFrame([{k: v for k, v in s_.items() if k != "traces"} for s_ in summ.values()])
    tsum.round(3).to_csv(OUT / "turbulence_summary.csv", index=False)

    res = pd.DataFrame(rows)
    res.round(3).to_csv(OUT / "release_results.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 30):
        print("")
        print("Kit release at 40 m above ground (t = 0); turbulence rows exclude ground hits and CL > CLmax runs:")
        print(res.round(2).to_string(index=False))
    calm = res.iloc[0]
    print("")
    print(f"Calm, autopilot: climbs at most {calm.h_gain_max_m:.1f} m, never below {calm.min_h_agl_m:.1f} m above ground, "
          f"peak CL {calm.cl_max:.2f} (CLmax {CL_MAX}), back within 1 m of the set height after {calm.settle_s:.1f} s")

    # ---- charts
    fig, axs = plt.subplots(2, 2, figsize=(11, 6.6), sharex=True)
    for name, col, lab in (("autopilot, calm, 15 m/s", BLUE, "autopilot on"),
                           ("controls frozen (autopilot off at release)", ORANGE, "controls frozen")):
        df = traces[name]
        for ax, k, yl in ((axs[0, 0], "h_agl", "Height above ground (m)"), (axs[0, 1], "theta", "Pitch (deg)"),
                          (axs[1, 0], "V", "Airspeed (m/s)"), (axs[1, 1], "elevon", "Elevon (deg, + = trailing edge down)")):
            ax.plot(df.t, df[k], color=col, lw=2, label=lab)
            ax.set_ylabel(yl, color=INK)
    for ax in axs.flat:
        ax.axvline(0, color=MUTED, lw=1, ls="--")
        ax.axvspan(-1, 2, color=GRID, zorder=0)
        ax.grid(color=GRID)
        ax.set_xlim(-5, 30)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axs[1, 0].set_xlabel("Seconds after release")
    axs[1, 1].set_xlabel("Seconds after release")
    axs[0, 0].legend(frameon=False, loc="upper right")
    axs[0, 0].text(0.3, axs[0, 0].get_ylim()[0] + 0.3, "kit drops", color=MUTED, fontsize=9)
    fig.suptitle("Kit release at 40 m, 15 m/s: the aircraft loses 23% of its weight in an instant (grey band: bay door open)",
                 color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "release_time_history.png", dpi=160)

    fig, axs = plt.subplots(1, 3, figsize=(14, 4.6), sharey=True)
    for ax, key, col in zip(axs, ("light", "moderate_18", "stress"), (BLUE, BLUE, ORANGE)):
        s_ = summ[key]
        for i, df in s_["traces"].items():
            ax.plot(df.t, df.h_agl, color=col, lw=1, alpha=0.35,
                    label=f"{len(s_['traces'])} of {s_['runs']} runs" if i == 0 else None)
        df = traces["autopilot, calm, 15 m/s"]
        ax.plot(df.t, df.h_agl, color=INK, lw=2, label="calm air")
        ax.axhline(0, color=INK, lw=1)
        ax.axvline(0, color=MUTED, lw=1, ls="--")
        ax.set_ylim(0, 60)
        ax.set_xlim(-5, 20)
        ax.set_xlabel("Seconds after release")
        ax.grid(color=GRID)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        ax.legend(frameon=False, loc="lower right")
        ax.set_title(s_["case"] + chr(10) + f"ground hits {s_['ground_hits']}, stalls {s_['stalls']} of {s_['runs']}",
                     color=INK, loc="left", fontsize=10)
    axs[0].set_ylabel("Height above ground (m)")
    fig.suptitle("Height through the kit release at 40 m", color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(OUT / "release_turbulence.png", dpi=160)
    print("\nWrote out/jsbsim/")


if __name__ == "__main__":
    main()
