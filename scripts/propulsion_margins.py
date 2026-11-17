"""Step 1: propulsion and battery worst-case margins across the operating envelope.

Baseline (v1 working baseline, 2026-11-14; v1.1 adds lodestar_v1.V11_MASS below): compact_eps20_light wing (1.881 kg), v3 propulsion
(Aeronaut CAM 15x10 folding prop + T-Motor AT3520-550, 6S5P pack in the wing roots), "halfway" finish as in the brief.

Envelope: density altitude 0 / 2,000 / 4,400 m x ambient -10 / 0 / +20 / +40 C, plus hot-and-high
(3,000 m pressure altitude at ISA+20 C). Air density comes from the density altitude; the actual
temperature sets viscosity, speed of sound, battery and motor temperature. The grid is a bounding
set: not every combination is a real day (e.g. density altitude 0 m at +40 C).

Models (all vectorised over an RPM grid, so power AVAILABLE and power REQUIRED use the same chain):
  prop     UIUC measured CT, CP vs J (nearest tested RPM, as v3_propulsion.coeffs) plus the UIUC static
           run below the lowest tested J; 5% pusher installation loss on power (v3 convention).
  motor    Kv / Rm / I0 three-constant model, Rm rises 0.393%/K with winding temperature.
           Full-throttle motor voltage = K_FF x bus voltage, K_FF calibrated on T-Motor's own 100%-throttle
           bench rows (the ideal model over-predicts RPM by about 20% otherwise).
  ESC      95% efficient; rating 60 A (v3 assumption).
  battery  6S5P, open-circuit voltage vs state of charge (generic NMC curve shifted to each cell's nominal),
           DC resistance from the datasheet (P50B) or 2 x ACIR (SiCore; P50B's own DC/AC ratio is 1.97),
           Arrhenius temperature factor and a low-charge factor (ASSUMPTIONS), pack thermal resistance
           through 10 mm of EPS foam with self-heating.
  airframe the mission_sim / v3 drag model (NeuralFoil MH45 + induced + pod), load factor n for turns.

Run:  ../.venv/Scripts/python.exe -X utf8 propulsion_margins.py      (from scripts/)
Writes: out/propulsion_margins/*.csv, margins_log.txt, power_curves.png
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb

sys.path.insert(0, str(Path(__file__).parent))
import sizing as sz  # noqa: E402
import design_v2 as d2  # noqa: E402
import v3_propulsion as vp  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "propulsion_margins"
G = 9.81
R_AIR = 287.05

# ---------------------------------------------------------------- airframe (baseline brief)
B, AR = 3.0, 7.0
S, CHORD = B**2 / AR, B / AR
import lodestar_v1 as _v1  # noqa: E402
# Wing plus the masses the Step 1 fixes added (pack thermal kit, elevon balance), as structure mass.
WING_KG = _v1.STRUCTURE["wing_mass_kg"] + _v1.ADDED_MASS["pack_thermal_kit_kg"] + 2 * _v1.ADDED_MASS["elevon_balance_kg_each"]
# v1.1: the Step 2 mass changes (lodestar_v1.V11_MASS) and the v4 kit chute, as wing_trade performance_v1.csv.
WING_KG += sum(m for m, *_ in getattr(_v1, "V11_MASS", {}).values())
import v4_upgrade as _v4u  # noqa: E402
_v4u.apply_v11()   # v4 kit chute + v1.1 skid/strobe drag
KIT_RELEASE_KG = 7.993412697273528 - 6.369412697273528     # mass balance: kit 1.5 + chute 0.124
PF = d2.PF_MID                         # "halfway" finish, as the baseline brief (structures.performance_sensitivity)
CLMAX = sz.A.cl_max_3d
P_AV = sz.A.p_avionics_w
FINISH = "halfway"

# ---------------------------------------------------------------- propulsion (makers' data + assumptions)
PROP_KEY = "ancf_15x10"
D_PROP = 15 * 0.0254
KV, I0, RM20 = 550.0, 1.10, 0.031      # T-Motor AT3520 KV550 spec page
I_MOTOR_PEAK = 50.0                    # A for 180 s (T-Motor); no continuous rating published
P_MOTOR_PEAK = 1100.0                  # W for 180 s
K_FF = 0.80                            # full-throttle motor voltage / bus voltage, fitted to T-Motor bench rows
ESC_ETA, ESC_A = vp.ESC_ETA, 60.0
INSTALL = vp.INSTALL
RTH_MOTOR = 0.7                        # K/W winding-to-air, ASSUMPTION (T-Motor 14x7 bench: case +34 K at ~95 W loss after 3 min)
T_WIND_MAX = 120.0                     # C winding limit, ASSUMPTION (enamel 180 C class; magnets are the real limit)
RPM_MAX = 9000.0                       # Aeronaut lists 9,000 1/min for the CAM C 15x8 blade; same family, ASSUMED for 15x10
TIP_MACH_MAX = 0.60                    # efficiency/noise guideline, ASSUMPTION

# ---------------------------------------------------------------- cells
N_S, N_P = 6, 5
R_WIRE = 0.010                         # ohm, connectors + leads + ESC input, ASSUMPTION
V_CELL_FLOOR = 2.8                     # V loaded, datasheet cutoff 2.5 V + 0.3 V margin, ASSUMPTION
B_ARRH = 2500.0                        # K, resistance temperature factor (x2.2 at 0 C, x3.1 at -10 C), ASSUMPTION
SOC_END = 0.15                         # end of the usable 85% depth of discharge
INS_T, INS_K, PACK_AREA = 0.010, 0.035, 0.20   # EPS around the packs (cold-battery note geometry, EPS k)
PACK_CP = 900.0


@dataclass(frozen=True)
class Cell:
    name: str
    cap_ah: float
    v_nom: float
    r_dc25: float        # ohm, DC at 25 C mid charge
    i_cont: float        # A continuous
    i_pulse: float       # A, 30 s
    t_min: float         # C discharge range
    t_max: float
    source: str


CELLS = {
    "Amprius SiCore 6.3 Ah": Cell("SiCore (Amprius SA112 datasheet)", 6.5, 3.46, 2 * 0.022, 13.0, 26.0, -20.0, 60.0,
                                  "https://amprius.com/documents/Amprius_Product_Catalog.pdf"),
    "Molicel P50B": Cell("Molicel P50B", 5.0, 3.6, 0.0128, 60.0, 60.0, -40.0, 60.0,
                         "https://www.fsaeonline.com/content/Molicel-TDS--INR-21700-P50B-80122.pdf"),
}
SOC_TAB = np.array([0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00])
OCV_TAB = np.array([3.00, 3.30, 3.42, 3.50, 3.55, 3.62, 3.68, 3.74, 3.82, 3.90, 3.98, 4.07, 4.18])  # generic NMC
OCV_TAB_NOM = 3.60

# usable-energy factors (cold-battery note: 0.90 pre-warmed / 0.80 cold-soaked at -10 C; linear to 1.0 at +20 C)
CAP_FACTOR = {"pre-warmed": {-10: 0.90, 0: 0.95, 20: 1.00, 40: 1.00},
              "cold-soaked": {-10: 0.80, 0: 0.90, 20: 1.00, 40: 1.00}}


def cap_factor(t_c, state):
    k = CAP_FACTOR[state]
    xs = sorted(k)
    return float(np.interp(t_c, xs, [k[x] for x in xs]))


# ---------------------------------------------------------------- atmosphere
def isa_density(alt):
    return float(asb.Atmosphere(altitude=alt).density())


def density_altitude(rho):
    lo, hi = -2000.0, 12000.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if isa_density(mid) > rho else (lo, mid)
    return 0.5 * (lo + hi)


def air(da_m, t_c):
    """Air state: density from density altitude; viscosity and speed of sound from actual temperature."""
    T = t_c + 273.15
    mu = 1.458e-6 * T**1.5 / (T + 110.4)
    return dict(rho=isa_density(da_m), mu=mu, a=math.sqrt(1.4 * R_AIR * T), t_c=t_c, da=da_m)


def hot_and_high(pa_m=3000.0, dt=20.0):
    atm = asb.Atmosphere(altitude=pa_m)
    T = float(atm.temperature()) + dt
    rho = float(atm.pressure()) / (R_AIR * T)
    da = density_altitude(rho)
    a = air(da, T - 273.15)
    a["rho"] = rho
    return a


# ---------------------------------------------------------------- airframe drag
def drag(V, m, ai, n=1.0):
    V = np.asarray(V, float)
    q = 0.5 * ai["rho"] * V**2
    CL = n * m * G / (q * S)
    Re = ai["rho"] * V * CHORD / ai["mu"]
    CD = sz.cd_profile(np.minimum(CL, 1.2), Re) * PF + CL**2 / (math.pi * sz.A.e_oswald * AR) + d2.CDA_BODY / S
    return q * S * CD


def v_stall(m, ai, n=1.0):
    return math.sqrt(2 * n * m * G / (ai["rho"] * S * CLMAX))


# ---------------------------------------------------------------- propeller
def _load_prop():
    prop = vp.load_props()[PROP_KEY]
    vp.coeffs(prop, 4000, 0.4)                         # builds prop["groups"] (merged RPM runs)
    groups = [(r, t.J.values, t.CT.values, t.CP.values) for r, t in prop["groups"]]
    st = pd.read_csv(ROOT / "data" / "uiuc_props" / "ancf_15x10_static_0982od.txt", sep=r"\s+")
    return groups, st.RPM.values, st.CT.values, st.CP.values


GROUPS, ST_RPM, ST_CT, ST_CP = _load_prop()


def ctcp(rpm, J):
    """CT, CP at (rpm, J), vectorised. In the tested J range: the nearest tested RPM whose data covers J
    (v3_propulsion.coeffs rule). Below the lowest tested J: linear in J from the UIUC static run (J = 0)
    to the first tested point of the nearest-RPM run. Above the data: nan."""
    rpm, J = np.broadcast_arrays(np.asarray(rpm, float), np.asarray(J, float))
    ct = np.full(J.shape, np.nan)
    cp = np.full(J.shape, np.nan)
    best = np.full(J.shape, np.inf)
    for r, Jg, CTg, CPg in GROUPS:
        cov = (J >= Jg[0]) & (J <= Jg[-1])
        d = np.abs(rpm - r)
        use = cov & (d < best)
        ct = np.where(use, np.interp(J, Jg, CTg), ct)
        cp = np.where(use, np.interp(J, Jg, CPg), cp)
        best = np.where(use, d, best)
    near = np.argmin(np.abs(rpm[..., None] - np.array([g[0] for g in GROUPS])), axis=-1)
    j1 = np.array([g[1][0] for g in GROUPS])[near]
    ct1 = np.array([g[2][0] for g in GROUPS])[near]
    cp1 = np.array([g[3][0] for g in GROUPS])[near]
    low = np.isnan(ct) & (J < j1)
    ct0 = np.interp(rpm, ST_RPM, ST_CT)
    cp0 = np.interp(rpm, ST_RPM, ST_CP)
    f = np.clip(J / j1, 0, 1)
    ct = np.where(low, ct0 + (ct1 - ct0) * f, ct)
    cp = np.where(low, cp0 + (cp1 - cp0) * f, cp)
    return ct, cp


# ---------------------------------------------------------------- battery
def ocv(cell: Cell, soc):
    return np.interp(soc, SOC_TAB, OCV_TAB) + (cell.v_nom - OCV_TAB_NOM)


def r_cell(cell: Cell, t_c, soc):
    f_t = math.exp(B_ARRH * (1 / (t_c + 273.15) - 1 / 298.15))
    f_soc = 1 + 0.6 * max(0.0, 0.30 - soc) / 0.30
    return cell.r_dc25 * f_t * f_soc


def bus(cell, t_c, soc, P):
    """Pack current and bus voltage drawing power P (W). nan where the pack can't deliver P."""
    E = N_S * ocv(cell, soc)
    R = N_S * r_cell(cell, t_c, soc) / N_P + R_WIRE
    disc = E**2 - 4 * R * np.asarray(P, float)
    I = np.where(disc >= 0, (E - np.sqrt(np.maximum(disc, 0))) / (2 * R), np.nan)
    V = E - I * R
    v_cell = ocv(cell, soc) - I / N_P * r_cell(cell, t_c, soc)
    return I, V, v_cell


def pack_self_heat_temp(cell, t_amb, I_pack, soc=0.5, ins_t=INS_T):
    """Steady pack temperature with self-heating through the insulation (fixed point on R(T))."""
    rth = ins_t / (INS_K * PACK_AREA)
    T = t_amb
    for _ in range(30):
        q = N_S * N_P * (I_pack / N_P) ** 2 * r_cell(cell, T, soc)
        T = t_amb + q * rth
    return T, q


# ---------------------------------------------------------------- full chain on an RPM grid
RPM_GRID = np.arange(800.0, 12001.0, 5.0)


def chain(V, ai, cell, t_cell, soc, t_wind):
    """Every quantity of the prop-motor-ESC-battery chain at airspeed V on the RPM grid."""
    n = RPM_GRID / 60
    J = V / (n * D_PROP)
    ct, cp = ctcp(RPM_GRID, J)
    T = ct * ai["rho"] * n**2 * D_PROP**4
    P_shaft = cp * ai["rho"] * n**3 * D_PROP**5 / INSTALL
    w = RPM_GRID * 2 * math.pi / 60
    Q = P_shaft / w
    kv = KV * 2 * math.pi / 60
    rm = RM20 * (1 + 0.00393 * (t_wind - 20))
    I = Q * kv + I0
    U = w / kv + I * rm
    P_elec = U * I
    P_batt = P_elec / ESC_ETA + P_AV
    Ib, Vb, vc = bus(cell, t_cell, soc, P_batt)
    loss = I**2 * rm + I0 * U
    return dict(rpm=RPM_GRID, J=J, T=T, P_shaft=P_shaft, I=I, U=U, P_elec=P_elec, P_batt=P_batt, I_batt=Ib,
                V_bus=Vb, v_cell=vc, I_cell=Ib / N_P, eta_prop=np.where(P_shaft > 0, T * V / P_shaft, np.nan),
                tip_mach=math.pi * D_PROP * n / ai["a"], t_wind_ss=ai["t_c"] + RTH_MOTOR * loss, loss=loss)


def limits_ok(c, cell, short):
    """Boolean mask of RPM-grid points inside every limit. short = <= 30 s (launch, go-around)."""
    i_cell = cell.i_pulse if short else cell.i_cont
    ok = (np.isfinite(c["T"]) & (c["T"] > 0) & np.isfinite(c["I_batt"])
          & (c["U"] <= K_FF * c["V_bus"]) & (c["I"] <= I_MOTOR_PEAK) & (c["P_elec"] <= P_MOTOR_PEAK)
          & (c["I"] <= ESC_A) & (c["I_cell"] <= i_cell) & (c["v_cell"] >= V_CELL_FLOOR)
          & (c["rpm"] <= RPM_MAX) & (c["tip_mach"] <= TIP_MACH_MAX))
    if not short:
        ok &= c["t_wind_ss"] <= T_WIND_MAX
    return ok


def binding(c, cell, short, k):
    """Name of the limit that stops the RPM from rising past grid index k."""
    j = min(k + 1, len(c["rpm"]) - 1)
    i_cell = cell.i_pulse if short else cell.i_cont
    tests = [("motor voltage (throttle)", c["U"][j] > K_FF * c["V_bus"][j]),
             ("motor current", c["I"][j] > I_MOTOR_PEAK), ("motor power", c["P_elec"][j] > P_MOTOR_PEAK),
             ("ESC current", c["I"][j] > ESC_A), ("cell current", c["I_cell"][j] > i_cell),
             ("cell voltage floor", not (c["v_cell"][j] >= V_CELL_FLOOR)),
             ("prop RPM", c["rpm"][j] > RPM_MAX), ("tip Mach", c["tip_mach"][j] > TIP_MACH_MAX)]
    if not short:
        tests.append(("winding temperature", c["t_wind_ss"][j] > T_WIND_MAX))
    hit = [n for n, t in tests if t]
    return hit[0] if hit else "data range"


def available(V, ai, cell, t_cell, soc, short):
    """Maximum thrust inside all limits at airspeed V; winding temperature iterated to steady state."""
    t_w = ai["t_c"] + 60
    for _ in range(3):
        c = chain(V, ai, cell, t_cell, soc, t_w)
        ok = limits_ok(c, cell, short)
        if not ok.any():
            return None
        k = int(np.nanargmax(np.where(ok, c["T"], -np.inf)))
        t_w = c["t_wind_ss"][k]
    out = {key: (val[k] if isinstance(val, np.ndarray) else val) for key, val in c.items()}
    out["limit"] = binding(c, cell, short, k)
    return out


def required(V, Treq, ai, cell, t_cell, soc):
    """Operating point giving thrust Treq at V (interpolated on the RPM grid); None if unreachable."""
    t_w = ai["t_c"] + 30
    for _ in range(3):
        c = chain(V, ai, cell, t_cell, soc, t_w)
        Tg = np.where(np.isfinite(c["T"]), c["T"], -1.0)
        idx = np.nonzero(Tg >= Treq)[0]
        if len(idx) == 0 or idx[0] == 0:
            return None
        k = idx[0]
        f = (Treq - Tg[k - 1]) / (Tg[k] - Tg[k - 1])
        out = {}
        for key, val in c.items():
            out[key] = val[k - 1] + f * (val[k] - val[k - 1]) if isinstance(val, np.ndarray) else val
        t_w = out["t_wind_ss"]
    return out


# ---------------------------------------------------------------- aircraft mass and energy
def baseline(cell_name):
    import v3_mission as v3m
    mtow, wh = v3m.mass_and_energy(cell_name, "kit", B, AR, PF, wing_mass_kg=WING_KG)
    return mtow, wh


# ---------------------------------------------------------------- performance checks
def v_max_level(m, ai, cell, t_cell, soc, n=1.0):
    vs = np.arange(14.0, 40.01, 0.25)
    excess = []
    for V in vs:
        a = available(V, ai, cell, t_cell, soc, short=False)
        excess.append(-1.0 if a is None else a["T"] - float(drag(V, m, ai, n)))
    excess = np.array(excess)
    pos = np.nonzero(excess > 0)[0]
    if len(pos) == 0:
        return float("nan")
    k = pos[-1]
    if k == len(vs) - 1:
        return float(vs[-1])
    return float(vs[k] + 0.25 * excess[k] / (excess[k] - excess[k + 1]))


def climb(V, m, ai, cell, t_cell, soc, short, n=1.0):
    """Rate of climb (m/s), gradient and the available-thrust point at airspeed V, load factor n."""
    a = available(V, ai, cell, t_cell, soc, short)
    if a is None:
        return float("nan"), float("nan"), None
    sin_g = (a["T"] - float(drag(V, m, ai, n))) / (m * G)
    return V * sin_g, math.tan(math.asin(max(-1, min(1, sin_g)))), a


def launch(m, ai, cell, t_cell, soc, v0, h0=2.0, h_target=50.0, dt=0.05):
    """Bungee release at v0 (TAS), full power (short-time limits): accelerate level to 1.25 Vs, then climb.
    Returns time and ground distance to h_target, or nan if the aircraft is below stall at release."""
    vs = v_stall(m, ai)
    v_cl = 1.25 * vs
    if v0 < vs:
        return dict(t=float("nan"), x=float("nan"), v_release_over_vs=v0 / vs, v_climb=v_cl, roc=float("nan"))
    vg = np.arange(max(10.0, v0 - 1), v_cl + 1.01, 0.25)
    tav = np.array([(available(V, ai, cell, t_cell, soc, True) or {"T": 0.0})["T"] for V in vg])
    t = x = 0.0
    h, V = h0, v0
    while h < h_target and t < 300:
        T = float(np.interp(V, vg, tav))
        D = float(drag(V, m, ai))
        if V < v_cl:                      # level acceleration
            V += (T - D) / m * dt
            x += V * dt
            if T <= D:
                return dict(t=float("nan"), x=float("nan"), v_release_over_vs=v0 / vs, v_climb=v_cl, roc=0.0)
        else:
            sg = (T - D) / (m * G)
            h += V * sg * dt
            x += V * math.cos(math.asin(sg)) * dt
        t += dt
    roc = (float(np.interp(v_cl, vg, tav)) - float(drag(v_cl, m, ai))) / (m * G) * v_cl
    return dict(t=t if h >= h_target else float("nan"), x=x, v_release_over_vs=v0 / vs, v_climb=v_cl, roc=roc)


def cruise_power(V, m, ai, cell, t_cell, soc=0.5):
    r = required(V, float(drag(V, m, ai)), ai, cell, t_cell, soc)
    return r


def search_speed(cell_name, m, ai):
    planned = 20.0 if "SiCore" in cell_name else 18.0          # v3 / performance_eps20_light convention
    return max(planned, 1.25 * v_stall(m, ai))


def rth_energy(d_m, wind, m, ai, cell, t_cell, v_hi):
    """Best return into a headwind: minimise battery energy per ground metre, P(V) / (V - wind)."""
    best = None
    for V in np.arange(max(1.25 * v_stall(m, ai), wind + 1.0), v_hi + 0.01, 0.25):
        r = cruise_power(V, m, ai, cell, t_cell, SOC_END + 0.10)
        if r is None:
            continue
        e = r["P_batt"] * d_m / (V - wind) / 3600
        if best is None or e < best[0]:
            best = (e, V, r["P_batt"], d_m / (V - wind) / 60)
    return best


# ---------------------------------------------------------------- the envelope
DAS = (0.0, 2000.0, 4400.0)
TEMPS = (-10.0, 0.0, 20.0, 40.0)
V_BUNGEE = 14.0              # m/s TAS at release: "13-15 m/s routinely" (2026-11-12 wing redesign note), mid value
ROUTE_GRAD = 0.15            # terrain_sim.CLIMB_GRAD: the route planner climbs at up to 15% at route speed
R_SEARCH_MAX = 10_000.0      # m, scenario B box is 10 km out (mission_sim)
HEADWIND = 12.0


def corners():
    out = [(f"DA {int(da)} m, {int(t):+d} C", air(da, t)) for da in DAS for t in TEMPS]
    hh = hot_and_high()
    out.append((f"hot-and-high: PA 3000 m ISA+20 (DA {hh['da']:.0f} m), {hh['t_c']:+.1f} C", hh))
    return out


def run():
    OUT.mkdir(parents=True, exist_ok=True)
    perf, batt, endu, motor_rows = [], [], [], []
    for cell_name, cell in CELLS.items():
        m_kit, wh = baseline(cell_name)
        for label, ai in corners():
            t_amb = ai["t_c"]
            # cruise point (kit on, mid charge) sets the self-heating and the end-of-flight pack temperature
            m = m_kit
            Vs_srch = search_speed(cell_name, m, ai)
            cr = cruise_power(Vs_srch, m, ai, cell, max(t_amb, 0.0))
            t_pack_end, q_self = pack_self_heat_temp(cell, t_amb, cr["I_batt"], soc=0.3)
            t_launch_soak = t_amb
            for kit_on in (True, False):
                m = m_kit if kit_on else m_kit - KIT_RELEASE_KG
                vs = v_stall(m, ai)
                vmax = v_max_level(m, ai, cell, t_pack_end, SOC_END)
                v_srch = 18.0 if 1.25 * vs <= 18.0 else 1.25 * vs
                roc, grad, a_cl = climb(v_srch, m, ai, cell, t_pack_end, SOC_END, short=False)
                n30 = 1 / math.cos(math.radians(30))
                v_route = max(18.0, 1.25 * v_stall(m, ai, n30))
                roc_b, grad_b, a_b = climb(v_route, m, ai, cell, t_pack_end, SOC_END, short=False, n=n30)
                v_ga = 1.3 * vs
                roc_ga, grad_ga, a_ga = climb(v_ga, m, ai, cell, t_pack_end, SOC_END, short=True)
                row = dict(pack=cell_name, corner=label, da_m=ai["da"], t_c=t_amb, rho=ai["rho"], kit_on=kit_on,
                           mass_kg=m, v_stall=vs, v_max=vmax, headwind_gs_margin=vmax - HEADWIND,
                           v_climb=v_srch, roc_search=roc, v_route_bank30=v_route, roc_bank30=roc_b,
                           grad_bank30=grad_b, v_goaround=v_ga, roc_goaround=roc_ga, grad_goaround=grad_ga,
                           t_pack_end=t_pack_end)
                if kit_on:
                    stat =available(0.01, ai, cell, t_launch_soak, 1.0, short=True)
                    row["static_thrust_N"] = stat["T"] if stat else float("nan")
                    row["static_T_over_W"] = row["static_thrust_N"] / (m * G)
                    la = launch(m, ai, cell, t_launch_soak, 1.0, V_BUNGEE)
                    row.update(launch_v_over_vs=la["v_release_over_vs"], launch_t50=la["t"], launch_x50=la["x"],
                               launch_roc=la["roc"], bungee_needed_1p1vs=1.1 * vs)
                perf.append(row)
                # full-power points (what limits the power) and the points each requirement needs
                needs = {"needed: 22 m/s level": (22.0, 1.0, 0.0),
                         "needed: 3 m/s climb at 18 m/s": (v_srch, 1.0, 3.0),
                         "needed: 15% climb in 30 deg bank": (v_route, n30, ROUTE_GRAD * v_route)}
                pts = [("max level speed", None, False), ("climb at search", a_cl, False),
                       ("30 deg banked climb", a_b, False), ("go-around", a_ga, True)]
                for nm, (V, n, roc_need) in needs.items():
                    T_need = float(drag(V, m, ai, n)) + m * G * roc_need / V
                    pts.append((nm, required(V, T_need, ai, cell, t_pack_end, SOC_END), False))
                for case, pt, short in pts:
                    if case == "max level speed":
                        pt = available(vmax, ai, cell, t_pack_end, SOC_END, short=False) if np.isfinite(vmax) else None
                        if pt is not None:
                            pt["limit"] = "full power: " + pt["limit"]
                    if pt is None:
                        continue
                    if case.startswith("needed"):
                        pt["limit"] = "within limits" if limits_ok({k: np.atleast_1d(v) for k, v in pt.items()},
                                                                   cell, short)[0] else "EXCEEDS a limit"
                    motor_rows.append(dict(pack=cell_name, corner=label, t_c=t_amb, da_m=ai["da"], kit_on=kit_on,
                                           case=case, rpm=pt["rpm"], tip_mach=pt["tip_mach"], I_motor=pt["I"],
                                           U_motor=pt["U"], P_elec=pt["P_elec"], P_batt=pt["P_batt"],
                                           I_batt=pt["I_batt"], I_cell=pt["I_cell"], C_rate=pt["I_cell"] / cell.cap_ah,
                                           v_cell=pt["v_cell"], V_bus=pt["V_bus"], t_winding=pt["t_wind_ss"],
                                           eta_prop=pt["eta_prop"], limit=pt["limit"]))
            # battery and endurance (kit carried throughout, design-case rules)
            m = m_kit
            # requirement definition (v3_mission.design_case_endurance): least power between max(14, 1.25 Vs) and 20 m/s
            vgrid = np.arange(max(14.0, 1.25 * v_stall(m, ai)), 20.001, 0.5)
            pgrid = [cruise_power(V, m, ai, cell, max(t_amb, 0.0))["P_batt"] for V in vgrid]
            k_best = int(np.argmin(pgrid))
            for state in ("pre-warmed", "cold-soaked"):
                f = cap_factor(t_amb, state)
                e_use = wh * 0.85 * f * 0.90
                endu.append(dict(pack=cell_name, corner=label, da_m=ai["da"], t_c=t_amb, state=state, wh_total=wh,
                                 cap_factor=f, usable_wh_after_reserve=e_use, v_search=Vs_srch,
                                 P_batt_search=cr["P_batt"], endurance_h=e_use / cr["P_batt"],
                                 v_best=vgrid[k_best], P_batt_best=pgrid[k_best], endurance_best_h=e_use / pgrid[k_best],
                                 eta_prop=cr["eta_prop"], rpm=cr["rpm"], tip_mach=cr["tip_mach"]))
            batt.append(dict(pack=cell_name, corner=label, da_m=ai["da"], t_c=t_amb, I_cell_cruise=cr["I_cell"],
                             C_cruise=cr["I_cell"] / cell.cap_ah, self_heat_W=q_self, t_pack_end=t_pack_end,
                             v_cell_cruise_end=float(bus(cell, t_pack_end, SOC_END, cr["P_batt"])[2])))
    perf, batt, endu, motor = map(pd.DataFrame, (perf, batt, endu, motor_rows))
    return perf, batt, endu, motor


def heater_trade(cell_name, t_amb=-10.0, t_hold=15.0):
    """Hold the pack at t_hold with a heater vs fly it cold (2,000 m density altitude, kit on)."""
    cell = CELLS[cell_name]
    m, wh = baseline(cell_name)
    ai = air(2000.0, t_amb)
    V = search_speed(cell_name, m, ai)
    cr = cruise_power(V, m, ai, cell, 10.0)
    rows = []
    for ins in (0.010, 0.020, 0.030):
        rth = ins / (INS_K * PACK_AREA)
        _, q = pack_self_heat_temp(cell, t_hold, cr["I_batt"], soc=0.3, ins_t=ins)
        p_heat = max(0.0, (t_hold - t_amb) / rth - q)
        e = wh * 0.85 * cap_factor(t_hold, "pre-warmed") * 0.90
        rows.append(dict(pack=cell_name, insulation_mm=ins * 1000, R_th_K_per_W=rth, heater_W=p_heat,
                         tau_h=N_S * N_P * 0.072 / d2.PACK_EFF * PACK_CP * rth / 3600,
                         endurance_heated_h=e / (cr["P_batt"] + p_heat),
                         endurance_prewarmed_h=wh * 0.85 * cap_factor(t_amb, "pre-warmed") * 0.9 / cr["P_batt"],
                         endurance_soaked_h=wh * 0.85 * cap_factor(t_amb, "cold-soaked") * 0.9 / cr["P_batt"]))
    return pd.DataFrame(rows)


def fixes_table(da=4400.0, t_amb=-10.0):
    """SiCore climb at the worst corner (kit on, end of usable charge) under candidate fixes."""
    import dataclasses
    base = CELLS["Amprius SiCore 6.3 Ah"]
    m, _ = baseline("Amprius SiCore 6.3 Ah")
    ai = air(da, t_amb)
    cr = cruise_power(20.0, m, ai, base, 0.0)
    t_end, _ = pack_self_heat_temp(base, t_amb, cr["I_batt"], soc=0.3)
    n30 = 1 / math.cos(math.radians(30))
    v_route = max(18.0, 1.25 * v_stall(m, ai, n30))
    global V_CELL_FLOOR
    floor0 = V_CELL_FLOOR
    soc_hi = _v1.OPERATIONS["no_full_power_climb_below_soc_above_3000m_da"]
    rows = []
    cases = [("baseline: cells at end-of-flight temperature, 15% charge", base, t_end, SOC_END, floor0),
             ("pack held at 15 C (insulation + heater)", base, 15.0, SOC_END, floor0),
             ("climbs only above 25% charge (reserve untouched)", base, t_end, 0.25, floor0),
             ("15 C and above 25% charge (v1.0 rule)", base, 15.0, 0.25, floor0),
             (f"both: 15 C and above {soc_hi:.0%} charge (v1.1 rule above 3,000 m DA)", base, 15.0, soc_hi, floor0),
             ("SiCore DCIR 30 mOhm instead of 44 (measure it)", dataclasses.replace(base, r_dc25=0.030), t_end, SOC_END, floor0),
             ("loaded floor 2.5 V (datasheet cutoff, no margin)", base, t_end, SOC_END, 2.5),
             ("P50B pack instead", CELLS["Molicel P50B"], None, SOC_END, floor0)]
    for name, cell, t_cell, soc, floor in cases:
        V_CELL_FLOOR = floor
        mm = m
        if t_cell is None:
            mm, _ = baseline("Molicel P50B")
            crp = cruise_power(18.0, mm, ai, cell, 0.0)
            t_cell, _ = pack_self_heat_temp(cell, t_amb, crp["I_batt"], soc=0.3)
        roc, _, a = climb(18.0, mm, ai, cell, t_cell, soc, short=False)
        _, gb, _ = climb(v_route, mm, ai, cell, t_cell, soc, short=False, n=n30)
        rows.append(dict(case=name, t_cell=t_cell, soc=soc, roc_search=roc, grad_bank30=gb,
                         v_cell=a["v_cell"] if a else np.nan, limit=a["limit"] if a else ""))
    V_CELL_FLOOR = floor0
    return pd.DataFrame(rows)


def rth_table():
    rows = []
    for cell_name, cell in CELLS.items():
        m_kit, wh = baseline(cell_name)
        for label, da, t in (("design case", 2000.0, 20.0), ("worst: high and cold", 4400.0, -10.0)):
            ai = air(da, t)
            for kit_on in (True, False):
                m = m_kit if kit_on else m_kit - KIT_RELEASE_KG
                vmax = v_max_level(m, ai, cell, max(t, 0.0), SOC_END + 0.10)
                b = rth_energy(R_SEARCH_MAX, HEADWIND, m, ai, cell, max(t, 0.0), vmax)
                still = cruise_power(search_speed(cell_name, m, ai), m, ai, cell, max(t, 0.0))
                usable = wh * 0.85 * cap_factor(t, "pre-warmed")
                rows.append(dict(pack=cell_name, case=label, kit_on=kit_on, v_max=vmax,
                                 v_return=b[1] if b else np.nan, P_batt=b[2] if b else np.nan,
                                 return_min=b[3] if b else np.nan, E_return_Wh=b[0] if b else np.nan,
                                 E_return_pct_usable=100 * b[0] / usable if b else np.nan,
                                 reserve_10pct_Wh=0.10 * usable,
                                 E_still_air_Wh=still["P_batt"] * R_SEARCH_MAX / search_speed(cell_name, m, ai) / 3600))
    return pd.DataFrame(rows)


def margins(perf, motor, batt, endu, rth):
    """Each requirement or limit at its worst envelope corner, per pack. margin > 0 passes."""
    rows = []

    def add(req, pack, df, col, limit, kind, unit, src):
        d = df[df.pack == pack].dropna(subset=[col])
        i = d[col].idxmin() if kind == ">=" else d[col].idxmax()
        v = float(d.loc[i, col])
        corner = d.loc[i, "corner"] + (f", kit {'on' if d.loc[i, 'kit_on'] else 'off'}" if "kit_on" in d else "") \
            + (f", {d.loc[i, 'case']}" if "case" in d else "") + (f", {d.loc[i, 'state']}" if "state" in d else "")
        marg = (v - limit) if kind == ">=" else (limit - v)
        rows.append(dict(requirement=req, pack=pack, worst_corner=corner, value=v, limit=f"{kind} {limit:g}",
                         unit=unit, margin=marg, margin_pct=100 * marg / abs(limit) if limit else float("nan"), pass_=marg >= 0, basis=src))

    p_on = perf[perf.kit_on]
    for pack in CELLS:
        add("Max level speed, continuous power", pack, perf, "v_max", 22.0, ">=", "m/s", "requirement")
        add("Groundspeed into 12 m/s headwind at max speed", pack, perf, "headwind_gs_margin", 0.0, ">=", "m/s",
            "requirement (positive)")
        add("Climb rate at 18 m/s search speed", pack, perf, "roc_search", 3.0, ">=", "m/s", "sizing.A.climb_rate")
        add("Climb gradient in a 30 deg bank at route speed", pack, perf, "grad_bank30", ROUTE_GRAD, ">=", "-",
            "terrain_sim.CLIMB_GRAD 15%")
        add("Go-around climb gradient at 1.3 Vs", pack, perf, "grad_goaround", 1 / 30, ">=", "-",
            "CS-VLA 77 balked landing 1:30 (adapted)")
        add("Bungee release speed / stall speed (14 m/s release)", pack, p_on, "launch_v_over_vs", 1.10, ">=", "-",
            "1.1 Vs launch margin (ASSUMED)")
        lc = p_on.assign(launch_grad=p_on.launch_roc / (1.25 * p_on.v_stall))
        add("Launch climb gradient at 1.25 Vs", pack, lc, "launch_grad", 0.10, ">=", "-", "CS-VLA 65 take-off climb 1:10 (adapted)")
        add("Time from release to 50 m", pack, p_on, "launch_t50", 30.0, "<=", "s", "ASSUMED")
        need = motor[motor.case.str.startswith("needed")].copy()
        need["v_cell"] = need.v_cell.fillna(0.0)          # nan = the pack cannot deliver that power at all
        add("Motor current needed (22 m/s, 3 m/s climb, banked climb)", pack, need, "I_motor", I_MOTOR_PEAK, "<=", "A",
            "T-Motor 50 A / 180 s")
        add("Motor winding temperature needed (steady)", pack, need, "t_winding", T_WIND_MAX, "<=", "C", "ASSUMED limit")
        add("ESC current needed", pack, need, "I_motor", ESC_A, "<=", "A", "60 A class ESC (v3)")
        add("Prop RPM, any case incl. full power", pack, motor, "rpm", RPM_MAX, "<=", "rpm",
            "Aeronaut CAM C 15x8 rating, ASSUMED for 15x10")
        add("Prop tip Mach, any case incl. full power", pack, motor, "tip_mach", TIP_MACH_MAX, "<=", "-",
            "ASSUMED guideline")
        add("Cell current needed", pack, need, "I_cell", CELLS[pack].i_cont, "<=", "A", "datasheet continuous")
        add("Cell loaded voltage needed (end of charge)", pack, need, "v_cell", V_CELL_FLOOR, ">=", "V",
            "2.5 V datasheet cutoff + 0.3 V")
        add("Pack temperature, end of flight (hot)", pack, batt, "t_pack_end", CELLS[pack].t_max, "<=", "C",
            "datasheet discharge max")
        add("Pack temperature, end of flight (cold)", pack, batt, "t_pack_end", CELLS[pack].t_min, ">=", "C",
            "datasheet discharge min")
        e2 = endu[(endu.da_m == 2000.0) & (endu.t_c == 20.0)]
        add("Endurance, design case (2,000 m, +20 C, best speed 14-20 m/s)", pack, e2, "endurance_best_h", 3.0, ">=",
            "h", "requirement")
        add("Endurance, worst corner, best speed, pre-warmed", pack, endu[endu.state == "pre-warmed"],
            "endurance_best_h", 3.0, ">=", "h", "requirement applied envelope-wide")
        add("Endurance, worst corner, best speed, cold-soaked", pack, endu[endu.state == "cold-soaked"],
            "endurance_best_h", 3.0, ">=", "h", "requirement applied envelope-wide")
        r = rth[(rth.pack == pack)]
        r = r.assign(corner=r.case + r.kit_on.map({True: ", kit on", False: ", kit off"}),
                     spare=r.reserve_10pct_Wh - r.E_return_Wh).drop(columns=["case", "kit_on"])
        add("10 km return into 12 m/s headwind vs fixed 10% reserve", pack, r, "spare", 0.0, ">=", "Wh",
            "reserve logic")
    return pd.DataFrame(rows)


def chart(perf_cases):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    BLUE, ORANGE, INK, MUTED, GRID = "#2a78d6", "#eb6834", "#1f1f1e", "#6b6a64", "#e4e3de"
    fig, ax = plt.subplots(figsize=(7.5, 4.4))
    cell_name = "Amprius SiCore 6.3 Ah"
    cell = CELLS[cell_name]
    m, _ = baseline(cell_name)
    for (label, ai, t_cell), col in zip(perf_cases, (BLUE, ORANGE)):
        vs = np.arange(max(12.0, 1.0 * v_stall(m, ai)), 32.01, 0.5)
        preq = [(cruise_power(V, m, ai, cell, t_cell, SOC_END) or {"P_batt": np.nan})["P_batt"] for V in vs]
        pav = [(available(V, ai, cell, t_cell, SOC_END, False) or {"T": np.nan})["T"] * V for V in vs]
        dv = [float(drag(V, m, ai)) * V for V in vs]
        ax.plot(vs, dv, color=col, lw=1.8, label=f"{label}: thrust power required")
        ax.plot(vs, pav, color=col, lw=1.8, ls="--", label=f"{label}: thrust power available (continuous)")
    ax.axvline(22, color=MUTED, lw=1, ls=":")
    ax.text(22.2, 60, "22 m/s requirement", color=MUTED, fontsize=8)
    ax.set_xlabel("True airspeed V [m/s]")
    ax.set_ylabel("Thrust power [W]")
    ax.set_title("Figure 1. Thrust power required and available vs. airspeed, by envelope corner",
                 loc="left", fontsize=9, color=INK)
    ax.grid(color=GRID)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, fontsize=7.5, loc="lower right")
    fig.tight_layout()
    fig.savefig(OUT / "power_curves.png", dpi=150)


if __name__ == "__main__":
    from stability import Tee
    OUT.mkdir(parents=True, exist_ok=True)
    sys.stdout = Tee(OUT / "margins_log.txt")
    print("Lodestar propulsion and battery margins, 2026-11-14\n")
    perf, batt, endu, motor = run()
    heat = pd.concat([heater_trade(c) for c in CELLS], ignore_index=True)
    rth = rth_table()
    fixes = fixes_table()
    marg = margins(perf, motor, batt, endu, rth)
    for name, df in (("performance", perf), ("battery", batt), ("endurance", endu), ("motor_points", motor),
                     ("heater", heat), ("rth", rth), ("fixes_sicore_climb", fixes), ("margins", marg)):
        df.round(4).to_csv(OUT / f"{name}.csv", index=False)
    with pd.option_context("display.width", 250, "display.max_columns", 40, "display.max_rows", 200):
        print(perf.round(2).to_string(index=False))
        print(batt.round(3).to_string(index=False))
        print(endu.round(3).to_string(index=False))
        print(motor.round(2).to_string(index=False))
        print(heat.round(2).to_string(index=False))
        print(rth.round(2).to_string(index=False))
        print(fixes.round(3).to_string(index=False))
        print(marg.round(3).to_string(index=False))
    chart([("DA 0 m, +20 C", air(0.0, 20.0), 20.0), ("DA 4,400 m, -10 C", air(4400.0, -10.0), 0.0)])
    print("\nWritten: out/propulsion_margins/*.csv, power_curves.png")
