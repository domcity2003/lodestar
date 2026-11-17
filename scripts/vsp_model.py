"""Milestone 3: 3D OpenVSP model of the Lodestar, with VSPAERO and parasite-drag checks.

What it does:
  1. Writes an OpenVSP script (vspscript) that builds the aircraft from the milestone 1-2 numbers:
     swept MH45 wing with 3 deg washout, 0.25 m winglets, a centre pod sized around the kit bay,
     and the camera gimbal ball under the nose.
  2. Runs it with vspscript.exe (OpenVSP 3.56, no GUI) to save a .vsp3 model and an STL mesh.
  3. Runs OpenVSP's parasite-drag build-up (skin friction x form factor, per component).
  4. Runs VSPAERO (panel method on the thick surfaces) for an alpha sweep, with and without the
     pod, to find how much the pod moves the neutral point.

Run:  uv run python scripts/vsp_model.py
Writes: out/vsp/<case>/ (vsp3, stl, VSPAERO files), out/vsp/*.csv, run_log.txt
"""
from pathlib import Path
import math
import subprocess
import sys

import numpy as np
import pandas as pd
import aerosandbox as asb

sys.path.insert(0, str(Path(__file__).parent))

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "vsp"
OUT.mkdir(parents=True, exist_ok=True)
VSP = ROOT.parent / "aero-tools" / "OpenVSP-3.56.0-win64"
VSPSCRIPT = VSP / "vspscript.exe"


class Tee:
    def __init__(self, path):
        self.f = open(path, "w", encoding="utf-8")

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()



# ---------------------------------------------------------------- inputs from milestones 1-2
DP = pd.read_csv(ROOT / "out" / "sizing" / "design_point.csv", index_col=0)["value"]
FC = pd.read_csv(ROOT / "out" / "stability" / "final_config.csv", index_col=0)["value"]
SPAN = float(DP["span_m"])
C_ROOT, C_TIP = float(DP["c_root"]), float(DP["c_tip"])
SWEEP_QC = float(DP["sweep_qc_deg"])
WASHOUT = float(FC["washout_deg"])
WL_H, WL_TAPER, WL_SWEEP = float(FC["winglet_h"]), 0.6, 35.0
Y_BREAK = 0.45
MAC, X_NP_AVL, X_CG = float(FC["mac"]), float(FC["x_np"]), float(FC["x_cg"])
S_REF = SPAN * (C_ROOT + C_TIP) / 2
ATMO = asb.Atmosphere(altitude=2000.0)
RHO, MU = float(ATMO.density()), float(ATMO.dynamic_viscosity())
V = float(DP["v_search"])
RE_C = RHO * V * MAC / MU
MACH = V / float(ATMO.speed_of_sound())
CD_NONINDUCED_SIZING = float(DP["CL_search"]) / float(DP["LD_search"]) - float(DP["CL_search"]) ** 2 / (math.pi * 0.8 * float(DP["AR"]))

# ---------------------------------------------------------------- centre pod design
# Sized around its contents (ASSUMPTIONS, checked against the mass layout from milestone 2):
#   kit bay 0.22 x 0.13 x 0.12 m (3.4 L: radio, 0.5 L water, foil blanket, light, first aid, packed chute),
#   centred on the balance point; gimbal ball 0.10 m under the nose; avionics behind the gimbal;
#   pusher motor at the tail.
POD_X0, POD_LEN, POD_Z = -0.17, 0.82, -0.035
# (x/L, width, height, corner radius); first and last are points
POD_SECTIONS = [(0.08, 0.09, 0.08, 0.035), (0.25, 0.16, 0.15, 0.04), (0.60, 0.16, 0.15, 0.04),
                (0.85, 0.16, 0.15, 0.04), (0.95, 0.09, 0.08, 0.03)]
KIT_BOX = dict(L=0.22, W=0.13, H=0.12)
GIMBAL = dict(x_nose=0.02, length=0.10, z=-0.12)


def pod_section_at(x):
    """Width and height of the pod at body x (linear between stations)."""
    xl = (x - POD_X0) / POD_LEN
    pts = [(0.0, 0, 0)] + [(s[0], s[1], s[2]) for s in POD_SECTIONS] + [(1.0, 0, 0)]
    xs = [p[0] for p in pts]
    return float(np.interp(xl, xs, [p[1] for p in pts])), float(np.interp(xl, xs, [p[2] for p in pts]))


def vspscript(with_pod: bool, stem: str, alphas, run_parasite: bool, symmetry: int = 1) -> str:
    sec1_span, sec2_span = Y_BREAK, SPAN / 2 - Y_BREAK
    c_break = C_ROOT + (C_TIP - C_ROOT) * Y_BREAK / (SPAN / 2)
    tw_break = -WASHOUT * Y_BREAK / (SPAN / 2)
    s = f'''
void SetSec( string wid, int i, double span, double rc, double tc, double sweep, double sloc, double dih, double tw, int tess )
{{
    SetDriverGroup( wid, i, SPAN_WSECT_DRIVER, ROOTC_WSECT_DRIVER, TIPC_WSECT_DRIVER );
    string g = "XSec_" + i;
    SetParmVal( wid, "Span", g, span );
    SetParmVal( wid, "Root_Chord", g, rc );
    SetParmVal( wid, "Tip_Chord", g, tc );
    SetParmVal( wid, "Sweep", g, sweep );
    SetParmVal( wid, "Sweep_Location", g, sloc );
    SetParmVal( wid, "Dihedral", g, dih );
    SetParmVal( wid, "Twist", g, tw );
    SetParmVal( wid, "Twist_Location", g, 0.25 );
    SetParmVal( wid, "SectTess_U", g, tess );
    Update();
}}

void main()
{{
    //==== Wing: root, elevon break, tip, winglet ====//
    string w = AddGeom( "WING", "" );
    SetGeomName( w, "Wing" );
    InsertXSec( w, 1, XS_FILE_AIRFOIL );
    InsertXSec( w, 2, XS_FOUR_SERIES );
    Update();
    SetSec( w, 1, {sec1_span:.5f}, {C_ROOT:.5f}, {c_break:.5f}, {SWEEP_QC}, 0.25, 0.0, {tw_break:.4f}, 8 );
    SetSec( w, 2, {sec2_span:.5f}, {c_break:.5f}, {C_TIP:.5f}, {SWEEP_QC}, 0.25, 0.0, {-WASHOUT:.4f}, 16 );
    SetSec( w, 3, {WL_H:.5f}, {C_TIP:.5f}, {C_TIP * WL_TAPER:.5f}, {WL_SWEEP}, 0.0, 90.0, {-WASHOUT:.4f}, 6 );
    SetParmVal( w, "RotateAirfoilMatchDideralFlag", "WingGeom", 1.0 );
    SetParmVal( w, "Tess_W", "Shape", 33 );
    string ws = GetXSecSurf( w, 0 );
    for ( int i = 0; i <= 2; i++ )
    {{
        ChangeXSecShape( ws, i, XS_FILE_AIRFOIL );
        ReadFileAirfoil( GetXSec( ws, i ), "mh45.dat" );
    }}
    ChangeXSecShape( ws, 3, XS_FOUR_SERIES );
    SetParmVal( GetXSecParm( GetXSec( ws, 3 ), "ThickChord" ), 0.08 );
    SetParmVal( GetXSecParm( GetXSec( ws, 3 ), "Camber" ), 0.0 );
    Update();
'''
    if with_pod:
        s += f'''
    //==== Centre pod (kit bay, avionics, motor mount) ====//
    string f = AddGeom( "FUSELAGE", "" );
    SetGeomName( f, "Pod" );
    SetParmVal( f, "Length", "Design", {POD_LEN} );
    SetParmVal( f, "X_Rel_Location", "XForm", {POD_X0} );
    SetParmVal( f, "Z_Rel_Location", "XForm", {POD_Z} );
    string fs = GetXSecSurf( f, 0 );
    InsertXSec( f, 1, XS_ROUNDED_RECTANGLE );
    InsertXSec( f, 1, XS_ROUNDED_RECTANGLE );
    Update();
'''
        for i, (xl, wdt, hgt, r) in enumerate(POD_SECTIONS, start=1):
            s += f'''
    ChangeXSecShape( fs, {i}, XS_ROUNDED_RECTANGLE );
    SetParmVal( GetXSecParm( GetXSec( fs, {i} ), "XLocPercent" ), {xl} );
    SetParmVal( GetXSecParm( GetXSec( fs, {i} ), "RoundedRect_Width" ), {wdt} );
    SetParmVal( GetXSecParm( GetXSec( fs, {i} ), "RoundedRect_Height" ), {hgt} );
    SetParmVal( GetXSecParm( GetXSec( fs, {i} ), "RoundRectXSec_Radius" ), {r} );
    SetParmVal( GetXSecParm( GetXSec( fs, {i} ), "SectTess_U" ), 6 );
'''
        s += f'''
    SetParmVal( f, "Tess_W", "Shape", 25 );
    Update();

    //==== Camera gimbal ball ====//
    string gb = AddGeom( "POD", "" );
    SetGeomName( gb, "Gimbal" );
    SetParmVal( gb, "Length", "Design", {GIMBAL["length"]} );
    SetParmVal( gb, "FineRatio", "Design", 1.0 );
    SetParmVal( gb, "X_Rel_Location", "XForm", {GIMBAL["x_nose"]} );
    SetParmVal( gb, "Z_Rel_Location", "XForm", {GIMBAL["z"]} );
    Update();
'''
    s += f'''
    WriteVSPFile( "{stem}.vsp3", SET_ALL );
    ExportFile( "{stem}.stl", SET_ALL, EXPORT_STL );
    Print( "SAVED" );
'''
    if run_parasite:
        s += f'''
    //==== Parasite drag build-up ====//
    SetAnalysisInputDefaults( "ParasiteDrag" );
    SetIntAnalysisInput( "ParasiteDrag", "LengthUnit", array<int> = {{ LEN_M }} );
    SetIntAnalysisInput( "ParasiteDrag", "VelocityUnit", array<int> = {{ V_UNIT_M_S }} );
    SetIntAnalysisInput( "ParasiteDrag", "AltLengthUnit", array<int> = {{ 1 }} );
    SetIntAnalysisInput( "ParasiteDrag", "FreestreamPropChoice", array<int> = {{ 0 }} );
    SetDoubleAnalysisInput( "ParasiteDrag", "Altitude", array<double> = {{ 2000.0 }} );
    SetDoubleAnalysisInput( "ParasiteDrag", "Vinf", array<double> = {{ {V} }} );
    SetIntAnalysisInput( "ParasiteDrag", "RefFlag", array<int> = {{ 0 }} );
    SetDoubleAnalysisInput( "ParasiteDrag", "Sref", array<double> = {{ {S_REF:.5f} }} );
    SetIntAnalysisInput( "ParasiteDrag", "GeomSet", array<int> = {{ SET_ALL }} );
    SetStringAnalysisInput( "ParasiteDrag", "FileName", array<string> = {{ "{stem}_parasite.csv" }} );
    string pid = ExecAnalysis( "ParasiteDrag" );
    PrintResults( pid );
'''
    s += f'''
    //==== VSPAERO: panel method on the thick surfaces ====//
    SetAnalysisInputDefaults( "VSPAEROComputeGeometry" );
    SetIntAnalysisInput( "VSPAEROComputeGeometry", "GeomSet", array<int> = {{ SET_ALL }} );
    SetIntAnalysisInput( "VSPAEROComputeGeometry", "ThinGeomSet", array<int> = {{ SET_NONE }} );
    SetIntAnalysisInput( "VSPAEROComputeGeometry", "Symmetry", array<int> = {{ {symmetry} }} );
    ExecAnalysis( "VSPAEROComputeGeometry" );

    string a = "VSPAEROSweep";
    SetAnalysisInputDefaults( a );
    SetIntAnalysisInput( a, "GeomSet", array<int> = {{ SET_ALL }} );
    SetIntAnalysisInput( a, "ThinGeomSet", array<int> = {{ SET_NONE }} );
    SetIntAnalysisInput( a, "Symmetry", array<int> = {{ {symmetry} }} );
    SetIntAnalysisInput( a, "RefFlag", array<int> = {{ 0 }} );
    SetDoubleAnalysisInput( a, "Sref", array<double> = {{ {S_REF:.5f} }} );
    SetDoubleAnalysisInput( a, "cref", array<double> = {{ {MAC:.5f} }} );
    SetDoubleAnalysisInput( a, "bref", array<double> = {{ {SPAN} }} );
    SetDoubleAnalysisInput( a, "Xcg", array<double> = {{ {X_CG:.5f} }} );
    SetDoubleAnalysisInput( a, "AlphaStart", array<double> = {{ {alphas[0]} }} );
    SetDoubleAnalysisInput( a, "AlphaEnd", array<double> = {{ {alphas[1]} }} );
    SetIntAnalysisInput( a, "AlphaNpts", array<int> = {{ {alphas[2]} }} );
    SetDoubleAnalysisInput( a, "MachStart", array<double> = {{ {MACH:.4f} }} );
    SetIntAnalysisInput( a, "MachNpts", array<int> = {{ 1 }} );
    SetDoubleAnalysisInput( a, "ReCref", array<double> = {{ {RE_C:.0f} }} );
    SetDoubleAnalysisInput( a, "Vinf", array<double> = {{ {V} }} );
    SetDoubleAnalysisInput( a, "Vref", array<double> = {{ {V} }} );
    SetDoubleAnalysisInput( a, "Rho", array<double> = {{ {RHO:.4f} }} );
    SetIntAnalysisInput( a, "WakeNumIter", array<int> = {{ 3 }} );
    SetIntAnalysisInput( a, "NCPU", array<int> = {{ 8 }} );
    string rid = ExecAnalysis( a );
    Print( "VSPAERO DONE" );

    while ( GetNumTotalErrors() > 0 )
    {{
        ErrorObj err = PopLastError();
        Print( "VSPERROR: " + err.GetErrorString() );
    }}
}}
'''
    return s


def write_airfoil(path: Path):
    coords = asb.Airfoil("mh45").coordinates
    with open(path, "w") as fh:
        fh.write("MH45\n")
        for x, y in coords:
            fh.write(f"{x:.6f} {y:.6f}\n")


def run_case(name: str, with_pod: bool, alphas=(0.0, 10.0, 6), parasite=True, timeout=3600, symmetry=1):
    d = OUT / name
    d.mkdir(parents=True, exist_ok=True)
    write_airfoil(d / "mh45.dat")
    (d / "build.vspscript").write_text(vspscript(with_pod, "lodestar", alphas, parasite, symmetry))
    r = subprocess.run([str(VSPSCRIPT), "-script", "build.vspscript"], cwd=d, capture_output=True, text=True,
                       timeout=timeout)
    (d / "vspscript_stdout.txt").write_text(r.stdout + "\n" + r.stderr)
    errs = [l for l in r.stdout.splitlines() if "VSPERROR" in l or "Error" in l]
    if errs:
        print(f"  [{name}] messages:", *errs[:8], sep="\n    ")
    return d


def read_polar(d: Path) -> pd.DataFrame:
    files = sorted(d.glob("*.polar"))
    if not files:
        raise FileNotFoundError(f"no .polar in {d}")
    lines = files[0].read_text().splitlines()
    i_hdr = next(i for i, l in enumerate(lines) if l.split()[:3] == ["Beta", "Mach", "AoA"])
    hdr = lines[i_hdr].split()
    rows = [l.split() for l in lines[i_hdr + 1:] if l.strip()]
    return pd.DataFrame([[float(v) for v in r] for r in rows], columns=hdr)


if __name__ == "__main__":
    sys.stdout = Tee(OUT / "run_log.txt")
    print("Lodestar 3D model and aero check, 2026-11-11")
    print(f"Search point: {V} m/s at 2,000 m, Re(MAC) {RE_C:,.0f}, Mach {MACH:.3f}; balance point x = {X_CG:.3f} m\n")

    # Pod fit check: does the kit box fit inside the pod at its two ends?
    print("Pod fit check (kit box 0.22 x 0.13 x 0.12 m centred on the balance point):")
    for xe in (X_CG - KIT_BOX["L"] / 2, X_CG, X_CG + KIT_BOX["L"] / 2):
        w, h = pod_section_at(xe)
        print(f"  x = {xe:.3f} m: pod {w * 1000:.0f} x {h * 1000:.0f} mm outside; box needs "
              f"{KIT_BOX['W'] * 1000:.0f} x {KIT_BOX['H'] * 1000:.0f} mm + walls -> "
              f"{'fits' if w - 0.006 >= KIT_BOX['W'] and h - 0.006 >= KIT_BOX['H'] else 'TOO SMALL'}")

    if "--post" in sys.argv:   # re-read results without re-running VSPAERO
        d_wing, d_full = OUT / "wing_only", OUT / "full"
    else:
        # Both cases run without the symmetry-plane shortcut: with the pod sitting on the
        # symmetry plane, VSPAERO's surface integration came out wrong (negative CL).
        print("\nBuilding and analysing: wing + winglets only")
        d_wing = run_case("wing_only", with_pod=False, parasite=True, symmetry=0)
        print("Building and analysing: full aircraft (wing + winglets + pod + gimbal)")
        d_full = run_case("full", with_pod=True, parasite=True, symmetry=0)

    out = {}
    for name, d in (("wing only", d_wing), ("full", d_full)):
        p = read_polar(d)
        p.to_csv(OUT / f"polar_{d.name}.csv", index=False)
        out[name] = p
    print("\nVSPAERO panel method, wake-based lift and induced drag (surface-integrated induced drag came out negative,")
    print("so it is not used):")
    for name, p in out.items():
        print(f"  {name}:")
        print(p[["AoA", "CLtot", "CLwtot", "CDiw", "Ew", "CMytot"]].round(4).to_string(index=False))

    # Neutral point from the Cm-CL slope:  dCm/dCL = (x_cg - x_np) / MAC
    res = {}
    for name, p in out.items():
        m = (p.AoA >= 2) & (p.AoA <= 10)
        slope = np.polyfit(p.CLtot[m], p.CMytot[m], 1)[0]
        cla = np.polyfit(np.radians(p.AoA[m]), p.CLtot[m], 1)[0]
        res[name] = dict(dCm_dCL=slope, CLa_per_rad=cla, x_np=X_CG - slope * MAC)
    pd.DataFrame(res).T.round(5).to_csv(OUT / "neutral_point_vspaero.csv")
    d_vsp = res["full"]["x_np"] - res["wing only"]["x_np"]
    print(f"\n  VSPAERO wing-only neutral point {res['wing only']['x_np']:.3f} m (AVL {X_NP_AVL:.3f} m). With the pod its lift slope")
    print(f"  falls {1 - res['full']['CLa_per_rad'] / res['wing only']['CLa_per_rad']:.0%}, which is not physical for a 16 cm pod: the panel solver drops the")
    print("  wing area inside the pod and the pod sheds no wake (body-wake option tried: identical result).")

    # Compare all neutral-point-shift estimates
    est = [("Raymer handbook (K_f method)", None), ("AeroSandbox AeroBuildup", None),
           ("AVL slender body", None), ("VSPAERO panel (unreliable here)", d_vsp)]
    f1, f2 = OUT / "pod_np_shift.csv", OUT / "pod_avl_check.csv"
    if f1.exists():
        s1 = pd.read_csv(f1).set_index("method")["dx_np_m"]
        est[0] = (est[0][0], float(s1["Raymer handbook"]))
        est[1] = (est[1][0], float(s1["AeroBuildup"]))
    if f2.exists():
        s2 = pd.read_csv(f2)
        a = s2[(s2.winglet_h == WL_H) & (s2.case == "search, kit on")].set_index("model")["x_np"]
        est[2] = (est[2][0], float(a["pod"] - a["no pod"]))
    print("\nHow far forward the pod and gimbal move the neutral point:")
    for name, v in est:
        if v is not None:
            print(f"  {name:<34} {v * 1000:+6.1f} mm  ({v / MAC:+.1%} MAC)")
    worst = min(v for _, v in est[:3] if v is not None)
    x_cg_new = X_NP_AVL + worst - 0.06 * MAC
    print(f"  -> design to the most cautious trusted estimate ({worst * 1000:+.1f} mm): balance point moves "
          f"{(x_cg_new - X_CG) * 1000:+.0f} mm to x = {x_cg_new:.3f} m")

    # Parasite drag build-up, with the gimbal corrected
    print("\nParasite drag build-up (OpenVSP: flat-plate skin friction x form factor, all turbulent), 15 m/s at 2,000 m:")
    comps = {}
    for line in (d_full / "lodestar_ParasiteBuildUp.csv").read_text().splitlines():
        p = [x.strip() for x in line.split(",")]
        if p and p[0] in ("Wing", "Pod", "Gimbal"):
            comps[p[0]] = dict(S_wet=float(p[1]), FF=float(p[4]), Re=float(p[6]), Cf=float(p[8]), f=float(p[10]))
    for k, v in comps.items():
        print(f"  {k:<7} wetted {v['S_wet']:.3f} m2, Re {v['Re']:,.0f}, Cf {v['Cf']:.4f}, form factor {v['FF']:.2f}, "
              f"drag area {v['f'] * 1e4:.1f} cm2")
    # Hoerner's streamlined-body form factor breaks down for a sphere (fineness 1): FF 53 is meaningless.
    # Replace with sphere drag (Cd 0.47 on frontal area) for the part hanging below the pod.
    r_b = GIMBAL["length"] / 2
    z_pod_bottom = POD_Z - 0.075
    h_above = z_pod_bottom - GIMBAL["z"]          # pod bottom is this far above the ball centre
    exposed = math.pi * r_b**2 / 2 + 2 * r_b * h_above if h_above >= 0 else math.pi * r_b**2 / 2
    f_gimbal = 0.47 * exposed
    print(f"  Gimbal corrected: Hoerner's body formula is not valid for a sphere; sphere drag on the exposed "
          f"{exposed * 1e4:.0f} cm2 gives {f_gimbal * 1e4:.1f} cm2")
    f_body = comps["Pod"]["f"] + f_gimbal
    cd_wing_turb = comps["Wing"]["f"] / S_REF
    cd_body = f_body / S_REF
    print(f"  Pod + gimbal drag area {f_body * 1e4:.1f} cm2 (sizing assumed 40 cm2 for pod, gimbal and skid) -> CD {cd_body:.4f}")

    # What that means for endurance
    sz_profile = CD_NONINDUCED_SIZING - 0.004 / S_REF          # sizing's NeuralFoil wing profile drag (x1.15)
    m_tow = 7.548
    W = m_tow * 9.81
    rows = []
    for label, cdw in (("wing: NeuralFoil, laminar runs as in sizing", sz_profile),
                       ("wing: halfway", (sz_profile + cd_wing_turb) / 2),
                       ("wing: fully turbulent (OpenVSP)", cd_wing_turb)):
        for e in (0.80, 0.90):
            best = (0, 0)
            for Vs in np.arange(15.0, 20.01, 0.25):  # at least 1.25 x stall (14.9 m/s)
                CL = W / (0.5 * RHO * Vs**2 * S_REF)
                CD = cdw + cd_body + CL**2 / (math.pi * e * float(DP["AR"]))
                P = W * Vs * CD / CL / 0.55 + 25
                E = float(DP["batt_wh"]) * 0.85 * 0.90 / P
                if E > best[0]:
                    best = (E, Vs)
            rows.append(dict(case=label, e=e, CD0=cdw + cd_body, endurance_h=best[0], best_speed=best[1]))
    en = pd.DataFrame(rows)
    en.round(4).to_csv(OUT / "endurance_vs_drag.csv", index=False)
    print("\nEndurance at 7.55 kg for each drag assumption (best speed between 15 and 20 m/s, i.e. at least 1.25 x stall):")
    print(en.round(3).to_string(index=False))
    import stability as stab  # import-safe: only defines functions
    from avl_run import Geometry
    g_m3 = Geometry(c_root=C_ROOT, c_tip=C_TIP, washout_deg=WASHOUT, winglet_h=WL_H)
    x_batt_new, _ = stab.place_for_cg(g_m3, x_cg_new)
    print(f"\nNew layout: balance point and kit bay at x = {x_cg_new:.3f} m, battery packs at x = {x_batt_new:.3f} m")
    pd.Series({"x_cg": x_cg_new, "x_kit": x_cg_new, "x_batt": x_batt_new, "np_shift_m": worst, "f_body_m2": f_body,
               "cd_wing_turbulent": cd_wing_turb, "cd_wing_sizing": sz_profile}).to_csv(OUT / "m3_config.csv", header=["value"])
