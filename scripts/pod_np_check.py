"""Independent estimates of how much the centre pod and gimbal move the neutral point.

1. AeroSandbox AeroBuildup (semi-empirical: wing from lifting-line + NeuralFoil, fuselage from
   slender-body / Jorgensen) with and without the pod modelled as a fuselage.
2. Raymer's handbook fuselage pitching-moment estimate (Raymer, Aircraft Design, eq. 16.25):
   Cm_alpha_fus = K_f * W_f^2 * L_f / (c * S)   [per degree]
3. VSPAERO panel method (from vsp_model.py), read in for comparison.

Run:  uv run python scripts/pod_np_check.py
"""
from pathlib import Path
import sys
import math

import numpy as np
import pandas as pd
import aerosandbox as asb

sys.path.insert(0, str(Path(__file__).parent))
import vsp_model as vm  # noqa: E402  (reuses geometry constants; does not rerun VSPAERO)

ROOT = Path(__file__).resolve().parents[1]
af = asb.Airfoil("mh45")
b2 = vm.SPAN / 2
c_break = vm.C_ROOT + (vm.C_TIP - vm.C_ROOT) * vm.Y_BREAK / b2


def x_le(y, c):
    return y * math.tan(math.radians(vm.SWEEP_QC)) + 0.25 * vm.C_ROOT - 0.25 * c


wing = asb.Wing(name="wing", symmetric=True, xsecs=[
    asb.WingXSec(xyz_le=[0, 0, 0], chord=vm.C_ROOT, twist=0, airfoil=af),
    asb.WingXSec(xyz_le=[x_le(vm.Y_BREAK, c_break), vm.Y_BREAK, 0], chord=c_break,
                 twist=-vm.WASHOUT * vm.Y_BREAK / b2, airfoil=af),
    asb.WingXSec(xyz_le=[x_le(b2, vm.C_TIP), b2, 0], chord=vm.C_TIP, twist=-vm.WASHOUT, airfoil=af)])
xt = x_le(b2, vm.C_TIP)
winglet = asb.Wing(name="winglet", symmetric=True, xsecs=[
    asb.WingXSec(xyz_le=[xt, b2, 0], chord=vm.C_TIP, airfoil=asb.Airfoil("naca0008")),
    asb.WingXSec(xyz_le=[xt + vm.WL_H * math.tan(math.radians(vm.WL_SWEEP)), b2, vm.WL_H],
                 chord=vm.C_TIP * vm.WL_TAPER, airfoil=asb.Airfoil("naca0008"))])

# Pod as a fuselage: rounded-rectangle sections approximated by ellipses of the same width and height
stations = [(0.0, 0.0, 0.0)] + [(s[0], s[1], s[2]) for s in vm.POD_SECTIONS] + [(1.0, 0.0, 0.0)]
pod = asb.Fuselage(name="pod", xsecs=[
    asb.FuselageXSec(xyz_c=[vm.POD_X0 + xl * vm.POD_LEN, 0, vm.POD_Z], width=max(w, 1e-3), height=max(h, 1e-3))
    for xl, w, h in stations])
g = vm.GIMBAL
gimbal = asb.Fuselage(name="gimbal", xsecs=[
    asb.FuselageXSec(xyz_c=[g["x_nose"] + g["length"] * t, 0, g["z"]], radius=max(g["length"] / 2 * math.sin(math.pi * t), 1e-3))
    for t in np.linspace(0, 1, 9)])

op = asb.OperatingPoint(atmosphere=vm.ATMO, velocity=vm.V, alpha=6.0)
rows = {}
for name, fus in (("wing + winglets", []), ("+ pod", [pod]), ("+ pod + gimbal", [pod, gimbal])):
    plane = asb.Airplane(name=name, xyz_ref=[vm.X_CG, 0, 0], wings=[wing, winglet], fuselages=fus,
                         s_ref=vm.S_REF, c_ref=vm.MAC, b_ref=vm.SPAN)
    r = asb.AeroBuildup(airplane=plane, op_point=op).run_with_stability_derivatives(alpha=True, beta=True, p=False, q=False, r=False)
    f = lambda k: float(np.ravel(r[k])[0])
    rows[name] = dict(CLa=f("CLa"), Cma=f("Cma"), x_np=f("x_np"), Cnb=f("Cnb"),
                      CD=f("CD"), CL=f("CL"))
ab = pd.DataFrame(rows).T
print("AeroBuildup at 6 deg, 15 m/s, 2,000 m (moments about the balance point):")
print(ab.round(4).to_string())
d_ab = ab.loc["+ pod + gimbal", "x_np"] - ab.loc["wing + winglets", "x_np"]
print(f"  pod + gimbal move the neutral point by {d_ab * 1000:+.1f} mm ({d_ab / vm.MAC:+.2%} MAC)")
d_cnb = ab.loc["+ pod + gimbal", "Cnb"] - ab.loc["wing + winglets", "Cnb"]
print(f"  and change yaw stability Cnb by {d_cnb:+.4f} per rad (milestone 2 winglets give about 0.023)")

# Raymer: K_f depends on where the wing root quarter-chord sits along the body
x_qc_root = 0.25 * vm.C_ROOT
pos = (x_qc_root - vm.POD_X0) / vm.POD_LEN
# Raymer fig. 16.14, digitised roughly: K_f vs position of root quarter-chord (% body length)
kf_tab = ([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [0.005, 0.007, 0.009, 0.012, 0.018, 0.027])
K_f = float(np.interp(pos, *kf_tab))
W_f, L_f = max(s[1] for s in vm.POD_SECTIONS), vm.POD_LEN
cma_fus_rad = K_f * W_f**2 * L_f / (vm.MAC * vm.S_REF) * 180 / math.pi
CLa_wing = ab.loc["wing + winglets", "CLa"]
d_raymer = -cma_fus_rad / CLa_wing * vm.MAC
print(f"\nRaymer: root quarter-chord at {pos:.0%} of pod length -> K_f = {K_f:.3f}; "
      f"Cm_alpha(pod) = {cma_fus_rad:+.4f}/rad -> neutral point {d_raymer * 1000:+.1f} mm ({d_raymer / vm.MAC:+.2%} MAC)")

pd.DataFrame({"method": ["AeroBuildup", "Raymer handbook"], "dx_np_m": [d_ab, d_raymer],
              "dx_np_pct_mac": [d_ab / vm.MAC * 100, d_raymer / vm.MAC * 100]}).round(5).to_csv(
    ROOT / "out" / "vsp" / "pod_np_shift.csv", index=False)
ab.round(5).to_csv(ROOT / "out" / "vsp" / "aerobuildup_pod.csv")
