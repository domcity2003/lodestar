"""Re-run the milestone 2 stability checks in AVL with the centre pod modelled as a slender body.

Why: the pod sits ahead of the balance point. Bodies ahead of the CG are destabilising in
pitch (they move the neutral point forward) and in yaw (they reduce Cnb). Milestone 2 had
no pod. This script answers whether the 6% margin and the 0.25 m winglets still hold.

Run:  uv run python scripts/pod_avl_check.py
Writes: out/vsp/pod_avl_check.csv, out/vsp/pod_avl_log.txt
"""
from dataclasses import replace
from pathlib import Path
import math
import sys

import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
import stability as st  # noqa: E402
import vsp_model as vm  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "vsp"
sys.stdout = st.Tee(OUT / "pod_avl_log.txt")

# Equivalent-area round body for the rounded-rectangle pod (AVL bodies are round)
xr = [(0.0, 0.0)]
for xl, w, h, rc in vm.POD_SECTIONS:
    area = w * h - (4 - math.pi) * rc**2
    xr.append((xl * vm.POD_LEN, math.sqrt(area / math.pi)))
xr.append((vm.POD_LEN, 0.0))

g_nopod = st.Geometry(c_root=vm.C_ROOT, c_tip=vm.C_TIP, washout_deg=vm.WASHOUT)
SM = 0.06
cases = [("search, kit on", 15.0, True), ("top speed, kit on", 22.0, True),
         ("top speed, kit off", 22.0, False), ("slow, kit off", 12.5, False)]

print("Lodestar: milestone 2 stability re-checked with the centre pod as an AVL slender body")
print(f"Pod equivalent radius profile (x from pod nose, m): {[(round(x, 3), round(r, 3)) for x, r in xr]}\n")
rows = []
for h in (0.25, 0.30):
    g0 = replace(g_nopod, winglet_h=h)
    gp = replace(g0, body_xr=tuple(xr), body_x0=vm.POD_X0, body_z0=vm.POD_Z)
    xnp0 = st.analyse(g0, 0.40, modes=False)["x_np"]
    xnp = st.analyse(gp, 0.40, modes=False)["x_np"]
    x_cg = xnp - SM * gp.mac
    print(f"Winglets {h:.2f} m: neutral point {xnp0:.4f} m without pod -> {xnp:.4f} m with pod "
          f"({(xnp - xnp0) * 1000:+.1f} mm, {(xnp - xnp0) / gp.mac:+.2%} MAC); 6% margin needs balance point x = {x_cg:.3f} m")
    for label, V, kit in cases:
        for pod_on, g in (("no pod", g0), ("pod", gp)):
            xc = (xnp0 if pod_on == "no pod" else xnp) - SM * g.mac
            r = st.analyse(g, xc, V=V, kit_on=kit)
            rows.append(dict(winglet_h=h, case=label, model=pod_on, x_np=r["x_np"], x_cg=r["x_cg"],
                             static_margin=r["static_margin"], elevon=r["elevon"], Cnb=r["Cnb"],
                             dutch_roll_zeta=r.get("dutch_roll_zeta"), dutch_roll_wn=r.get("dutch_roll_wn"),
                             spiral_t2=r.get("spiral_t2"), short_period_zeta=r.get("short_period_zeta"),
                             phugoid_zeta=r.get("phugoid_zeta"), pitch_ok=r.get("pitch_ok"), lateral_ok=r.get("lateral_ok")))
df = pd.DataFrame(rows)
df.round(4).to_csv(OUT / "pod_avl_check.csv", index=False)
print()
print(df[["winglet_h", "case", "model", "static_margin", "elevon", "Cnb", "dutch_roll_zeta", "spiral_t2",
          "pitch_ok", "lateral_ok"]].round(3).to_string(index=False))
