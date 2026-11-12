"""Preview renders of the OpenVSP model with Blender (Cycles, headless).

Shows the outer shape (hero shots) and an X-ray view with the kit bay (orange),
battery packs (blue) and balance point (black dot), at the milestone 3 layout.

Run:  uv run python scripts/render_preview.py
Writes: out/render/*.png
"""
from pathlib import Path
import subprocess

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "render"
OUT.mkdir(parents=True, exist_ok=True)
BLENDER = ROOT.parent / "aero-tools" / "blender-5.3.0-windows-x64" / "blender.exe"
STL = ROOT / "out" / "vsp" / "full" / "lodestar.stl"
M3 = pd.read_csv(ROOT / "out" / "vsp" / "m3_config.csv", index_col=0)["value"]

r = subprocess.run([str(BLENDER), "-b", "-P", str(ROOT / "scripts" / "blender_scene.py"), "--",
                    str(STL), str(OUT), str(M3["x_cg"]), str(M3["x_kit"]), str(M3["x_batt"])],
                   capture_output=True, text=True, timeout=3600)
(OUT / "blender_log.txt").write_text(r.stdout + "\n" + r.stderr)
print("\n".join(l for l in r.stdout.splitlines() if "Saved" in l or "Error" in l) or r.stdout[-2000:])
