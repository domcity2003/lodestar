"""Milestone 5b: renders of the v2 CAD (Blender Cycles, headless).

Run:  uv run python scripts/render_v2.py [samples] [shot ...]     (default 96 samples, all shots)
Needs: out/cad/stl/ from scripts/cad_model.py
Writes: out/render/v2_*.png, out/render/blender_v2_log.txt
"""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "render"
OUT.mkdir(parents=True, exist_ok=True)
BLENDER = ROOT.parent / "aero-tools" / "blender-5.3.0-windows-x64" / "blender.exe"
samples = sys.argv[1] if len(sys.argv) > 1 else "96"
shots = sys.argv[2:]
r = subprocess.run([str(BLENDER), "-b", "-P", str(ROOT / "scripts" / "blender_v2.py"), "--",
                    str(ROOT / "out" / "cad" / "stl"), str(OUT), samples, *shots],
                   capture_output=True, text=True, timeout=7200)
(OUT / "blender_v2_log.txt").write_text(r.stdout + "\n" + r.stderr)
print("\n".join(l for l in (r.stdout + r.stderr).splitlines()
                if l.startswith("Saved") or "Error" in l or "Traceback" in l) or r.stderr[-3000:])
