"""Smoke test: prove every tool in the chain actually runs on this machine.

Run:  uv run python scripts/smoke_test.py
Outputs go to out/smoke/.
"""
from pathlib import Path
import subprocess
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out" / "smoke"
OUT.mkdir(parents=True, exist_ok=True)
TOOLS = ROOT.parent / "aero-tools"
AVL_EXE = TOOLS / "avl" / "avl.exe"
VSPSCRIPT_EXE = TOOLS / "OpenVSP-3.56.0-win64" / "vspscript.exe"

results = {}


def check(name, fn):
    try:
        msg = fn()
        results[name] = ("OK", msg)
    except Exception as e:  # report and keep going
        if "Application Control" in str(e):
            # Blocked by the machine's Windows Application Control policy, not broken. CAD runs through
            # FreeCAD headless instead (scripts/fcad/, checked below and by fcad/test_fcad.py).
            results[name] = ("SKIP", f"blocked by Windows Application Control; CAD uses FreeCAD via scripts/fcad ({e})")
        else:
            results[name] = ("FAIL", f"{type(e).__name__}: {e}")


# A toy swept flying wing (1.2 m span) with a reflexed airfoil, used by several checks.
import aerosandbox as asb
import aerosandbox.numpy as np

AIRFOIL = asb.Airfoil("mh45")  # reflexed airfoil designed for flying wings


def toy_wing():
    return asb.Airplane(
        name="toy_wing",
        xyz_ref=[0.12, 0, 0],
        wings=[
            asb.Wing(
                name="Main",
                symmetric=True,
                xsecs=[
                    asb.WingXSec(xyz_le=[0, 0, 0], chord=0.30, twist=0, airfoil=AIRFOIL),
                    asb.WingXSec(xyz_le=[0.20, 0.60, 0.03], chord=0.15, twist=-3, airfoil=AIRFOIL),
                ],
            )
        ],
    )


def t_neuralfoil():
    a = AIRFOIL.get_aero_from_neuralfoil(alpha=4, Re=5e5)
    return f"MH45 at 4 deg: CL={float(a['CL'][0]):.3f}, CD={float(a['CD'][0]):.4f}, CM={float(a['CM'][0]):.3f}"


def t_vlm():
    op = asb.OperatingPoint(velocity=40, alpha=3)
    aero = asb.VortexLatticeMethod(airplane=toy_wing(), op_point=op).run_with_stability_derivatives()
    return f"VLM: CL={float(aero['CL']):.3f}, Cma={float(aero['Cma']):.3f} (negative = pitch-stable)"


def t_avl():
    op = asb.OperatingPoint(velocity=40, alpha=3)
    aero = asb.AVL(airplane=toy_wing(), op_point=op, avl_command=str(AVL_EXE)).run()
    return f"AVL: CL={float(aero['CL']):.3f}, Cma={float(aero['Cma']):.3f}, Cnb={float(aero['Cnb']):.4f}"


def t_openvsp():
    script = OUT / "toy_wing.vspscript"
    toy_wing().export_OpenVSP_vspscript(str(script))
    vsp3 = OUT / "toy_wing.vsp3"
    # Append a save command so vspscript writes a .vsp3 model file.
    text = script.read_text()
    text = text.replace("void main()\n{", f'void main()\n{{', 1)
    text = text.rstrip().rstrip("}") + f'\n    WriteVSPFile("{vsp3.as_posix()}", SET_ALL);\n}}\n'
    script.write_text(text)
    r = subprocess.run([str(VSPSCRIPT_EXE), "-script", str(script)], capture_output=True, text=True, timeout=120)
    if not vsp3.exists():
        raise RuntimeError(f"no .vsp3 written; stdout tail: {r.stdout[-300:]} stderr: {r.stderr[-300:]}")
    return f"OpenVSP model written: {vsp3.name} ({vsp3.stat().st_size} bytes)"


def t_jsbsim():
    import jsbsim
    fdm = jsbsim.FGFDMExec(None)  # uses the aircraft bundled with the pip package
    fdm.load_model("c172x")
    fdm.disable_output()  # no CSV file output
    fdm["ic/h-sl-ft"] = 3000
    fdm["ic/vc-kts"] = 100
    fdm.run_ic()
    for _ in range(600):  # about 5 s of flight
        fdm.run()
    msg = f"JSBSim c172x flew {fdm.get_sim_time():.1f} s, altitude {fdm['position/h-sl-ft']:.0f} ft"
    del fdm  # closes the CSV log the c172x model opens regardless, so it can be deleted
    for d in (Path.cwd(), ROOT):
        (d / "JSBout172B.csv").unlink(missing_ok=True)
    return msg


def t_build123d():
    from build123d import Box, export_step
    part = Box(100, 50, 10)
    path = OUT / "box.step"
    export_step(part, str(path))
    return f"build123d STEP export: {path.name} ({path.stat().st_size} bytes)"


def t_freecad():
    exe = shutil.which("FreeCADCmd") or next(
        (str(p) for p in Path.home().glob("AppData/Local/Programs/FreeCAD*/bin/FreeCADCmd.exe")), None
    ) or next((str(p) for p in Path("C:/Program Files").glob("FreeCAD*/bin/FreeCADCmd.exe")), None)
    if not exe:
        raise FileNotFoundError("FreeCADCmd.exe not found")
    r = subprocess.run([exe, "--version"], capture_output=True, text=True, timeout=60)
    return f"{exe} -> {(r.stdout or r.stderr).strip().splitlines()[0]}"


def t_flow5():
    exe = next(TOOLS.glob("flow5*/flow5.exe"), None)
    if not exe:
        raise FileNotFoundError("flow5.exe not found")
    return f"flow5 GUI present: {exe.relative_to(ROOT.parent)} (open it by hand; it has no command line mode)"


for name, fn in [
    ("NeuralFoil (airfoils)", t_neuralfoil),
    ("AeroSandbox VLM", t_vlm),
    ("AVL (stability)", t_avl),
    ("OpenVSP (3D model)", t_openvsp),
    ("JSBSim (flight sim)", t_jsbsim),
    ("build123d (code CAD)", t_build123d),
    ("FreeCAD", t_freecad),
    ("flow5", t_flow5),
]:
    check(name, fn)

width = max(len(k) for k in results)
for k, (status, msg) in results.items():
    print(f"{k:<{width}}  {status:<4}  {msg}")

# A failed tool used to print FAIL but still exit 0, so a runner could not see it.
failed = [k for k, (status, _) in results.items() if status == "FAIL"]
skipped = [k for k, (status, _) in results.items() if status == "SKIP"]
print()
print(f"{len(results) - len(failed) - len(skipped)} OK, {len(failed)} FAIL, {len(skipped)} SKIP")
sys.exit(1 if failed else 0)
