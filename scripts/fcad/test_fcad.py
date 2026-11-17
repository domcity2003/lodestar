"""Known-answer tests for fcad (one FreeCAD run covers them all).

Mesh-integrated properties are exact for flat-faced solids and within ~1e-3 for curved ones
(0.1 mm chordal, 0.05 rad angular tolerance), so curved-shape checks use that tolerance.
Run:  .venv/Scripts/python.exe scripts/fcad/test_fcad.py      (or pytest scripts/fcad/test_fcad.py)
The job uses metres with scale_to_mm = 1000, the same as the Lodestar model, so unit handling is tested too.
"""
from pathlib import Path
import math
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fcad.runner import run  # noqa: E402
from fcad.render import read_stl  # noqa: E402



def naca0012(n=60):
    x = 0.5 * (1 - np.cos(np.linspace(0, math.pi, n)))
    t = 5 * 0.12 * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
    t = t + 0.0015 * x               # open the trailing edge (0.3% chord), as the wing model does
    up = np.c_[x[::-1], t[::-1]]
    lo = np.c_[x[1:], -t[1:]]
    return np.vstack([up, lo])        # TE upper -> LE -> TE lower


def job():
    af = naca0012()
    area = 0.5 * abs(np.dot(af[:, 0], np.roll(af[:, 1], -1)) - np.dot(af[:, 1], np.roll(af[:, 0], -1)))
    sec = lambda y: dict(kind="spline_line", points=[[u, y, v] for u, v in af])  # noqa: E731
    rr = lambda x: dict(origin=[x, 0, 0], normal=[1, 0, 0], xdir=[0, 1, 0], w=0.2, h=0.1, r=0.02)  # noqa: E731
    parts = {
        "box": dict(op="box", size=[0.1, 0.2, 0.3], center=[1, 2, 3]),
        "cyl_axis": dict(op="cylinder", r=0.05, h=0.4, axis=[1, 1, 0], center=[0.5, 0.5, 0.5]),
        "cyl_p": dict(op="cylinder", r=0.02, p0=[0, 0, 0], p1=[0.3, 0.4, 0]),
        "sphere": dict(op="sphere", r=0.1, center=[0, 0, 1]),
        "cone": dict(op="cone", r1=0.1, r2=0.0, h=0.3, center=[0, 0, 0]),
        "loft_af": dict(op="loft", ruled=True, sections=[sec(0.0), sec(0.5)]),
        "loft_poly": dict(op="loft", ruled=True, sections=[
            dict(kind="polyline", points=[[0, 0, 0], [0.2, 0, 0], [0.2, 0, 0.1], [0, 0, 0.1]]),
            dict(kind="polyline", points=[[0, 0.3, 0], [0.2, 0.3, 0], [0.2, 0.3, 0.1], [0, 0.3, 0.1]])]),
        "rrect": dict(op="rrect_loft", sections=[rr(0.0), rr(0.5), rr(1.0)]),
        "boxA": dict(op="box", size=[0.1, 0.1, 0.1], center=[0, 0, 0]),
        "boxB": dict(op="box", size=[0.1, 0.1, 0.1], center=[0.25, 0, 0]),        # 0.15 m gap to A
        "boxC": dict(op="box", size=[0.1, 0.1, 0.1], center=[0.05, 0.03, 0]),     # overlaps A: 0.05*0.07*0.1
        "half": dict(op="split_by_plane", shape=dict(op="box", size=[1, 1, 1], center=[0, 0, 0]),
                     point=[0.25, 0, 0], normal=[1, 0, 0], keep="positive"),
        "rot": dict(op="box", size=[0.2, 0.02, 0.02], center=[0.1, 0, 0],
                    transform=[dict(rotate=dict(axis=[0, 0, 1], deg=90, center=[0, 0, 0])),
                               dict(translate=[0, 0, 1])]),
        "tube": dict(op="cut", base=dict(op="cylinder", r=0.01, h=0.3),
                     tools=[dict(op="cylinder", r=0.008, h=0.3)]),
        "two": dict(op="compound", shapes=[dict(op="box", size=[0.1, 0.1, 0.1], center=[0, 0, 0]),
                                           dict(op="box", size=[0.1, 0.1, 0.1], center=[1, 0, 0])]),
    }
    defs = {"big": dict(op="box", size=[1, 1, 1], center=[0, 0, 0])}
    checks = [
        dict(id="dist_AB", type="distance", a="boxA", b="boxB"),
        dict(id="shell_A_big", type="distance", a="boxA", b="big", mode="shell"),     # 0.45 m to the walls
        dict(id="inside_C_in_A", type="inside", part="boxC", parent="boxA"),          # 0.5*0.7 = 0.35
        dict(id="clash", type="interference", parts=["boxA", "boxB", "boxC"]),
        dict(id="clash_allowed", type="interference", parts=["boxA", "boxB", "boxC"], allow=[["boxA", "boxC"]]),
    ]
    return dict(scale_to_mm=1000, parts=parts, defs=defs, checks=checks,
                outputs=dict(stl_parts=["box", "loft_af"])), area


def close(a, b, rel=None, abs_=None, what=""):
    ok = abs(a - b) <= (abs_ if abs_ is not None else rel * abs(b))
    print(f"{'PASS' if ok else 'FAIL'}  {what}: got {a:.6g}, expected {b:.6g}")
    return ok


def test_all():
    j, af_area = job()
    with tempfile.TemporaryDirectory() as td:
        j["outputs"]["stl_dir"] = str(Path(td) / "stl")
        r = run(j, td)
        tri = read_stl(Path(td) / "stl" / "box.stl")
        stl_vol = abs(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()) / 6
    P, C = r["parts"], r["checks"]
    ok = []
    ok.append(close(P["box"]["volume"], 0.006, rel=1e-9, what="box volume (m3)"))
    ok.append(close(np.linalg.norm(np.subtract(P["box"]["com"], [1, 2, 3])), 0, abs_=1e-9, what="box centroid error"))
    ok.append(close(P["cyl_axis"]["volume"], math.pi * 0.05**2 * 0.4, rel=1e-3, what="cylinder volume"))
    ok.append(close(np.linalg.norm(np.subtract(P["cyl_axis"]["com"], [0.5] * 3)), 0, abs_=1e-6,
                    what="cylinder (axis 1,1,0) centroid error"))
    ok.append(close(np.linalg.norm(np.subtract(P["cyl_p"]["com"], [0.15, 0.2, 0])), 0, abs_=1e-6,
                    what="cylinder (end points) centroid error"))
    ok.append(close(P["sphere"]["volume"], 4 / 3 * math.pi * 0.1**3, rel=1e-3, what="sphere volume"))
    ok.append(close(P["cone"]["volume"], math.pi * 0.1**2 * 0.3 / 3, rel=1e-3, what="cone volume"))
    ok.append(close(P["cone"]["com"][2], -0.15 + 0.3 / 4, abs_=1e-5, what="cone centroid z (h/4 from base)"))
    ok.append(close(P["loft_af"]["volume"], af_area * 0.5, rel=0.005, what="airfoil loft vs prism"))
    print(f"      (BRep volume of the same airfoil loft: {P['loft_af']['brep_volume']:.6g}; "
          f"{100 * (P['loft_af']['brep_volume'] / (af_area * 0.5) - 1):+.1f}%, the OCC 7.8.1 gotcha in README)")
    ok.append(close(P["loft_poly"]["volume"], 0.2 * 0.1 * 0.3, rel=1e-9, what="polyline loft vs prism"))
    ok.append(close(P["rrect"]["volume"], (0.2 * 0.1 - (4 - math.pi) * 0.02**2) * 1.0, rel=0.002,
                    what="rounded-rectangle loft vs prism"))
    ok.append(close(P["half"]["volume"], 0.25, rel=1e-9, what="split_by_plane kept volume"))
    ok.append(close(P["half"]["com"][0], 0.375, abs_=1e-9, what="split_by_plane centroid x"))
    ok.append(close(np.linalg.norm(np.subtract(P["rot"]["com"], [0, 0.1, 1])), 0, abs_=1e-9,
                    what="rotate+translate centroid error"))
    ok.append(close(P["tube"]["volume"], math.pi * (0.01**2 - 0.008**2) * 0.3, rel=1e-3, what="tube (cut) volume"))
    ok.append(close(P["two"]["com"][0], 0.5, abs_=1e-9, what="compound centroid x"))
    Ixx = P["box"]["inertia_unit_density"][0][0]
    ok.append(close(Ixx, 0.006 * (0.2**2 + 0.3**2) / 12, rel=1e-6, what="box Ixx (unit density)"))
    ok.append(close(C["dist_AB"]["distance"], 0.15, abs_=1e-9, what="distance box-box"))
    ok.append(close(C["shell_A_big"]["distance"], 0.45, abs_=1e-9, what="distance to parent shell"))
    ok.append(close(C["inside_C_in_A"]["share_inside_total"], 0.35, abs_=1e-6, what="inside share"))
    cl = C["clash"]["clashes"]
    ok.append(close(len(cl), 1, abs_=0, what="clash count"))
    ok.append(close(cl[0]["volume"], 0.05 * 0.07 * 0.1, rel=1e-6, what="interference volume"))
    ok.append(close(C["clash"]["pairs_tested"], 1, abs_=0, what="bbox prefilter (A-B not tested)"))
    ok.append(close(len(C["clash_allowed"]["clashes"]), 0, abs_=0, what="allow-list"))
    ok.append(close(stl_vol, 0.006, rel=1e-6, what="STL written in job units (volume from mesh)"))
    print(f"\n{sum(ok)}/{len(ok)} passed; FreeCAD {r['freecad_version']}, {r['seconds']:.1f} s")
    assert all(ok)


if __name__ == "__main__":
    test_all()
