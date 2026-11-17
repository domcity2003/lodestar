"""Lodestar v1 wing (wing_trade design "v1"): finite-element structural verification with CalculiX.

What it does
  1. Builds a thin-walled wing-box model of one half-wing (centreline to tip, swept and tapered, MH45
     section) and writes a CalculiX deck directly from Python:
       - skins: S4 shells, +-45 glass (orthotropic, G12 = 9 GPa), over the full chord;
       - spar caps: the 12 mm cap strip of the skin carries the pultruded carbon (smeared skin+cap shell);
       - spar web (25% chord) and rear web (72% chord): S4 shells, smeared glass/foam sandwich;
       - foam core: C3D8I/C3D6 solids filling the section (EPS 20; XPS at the servo bay and the winglet root);
       - clamped at the centreline (symmetry, as the beam model); the tip section is a rigid winglet-root rib.
     The elevon is modelled as continuous with the wing (servo-held), like the beam model's section integrals.
  2. Frequencies (*FREQUENCY) with the beam model's masses (P50B packs, joints, servo, winglet, hinge),
     written first to out/fea/modes.csv for the flutter work, with mode shapes in modes_shapes.csv.
  3. Ultimate static cases from out/wing_trade/compact_eps20_light_loads.csv (limit x 1.5), unit tip torque
     (GJ) and unit tip bending moment (EI). FE vs beam model: tip deflection, cap stress, skin compression,
     skin shear, web shear, core stress.
  4. Mesh convergence (three meshes).
  5. Load-case matrix incl. the cases the flight envelope does not cover (belly landing, bungee launch,
     kit release, ground handling, winglet side load, elevon hinge), joint local checks and the margins table.

Run (from scripts/):  ../.venv/Scripts/python.exe -X utf8 fea_wing.py            (full run, ~10-20 min)
                      ../.venv/Scripts/python.exe -X utf8 fea_wing.py --modes    (frequencies only)
Writes: out/fea/ (modes.csv, modes_shapes.csv, fe_vs_beam.csv, fe_stations.csv, convergence.csv,
        load_cases.csv, extra_cases.csv, joint_checks.csv, margins.csv, run.log) and ccx working folders.
CalculiX 2.22 (FreeCAD 1.1.4 bundle) is called as a subprocess; nothing is written into its folder.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import pandas as pd
from scipy.linalg import eigh

import structures as s

ROOT = s.ROOT
OUT = ROOT / "out" / "fea"
WT = ROOT / "out" / "wing_trade"
# Inputs of this verification: snapshot of the out/wing_trade v1 files of 2026-11-14 17:16 (they are regenerated
# by later design changes). Set FEA_INPUTS to another folder holding the same file names to rerun on new inputs.
INPUTS = Path(os.environ.get("FEA_INPUTS", ROOT / "out" / "wing_trade"))   # live wing_trade v1 outputs
DESIGN = "v1"                        # wing_trade v1: compact_eps20_light wing, CAD v4 layout
from fcad.runner import find_freecad_bin  # noqa: E402  (CalculiX ships in FreeCAD's bin folder)
CCX = Path(os.environ.get("CCX_EXE") or find_freecad_bin("ccx.exe") or "ccx.exe")

FOS = s.FOS
LAM = s.SWEEP
CL, SL = math.cos(LAM), math.sin(LAM)
E_AX = np.array([SL, CL, 0.0])          # spar axis (quarter-chord line), in-plane
N_AX = np.array([CL, -SL, 0.0])         # in-plane normal to the spar axis (bending axis)
LAY = s.LAYUPS[0]                        # 1 ply +-45 glass 80 g/m2
TS = LAY.t_ply * LAY.n_ply              # skin thickness 0.08 mm
import lodestar_v1 as _v1  # noqa: E402
DOUBLER = _v1.STRUCTURE["upper_skin_doubler_y"]   # upper-skin doubler span (one extra ply)
DOUBLER_TAPER = _v1.STRUCTURE["upper_skin_doubler_taper_m"]   # 1.5x skin outboard of it (ply drop-off)
CAP_W = LAY.cap_w                        # 12 mm physical cap width
CAP_WS = CAP_W / CL                      # streamwise footprint of the cap strip
SKIN_RHO = (LAY.areal_ply * LAY.n_ply + LAY.finish_areal) / TS   # areal 0.19 kg/m2 incl. finish
SKIN_E, SKIN_G, SKIN_NU = LAY.skin.E, LAY.skin.G, 0.5             # +-45 glass laminate (E 11, G 9 GPa)
CARB_E1, CARB_E2, CARB_G12 = s.CARBON_UD.E, 8e9, 4.5e9            # pultruded strip, transverse ASSUMED
T_WEB = CAP_W + 2 * 0.08e-3             # Rohacell web block + one glass ply each side
T_REAR = 0.003 + 2 * 0.08e-3            # 3 mm XPS + one glass ply each side
HINGE_KG_M = 0.010
X_BATT_V1, Y_BATT_V1 = 0.412, 0.305     # frozen v1 layout (lodestar_v1.LAYOUT; "v1.1" below)
Y_BATT_V1_RANGE = (0.234, 0.376)        # pack centre span used by wing_trade "v1" (s.Y_BATT)
# Layouts: "v1" = inputs of this verification (out/wing_trade v1 files of 2026-11-14 17:16).
# "v1.1" = later change from the flutter work: packs at x 0.412, y 0.305 +- 0.071; 60 g mass balance per elevon at
# (x 0.715, y 0.87); 0.10 kg pack thermal kit, assumed 50 g per side carried with the packs. Modes only.
LAYOUTS = {"v1": dict(xb=0.427, yb=(0.259, 0.401), extra_pack=0.0, balance=None),
           "v1.1": dict(xb=0.412, yb=(0.234, 0.376), extra_pack=0.050, balance=(0.715, 0.87, 0.060))}


# ============================================================================ generic model / deck / runner
@dataclass
class Model:
    nodes: list = field(default_factory=list)            # xyz, id = index + 1
    elems: dict = field(default_factory=dict)            # etype -> list[(eid, conn)]
    elsets: dict = field(default_factory=dict)
    nsets: dict = field(default_factory=dict)
    materials: dict = field(default_factory=dict)        # name -> dict
    sections: list = field(default_factory=list)         # (kind, elset, material, thickness)
    masses: list = field(default_factory=list)           # (node, kg)
    rigid: tuple | None = None                           # (nset, ref, rot)
    fixed: list = field(default_factory=list)
    meta: list = field(default_factory=list)             # per element dict
    _eid: int = 0

    def node(self, xyz):
        self.nodes.append(tuple(float(v) for v in xyz))
        return len(self.nodes)

    def elem(self, etype, conn, elset, **meta):
        self._eid += 1
        self.elems.setdefault(etype, []).append((self._eid, tuple(conn)))
        self.elsets.setdefault(elset, []).append(self._eid)
        self.meta.append(dict(eid=self._eid, etype=etype, elset=elset, **meta))
        return self._eid


def ortho(E1, E2, E3, n12, n13, n23, G12, G13, G23, rho):
    C = np.array([[1 / E1, -n12 / E1, -n13 / E1], [-n12 / E1, 1 / E2, -n23 / E2], [-n13 / E1, -n23 / E2, 1 / E3]])
    if np.any(np.linalg.eigvalsh(C) <= 0):
        raise ValueError("Material compliance is not positive definite.")
    return dict(kind="ortho", c=(E1, E2, E3, n12, n13, n23, G12, G13, G23), rho=rho)


def iso(E, nu, rho):
    return dict(kind="iso", c=(E, nu), rho=rho)


def _ids(ids, per=10):
    ids = list(ids)
    return "\n".join(", ".join(str(i) for i in ids[k:k + per]) for k in range(0, len(ids), per))


def write_inp(m: Model, steps, path: Path, ori=(E_AX, N_AX)):
    L = ["*HEADING", "Lodestar wing FE (fea_wing.py)", "*NODE"]
    L += [f"{i + 1}, {x:.9e}, {y:.9e}, {z:.9e}" for i, (x, y, z) in enumerate(m.nodes)]
    for et, rows in m.elems.items():
        L.append(f"*ELEMENT, TYPE={et}, ELSET=E_{et}")
        L += [f"{e}, " + ", ".join(map(str, c)) for e, c in rows]
    for name, ids in m.elsets.items():
        L += [f"*ELSET, ELSET={name}", _ids(ids)]
    for name, ids in m.nsets.items():
        L += [f"*NSET, NSET={name}", _ids(ids)]
    if m.fixed:
        L += ["*NSET, NSET=FIXED", _ids(m.fixed)]
    a, b = ori
    L += ["*ORIENTATION, NAME=ORI, SYSTEM=RECTANGULAR", ", ".join(f"{v:.9f}" for v in (*a, *b))]
    for name, mt in m.materials.items():
        L.append(f"*MATERIAL, NAME={name}")
        if mt["kind"] == "iso":
            L += ["*ELASTIC", f"{mt['c'][0]:.6e}, {mt['c'][1]:.6f}"]
        else:
            c = mt["c"]
            L += ["*ELASTIC, TYPE=ENGINEERING CONSTANTS", ", ".join(f"{v:.6e}" for v in c[:8]), f"{c[8]:.6e}"]
        L += ["*DENSITY", f"{mt['rho']:.6e}"]
    for kind, elset, mat, t in m.sections:
        if kind == "shell":
            L += [f"*SHELL SECTION, ELSET={elset}, MATERIAL={mat}, ORIENTATION=ORI", f"{t:.6e}"]
        else:
            L += [f"*SOLID SECTION, ELSET={elset}, MATERIAL={mat}, ORIENTATION=ORI"]
    if m.masses:
        groups = {}
        for n, kg in m.masses:
            groups.setdefault(round(kg, 9), []).append(n)
        eid = m._eid
        for g, (kg, ns) in enumerate(groups.items()):
            L.append(f"*ELEMENT, TYPE=MASS, ELSET=PM{g}")
            for n in ns:
                eid += 1
                L.append(f"{eid}, {n}")
            L += [f"*MASS, ELSET=PM{g}", f"{kg:.9e}"]
    if m.rigid:
        nset, ref, rot = m.rigid
        L.append(f"*RIGID BODY, NSET={nset}, REF NODE={ref}, ROT NODE={rot}")
    if m.fixed:
        L += ["*BOUNDARY", "FIXED, 1, 3"]
    for st in steps:
        L.append("*STEP")
        if st["type"] == "frequency":
            L += ["*FREQUENCY", str(st["n"])]
        else:
            L.append("*STATIC" + (f", SOLVER={st['solver']}" if st.get("solver") else ""))
            if st.get("cloads"):
                L.append("*CLOAD, OP=NEW")          # ccx keeps earlier steps' loads unless OP=NEW
                L += [f"{n}, {d}, {v:.9e}" for n, d, v in st["cloads"]]
        for ns in st.get("node_print", []):
            L += [f"*NODE PRINT, NSET={ns}", "U"]
        for es in st.get("el_print", []):
            L += [f"*EL PRINT, ELSET={es}", "S, E"]
        L.append("*END STEP")
    path.write_text("\n".join(L) + "\n")


def run_ccx(m: Model, steps, workdir: Path, job="wing", ori=(E_AX, N_AX)):
    workdir.mkdir(parents=True, exist_ok=True)
    inp = workdir / f"{job}.inp"
    old = inp.read_text() if inp.exists() else None
    write_inp(m, steps, inp, ori)
    dat = workdir / f"{job}.dat"
    if old is not None and old == inp.read_text() and dat.exists() and "Job finished" in (workdir / f"{job}.log").read_text():
        return parse_dat(dat), 0.0          # identical deck already solved
    env = dict(os.environ, OMP_NUM_THREADS=str(os.cpu_count() or 4), CCX_NPROC_EQUATION_SOLVER=str(os.cpu_count() or 4))
    t0 = time.time()
    p = subprocess.run([str(CCX), job], cwd=workdir, capture_output=True, text=True, env=env, timeout=3600)
    (workdir / f"{job}.log").write_text(p.stdout + p.stderr)
    if "Job finished" not in p.stdout:
        raise RuntimeError(f"ccx failed in {workdir}:\n{(p.stdout + p.stderr)[-3000:]}")
    return parse_dat(workdir / f"{job}.dat"), time.time() - t0


_HDR = re.compile(r"\s*(displacements|stresses|strains)\s*\(.*\)\s*for set (\S+) and time\s+(\S+)")
_NUM = re.compile(r"^\s*-?\d")


def parse_dat(path: Path):
    """Returns dict(blocks=[(kind, set, time, ndarray)], eig=DataFrame or None, part=DataFrame or None).
    Stress/strain rows are averaged over integration points per element. Shell and solid stresses with an
    orientation are printed in the local (material) system: 1 = spar axis."""
    lines = path.read_text().splitlines()
    blocks, eig, part = [], [], []
    i = 0
    while i < len(lines):
        ln = lines[i]
        h = _HDR.match(ln)
        if h:
            kind, nset, t = h.group(1), h.group(2), float(h.group(3))
            i += 1
            rows = []
            while i < len(lines) and not lines[i].strip():
                i += 1
            while i < len(lines) and _NUM.match(lines[i]):
                p = lines[i].split()
                if kind == "displacements":
                    rows.append([float(v) for v in p[:4]])
                else:
                    rows.append([float(p[0])] + [float(v) for v in p[2:8]])
                i += 1
            a = np.array(rows)
            if kind != "displacements" and len(a):
                df = pd.DataFrame(a, columns=["eid", "c11", "c22", "c33", "c12", "c13", "c23"])
                a = df.groupby("eid", sort=False).mean().reset_index().to_numpy()
            blocks.append((kind, nset, t, a))
            continue
        if "E I G E N V A L U E   O U T P U T" in ln:
            i += 1
            while i < len(lines) and "P A R T I C I P" not in lines[i]:
                p = lines[i].split()
                if len(p) == 5 and p[0].isdigit():
                    eig.append([int(p[0])] + [float(v) for v in p[1:]])
                i += 1
            continue
        if "P A R T I C I P A T I O N" in ln:
            i += 1
            while i < len(lines) and "E F F E C T I V E" not in lines[i]:
                p = lines[i].split()
                if len(p) == 7 and p[0].isdigit():
                    part.append([int(p[0])] + [float(v) for v in p[1:]])
                i += 1
            continue
        i += 1
    eig_df = pd.DataFrame(eig, columns=["mode", "eigenvalue", "omega", "freq_Hz", "imag"]) if eig else None
    part_df = pd.DataFrame(part, columns=["mode", "px", "py", "pz", "rx", "ry", "rz"]) if part else None
    return dict(blocks=blocks, eig=eig_df, part=part_df)


def blocks_of(res, kind, nset=None):
    return [b for b in res["blocks"] if b[0] == kind and (nset is None or b[1] == nset.upper())]


# ============================================================================ simple box (tests)
def box_model(L=2.0, b=0.1, h=0.1, t=1e-3, n_len=40, n_side=4, material=None, axis_y=True):
    """Thin rectangular tube of S4 shells along +y, clamped at y = 0, rigid tip section (REF/ROT nodes)."""
    m = Model()
    mat = material or iso(70e9, 0.33, 2700)
    m.materials["M"] = mat
    per = []                                   # perimeter points (x, z) counter-clockwise
    for k in range(n_side):
        per.append((-b / 2 + b * k / n_side, -h / 2))
    for k in range(n_side):
        per.append((b / 2, -h / 2 + h * k / n_side))
    for k in range(n_side):
        per.append((b / 2 - b * k / n_side, h / 2))
    for k in range(n_side):
        per.append((-b / 2, h / 2 - h * k / n_side))
    ids = np.zeros((n_len + 1, len(per)), int)
    for j in range(n_len + 1):
        for k, (x, z) in enumerate(per):
            ids[j, k] = m.node((x, L * j / n_len, z))
    for j in range(n_len):
        for k in range(len(per)):
            k2 = (k + 1) % len(per)
            m.elem("S4", (ids[j, k], ids[j, k2], ids[j + 1, k2], ids[j + 1, k]), "SH", j=j, k=k)
    m.sections.append(("shell", "SH", "M", t))
    m.fixed = list(ids[0])
    ref = m.node((0, L, 0))
    rot = m.node((0, L, 0))
    m.nsets["TIP"] = list(ids[-1])
    m.rigid = ("TIP", ref, rot)
    m.nsets["LINE_TOP"] = list(ids[:, 3 * n_side - n_side // 2 - 0])      # mid top flange
    m.nsets["LINE_BOT"] = list(ids[:, n_side // 2])
    return m, ids, ref, rot


# ============================================================================ wing geometry
_coords = s.AF.coordinates
_ile = int(np.argmin(_coords[:, 0]))
_up = _coords[:_ile + 1][::-1]
_lo = _coords[_ile:]


def z_up(xc):
    return np.interp(xc, _up[:, 0], _up[:, 1])


def z_lo(xc):
    return np.interp(xc, _lo[:, 0], _lo[:, 1])


def load_tables():
    tab = pd.read_csv(INPUTS / f"{DESIGN}_sections.csv")
    loads = pd.read_csv(INPUTS / f"{DESIGN}_loads.csv")
    return tab, loads


def core_name(y):
    """compact_eps20_light: EPS 20 everywhere except XPS at the servo bay and the winglet root."""
    return "XPS" if abs(y - s.Y_SERVO) < 0.05 or y > 1.40 else "EPS"


def t_cap_at(tab, y):
    i = int(np.argmin(np.abs(tab.y.values - y)))
    return float(tab.t_cap_mm.values[i]) * 1e-3


def wing_materials(m: Model, tcaps):
    m.materials["SKIN"] = ortho(SKIN_E, SKIN_E, SKIN_E, SKIN_NU, SKIN_NU, 0.3, SKIN_G, SKIN_G, SKIN_E / 2.6, SKIN_RHO)
    for t in tcaps:
        tt = TS + t
        E1 = (SKIN_E * TS + CARB_E1 * t) / tt
        E2 = (SKIN_E * TS + CARB_E2 * t) / tt
        G12 = (SKIN_G * TS + CARB_G12 * t) / tt
        rho = (SKIN_RHO * TS + s.CARBON_UD.rho * t) / tt
        m.materials[f"CAP{round(t * 1e6)}"] = ortho(E1, E2, E2, 0.3, 0.3, 0.3, G12, G12, E2 / 2.6, rho)
    Ew = (2 * 0.08e-3 * SKIN_E + CAP_W * s.ROHA51.E) / T_WEB
    Gw = (2 * 0.08e-3 * SKIN_G + CAP_W * s.ROHA51.G) / T_WEB
    rho_w = (CAP_W * s.ROHA51.rho + 2 * s.GLASS45.rho * 0.08e-3) / T_WEB
    m.materials["WEB"] = ortho(Ew, Ew, s.ROHA51.E, 0.3, 0.05, 0.05, Gw, s.ROHA51.G, s.ROHA51.G, rho_w)
    Er = (2 * 0.08e-3 * SKIN_E + 0.003 * s.XPS.E) / T_REAR
    Gr = (2 * 0.08e-3 * SKIN_G + 0.003 * s.XPS.G) / T_REAR
    rho_r = (0.003 * s.XPS.rho + 2 * 0.16) / T_REAR
    m.materials["REAR"] = ortho(Er, Er, s.XPS.E, 0.3, 0.05, 0.05, Gr, s.XPS.G, s.XPS.G, rho_r)
    for nm, f in (("EPS", s.EPS20), ("XPS", s.XPS)):
        m.materials[nm] = ortho(f.E, f.E, f.E, 0.1, 0.1, 0.1, f.G, f.G, f.G, f.rho)


@dataclass
class Wing:
    model: Model
    ys: np.ndarray
    nid: dict                 # (j, i, k) -> node id
    cols: dict                # column indices
    xloc: list                # per row: local x of columns
    ref: int
    rot: int
    wl_node: int
    meta: pd.DataFrame = None


def build_wing(tab, n_span=60, nA=12, nB=10, nC=6, nz=4, battery="P50B", masses=True, layout="v1.1"):
    m = Model()
    ys = np.linspace(0, 1.5, n_span + 1)
    iF, iS, iA = nA, nA + 1, nA + 2
    iR = nA + 2 + nB
    last = iR + nC
    nid, xloc = {}, []
    for j, y in enumerate(ys):
        c = s.GEO.chord(y)
        xs, xr = s.X_SPAR * c, s.X_REAR * c
        a = xs - CAP_WS / 2
        xa = a * (1 - np.cos(np.linspace(0, math.pi / 2, nA + 1)))
        xb = np.linspace(xs + CAP_WS / 2, xr, nB + 1)[1:]
        xcc = np.linspace(xr, c, nC + 1)[1:]
        x = np.r_[xa, xs, xs + CAP_WS / 2, xb, xcc]
        assert len(x) == last + 1
        xloc.append(x)
        x0 = s.GEO.x_le(y)
        for i, xi in enumerate(x):
            zu, zl = z_up(xi / c) * c, z_lo(xi / c) * c
            if i in (0, last):
                zm = 0.5 * (zu + zl)
                n = m.node((x0 + xi, y, zm))
                for k in range(nz + 1):
                    nid[(j, i, k)] = n
            else:
                if i in (iF, iS, iA):
                    # cap strip: put the shell reference surface at the cap centroid (inside the skin), so the
                    # caps have the beam model's lever arm h_eff = h - 2 ts - t_cap instead of the full depth h
                    off = 0.5 * (TS + t_cap_at(tab, y))
                    zu, zl = zu - off, zl + off
                for k in range(nz + 1):
                    nid[(j, i, k)] = m.node((x0 + xi, y, zl + (zu - zl) * k / nz))
    P = np.array(m.nodes)
    tcaps = sorted({t_cap_at(tab, 0.5 * (ys[j] + ys[j + 1])) for j in range(n_span)})
    wing_materials(m, tcaps)

    def hexa(c8):
        p = P[np.array(c8) - 1]
        if np.linalg.det(np.array([p[1] - p[0], p[3] - p[0], p[4] - p[0]])) < 0:
            c8 = (*c8[4:], *c8[:4])
        return c8

    def wedge(c6):
        p = P[np.array(c6) - 1]
        if np.linalg.det(np.array([p[1] - p[0], p[2] - p[0], p[3] - p[0]])) < 0:
            c6 = (*c6[3:], *c6[:3])
        return c6

    for j in range(n_span):
        ym = 0.5 * (ys[j] + ys[j + 1])
        tc = t_cap_at(tab, ym)
        core = core_name(ym)
        for surf, k in (("U", nz), ("L", 0)):
            for i in range(last):
                conn = (nid[(j, i, k)], nid[(j, i + 1, k)], nid[(j + 1, i + 1, k)], nid[(j + 1, i, k)])
                if surf == "L":
                    conn = conn[::-1]
                if i in (iF, iS):
                    m.elem("S4", conn, f"CAP{round(tc * 1e6)}", kind="cap", surf=surf, j=j, i=i, y=ym, tcap=tc)
                else:
                    es = "SKIN"
                    if surf == "U" and DOUBLER[0] <= ym <= DOUBLER[1]:
                        es = "SKIN2"
                    elif surf == "U" and DOUBLER[1] < ym <= DOUBLER[1] + DOUBLER_TAPER:
                        es = "SKIN15"
                    m.elem("S4", conn, es, kind="skin", surf=surf, j=j, i=i, y=ym, core=core)
        for kind, ic in (("web", iS), ("rear", iR)):
            for k in range(nz):
                conn = (nid[(j, ic, k)], nid[(j, ic, k + 1)], nid[(j + 1, ic, k + 1)], nid[(j + 1, ic, k)])
                m.elem("S4", conn, kind.upper(), kind=kind, j=j, i=ic, k=k, y=ym)
        for i in range(last):
            for k in range(nz):
                if i == 0:
                    c6 = (nid[(j, 0, 0)], nid[(j, 1, k)], nid[(j, 1, k + 1)],
                          nid[(j + 1, 0, 0)], nid[(j + 1, 1, k)], nid[(j + 1, 1, k + 1)])
                    m.elem("C3D6", wedge(c6), f"CORE_{core}", kind="core", core=core, j=j, i=i, k=k, y=ym)
                elif i == last - 1:
                    c6 = (nid[(j, i, k)], nid[(j, i, k + 1)], nid[(j, last, 0)],
                          nid[(j + 1, i, k)], nid[(j + 1, i, k + 1)], nid[(j + 1, last, 0)])
                    m.elem("C3D6", wedge(c6), f"CORE_{core}", kind="core", core=core, j=j, i=i, k=k, y=ym)
                else:
                    c8 = (nid[(j, i, k)], nid[(j, i + 1, k)], nid[(j, i + 1, k + 1)], nid[(j, i, k + 1)],
                          nid[(j + 1, i, k)], nid[(j + 1, i + 1, k)], nid[(j + 1, i + 1, k + 1)], nid[(j + 1, i, k + 1)])
                    m.elem("C3D8I", hexa(c8), f"CORE_{core}", kind="core", core=core, j=j, i=i, k=k, y=ym)
    m.sections.append(("shell", "SKIN", "SKIN", TS))
    if "SKIN2" in m.elsets:
        m.sections.append(("shell", "SKIN2", "SKIN", 2 * TS))   # upper-skin doubler (lodestar_v1)
    if "SKIN15" in m.elsets:
        m.sections.append(("shell", "SKIN15", "SKIN", 1.5 * TS))   # doubler ply drop-off
    for t in tcaps:
        nm = f"CAP{round(t * 1e6)}"
        m.sections.append(("shell", nm, nm, TS + t))
    m.sections.append(("shell", "WEB", "WEB", T_WEB))
    m.sections.append(("shell", "REAR", "REAR", T_REAR))
    for core in ("EPS", "XPS"):
        if f"CORE_{core}" in m.elsets:
            m.sections.append(("solid", f"CORE_{core}", core, None))
    # boundary, tip rib, tracking sets
    m.fixed = sorted({nid[(0, i, k)] for i in range(last + 1) for k in range(nz + 1)})
    kz = nz // 2
    tip = sorted({nid[(n_span, i, k)] for i in range(last + 1) for k in range(nz + 1)})
    c_t = s.GEO.chord(1.5)
    wl = m.node((s.GEO.x_le(1.5) + 0.4 * c_t, 1.5, float(P[nid[(n_span, iS, kz)] - 1][2]) + s.GEO.winglet_h / math.sqrt(3)))
    ref = m.node(P[nid[(n_span, iS, kz)] - 1])
    rot = m.node(P[nid[(n_span, iS, kz)] - 1])
    m.nsets["TIP"] = tip + [wl]
    m.rigid = ("TIP", ref, rot)
    m.nsets["TRACK"] = [nid[(j, i, kz)] for j in range(n_span + 1) for i in (0, iS, iR)]
    cols = dict(iF=iF, iS=iS, iA=iA, iR=iR, last=last, nz=nz, kz=kz)
    w = Wing(m, ys, nid, cols, xloc, ref, rot, wl)
    if masses:
        add_masses(w, battery, layout)
    w.meta = pd.DataFrame(m.meta).set_index("eid")
    return w


def nearest_node(w: Wing, x, y, k=None, avoid=()):
    j = int(np.argmin(np.abs(w.ys - y)))
    xl = x - s.GEO.x_le(w.ys[j])
    cand = [i for i in range(1, w.cols["last"]) if i not in avoid]
    i = cand[int(np.argmin([abs(w.xloc[j][i] - xl) for i in cand]))]
    return w.nid[(j, i, w.cols["kz"] if k is None else k)]


def battery_side_mass(battery):
    if battery == "P50B":
        return s.M_BATT_SIDE
    from design_v2 import N_WING, pack
    return pack("Amprius SiCore 6.3 Ah", N_WING)[1] / 2


def joint_masses():
    import wing_trade as wt
    return tuple(wt.COMPACT.masses())


def add_masses(w: Wing, battery, layout="v1.1"):
    """Point masses the shell/solid densities do not carry: wing packs, joint, servo, winglet, hinge line."""
    m = w.model
    avoid = (w.cols["iS"], w.cols["iR"])
    L = LAYOUTS[layout]
    mb = battery_side_mass(battery) + L["extra_pack"]
    # Pack: 15 cells, 5 chordwise x 3 spanwise, bonded in a pocket. Spread over every node inside the pack
    # footprint (all layers), so the pack moves as a block on the foam; lumping it on a few interior foam nodes
    # gives spurious local mass-on-foam modes near 90 Hz.
    P = np.array(m.nodes)
    x0, x1 = L["xb"] - 2.5 * 0.0215, L["xb"] + 2.5 * 0.0215
    y0, y1 = L["yb"][0] - 0.035, L["yb"][1] + 0.035
    inside = sorted({n for n in w.nid.values() if x0 <= P[n - 1, 0] <= x1 and y0 <= P[n - 1, 1] <= y1})
    if len(inside) < 10:
        raise ValueError("Mesh too coarse for the battery footprint.")
    m.masses += [(n, mb / len(inside)) for n in inside]
    for x, y, kg in joint_masses():
        m.masses.append((nearest_node(w, x, y), kg))
    m.masses.append((nearest_node(w, s.GEO.x_le(s.Y_SERVO) + 0.6 * s.GEO.chord(s.Y_SERVO), s.Y_SERVO, avoid=avoid), s.M_SERVO))
    m.masses.append((w.wl_node, s.M_WINGLET))
    if L["balance"]:
        xm, ym, kg = L["balance"]
        near = sorted({n for n in w.nid.values() if abs(P[n - 1, 0] - xm) <= 0.02 and abs(P[n - 1, 1] - ym) <= 0.02})
        m.masses += [(n, kg / len(near)) for n in near]
    dy = np.gradient(w.ys)
    for j, y in enumerate(w.ys):
        if j == 0:
            continue
        m.masses.append((nearest_node(w, s.GEO.x_le(y) + 0.75 * s.GEO.chord(y), y, avoid=avoid), HINGE_KG_M * dy[j]))


def fe_mass(w: Wing):
    """Structural mass of the half-wing from element volumes and densities, plus point masses."""
    m = w.model
    P = np.array(m.nodes)
    dens = {}
    for kind, elset, mat, t in m.sections:
        dens[elset] = (m.materials[mat]["rho"], t)
    tot = 0.0
    for et, rows in m.elems.items():
        for eid, c in rows:
            es = m.meta[eid - 1]["elset"]
            rho, t = dens[es]
            p = P[np.array(c) - 1]
            if et == "S4":
                tot += rho * t * 0.5 * np.linalg.norm(np.cross(p[2] - p[0], p[3] - p[1]))
            else:
                tot += rho * _solid_volume(p, et)
    return tot, sum(kg for _, kg in m.masses)


def _solid_volume(p, et):
    faces = ([(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
              (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)] if et == "C3D8I" else
             [(0, 1, 2), (3, 5, 4), (0, 3, 4), (0, 4, 1), (1, 4, 5), (1, 5, 2), (2, 5, 3), (2, 3, 0)])
    c = p.mean(axis=0)
    return abs(sum(np.dot(p[a] - c, np.cross(p[b] - c, p[d] - c)) for a, b, d in faces) / 6)


# ============================================================================ loads
def case_points(loads, case, factor=FOS):
    """Discrete vertical forces that reproduce a case's shear and torque about the spar at every station:
    per station interval a force at the spar line and one at the rear-web line (the latter carries the torque).
    Returns list of (x_global_frac 'spar'/'rear', y, F)."""
    d = loads[loads.case == case].sort_values("y")
    y, V, T = d.y.values, d.V.values * factor, d.T_spar.values * factor
    ym = np.r_[0.5 * (y[:-1] + y[1:]), y[-1]]
    dV = np.r_[V[:-1] - V[1:], V[-1]]
    dT = np.r_[T[:-1] - T[1:], T[-1]]
    pts = []
    for yy, f, t in zip(ym, dV, dT):
        c = s.GEO.chord(yy)
        fb = -t / (CL * (s.X_REAR - s.X_SPAR) * c)
        pts += [("spar", yy, f - fb), ("rear", yy, fb)]
    return pts


def cut_loads(pts, y_st):
    """Same cut convention as structures.spar_loads (streamwise cuts at y_st, moments about the spar point)."""
    x = np.array([s.GEO.x_le(y) + (s.X_SPAR if w == "spar" else s.X_REAR) * s.GEO.chord(y) for w, y, _ in pts])
    y = np.array([p[1] for p in pts])
    F = np.array([p[2] for p in pts])
    out = []
    for yc in y_st:
        o = y > yc - 1e-9          # inclusive at the tip station, as spar_loads (y_m >= y_st)
        xc = s.GEO.x_le(yc) + s.X_SPAR * s.GEO.chord(yc)
        mx = np.sum((y[o] - yc) * F[o])
        my = np.sum((xc - x[o]) * F[o])
        out.append((F[o].sum(), mx * N_AX[0] + my * N_AX[1], mx * E_AX[0] + my * E_AX[1]))
    return np.array(out)


def nodal_loads(w: Wing, pts):
    """Distribute the case forces to the FE rows (linear in y) and over the web nodes of each row."""
    ys = w.ys
    Fa, Fb = np.zeros(len(ys)), np.zeros(len(ys))
    for where, y, f in pts:
        j = min(int(np.searchsorted(ys, y, side="right") - 1), len(ys) - 2)
        t = (y - ys[j]) / (ys[j + 1] - ys[j])
        arr = Fa if where == "spar" else Fb
        arr[j] += (1 - t) * f
        arr[j + 1] += t * f
    cl = []
    nz = w.cols["nz"]
    for j in range(1, len(ys)):
        for arr, ic in ((Fa, w.cols["iS"]), (Fb, w.cols["iR"])):
            if arr[j]:
                for k in range(nz + 1):
                    cl.append((w.nid[(j, ic, k)], 3, arr[j] / (nz + 1)))
    return cl


# ============================================================================ post-processing
def disp_table(res, nset="TRACK", idx=0):
    b = blocks_of(res, "displacements", nset)[idx][3]
    return pd.DataFrame(b, columns=["node", "ux", "uy", "uz"]).set_index("node")


def elem_results(res, w: Wing, step):
    """Per-element mean local stress and strain for one static step (index), joined with element metadata."""
    S = [b for b in res["blocks"] if b[0] == "stresses"]
    E = [b for b in res["blocks"] if b[0] == "strains"]
    times = sorted({b[2] for b in S})
    t = times[step]
    sd = pd.concat([pd.DataFrame(b[3], columns=["eid", "s11", "s22", "s33", "s12", "s13", "s23"]) for b in S if b[2] == t])
    ed = pd.concat([pd.DataFrame(b[3], columns=["eid", "e11", "e22", "e33", "e12", "e13", "e23"]) for b in E if b[2] == t])
    df = sd.merge(ed, on="eid").astype({"eid": int}).set_index("eid").join(w.meta)
    return df


def twist_bend(w: Wing, U):
    """Spar-line vertical deflection and streamwise twist (nose-up +) per row from the TRACK displacements."""
    rows = []
    kz = w.cols["kz"]
    for j, y in enumerate(w.ys):
        ns, nr = w.nid[(j, w.cols["iS"], kz)], w.nid[(j, w.cols["iR"], kz)]
        dx = w.xloc[j][w.cols["iR"]] - w.xloc[j][w.cols["iS"]]
        us, ur = U.loc[ns], U.loc[nr]
        rows.append(dict(y=y, w=us.uz, theta=(us.uz - ur.uz) / dx, ux=us.ux, uy=us.uy))
    return pd.DataFrame(rows)


def skin_principal(df):
    a, b, c = df.s11, df.s22, df.s12
    r = np.sqrt(((a - b) / 2) ** 2 + c**2)
    return (a + b) / 2 - r, (a + b) / 2 + r


def station_results(df, w: Wing):
    """Per element row: cap stress (from strain), skin min principal stress, skin shear, web shear, core stress."""
    out = []
    for j, g in df.groupby("j"):
        y = g.y.iloc[0]
        r = dict(j=int(j), y=y, core=core_name(y))
        for surf in ("U", "L"):
            cap = g[(g.kind == "cap") & (g.surf == surf)]
            e11 = cap.e11.mean()
            r[f"cap_{surf}_MPa"] = CARB_E1 * e11 / 1e6
            # skin over the cap: cap strain extrapolated from the cap centroid to the outer surface (h / h_eff,
            # as the beam model), skin plane-stress law
            c = s.GEO.chord(y)
            h = s.thick(s.X_SPAR) * c
            ext = h / (h - 2 * TS - cap.tcap.iloc[0])
            Q = SKIN_E / (1 - SKIN_NU**2)
            s11 = Q * (ext * cap.e11 + SKIN_NU * cap.e22)
            s22 = Q * (cap.e22 + SKIN_NU * ext * cap.e11)
            s12 = SKIN_G * 2 * cap.e12
            sk = g[(g.kind == "skin") & (g.surf == surf)]
            pmin, _ = skin_principal(sk)
            rr = np.sqrt(((s11 - s22) / 2) ** 2 + s12**2)
            pmin_cap = (s11 + s22) / 2 - rr
            r[f"skin_{surf}_min_MPa"] = min(pmin.min(), pmin_cap.min()) / 1e6
            r[f"skin_{surf}_s11_min_MPa"] = min(sk.s11.min(), s11.min()) / 1e6
            r[f"skin_{surf}_tau_MPa"] = sk.s12.abs().max() / 1e6
        web = g[g.kind == "web"]
        r["web_gamma"] = (2 * web.e12).abs().max()
        r["web_q_Nmm"] = (web.s12.abs() * T_WEB).max() / 1e3
        rear = g[g.kind == "rear"]
        r["rear_gamma"] = (2 * rear.e12).abs().max()
        core = g[g.kind == "core"]
        tau = np.sqrt(core.s12**2 + core.s13**2 + core.s23**2)
        r["core_tau_kPa"] = tau.max() / 1e3
        r["core_comp_kPa"] = -core[["s11", "s22", "s33"]].min(axis=1).min() / 1e3
        out.append(r)
    return pd.DataFrame(out)


def beam_case(tab, loads, case, factor=FOS):
    """Beam-model (structures.py) quantities for one case at the section-table stations."""
    d = loads[loads.case == case].sort_values("y")
    Mb, T, V = d.Mb.values * factor, d["T"].values * factor, d.V.values * factor
    rows = []
    for (_, r), mb, t, v in zip(tab.iterrows(), Mb, T, V):
        sec = s.section(r.chord, LAY, r.t_cap_mm / 1e3, s.EPS20 if "EPS" in r.core else s.XPS)
        A = CAP_W * r.t_cap_mm * 1e-3
        sig = mb / (A * sec["h_eff"])
        rows.append(dict(y=r.y, Mb=mb, T=t, V=v, cap_U_MPa=-sig / 1e6, cap_L_MPa=sig / 1e6,
                         skin_comp_MPa=abs(sig) * SKIN_E / CARB_E1 * sec["h"] / sec["h_eff"] / 1e6,
                         skin_tau_MPa=abs(t) / (2 * sec["A_cell"] * TS) / 1e6, web_q_Nmm=abs(v) / sec["h_eff"] / 1e3))
    return pd.DataFrame(rows)


# ============================================================================ frequencies
def classify_modes(w: Wing, res, n):
    """Label each mode from the spar-line deflection, streamwise twist and in-plane motion at the tip."""
    blocks = blocks_of(res, "displacements", "TRACK")
    rows, shapes = [], []
    c_t = s.GEO.chord(1.5)
    for mde in range(n):
        U = pd.DataFrame(blocks[mde][3], columns=["node", "ux", "uy", "uz"]).set_index("node")
        tb = twist_bend(w, U)
        tip = tb.iloc[-1]
        amp_b, amp_t, amp_i = abs(tip.w), abs(tip.theta) * c_t / 2, math.hypot(tip.ux, tip.uy)
        # in-plane only when it clearly dominates: a pitching tip section rocks the raised winglet mass fore-aft,
        # so torsion modes carry tip in-plane motion of the same size as the twist
        kind = "in-plane" if amp_i > 2 * max(amp_b, amp_t) else "bending" if amp_b >= amp_t else "torsion"
        sign_changes = int(np.sum(np.diff(np.sign(tb.w.values[2:])) != 0))
        rows.append(dict(mode=mde + 1, freq_Hz=res["eig"].freq_Hz.iloc[mde], type=kind, w_nodes=sign_changes,
                         tip_w=tip.w, tip_theta_rad=tip.theta, tip_inplane=amp_i,
                         twist_to_bend_ratio=amp_t / max(amp_b, 1e-30)))
        sc = max(np.abs(tb.w).max(), np.abs(tb.theta).max() * c_t / 2, 1e-30)
        shapes.append(tb.assign(mode=mde + 1, w=tb.w / sc, theta=tb.theta / sc, ux=tb.ux / sc, uy=tb.uy / sc))
    return pd.DataFrame(rows), pd.concat(shapes)


def name_modes(df):
    counters = {}
    names = []
    for r in df.itertuples():
        counters[r.type] = counters.get(r.type, 0) + 1
        names.append(f"{r.type} {counters[r.type]}")
    return df.assign(name=names)


class beam_v1:
    """Temporarily set the structures.py pack mass and the v1 pack position used by wing_trade "v1"."""

    def __init__(self, battery, layout="v1.1"):
        self.battery, self.L = battery, LAYOUTS[layout]

    def __enter__(self):
        self.old = (s.M_BATT_SIDE, s.Y_BATT, s.X_BATT_GLOBAL)
        s.M_BATT_SIDE = battery_side_mass(self.battery) + self.L["extra_pack"]
        s.Y_BATT, s.X_BATT_GLOBAL = self.L["yb"], self.L["xb"]

    def __exit__(self, *exc):
        s.M_BATT_SIDE, s.Y_BATT, s.X_BATT_GLOBAL = self.old


def beam_modes(tab, battery="P50B", ne=60, n=6, swept=False, layout="v1.1"):
    """In-vacuo natural frequencies of the structures.py flutter beam (projected-span straight beam, same masses
    as the flutter screen). swept=True re-lays the same beam along the 25 deg spar: length / cos, mass and
    inertia per unit length x cos, GJ x cos (Bredt over the perpendicular section); EI unchanged."""
    bal = LAYOUTS[layout]["balance"]
    extra = joint_masses() + ((bal,) if bal else ())
    with beam_v1(battery, layout):
        yn, EI, GJ, m, S_a, I_a, b, a = s.wing_beam(tab, ne=ne, extra_masses=extra)
    if swept:
        yn, m, S_a, I_a, GJ = yn / CL, m * CL, S_a * CL, I_a * CL, GJ * CL
    K, M = s.beam_matrices(yn, EI, GJ, m, S_a, I_a)
    lam, V = eigh(K, M)
    rows = []
    for i in range(n):
        v = V[:, i]
        ew = v[0::3] @ (M[0::3, 0::3] @ v[0::3])
        et = v[2::3] @ (M[2::3, 2::3] @ v[2::3])
        rows.append(dict(mode=i + 1, freq_Hz=math.sqrt(lam[i]) / (2 * math.pi), type="bending" if ew > et else "torsion"))
    return pd.DataFrame(rows), (yn, K, M, lam, V, EI)


# ============================================================================ main analyses
def log(*a):
    msg = " ".join(str(x) for x in a)
    print(msg, flush=True)
    with open(OUT / "run.log", "a", encoding="utf-8") as f:
        f.write(msg + "\n")


MESHES = {"coarse": dict(n_span=30, nA=6, nB=5, nC=3, nz=2),
          "medium": dict(n_span=60, nA=12, nB=10, nC=6, nz=4),
          "fine": dict(n_span=120, nA=20, nB=18, nC=10, nz=6)}


def run_modes(tab, mesh="medium", battery="P50B", n=10, layout="v1.1"):
    w = build_wing(tab, **MESHES[mesh], battery=battery, layout=layout)
    tag = "" if layout == "v1" else f"_{layout}"
    res, dt = run_ccx(w.model, [dict(type="frequency", n=n, node_print=["TRACK"])],
                      OUT / f"ccx_modes_{mesh}_{battery}{tag}", "modes")
    md, shapes = classify_modes(w, res, n)
    md = name_modes(md)
    ms, mp = fe_mass(w)
    log(f"modes {mesh} {battery}: {dt:.0f} s, FE structure {ms:.3f} kg + point masses {mp:.3f} kg (half wing)")
    return w, md, shapes, (ms, mp)


def write_modes(tab, layouts=("v1", "v1.1")):
    """modes.csv: FE modes (source 'FE shell/solid') and, for reference, the flutter beam's in-vacuo modes as
    used by the flutter screen ('beam, projected span') and re-laid along the swept spar ('beam, swept length')."""
    rows, shapes_all = [], []
    c_t = s.GEO.chord(1.5)
    for layout, battery in ((lo, b) for lo in layouts for b in ("P50B", "SiCore")):
        w, md, shapes, (ms, mp) = run_modes(tab, "medium", battery, layout=layout)
        cfg = (f"{battery}, v1 packs, clamped at centreline" if layout == "v1" else
               f"{battery}, v1.1 layout (packs x0.412 y0.305, 60 g elevon balance, +50 g/side), clamped")
        for r in md.itertuples():
            rows.append(dict(config=cfg, source="FE shell/solid", mode=r.mode, name=r.name, freq_Hz=r.freq_Hz,
                             w_nodes=r.w_nodes, tip_w=r.tip_w, tip_theta_x_semichord=r.tip_theta_rad * c_t / 2,
                             tip_inplane=r.tip_inplane, half_wing_kg=ms + mp))
        for src, sw in (("beam, projected span", False), ("beam, swept length", True)):
            bm, _ = beam_modes(tab, battery, swept=sw, layout=layout)
            cnt = {}
            for r in bm.itertuples():
                cnt[r.type] = cnt.get(r.type, 0) + 1
                rows.append(dict(config=cfg, source=src, mode=r.mode, name=f"{r.type} {cnt[r.type]}", freq_Hz=r.freq_Hz))
        shapes_all.append(shapes.assign(config=cfg))
        # written after each configuration so the flutter work can start on the first one
        pd.DataFrame(rows).to_csv(OUT / "modes.csv", index=False)
        pd.concat(shapes_all).to_csv(OUT / "modes_shapes.csv", index=False)
    return pd.DataFrame(rows)


def governing_cases(loads):
    nl = loads[~loads.sensitivity]
    root = nl[np.isclose(nl.y, 0)]
    br = nl[np.isclose(nl.y, s.Y_BREAK)]
    return dict(pos=root.loc[root.Mb.idxmax(), "case"], neg=root.loc[root.Mb.idxmin(), "case"],
                tor=nl.loc[nl["T"].abs().idxmax(), "case"], brk=br.loc[br.Mb.abs().idxmax(), "case"],
                sens=loads[loads.sensitivity & np.isclose(loads.y, s.Y_BREAK)].sort_values("Mb").case.iloc[-1])


def run_static(tab, loads, mesh="medium", cases=None):
    w = build_wing(tab, **MESHES[mesh], masses=False)
    steps = []
    for key, case in cases.items():
        steps.append(dict(type="static", cloads=nodal_loads(w, case_points(loads, case)), node_print=["TRACK"],
                          el_print=["E_S4", "E_C3D8I", "E_C3D6"]))
    T0, M0 = 10.0, 10.0
    steps.append(dict(type="static", cloads=[(w.rot, d + 1, T0 * E_AX[d]) for d in range(2)], node_print=["TRACK"]))
    steps.append(dict(type="static", cloads=[(w.rot, d + 1, M0 * N_AX[d]) for d in range(2)], node_print=["TRACK"]))
    res, dt = run_ccx(w.model, steps, OUT / f"ccx_static_{mesh}", "static")
    log(f"static {mesh}: {len(w.model.nodes)} nodes, {len(w.meta)} elements, {dt:.0f} s")
    out = {}
    disp = blocks_of(res, "displacements", "TRACK")
    for n, key in enumerate(cases):
        U = pd.DataFrame(disp[n][3], columns=["node", "ux", "uy", "uz"]).set_index("node")
        out[key] = dict(tb=twist_bend(w, U), st=station_results(elem_results(res, w, n), w))
    ntot = len(cases)
    tbT = twist_bend(w, pd.DataFrame(disp[ntot][3], columns=["node", "ux", "uy", "uz"]).set_index("node"))
    tbM = twist_bend(w, pd.DataFrame(disp[ntot + 1][3], columns=["node", "ux", "uy", "uz"]).set_index("node"))
    dth = np.gradient(tbT.theta.values, tbT.y.values)
    s_ax = tbM.y.values / CL
    kappa = np.gradient(np.gradient(tbM.w.values, s_ax), s_ax)
    stiff = pd.DataFrame(dict(y=tbT.y, GJ_perp=T0 / dth, GJ_stream_equiv=T0 / dth / CL, EI_fe=M0 / kappa))
    return w, out, stiff


def fe_vs_beam(tab, loads, w, out, stiff, cases):
    rows, stn = [], []
    for key, case in cases.items():
        bc = beam_case(tab, loads, case)
        st = out[key]["st"]
        tb = out[key]["tb"]
        lim = loads[loads.case == case].sort_values("y").Mb.values
        w_beam = s.tip_deflection(tab, lim)[-1]
        g = st.copy()
        for col in ("cap_U_MPa", "cap_L_MPa", "skin_comp_MPa", "skin_tau_MPa", "web_q_Nmm", "Mb", "T"):
            g[f"beam_{col}"] = np.interp(g.y, bc.y, bc[col])
        g["case_key"], g["case"] = key, case
        stn.append(g)
        for yq in (0.10, 0.30, 0.60, 1.00):
            r = g.iloc[int(np.argmin(np.abs(g.y - yq)))]
            cap_fe = r.cap_U_MPa if r.beam_Mb > 0 else r.cap_L_MPa
            cap_bm = r.beam_cap_U_MPa if r.beam_Mb > 0 else r.beam_cap_L_MPa
            skin_fe = r.skin_U_s11_min_MPa if r.beam_Mb > 0 else r.skin_L_s11_min_MPa
            rows.append(dict(case_key=key, y=r.y, cap_fe=cap_fe, cap_beam=cap_bm, cap_ratio=cap_fe / cap_bm,
                             skin_comp_fe=-skin_fe, skin_comp_beam=r.beam_skin_comp_MPa,
                             skin_tau_fe=max(r.skin_U_tau_MPa, r.skin_L_tau_MPa), skin_tau_beam=r.beam_skin_tau_MPa,
                             web_q_fe=r.web_q_Nmm, web_q_beam=r.beam_web_q_Nmm))
        rows.append(dict(case_key=key, y=1.5, tip_w_limit_fe_mm=1e3 * tb.w.iloc[-1] / FOS, tip_w_limit_beam_mm=1e3 * w_beam))
    sdf = pd.concat(stn)
    sdf.to_csv(OUT / "fe_stations.csv", index=False)
    cmp_ = pd.DataFrame(rows)
    st = stiff.copy()
    st["GJ_beam"] = np.interp(st.y, tab.y, tab.GJ)
    st["EI_beam"] = np.interp(st.y, tab.y, tab.EI)
    st.to_csv(OUT / "stiffness.csv", index=False)
    return cmp_, sdf, st


def convergence(tab, loads, case):
    rows = []
    for mesh in MESHES:
        w = build_wing(tab, **MESHES[mesh], masses=True)
        # the fine mesh (~0.2 M dof) exhausts the direct SPOOLES solver here, so it uses ccx's iterative Cholesky
        steps = [dict(type="static", cloads=nodal_loads(w, case_points(loads, case)), node_print=["TRACK"],
                      el_print=["E_S4"], solver="ITERATIVE CHOLESKY" if mesh == "fine" else None)]
        res, dt = run_ccx(w.model, steps, OUT / f"ccx_conv_{mesh}", "conv")
        U = disp_table(res)
        tb = twist_bend(w, U)
        df = elem_results_shell_only(res, w)
        f, dtf = {}, 0.0
        if mesh != "fine":       # frequencies are converged coarse -> medium (0.3%); fine eigen-solve too large here
            resf, dtf = run_ccx(w.model, [dict(type="frequency", n=6, node_print=["TRACK"])], OUT / f"ccx_conv_{mesh}", "convf")
            md = name_modes(classify_modes(w, resf, 6)[0])
            f = {r.name: r.freq_Hz for r in md.itertuples()}
        rows.append(dict(mesh=mesh, nodes=len(w.model.nodes), elements=len(w.meta), tip_w_ult_mm=1e3 * tb.w.iloc[-1],
                         cap_U_y0p3_MPa=df.loc[np.argmin(np.abs(df.y - 0.3)), "cap_U_MPa"],
                         skin_U_min_y0p3_MPa=df.loc[np.argmin(np.abs(df.y - 0.3)), "skin_U_min_MPa"],
                         skin_U_min_max_MPa=df[df.y > 0.06].skin_U_min_MPa.min(),
                         f_bend1=f.get("bending 1"), f_bend2=f.get("bending 2"), f_tors1=f.get("torsion 1"),
                         f_inplane1=f.get("in-plane 1"), time_s=dt + dtf))
        log(rows[-1])
    return pd.DataFrame(rows)


def elem_results_shell_only(res, w):
    S = [b for b in res["blocks"] if b[0] == "stresses"]
    E = [b for b in res["blocks"] if b[0] == "strains"]
    sd = pd.DataFrame(S[0][3], columns=["eid", "s11", "s22", "s33", "s12", "s13", "s23"])
    ed = pd.DataFrame(E[0][3], columns=["eid", "e11", "e22", "e33", "e12", "e13", "e23"])
    df = sd.merge(ed, on="eid").astype({"eid": int}).set_index("eid").join(w.meta)
    out = []
    for j, g in df.groupby("j"):
        r = dict(j=j, y=g.y.iloc[0])
        cap = g[(g.kind == "cap") & (g.surf == "U")]
        r["cap_U_MPa"] = CARB_E1 * cap.e11.mean() / 1e6
        sk = g[(g.kind == "skin") & (g.surf == "U")]
        r["skin_U_min_MPa"] = skin_principal(sk)[0].min() / 1e6
        out.append(r)
    return pd.DataFrame(out)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "run.log").write_text(f"fea_wing.py run {time.strftime('%Y-%m-%d %H:%M')}\n", encoding="utf-8")
    tab, loads = load_tables()
    md = write_modes(tab)
    log(md.round(3).to_string())
    if sys.argv[1:] == ["--modes"]:
        sys.exit(0)
    import fea_extra as fx       # statics, load-case matrix, extra cases, joint checks, margins
    fx.main(tab, loads)
