"""Small AVL driver for the Rescue Wing: writes the geometry and mass files, runs
avl.exe with keystrokes (trim, stability derivatives, strip loads, eigenmodes),
and parses the results.

Why not asb.AVL? AeroSandbox's wrapper ties every control surface to one fixed
deflection, so it can't trim the elevons for Cm = 0. Here AVL does the trimming.
"""
from dataclasses import dataclass, field
from pathlib import Path
import math
import re
import subprocess
import tempfile

import numpy as np
import aerosandbox as asb

ROOT = Path(__file__).resolve().parents[1]
AVL_EXE = ROOT.parent / "aero-tools" / "avl" / "avl.exe"
NUM = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[EeDd][-+]?\d+)?"


@dataclass
class Geometry:
    span: float = 3.0
    c_root: float = 0.5911
    c_tip: float = 0.2660
    sweep_qc_deg: float = 25.0
    washout_deg: float = 3.0       # tip nose-down twist relative to root
    root_incidence_deg: float = 0.0
    dihedral_deg: float = 0.0
    y_break: float = 0.45          # elevons run from here to the tip
    hinge_xc: float = 0.75         # elevon hinge at 75% chord (25% chord elevons)
    winglet_h: float = 0.20        # m, 0 = no winglets
    winglet_taper: float = 0.6
    winglet_sweep_deg: float = 35.0
    winglet_cant_deg: float = 0.0  # 0 = vertical
    airfoil: str = "mh45"
    # Optional centre pod as an AVL slender body: ((x, r_equivalent), ...) from its nose, plus its nose position
    body_xr: tuple = ()
    body_x0: float = 0.0
    body_z0: float = 0.0

    @property
    def S(self):
        return self.span * (self.c_root + self.c_tip) / 2

    @property
    def mac(self):
        lam = self.c_tip / self.c_root
        return 2 / 3 * self.c_root * (1 + lam + lam**2) / (1 + lam)

    def x_le(self, y):
        """Leading-edge x at spanwise station y (quarter-chord line swept)."""
        c = self.chord(y)
        return abs(y) * math.tan(math.radians(self.sweep_qc_deg)) + 0.25 * self.c_root - 0.25 * c

    def chord(self, y):
        return self.c_root + (self.c_tip - self.c_root) * abs(y) / (self.span / 2)

    def twist(self, y):
        return self.root_incidence_deg - self.washout_deg * abs(y) / (self.span / 2)

    def z(self, y):
        return abs(y) * math.tan(math.radians(self.dihedral_deg))

    @property
    def x_mac_le(self):
        lam = self.c_tip / self.c_root
        y_mac = self.span / 6 * (1 + 2 * lam) / (1 + lam)
        return self.x_le(y_mac)


def write_airfoil(path: Path, name: str):
    coords = asb.Airfoil(name).coordinates
    with open(path, "w") as f:
        f.write(name.upper() + "\n")
        for x, y in coords:
            f.write(f"{x:.6f} {y:.6f}\n")


def write_avl(g: Geometry, xref: float, path: Path, cdp: float = 0.0):
    af = f"{g.airfoil}.dat"
    write_airfoil(path.parent / af, g.airfoil)
    yb, yt = g.y_break, g.span / 2

    def section(y, controls=True):
        s = (f"SECTION\n{g.x_le(y):.5f} {y:.5f} {g.z(y):.5f} {g.chord(y):.5f} {g.twist(y):.4f}\n"
             f"AFILE\n{af}\n")
        if controls:
            s += f"CONTROL\nelevon 1.0 {g.hinge_xc} 0 0 0 1\nCONTROL\naileron 1.0 {g.hinge_xc} 0 0 0 -1\n"
        return s

    txt = (f"Rescue Wing\n0.0\n0 0 0.0\n{g.S:.5f} {g.mac:.5f} {g.span:.5f}\n{xref:.5f} 0 0\n{cdp:.5f}\n"
           "#--------------------------------------------------\n"
           "SURFACE\nWing\n10 1.0 32 -1.1\nYDUPLICATE\n0.0\n"
           + section(0.0, controls=False) + section(yb) + section(yt))
    if g.winglet_h > 0:
        xt, zt, ct = g.x_le(yt), g.z(yt), g.chord(yt)
        cant = math.radians(g.winglet_cant_deg)
        y2 = yt + g.winglet_h * math.sin(cant)
        z2 = zt + g.winglet_h * math.cos(cant)
        x2 = xt + g.winglet_h * math.tan(math.radians(g.winglet_sweep_deg)) + 0.25 * ct * (1 - g.winglet_taper)
        txt += ("#--------------------------------------------------\n"
                "SURFACE\nWinglet\n8 1.0 8 1.0\nYDUPLICATE\n0.0\n"
                f"SECTION\n{xt:.5f} {yt:.5f} {zt:.5f} {ct:.5f} 0\n"
                f"SECTION\n{x2:.5f} {y2:.5f} {z2:.5f} {ct * g.winglet_taper:.5f} 0\n")
    if g.body_xr:
        xs = [p[0] for p in g.body_xr]
        rs = [p[1] for p in g.body_xr]
        with open(path.parent / "pod.dat", "w") as fh:
            fh.write("POD\n")
            for x, r in zip(reversed(xs), reversed(rs)):
                fh.write(f"{x:.5f} {r:.5f}\n")
            for x, r in zip(xs[1:], rs[1:]):
                fh.write(f"{x:.5f} {-r:.5f}\n")
        txt += ("#--------------------------------------------------\n"
                f"BODY\nPod\n24 1.0\nTRANSLATE\n{g.body_x0:.5f} 0 {g.body_z0:.5f}\nBFIL\npod.dat\n")
    path.write_text(txt)


@dataclass
class MassItem:
    name: str
    m: float
    x: float
    y: float = 0.0
    z: float = 0.0
    I: tuple = (0.0, 0.0, 0.0)  # own Ixx, Iyy, Izz


def wing_mass_items(g: Geometry, m_total: float, n: int = 12):
    """Spread wing structure mass along the span in proportion to local area,
    each strip at 40% of its chord. Gives realistic roll and yaw inertia."""
    ys = (np.arange(n) + 0.5) / n * g.span / 2
    w = np.array([g.chord(y) for y in ys])
    w = w / w.sum() * m_total / 2
    items = []
    for side in (1, -1):
        for y, mi in zip(ys, w):
            items.append(MassItem(f"wing strip", mi, g.x_le(y) + 0.4 * g.chord(y), side * y, g.z(y)))
    return items


def cg_and_inertia(items):
    m = sum(i.m for i in items)
    x = sum(i.m * i.x for i in items) / m
    y = sum(i.m * i.y for i in items) / m
    z = sum(i.m * i.z for i in items) / m
    Ixx = sum(i.m * ((i.y - y) ** 2 + (i.z - z) ** 2) + i.I[0] for i in items)
    Iyy = sum(i.m * ((i.x - x) ** 2 + (i.z - z) ** 2) + i.I[1] for i in items)
    Izz = sum(i.m * ((i.x - x) ** 2 + (i.y - y) ** 2) + i.I[2] for i in items)
    Ixz = sum(i.m * (i.x - x) * (i.z - z) for i in items)
    return dict(m=m, x=x, y=y, z=z, Ixx=Ixx, Iyy=Iyy, Izz=Izz, Ixz=Ixz)


def write_mass(items, path: Path, rho: float):
    lines = ["Lunit = 1.0 m", "Munit = 1.0 kg", "Tunit = 1.0 s", "g = 9.81", f"rho = {rho:.4f}",
             "*   1.    1.    1.    1.     1.     1.      1.", "+   0.    0.    0.    0.     0.     0.      0."]
    for i in items:
        lines.append(f"{i.m:.5f} {i.x:.5f} {i.y:.5f} {i.z:.5f} {i.I[0]:.6f} {i.I[1]:.6f} {i.I[2]:.6f}  ! {i.name}")
    path.write_text("\n".join(lines) + "\n")


def parse_eq(text: str) -> dict:
    out = {}
    for k, v in re.findall(r"([A-Za-z][\w'/]*)\s*=\s*(" + NUM + ")", text):
        out.setdefault(k, float(v.replace("D", "E").replace("d", "e")))
    return out


def parse_strips(text: str):
    """Strip-force table from the FS command: returns arrays of y, chord, cl (wing surface only)."""
    rows = []
    in_wing = False
    cols = None
    for line in text.splitlines():
        if "Surface #" in line:
            in_wing = "Wing" in line and "Winglet" not in line and "(YDUP)" not in line
        parts = line.split()
        if parts[:1] == ["j"]:
            cols = parts  # header row, e.g. j Xle Yle Zle Chord Area c_cl ai cl_norm cl cd ...
            continue
        if in_wing and cols and len(parts) == len(cols) and parts[0].isdigit():
            try:
                vals = dict(zip(cols, (float(p) for p in parts)))
            except ValueError:
                continue
            rows.append((vals["Yle"], vals["Chord"], vals["cl"]))
    a = np.array(rows) if rows else np.zeros((0, 3))
    return a[:, 0], a[:, 1], a[:, 2]


@dataclass
class TrimResult:
    st: dict
    strips: tuple
    eig: list = field(default_factory=list)
    ok: bool = True


def run_trim(g: Geometry, items, V: float, rho: float, modes: bool = False,
             workdir: Path | None = None, timeout: float = 60, cdp: float = 0.0) -> TrimResult:
    """Trimmed level flight at speed V (CL set by weight), elevon trims Cm = 0,
    aileron trims roll moment = 0. Returns stability derivatives (ST), strip loads (FS)
    and, if modes=True, AVL's eigenvalues."""
    mi = cg_and_inertia(items)
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(workdir) if workdir else Path(tmp)
        d.mkdir(parents=True, exist_ok=True)
        for f in ("st.txt", "fs.txt", "eig.txt"):
            (d / f).unlink(missing_ok=True)
        write_avl(g, mi["x"], d / "rw.avl", cdp=cdp)  # cdp: profile + body drag, so phugoid damping is realistic
        write_mass(items, d / "rw.mass", rho)
        keys = ["mass rw.mass", "mset 0", "plop", "g", "", "oper",
                "c1", f"v {V}", f"m {mi['m']}", f"d {rho}", "g 9.81", f"x {mi['x']}", f"z {mi['z']}", "",
                "d1 pm 0", "d2 rm 0", "x", "st", "st.txt", "fs", "fs.txt", ""]
        if modes:
            keys += ["mode", "n", "w", "eig.txt", ""]
        keys += ["quit", ""]
        try:
            subprocess.run([str(AVL_EXE), "rw.avl"], cwd=d, input="\n".join(keys) + "\n", text=True,
                           capture_output=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return TrimResult({}, (np.array([]),) * 3, ok=False)
        if not (d / "st.txt").exists():
            return TrimResult({}, (np.array([]),) * 3, ok=False)
        st = parse_eq((d / "st.txt").read_text())
        strips = parse_strips((d / "fs.txt").read_text()) if (d / "fs.txt").exists() else (np.array([]),) * 3
        eig = []
        if modes and (d / "eig.txt").exists():
            for line in (d / "eig.txt").read_text().splitlines():
                p = line.split()
                if len(p) == 3 and not line.strip().startswith("#"):
                    try:
                        eig.append(complex(float(p[1]), float(p[2])))
                    except ValueError:
                        pass
        return TrimResult(st, strips, eig)
