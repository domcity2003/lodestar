"""fcad build engine: runs INSIDE FreeCAD's own Python (FreeCADCmd.exe), never in the project venv.

It reads a job JSON (path in the environment variable FCAD_JOB), builds every shape described in it
from a small tree of primitives and operations, exports STL/STEP, computes mass properties and the
requested geometric checks, and writes a result JSON (path given in the job as "result").

FreeCADCmd does not hand script arguments to the script in the usual way (extra arguments are treated
as files to open), which is why the job path travels in an environment variable.

Units: the job is written in any length unit; "scale_to_mm" (e.g. 1000 for metres) converts it to the
millimetres FreeCAD/OpenCASCADE works in. Every number in the result comes back in job units, STL files
are written in job units and the STEP file in millimetres (STEP carries its unit, so CAD programs open it
at true size). The job schema is documented in README.md next to this file.
"""
import json
import math
import os
import sys
import time
import traceback

import numpy as np

import FreeCAD
import Part
import MeshPart

V = FreeCAD.Vector
LOG = []


def log(msg):
    LOG.append(msg)
    print(msg, flush=True)


class Builder:
    def __init__(self, job):
        self.job = job
        self.k = float(job.get("scale_to_mm", 1.0))
        self.defs_src = job.get("defs", {})
        self.parts_src = job.get("parts", {})
        self.cache = {}

    # ------------------------------------------------------------ helpers
    def v(self, p):
        return V(*(float(c) * self.k for c in p))

    def length(self, x):
        return float(x) * self.k

    @staticmethod
    def unit(p):
        a = V(*(float(c) for c in p))
        if a.Length == 0:
            raise ValueError("zero-length direction")
        return a.normalize()

    def resolve(self, name):
        if name in self.cache:
            return self.cache[name]
        if name in self.parts_src:
            node = self.parts_src[name]
        elif name in self.defs_src:
            node = self.defs_src[name]
        else:
            raise KeyError(f"unknown shape name '{name}'")
        t0 = time.time()
        shp = self.build(node)
        self.cache[name] = shp
        log(f"  built {name}: {len(shp.Solids)} solid(s), {time.time() - t0:.2f} s")
        return shp

    def placement_to(self, shape, origin_mm, z_dir, x_dir=None):
        """Move a shape built around the global origin with its axis along +Z so that the origin goes to
        origin_mm and +Z points along z_dir (x_dir optional, sets the roll about z)."""
        z = V(z_dir).normalize()
        if x_dir is None:
            rot = FreeCAD.Rotation(V(0, 0, 1), z)
        else:
            x = V(x_dir)
            x = (x - z * x.dot(z)).normalize()
            y = z.cross(x)
            m = FreeCAD.Matrix(x.x, y.x, z.x, 0, x.y, y.y, z.y, 0, x.z, y.z, z.z, 0, 0, 0, 0, 1)
            rot = FreeCAD.Rotation(m)
        s = shape.copy()
        s.Placement = FreeCAD.Placement(origin_mm, rot).multiply(s.Placement)
        return s

    def apply_transforms(self, shp, node):
        for t in node.get("transform", []):
            s = shp.copy()
            if "rotate" in t:
                r = t["rotate"]
                c = self.v(r.get("center", [0, 0, 0]))
                s.rotate(c, V(*r["axis"]), float(r["deg"]))
            elif "translate" in t:
                s.translate(self.v(t["translate"]))
            else:
                raise ValueError(f"unknown transform {t}")
            shp = s
        return shp

    # ------------------------------------------------------------ sections
    def wire_from_points(self, sec):
        pts = [self.v(p) for p in sec["points"]]
        kind = sec.get("kind", "spline_line")
        if kind == "spline_line":          # open interpolating spline through all points, closed by a straight line
            bs = Part.BSplineCurve()
            bs.interpolate(pts)
            return Part.Wire([bs.toShape(), Part.LineSegment(pts[-1], pts[0]).toShape()])
        if kind == "closed_spline":        # periodic spline (first point must not be repeated)
            bs = Part.BSplineCurve()
            bs.interpolate(pts, PeriodicFlag=True)
            return Part.Wire([bs.toShape()])
        if kind == "polyline":             # closed polygon
            return Part.makePolygon(pts + [pts[0]])
        raise ValueError(f"unknown section kind {kind}")

    def rrect_wire(self, sec):
        """Rounded rectangle w x h, corner radius r, centred on origin; local x = xdir, local normal = normal."""
        w, h = self.length(sec["w"]) / 2, self.length(sec["h"]) / 2
        r = min(self.length(sec["r"]), 0.49 * 2 * min(w, h))
        edges = []
        # start mid-right, counter-clockwise: 4 lines + 4 arcs (same edge count for every section)
        corners = [(w - r, h - r, 0), (-w + r, h - r, 90), (-w + r, -h + r, 180), (w - r, -h + r, 270)]
        pts_line = [((w, -h + r), (w, h - r)), ((w - r, h), (-w + r, h)), ((-w, h - r), (-w, -h + r)),
                    ((-w + r, -h), (w - r, -h))]
        for (a, b), (cx, cy, ang) in zip(pts_line, corners):
            edges.append(Part.LineSegment(V(a[0], a[1], 0), V(b[0], b[1], 0)).toShape())
            if r > 0:
                circ = Part.Circle(V(cx, cy, 0), V(0, 0, 1), r)
                edges.append(Part.ArcOfCircle(circ, math.radians(ang), math.radians(ang + 90)).toShape())
        wire = Part.Wire(Part.__sortEdges__(edges))
        return self.placement_to(wire, self.v(sec["origin"]), self.unit(sec["normal"]), self.unit(sec["xdir"]))

    # ------------------------------------------------------------ node builder
    def build(self, node):
        op = node["op"]
        k = self.k
        if op == "ref":
            shp = self.resolve(node["name"]).copy()
        elif op == "box":
            sx, sy, sz = (self.length(c) for c in node["size"])
            c = self.v(node.get("center", [0, 0, 0]))
            shp = Part.makeBox(sx, sy, sz, c - V(sx / 2, sy / 2, sz / 2))
            if node.get("fillet"):
                ax = self.unit(node.get("fillet_edges_parallel", [1, 0, 0]))
                edges = [e for e in shp.Edges
                         if abs(abs((e.Vertexes[1].Point - e.Vertexes[0].Point).normalize().dot(ax)) - 1) < 1e-9]
                shp = shp.makeFillet(self.length(node["fillet"]), edges)
        elif op == "cylinder":
            r = self.length(node["r"])
            if "p0" in node:                # end points
                p0, p1 = self.v(node["p0"]), self.v(node["p1"])
                shp = Part.makeCylinder(r, (p1 - p0).Length, p0, (p1 - p0).normalize())
            else:                           # centred on "center" along "axis" (default z)
                h = self.length(node["h"])
                ax = self.unit(node.get("axis", [0, 0, 1]))
                shp = Part.makeCylinder(r, h, self.v(node.get("center", [0, 0, 0])) - ax * (h / 2), ax)
        elif op == "sphere":
            shp = Part.makeSphere(self.length(node["r"]), self.v(node.get("center", [0, 0, 0])))
        elif op == "cone":                  # r1 at the start, r2 at the end, centred on "center" along "axis"
            h = self.length(node["h"])
            ax = self.unit(node.get("axis", [0, 0, 1]))
            shp = Part.makeCone(self.length(node["r1"]), self.length(node["r2"]), h,
                                self.v(node.get("center", [0, 0, 0])) - ax * (h / 2), ax)
        elif op == "loft":
            wires = [self.wire_from_points(s) for s in node["sections"]]
            shp = Part.makeLoft(wires, True, bool(node.get("ruled", False)), bool(node.get("closed", False)),
                                int(node.get("max_degree", 8)))
        elif op == "rrect_loft":
            wires = [self.rrect_wire(s) for s in node["sections"]]
            shp = Part.makeLoft(wires, True, bool(node.get("ruled", False)), False, int(node.get("max_degree", 8)))
        elif op in ("fuse", "cut", "common"):
            if op == "cut":
                base = self.build(node["base"])
                tools = [self.build(t) for t in node["tools"]]
            else:
                shapes = [self.build(t) for t in node["shapes"]]
                base, tools = shapes[0], shapes[1:]
            shp = getattr(base, op)(tools) if tools else base
            if op == "fuse" and node.get("refine", True):
                shp = shp.removeSplitter()
            shp = solids_only(shp)
        elif op == "split_by_plane":
            base = self.build(node["shape"])
            n = self.unit(node["normal"])
            p = self.v(node["point"])
            L = 4 * base.BoundBox.DiagonalLength + 4 * (p - base.BoundBox.Center).Length
            half = Part.makeBox(L, L, L, V(-L / 2, -L / 2, 0))   # z >= 0 half
            keep = node.get("keep", "positive")
            half = self.placement_to(half, p, n if keep == "positive" else -n)
            shp = solids_only(base.common(half))
        elif op == "compound":
            shp = Part.Compound([self.build(t) for t in node["shapes"]])
        else:
            raise ValueError(f"unknown op {op}")
        shp = self.apply_transforms(shp, node)
        return shp

    def shape(self, ref):
        """A check/output reference: a name (part or def) or an inline node."""
        return self.resolve(ref) if isinstance(ref, str) else self.build(ref)


def solids_only(shp):
    sol = shp.Solids
    if len(sol) == 1:
        return sol[0]
    return Part.Compound(sol) if sol else shp


# ---------------------------------------------------------------- properties and checks
# GOTCHA (found 2026-11-14): OpenCASCADE 7.8.1's exact (BRep) volume integration, i.e. Shape.Volume,
# CenterOfMass and MatrixOfInertia, comes out ~12% LOW on solids bounded by an interpolated airfoil
# B-spline (e.g. a 0.0832 m2 x 1 m extrusion reports 0.0727 m3), while circles and boxes are exact. A fine
# tessellation is right to ~1e-3 (0.1 mm / 0.05 rad; finer meshes get slow fast: a wing centre section
# takes 1 s at 0.4 mm, 16 s at 0.1 mm and minutes at 0.02 mm). So all volumes, centroids and inertias here are integrated over a mesh
# (divergence theorem over signed tetrahedra); the BRep values are reported next to them for reference.
PROPS_TOL_MM = 0.1         # linear deflection for the property mesh, mm (overridable per job)
PROPS_ANG = 0.05           # angular deflection, rad
_MOMENTS = {}
CANON = np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]]) / 120.0


def mesh_moments(shp, tol_mm=None):
    """(volume, first moment vector, second-moment matrix about the origin) of a shape, summed over its
    solids, from a fine mesh. Units: mm."""
    tol_mm = tol_mm or PROPS_TOL_MM
    key = (shp.hashCode(), round(tol_mm, 9))
    if key in _MOMENTS:
        return _MOMENTS[key]
    vol, first, second = 0.0, np.zeros(3), np.zeros((3, 3))
    for s in (shp.Solids or [shp]):
        mesh = MeshPart.meshFromShape(Shape=s, LinearDeflection=tol_mm, AngularDeflection=PROPS_ANG, Relative=False)
        pts, fac = mesh.Topology
        if not fac:
            continue
        P = np.array([[p.x, p.y, p.z] for p in pts])
        T = P[np.array(fac)]                                  # (n, 3, 3): tetra with the origin as 4th vertex
        det = np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2]))
        v = det.sum() / 6.0
        sign = 1.0 if v >= 0 else -1.0                        # outward or inward mesh: same answer
        f = (det[:, None] * T.sum(axis=1)).sum(axis=0) / 24.0
        C = np.einsum("n,nki,kl,nlj->ij", det, T, CANON, T)
        vol += sign * v
        first += sign * f
        second += sign * C
    _MOMENTS[key] = (vol, first, second)
    return vol, first, second


def mesh_volume(shp, tol_mm=None):
    if not shp.Solids:
        return 0.0
    return mesh_moments(shp, tol_mm)[0]


def mass_props(shp, k, tol_mm=None):
    vol, first, second = mesh_moments(shp, tol_mm)
    if vol <= 0:
        log(f"  WARNING: mesh volume {vol} (meshing failed?); BRep volume {shp.Volume}")
        return dict(volume=0.0, brep_volume=shp.Volume / k**3)
    com = first / vol
    cov = second - vol * np.outer(com, com)                  # second moment about the centre of mass
    inertia = np.trace(cov) * np.eye(3) - cov                 # unit density
    bb = shp.BoundBox
    sols = shp.Solids or [shp]
    bvol = sum(s.Volume for s in sols)
    bcom = V(0, 0, 0)
    for s in sols:
        bcom += s.CenterOfMass * s.Volume
    bcom = bcom * (1.0 / bvol) if bvol else bcom
    return dict(volume=vol / k**3, com=(com / k).tolist(),
                bbox=[[bb.XMin / k, bb.YMin / k, bb.ZMin / k], [bb.XMax / k, bb.YMax / k, bb.ZMax / k]],
                inertia_unit_density=(inertia / k**5).tolist(), n_solids=len(sols),
                brep_volume=bvol / k**3, brep_com=[bcom.x / k, bcom.y / k, bcom.z / k])


def skin_of(shp, exclude_flat_normal=None):
    """Compound of the outer faces of a solid (optionally leaving out flat faces normal to an axis)."""
    faces = shp.Faces
    if exclude_flat_normal is not None:
        ax = V(*exclude_flat_normal).normalize()
        keep = []
        for f in faces:
            if f.Surface.__class__.__name__ == "Plane" and abs(abs(f.normalAt(0, 0).dot(ax)) - 1) < 1e-6:
                continue
            keep.append(f)
        faces = keep
    return Part.Compound(faces)


def run_check(b, chk):
    k = b.k
    typ = chk["type"]
    if typ == "inside":
        part = b.shape(chk["part"])
        parent = b.shape(chk["parent"])
        sols = part.Solids or [part]
        shares = []
        for s in sols:
            vs = mesh_volume(s)
            shares.append(mesh_volume(s.common(parent)) / vs if vs > 0 else float("nan"))
        tot = mesh_volume(part.common(parent)) / mesh_volume(part)
        return dict(share_inside_min_solid=min(shares), share_inside_total=tot)
    if typ == "distance":
        a = b.shape(chk["a"])
        tgt = b.shape(chk["b"])
        mode = chk.get("mode", "shape")
        if mode == "shell":                 # distance to the outer skin (faces), valid from inside a solid
            tgt = skin_of(tgt, chk.get("exclude_flat_normal"))
        d, pairs, _ = a.distToShape(tgt)
        pa, pb = pairs[0]
        return dict(distance=d / k, point_a=[c / k for c in pa], point_b=[c / k for c in pb])
    if typ == "interference":
        names = chk["parts"]
        allow = {frozenset(p) for p in chk.get("allow", [])}
        tol = float(chk.get("min_volume", 1e-9)) * k**3
        shapes = {n: b.shape(n) for n in names}
        boxes = {n: s.BoundBox for n, s in shapes.items()}
        clashes, tested = [], 0
        for i, a in enumerate(names):
            for c in names[i + 1:]:
                if frozenset((a, c)) in allow or not boxes[a].intersect(boxes[c]):
                    continue
                tested += 1
                v = mesh_volume(shapes[a].common(shapes[c]))
                if v > tol:
                    clashes.append(dict(a=a, b=c, volume=v / k**3))
        return dict(pairs_tested=tested, clashes=clashes)
    raise ValueError(f"unknown check type {typ}")


# ---------------------------------------------------------------- exports
def export_stl(shp, path, k, tol, ang):
    mesh = MeshPart.meshFromShape(Shape=shp, LinearDeflection=tol * k, AngularDeflection=ang, Relative=False)
    m = FreeCAD.Matrix()
    m.scale(1.0 / k, 1.0 / k, 1.0 / k)
    mesh.transform(m)
    mesh.write(path)


def export_step(named, path):
    import Import
    doc = FreeCAD.newDocument("fcad_asm")
    objs = []
    for name, shp in named:
        o = doc.addObject("Part::Feature", name)
        o.Label = name
        o.Shape = shp
        objs.append(o)
    doc.recompute()
    Import.export(objs, path)
    FreeCAD.closeDocument(doc.Name)


def main():
    job_path = os.environ.get("FCAD_JOB")
    if not job_path:
        raise SystemExit("FCAD_JOB is not set")
    with open(job_path, encoding="utf-8") as f:
        job = json.load(f)
    res_path = job["result"]
    result = dict(ok=False, freecad_version=".".join(FreeCAD.Version()[:3]), parts={}, checks={}, files={})
    t0 = time.time()
    try:
        b = Builder(job)
        log(f"fcad: building {len(b.parts_src)} part(s)")
        for name in b.parts_src:
            b.resolve(name)
        out = job.get("outputs", {})
        ptol = out.get("props_mesh_tolerance")            # job units; default PROPS_TOL_MM
        ptol = ptol * b.k if ptol else None
        if out.get("props", True):
            for name in b.parts_src:
                t1 = time.time()
                result["parts"][name] = mass_props(b.cache[name], b.k, ptol)
                log(f"  props {name}: {time.time() - t1:.2f} s")
        for name in out.get("props_extra", []):          # props for defs too, if asked
            result["parts"][name] = mass_props(b.resolve(name), b.k, ptol)
        for chk in job.get("checks", []):
            t1 = time.time()
            result["checks"][chk["id"]] = run_check(b, chk)
            log(f"  check {chk['id']}: {time.time() - t1:.2f} s")
        if out.get("stl_dir"):
            os.makedirs(out["stl_dir"], exist_ok=True)
            names = out.get("stl_parts", list(b.parts_src)) + out.get("stl_extra", [])
            for name in names:
                p = os.path.join(out["stl_dir"], f"{name}.stl")
                export_stl(b.resolve(name), p, b.k, float(out.get("stl_tolerance", 0.0004)),
                           float(out.get("stl_angular", 0.15)))
                result["files"][f"stl:{name}"] = p
        for step in out.get("step", []):                  # [{"path":..., "parts":[...]} ...]
            names = step.get("parts", list(b.parts_src))
            export_step([(n, b.resolve(n)) for n in names], step["path"])
            result["files"][f"step:{os.path.basename(step['path'])}"] = step["path"]
        result["ok"] = True
    except Exception:
        result["error"] = traceback.format_exc()
        log(result["error"])
    result["seconds"] = time.time() - t0
    result["log"] = LOG
    with open(res_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
    sys.stdout.flush()
    os._exit(0 if result["ok"] else 3)


main()
