"""Hidden-line projection of a STEP assembly, run inside FreeCAD (FreeCADCmd), for drawings.

Reads the job from the JSON file named in FCAD_JOB:
  {"step": path, "out": path, "deflection_mm": 0.05, "parts": [label, ...] (optional: only these solids),
   "views": {name: [dx, dy, dz], ...}}           # TechDraw convention: from the model towards the eye
Writes {"views": {name: {"visible": [[[u, v], ...], ...], "up": [ux, uy]}}} to `out`, in mm.

OpenCASCADE's HLR (TechDraw.projectEx) returns visible sharp, smooth, seam and outline edges; the sheet uses
sharp + outline, plus smooth edges so tangent breaks (wing panel joints, pod fairings) still show. "up" is where
the model +z axis points on the projected plane, so callers can turn an oblique view upright.
"""
import json
import os
import traceback


def main():
    job = json.load(open(os.environ["FCAD_JOB"]))
    result = {"views": {}}
    try:
        import FreeCAD
        import Part
        import Import
        import TechDraw

        doc = FreeCAD.newDocument("proj")
        Import.insert(job["step"], doc.Name)
        keep = set(job.get("parts") or [])
        solids = [o.Shape for o in doc.Objects if o.TypeId == "Part::Feature" and (not keep or o.Label in keep)]
        missing = keep - {o.Label for o in doc.Objects}
        if missing:
            result["missing_parts"] = sorted(missing)
        shape = Part.Compound(solids)
        defl = float(job.get("deflection_mm", 0.05))
        for name, d in job["views"].items():
            direction = FreeCAD.Vector(*d)
            parts = TechDraw.projectEx(shape, direction)
            lines = []
            for comp in (parts[0], parts[1], parts[3]):          # visible sharp, smooth, outline
                if comp.isNull():
                    continue
                for e in comp.Edges:
                    pts = e.discretize(QuasiDeflection=defl)
                    if len(pts) >= 2:
                        lines.append([[round(p.x, 3), round(p.y, 3)] for p in pts])
            ends = []
            for z in (0.0, 1000.0):                                  # small cubes at z = 0 and z = 1000
                cube = Part.makeBox(1, 1, 1, FreeCAD.Vector(-0.5, -0.5, z - 0.5))
                bb = Part.Compound([c for c in TechDraw.projectEx(cube, direction)[:4] if not c.isNull()]).BoundBox
                ends.append(((bb.XMin + bb.XMax) / 2, (bb.YMin + bb.YMax) / 2))
            up = [ends[1][0] - ends[0][0], ends[1][1] - ends[0][1]]
            result["views"][name] = {"visible": lines, "up": up}
    except Exception:
        result["error"] = traceback.format_exc()
    with open(job["out"], "w") as f:
        json.dump(result, f)


main()
os._exit(0)
