# fcad: headless FreeCAD from a normal Python venv

Small, project-neutral helper for building CAD models with FreeCAD's OpenCASCADE kernel when
build123d/CadQuery/OCP cannot load (on this machine their DLLs are blocked by Windows Application
Control; FreeCAD's own portable build is allowed). Written 2026-11-14 for Lodestar; copy the folder
into any other project.

| file | runs in | does |
|---|---|---|
| `runner.py` | your venv | `run(job, workdir)`: writes `job.json`, runs `FreeCADCmd.exe fc_build.py`, returns `result.json` |
| `fc_build.py` | FreeCAD's Python 3.11 | builds the shapes, exports STL/STEP, mass properties, checks |
| `render.py` | your venv | STL to PNG (front/side/top/iso), numpy + matplotlib only |
| `test_fcad.py` | your venv | 25 known-answer checks (`python scripts/fcad/test_fcad.py`) |

FreeCADCmd is found from the environment variable `FREECADCMD`, else on PATH, in Program Files, or in a portable
`FreeCAD_*` folder under a `.tools` or `aero-tools` directory next to the repository (`runner.find_freecad_bin`).
Tested with FreeCAD 1.1.4 portable for Windows, used read-only. `fea_wing.py` finds CalculiX (`ccx.exe`) the same
way, or from `CCX_EXE`.

## Usage

```python
from fcad.runner import run
job = {
  "scale_to_mm": 1000,                      # job lengths are metres
  "parts": {"plate": {"op": "box", "size": [0.2, 0.1, 0.01], "center": [0, 0, 0]}},
  "defs":  {},                              # helper shapes: referenced, not exported
  "checks": [],
  "outputs": {"stl_dir": "out/stl", "step": [{"path": "out/model.step"}]},
}
res = run(job, "out/fcad_work")             # raises FcadError with FreeCAD's log on failure
res["parts"]["plate"]["volume"]             # m3
```

## Job schema

Top level: `scale_to_mm` (job unit -> mm, default 1), `parts` (ordered dict name -> node, exported),
`defs` (name -> node, built on demand), `checks` (list), `outputs`.

Every node is `{"op": ..., ..., "transform": [...]}`; `transform` is optional and applied in order:
`{"rotate": {"axis": [x,y,z], "deg": a, "center": [x,y,z]}}` or `{"translate": [x,y,z]}`.

| op | fields |
|---|---|
| `box` | `size` [dx,dy,dz], `center` (box is centred); optional `fillet` radius on edges parallel to `fillet_edges_parallel` (default x) |
| `cylinder` | `r` and either `p0`,`p1` (end points) or `h`, `center`, `axis` (centred, default axis z) |
| `sphere` | `r`, `center` |
| `cone` | `r1` (start), `r2` (end), `h`, `center`, `axis` (centred) |
| `loft` | `sections`: list of `{"kind", "points": [[x,y,z],...]}`; `ruled` (bool), `closed` (bool), `max_degree` (default 8) |
| `rrect_loft` | `sections`: list of `{"origin", "normal", "xdir", "w", "h", "r"}` (rounded rectangle, w along xdir); `ruled`, `max_degree` (default 8) |
| `fuse` / `common` | `shapes`: list of nodes; fuse is refined (`removeSplitter`) unless `"refine": false` |
| `cut` | `base`: node, `tools`: list of nodes |
| `split_by_plane` | `shape`, `point`, `normal`, `keep`: `"positive"` (side the normal points to) or `"negative"` |
| `compound` | `shapes`: list of nodes (kept as separate solids) |
| `ref` | `name` of a part or def (built once, cached, copied) |

Section kinds: `spline_line` (open interpolating B-spline through all points, closed by a straight
line from the last point to the first: an airfoil with an open trailing edge), `closed_spline`
(periodic, do not repeat the first point), `polyline` (closed polygon).

Checks (each has an `id`; results land in `result["checks"][id]`):
- `{"type": "inside", "part", "parent"}` -> `share_inside_min_solid` (worst solid of a compound), `share_inside_total`.
- `{"type": "distance", "a", "b", "mode"}` -> `distance`, closest points. `mode: "shape"` (default) is
  solid to solid (0 if they touch or one is inside the other); `mode: "shell"` measures to `b`'s outer
  faces, which is what you want for "clearance to the skin" from inside. `exclude_flat_normal: [0,1,0]`
  leaves out flat faces normal to that axis (e.g. a wing panel's end faces).
- `{"type": "interference", "parts": [...], "allow": [[a,b],...], "min_volume"}` -> `pairs_tested`,
  `clashes` [{a, b, volume}]. Bounding boxes are compared first; allowed pairs are skipped.

`a`, `b`, `part`, `parent` may be names or inline nodes.

Outputs: `stl_dir`, `stl_parts` (default all parts), `stl_extra` (defs), `stl_tolerance` (job units,
default 0.0004), `stl_angular` (rad, 0.15); `step`: list of `{"path", "parts"}` (one named-solid
assembly per file, written in mm); `props` (default true), `props_extra` (defs), `props_mesh_tolerance`.

Result per part: `volume`, `com`, `bbox`, `inertia_unit_density` (3x3 about the centre of mass;
multiply by density), `n_solids`, and the kernel's own `brep_volume`, `brep_com` for comparison.

## Gotchas found (2026-11-14, FreeCAD 1.1.4 / OCCT 7.8.1)

1. **OpenCASCADE's exact volume integration is wrong on airfoil B-spline solids.** `Shape.Volume`,
   `CenterOfMass` and `MatrixOfInertia` come out 12-20% low on a loft or extrusion of an interpolated
   airfoil spline (test: a 0.0832 m2 section extruded 1 m reports 0.0727 m3); boxes, cylinders and a
   periodic spline circle are exact. The face *area* is nearly right, so it is the volume integral.
   The archived build123d results (same kernel family) carry this error: Lodestar v3's wing centroid was
   reported 45 mm too far aft. fcad therefore integrates volume, centroid and inertia over a fine mesh
   (default 0.1 mm chordal, 0.05 rad angular; ~1e-3 relative, results cached per shape) and also reports the BRep numbers. `inside` and
   `interference` volumes use the mesh too. Never trust `Shape.Volume` on spline geometry without a check.
2. **Arguments:** FreeCADCmd treats extra command-line arguments as files to open, so the job path is
   passed in the environment variable `FCAD_JOB`. The script's `__name__` is the file stem, not
   `"__main__"`, so `fc_build.py` runs `main()` unconditionally.
3. **Exit codes and output:** an uncaught exception in the script still exits 0, and buffered `print`
   output before it can be lost. `fc_build.py` catches everything, writes the traceback into the result
   JSON, flushes, and leaves with `os._exit(3)`; the runner checks both.
4. **Units:** OCC works in mm with a 1e-7 confusion tolerance; feed it mm (`scale_to_mm`). STEP is
   written in mm (carries its unit); STL is scaled back to job units.
5. **Near-zero trailing edges:** a spline section whose closing line is ~0.1 mm long lofts into an
   invalid solid (`isValid()` False, mesh volume 0). Open the trailing edge (Lodestar uses 0.3% chord).
6. **Rendering:** PyVista/VTK DLLs are blocked here too, so `render.py` is a numpy/matplotlib
   painter's-algorithm renderer. Meshing a sphere finely is slow (a 0.1 m sphere at 0.005 mm took 47 s),
   and the mesher scales badly: a Lodestar wing centre section takes 1 s at 0.4 mm, 16 s at 0.1 mm
   and several minutes at 0.02 mm. Keep `props_mesh_tolerance` near 0.1 mm.
7. **Smooth lofts depend on max degree.** `Part.makeLoft` defaults to max degree 5, OpenCASCADE's
   ThruSections (and build123d) to 8. A smooth loft bulges between sections differently: the Lodestar
   pod came out 4% larger at degree 5. fcad passes 8 by default so models ported from build123d match.
8. **Fused wing pieces:** `fuse` removes faces shared by touching solids, so the fused wing has no
   internal faces at the panel breaks; `removeSplitter()` then merges coplanar leftovers.
