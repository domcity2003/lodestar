"""fcad render: STL meshes to PNG views (front, side, top, isometric), in the normal venv.

Why not PyVista: VTK's DLLs are blocked by Windows Application Control on this machine (tested
2026-11-14), so this uses only numpy + matplotlib. It is an orthographic painter's-algorithm renderer:
every triangle is projected, flat-shaded with one directional light and drawn back to front as a 2-D
PolyCollection. That is exact for opaque, non-intersecting meshes and good enough to eyeball a layout;
translucent parts (alpha < 1) let you see internals through a skin.

    from fcad.render import Item, render_views
    items = [Item("out/stl/wing.stl", color="#c8d0d8", alpha=0.3), Item("out/stl/battery.stl", "#d9534f")]
    render_views(items, "views.png", views=("front", "side", "top", "iso"), title="...")

Axes convention used for the named views: x aft, y right (starboard), z up (the aero convention).
"""
from dataclasses import dataclass
from pathlib import Path
import struct

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from matplotlib.colors import to_rgba  # noqa: E402

# view name: (direction the viewer looks along, screen-up vector)
VIEWS = {
    "front": ((1.0, 0.0, 0.0), (0, 0, 1)),        # from ahead of the nose, looking aft
    "side": ((0.0, 1.0, 0.0), (0, 0, 1)),         # from the left, nose to the left
    "top": ((0.0, 0.0, -1.0), (-1, 0, 0)),        # from above, nose up the page
    "iso": ((1.0, -1.2, -0.9), (0, 0, 1)),        # from front-right, above
    "iso_rear": ((-1.0, -1.2, -0.7), (0, 0, 1)),  # from behind-right, above
}
LIGHT = np.array([0.4, -0.5, 0.75])


@dataclass
class Item:
    path: str
    color: str = "#b0b8c0"
    alpha: float = 1.0


def read_stl(path):
    """Triangles (n, 3, 3) from a binary or ASCII STL."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                                count=n, offset=84)
            return rec["v"].astype(float)
    verts = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="replace").splitlines()
             if ln.strip().startswith("vertex")]
    return np.array(verts, dtype=float).reshape(-1, 3, 3)


def _basis(view):
    d, up = VIEWS[view] if isinstance(view, str) else view
    d = np.asarray(d, float) / np.linalg.norm(d)
    up = np.asarray(up, float)
    up = up - d * up.dot(d)
    up /= np.linalg.norm(up)
    right = np.cross(d, up)
    return d, right, up


def draw(ax, meshes, view, crop=None):
    d, r, u = _basis(view)
    light = LIGHT / np.linalg.norm(LIGHT)
    polys, cols, depth = [], [], []
    for tri, rgba in meshes:
        if len(tri) == 0:
            continue
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nn = np.linalg.norm(n, axis=1)
        keep = nn > 0
        tri, n = tri[keep], n[keep] / nn[keep, None]
        shade = 0.35 + 0.65 * np.abs(n @ light)
        c = np.tile(np.asarray(rgba), (len(tri), 1))
        c[:, :3] *= shade[:, None]
        polys.append(np.stack([tri @ r, tri @ u], axis=2))
        cols.append(c)
        depth.append((tri @ d).mean(axis=1))
    P, C, D = np.concatenate(polys), np.concatenate(cols), np.concatenate(depth)
    order = np.argsort(-D)                       # far first
    P, C = P[order], C[order]
    ax.add_collection(PolyCollection(P, facecolors=C, edgecolors=C, linewidths=0.15))
    lo, hi = P.reshape(-1, 2).min(0), P.reshape(-1, 2).max(0)
    if crop is not None:
        lo, hi = np.array(crop[0], float), np.array(crop[1], float)
    pad = 0.03 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def render_views(items, out_png, views=("front", "side", "top", "iso"), title=None, crops=None, ncols=2,
                 panel_in=6.0, dpi=130, labels=None):
    """One PNG with one panel per view. crops: {view: ((x0, y0), (x1, y1))} in screen units (job units)."""
    meshes = [(read_stl(it.path), to_rgba(it.color, it.alpha)) for it in items]
    nrows = int(np.ceil(len(views) / ncols))
    fig, axs = plt.subplots(nrows, ncols, figsize=(panel_in * ncols, panel_in * 0.7 * nrows), squeeze=False)
    for ax, v in zip(axs.flat, views):
        draw(ax, meshes, v, (crops or {}).get(v))
        ax.set_title((labels or {}).get(v, v), fontsize=10)
    for ax in axs.flat[len(views):]:
        ax.axis("off")
    if title:
        fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=dpi)
    plt.close(fig)
    return out_png
