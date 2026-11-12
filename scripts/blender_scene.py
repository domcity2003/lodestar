"""Blender scene for Rescue Wing preview renders. Run by render_preview.py, not directly:
    blender -b -P scripts/blender_scene.py -- <stl> <out_dir> <x_cg> <x_kit> <x_batt>

OpenVSP axes: x aft, y right (span), z up. Blender keeps the same axes here.
"""
import math
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
STL, OUT, X_CG, X_KIT, X_BATT = argv[0], argv[1], float(argv[2]), float(argv[3]), float(argv[4])

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = 64
scene.cycles.use_denoising = True
scene.render.film_transparent = False
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"

world = bpy.data.worlds.new("World")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.93, 0.94, 0.95, 1)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.8


def material(name, color, rough=0.45, alpha=1.0, metallic=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Alpha"].default_value = alpha
    if alpha < 1:
        m.surface_render_method = "BLENDED" if hasattr(m, "surface_render_method") else None
    return m


def hex2rgb(h):
    h = h.lstrip("#")
    srgb = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb)


BODY = material("body", hex2rgb("#c9d2dc"), rough=0.28)
BODY_XRAY = material("body_xray", hex2rgb("#c9d3de"), rough=0.3, alpha=0.18)
KIT = material("kit", hex2rgb("#eb6834"), rough=0.5)
BATT = material("batt", hex2rgb("#2a78d6"), rough=0.5)
CG = material("cg", hex2rgb("#1f1f1e"), rough=0.4)
PROP = material("prop", hex2rgb("#2b2f36"), rough=0.4, alpha=0.12)
FLOOR = material("floor", hex2rgb("#e6e8eb"), rough=0.9)
LENS = material("lens", hex2rgb("#1b1e24"), rough=0.1, metallic=0.3)

bpy.ops.wm.stl_import(filepath=STL)
plane = bpy.context.selected_objects[0]
plane.name = "RescueWing"
bpy.ops.object.shade_smooth()
plane.data.materials.append(BODY)

# Pusher propeller disc (visual only): 13" prop behind the pod tail
bpy.ops.mesh.primitive_cylinder_add(radius=0.165, depth=0.004, location=(0.665, 0, -0.035),
                                    rotation=(0, math.pi / 2, 0), vertices=64)
prop = bpy.context.object
prop.data.materials.append(PROP)
# One two-bladed prop through the hub, plus a spinner
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.668, 0, -0.035))
bl = bpy.context.object
bl.scale = (0.008, 0.32, 0.022)
bl.rotation_euler = (math.radians(60), 0, 0)
bl.data.materials.append(CG)
bpy.ops.mesh.primitive_cone_add(radius1=0.022, depth=0.05, location=(0.69, 0, -0.035), rotation=(0, math.pi / 2, 0))
bpy.context.object.data.materials.append(CG)
# Gimbal lens
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.018, location=(0.035, 0, -0.135))
bpy.context.object.data.materials.append(LENS)

# Internal layout boxes (hidden in the hero shot)
inner = []
bpy.ops.mesh.primitive_cube_add(size=1, location=(X_KIT, 0, -0.035))
k = bpy.context.object
k.scale = (0.22, 0.13, 0.12)
k.data.materials.append(KIT)
inner.append(k)
for s in (1, -1):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(X_BATT, s * 0.355, 0.004))
    b = bpy.context.object
    b.scale = (0.105, 0.21, 0.021)
    b.data.materials.append(BATT)
    inner.append(b)
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.02, location=(X_CG, 0, 0.07))
c = bpy.context.object
c.data.materials.append(CG)
inner.append(c)

# Floor with soft shadow
bpy.ops.mesh.primitive_plane_add(size=400, location=(0, 0, -0.45))
bpy.context.object.data.materials.append(FLOOR)
floor = bpy.context.object

# Lights
bpy.ops.object.light_add(type="SUN", location=(0, 0, 5))
sun = bpy.context.object
sun.data.energy = 3.0
sun.data.angle = math.radians(8)
sun.rotation_euler = (math.radians(35), math.radians(-20), math.radians(30))
bpy.ops.object.light_add(type="AREA", location=(-2.5, -2.0, 2.5))
area = bpy.context.object
area.data.energy = 400
area.data.size = 3
area.rotation_euler = (math.radians(50), 0, math.radians(-50))

# Camera aimed at the model centre
target_loc = Vector((0.5, 0, -0.02))
bpy.ops.object.empty_add(location=target_loc)
target = bpy.context.object
bpy.ops.object.camera_add()
cam = bpy.context.object
scene.camera = cam
con = cam.constraints.new("TRACK_TO")
con.target = target
con.track_axis = "TRACK_NEGATIVE_Z"
con.up_axis = "UP_Y"


def render(name, cam_loc, lens=50, xray=False, res=(1800, 1100), show_floor=True):
    cam.location = cam_loc
    cam.data.lens = lens
    plane.data.materials[0] = BODY_XRAY if xray else BODY
    for o in inner:
        o.hide_render = not xray
    floor.hide_render = not show_floor
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.filepath = f"{OUT}/{name}.png"
    bpy.ops.render.render(write_still=True)


render("hero", (-3.9, -4.6, 2.6), lens=50)
render("hero_low", (-4.6, 2.6, 0.45), lens=50)
render("xray_top", (0.5, 0.0001, 7.0), lens=50, xray=True, res=(1800, 1100), show_floor=False)
render("xray_iso", (3.0, -3.3, 2.6), lens=50, xray=True)
