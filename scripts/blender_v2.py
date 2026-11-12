"""Blender scene for the milestone 5 renders of Lodestar v2. Run by render_v2.py, not directly:
    blender -b -P scripts/blender_v2.py -- <stl_dir> <out_dir> <samples> [shot ...]

Imports every CAD part (out/cad/stl/*.stl), gives it a material by name, and renders:
  hero      studio 3/4 view, search-and-rescue livery
  inflight  banking over a sky, seen from below
  cutaway   skins made see-through to show the internal layout
  exploded  three-piece wing pulled apart, internals lifted out
  kitdrop   bay door open, kit falling under its cross parachute
Axes as in the CAD: x aft, y right, z up (metres).
"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
STL_DIR, OUT, SAMPLES = Path(argv[0]), argv[1], int(argv[2])
SHOTS = argv[3:] or ["hero", "inflight", "cutaway", "exploded", "kitdrop"]

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = SAMPLES
scene.cycles.use_denoising = True
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"
scene.render.resolution_x, scene.render.resolution_y = 1800, 1100


def lin(h):
    h = h.lstrip("#")
    s = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in s)


def mat(name, hexc, rough=0.4, metal=0.0, alpha=1.0, coat=0.0, transmission=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*lin(hexc), 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Alpha"].default_value = alpha
    if "Coat Weight" in b.inputs:
        b.inputs["Coat Weight"].default_value = coat
    if transmission and "Transmission Weight" in b.inputs:
        b.inputs["Transmission Weight"].default_value = transmission
    return m


M = dict(
    white=mat("white", "#eef0f2", 0.32, coat=0.4),
    orange=mat("orange", "#eb5a1e", 0.35, coat=0.4),
    grey=mat("grey", "#9aa3ad", 0.4, coat=0.2),
    carbon=mat("carbon", "#1d1f22", 0.25, coat=0.6),
    cell=mat("cell", "#2a78d6", 0.35),
    kit=mat("kit", "#d8342a", 0.6),
    chute=mat("chute", "#f2c230", 0.7),
    pcb=mat("pcb", "#1f7a4d", 0.5),
    dark=mat("dark", "#2b2f36", 0.35, metal=0.4),
    lens=mat("lens", "#0d1016", 0.05, metal=0.6),
    alu=mat("alu", "#b9bfc6", 0.3, metal=1.0),
)
GHOST = mat("ghost", "#c9d6e4", 0.15, alpha=0.28)
GHOST_ORANGE = mat("ghost_orange", "#eb5a1e", 0.15, alpha=0.35)


def material_for(name):
    if name.startswith(("winglet", "bay_door")):
        return M["orange"]
    if name.startswith("elevon_servo") or name in ("release_servo", "esc", "motor"):
        return M["dark"]
    if name.startswith("elevon"):
        return M["grey"]
    if name.startswith(("wing", "pod")):
        return M["white"]
    if name.startswith(("battery", "alt_bay_battery")):
        return M["cell"]
    if name.startswith(("spar", "propeller", "belly_skid", "gps_mast", "pitot")):
        return M["carbon"]
    if name == "rescue_kit":
        return M["kit"]
    if name in ("kit_parachute",):
        return M["chute"]
    if name == "kit_container":
        return M["grey"]
    if name in ("flight_controller", "gps"):
        return M["pcb"]
    if name == "gimbal_lens":
        return M["lens"]
    if name.startswith(("gimbal", "spinner")):
        return M["dark"]
    return M["grey"]


SKINS = ("wing_centre", "wing_outer_R", "wing_outer_L", "elevon_R", "elevon_L", "winglet_R", "winglet_L", "pod",
         "bay_door")

root = bpy.data.objects.new("aircraft", None)
scene.collection.objects.link(root)
objs = {}
for f in sorted(STL_DIR.glob("*.stl")):
    bpy.ops.wm.stl_import(filepath=str(f))
    o = bpy.context.selected_objects[0]
    o.name = f.stem
    o.data.materials.append(material_for(f.stem))
    if f.stem.startswith(("wing", "pod", "winglet", "elevon", "gimbal_ball", "spinner", "bay_door")):
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(40))
    o.parent = root
    objs[f.stem] = o
objs["alt_bay_battery"].hide_render = True
HOME = {n: o.location.copy() for n, o in objs.items()}

# Floor (shadow catcher look), lights, camera
bpy.ops.mesh.primitive_plane_add(size=400, location=(0, 0, -0.5))
floor = bpy.context.object
floor.data.materials.append(mat("floor", "#e9ebee", 0.9))
bpy.ops.object.light_add(type="SUN", location=(0, 0, 5))
sun = bpy.context.object
sun.data.energy = 3.2
sun.data.angle = math.radians(6)
sun.rotation_euler = (math.radians(30), math.radians(-25), math.radians(40))
bpy.ops.object.light_add(type="AREA", location=(-2.5, -2.5, 2.5))
fill = bpy.context.object
fill.data.energy = 350
fill.data.size = 4
fill.rotation_euler = (math.radians(50), 0, math.radians(-45))

target = bpy.data.objects.new("target", None)
scene.collection.objects.link(target)
bpy.ops.object.camera_add()
cam = bpy.context.object
scene.camera = cam
tc = cam.constraints.new("TRACK_TO")
tc.target = target
tc.track_axis = "TRACK_NEGATIVE_Z"
tc.up_axis = "UP_Y"

world = bpy.data.worlds.new("World")
scene.world = world
world.use_nodes = True
wn = world.node_tree.nodes
bg = wn["Background"]


def studio_world():
    for n in list(wn):
        if n.type == "TEX_SKY":
            wn.remove(n)
    bg.inputs[0].default_value = (0.92, 0.93, 0.95, 1)
    bg.inputs[1].default_value = 0.8
    floor.hide_render = False
    sun.data.energy = 3.2


def sky_world():
    sky = wn.new("ShaderNodeTexSky")
    try:
        sky.sky_type = "NISHITA"
        sky.sun_elevation = math.radians(35)
        sky.sun_rotation = math.radians(140)
    except Exception:
        pass
    world.node_tree.links.new(sky.outputs[0], bg.inputs[0])
    bg.inputs[1].default_value = 0.35
    floor.hide_render = True
    sun.data.energy = 4.0


def reset():
    root.rotation_euler = (0, 0, 0)
    root.location = (0, 0, 0)
    for n, o in objs.items():
        o.location = HOME[n].copy()
        o.rotation_euler = (0, 0, 0)
        o.hide_render = n == "alt_bay_battery"
        o.data.materials[0] = material_for(n)
    for o in extras:
        o.hide_render = True


def shoot(name, cam_loc, aim, lens=50):
    cam.location = cam_loc
    target.location = aim
    cam.data.lens = lens
    scene.render.filepath = f"{OUT}/{name}.png"
    bpy.ops.render.render(write_still=True)
    print(f"Saved {name}")


# Parachute for the kit-drop shot: cross canopy (two 1.5 x 0.5 m arms, slightly domed) + 4 lines
extras = []
CANOPY = mat("canopy", "#f2c230", 0.8)
_b = CANOPY.node_tree.nodes["Principled BSDF"]
_b.inputs["Emission Color"].default_value = (*lin("#f2c230"), 1)
_b.inputs["Emission Strength"].default_value = 0.5
if "Subsurface Weight" in _b.inputs:
    _b.inputs["Subsurface Weight"].default_value = 0.3
canopy_z, kit_z, kit_x = -1.0, -2.0, 0.65


def add(o, m):
    o.data.materials.append(m)
    extras.append(o)
    return o


for ang in (0, 90):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(kit_x, 0, canopy_z))
    arm = bpy.context.object
    arm.scale = (1.5, 0.5, 0.01)
    arm.rotation_euler = (0, 0, math.radians(ang + 20))
    bpy.ops.object.modifier_add(type="SUBSURF")
    add(arm, CANOPY)
for k in range(4):
    a = math.radians(20 + 90 * k)
    tip = Vector((kit_x + 0.7 * math.cos(a), 0.7 * math.sin(a), canopy_z))
    end = Vector((kit_x, 0, kit_z + 0.04))
    mid, d = (tip + end) / 2, end - tip
    bpy.ops.mesh.primitive_cylinder_add(radius=0.003, depth=d.length, location=mid)
    ln = bpy.context.object
    ln.rotation_mode = "QUATERNION"
    ln.rotation_quaternion = d.to_track_quat("Z", "Y")
    add(ln, M["dark"])
reset()

if "hero" in SHOTS:
    studio_world()
    reset()
    shoot("v2_hero", (-3.6, -4.4, 2.3), (0.55, -0.3, -0.05), lens=64)

if "inflight" in SHOTS:
    sky_world()
    reset()
    root.rotation_euler = (math.radians(18), math.radians(-4), 0)   # right-wing-down bank, slight nose up
    root.location = (0, 0, 2.0)
    shoot("v2_inflight", (-3.0, 3.2, 0.9), (0.5, 0, 2.0), lens=45)
    studio_world()

if "cutaway" in SHOTS:
    reset()
    for n in SKINS:
        objs[n].data.materials[0] = GHOST_ORANGE if n.startswith(("winglet", "bay_door")) else GHOST
    objs["kit_container"].data.materials[0] = GHOST
    shoot("v2_cutaway", (-1.5, -1.75, 1.45), (0.40, 0, -0.03), lens=50)

if "exploded" in SHOTS:
    reset()
    moves = {
        "wing_outer_R": (0, 0.30, 0), "elevon_R": (0.06, 0.30, 0), "winglet_R": (0, 0.30, 0), "elevon_servo_R": (0, 0.30, 0.05),
        "wing_outer_L": (0, -0.30, 0), "elevon_L": (0.06, -0.30, 0), "winglet_L": (0, -0.30, 0), "elevon_servo_L": (0, -0.30, 0.05),
        "spar_joiner_R": (0, 0.12, 0.10), "spar_joiner_L": (0, -0.12, 0.10),
        "battery_R": (0, 0, 0.22), "battery_L": (0, 0, 0.22),
        "flight_controller": (0, 0, 0.30), "gps": (0, 0, 0.32), "gps_mast": (0, 0, 0.32), "esc": (0, 0, 0.25),
        "kit_container": (0, 0, -0.22), "rescue_kit": (0, 0, -0.42), "kit_parachute": (0, 0, -0.30),
        "release_servo": (0, 0, -0.22), "bay_door": (0, 0, -0.55),
        "gimbal_ball": (0, 0, -0.25), "gimbal_lens": (0, 0, -0.25), "gimbal_mount": (0, 0, -0.18),
        "motor": (0.15, 0, 0), "spinner": (0.30, 0, 0), "propeller": (0.24, 0, 0),
        "belly_skid": (0, 0, -0.15), "pitot": (-0.12, 0, 0),
    }
    for n, d in moves.items():
        objs[n].location = HOME[n] + Vector(d)
    floor.location.z = -1.2
    shoot("v2_exploded", (-2.4, -3.9, 2.6), (0.45, 0, -0.1), lens=42)
    floor.location.z = -0.5

if "kitdrop" in SHOTS:
    sky_world()
    reset()
    root.location = (0, 0, 0)
    door = objs["bay_door"]
    door.location = HOME["bay_door"] + Vector((0, 0.07, -0.04))
    door.rotation_euler = (math.radians(-70), 0, 0)
    for n in ("rescue_kit",):
        objs[n].location = HOME[n] + Vector((kit_x - 0.42, 0, kit_z + 0.062))
    objs["kit_parachute"].hide_render = True
    for o in extras:
        o.hide_render = False
    shoot("v2_kitdrop", (-2.6, -4.2, -2.6), (0.45, 0, -0.9), lens=40)
    studio_world()
