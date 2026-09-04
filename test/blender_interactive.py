"""
Interactive Blender session: builds a robot rig and wires a live IK handler
per limb so moving any target_CTRL in the viewport re-solves and drives
that limb in real time -- the Blender equivalent of dragging Mimic's
target_CTRL in Maya. Branched robots (humanoids, quadrupeds) get one
independent, live target per limb (see rig['chains']).

Run with the real Blender GUI (no --background):
    blender --python test/blender_interactive.py -- <urdf_or_mjcf_path> [robot_name]
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'mimic', 'scripts'))
sys.path.insert(0, os.path.join(HERE, '..', 'mimic', 'scripts', 'urdf_import'))

import bpy
import mathutils

import blender_rig_builder as brb

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
URDF = argv[0] if argv else os.path.join(
    HERE, '..', 'mimic', 'robot_descriptions', 'SO100', 'so100.urdf')
ROBOT_NAME = argv[1] if len(argv) > 1 else None


def _frame_rig():
    bpy.context.view_layer.update()
    objs = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    if not objs:
        return
    pts = []
    for o in objs:
        for v in o.bound_box:
            pts.append(o.matrix_world @ mathutils.Vector(v))
    xs = [p.x for p in pts]
    ys = [p.y for p in pts]
    zs = [p.z for p in pts]
    center = mathutils.Vector(((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2))
    diag = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.05)

    for area in bpy.context.screen.areas:
        if area.type != 'VIEW_3D':
            continue
        for space in area.spaces:
            if space.type != 'VIEW_3D':
                continue
            space.shading.type = 'SOLID'
            space.shading.color_type = 'MATERIAL'
            space.clip_end = max(space.clip_end, diag * 20)
            r3d = space.region_3d
            r3d.view_location = center
            r3d.view_distance = diag * 2.6
            r3d.view_rotation = mathutils.Euler((math.radians(65), 0, math.radians(35))).to_quaternion()

    light_data = bpy.data.lights.new('key_light', type='SUN')
    light_data.energy = 3.0
    light = bpy.data.objects.new('key_light', light_data)
    bpy.context.collection.objects.link(light)
    light.rotation_euler = (math.radians(55), 0, math.radians(35))


def main():
    # Clear Blender's default startup scene (Cube/Camera/Light) so it
    # doesn't sit in the viewport occluding the rig or get swept into the
    # rig-framing bounding box.
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for coll in ('Cube', 'Camera', 'Light'):
        obj = bpy.data.objects.get(coll)
        if obj is not None:
            bpy.data.objects.remove(obj, do_unlink=True)

    print('[urdf_import] Building rig from', URDF)
    rig = brb.build_rig(URDF, robot_name=ROBOT_NAME)
    print('[urdf_import] branched=%s pose joints=%d chains=%d' %
          (rig.get('branched'), rig['pose_joint_count'], len(rig['chains'])))
    for w in rig['warnings']:
        print('[urdf_import] warning:', w)
    for chain in rig['chains']:
        print('[urdf_import] chain -> tip=%s joints=%d target=%s' %
              (chain.get('tip_link'), len(chain['joint_specs']), chain['target'].name))

    _frame_rig()
    brb.install_live_ik(rig)

    bpy.ops.object.select_all(action='DESELECT')
    for chain in rig['chains']:
        chain['target'].select_set(True)
    bpy.context.view_layer.objects.active = rig['chains'][0]['target']

    print('[urdf_import] Ready. %d target_CTRL(s) selected -- move any (G) and its' % len(rig['chains']))
    print('[urdf_import] limb re-solves live as you drag.')


main()
