"""
Run inside Blender:
    blender --background --python test/blender_check.py -- <urdf> <out_json> [png]

Builds the rig, applies a known FK pose, dumps joint-frame world positions
and per-mesh world bounding boxes to JSON (for comparison against pybullet
ground truth by verify_blender_vs_pybullet.py), runs an IK round-trip, and
optionally renders a viewport preview.
"""
import json
import math
import os
import sys

import numpy as np

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
URDF, OUT_JSON = argv[0], argv[1]
OUT_PNG = argv[2] if len(argv) > 2 else None

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'mimic', 'scripts'))
sys.path.insert(0, os.path.join(HERE, '..', 'mimic', 'scripts', 'urdf_import'))

import bpy

import blender_rig_builder as brb
from robotmath import generic_ik as gik

TEST_POSE_DEG = [20.0, -35.0, 45.0, 15.0, -60.0]


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def main():
    clear_scene()
    rig = brb.build_rig(URDF)
    print('built: %d pose joints, %d tool joints' %
          (rig['pose_joint_count'], rig['tool_joint_count']))
    for w in rig['warnings']:
        print('  warning:', w)

    # --- FK pose ---
    brb.set_joint_angles(rig, [math.radians(d) for d in TEST_POSE_DEG[:len(rig['axis_objs'])]])
    bpy.context.view_layer.update()

    out = {'pose_deg': TEST_POSE_DEG[:len(rig['axis_objs'])], 'axis_world': [], 'meshes': []}
    for obj in rig['axis_objs']:
        out['axis_world'].append(list(obj.matrix_world.translation))

    for obj in bpy.context.scene.objects:
        if obj.type != 'MESH':
            continue
        pts = [obj.matrix_world @ v.co for v in obj.data.vertices]
        xs = [p.x for p in pts]; ys = [p.y for p in pts]; zs = [p.z for p in pts]
        out['meshes'].append({
            'name': obj.name,
            'centre': [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2],
            'size': [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)],
            'verts': len(obj.data.vertices),
        })

    # --- IK round-trip: move the goal, solve, confirm the TCP reaches it ---
    tcp_before = np.array(rig['tcp'].matrix_world.translation)
    rig['target'].matrix_world = rig['tcp'].matrix_world.copy()
    tgt = rig['target']
    tgt.location = (tgt.location.x + 0.02, tgt.location.y + 0.02, tgt.location.z + 0.01)
    bpy.context.view_layer.update()

    thetas, converged, pos_err, rot_err = brb.solve_to_target(rig)
    bpy.context.view_layer.update()
    tcp_after = np.array(rig['tcp'].matrix_world.translation)
    goal = np.array(tgt.matrix_world.translation)
    residual = float(np.linalg.norm(tcp_after - goal))

    out['ik'] = {
        'converged': bool(converged),
        'solver_pos_err_m': float(pos_err),
        'scene_residual_m': residual,
        'tcp_moved_m': float(np.linalg.norm(tcp_after - tcp_before)),
        'thetas_deg': [math.degrees(t) for t in thetas],
    }
    print('IK: converged=%s solver_err=%.3e m  scene residual=%.3e m (%.4f mm)' %
          (converged, pos_err, residual, residual * 1000))

    json.dump(out, open(OUT_JSON, 'w'), indent=1)
    print('wrote', OUT_JSON)

    if OUT_PNG:
        render_preview(rig, OUT_PNG)


def render_preview(rig, out_png):
    """Frame the rig and render with EEVEE (works headless)."""
    bpy.context.view_layer.update()
    pts = []
    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH':
            pts += [obj.matrix_world @ mathutils_vec(v.co) for v in obj.data.vertices]
    if not pts:
        return
    xs = [p.x for p in pts]; ys = [p.y for p in pts]; zs = [p.z for p in pts]
    centre = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, (min(zs) + max(zs)) / 2)
    diag = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))

    import mathutils
    cam_data = bpy.data.cameras.new('cam')
    cam = bpy.data.objects.new('cam', cam_data)
    bpy.context.collection.objects.link(cam)
    r = diag * 2.6
    az, el = math.radians(50), math.radians(20)
    cam.location = (centre[0] + r * math.cos(el) * math.sin(az),
                    centre[1] - r * math.cos(el) * math.cos(az),
                    centre[2] + r * math.sin(el))
    direction = mathutils.Vector(centre) - cam.location
    cam.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()
    bpy.context.scene.camera = cam

    light_data = bpy.data.lights.new('key', type='SUN')
    light_data.energy = 4.0
    light = bpy.data.objects.new('key', light_data)
    bpy.context.collection.objects.link(light)
    light.rotation_euler = (math.radians(50), 0, math.radians(40))

    scene = bpy.context.scene
    # Workbench: fast, deterministic, and needs no GPU/OptiX -- ideal headless.
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.render.resolution_x = 800
    scene.render.resolution_y = 600
    scene.render.filepath = out_png
    bpy.ops.render.render(write_still=True)
    print('rendered', out_png)


def mathutils_vec(v):
    return v


if __name__ == '__main__':
    main()
