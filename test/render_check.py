"""
Run inside mayapy: builds a rig, poses it, frames a camera, and renders a
still image (software renderer, no GL context needed) for visual
verification of mesh placement/orientation.
Usage: mayapy render_check.py <urdf_path> <out_png>
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts', 'urdf_import'))

import math

import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as cmds
import maya.mel as mel


def _aim_camera(cam, eye, target):
    """
    Point a camera (looks down its local -Z, +Y up) from eye at target.
    Uses Maya's own aimConstraint rather than hand-rolled Euler math --
    getting the yaw/pitch signs right by hand for a given rotateOrder is
    easy to typo and produces a silently blank frame.
    """
    cmds.setAttr(cam + '.translate', *eye)
    loc = cmds.spaceLocator(name='aimTarget')[0]
    cmds.setAttr(loc + '.translate', *target)
    con = cmds.aimConstraint(loc, cam,
                              aimVector=(0, 0, -1),
                              upVector=(0, 1, 0),
                              worldUpType='vector',
                              worldUpVector=(0, 1, 0))
    cmds.delete(con)   # bake the resulting rotation, drop the constraint
    cmds.delete(loc)

PLUGIN_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'mimic', 'plug-ins', 'robotIKGeneric.py'))


def main(urdf_path, out_png):
    cmds.file(new=True, force=True)
    cmds.loadPlugin(PLUGIN_PATH)

    import rig_builder
    result = rig_builder.build_rig(urdf_path)

    for i, fk_node in enumerate(result['fk_nodes']):
        cmds.setAttr(fk_node + '.rotateZ', [20, -30, 40, 25, 10, 0][i % 6])

    cmds.setAttr(result['target_ctrl'] + '.ik', False)
    cmds.refresh()

    bbox = cmds.exactWorldBoundingBox(result['top_node'])
    center = [(bbox[0] + bbox[3]) / 2, (bbox[1] + bbox[4]) / 2, (bbox[2] + bbox[5]) / 2]
    diag = max(bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2], 1.0)

    cam, cam_shape = cmds.camera()
    cmds.setAttr(cam_shape + '.renderable', True)
    cmds.setAttr(cam_shape + '.focalLength', 50)
    az = math.radians(float(os.environ.get('CAM_AZIM', '35')))
    el = math.radians(float(os.environ.get('CAM_ELEV', '15')))
    r = diag * 2.4
    eye = [center[0] + r * math.cos(el) * math.sin(az),
           center[1] + r * math.sin(el),
           center[2] + r * math.cos(el) * math.cos(az)]
    _aim_camera(cam, eye, center)
    print('camera eye:', eye, 'center:', center, 'diag:', diag)

    cmds.setAttr('defaultRenderGlobals.currentRenderer', 'mayaSoftware', type='string')
    cmds.setAttr('defaultRenderGlobals.imageFormat', 32)  # PNG
    cmds.setAttr('defaultResolution.width', 800)
    cmds.setAttr('defaultResolution.height', 600)

    out_dir = os.path.dirname(out_png)
    out_name = os.path.splitext(os.path.basename(out_png))[0]
    cmds.workspace(out_dir, o=True)
    cmds.setAttr('defaultRenderGlobals.imageFilePrefix', out_name, type='string')

    rendered = cmds.render(cam_shape, x=800, y=600)
    print('Rendered to (reported):', rendered)

    # mayaSoftware writes into workspace's images/ dir by convention; find it.
    candidates = []
    for root, _dirs, files in os.walk(out_dir):
        for f in files:
            if f.startswith(out_name) and (f.endswith('.png') or f.endswith('.iff')):
                candidates.append(os.path.join(root, f))
    print('Found render outputs:', candidates)
    if candidates:
        import shutil
        shutil.copy(candidates[0], out_png)
        print('Copied to', out_png)
    else:
        raise SystemExit('No rendered image found')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
