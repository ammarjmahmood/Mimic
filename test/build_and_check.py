"""
Run inside mayapy: builds a rig from a URDF and does basic sanity checks.
Usage: mayapy build_and_check.py <urdf_path>
"""
import sys
import os
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts', 'urdf_import'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'plug-ins'))

import maya.standalone
maya.standalone.initialize(name='python')

import maya.cmds as cmds

PLUGIN_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'mimic', 'plug-ins', 'robotIKGeneric.py'))


def main(urdf_path):
    cmds.file(new=True, force=True)
    cmds.loadPlugin(PLUGIN_PATH)

    import rig_builder
    result = rig_builder.build_rig(urdf_path)
    print('BUILD OK:', {k: v for k, v in result.items() if k != 'axis_nodes'})
    for w in result['warnings']:
        print('  warning:', w)

    # Sanity: node exists, attribute wiring exists.
    ik_node = result['ik_node']
    assert cmds.objExists(ik_node), 'ik node missing'
    assert cmds.getAttr(ik_node + '.numJoints') == result['pose_joint_count']

    axis_nodes = result['axis_nodes']
    for i, axis_node in enumerate(axis_nodes):
        conns = cmds.listConnections(axis_node + '.rotateZ', s=True, d=False, plugs=True) or []
        assert conns, 'axis %s rotateZ not driven by IK node' % axis_node
        print('  %s.rotateZ <- %s' % (axis_node, conns[0]))

    # Drive FK to a nonzero pose and confirm axis nodes rotate (IK mode off).
    target_ctrl = result['target_ctrl']
    cmds.setAttr(target_ctrl + '.ik', False)
    for i, fk_node in enumerate(result['fk_nodes']):
        cmds.setAttr(fk_node + '.rotateZ', 15.0)
    cmds.refresh()
    for axis_node in axis_nodes:
        val = cmds.getAttr(axis_node + '.rotateZ')
        print('  FK check', axis_node, 'rotateZ =', val)
        assert abs(val - 15.0) < 1e-6, 'FK passthrough not working for %s (got %s)' % (axis_node, val)

    # Snap target_CTRL to the current (FK-posed) end effector position first,
    # so the IK target starts reachable, then nudge it a small, plausible
    # amount -- this mirrors how a real animator would enable IK (align the
    # handle to the arm's current pose, then drag it) rather than testing an
    # arbitrary/unreachable teleport.
    tcp_hdl = [n for n in cmds.ls(type='transform') if n.endswith('tcp_HDL')][0]
    tcp_t = cmds.xform(tcp_hdl, q=True, ws=True, t=True)
    tcp_r = cmds.xform(tcp_hdl, q=True, ws=True, ro=True)
    cmds.xform(target_ctrl, ws=True, t=tcp_t)
    cmds.xform(target_ctrl, ws=True, ro=tcp_r)

    cmds.setAttr(target_ctrl + '.ik', True)
    cmds.refresh()
    before_axis_rot = [cmds.getAttr(a + '.rotateZ') for a in axis_nodes]
    cur_t = cmds.xform(target_ctrl, q=True, ws=True, t=True)
    new_t = [cur_t[0] + 2.0, cur_t[1] + 2.0, cur_t[2] + 1.0]
    cmds.xform(target_ctrl, ws=True, t=new_t)
    cmds.refresh()
    after_axis_rot = [cmds.getAttr(a + '.rotateZ') for a in axis_nodes]
    print('  IK check: axis rotateZ before', before_axis_rot)
    print('  IK check: axis rotateZ after ', after_axis_rot)
    pos_err = cmds.getAttr(ik_node + '.ikPositionError')
    converged = cmds.getAttr(ik_node + '.ikConverged')
    print('  IK converged=%s posError(cm)=%s' % (converged, pos_err))

    # Geometric sanity: every mesh should have a finite, non-degenerate
    # bounding box, sit reasonably close to its axis pivot (catches
    # "mesh flew off into space" orientation/compensation bugs), and the
    # overall arm reach should be in the right ballpark for a desktop arm
    # (tens of cm, not micrometers or kilometers).
    all_meshes = cmds.ls(result['top_node'], dagObjects=True, long=True, type='mesh') or []
    assert all_meshes, 'no meshes found under rig'
    overall_bbox_min = [float('inf')] * 3
    overall_bbox_max = [float('-inf')] * 3
    for mesh in all_meshes:
        bbox = cmds.exactWorldBoundingBox(mesh)
        assert all(abs(v) < 1e5 for v in bbox), 'mesh %s has an absurd bounding box: %s' % (mesh, bbox)
        size = [bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2]]
        assert all(s > 1e-4 for s in size), 'mesh %s is degenerate: %s' % (mesh, bbox)

        # PER-PART size check. This is the assertion that matters: a units bug
        # that leaves every part 100x too small keeps the *assembly spread*
        # correct (joint origins are scaled independently of mesh vertices),
        # so an overall-bbox check alone silently passes while the rig renders
        # as a scatter of ~1mm specks. A real printed part on a desktop arm is
        # centimetres across, never sub-millimetre.
        longest = max(size)
        assert 0.5 < longest < 100.0, (
            'mesh %s longest dimension is %.4f cm -- outside the plausible range for a '
            'robot part; likely a units-conversion bug (meters vs cm)' % (mesh, longest))

        for k in range(3):
            overall_bbox_min[k] = min(overall_bbox_min[k], bbox[k])
            overall_bbox_max[k] = max(overall_bbox_max[k], bbox[k + 3])
    overall_size = [overall_bbox_max[k] - overall_bbox_min[k] for k in range(3)]
    print('  overall rig bounding box size (cm):', overall_size)
    assert all(1.0 < s < 200.0 for s in overall_size), \
        'overall rig size %s cm is outside the sane range for a desktop arm' % overall_size

    # The parts must also actually TOUCH each other. Correctly-scaled parts on
    # a real arm overlap/abut at the joints; if geometry were mis-scaled or
    # mis-placed, the union of the parts would be far smaller than the span
    # they're spread across (the "scattered specks" signature).
    total_part_volume_span = sum(max(cmds.exactWorldBoundingBox(m)[3] - cmds.exactWorldBoundingBox(m)[0],
                                      cmds.exactWorldBoundingBox(m)[4] - cmds.exactWorldBoundingBox(m)[1],
                                      cmds.exactWorldBoundingBox(m)[5] - cmds.exactWorldBoundingBox(m)[2])
                                  for m in all_meshes)
    assembly_span = max(overall_size)
    print('  sum of part sizes: %.2f cm vs assembly span: %.2f cm' %
          (total_part_volume_span, assembly_span))
    assert total_part_volume_span > assembly_span * 0.8, (
        'parts sum to only %.2f cm across an assembly spanning %.2f cm -- geometry is '
        'too small/sparse to form a connected arm' % (total_part_volume_span, assembly_span))

    print('ALL CHECKS PASSED for', urdf_path)


if __name__ == '__main__':
    try:
        main(sys.argv[1])
    except Exception:
        traceback.print_exc()
        sys.exit(1)
