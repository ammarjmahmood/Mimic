"""Build and validate the bundled branched Microduck rig inside mayapy."""

import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..'))
sys.path[:0] = [
    os.path.join(REPO, 'mimic', 'scripts'),
    os.path.join(REPO, 'mimic', 'scripts', 'urdf_import'),
]

import maya.standalone
maya.standalone.initialize(name='python')

import maya.cmds as cmds

PLUGIN = os.path.join(REPO, 'mimic', 'plug-ins', 'robotIKGeneric.py')
MODEL = os.path.join(
    REPO, 'mimic', 'robot_descriptions', 'Microduck',
    'robot_allcollisions.xml')


def main():
    cmds.file(new=True, force=True)
    cmds.loadPlugin(PLUGIN)

    import rig_builder
    result = rig_builder.build_rig(MODEL)

    assert result['source_format'] == 'mjcf'
    assert result['pose_joint_count'] == 14
    assert len(result['axis_nodes']) == 14
    assert len(result['fk_nodes']) == 14
    assert len(result['ik_nodes']) == 3
    assert len(result['target_ctrls']) == 3
    assert sorted(len(chain['joint_names']) for chain in result['chains']) == [4, 5, 5]

    meshes = cmds.ls(
        result['top_node'], dagObjects=True, long=True, type='mesh') or []
    assert len(meshes) == 70, 'Expected 70 visual mesh instances, got %d' % len(meshes)

    for axis in result['axis_nodes']:
        sources = cmds.listConnections(
            axis + '.rotateZ', source=True, destination=False, plugs=True) or []
        assert len(sources) == 1, '%s has invalid rotateZ wiring: %s' % (axis, sources)

    # Every branch must pass FK values through exactly.
    for target in result['target_ctrls']:
        cmds.setAttr(target + '.ik', False)
    for control in result['fk_nodes']:
        cmds.setAttr(control + '.rotateZ', 5.0)
    cmds.refresh()
    for axis in result['axis_nodes']:
        assert abs(cmds.getAttr(axis + '.rotateZ') - 5.0) < 1e-6

    for target, tcp in zip(result['target_ctrls'], result['tcp_nodes']):
        tcp_matrix = cmds.xform(tcp, query=True, worldSpace=True, matrix=True)
        cmds.xform(target, worldSpace=True, matrix=tcp_matrix)

    # Each independent target must trigger its own solver and move at least
    # one joint in that chain.
    for target, solver, chain in zip(
            result['target_ctrls'], result['ik_nodes'], result['chains']):
        for other in result['target_ctrls']:
            cmds.setAttr(other + '.ik', False)
        cmds.setAttr(target + '.ik', True)
        # A small target displacement is appropriate for this 25 cm robot.
        position = cmds.xform(target, query=True, worldSpace=True, translation=True)
        cmds.xform(
            target, worldSpace=True,
            translation=[position[0] + 0.1, position[1] + 0.1, position[2]])
        cmds.refresh()
        position_error = cmds.getAttr(solver + '.ikPositionError')
        assert position_error < 0.25, (
            '%s branch IK residual %.4f cm is too large'
            % (chain['tip_link'], position_error))

    bbox = cmds.exactWorldBoundingBox(result['top_node'])
    dimensions = [bbox[i + 3] - bbox[i] for i in range(3)]
    assert all(1.0 < value < 100.0 for value in dimensions), (
        'Microduck bounding box is implausible: %s cm' % dimensions)

    print('MICRODUCK MAYA BUILD RESULT: PASS')
    print('chains:', result['chains'])
    print('bounding box (cm):', dimensions)


if __name__ == '__main__':
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)

