"""
Ground-truth check of the BUILT MAYA RIG (not just the math) against pybullet.

Runs in two stages because pybullet isn't importable inside mayapy:
  stage 'maya'    -- build rig in Maya, pose it, dump world positions to JSON
  stage 'compare' -- load that JSON, compute pybullet truth, compare

Also dumps each imported MESH's world centroid so we can tell whether the
geometry is assembled into an arm or scattered around the origin.
"""
import sys, os, json, math

STAGE = sys.argv[1]
JSON_PATH = sys.argv[2]
URDF = sys.argv[3]

# Same test pose used by both stages (degrees -> radians)
TEST_POSE_DEG = [20.0, -35.0, 45.0, 15.0, -60.0]


def stage_maya():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts'))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts', 'urdf_import'))
    import maya.standalone
    maya.standalone.initialize(name='python')
    import maya.cmds as cmds
    cmds.file(new=True, force=True)
    cmds.loadPlugin(os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'mimic', 'plug-ins', 'robotIKGeneric.py')))
    import rig_builder
    r = rig_builder.build_rig(URDF)

    # FK mode, apply the test pose
    cmds.setAttr(r['target_ctrl'] + '.ik', False)
    for node, deg in zip(r['fk_nodes'], TEST_POSE_DEG):
        cmds.setAttr(node + '.rotateZ', deg)
    cmds.dgeval(r['ik_node'])
    for a in r['axis_nodes']:
        cmds.dgeval(a)

    out = {'axis_world': [], 'meshes': [], 'pose_deg': TEST_POSE_DEG}
    for a in r['axis_nodes']:
        t = cmds.xform(a, q=True, ws=True, t=True)
        out['axis_world'].append(t)

    # every mesh's world-space bbox centre, with which axis it hangs under
    for mesh in cmds.ls(r['top_node'], dagObjects=True, long=True, type='mesh') or []:
        bb = cmds.exactWorldBoundingBox(mesh)
        centre = [(bb[0]+bb[3])/2, (bb[1]+bb[4])/2, (bb[2]+bb[5])/2]
        size = [bb[3]-bb[0], bb[4]-bb[1], bb[5]-bb[2]]
        nverts = cmds.polyEvaluate(mesh, vertex=True)
        out['meshes'].append({'name': mesh.split('|')[-1], 'centre': centre,
                              'size': size, 'verts': nverts,
                              'parent': cmds.listRelatives(mesh, parent=True, fullPath=True)[0]})
    json.dump(out, open(JSON_PATH, 'w'), indent=1)
    print('wrote', JSON_PATH, 'axes:', len(out['axis_world']), 'meshes:', len(out['meshes']))


def stage_compare():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts'))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts', 'urdf_import'))
    import numpy as np, pybullet as p
    import urdf_parser as up
    data = json.load(open(JSON_PATH))
    robot = up.parse(URDF)
    pose = [cj for cj in robot.kinematic_chain() if cj.is_pose_dof]

    p.connect(p.DIRECT)
    body = p.loadURDF(URDF, useFixedBase=True)
    name2idx = {}
    for j in range(p.getNumJoints(body)):
        name2idx[p.getJointInfo(body, j)[1].decode()] = j
    idxs = [name2idx[cj.joint.name] for cj in pose]
    for j in range(p.getNumJoints(body)):
        p.resetJointState(body, j, 0.0)
    for idx, deg in zip(idxs, data['pose_deg']):
        p.resetJointState(body, idx, math.radians(deg))

    # Maya rig is cm + Y-up (Rx(-90) at local_CTRL): urdf(x,y,z)m -> maya(x, z, -y)cm
    def urdf_to_maya(v):
        return np.array([v[0]*100.0, v[2]*100.0, -v[1]*100.0])

    print('%-6s %-34s %-34s %s' % ('axis', 'maya world (cm)', 'pybullet truth (cm)', 'err cm'))
    worst = 0.0
    for i, (cj, idx) in enumerate(zip(pose, idxs)):
        ls = p.getLinkState(body, idx, computeForwardKinematics=True)
        # link frame ORIGIN of the child link == the joint's own position
        truth = urdf_to_maya(np.array(ls[4]))
        got = np.array(data['axis_world'][i])
        err = float(np.linalg.norm(got - truth))
        worst = max(worst, err)
        fmt = lambda v: '(%8.3f %8.3f %8.3f)' % tuple(v)
        print('%-6s %-34s %-34s %.5f' % ('axis%d' % (i+1), fmt(got), fmt(truth), err))
    print()
    print('worst axis-position error: %.6f cm' % worst)

    print()
    print('--- mesh assembly check ---')
    cs = np.array([m['centre'] for m in data['meshes']])
    print('mesh count: %d, total verts: %d' % (len(data['meshes']), sum(m['verts'] for m in data['meshes'])))
    print('mesh centroid spread (cm): x %.2f  y %.2f  z %.2f' %
          tuple(cs.max(axis=0) - cs.min(axis=0)))
    for m in data['meshes']:
        print('  %-22s under %-26s centre=(%7.2f %7.2f %7.2f) size=(%5.2f %5.2f %5.2f) v=%d' % (
            m['name'], m['parent'].split('|')[-1], m['centre'][0], m['centre'][1], m['centre'][2],
            m['size'][0], m['size'][1], m['size'][2], m['verts']))
    print()
    print('VERDICT:', 'RIG MATCHES PYBULLET' if worst < 1e-3 else '*** RIG PLACEMENT WRONG (%.4f cm) ***' % worst)


if __name__ == '__main__':
    (stage_maya if STAGE == 'maya' else stage_compare)()
