"""
Compare the JSON dumped by blender_check.py against pybullet ground truth.

    python3 test/verify_blender_vs_pybullet.py <json> <urdf>

Blender is metres/Z-up, exactly like URDF, so unlike the Maya comparison
there is no unit or up-axis conversion here -- positions are compared
directly. That is itself part of what makes Blender a useful cross-check on
the Maya path: an error in Maya's metres->cm / Z-up->Y-up conversion shows up
as a discrepancy between the two hosts.
"""
import json
import math
import sys

import numpy as np
import pybullet as p


def main(json_path, urdf):
    sys.path.insert(0, '/home/amarsbar/mimic/mimic/scripts')
    sys.path.insert(0, '/home/amarsbar/mimic/mimic/scripts/urdf_import')
    import urdf_parser as up

    data = json.load(open(json_path))
    robot = up.parse(urdf)
    pose = [cj for cj in robot.kinematic_chain() if cj.is_pose_dof]

    p.connect(p.DIRECT)
    body = p.loadURDF(urdf, useFixedBase=True)
    name2idx = {}
    for j in range(p.getNumJoints(body)):
        name2idx[p.getJointInfo(body, j)[1].decode()] = j
    idxs = [name2idx[cj.joint.name] for cj in pose]

    for j in range(p.getNumJoints(body)):
        p.resetJointState(body, j, 0.0)
    for idx, deg in zip(idxs, data['pose_deg']):
        p.resetJointState(body, idx, math.radians(deg))

    print('%-7s %-32s %-32s %s' % ('axis', 'blender world (m)', 'pybullet truth (m)', 'err m'))
    worst = 0.0
    for i, idx in enumerate(idxs):
        truth = np.array(p.getLinkState(body, idx, computeForwardKinematics=True)[4])
        got = np.array(data['axis_world'][i])
        err = float(np.linalg.norm(got - truth))
        worst = max(worst, err)
        f = lambda v: '(%9.5f %9.5f %9.5f)' % tuple(v)
        print('%-7s %-32s %-32s %.3e' % ('axis%d' % (i + 1), f(got), f(truth), err))

    print()
    print('worst axis-position error: %.3e m (%.6f mm)' % (worst, worst * 1000))

    sizes = [max(m['size']) for m in data['meshes']]
    print()
    print('meshes: %d, verts %d' % (len(data['meshes']), sum(m['verts'] for m in data['meshes'])))
    print('per-part longest dimension: min %.4f m  max %.4f m' % (min(sizes), max(sizes)))
    scale_ok = all(0.005 < s < 1.0 for s in sizes)
    print('per-part scale sane (5mm..1m):', scale_ok)

    ik = data.get('ik', {})
    residual_mm = ik.get('scene_residual_m', -1) * 1000
    print()
    # NOTE: converged=False is the EXPECTED result here, not a failure. The
    # IK round-trip drags the goal to a new position while keeping the
    # previous orientation, and a 5-DOF arm generically cannot satisfy both.
    # What matters is that the tool tip lands ON the goal position; the
    # orientation compromise is the documented rot_weight tradeoff (see
    # generic_ik.recommended_rot_weight).
    print('IK round-trip: full-6DOF-converged=%s (False is expected on a 5-DOF arm),'
          % ik.get('converged'))
    print('               tool tip landed %.3f mm from the dragged goal' % residual_mm)

    # Tolerance is set by Blender's precision, not by our maths: Blender
    # stores object transforms as float32 (verified: a float64 location
    # reads back as exactly its float32 value), so a 5-joint chain at
    # ~0.25 m accumulates ~1e-6 m. 1e-5 m (10 micrometres) sits far below
    # anything physically or visually meaningful on a 3D-printed hobby arm
    # while staying safely above that float32 noise floor. The Maya path,
    # which is double precision throughout, is held to 1e-8 m by
    # verify_rig_vs_pybullet.py.
    POS_TOL_M = 1e-5
    pos_ok = worst < POS_TOL_M
    print()
    print('position tolerance %.0e m (Blender float32 noise floor ~1e-6 m): %s'
          % (POS_TOL_M, 'PASS' if pos_ok else 'FAIL'))

    ok = pos_ok and scale_ok
    print('VERDICT:', 'BLENDER RIG MATCHES PYBULLET' if ok else '*** MISMATCH ***')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
