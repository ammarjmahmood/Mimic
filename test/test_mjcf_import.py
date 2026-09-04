#!/usr/bin/env python3
"""Standalone parser/FK/IK validation for the bundled Microduck MJCF."""

import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..'))
sys.path[:0] = [
    os.path.join(REPO, 'mimic', 'scripts'),
    os.path.join(REPO, 'mimic', 'scripts', 'urdf_import'),
]

import model_parser
import rig_builder
from robotmath import generic_ik


MODEL = os.path.join(
    REPO, 'mimic', 'robot_descriptions', 'Microduck',
    'robot_allcollisions.xml')

EXPECTED_CHAINS = {
    'ankle_left': [
        'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch',
        'left_knee', 'left_ankle'],
    'jaw_soft': [
        'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll'],
    'ankle_right': [
        'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
        'right_knee', 'right_ankle'],
}


def validate_ik(chain, seed):
    specs = [
        generic_ik.JointSpec(
            np.array(c.origin_matrix), c.joint.axis,
            c.joint.limit_lower, c.joint.limit_upper)
        for c in chain
    ]
    rng = random.Random(seed)
    worst = 0.0
    for trial in range(10):
        desired = np.array([
            rng.uniform(spec.lower if spec.lower is not None else -math.pi,
                        spec.upper if spec.upper is not None else math.pi)
            for spec in specs
        ])
        _, target = generic_ik.forward_kinematics(specs, desired)
        solved, converged, _pos_err, _rot_err = generic_ik.solve_ik_robust(
            specs, np.zeros(len(specs)), target,
            rng=np.random.default_rng(seed * 100 + trial))
        _, achieved = generic_ik.forward_kinematics(specs, solved)
        error = float(np.linalg.norm(achieved[:3, 3] - target[:3, 3]))
        worst = max(worst, error)
        assert converged and error < 1e-3, (
            '%s failed trial %d: converged=%s position error=%g'
            % (chain[-1].joint.child, trial, converged, error))
    return worst


def main():
    robot = model_parser.parse(MODEL)
    assert robot.source_format == 'mjcf'
    assert robot.name == 'microduck'
    assert len(robot.links) == 15
    assert len(robot.joints) == 14
    assert all(joint.type == 'revolute' for joint in robot.joints.values())
    assert abs(robot.root_origin_matrix[2][3] - 0.12) < 1e-12

    meshes = [mesh for link in robot.links.values() for mesh in link.meshes]
    assert len(meshes) == 70
    missing = [mesh.filename for mesh in meshes if not os.path.isfile(mesh.filename)]
    assert not missing, 'Missing mesh assets: %s' % missing

    selected, skipped = rig_builder._select_independent_chains(
        robot, EXPECTED_CHAINS.keys())
    assert not skipped
    assert {tip for tip, _chain in selected} == set(EXPECTED_CHAINS)

    for index, (tip, expected_names) in enumerate(EXPECTED_CHAINS.items()):
        chain = rig_builder._chain_to_link(robot, tip)
        names = [c.joint.name for c in chain]
        assert names == expected_names, '%s: expected %s, got %s' % (
            tip, expected_names, names)
        worst = validate_ik(chain, index + 1)
        print('%s: %d DOF, 10/10 IK round trips, worst %.6f m'
              % (tip, len(chain), worst))

    print('MICRODUCK MJCF RESULT: PASS')


if __name__ == '__main__':
    main()

