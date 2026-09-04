#!usr/bin/env python
"""
Standalone (no Maya) validation: parse SO-ARM100/101 URDFs, build JointSpecs
in the URDF's own frame (meters, whatever origin convention it uses), run
forward kinematics for random poses, then solve IK back to that same target
from a different seed and check convergence.

Run with plain python3 (needs numpy, which is on this machine).
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mimic', 'scripts', 'urdf_import'))

import numpy as np

import urdf_parser as up
from robotmath import generic_ik as gik

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
URDFS = [
    os.path.join(REPO_ROOT, "mimic", "robot_descriptions", "SO100", "so100.urdf"),
    os.path.join(REPO_ROOT, "mimic", "robot_descriptions", "SO101", "so101_new_calib.urdf"),
]


def joint_specs_from_pose_chain(pose_chain):
    specs = []
    for cj in pose_chain:
        origin = np.array(cj.origin_matrix, dtype=float)
        specs.append(gik.JointSpec(origin, cj.joint.axis, cj.joint.limit_lower, cj.joint.limit_upper))
    return specs


def random_valid_angles(specs, rng):
    angles = []
    for s in specs:
        lo = s.lower if s.lower is not None else -math.pi
        hi = s.upper if s.upper is not None else math.pi
        angles.append(rng.uniform(lo, hi))
    return np.array(angles)


def run_for_urdf(path, n_trials=25, seed=0):
    print('=' * 20, path)
    robot = up.parse(path)
    chain = robot.kinematic_chain()
    pose_chain = [cj for cj in chain if cj.is_pose_dof]
    print('pose DOFs (%d):' % len(pose_chain), [cj.joint.child for cj in pose_chain])
    for w in robot.warnings:
        print('  warning:', w)

    specs = joint_specs_from_pose_chain(pose_chain)
    rng = random.Random(seed)

    n_ok = 0
    max_pos_err = 0.0
    max_rot_err = 0.0
    for trial in range(n_trials):
        true_angles = random_valid_angles(specs, rng)
        _, target = gik.forward_kinematics(specs, true_angles)

        seed_angles = np.zeros(len(specs))  # deliberately different from true_angles
        solved, converged, pos_err, rot_err = gik.solve_ik_robust(
            specs, seed_angles, target, rng=np.random.default_rng(trial))

        _, achieved = gik.forward_kinematics(specs, solved)
        real_pos_err = np.linalg.norm(achieved[:3, 3] - target[:3, 3])
        max_pos_err = max(max_pos_err, real_pos_err)
        max_rot_err = max(max_rot_err, rot_err)

        ok = converged and real_pos_err < 1e-3  # 1e-3 m = 1mm (URDF is in meters)
        n_ok += int(ok)
        if not ok:
            print('  FAIL trial %d: converged=%s pos_err=%.6f rot_err=%.6f' %
                  (trial, converged, real_pos_err, rot_err))

    print('  %d/%d trials converged to <1mm position error (max pos err %.6f m, max rot err %.6f rad)' %
          (n_ok, n_trials, max_pos_err, max_rot_err))
    return n_ok == n_trials


if __name__ == '__main__':
    all_pass = True
    for path in URDFS:
        all_pass &= run_for_urdf(path)
    print()
    print('RESULT:', 'PASS' if all_pass else 'FAIL')
    sys.exit(0 if all_pass else 1)
