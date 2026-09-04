#!usr/bin/env python
"""
Generic N-DOF numerical inverse kinematics.

Unlike inverse_kinematics.py (which implements two closed-form solvers, each
hardcoded to a specific 6-revolute-joint topology: spherical-wrist or
Hawkins-Keating DH), this module makes no assumption about joint count or
arrangement. It works from an explicit list of JointSpec objects (each
joint's rest origin relative to its parent, plus the axis it rotates about)
and solves via damped least-squares (Levenberg-Marquardt) on the geometric
Jacobian, seeded from the current joint angles so solutions stay continuous
under small target motions (matches the "drag the target and the arm
follows smoothly" behavior Mimic's rigs already have).

No Maya dependency -- unit-testable with plain `python3` + numpy. The
robotIKGeneric plug-in node (plug-ins/robotIKGeneric.py) is a thin Maya
wrapper around solve_ik().

All quantities are plain 4x4 numpy arrays / 3-vectors, in whatever single
consistent frame/unit the caller uses throughout (rig_builder.py is
responsible for converting URDF meters/Z-up into Maya centimeters/Y-up
*before* constructing JointSpecs, so this module never has to know about
either convention).
"""

import numpy as np


class JointSpec(object):
    def __init__(self, origin, axis, lower=None, upper=None):
        """
        :param origin: 4x4 ndarray, this joint's rest transform relative to
            the previous joint's frame (translation + any fixed rotation
            folded in already by the URDF parser).
        :param axis: 3-vector, unit length, the axis this joint rotates
            about, expressed in the joint's own frame (i.e. after `origin`
            is applied, before the joint's own rotation).
        :param lower: radians, or None for unlimited.
        :param upper: radians, or None for unlimited.
        """
        self.origin = np.asarray(origin, dtype=float)
        axis = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            raise ValueError('joint axis must be non-zero')
        self.axis = axis / norm
        self.lower = lower
        self.upper = upper


def rotation_from_axis_angle(axis, theta):
    """Rodrigues' formula. axis must already be unit length."""
    x, y, z = axis
    c = np.cos(theta)
    s = np.sin(theta)
    C = 1 - c
    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def _homogeneous(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def forward_kinematics(joints, thetas, tcp_offset=None):
    """
    Returns (frames, tcp_matrix):
      frames[i] = 4x4 world transform of joint i's origin frame (before its
                  own rotation is baked in that frame stays the *pre-joint*
                  reference used for the Jacobian's axis/point calc), and
      tcp_matrix = final end-effector world transform (after the last
                   joint's rotation and the optional fixed tcp_offset).
    """
    if tcp_offset is None:
        tcp_offset = np.eye(4)

    world = np.eye(4)
    frames = []
    for j, theta in zip(joints, thetas):
        world = world @ j.origin
        frames.append(world.copy())
        R = rotation_from_axis_angle(j.axis, theta)
        world = world @ _homogeneous(R, np.zeros(3))
    tcp_matrix = world @ tcp_offset
    return frames, tcp_matrix


def geometric_jacobian(joints, thetas, tcp_offset=None):
    """
    6xN Jacobian (rows 0-2 linear velocity, rows 3-5 angular velocity of the
    TCP) at the given joint angles, all in world frame.
    """
    n = len(joints)
    frames, tcp_matrix = forward_kinematics(joints, thetas, tcp_offset)
    tcp_pos = tcp_matrix[:3, 3]

    J = np.zeros((6, n))
    for i, (j, frame) in enumerate(zip(joints, frames)):
        axis_world = frame[:3, :3] @ j.axis
        joint_pos = frame[:3, 3]
        J[:3, i] = np.cross(axis_world, tcp_pos - joint_pos)
        J[3:, i] = axis_world
    return J, tcp_matrix


def _orientation_error(R_target, R_current):
    """so(3) log map of R_target * R_current^T -> axis*angle error vector."""
    R_err = R_target @ R_current.T
    cos_theta = (np.trace(R_err) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-8:
        return np.zeros(3)
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) / (2.0 * np.sin(theta))
    return axis * theta


def clamp_to_limits(joints, thetas):
    out = np.array(thetas, dtype=float)
    for i, j in enumerate(joints):
        if j.lower is not None:
            out[i] = max(out[i], j.lower)
        if j.upper is not None:
            out[i] = min(out[i], j.upper)
    return out


def _pose_error(joints, thetas, target_R, target_t, tcp_offset, rot_weight):
    _, tcp_matrix = forward_kinematics(joints, thetas, tcp_offset)
    pos_err = target_t - tcp_matrix[:3, 3]
    rot_err = _orientation_error(target_R, tcp_matrix[:3, :3])
    cost = np.linalg.norm(pos_err) ** 2 + (rot_weight * np.linalg.norm(rot_err)) ** 2
    return pos_err, rot_err, cost


def solve_ik(joints, thetas0, target_matrix, tcp_offset=None,
             max_iters=200, tol_pos=1e-4, tol_rot=1e-3,
             rot_weight=1.0):
    """
    Levenberg-Marquardt IK with adaptive damping: a step is only accepted if
    it actually reduces the pose error, otherwise damping is increased and
    the step retried. This converges far more reliably than a fixed-damping
    Gauss-Newton step, especially for the rank-deficient Jacobians a <6-DOF
    arm (e.g. the SO-ARM100/101's 5-DOF chain) produces against a full 6D
    position+orientation target.

    :param joints: list[JointSpec]
    :param thetas0: initial guess (radians), typically the rig's current
        pose so the solve stays continuous under interactive dragging.
    :param target_matrix: 4x4 desired TCP world transform.
    :param tol_pos: convergence tolerance on position error (same units as
        the joint origins, e.g. cm if the rig is built in cm).
    :param tol_rot: convergence tolerance on orientation error (radians).
        Ignored entirely when rot_weight == 0 (position-only solve).
    :param rot_weight: how strongly orientation error is weighted relative
        to position error. 1.0 = full 6-DOF pose matching. 0.0 = solve for
        position only and let orientation fall where it may -- the right
        choice for arms with fewer than 6 DOF, which generically cannot
        reach an arbitrary position AND orientation simultaneously.
    :return: (thetas, converged, pos_error_norm, rot_error_norm)
    """
    thetas = clamp_to_limits(joints, np.array(thetas0, dtype=float))
    target_R = target_matrix[:3, :3]
    target_t = target_matrix[:3, 3]

    # Row weights for the weighted least-squares problem. These must be
    # applied to the Jacobian as well as the error vector: weighting only
    # the gradient (J.T @ We) while leaving the Gauss-Newton Hessian
    # unweighted (J.T @ J) is inconsistent, and makes the solver damp
    # position corrections in proportion to orientation error it has been
    # told not to care about.
    W = np.ones(6)
    W[3:] = rot_weight

    lam = 1e-2
    pos_err, rot_err, cost = _pose_error(joints, thetas, target_R, target_t, tcp_offset, rot_weight)

    for _ in range(max_iters):
        pos_err_norm = np.linalg.norm(pos_err)
        rot_err_norm = np.linalg.norm(rot_err)
        if pos_err_norm < tol_pos and (rot_weight == 0.0 or rot_err_norm < tol_rot):
            return thetas, True, pos_err_norm, rot_err_norm

        J, _ = geometric_jacobian(joints, thetas, tcp_offset)
        Jw = J * W[:, None]
        ew = np.concatenate([pos_err, rot_err]) * W
        JtJ = Jw.T @ Jw
        grad = Jw.T @ ew

        # Try a step; on failure to improve, grow lambda and retry (classic LM).
        improved = False
        for _sub in range(12):
            try:
                delta = np.linalg.solve(JtJ + lam * np.eye(len(joints)), grad)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(JtJ + lam * np.eye(len(joints)), grad, rcond=None)[0]

            candidate = clamp_to_limits(joints, thetas + delta)
            c_pos_err, c_rot_err, c_cost = _pose_error(
                joints, candidate, target_R, target_t, tcp_offset, rot_weight)

            if c_cost < cost:
                thetas, pos_err, rot_err, cost = candidate, c_pos_err, c_rot_err, c_cost
                lam = max(lam / 3.0, 1e-7)
                improved = True
                break
            else:
                lam = min(lam * 3.0, 1e7)

        if not improved:
            break  # stuck at a local minimum; report what we have

    pos_err_norm = np.linalg.norm(pos_err)
    rot_err_norm = np.linalg.norm(rot_err)
    converged = pos_err_norm < tol_pos and (rot_weight == 0.0 or rot_err_norm < tol_rot)
    return thetas, converged, pos_err_norm, rot_err_norm


def recommended_rot_weight(n_joints):
    """
    Sensible default orientation weight for an arm with `n_joints` DOF.

    An arm with 6+ DOF can generically reach an arbitrary position AND
    orientation, so orientation is weighted fully. An arm with fewer than 6
    DOF cannot -- the SO-ARM100/101 have 5 -- so demanding an exact 6-DOF
    pose asks for something usually impossible: the solver splits the
    difference and the tool tip visibly refuses to sit on the animator's
    handle. Down-weighting orientation makes position win when the two
    conflict, while still matching orientation almost exactly whenever it
    IS achievable.

    Measured on both SO-ARMs. Reachable 6-DOF targets: every weight below
    lands position within ~0.02mm, and 0.05 still nails orientation to
    ~0.01 deg. Unreachable targets (drag the handle, keep the orientation
    -- the routine interactive case on a 5-DOF arm), median residuals:

        rot_weight   position     orientation
          1.0         18.0 mm       0.3 deg     <- tip visibly off the handle
          0.2          8.7 mm       2.3 deg
          0.05         1.4 mm       5.5 deg     <- chosen compromise
          0.0          0.03 mm     18.7 deg     <- gripper orientation adrift

    0.05 keeps the tip within ~1.4mm of the handle on a ~20cm arm (visually
    "on it") without letting orientation wander the way a pure position
    solve does.
    """
    return 1.0 if n_joints >= 6 else 0.05


def solve_ik_robust(joints, thetas0, target_matrix, tcp_offset=None,
                     n_restarts=16, rng=None, **kwargs):
    """
    Wraps solve_ik() with random-restart fallback: tries the given seed
    first (so interactive, incremental drags stay maximally continuous),
    and only if that fails to converge, retries from a handful of random
    valid poses and keeps the best result. Cold starts (e.g. right after
    importing a rig, or a large target teleport) are the case this exists
    for -- small interactive motions almost always converge on the first
    try since the seed is already close to the answer.
    """
    if rng is None:
        rng = np.random.default_rng()

    tol_pos = kwargs.get('tol_pos', 1e-4)

    best = solve_ik(joints, thetas0, target_matrix, tcp_offset, **kwargs)
    if best[1]:
        return best

    for _ in range(n_restarts):
        # Restarts exist to escape local minima that leave the tool far from
        # the goal. If position is already at tolerance and only orientation
        # is outstanding, the goal is most likely simply unreachable in
        # orientation (routine on a <6-DOF arm) -- more restarts cannot fix
        # that, and grinding through all of them on every interactive drag
        # frame is what makes dragging feel sluggish. Stop early instead.
        if best[2] < tol_pos:
            break

        random_seed = np.array([
            rng.uniform(j.lower if j.lower is not None else -np.pi,
                        j.upper if j.upper is not None else np.pi)
            for j in joints
        ])
        candidate = solve_ik(joints, random_seed, target_matrix, tcp_offset, **kwargs)
        if candidate[1]:
            return candidate
        if candidate[2] < best[2]:
            best = candidate

    return best
