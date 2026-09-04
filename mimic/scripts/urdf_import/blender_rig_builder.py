#!usr/bin/env python
"""
Blender port of rig_builder.py.

Deliberately shares BOTH the URDF parser (urdf_parser.py) and the IK solver
(robotmath/generic_ik.py) with the Maya path -- neither has any DCC
dependency, so the only thing that differs between hosts is scene-graph
construction. That means a rig built here and a rig built in Maya are
driven by identical kinematics, which is what makes cross-validating one
against the other meaningful.

Two things are simpler here than in Maya:

  * Units/orientation. URDF is metres, Z-up; Blender is metres, Z-up. So
    unlike the Maya builder -- which converts metres->centimetres and
    Z-up->Y-up, and must apply that conversion to joint origins AND mesh
    vertex data alike -- this builder does no conversion at all.
  * Rotation. Blender objects expose rotation_euler directly, so the
    "rotate about local Z" convention needs no channel remapping.

The joint-frame decomposition is the same as the Maya builder's: each pose
joint becomes a static `axis{i}_frame` empty holding the rest origin with
its local +Z aligned to the URDF joint axis, plus an animated `axis{i}`
child that only ever rotates about local Z. Children of that joint are
premultiplied by the inverse alignment so raw URDF-relative numbers still
land correctly (see rig_builder.py's module docstring for the full
rationale).
"""

import math
import os

import numpy as np

try:
    import bpy
    import mathutils
    BLENDER_IS_RUNNING = True
except ImportError:
    bpy = None
    mathutils = None
    BLENDER_IS_RUNNING = False

from robotmath import generic_ik
import model_parser
import urdf_parser
import rig_builder as _mrb  # Maya builder's module; only its pure-Python
                             # branch-selection helpers are reused here
                             # (_is_branched, _select_independent_chains),
                             # never anything touching maya.cmds. Its own
                             # import is guarded, so importing it works fine
                             # with no Maya present -- see rig_builder.py's
                             # top-of-file try/except.


class RigBuildError(Exception):
    pass


def _require_blender():
    if not BLENDER_IS_RUNNING:
        raise RigBuildError('blender_rig_builder requires bpy; run inside Blender')


def _homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _rpy_matrix(rpy):
    return np.array(urdf_parser._rpy_to_matrix(rpy))


def _align_z_to(axis):
    """3x3 rotation R such that R @ [0,0,1] == axis (axis must be unit length)."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    dot = float(np.dot(z, axis))
    if dot > 1 - 1e-9:
        return np.eye(3)
    if dot < -1 + 1e-9:
        return generic_ik.rotation_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.pi)
    cross = np.cross(z, axis)
    return generic_ik.rotation_from_axis_angle(
        cross / np.linalg.norm(cross), math.acos(max(-1.0, min(1.0, dot))))


def _to_bl(M):
    """numpy 4x4 -> mathutils.Matrix (both use the column-vector convention)."""
    return mathutils.Matrix([list(row) for row in M])


def _new_empty(name, parent=None, local_matrix=None, size=0.02):
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = 'PLAIN_AXES'
    empty.empty_display_size = size
    bpy.context.collection.objects.link(empty)
    if parent is not None:
        _parent_keep_local(empty, parent, local_matrix if local_matrix is not None else np.eye(4))
    elif local_matrix is not None:
        empty.matrix_basis = _to_bl(local_matrix)
    return empty


def _parent_keep_local(child, parent, local_matrix):
    """
    Parent `child` under `parent` with an exact local transform.

    Blender composes world as:
        matrix_world = parent.matrix_world @ matrix_parent_inverse @ matrix_basis
    so pinning matrix_parent_inverse to identity makes matrix_basis mean
    "transform relative to parent", which is what the URDF gives us.
    """
    child.parent = parent
    child.matrix_parent_inverse = mathutils.Matrix.Identity(4)
    child.matrix_basis = _to_bl(local_matrix)


def _import_stl(path):
    if not os.path.isfile(path):
        raise RigBuildError('Mesh file not found: %s' % path)
    before = set(bpy.context.scene.objects)
    if hasattr(bpy.ops.wm, 'stl_import'):
        bpy.ops.wm.stl_import(filepath=path)          # Blender 4.2+
    else:
        bpy.ops.import_mesh.stl(filepath=path)         # legacy
    new = [o for o in bpy.context.scene.objects if o not in before]
    if not new:
        raise RigBuildError('STL import produced no new object for %s' % path)
    return new[0]


def _attach_mesh(mesh_path, local_matrix, scale, parent, name):
    obj = _import_stl(mesh_path)
    obj.name = name
    M = local_matrix.copy()
    # URDF metres == Blender metres, so the only scaling here is the URDF's
    # own <mesh scale="...">, if any. (The Maya builder additionally folds in
    # a metres->centimetres factor; omitting it there leaves every part 100x
    # too small.)
    M[:3, :3] = M[:3, :3] @ np.diag(np.array(scale, dtype=float))
    _parent_keep_local(obj, parent, M)
    return obj


def _attach_link_meshes(robot, link_name, parent, compensation, prefix):
    link = robot.links.get(link_name, urdf_parser.Link(link_name))
    out = []
    for idx, mesh in enumerate(link.meshes):
        raw = np.array(mesh.origin_matrix, dtype=float)
        out.append(_attach_mesh(mesh.filename, compensation @ raw, mesh.scale,
                                 parent, '%s_mesh%d' % (prefix, idx)))
    return out


def _joint_origin(joint):
    return np.array(joint.origin_matrix, dtype=float)


def _build_branched_rig(robot, robot_name):
    """
    Blender counterpart of rig_builder._build_branched_rig(): builds the
    COMPLETE FK tree (every revolute/continuous joint in the URDF/MJCF, not
    just the longest chain) plus one independent IK target per disjoint
    limb -- e.g. Microduck's left leg, head, and right leg each get their
    own target_CTRL and solve separately. Joints two limbs would both need
    (shared upstream joints) are left FK-only for the shallower limb, same
    policy as the Maya builder, since one rotation channel can't take
    output from two independent solvers.

    Branch selection itself (_is_branched, _select_independent_chains) is
    reused verbatim from rig_builder.py -- pure Python, no maya.cmds -- so
    Maya and Blender always agree on which limbs get IK and which don't.
    """
    name = robot_name or robot.name
    top = _new_empty('%s_RIG' % name, size=0.05)
    base = _new_empty('Base', top, np.array(robot.root_origin_matrix, dtype=float))
    root_link = robot.root_link()
    _attach_link_meshes(robot, root_link, base, np.eye(4), 'base')

    warnings = list(robot.warnings)
    axis_objs_by_joint = {}
    axis_objs = []
    leaf_nodes = {}

    def build_children(link_name, parent_obj, parent_align_inv):
        children = robot.children_of(link_name)
        supported = [j for j in children
                     if j.type in urdf_parser.SUPPORTED_DOF_TYPES or j.type == 'fixed']
        if not supported:
            leaf_nodes[link_name] = parent_obj

        for joint in children:
            if joint.type not in urdf_parser.SUPPORTED_DOF_TYPES and joint.type != 'fixed':
                warnings.append(
                    'Joint "%s" has unsupported type "%s"; its subtree was skipped'
                    % (joint.name, joint.type))
                continue

            origin = _joint_origin(joint)
            if joint.type == 'fixed':
                frame = _new_empty(joint.name + '_frame', parent_obj,
                                    parent_align_inv @ origin, size=0.01)
                _attach_link_meshes(robot, joint.child, frame, np.eye(4), joint.child)
                build_children(joint.child, frame, np.eye(4))
                continue

            A = _align_z_to(joint.axis)
            A_h = _homog(A, np.zeros(3))
            frame = _new_empty(joint.name + '_frame', parent_obj,
                                (parent_align_inv @ origin) @ A_h, size=0.015)
            axis = _new_empty(joint.name + '_axis', frame, np.eye(4), size=0.03)
            axis.rotation_mode = 'XYZ'
            axis_objs.append(axis)
            axis_objs_by_joint[joint.name] = axis

            A_inv_h = _homog(A.T, np.zeros(3))
            _attach_link_meshes(robot, joint.child, axis, A_inv_h, joint.child)
            build_children(joint.child, axis, A_inv_h)

    build_children(root_link, base, np.eye(4))

    selected_chains, skipped_chains = _mrb._select_independent_chains(
        robot, leaf_nodes.keys())
    for tip_link, reason in skipped_chains:
        warnings.append('Branch ending at "%s" was built FK-only (%s)' % (tip_link, reason))

    chains = []
    claimed = set()
    for tip_link, chain in selected_chains:
        tip_obj = leaf_nodes[tip_link]
        joint_names = [cj.joint.name for cj in chain]
        claimed.update(joint_names)

        chain_axis_objs = [axis_objs_by_joint[n] for n in joint_names]
        joint_specs = []
        previous_align_inv = np.eye(4)
        for cj in chain:
            joint = cj.joint
            A = _align_z_to(joint.axis)
            frame_local = (previous_align_inv @ np.array(cj.origin_matrix, dtype=float)) \
                @ _homog(A, np.zeros(3))
            joint_specs.append(generic_ik.JointSpec(
                frame_local, [0.0, 0.0, 1.0], joint.limit_lower, joint.limit_upper))
            previous_align_inv = _homog(A.T, np.zeros(3))

        tcp_offset = previous_align_inv.copy()
        tcp = _new_empty('tcp_' + tip_link, tip_obj, tcp_offset, size=0.03)
        bpy.context.view_layer.update()
        target = _new_empty('target_CTRL_' + tip_link, top, np.eye(4), size=0.04)
        target.empty_display_type = 'ARROWS'
        target.matrix_world = tcp.matrix_world.copy()

        chains.append({
            'tip_link': tip_link,
            'target': target,
            'tcp': tcp,
            'tcp_offset': tcp_offset,
            'joint_specs': joint_specs,
            'axis_objs': chain_axis_objs,
        })

    if not chains:
        raise RigBuildError('No solvable revolute/continuous branch found in %s' % robot.urdf_path)

    return {
        'top': top,
        'base': base,
        'axis_objs': axis_objs,
        'tool_objs': [],
        'chains': chains,
        'target': chains[0]['target'],
        'tcp': chains[0]['tcp'],
        'tcp_offset': chains[0]['tcp_offset'],
        'joint_specs': chains[0]['joint_specs'],
        'pose_joint_count': len(axis_objs),
        'tool_joint_count': 0,
        'branched': True,
        'warnings': warnings,
    }


def build_rig(urdf_path, robot_name=None, gripper_joint_names=None, tip_link=None):
    """
    Builds the rig in the current Blender scene. Returns a dict summary
    mirroring rig_builder.build_rig()'s, plus 'joint_specs' (the JointSpec
    list) and 'target' (the IK goal empty) so callers can drive IK via
    solve_to_target(). For a branched robot (humanoid, quadruped, dual-arm),
    see _build_branched_rig() -- 'chains' holds one entry per independently
    solvable limb, and the top-level 'target'/'joint_specs'/etc keys are
    just chains[0], kept for callers that only care about one arm.
    """
    _require_blender()

    robot = model_parser.parse(urdf_path)
    if _mrb._is_branched(robot) and tip_link is None:
        return _build_branched_rig(robot, robot_name)

    chain = robot.kinematic_chain(tip_link=tip_link, gripper_joint_names=gripper_joint_names)
    pose_chain = [cj for cj in chain if cj.is_pose_dof]
    tool_chain = [cj for cj in chain if not cj.is_pose_dof]

    if not pose_chain:
        raise RigBuildError('No revolute/continuous pose joints found in %s' % urdf_path)

    name = robot_name or robot.name

    top = _new_empty('%s_RIG' % name, size=0.05)
    base = _new_empty('Base', top, np.array(robot.root_origin_matrix, dtype=float))

    _attach_link_meshes(robot, robot.root_link(), base, np.eye(4), 'base')

    prev_rot = base
    prev_align_inv = np.eye(4)
    axis_objs = []
    joint_specs = []

    for i, cj in enumerate(pose_chain):
        joint = cj.joint
        origin = np.array(cj.origin_matrix, dtype=float)  # exact; see ChainJoint
        A = _align_z_to(joint.axis)
        A_h = _homog(A, np.zeros(3))

        frame_local = (prev_align_inv @ origin) @ A_h

        frame_obj = _new_empty('axis%d_frame' % (i + 1), prev_rot, frame_local, size=0.015)
        axis_obj = _new_empty('axis%d' % (i + 1), frame_obj, np.eye(4), size=0.03)
        axis_obj.rotation_mode = 'XYZ'
        axis_objs.append(axis_obj)

        # Solver-side spec must describe exactly the geometry built above:
        # this origin relative to the previous joint, rotating about local Z.
        joint_specs.append(generic_ik.JointSpec(
            frame_local, [0.0, 0.0, 1.0], joint.limit_lower, joint.limit_upper))

        A_inv_h = _homog(A.T, np.zeros(3))
        _attach_link_meshes(robot, joint.child, axis_obj, A_inv_h, 'link%d' % (i + 1))

        prev_rot = axis_obj
        prev_align_inv = A_inv_h

    # prev_align_inv is the constant rotation (zero translation) that
    # compensates for the last joint's Z-axis alignment -- see the module
    # docstring. `tcp` (and target_CTRL, seeded from it below) is placed
    # with this baked in, so the solver must be told about the same offset
    # via tcp_offset, or its internal notion of "tip orientation" disagrees
    # with where target_CTRL visibly sits and the down-weighted orientation
    # term drags position off-target even for a reachable/rest pose.
    tcp_offset = prev_align_inv.copy()
    tcp = _new_empty('tcp', prev_rot, tcp_offset, size=0.04)

    tool_objs = []
    tp, ta = prev_rot, prev_align_inv
    for j, cj in enumerate(tool_chain):
        joint = cj.joint
        origin = np.array(cj.origin_matrix, dtype=float)  # exact; see ChainJoint
        A = _align_z_to(joint.axis)
        frame_local = (ta @ origin) @ _homog(A, np.zeros(3))
        frame_obj = _new_empty('tool%d_frame' % (j + 1), tp, frame_local, size=0.01)
        tool_obj = _new_empty('tool%d' % (j + 1), frame_obj, np.eye(4), size=0.02)
        tool_obj.rotation_mode = 'XYZ'
        tool_objs.append(tool_obj)
        A_inv_h = _homog(A.T, np.zeros(3))
        _attach_link_meshes(robot, joint.child, tool_obj, A_inv_h, 'tool%d' % (j + 1))
        tp, ta = tool_obj, A_inv_h

    # IK goal, parked at the rest-pose TCP so it starts reachable.
    bpy.context.view_layer.update()
    target = _new_empty('target_CTRL', top, np.eye(4), size=0.05)
    target.empty_display_type = 'ARROWS'
    target.matrix_world = tcp.matrix_world.copy()

    return {
        'top': top,
        'base': base,
        'axis_objs': axis_objs,
        'tool_objs': tool_objs,
        'tcp': tcp,
        'target': target,
        'tcp_offset': tcp_offset,
        'joint_specs': joint_specs,
        'chains': [{
            'tip_link': None,
            'target': target,
            'tcp': tcp,
            'tcp_offset': tcp_offset,
            'joint_specs': joint_specs,
            'axis_objs': axis_objs,
        }],
        'pose_joint_count': len(pose_chain),
        'tool_joint_count': len(tool_chain),
        'branched': False,
        'warnings': list(robot.warnings),
    }


def get_joint_angles(rig):
    return np.array([obj.rotation_euler.z for obj in rig['axis_objs']])


def set_joint_angles(rig, thetas):
    for obj, theta in zip(rig['axis_objs'], thetas):
        obj.rotation_euler.z = float(theta)
    bpy.context.view_layer.update()


def solve_chain(rig, chain, **kwargs):
    """
    Solves IK for one specific limb/chain (an entry from rig['chains']) so
    its tip reaches its own target_CTRL_<tip_link> empty, applies the
    result, and returns (thetas, converged, pos_err, rot_err).

    The target is taken relative to `base`, NOT `top`: every chain's first
    joint frame is built as a child of `base` (see build_rig /
    _build_branched_rig), so that -- not the rig's top-level group -- is
    where a JointSpec chain's implicit identity origin actually lives.
    `base` itself carries robot.root_origin_matrix relative to `top` (the
    MJCF floating-base offset, for instance), which is otherwise silently
    dropped: measuring against `top` instead of `base` is exactly what
    produced a constant, robot-specific mistranslation equal to that
    offset -- invisible on SO-ARM100/101 (whose root_origin_matrix happens
    to be identity) but a firm 0.12m Z error on Microduck, whose MJCF
    trunk sits above the ground.
    """
    _require_blender()
    bpy.context.view_layer.update()

    root_inv = np.array(rig['base'].matrix_world.inverted())
    target_local = root_inv @ np.array(chain['target'].matrix_world)

    # A <6-DOF limb cannot reach an arbitrary 6-DOF pose, so orientation is
    # down-weighted by default to keep the tip on the handle. See
    # generic_ik.recommended_rot_weight().
    kwargs.setdefault('rot_weight',
                      generic_ik.recommended_rot_weight(len(chain['joint_specs'])))
    kwargs.setdefault('tcp_offset', chain.get('tcp_offset'))

    thetas, converged, pos_err, rot_err = generic_ik.solve_ik_robust(
        chain['joint_specs'], get_joint_angles(chain), target_local, **kwargs)
    set_joint_angles(chain, thetas)
    return thetas, converged, pos_err, rot_err


_live_ik_poll_state = {}


def install_live_ik(rig):
    """
    Polls every chain's target_CTRL ~20x/sec and re-solves that chain's IK
    whenever its target moves -- this is what makes dragging a target in
    the viewport feel live. One timer per chain, so limbs solve
    independently (see the Microduck/G1 per-limb drag tests in
    URDF_IMPORT.md).

    Uses bpy.app.timers rather than a depsgraph handler because mutating
    object transforms from inside depsgraph_update_post is unsafe in
    Blender -- a timer is the sanctioned way to react to scene state and
    then write back to it.

    Safe to call more than once for the same rig (e.g. re-enabling the
    add-on): each chain's poll state is keyed by its target object's
    identity, so re-registering just starts fresh from the current pose
    rather than stacking duplicate timers with stale closures.
    """
    _require_blender()

    def make_poll(chain):
        key = id(chain['target'])
        _live_ik_poll_state[key] = None

        def poll():
            try:
                cur = chain['target'].matrix_world.copy()
            except (ReferenceError, KeyError):
                return None  # target/rig gone; stop polling
            if _live_ik_poll_state.get(key) is None or cur != _live_ik_poll_state[key]:
                _live_ik_poll_state[key] = cur.copy()
                try:
                    solve_chain(rig, chain)
                except Exception as exc:
                    print('[urdf_import] IK solve error (%s):' % chain.get('tip_link'), exc)
            return 0.05
        return poll

    for chain in rig['chains']:
        bpy.app.timers.register(make_poll(chain))


def record_waypoint(rig, frame=None):
    """
    Solves every chain fresh, then keyframes every joint's resulting
    rotation_euler.z at `frame` (defaults to the current playhead) -- i.e.
    "record a waypoint" in one call, the one-click equivalent of manually
    pressing I on every axis object after dragging a target. Includes
    tool/gripper joints (rig['tool_objs'], or already folded into
    rig['axis_objs'] for a branched rig -- see _build_branched_rig), so a
    gripper pose is part of the recording too.

    Solving here rather than trusting install_live_ik()'s timer to have
    already caught up matters more than it looks: bpy.app.timers only
    ticks during Blender's normal running event loop, so it silently does
    nothing in --background scripts, and even interactively a timer poll
    is at most ~50ms behind a drag -- recording immediately after moving a
    target could otherwise keyframe the pose from *before* the move. This
    makes the recorded waypoint always match wherever the target actually
    is at record time, independent of timer timing.

    Returns the frame number the waypoint was recorded at.

    Raises whatever solve_chain() raises (doesn't swallow it): a caller
    asked to record a waypoint should find out if the pose being keyframed
    couldn't actually be solved, not silently get one keyframed anyway.
    """
    _require_blender()
    for chain in rig['chains']:
        solve_chain(rig, chain)
    if frame is None:
        frame = bpy.context.scene.frame_current
    for obj in list(rig['axis_objs']) + list(rig.get('tool_objs', [])):
        obj.keyframe_insert(data_path='rotation_euler', index=2, frame=frame)
    return frame


def solve_to_target(rig, target=None, **kwargs):
    """
    Single-chain convenience wrapper around solve_chain(): solves
    rig['chains'][0] (every rig has at least one chain). Pass `target` to
    override which empty is used as the IK goal for that first chain
    (defaults to its own target_CTRL). For a branched rig with more than
    one independently-solvable limb, use solve_chain(rig, rig['chains'][i])
    for the other limbs.
    """
    chain = dict(rig['chains'][0])
    if target is not None:
        chain['target'] = target
    return solve_chain(rig, chain, **kwargs)
