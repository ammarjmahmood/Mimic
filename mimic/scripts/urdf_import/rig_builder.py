#!usr/bin/env python
"""
Builds a Mimic-style Maya rig from a parsed URDF or MJCF description and
wires it to robotIKGeneric plug-in node(s) for live N-DOF numerical IK.

---------------------------------------------------------------------------
Frame convention (why the matrix math below looks the way it does)
---------------------------------------------------------------------------
URDF is meters, and each joint's rotation axis can point anywhere (not
necessarily a principal axis). To keep every generated rig uniform -- so a
single `rotateZ` connection always drives a joint regardless of what
direction its URDF <axis> happened to be authored in -- each pose joint is
built as TWO Maya nodes instead of one:

  axis{i}_frame   (static)   -- holds the joint's rest origin, reoriented so
                                 its local +Z equals the URDF joint axis
  axis{i}         (animated) -- child of axis{i}_frame, ONLY ever has
                                 rotateZ set/connected. Since its parent
                                 already aligned Z to the joint axis, this
                                 is a correct rotation about that axis.

Because axis{i} definitionally has a rest orientation `A_i` (the alignment
rotation) relative to the "pure" URDF link frame, anything positioned using
raw URDF-relative numbers as a child of axis{i} -- the next joint's frame,
or this link's meshes -- must be premultiplied by `A_i^-1` to land in the
same place URDF would put it. See `_align_z_to()` / `_JointFrame` below;
this compensation is the only non-obvious piece of geometry in this file,
everything else is a direct translation of the URDF tree into Maya nodes.

Units/up-axis: URDF meters -> Maya centimeters (x100) is applied once, as
part of every origin matrix. URDF's up-axis (usually Z) -> Maya's Y-up is
applied once, as a fixed rotation on `local_CTRL`, which every axis chains
from -- nothing else needs to know about it.
"""

import math
import os

import numpy as np

try:
    import maya.cmds as cmds
    MAYA_IS_RUNNING = True
except ImportError:
    cmds = None
    MAYA_IS_RUNNING = False

from robotmath import generic_ik
import model_parser
import urdf_parser

METERS_TO_CM = 100.0

# URDF is (by convention) Z-up. Maya is Y-up. This is the one-time fixed
# rotation applied at local_CTRL that reinterprets everything below it
# (built with raw, unrotated URDF numbers) as correctly-oriented Maya
# world geometry: Maya_Y-up = Rx(-90) * URDF_Z-up.
WORLD_UP_CONVERSION_DEG = (-90.0, 0.0, 0.0)


class RigBuildError(Exception):
    pass


def _require_maya():
    if not MAYA_IS_RUNNING:
        raise RigBuildError('rig_builder requires maya.cmds; run inside Maya or mayapy')


def _homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _rpy_matrix(rpy):
    return np.array(urdf_parser._rpy_to_matrix(rpy))


def _chain_origin_cm(chain_joint):
    """ChainJoint.origin_matrix (metres) -> 4x4 numpy matrix in centimetres."""
    M = np.array(chain_joint.origin_matrix, dtype=float)
    M[:3, 3] *= METERS_TO_CM
    return M


def _align_z_to(axis):
    """3x3 rotation R such that R @ [0,0,1] == axis (axis must be unit length)."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    dot = float(np.dot(z, axis))
    if dot > 1 - 1e-9:
        return np.eye(3)
    if dot < -1 + 1e-9:
        # 180 degrees: any axis perpendicular to z works.
        return generic_ik.rotation_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.pi)
    cross = np.cross(z, axis)
    cross_norm = np.linalg.norm(cross)
    angle = math.acos(max(-1.0, min(1.0, dot)))
    return generic_ik.rotation_from_axis_angle(cross / cross_norm, angle)


def _maya_matrix_flat(M_std):
    """numpy 4x4 (column-vector convention) -> Maya's flat row-major list."""
    return M_std.T.flatten().tolist()


def _set_local_matrix(node, M_std):
    cmds.xform(node, objectSpace=True, matrix=_maya_matrix_flat(M_std))


def _group(name, parent=None):
    node = cmds.group(empty=True, name=name)
    if parent:
        node = cmds.parent(node, parent, relative=True)[0]
    return node


def _import_stl(path):
    if not os.path.isfile(path):
        raise RigBuildError('Mesh file not found: %s' % path)
    cmds.loadPlugin('stlTranslator', quiet=True)
    before = set(cmds.ls(assemblies=True, long=True) or [])
    cmds.file(path, i=True, type='STLImport', ignoreVersion=True,
              mergeNamespacesOnClash=False, options='',
              preserveReferences=True)
    after = set(cmds.ls(assemblies=True, long=True) or [])
    new_transforms = list(after - before)
    if not new_transforms:
        raise RigBuildError('STL import produced no new transform for %s' % path)
    return new_transforms[0]


def _attach_mesh(mesh_path, local_matrix, scale, parent, name):
    transform = _import_stl(mesh_path)
    transform = cmds.rename(transform, name)
    cmds.parent(transform, parent, relative=True)
    M = local_matrix.copy()
    # The mesh's VERTEX DATA is in the URDF's units (meters), and Maya's STL
    # importer takes those raw numbers as Maya units (cm). So the same
    # meters->cm conversion applied to joint origins must also be applied to
    # the geometry itself, on top of any <mesh scale="..."> from the URDF.
    # Omitting it leaves every part 100x too small: the joints land in the
    # right places but each link renders as a ~1mm speck.
    mesh_scale = np.array(scale, dtype=float) * METERS_TO_CM
    M[:3, :3] = M[:3, :3] @ np.diag(mesh_scale)
    _set_local_matrix(transform, M)
    return transform


def _attach_link_meshes(robot, link_name, parent, link_frame_compensation, prefix):
    for idx, mesh in enumerate(robot.links.get(link_name, urdf_parser.Link(link_name)).meshes):
        raw = np.array(mesh.origin_matrix, dtype=float)
        raw[:3, 3] *= METERS_TO_CM
        local = link_frame_compensation @ raw
        _attach_mesh(mesh.filename, local, mesh.scale, parent, '%s_mesh%d' % (prefix, idx))


def _joint_origin_cm(joint):
    matrix = np.array(joint.origin_matrix, dtype=float)
    matrix[:3, 3] *= METERS_TO_CM
    return matrix


def _is_branched(robot):
    return any(len([
        joint for joint in robot.children_of(link_name)
        if joint.type in urdf_parser.SUPPORTED_DOF_TYPES or joint.type == 'fixed'
    ]) > 1 for link_name in robot.links)


def _chain_to_link(robot, tip_link):
    """Return a composed ChainJoint path from the root to ``tip_link``."""
    by_child = {joint.child: joint for joint in robot.joints.values()}
    raw = []
    link = tip_link
    root = robot.root_link()
    while link != root:
        joint = by_child.get(link)
        if joint is None:
            return []
        raw.append(joint)
        link = joint.parent
    raw.reverse()

    chain = []
    pending = urdf_parser._identity4()
    for joint in raw:
        compound = urdf_parser._mat4_mul(pending, joint.origin_matrix)
        if joint.type == 'fixed':
            pending = compound
        elif joint.type in urdf_parser.SUPPORTED_DOF_TYPES:
            chain.append(urdf_parser.ChainJoint(joint, compound, is_pose_dof=True))
            pending = urdf_parser._identity4()
        else:
            return []
    return chain


def _select_independent_chains(robot, tip_links, max_joints=12):
    """Choose longest non-overlapping IK chains from a set of leaf links."""
    candidates = []
    for tip_link in tip_links:
        chain = _chain_to_link(robot, tip_link)
        if chain:
            candidates.append((len(chain), tip_link, chain))
    candidates.sort(key=lambda item: (-item[0], item[1]))

    selected = []
    skipped = []
    claimed = set()
    for _length, tip_link, chain in candidates:
        names = [cj.joint.name for cj in chain]
        overlap = claimed.intersection(names)
        if overlap:
            skipped.append((tip_link, 'shared joints: %s' % ', '.join(sorted(overlap))))
        elif len(chain) > max_joints:
            skipped.append((tip_link, '%d joints exceeds maximum %d'
                            % (len(chain), max_joints)))
        else:
            claimed.update(names)
            selected.append((tip_link, chain))
    return selected, skipped


def _add_target_metadata(target, name, joint_count):
    cmds.addAttr(target, longName='robotType', dataType='string')
    cmds.setAttr(target + '.robotType', 'genericImport', type='string', lock=True)
    cmds.addAttr(target, longName='robotSubtype', dataType='string')
    cmds.setAttr(target + '.robotSubtype', name, type='string', lock=True)
    cmds.addAttr(target, longName='numJoints', attributeType='long')
    cmds.setAttr(target + '.numJoints', joint_count, lock=True)
    cmds.addAttr(target, longName='ik', attributeType='bool', defaultValue=False)


def _build_branched_rig(robot, robot_name):
    """Build a complete FK tree and independent IK controls for its limbs.

    IK is assigned to disjoint root-to-leaf DOF paths. This covers biped
    layouts such as Microduck (left leg, head and right leg are independent).
    If paths share a driven joint, the first/deepest path owns it and the
    overlapping limb remains FK-only; one Maya rotate channel cannot safely
    accept outputs from two independent solvers.
    """
    name = robot_name or robot.name
    top = cmds.group(empty=True, name='%s_GRP#' % name)
    robot_grp = _group('robot_GRP', top)
    local_ctrl = _group('local_CTRL', robot_grp)
    cmds.setAttr(local_ctrl + '.rotate', *WORLD_UP_CONVERSION_DEG)
    fk_ctrls_grp = _group('FK_CTRLS', robot_grp)
    robot_geom = _group('robot_GEOM', local_ctrl)
    base = _group('Base', robot_geom)

    root_origin = np.array(robot.root_origin_matrix, dtype=float)
    root_origin[:3, 3] *= METERS_TO_CM
    _set_local_matrix(base, root_origin)
    root_link = robot.root_link()
    _attach_link_meshes(robot, root_link, base, np.eye(4), 'base')

    warnings = list(robot.warnings)
    axis_nodes_by_joint = {}
    fk_nodes_by_joint = {}
    leaf_nodes = {}
    axis_nodes = []
    fk_nodes = []
    axis_index = [0]

    def build_children(link_name, parent_node, parent_align_inv):
        children = robot.children_of(link_name)
        supported_children = [
            joint for joint in children
            if joint.type in urdf_parser.SUPPORTED_DOF_TYPES or joint.type == 'fixed'
        ]
        if not supported_children:
            leaf_nodes[link_name] = parent_node

        for joint in children:
            if joint.type not in urdf_parser.SUPPORTED_DOF_TYPES and joint.type != 'fixed':
                warnings.append(
                    'Joint "%s" has unsupported type "%s"; its subtree was skipped'
                    % (joint.name, joint.type))
                continue

            origin = _joint_origin_cm(joint)
            if joint.type == 'fixed':
                frame = _group(joint.name + '_frame', parent_node)
                _set_local_matrix(frame, parent_align_inv @ origin)
                _attach_link_meshes(
                    robot, joint.child, frame, np.eye(4), joint.child)
                build_children(joint.child, frame, np.eye(4))
                continue

            axis_index[0] += 1
            index = axis_index[0]
            A = _align_z_to(joint.axis)
            A_h = _homog(A, np.zeros(3))
            frame = _group('axis%d_frame' % index, parent_node)
            _set_local_matrix(frame, parent_align_inv @ origin @ A_h)
            axis = _group('axis%d' % index, frame)
            control = _group('a%dFK_CTRL' % index, fk_ctrls_grp)
            axis_nodes.append(axis)
            fk_nodes.append(control)
            axis_nodes_by_joint[joint.name] = axis
            fk_nodes_by_joint[joint.name] = control

            A_inv = _homog(A.T, np.zeros(3))
            _attach_link_meshes(robot, joint.child, axis, A_inv, joint.child)
            build_children(joint.child, axis, A_inv)

    build_children(root_link, base, np.eye(4))

    selected_chains, skipped_chains = _select_independent_chains(
        robot, leaf_nodes.keys())
    for tip_link, reason in skipped_chains:
        warnings.append(
            'Branch ending at "%s" was built FK-only (%s)' % (tip_link, reason))

    claimed = set()
    target_ctrls = []
    tcp_nodes = []
    ik_nodes = []
    chain_summaries = []

    for tip_link, chain in selected_chains:
        tip_node = leaf_nodes[tip_link]
        joint_names = [cj.joint.name for cj in chain]
        claimed.update(joint_names)
        solver = cmds.createNode(
            'robotIKGeneric', name='%s_%s_ikSolver' % (name, tip_link))
        cmds.setAttr(solver + '.numJoints', len(chain))
        target = _group('target_CTRL_' + tip_link, robot_grp)
        _add_target_metadata(target, name + ':' + tip_link, len(chain))
        tcp = _group('tcp_HDL_' + tip_link, tip_node)

        previous_align_inv = np.eye(4)
        for i, cj in enumerate(chain):
            joint = cj.joint
            origin = _chain_origin_cm(cj)
            A = _align_z_to(joint.axis)
            frame_local = previous_align_inv @ origin @ _homog(A, np.zeros(3))
            cmds.setAttr(
                '%s.jointOriginMatrix[%d]' % (solver, i),
                *_maya_matrix_flat(frame_local), type='matrix')
            cmds.setAttr(
                '%s.jointAxis[%d]' % (solver, i), 0.0, 0.0, 1.0, type='double3')
            cmds.setAttr(
                '%s.jointLimitLower[%d]' % (solver, i),
                joint.limit_lower if joint.limit_lower is not None else -1e7)
            cmds.setAttr(
                '%s.jointLimitUpper[%d]' % (solver, i),
                joint.limit_upper if joint.limit_upper is not None else 1e7)
            cmds.connectAttr(
                fk_nodes_by_joint[joint.name] + '.rotateZ',
                '%s.fk[%d]' % (solver, i))
            cmds.connectAttr(
                '%s.theta[%d]' % (solver, i),
                axis_nodes_by_joint[joint.name] + '.rotateZ')
            previous_align_inv = _homog(A.T, np.zeros(3))

        cmds.connectAttr(base + '.worldMatrix[0]', solver + '.lcsMatrix')
        cmds.connectAttr(target + '.worldMatrix[0]', solver + '.targetMatrix')
        cmds.connectAttr(target + '.ik', solver + '.ik')
        cmds.dgeval(tcp)
        tcp_world = cmds.xform(tcp, query=True, worldSpace=True, matrix=True)
        cmds.xform(target, worldSpace=True, matrix=tcp_world)
        cmds.setAttr(target + '.ik', True)

        target_ctrls.append(target)
        tcp_nodes.append(tcp)
        ik_nodes.append(solver)
        chain_summaries.append({'tip_link': tip_link, 'joint_names': joint_names})

    # Joints not owned by an IK branch still remain fully animatable in FK.
    for joint_name, axis in axis_nodes_by_joint.items():
        if joint_name not in claimed:
            cmds.connectAttr(fk_nodes_by_joint[joint_name] + '.rotateZ', axis + '.rotateZ')

    if not target_ctrls:
        raise RigBuildError('No solvable revolute/continuous branch found in %s'
                            % robot.urdf_path)

    return {
        'top_node': top,
        'ik_node': ik_nodes[0],
        'ik_nodes': ik_nodes,
        'target_ctrl': target_ctrls[0],
        'target_ctrls': target_ctrls,
        'local_ctrl': local_ctrl,
        'axis_nodes': axis_nodes,
        'fk_nodes': fk_nodes,
        'gripper_nodes': [],
        'tcp_nodes': tcp_nodes,
        'pose_joint_count': len(axis_nodes),
        'tool_joint_count': 0,
        'chains': chain_summaries,
        'source_format': robot.source_format,
        'warnings': warnings,
    }


def build_rig(urdf_path, robot_name=None, gripper_joint_names=None, tip_link=None):
    """
    Builds the rig in the currently-open Maya scene. Returns a dict summary
    (names created, pose/tool joint counts, and any urdf_parser/build
    warnings) for the caller (UI or test harness) to report.
    """
    _require_maya()

    robot = model_parser.parse(urdf_path)
    if _is_branched(robot) and tip_link is None:
        return _build_branched_rig(robot, robot_name)

    chain = robot.kinematic_chain(tip_link=tip_link, gripper_joint_names=gripper_joint_names)
    pose_chain = [cj for cj in chain if cj.is_pose_dof]
    tool_chain = [cj for cj in chain if not cj.is_pose_dof]

    if not pose_chain:
        raise RigBuildError('No revolute/continuous pose joints found in %s' % urdf_path)
    if len(pose_chain) > 12:
        raise RigBuildError(
            '%d pose joints found; this importer supports up to 12 (see MAX_JOINTS '
            'in robotIKGeneric.py)' % len(pose_chain))

    name = robot_name or robot.name
    top = cmds.group(empty=True, name='%s_GRP#' % name)
    robot_grp = _group('robot_GRP', top)

    target_ctrl = _group('target_CTRL', robot_grp)
    local_ctrl = _group('local_CTRL', robot_grp)
    cmds.setAttr(local_ctrl + '.rotate', *WORLD_UP_CONVERSION_DEG)
    fk_ctrls_grp = _group('FK_CTRLS', robot_grp)

    robot_geom = _group('robot_GEOM', local_ctrl)
    base = _group('Base', robot_geom)
    root_origin = np.array(robot.root_origin_matrix, dtype=float)
    root_origin[:3, 3] *= METERS_TO_CM
    _set_local_matrix(base, root_origin)

    root_link = robot.root_link()
    _attach_link_meshes(robot, root_link, base, np.eye(4), 'base')

    ik_node = cmds.createNode('robotIKGeneric', name='%s_ikSolver' % name)
    cmds.setAttr(ik_node + '.numJoints', len(pose_chain))

    prev_rotating_node = base
    prev_align_inv = np.eye(4)  # A_{i-1}^-1, identity for the first joint

    axis_nodes = []
    fk_nodes = []

    for i, cj in enumerate(pose_chain):
        joint = cj.joint
        # Use the parser's exact composed matrix rather than re-deriving it
        # from Euler angles, which is lossy near gimbal lock.
        origin = _chain_origin_cm(cj)

        A = _align_z_to(joint.axis)
        A_h = _homog(A, np.zeros(3))

        compensated_origin = prev_align_inv @ origin
        frame_local = compensated_origin @ A_h

        frame_node = _group('axis%d_frame' % (i + 1), prev_rotating_node)
        _set_local_matrix(frame_node, frame_local)

        axis_node = _group('axis%d' % (i + 1), frame_node)
        axis_nodes.append(axis_node)

        fk_ctrl = _group('a%dFK_CTRL' % (i + 1), fk_ctrls_grp)
        fk_nodes.append(fk_ctrl)

        # Solver-side JointSpec must describe exactly the same geometry:
        # origin relative to the previous joint's rotating frame, rotating
        # about local Z (matches frame_local's construction above).
        cmds.setAttr('%s.jointOriginMatrix[%d]' % (ik_node, i), *_maya_matrix_flat(frame_local), type='matrix')
        cmds.setAttr('%s.jointAxis[%d]' % (ik_node, i), 0.0, 0.0, 1.0, type='double3')
        cmds.setAttr('%s.jointLimitLower[%d]' % (ik_node, i),
                      joint.limit_lower if joint.limit_lower is not None else -1e7)
        cmds.setAttr('%s.jointLimitUpper[%d]' % (ik_node, i),
                      joint.limit_upper if joint.limit_upper is not None else 1e7)

        cmds.connectAttr(fk_ctrl + '.rotateZ', '%s.fk[%d]' % (ik_node, i))
        cmds.connectAttr('%s.theta[%d]' % (ik_node, i), axis_node + '.rotateZ')

        A_inv_h = _homog(A.T, np.zeros(3))
        _attach_link_meshes(robot, joint.child, axis_node, A_inv_h, 'link%d' % (i + 1))

        prev_rotating_node = axis_node
        prev_align_inv = A_inv_h

    tcp_grp = _group('tcp_GRP', prev_rotating_node)
    tcp_hdl = _group('tcp_HDL', tcp_grp)

    # Tool/gripper joints: attached as plain FK (not part of the IK chain).
    # They still get correct static placement via the same compensation
    # math, just no connection to the solver.
    gripper_parent = prev_rotating_node
    gripper_align_inv = prev_align_inv
    gripper_ctrls = []
    for j, cj in enumerate(tool_chain):
        joint = cj.joint
        # Use the parser's exact composed matrix rather than re-deriving it
        # from Euler angles, which is lossy near gimbal lock.
        origin = _chain_origin_cm(cj)
        A = _align_z_to(joint.axis)
        A_h = _homog(A, np.zeros(3))
        compensated_origin = gripper_align_inv @ origin
        frame_local = compensated_origin @ A_h

        frame_node = _group('tool%d_frame' % (j + 1), gripper_parent)
        _set_local_matrix(frame_node, frame_local)
        tool_node = _group('tool%d_CTRL' % (j + 1), frame_node)
        gripper_ctrls.append(tool_node)
        if joint.limit_lower is not None:
            cmds.setAttr(tool_node + '.minRotZLimit', math.degrees(joint.limit_lower))
            cmds.setAttr(tool_node + '.minRotZLimitEnable', True)
        if joint.limit_upper is not None:
            cmds.setAttr(tool_node + '.maxRotZLimit', math.degrees(joint.limit_upper))
            cmds.setAttr(tool_node + '.maxRotZLimitEnable', True)

        A_inv_h = _homog(A.T, np.zeros(3))
        _attach_link_meshes(robot, joint.child, tool_node, A_inv_h, 'tool%d' % (j + 1))
        gripper_parent = tool_node
        gripper_align_inv = A_inv_h

    cmds.connectAttr(base + '.worldMatrix[0]', ik_node + '.lcsMatrix')
    cmds.connectAttr(target_ctrl + '.worldMatrix[0]', ik_node + '.targetMatrix')

    cmds.addAttr(target_ctrl, longName='robotType', dataType='string')
    cmds.setAttr(target_ctrl + '.robotType', 'urdfImport', type='string', lock=True)
    cmds.addAttr(target_ctrl, longName='robotSubtype', dataType='string')
    cmds.setAttr(target_ctrl + '.robotSubtype', name, type='string', lock=True)
    cmds.addAttr(target_ctrl, longName='numJoints', attributeType='long')
    cmds.setAttr(target_ctrl + '.numJoints', len(pose_chain), lock=True)
    cmds.addAttr(target_ctrl, longName='ik', attributeType='bool', defaultValue=True)
    cmds.connectAttr(target_ctrl + '.ik', ik_node + '.ik')

    warnings = list(robot.warnings)

    return {
        'top_node': top,
        'ik_node': ik_node,
        'ik_nodes': [ik_node],
        'target_ctrl': target_ctrl,
        'target_ctrls': [target_ctrl],
        'local_ctrl': local_ctrl,
        'axis_nodes': axis_nodes,
        'fk_nodes': fk_nodes,
        'gripper_nodes': gripper_ctrls,
        'pose_joint_count': len(pose_chain),
        'tool_joint_count': len(tool_chain),
        'source_format': robot.source_format,
        'warnings': warnings,
    }
