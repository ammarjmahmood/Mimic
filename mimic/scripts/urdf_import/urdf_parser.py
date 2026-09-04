#!usr/bin/env python
"""
Stdlib-only URDF parser. No Maya dependency, so it can be unit tested with a
plain `python3` interpreter as well as `mayapy`.

Parses <link>/<joint> elements into a flat model, then walks the tree from
the root link to produce an ordered, serial kinematic chain suitable for
feeding a numerical IK solver (see robotmath/generic_ik.py) and a Maya rig
builder (see rig_builder.py).

Only 'revolute' and 'continuous' joints are treated as chain DOFs.
'fixed' joints are baked (their child mesh is attached rigidly, no axis
created). 'prismatic'/'planar'/'floating' joints are not supported by this
importer and are reported, not silently mishandled.
"""

import math
import os
import xml.etree.ElementTree as ET

SUPPORTED_DOF_TYPES = ('revolute', 'continuous')
UNSUPPORTED_DOF_TYPES = ('prismatic', 'planar', 'floating')

# Child-link name fragments that mark a trailing joint as a tool/gripper
# actuator rather than a pose DOF. Case-insensitive substring match.
#
# Deliberately narrow: words like "gripper"/"hand"/"tool" are excluded from
# this list because they're commonly used to name the *static mounting body*
# of an end effector, which can still be attached to a legitimate pose DOF
# (e.g. the SO-ARM100's wrist-roll joint's child link is literally named
# "gripper" even though it's the 5th arm axis, not the actuator). Only
# words that name the actual moving digit/actuator are matched here; the
# ambiguous ones are left as an informational warning instead (see
# _warn_ambiguous_trailing_names below).
GRIPPER_KEYWORDS = ('jaw', 'finger', 'claw', 'knuckle', 'pincer')
AMBIGUOUS_KEYWORDS = ('gripper', 'tool', 'hand', 'end_effector', 'ee_link')


class UrdfError(Exception):
    pass


class Mesh(object):
    def __init__(self, filename, xyz, rpy, scale, origin_matrix=None):
        self.filename = filename  # absolute path, resolved against the URDF's directory
        self.xyz = xyz            # (x, y, z) meters, visual origin relative to the link frame
        self.rpy = rpy            # (r, p, y) radians
        self.scale = scale        # (sx, sy, sz)
        self.origin_matrix = (origin_matrix if origin_matrix is not None
                              else _mat4_from_xyz_rpy(xyz, rpy))


class Link(object):
    def __init__(self, name):
        self.name = name
        self.meshes = []  # list[Mesh]


class Joint(object):
    def __init__(self, name, joint_type, parent, child, xyz, rpy, axis,
                 limit_lower, limit_upper, origin_matrix=None):
        self.name = name
        self.type = joint_type
        self.parent = parent          # parent link name
        self.child = child            # child link name
        self.xyz = xyz                # (x, y, z) meters, joint origin relative to parent link frame
        self.rpy = rpy                # (r, p, y) radians
        self.axis = axis              # (x, y, z) unit vector in the joint (child-pre-rotation) frame
        self.limit_lower = limit_lower  # radians, or None
        self.limit_upper = limit_upper  # radians, or None
        self.origin_matrix = (origin_matrix if origin_matrix is not None
                              else _mat4_from_xyz_rpy(xyz, rpy))


class ChainJoint(object):
    """A joint resolved into a flattened, ordered kinematic chain."""

    def __init__(self, joint, origin_matrix, is_pose_dof):
        self.joint = joint
        # Origin of this joint relative to the *previous chain joint's* frame
        # (fixed joints along the way have already been folded in), as a 4x4
        # row-of-rows matrix in the standard column-vector convention.
        #
        # This is deliberately the matrix, not (xyz, rpy). Composing fixed
        # joints by converting back to Euler angles at each step loses
        # precision near gimbal lock (pitch ~= +/-pi/2), which real URDFs hit
        # constantly -- the SO-ARM101's joint 2 has pitch = -1.57079, where
        # cos(pitch) is 3.7e-06. Round-tripping there perturbs the rotation
        # by ~1e-11 per step, which a chain of fixed joints would compound.
        # Consumers that want a readable xyz/rpy can use the derived
        # properties below; nothing in the maths path does.
        self.origin_matrix = origin_matrix
        self.is_pose_dof = is_pose_dof  # False => tool/gripper joint, excluded from IK

    @property
    def origin_xyz(self):
        return (self.origin_matrix[0][3], self.origin_matrix[1][3], self.origin_matrix[2][3])

    @property
    def origin_rpy(self):
        """Euler form, for display/reporting only -- see origin_matrix."""
        return _matrix_to_rpy([row[:3] for row in self.origin_matrix[:3]])


class Robot(object):
    def __init__(self, name, links, joints, urdf_path, root_origin_matrix=None,
                 source_format='urdf'):
        self.name = name
        self.links = links      # dict[name -> Link]
        self.joints = joints    # dict[name -> Joint]
        self.urdf_path = urdf_path
        self.root_origin_matrix = root_origin_matrix or _identity4()
        self.source_format = source_format
        self.warnings = []

    def children_of(self, link_name):
        return [j for j in self.joints.values() if j.parent == link_name]

    def root_link(self):
        child_links = set(j.child for j in self.joints.values())
        roots = [name for name in self.links if name not in child_links]
        if not roots:
            raise UrdfError('URDF has no root link (every link is some joint\'s child)')
        if len(roots) > 1:
            self.warnings.append(
                'URDF has multiple disconnected root links (%s); using "%s"'
                % (roots, roots[0]))
        return roots[0]

    def longest_chain(self):
        """
        Returns the longest root-to-leaf path of joints (list[Joint]),
        following only SUPPORTED_DOF_TYPES and 'fixed' joints. This is the
        heuristic used to pick the "main arm" out of a tree that may also
        contain camera mounts, gripper fingers, etc.
        """
        root = self.root_link()

        best = []

        def dfs(link_name, path):
            nonlocal best
            children = self.children_of(link_name)
            if not children:
                if len(path) > len(best):
                    best = list(path)
                return
            for j in children:
                if j.type in SUPPORTED_DOF_TYPES or j.type == 'fixed':
                    dfs(j.child, path + [j])
                elif j.type in UNSUPPORTED_DOF_TYPES:
                    self.warnings.append(
                        'Joint "%s" has unsupported type "%s" and its subtree is skipped'
                        % (j.name, j.type))
                else:
                    self.warnings.append(
                        'Joint "%s" has unrecognized type "%s" and its subtree is skipped'
                        % (j.name, j.type))
            if not any(j.type in SUPPORTED_DOF_TYPES or j.type == 'fixed' for j in children):
                if len(path) > len(best):
                    best = list(path)

        dfs(root, [])
        if not best:
            raise UrdfError('Could not find any revolute/continuous joint chain from root "%s"' % root)

        # This importer intentionally solves one serial chain. Make that
        # limitation explicit for branched robots (humanoids, quadrupeds,
        # dual-arm systems) instead of silently building only the longest
        # limb and giving the impression that the whole robot was imported.
        selected_names = set(j.name for j in best)
        selected_parent_links = set([root] + [j.child for j in best])
        ignored_branch_roots = [
            j.name for j in self.joints.values()
            if j.parent in selected_parent_links
            and j.name not in selected_names
            and (j.type in SUPPORTED_DOF_TYPES or j.type == 'fixed')
        ]
        if ignored_branch_roots:
            self.warnings.append(
                'Robot description is branched; this importer selected one longest serial chain and '
                'ignored branch root joint(s): %s'
                % ', '.join(ignored_branch_roots))
        return best

    def kinematic_chain(self, tip_link=None, gripper_joint_names=None):
        """
        Builds the ordered ChainJoint list used by the rig builder / IK solver.

        Fixed joints are folded into the *next* dof joint's origin (their
        translation/rotation compounds rather than producing their own
        axis), so the returned chain contains exactly one ChainJoint per
        SUPPORTED_DOF_TYPES joint.

        Trailing joints whose child link name matches GRIPPER_KEYWORDS (or
        is explicitly listed in gripper_joint_names) are marked
        is_pose_dof=False and excluded from the IK chain, but are still
        returned so the rig builder can attach them as plain-FK tool
        controls.
        """
        raw_chain = self.longest_chain()
        if tip_link is not None:
            # Truncate the longest-chain path at the requested tip link.
            truncated = []
            for j in raw_chain:
                truncated.append(j)
                if j.child == tip_link:
                    break
            else:
                raise UrdfError('tip_link "%s" not found on the longest chain' % tip_link)
            raw_chain = truncated

        gripper_joint_names = set(gripper_joint_names or [])

        chain = []
        pending = _identity4()

        for j in raw_chain:
            compound = _mat4_mul(pending, j.origin_matrix)
            if j.type == 'fixed':
                pending = compound
                continue
            chain.append(ChainJoint(j, compound, is_pose_dof=True))
            pending = _identity4()

        if pending != _identity4():
            self.warnings.append(
                'Trailing fixed joint(s) after the last DOF have no axis to attach to; '
                'their offset is folded into the tip transform by the rig builder.')

        # Classify trailing joints as tool/gripper.
        i = len(chain) - 1
        while i >= 0:
            cj = chain[i]
            child = cj.joint.child.lower()
            name = cj.joint.name
            is_gripper = name in gripper_joint_names or any(k in child for k in GRIPPER_KEYWORDS)
            if is_gripper:
                cj.is_pose_dof = False
                i -= 1
            else:
                if cj.is_pose_dof and any(k in child for k in AMBIGUOUS_KEYWORDS):
                    self.warnings.append(
                        'Joint "%s" (child link "%s") looks end-effector-ish by name but was '
                        'kept as a pose DOF; pass it in gripper_joint_names if it should '
                        'actually be excluded from IK.' % (name, cj.joint.child))
                break

        return chain


def _rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # URDF: R = Rz(y) * Ry(p) * Rx(r)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_to_rpy(m):
    sp = -m[2][0]
    sp = max(-1.0, min(1.0, sp))
    p = math.asin(sp)
    cp = math.cos(p)
    if abs(cp) > 1e-8:
        r = math.atan2(m[2][1], m[2][2])
        y = math.atan2(m[1][0], m[0][0])
    else:
        # Gimbal lock; fall back to yaw = 0.
        r = math.atan2(-m[1][2], m[1][1])
        y = 0.0
    return (r, p, y)


def _mat_vec(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _mat_mat(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _identity4():
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def _mat4_from_xyz_rpy(xyz, rpy):
    r = _rpy_to_matrix(rpy)
    return [[r[0][0], r[0][1], r[0][2], xyz[0]],
            [r[1][0], r[1][1], r[1][2], xyz[1]],
            [r[2][0], r[2][1], r[2][2], xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _mat4_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _compose(xyz1, rpy1, xyz2, rpy2):
    """
    Composes two (xyz, rpy) transforms: result = T1 * T2, returned as
    (xyz, rpy).

    Retained for callers that genuinely want Euler output. The chain builder
    does NOT use this -- it composes 4x4 matrices directly, because the
    matrix->rpy step here is lossy near gimbal lock (see ChainJoint).
    """
    m = _mat4_mul(_mat4_from_xyz_rpy(xyz1, rpy1), _mat4_from_xyz_rpy(xyz2, rpy2))
    out_xyz = (m[0][3], m[1][3], m[2][3])
    out_rpy = _matrix_to_rpy([row[:3] for row in m[:3]])
    return out_xyz, out_rpy


def _parse_xyz(elem, attr='xyz'):
    if elem is None:
        return (0.0, 0.0, 0.0)
    s = elem.get(attr)
    if not s:
        return (0.0, 0.0, 0.0)
    parts = [float(x) for x in s.split()]
    return (parts[0], parts[1], parts[2])


def _parse_origin(origin_elem):
    xyz = _parse_xyz(origin_elem, 'xyz')
    rpy = _parse_xyz(origin_elem, 'rpy')
    return xyz, rpy


def parse(urdf_path):
    """
    Parses a URDF file at `urdf_path` into a Robot. Mesh filenames are
    resolved to absolute paths relative to the URDF's own directory
    (package:// URIs are stripped of their scheme and treated the same way,
    since most hobby-robot URDF exports keep meshes alongside the URDF).
    """
    if not os.path.isfile(urdf_path):
        raise UrdfError('URDF file not found: %s' % urdf_path)

    base_dir = os.path.dirname(os.path.abspath(urdf_path))
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    if root.tag != 'robot':
        raise UrdfError('Not a URDF file (root element is <%s>, expected <robot>)' % root.tag)

    name = root.get('name', os.path.splitext(os.path.basename(urdf_path))[0])

    links = {}
    for link_elem in root.findall('link'):
        link = Link(link_elem.get('name'))
        for visual_elem in link_elem.findall('visual'):
            geom = visual_elem.find('geometry')
            if geom is None:
                continue
            mesh_elem = geom.find('mesh')
            if mesh_elem is None:
                continue
            filename = mesh_elem.get('filename', '')
            resolved = _resolve_mesh_path(filename, base_dir)
            xyz, rpy = _parse_origin(visual_elem.find('origin'))
            scale_str = mesh_elem.get('scale')
            scale = tuple(float(x) for x in scale_str.split()) if scale_str else (1.0, 1.0, 1.0)
            link.meshes.append(Mesh(resolved, xyz, rpy, scale))
        links[link.name] = link

    joints = {}
    for joint_elem in root.findall('joint'):
        jname = joint_elem.get('name')
        jtype = joint_elem.get('type')
        parent = joint_elem.find('parent').get('link')
        child = joint_elem.find('child').get('link')
        xyz, rpy = _parse_origin(joint_elem.find('origin'))
        axis_elem = joint_elem.find('axis')
        axis = _parse_xyz(axis_elem) if axis_elem is not None else (1.0, 0.0, 0.0)
        limit_elem = joint_elem.find('limit')
        lower = float(limit_elem.get('lower')) if limit_elem is not None and limit_elem.get('lower') else None
        upper = float(limit_elem.get('upper')) if limit_elem is not None and limit_elem.get('upper') else None
        joints[jname] = Joint(jname, jtype, parent, child, xyz, rpy, axis, lower, upper)

    for link_name in list(links.keys()):
        pass  # links dict already keyed by name; nothing further to validate here

    referenced_links = set()
    for j in joints.values():
        referenced_links.add(j.parent)
        referenced_links.add(j.child)
    for lname in referenced_links:
        if lname not in links:
            links[lname] = Link(lname)  # tolerate links with no <link> element / no geometry

    return Robot(name, links, joints, os.path.abspath(urdf_path))


def _resolve_mesh_path(filename, base_dir):
    if filename.startswith('package://'):
        # Strip the ROS package scheme; assume assets ship next to the URDF
        # (true for the SO-ARM100/101 exports and most hobby-robot URDFs).
        stripped = filename[len('package://'):]
        stripped = stripped.split('/', 1)[1] if '/' in stripped else stripped
        return os.path.normpath(os.path.join(base_dir, stripped))
    if os.path.isabs(filename):
        return filename
    return os.path.normpath(os.path.join(base_dir, filename))
