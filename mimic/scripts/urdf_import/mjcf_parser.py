#!/usr/bin/env python
"""Pure-stdlib MJCF reader that adapts MuJoCo bodies to the URDF rig model.

The Maya builder consumes the small Robot/Link/Joint/Mesh model defined by
``urdf_parser``.  This module translates the MJCF subset used by exported
robot descriptions (nested bodies, hinge/slide joints and mesh geoms) into
that model.  Simulation-only data such as actuators, contacts and inertials
is intentionally ignored.
"""

import math
import os
import xml.etree.ElementTree as ET

import urdf_parser


class MjcfError(urdf_parser.UrdfError):
    pass


def _numbers(value, count, default):
    if not value:
        return tuple(default)
    values = tuple(float(v) for v in value.split())
    if len(values) != count:
        raise MjcfError('Expected %d numbers, got "%s"' % (count, value))
    return values


def _translation(xyz):
    return [[1.0, 0.0, 0.0, xyz[0]],
            [0.0, 1.0, 0.0, xyz[1]],
            [0.0, 0.0, 1.0, xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _quat_matrix(quat):
    # MJCF quaternions are scalar-first: w x y z.
    w, x, y, z = quat
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-15:
        raise MjcfError('Zero-length quaternion')
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    return [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]


def _axis_angle_matrix(values, angle_scale):
    x, y, z, angle = values
    length = math.sqrt(x*x + y*y + z*z)
    if length < 1e-15:
        raise MjcfError('Zero-length axis in axisangle')
    x, y, z = x/length, y/length, z/length
    angle *= angle_scale
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [t*x*x + c, t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c, t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c],
    ]


def _pose_matrix(elem, angle_scale):
    pos = _numbers(elem.get('pos'), 3, (0.0, 0.0, 0.0))
    if elem.get('quat'):
        rotation = _quat_matrix(_numbers(elem.get('quat'), 4, (1.0, 0.0, 0.0, 0.0)))
    elif elem.get('axisangle'):
        rotation = _axis_angle_matrix(
            _numbers(elem.get('axisangle'), 4, (0.0, 0.0, 1.0, 0.0)),
            angle_scale)
    elif elem.get('euler'):
        euler = tuple(v * angle_scale for v in
                      _numbers(elem.get('euler'), 3, (0.0, 0.0, 0.0)))
        rotation = urdf_parser._rpy_to_matrix(euler)
    else:
        rotation = urdf_parser._rpy_to_matrix((0.0, 0.0, 0.0))
    matrix = [list(row) + [pos[i]] for i, row in enumerate(rotation)]
    matrix.append([0.0, 0.0, 0.0, 1.0])
    return matrix


def _xyz_rpy(matrix):
    xyz = tuple(matrix[i][3] for i in range(3))
    rpy = urdf_parser._matrix_to_rpy([row[:3] for row in matrix[:3]])
    return xyz, rpy


def parse(mjcf_path):
    if not os.path.isfile(mjcf_path):
        raise MjcfError('MJCF file not found: %s' % mjcf_path)

    path = os.path.abspath(mjcf_path)
    base_dir = os.path.dirname(path)
    root = ET.parse(path).getroot()
    if root.tag != 'mujoco':
        raise MjcfError('Not an MJCF file (root element is <%s>, expected <mujoco>)'
                        % root.tag)

    compiler = root.find('compiler')
    angle_name = compiler.get('angle', 'degree') if compiler is not None else 'degree'
    angle_scale = math.pi / 180.0 if angle_name == 'degree' else 1.0
    if angle_name not in ('degree', 'radian'):
        raise MjcfError('Unsupported MJCF compiler angle "%s"' % angle_name)
    if compiler is not None and compiler.get('coordinate', 'local') != 'local':
        raise MjcfError('MJCF global coordinates are not supported; use local coordinates')
    mesh_dir = compiler.get('meshdir', '') if compiler is not None else ''

    assets = {}
    for mesh in root.findall('./asset/mesh'):
        filename = mesh.get('file')
        if not filename:
            continue
        name = mesh.get('name') or os.path.splitext(os.path.basename(filename))[0]
        resolved = filename if os.path.isabs(filename) else os.path.join(base_dir, mesh_dir, filename)
        scale = _numbers(mesh.get('scale'), 3, (1.0, 1.0, 1.0))
        assets[name] = (os.path.normpath(resolved), scale)

    worldbody = root.find('worldbody')
    root_bodies = worldbody.findall('body') if worldbody is not None else []
    if not root_bodies:
        raise MjcfError('MJCF has no <worldbody><body> robot body')

    links = {}
    joints = {}
    warnings = []
    body_counter = [0]

    def body_name(elem):
        name = elem.get('name')
        if name:
            return name
        body_counter[0] += 1
        return 'mjcf_body_%d' % body_counter[0]

    def add_meshes(body_elem, link, link_to_body):
        for index, geom in enumerate(body_elem.findall('geom')):
            mesh_name = geom.get('mesh')
            if not mesh_name:
                continue
            # Exported MJCFs generally emit paired visual/collision geoms.
            # Import only explicit visual geometry to avoid duplicates.
            geom_class = geom.get('class', '')
            if geom_class and geom_class != 'visual':
                continue
            if mesh_name not in assets:
                warnings.append('MJCF geom references unknown mesh asset "%s"' % mesh_name)
                continue
            filename, asset_scale = assets[mesh_name]
            local = urdf_parser._mat4_mul(link_to_body, _pose_matrix(geom, angle_scale))
            xyz, rpy = _xyz_rpy(local)
            link.meshes.append(
                urdf_parser.Mesh(filename, xyz, rpy, asset_scale, origin_matrix=local))

    def visit(body_elem, parent_link, parent_link_to_body, is_root=False):
        name = body_name(body_elem)
        if name in links:
            raise MjcfError('Duplicate MJCF body name "%s"' % name)

        body_pose = _pose_matrix(body_elem, angle_scale)
        joint_elems = body_elem.findall('joint')
        free_elems = body_elem.findall('freejoint')
        if len(joint_elems) > 1:
            raise MjcfError(
                'Body "%s" has %d joints; multiple joints per body are not supported'
                % (name, len(joint_elems)))

        link_to_body = urdf_parser._identity4()
        if is_root:
            root_pose = body_pose
            if free_elems:
                warnings.append(
                    'MJCF freejoint "%s" is represented by the host root control'
                    % (free_elems[0].get('name') or name))
        else:
            if free_elems:
                raise MjcfError('Non-root freejoint on body "%s" is not supported' % name)

            joint_elem = joint_elems[0] if joint_elems else None
            joint_pos = (_numbers(joint_elem.get('pos'), 3, (0.0, 0.0, 0.0))
                         if joint_elem is not None else (0.0, 0.0, 0.0))
            parent_to_joint = urdf_parser._mat4_mul(
                urdf_parser._mat4_mul(parent_link_to_body, body_pose),
                _translation(joint_pos))
            link_to_body = _translation(tuple(-v for v in joint_pos))
            xyz, rpy = _xyz_rpy(parent_to_joint)

            if joint_elem is None:
                joint_type = 'fixed'
                axis = (0.0, 0.0, 1.0)
                lower = upper = None
                joint_name = name + '_fixed'
            else:
                mjcf_type = joint_elem.get('type', 'hinge')
                joint_type = {'hinge': 'revolute', 'slide': 'prismatic',
                              'ball': 'floating'}.get(mjcf_type, mjcf_type)
                axis = _numbers(joint_elem.get('axis'), 3, (0.0, 0.0, 1.0))
                joint_name = joint_elem.get('name') or name + '_joint'
                limits = joint_elem.get('range')
                if limits and joint_type in ('revolute', 'continuous'):
                    lower, upper = _numbers(limits, 2, (0.0, 0.0))
                    lower *= angle_scale
                    upper *= angle_scale
                elif limits:
                    lower, upper = _numbers(limits, 2, (0.0, 0.0))
                else:
                    lower = upper = None
                    if joint_type == 'revolute':
                        joint_type = 'continuous'

            if joint_name in joints:
                raise MjcfError('Duplicate MJCF joint name "%s"' % joint_name)
            joints[joint_name] = urdf_parser.Joint(
                joint_name, joint_type, parent_link, name, xyz, rpy, axis, lower, upper,
                origin_matrix=parent_to_joint)
            root_pose = None

        link = urdf_parser.Link(name)
        links[name] = link
        add_meshes(body_elem, link, link_to_body)
        for child in body_elem.findall('body'):
            visit(child, name, link_to_body, is_root=False)
        return name, root_pose

    if len(root_bodies) == 1:
        root_name, root_origin = visit(
            root_bodies[0], None, urdf_parser._identity4(), is_root=True)
    else:
        # Preserve disconnected MJCF roots under a synthetic fixed root.
        root_name = '__mjcf_world__'
        links[root_name] = urdf_parser.Link(root_name)
        root_origin = urdf_parser._identity4()
        for body in root_bodies:
            visit(body, root_name, urdf_parser._identity4(), is_root=False)
        warnings.append('MJCF has multiple root bodies; attached them under a synthetic root')

    robot = urdf_parser.Robot(
        root.get('model', os.path.splitext(os.path.basename(path))[0]),
        links, joints, path, root_origin_matrix=root_origin, source_format='mjcf')
    robot.warnings.extend(warnings)

    # Sanity-check that our selected root agrees with the generic tree model.
    if robot.root_link() != root_name:
        raise MjcfError('Internal MJCF tree conversion produced the wrong root link')
    return robot

