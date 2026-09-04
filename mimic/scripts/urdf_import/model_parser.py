#!/usr/bin/env python
"""Dispatch URDF and MJCF files to a shared robot-model representation."""

import os
import xml.etree.ElementTree as ET

import mjcf_parser
import urdf_parser


def parse(path):
    if not os.path.isfile(path):
        raise urdf_parser.UrdfError('Robot description file not found: %s' % path)
    root_tag = ET.parse(path).getroot().tag
    if root_tag == 'robot':
        return urdf_parser.parse(path)
    if root_tag == 'mujoco':
        return mjcf_parser.parse(path)
    raise urdf_parser.UrdfError(
        'Unsupported robot description root <%s>; expected URDF <robot> or MJCF <mujoco>'
        % root_tag)

