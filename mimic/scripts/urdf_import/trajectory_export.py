#!usr/bin/env python
"""
Exports a keyframed rig animation (Maya or Blender) to a small, DCC-agnostic
JSON trajectory format that mimic/scripts/hardware/ consumes to drive real
hardware -- either directly over Feetech servos or via a ROS2
JointTrajectory publish. See mimic/scripts/hardware/README.md.

This is deliberately the seam between "animate" and "drive hardware": the
DCC side (this file) only ever needs to know how to read keyframes off the
rig it built; the hardware side only ever needs to read this JSON. Neither
needs to know about the other, which is what lets one JSON drive either a
Feetech-servo arm or a ROS-controlled one from the same recorded animation.

Waypoint timing: only the UNION of times where *something* on the rig is
actually keyframed is exported (not every frame), sampled from each
control's own curve at those times. That keeps a hand-posed "record a
waypoint every few seconds" animation compact, while still exporting a
correct pose at every frame for a densely-keyframed (e.g. baked mocap-style)
animation, since in that case every frame IS a keyframe time.

Joint ordering: waypoints list positions in exactly rig['axis_objs'] order
(Blender) / axis_nodes order (Maya) -- i.e. the order the rig builder
walked the URDF tree in. mimic/scripts/hardware/servo_maps/*.json map into
this same order by "joint_index", not by name, because URDF joint names are
frequently meaningless (SO-ARM100/101's are literally "1".."6") while walk
order is always well-defined and stable for a given URDF.
"""
import json
import math
import os


class TrajectoryExportError(Exception):
    pass


def _times_from_frames(sorted_frames, fps):
    """
    Converts an ordered list of keyframe frame numbers into seconds,
    normalized so the FIRST keyframe is t=0 -- if an animator's first
    waypoint happens to sit at frame 50 rather than frame 1, playback
    shouldn't include a 50-frame dead pause before anything moves.
    Shared by both host exporters so they can't disagree on this again
    (an earlier version of this file normalized in Maya but not Blender).
    """
    origin = sorted_frames[0]
    return [(frame - origin) / fps for frame in sorted_frames]


def _write_json(path, joint_names, fps, waypoints, source_format, robot_name):
    doc = {
        'schema': 'mimic.trajectory.v1',
        'robot_name': robot_name,
        'source_format': source_format,
        'joint_names': joint_names,
        'fps': fps,
        'waypoints': [
            {'time': round(t, 6), 'positions_deg': [round(p, 4) for p in positions]}
            for t, positions in waypoints
        ],
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(doc, f, indent=1)
    return doc


# --------------------------------------------------------------------------
# Blender
# --------------------------------------------------------------------------

def export_from_blender(rig, out_path, robot_name=None, fps=None):
    """
    Reads keyframes off rig['axis_objs'] (every joint the rig builder
    created, branched or not -- FK-only joints included, since they're
    still part of "the animation" even if no IK solver drives them) and
    writes a trajectory JSON.

    Call this any time after you've keyframed axis rotations (Blender's own
    I / auto-key on the axis or target_CTRL objects -- see
    blender_rig_builder.solve_chain(), which is exactly what you're keying
    when you insert a keyframe after dragging a target_CTRL).
    """
    import bpy

    # Combined in the same order servo_maps/*.json's joint_index assumes:
    # IK/FK pose joints first, then tool/gripper joints. For a branched rig
    # tool_objs is always [] (everything already lives in axis_objs -- see
    # _build_branched_rig), so this is a no-op there.
    axis_objs = list(rig['axis_objs']) + list(rig.get('tool_objs', []))
    if not axis_objs:
        raise TrajectoryExportError('rig has no axis objects to export')

    joint_names = ['joint%d' % i for i in range(len(axis_objs))]
    fps = fps or bpy.context.scene.render.fps

    times = set()
    curves_by_index = {}
    for i, obj in enumerate(axis_objs):
        if obj.animation_data is None or obj.animation_data.action is None:
            continue
        # Blender 4.2+ "layered actions": fcurves live under
        # action.layers[].strips[].channelbags[], not directly on the
        # action. fcurve_ensure_for_datablock() is the supported way to
        # reach the right one without hand-walking that structure -- it
        # returns the existing curve when one is already there (as it will
        # be, since we're reading an already-keyframed object).
        fcurve = obj.animation_data.action.fcurve_ensure_for_datablock(
            obj, 'rotation_euler', index=2)
        if fcurve is None or not fcurve.keyframe_points:
            continue
        curves_by_index[i] = fcurve
        for kp in fcurve.keyframe_points:
            times.add(kp.co.x)  # frame number

    if not times:
        raise TrajectoryExportError(
            'No keyframes found on any axis object rotation_euler.z. '
            'Insert a keyframe (I) after each pose you want recorded as a waypoint.')

    sorted_frames = sorted(times)
    wp_times = _times_from_frames(sorted_frames, fps)
    waypoints = []
    for frame, t in zip(sorted_frames, wp_times):
        positions = []
        for i, obj in enumerate(axis_objs):
            fcurve = curves_by_index.get(i)
            if fcurve is not None:
                positions.append(math.degrees(fcurve.evaluate(frame)))
            else:
                positions.append(math.degrees(obj.rotation_euler.z))
        waypoints.append((t, positions))

    return _write_json(out_path, joint_names, fps, waypoints, 'blender', robot_name or rig.get('name', 'robot'))


# --------------------------------------------------------------------------
# Maya
# --------------------------------------------------------------------------

def export_from_maya(rig, out_path, robot_name=None, fps=None):
    """
    Reads keyframes off rig['axis_nodes'] (rig_builder.build_rig()'s
    return dict) and writes a trajectory JSON. Reads whichever of
    axis{i}.rotateZ has keys; for a branched rig this is every joint in
    rig['axis_nodes'], IK-driven or FK-only alike.

    For usability, keyframe times are gathered from the union of the
    driven axis nodes AND the FK controls / target controls that drive
    them.  A natural Maya workflow is to key the FK sliders or the
    target_CTRL itself (drag then S) rather than keying the driven axis
    nodes directly -- even though Maya technically allows keying a driven
    channel via pairBlend, export should still succeed when the user keys
    the control they actually interacted with.  The exported *values* are
    always sampled from the axis nodes (the authoritative joint angles
    after IK/FK evaluation), so the timeline being defined by FK/target
    keys does not change what is recorded.
    """
    import maya.cmds as cmds

    # Same ordering rationale as export_from_blender(): pose joints, then
    # gripper/tool joints, matching servo_maps/*.json's joint_index.
    axis_nodes = list(rig['axis_nodes']) + list(rig.get('gripper_nodes', []))
    if not axis_nodes:
        raise TrajectoryExportError('rig has no axis nodes to export')

    joint_names = ['joint%d' % i for i in range(len(axis_nodes))]
    fps = fps or _maya_fps()

    times = set()
    # Primary: keys on the driven axis nodes themselves (e.g. after baking,
    # or if the user explicitly keyed them with S while they were selected).
    for node in axis_nodes:
        attr = node + '.rotateZ'
        if cmds.keyframe(attr, query=True, keyframeCount=True) or 0:
            for t in cmds.keyframe(attr, query=True, timeChange=True):
                times.add(t)

    # Fallback / union: also consider FK controls and target controls as
    # keyframe sources.  Sampling still reads axis_nodes at those times, so
    # the IK/FK-driven values are captured correctly regardless of where the
    # user placed keys.
    if not times:
        extra_nodes = list(rig.get('fk_nodes', [])) + list(rig.get('target_ctrls', []))
        # also consider the single-target alias for pre-branched rigs
        if rig.get('target_ctrl') and rig['target_ctrl'] not in extra_nodes:
            extra_nodes.append(rig['target_ctrl'])
        for node in extra_nodes:
            for suffix in ('.rotateZ', '.rotateX', '.rotateY',
                           '.translateX', '.translateY', '.translateZ'):
                attr = node + suffix
                try:
                    if cmds.keyframe(attr, query=True, keyframeCount=True) or 0:
                        for t in cmds.keyframe(attr, query=True, timeChange=True):
                            times.add(t)
                except RuntimeError:
                    continue

    if not times:
        raise TrajectoryExportError(
            'No keyframes found on any axis node rotateZ (or FK/target controls). '
            'Insert a keyframe (S, or right-click > Key Selected) after each pose '
            'you want recorded as a waypoint.')

    sorted_frames = sorted(times)
    wp_times = _times_from_frames(sorted_frames, fps)
    waypoints = []
    for frame, t in zip(sorted_frames, wp_times):
        positions = [cmds.getAttr(node + '.rotateZ', time=frame) for node in axis_nodes]
        waypoints.append((t, positions))

    return _write_json(out_path, joint_names, fps, waypoints, 'maya', robot_name or rig.get('name', 'robot'))


def _maya_fps():
    """
    cmds.currentUnit(query=True, time=True) returns either a named unit
    (game/film/pal/ntsc/show/palf/ntscf) or a literal "<number>fps" string
    for anything else Maya's Preferences > Settings > Time lets you pick
    (23.976fps, 29.97fps, 50fps, a fully custom rate, ...). Only handling
    the named table and silently defaulting anything else to 24 would
    export correct-looking-but-wrong timing for every one of those numeric
    rates except the one that happens to already be 24.
    """
    import re
    import maya.cmds as cmds
    unit_to_fps = {
        'game': 15, 'film': 24, 'pal': 25, 'ntsc': 30, 'show': 48,
        'palf': 50, 'ntscf': 60,
    }
    unit = cmds.currentUnit(query=True, time=True)
    if unit in unit_to_fps:
        return unit_to_fps[unit]
    m = re.match(r'^([\d.]+)fps$', unit)
    if m:
        return float(m.group(1))
    raise TrajectoryExportError(
        'Unrecognized Maya time unit %r; pass fps= explicitly to export_from_maya().' % unit)
