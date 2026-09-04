#!usr/bin/env python
"""
Blender add-on: import any URDF/MJCF robot, drag to pose with live IK,
record waypoints, export a trajectory -- no Python console, no typing
commands. Everything below is a button in a sidebar panel.

INSTALL (one time):
  1. Blender > Edit > Preferences > Add-ons > Install...
     Select this file (mimic_robot_importer_addon.py).
  2. Enable the checkbox next to "Import: Mimic Robot Importer".
  3. Still in Preferences, expand this add-on and set "Repo Path" to the
     folder you cloned this repo into (the one containing a "mimic" folder).
     That's the only setup step; everything else is buttons.

USE:
  In the 3D Viewport, open the sidebar (press N) -- there's a "Robot
  Import" tab. Browse to a .urdf or .xml (MJCF) file, click Build Rig.
  Drag any target handle in the viewport (orange arrows) to pose that
  limb; the arm/leg/etc follows live. Click "Record Waypoint" to keyframe
  the current pose, move the timeline forward, pose again, repeat. Click
  "Export Trajectory..." to save a JSON you can play back on real
  hardware (see mimic/scripts/hardware/README.md).
"""

bl_info = {
    'name': 'Mimic Robot Importer',
    'author': 'Mimic URDF/MJCF import project',
    'version': (1, 0, 0),
    'blender': (4, 2, 0),
    'location': 'View3D > Sidebar > Robot Import',
    'description': 'Import any URDF/MJCF robot, pose it with live IK by dragging, '
                    'record a trajectory, export it for real hardware.',
    'category': 'Import-Export',
}

import os
import sys
import traceback

import bpy
from bpy.props import StringProperty, IntProperty, PointerProperty
from bpy.types import Operator, Panel, PropertyGroup, AddonPreferences


ADDON_ID = __name__

_rig = None  # the currently-built rig dict (blender_rig_builder.build_rig()'s return value)


# --------------------------------------------------------------------------
# Wiring to the actual importer code, which lives in this repo's
# mimic/scripts tree, not inside the add-on itself -- see the Preferences'
# repo_path. Kept as a separate step (not a plain top-of-file import) since
# the add-on has to work before that path is known, and needs to tolerate
# a not-yet-configured or since-moved repo_path with a clear message
# instead of a bare traceback.
# --------------------------------------------------------------------------

def _get_prefs():
    return bpy.context.preferences.addons[ADDON_ID].preferences


class ImportError_(Exception):
    pass


def _load_backend():
    prefs = _get_prefs()
    repo = prefs.repo_path.strip()
    if not repo:
        raise ImportError_(
            'Set "Repo Path" in Edit > Preferences > Add-ons > Mimic Robot Importer '
            'to the folder you cloned this repo into (the one containing a "mimic" folder).')
    scripts_dir = os.path.join(repo, 'mimic', 'scripts')
    urdf_import_dir = os.path.join(scripts_dir, 'urdf_import')
    if not os.path.isdir(urdf_import_dir):
        raise ImportError_(
            'No mimic/scripts/urdf_import found under Repo Path %r -- check it points at '
            'the repo root (the folder containing "mimic", not "mimic" itself).' % repo)
    for p in (scripts_dir, urdf_import_dir):
        if p not in sys.path:
            sys.path.insert(0, p)

    import importlib
    import blender_rig_builder as brb
    import trajectory_export as te
    importlib.reload(brb)  # picks up local edits during development; harmless otherwise
    importlib.reload(te)
    return brb, te


# --------------------------------------------------------------------------
# Operators
# --------------------------------------------------------------------------

class MIMIC_OT_build_rig(Operator):
    bl_idname = 'mimic.build_rig'
    bl_label = 'Build Rig'
    bl_description = 'Parse the selected URDF/MJCF file and build an animatable rig from it'

    def execute(self, context):
        global _rig
        props = context.scene.mimic_importer

        try:
            brb, _te = _load_backend()
        except ImportError_ as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        path = bpy.path.abspath(props.robot_path).strip()
        if not path or not os.path.isfile(path):
            self.report({'ERROR'}, 'Choose a valid .urdf or .xml (MJCF) file first.')
            return {'CANCELLED'}

        try:
            rig = brb.build_rig(path, robot_name=(props.robot_name.strip() or None))
        except brb.RigBuildError as exc:
            self.report({'ERROR'}, 'Build failed: %s' % exc)
            return {'CANCELLED'}
        except Exception:
            traceback.print_exc()
            self.report({'ERROR'}, 'Build failed -- see System Console for the full error.')
            return {'CANCELLED'}

        brb.install_live_ik(rig)
        _rig = rig

        bpy.ops.object.select_all(action='DESELECT')
        for chain in rig['chains']:
            chain['target'].select_set(True)
        if rig['chains']:
            context.view_layer.objects.active = rig['chains'][0]['target']

        msg = '%d pose joint(s), %d independent target(s) built.' % (
            rig['pose_joint_count'], len(rig['chains']))
        if rig['warnings']:
            for w in rig['warnings']:
                self.report({'WARNING'}, w)
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class MIMIC_OT_select_target(Operator):
    bl_idname = 'mimic.select_target'
    bl_label = 'Select'
    bl_description = 'Select and frame this limb\'s target handle so you can drag it'
    chain_index: IntProperty()

    def execute(self, context):
        if not _rig or self.chain_index >= len(_rig['chains']):
            self.report({'ERROR'}, 'No rig built yet.')
            return {'CANCELLED'}
        target = _rig['chains'][self.chain_index]['target']
        bpy.ops.object.select_all(action='DESELECT')
        target.select_set(True)
        context.view_layer.objects.active = target

        # Framing the view is a convenience, not the point of this button --
        # selection above must succeed regardless of whether a 3D viewport
        # happens to be available to frame in (it isn't in --background
        # mode, and might not be in an unusual workspace layout either).
        for area in context.screen.areas if context.screen else []:
            if area.type == 'VIEW_3D':
                try:
                    with context.temp_override(area=area):
                        bpy.ops.view3d.view_selected()
                except RuntimeError:
                    pass
                break
        return {'FINISHED'}


class MIMIC_OT_record_waypoint(Operator):
    bl_idname = 'mimic.record_waypoint'
    bl_label = 'Record Waypoint'
    bl_description = ('Keyframe every joint at the current pose (the one-click version of '
                       'pressing I on every joint) -- this is what "recording a waypoint" is')

    def execute(self, context):
        if not _rig:
            self.report({'ERROR'}, 'No rig built yet.')
            return {'CANCELLED'}
        try:
            _brb, _te = _load_backend()
        except ImportError_ as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        import blender_rig_builder as brb
        try:
            frame = brb.record_waypoint(_rig)
        except Exception:
            traceback.print_exc()
            self.report({'ERROR'}, 'Could not solve the current pose -- see System Console for details.')
            return {'CANCELLED'}
        step = context.scene.mimic_importer.waypoint_step
        if step > 0:
            context.scene.frame_set(frame + step)
        self.report({'INFO'}, 'Waypoint recorded at frame %d.' % frame)
        return {'FINISHED'}


class MIMIC_OT_export_trajectory(Operator):
    bl_idname = 'mimic.export_trajectory'
    bl_label = 'Export Trajectory...'
    bl_description = ('Save every recorded waypoint to a JSON file, ready for '
                       'mimic/scripts/hardware/feetech_playback.py or ros_trajectory_export.py')

    filepath: StringProperty(subtype='FILE_PATH', default='trajectory.json')

    def invoke(self, context, event):
        # A name tied to the actual robot (so_arm_trajectory.json rather
        # than a generic trajectory.json) survives a rushed click into a
        # bookmark or folder mid-dialog -- exactly what happens if you
        # click through this quickly -- much better than silently landing
        # on a near-empty/overwritten default name.
        name = (context.scene.mimic_importer.robot_name or 'robot').strip()
        safe = ''.join(c if c.isalnum() or c in '-_' else '_' for c in name.lower())
        self.filepath = '%s_trajectory.json' % (safe or 'robot')
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        if not _rig:
            self.report({'ERROR'}, 'No rig built yet.')
            return {'CANCELLED'}
        try:
            _brb, te = _load_backend()
        except ImportError_ as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        path = self.filepath
        if not path.lower().endswith('.json'):
            path += '.json'
        try:
            doc = te.export_from_blender(_rig, path, robot_name=context.scene.mimic_importer.robot_name or None)
        except te.TrajectoryExportError as exc:
            self.report({'ERROR'}, str(exc))
            return {'CANCELLED'}

        self.report({'INFO'}, 'Wrote %d waypoint(s) to %s' % (len(doc['waypoints']), path))
        return {'FINISHED'}


# --------------------------------------------------------------------------
# Panel
# --------------------------------------------------------------------------

class MimicImporterProps(PropertyGroup):
    robot_path: StringProperty(
        name='Robot File', subtype='FILE_PATH',
        description='A .urdf or .xml (MJCF) robot description file')
    robot_name: StringProperty(
        name='Name (optional)',
        description='Override the rig\'s name; defaults to the name in the file')
    waypoint_step: IntProperty(
        name='Frames Between Waypoints', default=24, min=0,
        description='After recording a waypoint, jump the timeline forward by this many '
                    'frames (0 = stay put). 24 = one second at 24fps.')


class MIMIC_PT_panel(Panel):
    bl_label = 'Robot Import'
    bl_idname = 'MIMIC_PT_robot_importer'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Robot Import'

    def draw(self, context):
        layout = self.layout
        props = context.scene.mimic_importer

        col = layout.column()
        col.prop(props, 'robot_path')
        col.prop(props, 'robot_name')
        col.operator('mimic.build_rig', icon='ARMATURE_DATA')

        if not _rig:
            layout.label(text='Build a rig to continue.', icon='INFO')
            return

        layout.separator()
        box = layout.box()
        box.label(text='%d pose joint(s), %d target(s):' % (
            _rig['pose_joint_count'], len(_rig['chains'])))
        for i, chain in enumerate(_rig['chains']):
            row = box.row(align=True)
            label = chain.get('tip_link') or 'target_CTRL'
            row.label(text=label)
            op = row.operator('mimic.select_target', text='', icon='RESTRICT_SELECT_OFF')
            op.chain_index = i

        layout.separator()
        col = layout.column()
        col.label(text='Drag a target above in the viewport to pose that limb.')
        col.prop(props, 'waypoint_step')
        col.operator('mimic.record_waypoint', icon='KEY_HLT')
        col.operator('mimic.export_trajectory', icon='EXPORT')


# --------------------------------------------------------------------------
# Preferences
# --------------------------------------------------------------------------

class MimicImporterPreferences(AddonPreferences):
    bl_idname = ADDON_ID

    repo_path: StringProperty(
        name='Repo Path', subtype='DIR_PATH',
        description='Folder you cloned this repo into (the one containing a "mimic" folder)')

    def draw(self, context):
        self.layout.prop(self, 'repo_path')


# --------------------------------------------------------------------------
# Registration
# --------------------------------------------------------------------------

_classes = (
    MimicImporterProps,
    MIMIC_OT_build_rig,
    MIMIC_OT_select_target,
    MIMIC_OT_record_waypoint,
    MIMIC_OT_export_trajectory,
    MIMIC_PT_panel,
    MimicImporterPreferences,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.mimic_importer = PointerProperty(type=MimicImporterProps)


def unregister():
    del bpy.types.Scene.mimic_importer
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)


if __name__ == '__main__':
    register()
