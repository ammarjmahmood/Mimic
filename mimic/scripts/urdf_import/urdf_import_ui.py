#!usr/bin/env python
"""
"Import Robot Description" tool for Mimic: pick a URDF or MJCF file, build a
rig from it (see rig_builder.py), then drive it -- drag target controls in
the viewport for IK, or use the joint sliders here for FK. Same
cmds.window-based tool pattern as mimic/scripts/rigging/rigging.py.
"""
try:
    import maya.cmds as cmds
    MAYA_IS_RUNNING = True
except ImportError:
    cmds = None
    MAYA_IS_RUNNING = False

import os
import traceback

import rig_builder
import importlib

importlib.reload(rig_builder)

FONT = 'smallObliqueLabelFont'
WIN = 'urdfImport_win'
JOINTS_COLUMN = 'urdfImport_jointsColumn'

_last_result = {}


def urdf_import_ui():
    if cmds.window(WIN, exists=True):
        cmds.deleteUI(WIN, window=True)

    win = cmds.window(
        WIN, title='Mimic: Import URDF / MJCF', width=320, sizeable=True)
    cmds.columnLayout(adjustableColumn=True, rowSpacing=4)

    cmds.separator(height=4, style='none')
    cmds.text(label='Robot description:', align='left')
    cmds.rowLayout(numberOfColumns=2, adjustableColumn=1, columnAttach=(1, 'left', 0))
    cmds.textField('t_urdfPath', font=FONT)
    cmds.button(label='Browse...', command=_browse_for_urdf, width=70)
    cmds.setParent('..')

    cmds.separator(height=8, style='in')
    cmds.button(label='Build Rig', height=30, command=_build_rig_cb)
    cmds.text('t_status', label='', align='left', wordWrap=True, height=40)

    cmds.separator(height=8, style='in')
    cmds.text(label='Joint Controls (FK)', align='left')
    cmds.checkBox('cb_ik', label='IK (drag target_CTRL in viewport)',
                   value=True, changeCommand=_ik_toggle_cb)
    cmds.columnLayout(JOINTS_COLUMN, adjustableColumn=True)
    cmds.setParent('..')

    cmds.setParent('..')
    cmds.showWindow(win)


def _browse_for_urdf(*_args):
    result = cmds.fileDialog2(
        fileMode=1,
        caption='Select a URDF or MJCF file',
        fileFilter='Robot Descriptions (*.urdf *.xml);;URDF (*.urdf);;MJCF (*.xml);;All Files (*.*)')
    if result:
        cmds.textField('t_urdfPath', edit=True, text=result[0])


def _build_rig_cb(*_args):
    urdf_path = cmds.textField('t_urdfPath', query=True, text=True).strip()
    if not urdf_path or not os.path.isfile(urdf_path):
        cmds.text('t_status', edit=True, label='Please choose a valid URDF or MJCF file.')
        return

    try:
        result = rig_builder.build_rig(urdf_path)
    except rig_builder.RigBuildError as exc:
        cmds.text('t_status', edit=True, label='Build failed: %s' % exc)
        return
    except Exception:
        cmds.text('t_status', edit=True, label='Build failed (see Script Editor for details).')
        traceback.print_exc()
        return

    global _last_result
    _last_result = result

    msg = '%s: %s pose joints, %s tool/gripper joints built.' % (
        result.get('source_format', 'urdf').upper(),
        result['pose_joint_count'], result['tool_joint_count'])
    if len(result.get('target_ctrls', [])) > 1:
        msg += '\n%s independent IK targets built.' % len(result['target_ctrls'])
    if result['warnings']:
        msg += '\nWarnings:\n' + '\n'.join('- %s' % w for w in result['warnings'])
    cmds.text('t_status', edit=True, label=msg)

    _rebuild_joint_sliders(result)


def _rebuild_joint_sliders(result):
    if cmds.columnLayout(JOINTS_COLUMN, exists=True):
        children = cmds.columnLayout(JOINTS_COLUMN, query=True, childArray=True) or []
        for c in children:
            cmds.deleteUI(c)

    cmds.setParent(JOINTS_COLUMN)
    fk_nodes = result['fk_nodes']
    for i, fk_node in enumerate(fk_nodes):
        cmds.floatSliderGrp(
            'fk_slider_%d' % i,
            label='A%d' % (i + 1),
            field=True,
            minValue=-180, maxValue=180,
            fieldMinValue=-360, fieldMaxValue=360,
            value=0,
            columnWidth=[(1, 30), (2, 50), (3, 150)],
            changeCommand=lambda v, node=fk_node: cmds.setAttr(node + '.rotateZ', v),
            dragCommand=lambda v, node=fk_node: cmds.setAttr(node + '.rotateZ', v))

    for i, tool_node in enumerate(result.get('gripper_nodes', [])):
        cmds.floatSliderGrp(
            'tool_slider_%d' % i,
            label='Tool%d' % (i + 1),
            field=True,
            minValue=-180, maxValue=180,
            fieldMinValue=-360, fieldMaxValue=360,
            value=0,
            columnWidth=[(1, 30), (2, 50), (3, 150)],
            changeCommand=lambda v, node=tool_node: cmds.setAttr(node + '.rotateZ', v),
            dragCommand=lambda v, node=tool_node: cmds.setAttr(node + '.rotateZ', v))


def _ik_toggle_cb(value):
    if not _last_result:
        return
    for target in _last_result.get('target_ctrls', [_last_result['target_ctrl']]):
        cmds.setAttr(target + '.ik', bool(value))
