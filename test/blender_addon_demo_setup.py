"""
Sets up a Blender session for a click-driven add-on demo: installs and
enables the add-on, configures the repo path, pre-fills the robot file
(so the demo focuses on Build Rig / drag / Record / Export rather than
fighting a native file-browser dialog on camera), and clears the default
scene. Everything after this is real mouse clicks on the actual panel.

    blender --python test/blender_addon_demo_setup.py -- <urdf_or_mjcf> [robot_name]
"""
import os
import sys

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..'))

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
ROBOT_PATH = argv[0] if argv else os.path.join(
    REPO, 'mimic', 'robot_descriptions', 'SO100', 'so100.urdf')
ROBOT_NAME = argv[1] if len(argv) > 1 else ''

addon_path = os.path.join(REPO, 'mimic', 'scripts', 'urdf_import', 'mimic_robot_importer_addon.py')
bpy.ops.preferences.addon_install(filepath=addon_path)
bpy.ops.preferences.addon_enable(module='mimic_robot_importer_addon')
bpy.context.preferences.addons['mimic_robot_importer_addon'].preferences.repo_path = REPO

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
for coll in ('Cube', 'Camera', 'Light'):
    obj = bpy.data.objects.get(coll)
    if obj is not None:
        bpy.data.objects.remove(obj, do_unlink=True)

props = bpy.context.scene.mimic_importer
props.robot_path = ROBOT_PATH
props.robot_name = ROBOT_NAME

# Open the sidebar (N-panel) on the "Robot Import" tab so it's visible
# without the presenter needing to press N on camera.
for window in bpy.context.window_manager.windows:
    for area in window.screen.areas:
        if area.type == 'VIEW_3D':
            area.spaces[0].show_region_ui = True
            area.spaces[0].shading.type = 'SOLID'
            area.spaces[0].shading.color_type = 'MATERIAL'

print('[demo_setup] Ready. Robot Import panel should be visible in the N-sidebar.')
