"""
End-to-end test of the hardware-output pipeline: Blender rig -> keyframes ->
trajectory_export.py -> feetech_playback.py (--dry-run) and
ros_trajectory_export.py (--bag, read back and diffed).

Run with the real Blender GUI's python missing is fine -- this drives
Blender headlessly via subprocess, then plain python3 for the hardware
scripts (matching how a user would actually run them).

    python3 test/test_hardware_export.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, '..')
URDF = os.path.join(REPO, 'mimic', 'robot_descriptions', 'SO100', 'so100.urdf')
SERVO_MAP = os.path.join(REPO, 'mimic', 'scripts', 'hardware', 'servo_maps', 'so_arm_feetech.json')

# Same three poses feetech_playback / ros_trajectory_export tests below expect.
POSES = [
    (1, [0, 0, 0, 0, 0]),
    (24, [10, -20, 15, 5, -30]),
    (48, [-15, 10, -10, 20, 45]),
]


def build_and_export(traj_path):
    script = '''
import sys, math
sys.path.insert(0, "%(repo)s/mimic/scripts")
sys.path.insert(0, "%(repo)s/mimic/scripts/urdf_import")
import bpy
import blender_rig_builder as brb
import trajectory_export as te

rig = brb.build_rig("%(urdf)s", robot_name="SO100")
bpy.context.scene.render.fps = 24
poses = %(poses)r
for frame, angles in poses:
    bpy.context.scene.frame_set(frame)
    for obj, deg in zip(rig["axis_objs"], angles):
        obj.rotation_euler.z = math.radians(deg)
        obj.keyframe_insert(data_path="rotation_euler", index=2, frame=frame)
    if rig["tool_objs"]:
        rig["tool_objs"][0].rotation_euler.z = math.radians(10 if frame == 24 else 0)
        rig["tool_objs"][0].keyframe_insert(data_path="rotation_euler", index=2, frame=frame)

doc = te.export_from_blender(rig, "%(out)s")
print("EXPORT_OK joints=%%d waypoints=%%d" %% (len(doc["joint_names"]), len(doc["waypoints"])))
''' % {'repo': REPO, 'urdf': URDF, 'poses': POSES, 'out': traj_path}

    result = subprocess.run(
        ['blender', '--background', '--python-expr', script],
        capture_output=True, text=True, timeout=120)
    assert 'EXPORT_OK joints=6 waypoints=3' in result.stdout, (
        'trajectory export failed:\nSTDOUT:\n%s\nSTDERR:\n%s' % (result.stdout, result.stderr))
    with open(traj_path) as f:
        return json.load(f)


def check_trajectory_content(doc):
    assert doc['schema'] == 'mimic.trajectory.v1'
    assert doc['joint_names'] == ['joint%d' % i for i in range(6)]
    assert len(doc['waypoints']) == 3

    expected_times = [1 / 24, 1.0, 2.0]
    expected_positions = [pose + [0.0] for _, pose in [(1, [0, 0, 0, 0, 0])]] + [
        POSES[1][1] + [10.0], POSES[2][1] + [0.0]]
    # first waypoint's gripper is 0 (rest pose)
    expected_positions[0] = POSES[0][1] + [0.0]

    for wp, exp_t, exp_pos in zip(doc['waypoints'], expected_times, expected_positions):
        assert abs(wp['time'] - exp_t) < 1e-3, (wp['time'], exp_t)
        for got, exp in zip(wp['positions_deg'], exp_pos):
            assert abs(got - exp) < 1e-2, (wp['positions_deg'], exp_pos)
    print('trajectory content check: PASS')


def check_feetech_dry_run(traj_path):
    result = subprocess.run(
        [sys.executable, os.path.join(REPO, 'mimic', 'scripts', 'hardware', 'feetech_playback.py'),
         '--trajectory', traj_path, '--servo-map', SERVO_MAP, '--dry-run'],
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, 'feetech_playback --dry-run failed:\n%s' % result.stderr
    out = result.stdout
    assert 'shoulder_pan(id1)' in out and 'gripper(id6)' in out
    assert '9.09pct' in out or '9.1' in out, (
        'gripper degrees->percent conversion looks wrong:\n%s' % out)
    print('feetech --dry-run check: PASS')

    # Missing range_deg on a non-degrees servo must be a hard error, not a
    # silent wrong-unit write -- this is the safety check that exists
    # specifically so a mis-specified gripper servo can't slam to the wrong
    # physical position on real hardware.
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
        json.dump({'schema': 'mimic.servo_map.v1', 'robot': 'bad', 'servos': [
            {'joint_index': 0, 'name': 'g', 'id': 1, 'model': 'sts3215', 'norm_mode': 'range_0_100'}]}, f)
        bad_map = f.name
    try:
        bad_traj = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        json.dump({'schema': 'mimic.trajectory.v1', 'joint_names': ['g'], 'fps': 24,
                    'waypoints': [{'time': 0, 'positions_deg': [0]}]}, bad_traj)
        bad_traj.close()
        result = subprocess.run(
            [sys.executable, os.path.join(REPO, 'mimic', 'scripts', 'hardware', 'feetech_playback.py'),
             '--trajectory', bad_traj.name, '--servo-map', bad_map, '--dry-run'],
            capture_output=True, text=True, timeout=30)
        assert result.returncode != 0 and 'range_deg' in result.stderr, (
            'expected a hard error for missing range_deg, got:\n%s' % result.stderr)
        print('feetech missing-range_deg safety check: PASS')
    finally:
        os.unlink(bad_map)
        os.unlink(bad_traj.name)


def check_ros_bag(traj_path, bag_path):
    result = subprocess.run(
        [sys.executable, os.path.join(REPO, 'mimic', 'scripts', 'hardware', 'ros_trajectory_export.py'),
         '--trajectory', traj_path, '--servo-map', SERVO_MAP, '--bag', bag_path],
        capture_output=True, text=True, timeout=30,
        env={**os.environ})
    assert result.returncode == 0, 'ros_trajectory_export --bag failed:\n%s' % result.stderr

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from trajectory_msgs.msg import JointTrajectory

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    messages = []
    while reader.has_next():
        _topic, data, _t = reader.read_next()
        messages.append(deserialize_message(data, JointTrajectory))
    assert len(messages) == 1
    msg = messages[0]
    assert list(msg.joint_names) == [
        'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
    assert len(msg.points) == 3
    with open(traj_path) as f:
        traj = json.load(f)
    for point, wp in zip(msg.points, traj['waypoints']):
        for got_rad, exp_deg in zip(point.positions, wp['positions_deg']):
            assert abs(got_rad - math.radians(exp_deg)) < 1e-6
    print('ROS bag round-trip check: PASS')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as tmp:
        traj_path = os.path.join(tmp, 'traj.json')
        doc = build_and_export(traj_path)
        check_trajectory_content(doc)
        check_feetech_dry_run(traj_path)
        try:
            check_ros_bag(traj_path, os.path.join(tmp, 'bag'))
        except ImportError:
            print('ROS2 not sourced in this environment -- skipping ROS bag check '
                  '(run `source /opt/ros/humble/setup.bash` first to include it)')
    print()
    print('ALL HARDWARE EXPORT CHECKS PASSED')
