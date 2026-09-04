#!usr/bin/env python
"""
Publishes a mimic trajectory JSON (see urdf_import/trajectory_export.py) as
a standard ROS2 trajectory_msgs/JointTrajectory -- the message type
ros2_control's JointTrajectoryController (and most robot arm drivers) take
directly, whether the target is a real robot's controller or a simulator.
Works for any robot; the joint names are whatever's in --servo-map (or
--joint-names), not anything Feetech/servo-specific -- this is the
non-Feetech counterpart to feetech_playback.py.

Two ways to use the trajectory once it's a JointTrajectory:

  --publish-topic NAME
      Publish once to a topic (e.g. a *_controller/joint_trajectory topic)
      and exit. This is the "no-hardware-needed" path to verify against --
      run `ros2 topic echo NAME trajectory_msgs/msg/JointTrajectory` in
      another terminal first, then run this: you'll see the exact message
      a real controller would receive.

  --action-server NAME
      Send as a FollowJointTrajectory goal (the standard ros2_control
      action interface) and wait for the result -- this is how you'd
      actually drive a real ros2_control-managed robot.

Requires a sourced ROS2 environment (tested against Humble):
    source /opt/ros/humble/setup.bash
"""
import argparse
import json
import sys


class RosExportError(Exception):
    pass


def load_trajectory(path):
    with open(path) as f:
        doc = json.load(f)
    if doc.get('schema') != 'mimic.trajectory.v1':
        raise RosExportError('unrecognized trajectory schema: %r' % doc.get('schema'))
    return doc


def joint_names_from_servo_map(path):
    with open(path) as f:
        doc = json.load(f)
    servos = sorted(doc['servos'], key=lambda s: s['joint_index'])
    return [s['name'] for s in servos]


def build_joint_trajectory_msg(traj_doc, joint_names):
    import math
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    if len(joint_names) != len(traj_doc['joint_names']):
        raise RosExportError(
            'got %d joint names but trajectory has %d joints -- they must match 1:1 by position'
            % (len(joint_names), len(traj_doc['joint_names'])))

    msg = JointTrajectory()
    msg.joint_names = list(joint_names)
    for wp in traj_doc['waypoints']:
        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in wp['positions_deg']]  # ROS convention: radians
        seconds = wp['time']
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))
        msg.points.append(point)
    return msg


def write_bag(traj_doc, joint_names, bag_path):
    """Writes a rosbag2 (sqlite3) containing a single JointTrajectory message on /joint_trajectory."""
    import rclpy
    from rclpy.serialization import serialize_message
    import rosbag2_py

    msg = build_joint_trajectory_msg(traj_doc, joint_names)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name='/joint_trajectory', type='trajectory_msgs/msg/JointTrajectory', serialization_format='cdr'))
    writer.write('/joint_trajectory', serialize_message(msg), 0)
    del writer
    print('Wrote %s (topic /joint_trajectory, %d points)' % (bag_path, len(msg.points)))


def publish_once(traj_doc, joint_names, topic):
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init()
    node = Node('mimic_trajectory_publisher')
    pub = node.create_publisher(JointTrajectory, topic, 10)
    msg = build_joint_trajectory_msg(traj_doc, joint_names)

    # Give the publisher a moment to match with any existing subscriber
    # before publishing -- otherwise a fresh publisher's first message can
    # be dropped before subscribers discover it.
    import time
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() > 0:
            break
        time.sleep(0.05)

    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.2)
    print('Published %d points to %s (%d subscriber(s) connected)' % (
        len(msg.points), topic, pub.get_subscription_count()))
    node.destroy_node()
    rclpy.shutdown()


def send_action_goal(traj_doc, joint_names, action_name, timeout_sec=30.0):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory

    rclpy.init()
    node = Node('mimic_trajectory_action_client')
    client = ActionClient(node, FollowJointTrajectory, action_name)

    if not client.wait_for_server(timeout_sec=timeout_sec):
        node.destroy_node()
        rclpy.shutdown()
        raise RosExportError('action server %r not available after %.0fs' % (action_name, timeout_sec))

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = build_joint_trajectory_msg(traj_doc, joint_names)

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    goal_handle = future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.destroy_node()
        rclpy.shutdown()
        raise RosExportError('action server rejected the trajectory goal')

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)
    print('FollowJointTrajectory result: error_code=%s' % result_future.result().result.error_code)
    node.destroy_node()
    rclpy.shutdown()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--trajectory', required=True, help='trajectory JSON from trajectory_export.py')
    names = p.add_mutually_exclusive_group(required=True)
    names.add_argument('--servo-map', help='servo_maps/*.json -- joint names taken from here, in order')
    names.add_argument('--joint-names', help='comma-separated joint names, in the trajectory\'s order')
    p.add_argument('--bag', help='write a rosbag2 (sqlite3) to this path instead of publishing live')
    p.add_argument('--publish-topic', help='publish once to this topic and exit')
    p.add_argument('--action-server', help='send as a FollowJointTrajectory goal to this action server')
    args = p.parse_args(argv)

    if not (args.bag or args.publish_topic or args.action_server):
        p.error('one of --bag, --publish-topic, --action-server is required')

    traj = load_trajectory(args.trajectory)
    joint_names = (joint_names_from_servo_map(args.servo_map) if args.servo_map
                   else [n.strip() for n in args.joint_names.split(',')])

    print('Trajectory: %s (%s), %d joints, %d waypoints, %.2fs total' % (
        traj.get('robot_name'), traj.get('source_format'),
        len(traj['joint_names']), len(traj['waypoints']),
        traj['waypoints'][-1]['time'] if traj['waypoints'] else 0.0))
    print('Joint names:', joint_names)
    print()

    if args.bag:
        write_bag(traj, joint_names, args.bag)
    if args.publish_topic:
        publish_once(traj, joint_names, args.publish_topic)
    if args.action_server:
        send_action_goal(traj, joint_names, args.action_server)


if __name__ == '__main__':
    try:
        main()
    except RosExportError as exc:
        print('Error:', exc, file=sys.stderr)
        sys.exit(1)
