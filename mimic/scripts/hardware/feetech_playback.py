#!usr/bin/env python
"""
Plays a mimic trajectory JSON (see urdf_import/trajectory_export.py) back
onto real Feetech servos -- works for any robot on Feetech servos, not just
the SO-ARM100/101, given a matching servo_maps/*.json.

Built on lerobot.motors.feetech.FeetechMotorsBus (the same servo driver
LeRobot itself uses for the SO-ARM100/101 and other Feetech-servo robots),
rather than talking the STS/SCS protocol directly -- that library is the
tested reference implementation for these exact servos, no reason to
reinvent it.

Usage:
    # No hardware needed -- prints every command it WOULD send, with timing:
    python3 feetech_playback.py --trajectory traj.json \\
        --servo-map servo_maps/so_arm_feetech.json --dry-run

    # Real hardware:
    python3 feetech_playback.py --trajectory traj.json \\
        --servo-map servo_maps/so_arm_feetech.json --port /dev/ttyUSB0

Requires: pip install lerobot   (already vendors scservo_sdk)
"""
import argparse
import json
import sys
import time


class PlaybackError(Exception):
    pass


def load_servo_map(path):
    with open(path) as f:
        doc = json.load(f)
    servos = sorted(doc['servos'], key=lambda s: s['joint_index'])
    expected = list(range(len(servos)))
    got = [s['joint_index'] for s in servos]
    if got != expected:
        raise PlaybackError(
            'servo_map joint_index values must be a contiguous 0..N-1 range, got %s' % got)
    for s in servos:
        if s['norm_mode'] != 'degrees' and 'range_deg' not in s:
            raise PlaybackError(
                'servo "%s" has norm_mode %r but no range_deg -- trajectories store raw '
                'degrees uniformly for every joint (see trajectory_export.py), so a '
                'non-degrees servo needs range_deg to know how to convert. Without it, '
                'sending raw degrees to a %s-mode servo would command the wrong physical '
                'position -- refusing rather than guessing.' % (s['name'], s['norm_mode'], s['norm_mode']))
    return doc, servos


def _degrees_to_value(deg, servo):
    """Converts an exported joint-rotation degree value into whatever unit
    this specific servo's norm_mode actually expects (see load_servo_map's
    range_deg check for why this conversion exists at all)."""
    if servo['norm_mode'] == 'degrees':
        return deg
    lo, hi = servo['range_deg']
    pct = (deg - lo) / (hi - lo) * 100.0
    pct = max(0.0, min(100.0, pct))
    if servo['norm_mode'] == 'range_0_100':
        return pct
    if servo['norm_mode'] == 'range_m100_100':
        return pct * 2.0 - 100.0
    raise PlaybackError('unknown norm_mode %r' % servo['norm_mode'])


def load_trajectory(path):
    with open(path) as f:
        doc = json.load(f)
    if doc.get('schema') != 'mimic.trajectory.v1':
        raise PlaybackError('unrecognized trajectory schema: %r' % doc.get('schema'))
    return doc


def _build_bus(servos, port):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    norm_modes = {
        'degrees': MotorNormMode.DEGREES,
        'range_0_100': MotorNormMode.RANGE_0_100,
        'range_m100_100': MotorNormMode.RANGE_M100_100,
    }
    motors = {}
    for s in servos:
        motors[s['name']] = Motor(s['id'], s['model'], norm_modes[s['norm_mode']])
    return FeetechMotorsBus(port=port, motors=motors)


def play(trajectory_doc, servos, port, dry_run=True, speed_scale=1.0):
    """
    Walks waypoints in order, sync-writing Goal_Position to all servos at
    each one and sleeping for the (scaled) recorded time delta -- this is
    literal trajectory PLAYBACK (respecting recorded timing), not just a
    sequence of independent moves. speed_scale > 1 plays back faster,
    < 1 slower; 1.0 replays at the timing it was recorded with.
    """
    waypoints = trajectory_doc['waypoints']
    n_joints = len(trajectory_doc['joint_names'])
    if len(servos) != n_joints:
        raise PlaybackError(
            'servo map has %d servos but trajectory has %d joints -- they must match 1:1 by position'
            % (len(servos), n_joints))

    bus = None
    if not dry_run:
        bus = _build_bus(servos, port)
        bus.connect()
        for s in servos:
            bus.write('Torque_Enable', s['name'], 1)

    try:
        prev_time = 0.0
        for wp in waypoints:
            dt = (wp['time'] - prev_time) / speed_scale
            prev_time = wp['time']
            if dt > 0 and not dry_run:
                time.sleep(dt)

            values = {}
            for s, deg in zip(servos, wp['positions_deg']):
                values[s['name']] = _degrees_to_value(deg, s)

            if dry_run:
                unit = {'degrees': 'deg', 'range_0_100': 'pct', 'range_m100_100': 'pct'}
                print('t=%7.3fs  dt=%6.3fs  %s' % (
                    wp['time'], dt,
                    ', '.join('%s(id%d)=%.2f%s' % (
                        s['name'], s['id'], values[s['name']], unit[s['norm_mode']])
                        for s in servos)))
            else:
                bus.sync_write('Goal_Position', values)
    finally:
        if bus is not None:
            bus.disconnect()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--trajectory', required=True, help='trajectory JSON from trajectory_export.py')
    p.add_argument('--servo-map', required=True, help='servo_maps/*.json for the target robot')
    p.add_argument('--port', default=None, help='serial port, e.g. /dev/ttyUSB0 or COM3 (required unless --dry-run)')
    p.add_argument('--dry-run', action='store_true',
                   help='print every command that would be sent, with timing; touches no hardware/port')
    p.add_argument('--speed', type=float, default=1.0,
                   help='playback speed multiplier (default 1.0 = recorded timing)')
    args = p.parse_args(argv)

    if not args.dry_run and not args.port:
        p.error('--port is required unless --dry-run')

    traj = load_trajectory(args.trajectory)
    _map_doc, servos = load_servo_map(args.servo_map)

    print('Trajectory: %s (%s), %d joints, %d waypoints, %.2fs total' % (
        traj.get('robot_name'), traj.get('source_format'),
        len(traj['joint_names']), len(traj['waypoints']),
        traj['waypoints'][-1]['time'] if traj['waypoints'] else 0.0))
    print('Servo map: %s, %d servos%s' % (
        _map_doc.get('robot'), len(servos), ' (DRY RUN -- no hardware touched)' if args.dry_run else ''))
    print()

    play(traj, servos, args.port, dry_run=args.dry_run, speed_scale=args.speed)
    print('Done.')


if __name__ == '__main__':
    try:
        main()
    except PlaybackError as exc:
        print('Error:', exc, file=sys.stderr)
        sys.exit(1)
