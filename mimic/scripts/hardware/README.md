# Hardware output

Gets a keyframed animation (from either DCC's URDF-imported rig) onto real
hardware. Two independent paths, both consuming the same trajectory JSON:

```
 Maya or Blender rig          trajectory_export.py           trajectory.json
 (keyframe target_CTRL   -->  (reads the keyframes)     -->  {joint_names,
  or axis rotations)                                          fps, waypoints}
                                                                     |
                                          -----------------------------------------
                                          |                                       |
                                          v                                       v
                              feetech_playback.py                  ros_trajectory_export.py
                              (direct servo control,                (JointTrajectory: publish,
                               any Feetech-servo robot)               bag, or FollowJointTrajectory)
```

Neither hardware path needs to know about Maya/Blender, and the export
step doesn't need to know which hardware path you'll use -- that's the
point of the JSON seam. `trajectory.json` schema (`mimic.trajectory.v1`):

```json
{
  "robot_name": "SO100", "source_format": "blender",
  "joint_names": ["joint0", "joint1", ...], "fps": 24,
  "waypoints": [{"time": 0.04, "positions_deg": [0, 0, 0, 0, 0, 0]}, ...]
}
```

`positions_deg` is always raw degrees of joint rotation, uniformly, for
every joint including grippers -- never a servo-specific percent/tick
value. That conversion is each hardware path's own job (see
`feetech_playback.py`'s `range_deg` handling below), because "degrees of
rotation" is the one thing every joint means the same way regardless of
what's attached to it.

## Recording a trajectory (the "animate it" part)

`target_CTRL` (and every `axis`/FK object) is an ordinary Maya/Blender
object. Recording a waypoint IS keyframing it, the normal way:

- **Blender**: drag `target_CTRL`, then press **I** over the 3D viewport
  (Insert Keyframe) on the axis objects -- or turn on auto-key. Scrub the
  timeline to preview; it plays back like any character animation.
- **Maya**: drag `target_CTRL`, then **S** (Set Key) or right-click >
  Key Selected on the FK/axis controls.

Then export:

```python
# Blender
import trajectory_export as te
te.export_from_blender(rig, "traj.json", robot_name="SO100")

# Maya
import trajectory_export as te
te.export_from_maya(rig, "traj.json", robot_name="SO100")
```

Only the *keyframed* times get exported as waypoints (not every frame), so
a handful of hand-posed waypoints stays a handful of waypoints.

## Path 1: direct Feetech servo control

For SO-ARM100/101 or any other robot on Feetech (STS/SCS-series) servos.
Built on `lerobot.motors.feetech.FeetechMotorsBus` -- the same driver
LeRobot itself uses for these servos -- rather than re-implementing the
STS/SCS protocol.

```bash
pip install lerobot   # vendors scservo_sdk

# No hardware needed -- prints every command, with timing:
python3 feetech_playback.py --trajectory traj.json \
    --servo-map servo_maps/so_arm_feetech.json --dry-run

# Real hardware:
python3 feetech_playback.py --trajectory traj.json \
    --servo-map servo_maps/so_arm_feetech.json --port /dev/ttyUSB0
```

`servo_maps/*.json` maps the trajectory's joint *positions* (not names --
URDF joint names are frequently meaningless, e.g. SO-ARM's are literally
"1".."6") to a servo id + model + `norm_mode`:

- `so_arm_feetech.json` / `so101_arm_feetech.json` -- the SO-ARM100/101's
  actual servo IDs, taken directly from LeRobot's own `so_follower.py`
  config, so a trajectory recorded from the URDF import lines up with a
  real SO-ARM's motor bus with no manual remapping.
- To support another Feetech-servo robot: copy one of these and change
  the `id`/`model`/`name` list to that robot's actual servos, in the same
  base-to-tip order the rig builder walks the URDF/MJCF in.

`norm_mode: "range_0_100"` (e.g. a gripper) needs a `range_deg: [lo, hi]`
in its servo entry -- the joint's own URDF limits, in degrees -- because
the trajectory stores raw degrees uniformly and this path has to convert
"degrees of joint rotation" into "percent of this specific servo's
configured range" before writing `Goal_Position`. Omitting `range_deg`
on such a servo is a hard error (refuses to guess) rather than silently
sending the wrong physical position to a servo whose calibration means
something different than plain degrees -- worth being strict about,
since getting a gripper's range wrong is exactly the kind of mistake that
looks fine in a dry run and then slams the real hardware.

## Path 2: ROS2

For anything using `ros2_control` (or another `JointTrajectory` consumer).
No Feetech-specific knowledge here -- straight `trajectory_msgs/JointTrajectory`,
positions in radians (ROS convention), which is what a `JointTrajectoryController`
or a `FollowJointTrajectory` action server expects natively.

```bash
source /opt/ros/humble/setup.bash   # or your ROS2 distro

# Inspect without any running robot/controller:
python3 ros_trajectory_export.py --trajectory traj.json \
    --servo-map ../hardware/servo_maps/so_arm_feetech.json --bag traj_bag
ros2 bag info traj_bag

# Publish once to a controller's command topic:
python3 ros_trajectory_export.py --trajectory traj.json \
    --joint-names shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper \
    --publish-topic /arm_controller/joint_trajectory

# Send as a real FollowJointTrajectory goal and wait for completion:
python3 ros_trajectory_export.py --trajectory traj.json \
    --servo-map ../hardware/servo_maps/so_arm_feetech.json \
    --action-server /arm_controller/follow_joint_trajectory
```

`--servo-map` here is just a convenient source of joint names in the
right order (its Feetech ids/norm_modes are ignored) -- use `--joint-names`
directly for a non-Feetech robot with no servo map at all.

## What's verified vs. what needs real hardware to verify

Everything up to the servo/ROS write is tested in CI-reachable ways (no
physical robot needed): trajectory export round-trips real keyframes
correctly, Feetech `--dry-run` exercises the full degree→servo-unit
conversion including the `range_0_100` gripper math, and the ROS path has
been exercised for real against ROS2 Humble -- `--bag` output read back
and diffed against the source trajectory, and `--publish-topic` confirmed
delivered to a live subscriber, both byte-exact.

What can only be verified against actual hardware: that `--port` talks to
a real servo bus correctly, and that a real `FollowJointTrajectory` action
server accepts a goal built this way. Both use well-established libraries
(`lerobot`'s servo bus, `rclpy`'s action client) doing exactly what their
own reference implementations do, but "the bytes on the wire are right"
and "the robot moves correctly" are different claims -- test on hardware
before trusting a recorded trajectory unattended.
