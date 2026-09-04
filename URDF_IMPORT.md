# Generic URDF/MJCF Importer for Mimic — Maya + Blender

Import URDF or MJCF robot descriptions (validated on **SO-ARM100**,
**SO-ARM101**, **Unitree G1**, and **Microduck**) and generate a rig with
working IK/FK animation. Runs in **Maya** (extending Mimic) and in
**Blender** (no licence required).

Mimic upstream only supports 6-axis industrial arms with one of two
closed-form solver topologies. This adds a path for arbitrary URDF robots
with an N-DOF numerical solver, without touching the existing industrial
code path.

---

## Layout

| File | Host | Purpose |
|---|---|---|
| `mimic/scripts/urdf_import/urdf_parser.py` | none (stdlib) | URDF → ordered kinematic chain |
| `mimic/scripts/urdf_import/mjcf_parser.py` | none (stdlib) | MJCF → shared robot/tree model |
| `mimic/scripts/urdf_import/model_parser.py` | none (stdlib) | URDF/MJCF format dispatch |
| `mimic/scripts/robotmath/generic_ik.py` | none (numpy) | N-DOF FK + damped-least-squares IK |
| `mimic/scripts/urdf_import/rig_builder.py` | Maya | builds the Maya rig |
| `mimic/plug-ins/robotIKGeneric.py` | Maya | `robotIKGeneric` DG node (live IK) |
| `mimic/scripts/urdf_import/urdf_import_ui.py` | Maya | import window + FK sliders |
| `mimic/scripts/urdf_import/blender_rig_builder.py` | Blender | builds the Blender rig |
| `mimic/scripts/urdf_import/trajectory_export.py` | Maya + Blender | keyframes → trajectory JSON |
| `mimic/scripts/hardware/feetech_playback.py` | none (plain python3) | trajectory JSON → real Feetech servos |
| `mimic/scripts/hardware/ros_trajectory_export.py` | none (plain python3 + ROS2) | trajectory JSON → ROS2 JointTrajectory |

The parser and solver have **no DCC dependency** and are shared verbatim by
both hosts — only scene-graph construction differs. That is what makes the
Maya↔Blender cross-check meaningful.

Blender's rig builder now shares Maya's branched-robot support (multiple
independent IK chains — one per limb — for humanoids/quadrupeds; see
"Branched robots" below): it reuses Maya's branch-selection logic directly
(`rig_builder._is_branched`, `_select_independent_chains` — pure Python, no
`maya.cmds`), so the two hosts always agree on which limbs get IK.

---

## Branched robots

Maya builds the complete geometry and FK hierarchy for branched descriptions.
Each disjoint root-to-leaf joint chain receives its own numerical IK node and
target control. Microduck therefore gets independent left-foot, head, and
right-foot targets plus FK controls for all 14 hinge joints.

If two end-effectors share an upstream driven joint, Maya cannot connect two
independent solver outputs to that same rotation channel. The deepest chain
gets IK and the overlapping branch remains FK-only, with a build warning.
This preserves the complete animatable robot without pretending independent
IK goals can safely own the same joints.

MJCF root `freejoint` motion maps to Maya's `local_CTRL`. Simulation-only
contacts, inertials, sensors, actuators, and equality constraints are not
needed for animation and are not imported.

---

## Why a new node instead of extending `robotIKS`

`robotIK.py`'s `robotIKS` has a **fixed 6-slot** attribute schema
(`theta1..theta6`, `a1_fk..a6_fk`, …) and `mimic_utils.py` hardcodes a
different rotate channel per axis (`axis1→rotateY, axis2→rotateX, …`) in
~15 places. Both are correct for the ~60 hand-built industrial rigs Mimic
ships, and refactoring them in place would risk all of those.

`robotIKGeneric` instead uses Maya **array attributes** sized by
`numJoints`, and every generated rig rotates uniformly about **local Z** —
each joint is built as a static `axis{i}_frame` (rest origin, local +Z
aligned to the URDF joint axis) plus an animated `axis{i}` child. So the
URDF's arbitrary axis directions need no per-axis channel mapping.

---

## The 5-DOF problem (important for SO-ARM)

**The SO-ARM100/101 have 5 pose DOFs**, not 6 (the 6th joint is the gripper
jaw — an actuator, not a pose DOF). A 5-DOF arm **cannot** generically reach
an arbitrary position *and* orientation.

So when you drag the IK handle to a new position while keeping its
orientation, that combination is usually impossible, and the solver must
choose. `rot_weight` controls the choice. Measured medians on both arms:

| `rot_weight` | position error | orientation error | |
|---|---|---|---|
| 1.0 | **18.0 mm** | 0.3° | tip visibly off the handle |
| 0.2 | 8.7 mm | 2.3° | |
| **0.05** (default for <6 DOF) | **1.4 mm** | 5.5° | chosen compromise |
| 0.0 | 0.03 mm | **18.7°** | gripper orientation adrift |

Defaults come from `generic_ik.recommended_rot_weight()`: `1.0` for ≥6 DOF,
`0.05` below that. Override per-rig via the node's `rotWeight` attribute
(negative = auto).

`converged=False` on a 5-DOF arm is **expected**, not a failure — it means
the full 6-DOF pose wasn't reachable. Check `ikPositionError` instead.

---

## Verification

Everything is checked against **pybullet**, an independent URDF parser and
FK implementation.

| Check | Result |
|---|---|
| Our FK vs pybullet, 200 random poses × 2 robots | max **2.7e-08 m** (0.000027 mm) |
| Maya rig joint positions vs pybullet | max **1.2e-08 m** (SO-100), 1.2e-06 m (SO-101) |
| Blender rig joint positions vs pybullet | max **3.5e-08 m** (SO-100), 1.2e-06 m (SO-101) |
| IK round-trip, reachable targets | 25/25 within **0.1 mm** |
| Interactive-drag convergence (single-shot) | 49/50 and 50/50 |
| FK passthrough (Maya, IK off) | exact |
| Visual render | correct assembled arms, both hosts |

### Known precision floors (not bugs)

* **Blender: ~1e-6 m.** Blender stores object transforms as **float32**
  (verified: a float64 location reads back as exactly its float32 value).
  Over a 5-joint chain at ~0.25 m this accumulates ~1 µm.
* **Maya SO-101: 1.18 µm.** Maya stores rotation as **Euler angles**, and
  the SO-101's joint-2 frame sits at gimbal lock (−90.0002°, −89.9998°), so
  its matrix cannot round-trip exactly. Affects only the *drawn* mesh — the
  solver receives exact matrices via the `jointOriginMatrix` matrix
  attribute, bypassing Euler decomposition entirely.

Both are ~170× smaller than the ≈0.2 mm tolerance of a 3D-printed part.

---

## Running it

All paths below are relative to a checkout of this repo (`<repo>` =
wherever you cloned it — on Windows that's just the checkout folder, same
idea, Windows paths).

**Maya** (GUI, needs a valid Maya licence):
```python
import sys
sys.path.insert(0, r"<repo>\mimic\scripts")
sys.path.insert(0, r"<repo>\mimic\scripts\urdf_import")
import maya.cmds as cmds
cmds.loadPlugin(r"<repo>\mimic\plug-ins\robotIKGeneric.py")
import urdf_import_ui; urdf_import_ui.urdf_import_ui()
```
(On Linux/macOS, use `/` and forward-slash paths instead of the `r"..."` Windows form.)

Bundled validation descriptions (including their STL assets) are available at:

```text
<repo>/mimic/robot_descriptions/SO100/so100.urdf
<repo>/mimic/robot_descriptions/SO101/so101_new_calib.urdf
<repo>/mimic/robot_descriptions/UnitreeG1/g1_23dof.urdf
<repo>/mimic/robot_descriptions/Microduck/robot_allcollisions.xml
```

**Blender** (no licence):
```python
import sys
sys.path += [r"<repo>\mimic\scripts", r"<repo>\mimic\scripts\urdf_import"]
import blender_rig_builder as brb
rig = brb.build_rig(r"<repo>\path\to\robot.urdf")
brb.solve_to_target(rig)          # drag `target_CTRL`, then re-solve
```

## Tests

Run these from the repo root.

```bash
python3 test/test_generic_ik.py
python3 test/test_mjcf_import.py
python3 test/test_hardware_export.py
```
```bash
mayapy test/build_and_check.py mimic/robot_descriptions/SO100/so100.urdf
mayapy test/build_and_check.py mimic/robot_descriptions/SO101/so101_new_calib.urdf
mayapy test/build_mjcf_and_check.py
```
(`mayapy` ships inside the Maya install, e.g.
`C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe` on Windows.)
```bash
blender --background --python test/blender_check.py -- <urdf> out.json out.png
```

---

## Hardware output

Two independent paths from a keyframed animation to real hardware — both
consuming the same DCC-agnostic trajectory JSON, so recording once gets you
both. Full details, including the real bugs this surfaced and fixed (a
gripper `range_0_100` conversion that would otherwise have commanded the
wrong physical position) in [`mimic/scripts/hardware/README.md`](mimic/scripts/hardware/README.md).

```
keyframe target_CTRL/axis objects (Maya or Blender's own animation tools)
              |
              v  trajectory_export.py
      trajectory.json  (DCC-agnostic: joint_names, fps, waypoints)
              |
      -----------------------------------------
      |                                       |
      v                                       v
feetech_playback.py                 ros_trajectory_export.py
(any Feetech-servo robot,           (JointTrajectory: publish to a
 built on lerobot's motor bus)       topic, write a rosbag2, or send a
                                      FollowJointTrajectory goal)
```

```bash
# Record: drag target_CTRL, press I (Blender) or S (Maya) to keyframe it,
# repeat for each waypoint, then export from that DCC's python console:
#   te.export_from_blender(rig, "traj.json")   /   te.export_from_maya(rig, "traj.json")

# See it without touching hardware:
python3 mimic/scripts/hardware/feetech_playback.py --trajectory traj.json \
    --servo-map mimic/scripts/hardware/servo_maps/so_arm_feetech.json --dry-run

# Real SO-ARM100/101 (or any Feetech-servo robot, with its own servo_map):
python3 mimic/scripts/hardware/feetech_playback.py --trajectory traj.json \
    --servo-map mimic/scripts/hardware/servo_maps/so_arm_feetech.json --port /dev/ttyUSB0

# ROS2 (any ros2_control robot):
source /opt/ros/humble/setup.bash
python3 mimic/scripts/hardware/ros_trajectory_export.py --trajectory traj.json \
    --servo-map mimic/scripts/hardware/servo_maps/so_arm_feetech.json \
    --action-server /arm_controller/follow_joint_trajectory
```

Tested end-to-end here (see `test/test_hardware_export.py`): real keyframes
→ export → content-checked JSON → Feetech `--dry-run` (including the
gripper degree→percent math) → ROS2 bag written and read back byte-exact,
plus a live publish confirmed delivered to a real subscriber. What isn't
testable without physical hardware: a real servo bus on `--port`, and a
real `FollowJointTrajectory` action server accepting the goal — both use
well-established libraries doing exactly what their reference
implementations do, but "the bytes on the wire are right" and "the robot
moves correctly" are different claims.

## Scope

Working: URDF/MJCF parse, mesh import, auto-rig, N-DOF IK/FK animation,
gripper as FK tool control, complete branched FK trees with independent
per-limb IK in both Maya and Blender, and hardware output (Feetech serial
and ROS2) from a recorded trajectory.

Not included: Mimic's existing keyframe-baking and program-export tools
(those remain hardcoded to 6 axes — the new hardware-output path above is
a separate, parallel export, not an extension of Mimic's own
postprocessors); prismatic/planar joints (reported and skipped). MJCF root
freejoints map to `local_CTRL`; non-root freejoints are rejected.
