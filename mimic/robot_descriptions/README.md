# Bundled robot descriptions

These robot descriptions are local examples and validation fixtures for the
generic URDF/MJCF importer.

## Models

- `SO100/so100.urdf`
- `SO101/so101_new_calib.urdf`
- `UnitreeG1/g1_23dof.urdf` (branched-humanoid compatibility fixture)
- `Microduck/robot_allcollisions.xml` (branched MJCF validation fixture)

Each model's relative mesh references resolve within its model directory, so
the models do not depend on a user cache or external checkout.

The SO-ARM100/SO-ARM101 descriptions and meshes come from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100).
They are distributed under Apache License 2.0. See
`LICENSE-SO-ARM100` and `SO-ARM100-CITATION.cff` in this directory.

The Unitree G1 description and its referenced meshes come from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)
and are distributed under the BSD 3-Clause license. See
`UnitreeG1/LICENSE`.

The G1 fixture exercises a larger branched humanoid. Maya builds its complete
geometry and FK tree. Disjoint limbs receive separate IK targets; when two
end-effectors share an upstream driven joint, one receives IK and the
overlapping branch remains FK-only.

The Microduck MJCF and meshes come from
[pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl)
revision `5bbe9637294d0c794edb185e284cfb1a77c6a0b4`. The software/model XML is
Apache-2.0 and the source project identifies its 3D model files as Creative
Commons BY-SA-NC. See `Microduck/LICENSE-APACHE-2.0` and
`Microduck/SOURCE_README.md`.
