#!usr/bin/env python

"""
Dependency Graph plug-in that performs generic N-DOF numerical IK, for rigs
built by the URDF importer (mimic/scripts/urdf_import/).

This is deliberately a *separate* node from robotIK.py's `robotIKS` rather
than a modification of it: robotIKS's attribute schema is a fixed 6-slot
compound (a1_fk_attr..a6_fk_attr, theta1_attr..theta6_attr, etc) hardcoded
throughout mimic_utils.py, and is exactly right for the ~60 hand-built
industrial rigs Mimic already ships. A URDF-imported arm can have any joint
count (the SO-ARM100/101 have 5 pose DOFs, not 6) and arbitrary joint axes
(no assumption of a spherical wrist or standard DH form), so this node uses
Maya *array* attributes sized by `numJoints` instead, and solves with the
numerical damped-least-squares solver in robotmath/generic_ik.py instead of
a closed-form formula.
"""

import sys
import maya.api.OpenMaya as OpenMaya
import numpy as np

from robotmath import generic_ik
import importlib

importlib.reload(generic_ik)  # For debugging


def maya_useNewAPI():
    pass


kPluginNodeName = 'robotIKGeneric'
kPluginNodeClassify = 'utility/general'
kPluginNodeId = OpenMaya.MTypeId(0x87002)  # distinct from robotIKS's 0x87001

MAX_JOINTS = 12  # generous upper bound for array attribute sizing; actual count is numJoints


def _maya_matrix_to_numpy(m):
    """
    Maya's MMatrix uses the row-vector convention (p' = p * M, translation
    in the last row). generic_ik.py uses the standard column-vector
    convention (p' = M * p, translation in the last column), so the
    conversion is a transpose.
    """
    flat = np.array(list(m), dtype=float).reshape(4, 4)
    return flat.T


class robotIKGeneric(OpenMaya.MPxNode):

    numJoints_attr = OpenMaya.MObject()
    jointAxis_attr = OpenMaya.MObject()
    jointOriginMatrix_attr = OpenMaya.MObject()
    jointLimitLower_attr = OpenMaya.MObject()
    jointLimitUpper_attr = OpenMaya.MObject()
    fk_attr = OpenMaya.MObject()

    tcp_mat_attr = OpenMaya.MObject()
    lcs_mat_attr = OpenMaya.MObject()
    target_mat_attr = OpenMaya.MObject()

    ik_attr = OpenMaya.MObject()
    rot_weight_attr = OpenMaya.MObject()

    theta_attr = OpenMaya.MObject()
    ik_converged_attr = OpenMaya.MObject()
    ik_pos_error_attr = OpenMaya.MObject()

    def __init__(self):
        OpenMaya.MPxNode.__init__(self)

    def compute(self, pPlug, pDataBlock):
        num_joints_handle = pDataBlock.inputValue(robotIKGeneric.numJoints_attr)
        num_joints = num_joints_handle.asInt()
        num_joints = max(0, min(num_joints, MAX_JOINTS))

        axis_array = pDataBlock.inputArrayValue(robotIKGeneric.jointAxis_attr)
        origin_array = pDataBlock.inputArrayValue(robotIKGeneric.jointOriginMatrix_attr)
        lower_array = pDataBlock.inputArrayValue(robotIKGeneric.jointLimitLower_attr)
        upper_array = pDataBlock.inputArrayValue(robotIKGeneric.jointLimitUpper_attr)
        fk_array = pDataBlock.inputArrayValue(robotIKGeneric.fk_attr)

        joints = []
        fk_radians = []
        for i in range(num_joints):
            axis_array.jumpToLogicalElement(i)
            axis = axis_array.inputValue().asDouble3()

            origin_array.jumpToLogicalElement(i)
            origin_m = _maya_matrix_to_numpy(origin_array.inputValue().asMatrix())

            lower_array.jumpToLogicalElement(i)
            lower = lower_array.inputValue().asDouble()

            upper_array.jumpToLogicalElement(i)
            upper = upper_array.inputValue().asDouble()

            lower = None if lower <= -1e6 else lower
            upper = None if upper >= 1e6 else upper

            joints.append(generic_ik.JointSpec(origin_m, list(axis), lower, upper))

            fk_array.jumpToLogicalElement(i)
            fk_radians.append(fk_array.inputValue().asAngle().asRadians())

        tcp_mat = _maya_matrix_to_numpy(pDataBlock.inputValue(robotIKGeneric.tcp_mat_attr).asMatrix())
        lcs_mat = _maya_matrix_to_numpy(pDataBlock.inputValue(robotIKGeneric.lcs_mat_attr).asMatrix())
        target_mat = _maya_matrix_to_numpy(pDataBlock.inputValue(robotIKGeneric.target_mat_attr).asMatrix())

        ik = pDataBlock.inputValue(robotIKGeneric.ik_attr).asBool()

        theta_out_array = pDataBlock.outputArrayValue(robotIKGeneric.theta_attr)
        converged_handle = pDataBlock.outputValue(robotIKGeneric.ik_converged_attr)
        pos_error_handle = pDataBlock.outputValue(robotIKGeneric.ik_pos_error_attr)

        if num_joints == 0:
            converged_handle.setBool(True)
            converged_handle.setClean()
            pos_error_handle.setDouble(0.0)
            pos_error_handle.setClean()
            return

        if ik and num_joints > 0:
            # Seed the solve from the previously computed joint angles (read
            # the output plug's current value before we overwrite it) so
            # interactive dragging of target_CTRL stays continuous instead
            # of jumping between arbitrary IK solutions every recompute.
            seed = []
            for i in range(num_joints):
                try:
                    theta_out_array.jumpToLogicalElement(i)
                    seed.append(theta_out_array.inputValue().asAngle().asRadians())
                except RuntimeError:
                    seed.append(0.0)  # no prior value yet (first-ever compute)
            seed = np.array(seed)

            target_local = np.linalg.inv(lcs_mat) @ target_mat

            # Rig geometry here is in centimeters (see rig_builder.py's
            # METERS_TO_CM), so the tolerances are scaled accordingly:
            # 0.01cm = 0.1mm position, ~0.03deg orientation. generic_ik's
            # own defaults assume meter-scale robots and would be far too
            # tight (micron-level) at this scale.
            #
            # rotWeight defaults per DOF count (see
            # generic_ik.recommended_rot_weight): a <6-DOF arm cannot hit an
            # arbitrary 6-DOF pose, so orientation is down-weighted to keep
            # the tool tip pinned to the animator's handle. Exposed as an
            # attribute so it can be dialled per-rig.
            rot_weight = pDataBlock.inputValue(robotIKGeneric.rot_weight_attr).asDouble()
            if rot_weight < 0.0:
                rot_weight = generic_ik.recommended_rot_weight(num_joints)

            thetas, converged, pos_err, rot_err = generic_ik.solve_ik_robust(
                joints, seed, target_local, tcp_offset=tcp_mat,
                tol_pos=0.01, tol_rot=0.0005, rot_weight=rot_weight)
        else:
            thetas = np.array(fk_radians)
            converged = True
            pos_err = 0.0

        builder_handle = theta_out_array.builder()
        for i in range(num_joints):
            out_elem = builder_handle.addElement(i)
            out_elem.setMAngle(OpenMaya.MAngle(float(thetas[i]), OpenMaya.MAngle.kRadians))
        theta_out_array.set(builder_handle)
        theta_out_array.setAllClean()

        converged_handle.setBool(bool(converged))
        converged_handle.setClean()
        pos_error_handle.setDouble(float(pos_err))
        pos_error_handle.setClean()


def nodeCreator():
    return robotIKGeneric()


def nodeInitializer():
    numericAttributeFn = OpenMaya.MFnNumericAttribute()
    angleAttributeFn = OpenMaya.MFnUnitAttribute()
    matrixAttributeFn = OpenMaya.MFnMatrixAttribute()

    #==================================#
    #      INPUT NODE ATTRIBUTE(S)     #
    #==================================#

    robotIKGeneric.numJoints_attr = numericAttributeFn.create(
        'numJoints', 'numJoints', OpenMaya.MFnNumericData.kInt, 0)
    numericAttributeFn.storable = True
    numericAttributeFn.writable = True
    robotIKGeneric.addAttribute(robotIKGeneric.numJoints_attr)

    robotIKGeneric.jointAxis_attr = numericAttributeFn.create(
        'jointAxis', 'jAxis', OpenMaya.MFnNumericData.k3Double)
    numericAttributeFn.storable = True
    numericAttributeFn.writable = True
    numericAttributeFn.array = True
    numericAttributeFn.usesArrayDataBuilder = True
    robotIKGeneric.addAttribute(robotIKGeneric.jointAxis_attr)

    robotIKGeneric.jointOriginMatrix_attr = matrixAttributeFn.create(
        'jointOriginMatrix', 'jOrigin', OpenMaya.MFnMatrixAttribute.kDouble)
    matrixAttributeFn.storable = True
    matrixAttributeFn.writable = True
    matrixAttributeFn.array = True
    matrixAttributeFn.usesArrayDataBuilder = True
    robotIKGeneric.addAttribute(robotIKGeneric.jointOriginMatrix_attr)

    robotIKGeneric.jointLimitLower_attr = numericAttributeFn.create(
        'jointLimitLower', 'jLo', OpenMaya.MFnNumericData.kDouble, -1e7)
    numericAttributeFn.storable = True
    numericAttributeFn.writable = True
    numericAttributeFn.array = True
    numericAttributeFn.usesArrayDataBuilder = True
    robotIKGeneric.addAttribute(robotIKGeneric.jointLimitLower_attr)

    robotIKGeneric.jointLimitUpper_attr = numericAttributeFn.create(
        'jointLimitUpper', 'jHi', OpenMaya.MFnNumericData.kDouble, 1e7)
    numericAttributeFn.storable = True
    numericAttributeFn.writable = True
    numericAttributeFn.array = True
    numericAttributeFn.usesArrayDataBuilder = True
    robotIKGeneric.addAttribute(robotIKGeneric.jointLimitUpper_attr)

    robotIKGeneric.fk_attr = angleAttributeFn.create(
        'fk', 'fk', OpenMaya.MFnUnitAttribute.kAngle)
    angleAttributeFn.storable = True
    angleAttributeFn.writable = True
    angleAttributeFn.array = True
    angleAttributeFn.usesArrayDataBuilder = True
    robotIKGeneric.addAttribute(robotIKGeneric.fk_attr)

    robotIKGeneric.tcp_mat_attr = matrixAttributeFn.create(
        'tcpMatrix', 'tcpMat', OpenMaya.MFnMatrixAttribute.kDouble)
    matrixAttributeFn.storable = True
    matrixAttributeFn.writable = True
    robotIKGeneric.addAttribute(robotIKGeneric.tcp_mat_attr)

    robotIKGeneric.lcs_mat_attr = matrixAttributeFn.create(
        'lcsMatrix', 'lcsMat', OpenMaya.MFnMatrixAttribute.kDouble)
    matrixAttributeFn.storable = True
    matrixAttributeFn.writable = True
    robotIKGeneric.addAttribute(robotIKGeneric.lcs_mat_attr)

    robotIKGeneric.target_mat_attr = matrixAttributeFn.create(
        'targetMatrix', 'targetMat', OpenMaya.MFnMatrixAttribute.kDouble)
    matrixAttributeFn.storable = True
    matrixAttributeFn.writable = True
    robotIKGeneric.addAttribute(robotIKGeneric.target_mat_attr)

    robotIKGeneric.ik_attr = numericAttributeFn.create(
        'ik', 'ik', OpenMaya.MFnNumericData.kBoolean, True)
    numericAttributeFn.storable = True
    numericAttributeFn.writable = True
    robotIKGeneric.addAttribute(robotIKGeneric.ik_attr)

    # Negative = "auto", i.e. pick from the joint count at solve time.
    robotIKGeneric.rot_weight_attr = numericAttributeFn.create(
        'rotWeight', 'rotWeight', OpenMaya.MFnNumericData.kDouble, -1.0)
    numericAttributeFn.storable = True
    numericAttributeFn.writable = True
    robotIKGeneric.addAttribute(robotIKGeneric.rot_weight_attr)

    #==================================#
    #     OUTPUT NODE ATTRIBUTE(S)     #
    #==================================#

    robotIKGeneric.theta_attr = angleAttributeFn.create(
        'theta', 'theta', OpenMaya.MFnUnitAttribute.kAngle)
    angleAttributeFn.storable = False
    angleAttributeFn.writable = False
    angleAttributeFn.readable = True
    angleAttributeFn.array = True
    angleAttributeFn.usesArrayDataBuilder = True
    robotIKGeneric.addAttribute(robotIKGeneric.theta_attr)

    robotIKGeneric.ik_converged_attr = numericAttributeFn.create(
        'ikConverged', 'ikConverged', OpenMaya.MFnNumericData.kBoolean, True)
    numericAttributeFn.storable = False
    numericAttributeFn.writable = False
    numericAttributeFn.readable = True
    robotIKGeneric.addAttribute(robotIKGeneric.ik_converged_attr)

    robotIKGeneric.ik_pos_error_attr = numericAttributeFn.create(
        'ikPositionError', 'ikPosErr', OpenMaya.MFnNumericData.kDouble, 0.0)
    numericAttributeFn.storable = False
    numericAttributeFn.writable = False
    numericAttributeFn.readable = True
    robotIKGeneric.addAttribute(robotIKGeneric.ik_pos_error_attr)

    #===================================#
    #    NODE ATTRIBUTE DEPENDENCIES    #
    #===================================#

    input_attrs = [
        robotIKGeneric.numJoints_attr,
        robotIKGeneric.jointAxis_attr,
        robotIKGeneric.jointOriginMatrix_attr,
        robotIKGeneric.jointLimitLower_attr,
        robotIKGeneric.jointLimitUpper_attr,
        robotIKGeneric.fk_attr,
        robotIKGeneric.tcp_mat_attr,
        robotIKGeneric.lcs_mat_attr,
        robotIKGeneric.target_mat_attr,
        robotIKGeneric.ik_attr,
        robotIKGeneric.rot_weight_attr,
    ]
    output_attrs = [
        robotIKGeneric.theta_attr,
        robotIKGeneric.ik_converged_attr,
        robotIKGeneric.ik_pos_error_attr,
    ]
    for input_attr in input_attrs:
        for output_attr in output_attrs:
            robotIKGeneric.attributeAffects(input_attr, output_attr)


def initializePlugin(mobject):
    mplugin = OpenMaya.MFnPlugin(mobject)
    try:
        mplugin.registerNode(kPluginNodeName, kPluginNodeId, nodeCreator,
                              nodeInitializer, OpenMaya.MPxNode.kDependNode, kPluginNodeClassify)
    except Exception:
        sys.stderr.write('Failed to register node: ' + kPluginNodeName)
        raise


def uninitializePlugin(mobject):
    mplugin = OpenMaya.MFnPlugin(mobject)
    try:
        mplugin.deregisterNode(kPluginNodeId)
    except Exception:
        sys.stderr.write('Failed to deregister node: ' + kPluginNodeName)
        raise
