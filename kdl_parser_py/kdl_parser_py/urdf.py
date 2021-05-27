# Copyright 2019 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import urdf_parser_py.urdf as urdf

import PyKDL as kdl

def treeFromFile(filename):
    """
    Construct a PyKDL.Tree from an URDF file.

    :param filename: URDF file path
    """
    with open(filename) as urdf_file:
        return treeFromUrdfModel(urdf.URDF.from_xml_string(urdf_file.read()))

def treeFromString(xml):
    """
    Construct a PyKDL.Tree from an URDF xml string.

    :param xml: URDF xml string, ``str``
    """
    return treeFromUrdfModel(urdf.URDF.from_xml_string(xml))

def _toKdlPose(pose):
    # URDF might have RPY OR XYZ unspecified. Both default to zeros
    rpy = pose.rpy if pose and pose.rpy and len(pose.rpy) == 3 else [0, 0, 0]
    xyz = pose.xyz if pose and pose.xyz and len(pose.xyz) == 3 else [0, 0, 0]

    return kdl.Frame(
          kdl.Rotation.RPY(*rpy),
          kdl.Vector(*xyz))


def _toKdlInertia(i):
    # kdl specifies the inertia in the reference frame of the link, the urdf
    # specifies the inertia in the inertia reference frame
    origin = _toKdlPose(i.origin)
    inertia = i.inertia
    return origin.M * kdl.RigidBodyInertia(
            i.mass, origin.p,
            kdl.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz));

def _toKdlJoint(jnt):
    fixed = lambda j,F: kdl.Joint(j.name, kdl.Joint.JointType(8))
    rotational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.RotAxis)
    translational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.TransAxis)

    type_map = {
            'fixed': fixed,
            'revolute': rotational,
            'continuous': rotational,
            'prismatic': translational,
            'floating': fixed,
            'planar': fixed,
            'unknown': fixed,
            }

    return type_map[jnt.type](jnt, _toKdlPose(jnt.origin))

def _add_children_to_tree(robot_model, root, tree):
    # constructs the optional inertia
    inert = kdl.RigidBodyInertia(0)
    if root.inertial:
        inert = _toKdlInertia(root.inertial)

    # constructs the kdl joint
    (parent_joint_name, parent_link_name) = robot_model.parent_map[root.name]
    parent_joint = robot_model.joint_map[parent_joint_name]

    # construct the kdl segment
    sgm = kdl.Segment(
        root.name,
        _toKdlJoint(parent_joint),
        _toKdlPose(parent_joint.origin),
        inert)

    # add segment to tree
    if not tree.addSegment(sgm, parent_link_name):
        return False

    if root.name not in robot_model.child_map:
        return True

    children = [robot_model.link_map[l] for (j,l) in robot_model.child_map[root.name]]

    # recurslively add all children
    for child in children:
        if not _add_children_to_tree(robot_model, child, tree):
            return False

    return True;

def treeFromUrdfModel(robot_model, quiet=False):
    """
    Construct a PyKDL.Tree from an URDF model from urdf_parser_python.

    :param robot_model: URDF xml string, ``str``
    :param quiet: If true suppress messages to stdout, ``bool``
    """
    root = robot_model.link_map[robot_model.get_root()]

    if root.inertial and not quiet:
        print("The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF." % root.name);

    ok = True
    tree = kdl.Tree(root.name)

    #  add all children
    for (joint,child) in robot_model.child_map[root.name]:
        if not _add_children_to_tree(robot_model, robot_model.link_map[child], tree):
            ok = False
            break

    return (ok, tree)
