# Copyright (c) 2021 Open Source Robotics Foundation, Inc.
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

import os
import sys
import unittest

import ament_index_python as ament_index

import kdl_parser_py.urdf


PACKAGE_NAME = "kdl_parser_py"


class TestKdlParser(unittest.TestCase):
    def runTest(self):
        filename = None
        if len(sys.argv) > 1:
            filename = sys.argv[1]
        else:
            urdf_dir = ament_index.get_package_prefix(PACKAGE_NAME)
            filename = os.path.join(
                urdf_dir, "share", PACKAGE_NAME, "assets", "test.urdf"
            )
            if not os.path.isfile(filename):
                self.fail("Expected filename!")
        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        self.assertTrue(ok)
        # KDL doesn't count fixed joints (since they aren't kinematic)
        self.assertEqual(tree.getNrOfJoints(), 8)
        # KDL doesn't count base link (since it's attached by fixed links
        self.assertEqual(tree.getNrOfSegments(), 10)
        chain = tree.getChain("base_link", "right_gripper")
        self.assertEqual(chain.getNrOfSegments(), 2)
        self.assertEqual(chain.getNrOfJoints(), 2)
        self.assertEqual(chain.getSegment(0).getName(), "gripper_pole")
        self.assertEqual(chain.getSegment(0).getJoint().getName(), "gripper_extension")
        self.assertEqual(chain.getSegment(1).getName(), "right_gripper")
        self.assertEqual(
            chain.getSegment(1).getJoint().getName(), "right_gripper_joint"
        )

        inertia = chain.getSegment(1).getInertia()
        self.assertAlmostEqual(inertia.getCOG().z(), 3.0)


if __name__ == "__main__":
    unittest.main()
