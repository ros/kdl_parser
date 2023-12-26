#!/usr/bin/env python

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

import kdl_parser_py.urdf
import unittest
from ament_index_python.packages import get_package_share_directory

PKG = "kdl_parser_py"

class TestKdlParser(unittest.TestCase):
    def runTest(self):
        package_dir = get_package_share_directory(PKG)

        filename  = os.path.join(package_dir, 'test.urdf')
        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        self.assertTrue(ok)
        # KDL doesn't count fixed joints (since they aren't kinematic)
        self.assertEqual(tree.getNrOfJoints(), 8)
        # KDL doesn't count base link (since it's attached by fixed links
        self.assertEqual(tree.getNrOfSegments(), 10)
        chain = tree.getChain('base_link', 'right_gripper')
        self.assertEqual(chain.getNrOfSegments(), 2)
        self.assertEqual(chain.getNrOfJoints(), 2)
        self.assertEqual(chain.getSegment(0).getName(), 'gripper_pole')
        self.assertEqual(chain.getSegment(0).getJoint().getName(), 'gripper_extension')
        self.assertEqual(chain.getSegment(1).getName(), 'right_gripper')
        self.assertEqual(chain.getSegment(1).getJoint().getName(), 'right_gripper_joint')

        inertia = chain.getSegment(1).getInertia()
        self.assertAlmostEqual(inertia.getCOG().z(), 3.0)

if __name__ == '__main__':
    unittest.main()
