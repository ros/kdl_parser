#!/usr/bin/env python

import sys
import os
import ament_index_python as ament_index

import kdl_parser_py.urdf
import unittest

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
