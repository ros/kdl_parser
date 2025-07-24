// Copyright (c) 2015, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Wim Meeussen */

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "kdl/jntarray.hpp"
#include "kdl_parser/kdl_parser.hpp"

const char model1[] =
  "<?xml version=\"1.0\" ?>"
  "<robot name=\"testInertialRPYmodel1\">"
  "  <link name=\"base_link\" />"
  "  <joint name=\"base_fixed_joint\" type=\"fixed\">"
  "    <origin xyz=\"0 0 0\" rpy=\"0 -0 0\" />"
  "    <axis xyz=\"0 0 0\" />"
  "    <parent link=\"base_link\" />"
  "    <child link=\"link1\" />"
  "  </joint>"
  "  <link name=\"link1\">"
  "    <inertial>"
  "      <mass value=\"1\" />"
  "      <inertia ixx=\"0.01\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.01\" iyz=\"0.0\" izz=\"0.01\" />"
  "    </inertial>"
  "  </link>"
  "  <joint name=\"joint_1_2\" type=\"continuous\">"
  "    <origin xyz=\"0 0 0\" rpy=\"0 -0 0\" />"
  "    <axis xyz=\"0 0 1\" />"
  "    <parent link=\"link1\" />"
  "    <child link=\"link2\" />"
  "  </joint>"
  "  <link name=\"link2\">"
  "    <inertial>"
  "      <mass value=\"100\" />"
  "      <origin xyz=\"1 2 3\" rpy=\"0 -0 0\" />"
  "      <inertia ixx=\"5\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"15\" iyz=\"0.0\" izz=\"25\" />"
  "    </inertial>"
  "  </link>"
  "</robot>";

const char model2[] =
  "<?xml version=\"1.0\" ?>"
  "<robot name=\"testInertialRPYmodel2\">"
  "  <link name=\"base_link\" />"
  "  <joint name=\"base_fixed_joint\" type=\"fixed\">"
  "    <origin xyz=\"0 0 0\" rpy=\"0 -0 0\" />"
  "    <axis xyz=\"0 0 0\" />"
  "    <parent link=\"base_link\" />"
  "    <child link=\"link1\" />"
  "  </joint>"
  "  <link name=\"link1\">"
  "    <inertial>"
  "      <mass value=\"1\" />"
  "      <inertia ixx=\"0.01\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.01\" iyz=\"0.0\" izz=\"0.01\" />"
  "    </inertial>"
  "  </link>"
  "  <joint name=\"joint_1_2\" type=\"continuous\">"
  "    <origin xyz=\"0 0 0\" rpy=\"0 -0 0\" />"
  "    <axis xyz=\"0 0 1\" />"
  "    <parent link=\"link1\" />"
  "    <child link=\"link2\" />"
  "  </joint>"
  "  <link name=\"link2\">"
  "    <inertial>"
  "      <mass value=\"100\" />"
  "      <origin xyz=\"1 2 3\" rpy=\"3.141592653589793 0 0\" />"
  "      <inertia ixx=\"5\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"15\" iyz=\"0.0\" izz=\"25\" />"
  "    </inertial>"
  "  </link>"
  "</robot>";

TEST(TestInertiaRPY, test_torques)
{
  // workaround for segfault issue with parsing 2 trees instantiated on the stack
  std::unique_ptr<KDL::Tree> tree_1 = std::make_unique<KDL::Tree>();
  std::unique_ptr<KDL::Tree> tree_2 = std::make_unique<KDL::Tree>();
  KDL::JntArray torques_1;
  KDL::JntArray torques_2;

  {
    ASSERT_TRUE(kdl_parser::treeFromString(model1, *tree_1));
    KDL::Vector gravity(0, 0, -9.81);
    KDL::Chain chain;

    ASSERT_TRUE(tree_1->getChain("base_link", "link2", chain));
    KDL::ChainIdSolver_RNE solver(chain, gravity);
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qdot(chain.getNrOfJoints());
    KDL::JntArray qdotdot(chain.getNrOfJoints());
    std::vector<KDL::Wrench> wrenches(chain.getNrOfJoints());
    solver.CartToJnt(q, qdot, qdotdot, wrenches, torques_1);
  }

  {
    ASSERT_TRUE(kdl_parser::treeFromString(model2, *tree_2));
    KDL::Vector gravity(0, 0, -9.81);
    KDL::Chain chain;

    ASSERT_TRUE(tree_2->getChain("base_link", "link2", chain));
    KDL::ChainIdSolver_RNE solver(chain, gravity);
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qdot(chain.getNrOfJoints());
    KDL::JntArray qdotdot(chain.getNrOfJoints());
    std::vector<KDL::Wrench> wrenches(chain.getNrOfJoints());
    solver.CartToJnt(q, qdot, qdotdot, wrenches, torques_2);
  }

  ASSERT_TRUE(torques_1 == torques_2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
