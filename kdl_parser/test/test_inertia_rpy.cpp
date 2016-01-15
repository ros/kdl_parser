
#include <string>
#include <gtest/gtest.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include "kdl_parser/kdl_parser.hpp"

using namespace kdl_parser;

int g_argc;
char** g_argv;

class TestInertiaRPY : public testing::Test
{
public:

protected:
  /// constructor
  TestInertiaRPY()
  {
  }


  /// Destructor
  ~TestInertiaRPY()
  {
  }
};


TEST_F(TestInertiaRPY, test_torques)
{
  //ASSERT_EQ(g_argc, 3);

  // workaround for segfault issue with parsing 2 trees instantiated on the stack
  KDL::Tree * tree_1 = new KDL::Tree;
  KDL::Tree * tree_2 = new KDL::Tree;
  KDL::JntArray torques_1;
  KDL::JntArray torques_2;

  {
    ASSERT_TRUE(treeFromFile(g_argv[1], *tree_1));
    KDL::Vector gravity(0, 0, -9.81);
    KDL::Chain chain;
    std::cout << "number of joints: " << tree_1->getNrOfJoints() << std::endl;
    std::cout << "number of segments: " << tree_1->getNrOfSegments() << std::endl;

    ASSERT_TRUE(tree_1->getChain("base_link", "link2", chain));
    KDL::ChainIdSolver_RNE solver(chain, gravity);
    //JntArrays get initialized with all 0 values
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qdot(chain.getNrOfJoints());
    KDL::JntArray qdotdot(chain.getNrOfJoints());
    //KDL::JntArray torques(chain.getNrOfJoints());
    std::vector<KDL::Wrench> wrenches(chain.getNrOfJoints());
    solver.CartToJnt(q, qdot, qdotdot, wrenches, torques_1);

    delete tree_1;
    tree_1 = NULL;
  }

  {
    ASSERT_TRUE(treeFromFile(g_argv[2], *tree_2));
    KDL::Vector gravity(0, 0, -9.81);
    KDL::Chain chain;

    ASSERT_TRUE(tree_2->getChain("base_link", "link2", chain));
    KDL::ChainIdSolver_RNE solver(chain, gravity);
    //JntArrays get initialized with all 0 values
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qdot(chain.getNrOfJoints());
    KDL::JntArray qdotdot(chain.getNrOfJoints());
    //KDL::JntArray torques(chain.getNrOfJoints());
    std::vector<KDL::Wrench> wrenches(chain.getNrOfJoints());
    solver.CartToJnt(q, qdot, qdotdot, wrenches, torques_2);

    delete tree_2;
    tree_2 = NULL;
  }

  ASSERT_TRUE(torques_1 == torques_2);

  SUCCEED();
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_kdl_parser");
  for (size_t i = 0; i < argc; ++i) {
    std::cout << argv[i] << std::endl;
  }
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
