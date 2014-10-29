#include <gtest/gtest.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>

namespace {

class testSelfCollisionAvoidance : public ::testing::Test{
 protected:

  testSelfCollisionAvoidance()
  {}

  virtual ~testSelfCollisionAvoidance()
  {}

  virtual void SetUp()
  {}

  virtual void TearDown()
  {}
};

// Tests that the Foo::getLowerBounds() are zero at the bounds
TEST_F(testSelfCollisionAvoidance, computeKeypoints) {
    iDynUtils robot;
    yarp::sig::Vector q(robot.coman_iDyn3.getNrOfDOFs(), 0.0);
    robot.updateiDyn3Model(q, true);

    OpenSoT::constraints::velocity::SelfCollisionAvoidance self_collision_avoidance(q, robot);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
