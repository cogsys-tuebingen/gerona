#include <gtest/gtest.h>
#include <path_follower/utils/pidcontroller.hpp>

TEST(TestPidController, simpleTest)
{
    //PidController<1> pid(1,0,0,1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
