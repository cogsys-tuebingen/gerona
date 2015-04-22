/**
 * Test of PidController.
 */
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <path_follower/utils/pidcontroller.hpp>

TEST(TestPidController, timing)
{
    // test timing
    PidController<1> pid(1,0,0,1);
    float u;

    ASSERT_FALSE(pid.execute(0, &u));  // first call is too early

    std::this_thread::sleep_for(std::chrono::seconds(1));  // sleep for one second

    ASSERT_TRUE(pid.execute(0, &u));  // now it is okay
    ASSERT_FALSE(pid.execute(0, &u)); // no new command yet

    std::this_thread::sleep_for(std::chrono::seconds(1));  // sleep for one second

    ASSERT_TRUE(pid.execute(0, &u));  // now enought time has passed to expect a `true`
    ASSERT_FALSE(pid.execute(0, &u)); // and again no new command yet
}

TEST(Test1dPidController, pureP)
{
    float dt = 0.1;
    // sleep_duration = 1.5*dt to make sure, the pause is long enough.
    auto sleep_duration = std::chrono::milliseconds(int(dt*1000 * 1.5));

    PidController<1> pid(1,0,0,dt);

    std::vector<float> errors = {0,1,-1.5,2,-4,13,23,42,3.1415};
    for (float err: errors) {
        std::this_thread::sleep_for(sleep_duration);
        float u;
        bool ret = pid.execute(err, &u);
        ASSERT_TRUE(ret);
        ASSERT_FLOAT_EQ(err, u);  // with K_p = 1, u == err
    }


    // test with K_p != 1
    pid = PidController<1>(3.1415,0,0,dt);

    for (float err: errors) {
        std::this_thread::sleep_for(sleep_duration);
        float u;
        bool ret = pid.execute(err, &u);
        ASSERT_TRUE(ret);
        float expect = err * 3.1415;
        ASSERT_FLOAT_EQ(expect, u);  // with K_p = x, u == x*err
    }
}

TEST(Test1dPidController, pureI)
{
    float u;
    PidController<1> pid(0,3,0,.5);

    // Use ASSERT_NEAR with high tolerance, as the results does not directly depend on dt but
    // on the actuall elapsed time, which can not be determined exactly.

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(pid.execute(1, &u));
    ASSERT_NEAR(1.5, u, 0.02);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(pid.execute(1, &u));
    ASSERT_NEAR(3, u, 0.02);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(pid.execute(.5, &u));
    ASSERT_NEAR(3.75, u, 0.02);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_TRUE(pid.execute(3, &u));
    ASSERT_NEAR(8.25, u, 0.02);
}

TEST(Test1dPidController, pureD)
{
    float u;
    PidController<1> pid(0,0,2,1);

    // Use ASSERT_NEAR with high tolerance, as the results does not directly depend on dt but
    // on the actuall elapsed time, which can not be determined exactly.

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(1, &u));
    ASSERT_NEAR(2, u, 0.02);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(1, &u));
    ASSERT_NEAR(0, u, 0.02);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(.5, &u));
    ASSERT_NEAR(-1, u, 0.02);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(3, &u));
    ASSERT_NEAR(5, u, 0.02);
}

TEST(Test1dPidController, fullWithReset)
{
    float u;
    PidController<1> pid(1,1,1,1);

    // Use ASSERT_NEAR with high tolerance, as the results does not directly depend on dt but
    // on the actuall elapsed time, which can not be determined exactly.

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(1, &u));
    ASSERT_NEAR(3, u, 0.02);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(2, &u));
    ASSERT_NEAR(6, u, 0.02);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(4, &u));
    ASSERT_NEAR(13, u, 0.02);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(0, &u));
    ASSERT_NEAR(3, u, 0.02);

    pid.reset();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(-1, &u));
    ASSERT_NEAR(-3, u, 0.02);
}


TEST(Test3dPidController, simpleTest)
{
    typedef PidController<3>::Vector Vec;
    Vec u;

    PidController<3> pid(Vec(1,0,0),
                         Vec(0,1,0),
                         Vec(0,0,1),
                         1);


    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(Vec::Constant(1), &u));
    ASSERT_TRUE(u.isApprox(Vec(1,1,1), 0.02));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(Vec::Constant(2), &u));
    ASSERT_TRUE(u.isApprox(Vec(2,3,1), 0.02));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(Vec::Constant(4), &u));
    ASSERT_TRUE(u.isApprox(Vec(4,7,2), 0.02));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(Vec::Constant(0), &u));
    ASSERT_TRUE(u.isApprox(Vec(0,7,-4), 0.02));

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(pid.execute(Vec::Constant(-1), &u));
    ASSERT_TRUE(u.isApprox(Vec(-1,6,-1), 0.02));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
