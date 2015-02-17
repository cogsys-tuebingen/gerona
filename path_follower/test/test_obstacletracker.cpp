#include <gtest/gtest.h>
#include <ros/ros.h>
#include <list>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <path_follower/supervisor/pathlookout.h>

using namespace std;

//! Compare two obstacles
bool operator==(const Obstacle& o1, const Obstacle& o2)
{
    return (o1.center == o2.center) && (o1.radius == o2.radius);
}

TEST(TestPathLookout, obstacleTracker)
{
    typedef ObstacleTracker::TrackedObstacle TrackedObstacle;

    Obstacle obs1;
    obs1.center = cv::Point2f(1,1);
    obs1.radius = 0.3;

    Obstacle obs2;
    obs2.center = cv::Point2f(5,5);
    obs2.radius = 0.3;

    Obstacle obs3;
    obs3.center = cv::Point2f(1,2);
    obs3.radius = 0.3;

    vector<Obstacle> v_obs = {obs1, obs2, obs3};

    ObstacleTracker tracker;
    tracker.setMaxDist(1);

    // first observation
    tracker.update(v_obs);
    const std::vector<TrackedObstacle> tracked0 = tracker.getTrackedObstacles();

    ASSERT_EQ(3, tracked0.size());
    ASSERT_EQ(tracked0[0].obstacle(), obs1);
    ASSERT_EQ(tracked0[1].obstacle(), obs2);
    ASSERT_EQ(tracked0[2].obstacle(), obs3);


    // obstacles did not move
    tracker.update(v_obs);
    const std::vector<TrackedObstacle> tracked1 = tracker.getTrackedObstacles();

    ASSERT_EQ(3, tracked1.size());
    // obstacles should still be equal (moving moved)
    ASSERT_EQ(tracked1[0].obstacle(), obs1);
    ASSERT_EQ(tracked1[1].obstacle(), obs2);
    ASSERT_EQ(tracked1[2].obstacle(), obs3);
    // time of first sight should not change
    ASSERT_EQ(tracked0[0].time_of_first_sight(), tracked1[0].time_of_first_sight());
    ASSERT_EQ(tracked0[1].time_of_first_sight(), tracked1[1].time_of_first_sight());
    ASSERT_EQ(tracked0[2].time_of_first_sight(), tracked1[2].time_of_first_sight());
    // last sight is expected to be greater than before
    ASSERT_LT(tracked0[0].time_of_last_sight(), tracked1[0].time_of_last_sight());
    ASSERT_LT(tracked0[1].time_of_last_sight(), tracked1[1].time_of_last_sight());
    ASSERT_LT(tracked0[2].time_of_last_sight(), tracked1[2].time_of_last_sight());

    // obs1 moves to far to be tracked
    v_obs[0].center = cv::Point2f(10,10);
    tracker.update(v_obs);
    const std::vector<TrackedObstacle> tracked2 = tracker.getTrackedObstacles();

    // old obs 1 should still be tracked
    ASSERT_EQ(4, tracked2.size());
    ASSERT_EQ(tracked2[0].obstacle(), obs1);
    ASSERT_EQ(tracked2[1].obstacle(), obs2);
    ASSERT_EQ(tracked2[2].obstacle(), obs3);
    ASSERT_EQ(tracked2[3].obstacle().center, cv::Point2f(10,10));
    // time of first sight should not change
    ASSERT_EQ(tracked0[0].time_of_first_sight(), tracked2[0].time_of_first_sight());
    ASSERT_EQ(tracked0[1].time_of_first_sight(), tracked2[1].time_of_first_sight());
    ASSERT_EQ(tracked0[2].time_of_first_sight(), tracked2[2].time_of_first_sight());
    // no new observation for obstacle 0
    ASSERT_EQ(tracked1[0].time_of_last_sight(), tracked2[0].time_of_last_sight());
    // the new obstacle has a greater time of first sight
    ASSERT_LT(tracked2[2].time_of_first_sight(), tracked2[3].time_of_first_sight());
    // for other obstacles, last sight is expected to be greater than before
    ASSERT_LT(tracked1[1].time_of_last_sight(), tracked2[1].time_of_last_sight());
    ASSERT_LT(tracked1[2].time_of_last_sight(), tracked2[2].time_of_last_sight());


    // obs2 moves a bit
    v_obs[1].center = cv::Point2f(5.1,5.4);
    tracker.update(v_obs);
    const std::vector<TrackedObstacle> tracked3 = tracker.getTrackedObstacles();
    ASSERT_FALSE(tracked3[1].obstacle() == obs2);
    ASSERT_EQ(tracked3[1].obstacle().center, cv::Point2f(5.1,5.4));
    ASSERT_EQ(tracked0[1].time_of_first_sight(), tracked3[1].time_of_first_sight());
    ASSERT_LT(tracked2[1].time_of_last_sight(), tracked3[1].time_of_last_sight());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_obstacletracker");
  ros::start();
  return RUN_ALL_TESTS();
}
