/**
 * Test of the Path class.
 */
#include <gtest/gtest.h>
#include <path_follower/utils/path.h>

TEST(TestWaypoint, constructFromValues)
{
    Waypoint wp(13, 4.2, 2.3);

    ASSERT_EQ(13, wp.x);
    ASSERT_EQ(4.2, wp.y);
    ASSERT_EQ(2.3, wp.orientation);
}

TEST(TestWaypoint, constructionFromGmStampedPose)
{
    // init pose
    geometry_msgs::PoseStamped gm_pose;
    gm_pose.pose.position.x = 13;
    gm_pose.pose.position.y = 4.2;
    gm_pose.pose.orientation = tf::createQuaternionMsgFromYaw(2.3);

    // construct from pose and test
    Waypoint wp = gm_pose;
    ASSERT_EQ(13, wp.x);
    ASSERT_EQ(4.2, wp.y);
    ASSERT_EQ(2.3, wp.orientation);
}


TEST(TestWaypoint, castToGmPose)
{
    Waypoint wp;
    wp.x = 13;
    wp.y = 4.2;
    wp.orientation = 2.3;

    geometry_msgs::Pose gm_pose = wp;
    ASSERT_EQ(13, gm_pose.position.x);
    ASSERT_EQ(4.2, gm_pose.position.y);
    ASSERT_EQ(2.3, tf::getYaw(gm_pose.orientation));
}


TEST(TestWaypoint, castToEigenVector)
{
    Waypoint wp;
    wp.x = 13;
    wp.y = 4.2;
    wp.orientation = 2.3;

    Eigen::Vector2d vec = wp;
    ASSERT_EQ(13, vec[0]);
    ASSERT_EQ(4.2, vec[1]);
}


TEST(TestWaypoint, distanceTo)
{
    Waypoint wp1;
    wp1.x = 13;
    wp1.y = 4.2;
    wp1.orientation = 2.3;

    Waypoint wp2;
    wp2.x = 4.2;
    wp2.y = 3.14;
    wp2.orientation = 0.123;

    ASSERT_DOUBLE_EQ(8.86361100229472, wp1.distanceTo(wp2));
    ASSERT_DOUBLE_EQ(8.86361100229472, wp2.distanceTo(wp1));
}


//! Test fixture for testing the Path class
class TestPath : public ::testing::Test
{
protected:
    virtual void SetUp() {
        SubPath sp1;
        sp1.push_back(Waypoint(0,0,0));
        sp1.push_back(Waypoint(2,1,0));
        sp1.push_back(Waypoint(5,1,0));
        sp1.push_back(Waypoint(5,3,0));

        // Note: make sure, that sp2 has less points than sp1 so that the test fails if the
        //       precomputed remaining distances are not correctly initialized (this has been
        //       fixed in commit 0548da8).
        SubPath sp2;
        sp2.push_back(Waypoint(5,3,0));
        sp2.push_back(Waypoint(6,2,0));
        sp2.push_back(Waypoint(7,2,0));

        std::vector<SubPath> subpaths = {sp1, sp2};
        path_.setPath(subpaths);
    }

    Path path_;
};

bool operator==(Waypoint wp1, Waypoint wp2)
{
    return (wp1.x == wp2.x) && (wp1.y == wp2.y) && (wp1.orientation == wp2.orientation);
}


TEST_F(TestPath, testEmpty)
{
    Path empty_path;
    ASSERT_TRUE(empty_path.empty());
    ASSERT_FALSE(path_.empty());
}

TEST_F(TestPath, testClear)
{
    ASSERT_FALSE(path_.empty());
    path_.clear();
    ASSERT_TRUE(path_.empty());
}

//TODO: test waypoint callback

TEST_F(TestPath, testSubPathCount)
{
    ASSERT_EQ(2, path_.subPathCount());
    path_.clear();
    ASSERT_EQ(0, path_.subPathCount());
}

TEST_F(TestPath, testGetCurrentSubPath)
{
    // I am confident, that the returned sub path is correct, when lenght and first element fit
    SubPath sp = path_.getCurrentSubPath();
    ASSERT_EQ(4, sp.size());
    ASSERT_EQ(Waypoint(0,0,0), sp[0]);

    path_.switchToNextSubPath();
    sp = path_.getCurrentSubPath();
    ASSERT_EQ(3, sp.size());
    ASSERT_EQ(Waypoint(5,3,0), sp[0]);
}

TEST_F(TestPath, testGetWaypointByIndex)
{
    ASSERT_EQ(Waypoint(0,0,0), path_.getWaypoint(0));
    ASSERT_EQ(Waypoint(2,1,0), path_.getWaypoint(1));
    ASSERT_EQ(Waypoint(5,1,0), path_.getWaypoint(2));
    ASSERT_EQ(Waypoint(5,3,0), path_.getWaypoint(3));
    path_.switchToNextSubPath();
    ASSERT_EQ(Waypoint(5,3,0), path_.getWaypoint(0));
    ASSERT_EQ(Waypoint(6,2,0), path_.getWaypoint(1));
    ASSERT_EQ(Waypoint(7,2,0), path_.getWaypoint(2));
}

TEST_F(TestPath, testGetLastWaypoint)
{
    ASSERT_EQ(Waypoint(5,3,0), path_.getLastWaypoint());
    path_.switchToNextSubPath();
    ASSERT_EQ(Waypoint(7,2,0), path_.getLastWaypoint());
}

TEST_F(TestPath, testPathExecution)
{
    /* Walk along the path and check the following methods after each step where they are
     * defined:
     *
     *  - isDone
     *  - isSubPathDone
     *  - isLastWaypoint
     *  - getCurrentWaypoint
     *  - getRemainingSubPathDistance
     */

    ASSERT_EQ(Waypoint(0,0,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_FALSE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(7.2360678, path_.getRemainingSubPathDistance()); // 5 + sqrt(5)

    path_.switchToNextWaypoint(); // wp 2,1
    ASSERT_EQ(Waypoint(2,1,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_FALSE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(5, path_.getRemainingSubPathDistance());

    path_.switchToNextWaypoint(); // wp 5,1
    ASSERT_EQ(Waypoint(5,1,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_FALSE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(2, path_.getRemainingSubPathDistance());

    path_.switchToNextWaypoint(); // wp 5,3 -- last of sub path
    ASSERT_EQ(Waypoint(5,3,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_TRUE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(0, path_.getRemainingSubPathDistance());

    path_.switchToNextWaypoint(); // sub path end
    ASSERT_FALSE(path_.isDone());
    ASSERT_TRUE(path_.isSubPathDone());



    path_.switchToNextSubPath(); // wp 5,3
    ASSERT_EQ(Waypoint(5,3,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_FALSE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(2.4142137, path_.getRemainingSubPathDistance()); // 1 + sqrt(2)

    path_.switchToNextWaypoint(); // wp 6,2
    ASSERT_EQ(Waypoint(6,2,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_FALSE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(1, path_.getRemainingSubPathDistance());

    path_.switchToNextWaypoint(); // wp 7,2
    ASSERT_EQ(Waypoint(7,2,0), path_.getCurrentWaypoint());
    ASSERT_FALSE(path_.isDone());
    ASSERT_FALSE(path_.isSubPathDone());
    ASSERT_TRUE(path_.isLastWaypoint());
    ASSERT_FLOAT_EQ(0, path_.getRemainingSubPathDistance());

    path_.switchToNextWaypoint(); // sub path end
    ASSERT_FALSE(path_.isDone());
    ASSERT_TRUE(path_.isSubPathDone());

    path_.switchToNextSubPath(); // path end
    ASSERT_TRUE(path_.isDone());
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
