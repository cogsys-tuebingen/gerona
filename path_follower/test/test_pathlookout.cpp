#include <gtest/gtest.h>
#include <list>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <path_follower/supervisor/pathlookout.h>

using namespace std;

// Declare a test
TEST(TestPathLookout, clusterPoints)
{
    list<cv::Point2f> obs1; // size = 6
    obs1.push_back(cv::Point2f(1,1));
    obs1.push_back(cv::Point2f(1,3));
    obs1.push_back(cv::Point2f(2,1));
    obs1.push_back(cv::Point2f(2,3));
    obs1.push_back(cv::Point2f(3,2));
    obs1.push_back(cv::Point2f(4,2));

    list<cv::Point2f> obs2; // size = 5
    obs2.push_back(cv::Point2f(2,6));
    obs2.push_back(cv::Point2f(3,6));
    obs2.push_back(cv::Point2f(4,6));
    obs2.push_back(cv::Point2f(5,7));
    obs2.push_back(cv::Point2f(6,6));

    list<cv::Point2f> obs3; // size = 8
    obs3.push_back(cv::Point2f( 9,2));
    obs3.push_back(cv::Point2f(10,2));
    obs3.push_back(cv::Point2f(11,2));
    obs3.push_back(cv::Point2f(11,3));
    obs3.push_back(cv::Point2f(11,4));
    obs3.push_back(cv::Point2f(11,5));
    obs3.push_back(cv::Point2f(11,6));
    obs3.push_back(cv::Point2f(13,2));

    list<cv::Point2f> obs4; // size = 1
    obs4.push_back(cv::Point2f(16,4));

    list<cv::Point2f> obs5; // size = 2
    obs5.push_back(cv::Point2f(11,8.5));
    obs5.push_back(cv::Point2f(11,10));

    list<list<cv::Point2f> > ordered_obstacles;
    ordered_obstacles.push_back(obs4);
    ordered_obstacles.push_back(obs5);
    ordered_obstacles.push_back(obs2);
    ordered_obstacles.push_back(obs1);
    ordered_obstacles.push_back(obs3);


    // merge all obstacles together
    list<cv::Point2f> points;
    points.splice(points.end(), obs1);
    points.splice(points.end(), obs2);
    points.splice(points.end(), obs3);
    points.splice(points.end(), obs4);
    points.splice(points.end(), obs5);

    // shuffle points
    vector<cv::Point2f> v_points(points.begin(), points.end());
    random_shuffle(v_points.begin(), v_points.end());
    points.assign(v_points.begin(), v_points.end());

    // cluster with threshold 2
    list<list<cv::Point2f> > clusters = PathLookout::clusterPoints(points, 2);

    // expect 5 clusters
    ASSERT_EQ(5, clusters.size());

    // order clusters by size
    clusters.sort([](list<cv::Point2f> &a, list<cv::Point2f> &b) { return a.size() < b.size(); });

    // order each cluster by (x,y)
    for_each(clusters.begin(), clusters.end(), [](list<cv::Point2f> &c) {
        c.sort([](cv::Point2f &a, cv::Point2f &b){
            return (a.x < b.x) || ((a.x == b.x) && (a.y < b.y));
        });
    });

    // now clusters and ordered_obstacles should be equal
    auto c_it = clusters.cbegin();
    auto o_it = ordered_obstacles.cbegin();
    for (; c_it != clusters.end(); ++c_it, ++o_it) {
        ASSERT_EQ(c_it->size(), o_it->size());
        auto cp_it = c_it->cbegin();
        auto op_it = o_it->cbegin();
        for (; cp_it != c_it->end(); ++cp_it, ++op_it) {
            ASSERT_EQ(*cp_it, *op_it);
        }
    }
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
