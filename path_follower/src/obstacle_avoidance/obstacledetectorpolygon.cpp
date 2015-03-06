#include <path_follower/obstacle_avoidance/obstacledetectorpolygon.h>

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.h>
#include <path_follower/utils/visualizer.h>

using namespace std;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "obstacle_avoider";
}

bool ObstacleDetectorPolygon::checkOnCloud(ObstacleCloud::ConstPtr obstacles, float width, float length, float course_angle, float curve_enlarge_factor)
{
    bool collision = false;
    PolygonWithTfFrame pwf = getPolygon(width, length, course_angle, curve_enlarge_factor);


    if (pwf.polygon.size() == 0) {
        ROS_WARN_NAMED(MODULE, "Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }



    ObstacleCloud cloud;
    try {
        //TODO: it would be faster to transform the (comparatively small) polygon instead of the
        //      whole point cloud
        pcl_ros::transformPointCloud(pwf.frame, *obstacles, cloud, *tf_listener_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_NAMED(MODULE, "Failed to transform obstacle cloud to polygon frame: %s", ex.what());
        // can't check for obstacles, so better assume there is one.
        return true;
    }


    /// now check each point of the scan
    ObstacleCloud::const_iterator point_it;
    for (point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
        // check if this scan point is inside the polygon
        cv::Point2f point( point_it->x, point_it->y );

        if (cv::pointPolygonTest(pwf.polygon, point, false) == 1) {
            collision = true;
            break; // no need to check the remaining points
        }
    }


    // visualization
    visualize(pwf, collision);

    return collision;
}

void ObstacleDetectorPolygon::visualize(ObstacleDetectorPolygon::PolygonWithTfFrame polygon,
                                        bool hasObstacle) const
{
    Visualizer* vis = Visualizer::getInstance();
    if (vis->hasSubscriber()) {
        // push first element to back, to close the polygon.
        polygon.polygon.push_back(polygon.polygon.front());

        vector<cv::Point2f>::iterator it = polygon.polygon.begin();
        cv::Point2f p1 = *it;

        int marker_id = 0;
        for (++it; it != polygon.polygon.end(); ++it) {
            cv::Point2f p2 = *it;

            // convert cv::Point2f to ros points...
            geometry_msgs::Point gp1, gp2;
            gp1.x = p1.x;  gp1.y = p1.y;
            gp2.x = p2.x;  gp2.y = p2.y;

            // colour is green when the box is empty and red if there is an obstacle
            float r = hasObstacle ? 1 : 0;
            float g = 1 - r;
            vis->drawLine(marker_id++, gp1, gp2, polygon.frame, "collision_box", r,g,0, 0.1, 0.05);

            p1 = p2;
        }
    }
}
