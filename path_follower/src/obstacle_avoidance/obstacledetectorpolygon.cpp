#include <path_follower/obstacle_avoidance/obstacledetectorpolygon.h>
#include <path_follower/utils/obstacle_cloud.h>

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <path_follower/utils/visualizer.h>

using namespace std;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "obstacle_avoider";
}

bool ObstacleDetectorPolygon::checkOnCloud(std::shared_ptr<ObstacleCloud const> obstacles_container, float width, float length, float course_angle, float curve_enlarge_factor)
{
    ObstacleCloud::Cloud::ConstPtr obstacles = obstacles_container->cloud;

    bool collision = false;
    PolygonWithTfFrame pwf = getPolygon(width, length, course_angle, curve_enlarge_factor);

    if (pwf.polygon.size() == 0) {
        ROS_WARN_NAMED(MODULE, "Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }

    if(obstacles->header.frame_id != pwf.frame) {
        /// transform the polygon to the obstacle cloud frame
        try {
            geometry_msgs::PointStamped gm_p, gm_pt;
            gm_p.header.frame_id = pwf.frame;
            gm_p.header.stamp = pcl_conversions::fromPCL(obstacles->header.stamp);
    
            for (cv::Point2f &p : pwf.polygon) {
                gm_p.point.x = p.x;
                gm_p.point.y = p.y;
                tf_listener_->transformPoint(obstacles->header.frame_id, gm_p, gm_pt);
                p.x = gm_pt.point.x;
                p.y = gm_pt.point.y;
            }
            pwf.frame = obstacles->header.frame_id;

        } catch (tf::TransformException& ex) {
            ROS_ERROR_NAMED(MODULE, "Failed to transform polygon to obstacle cloud frame: %s", ex.what());
            // can't check for obstacles, so better assume there is one.
            return true;
        }
    }

    /// now check each point of the scan
    for (auto point_it = obstacles->begin(); point_it != obstacles->end(); ++point_it) {
        // check if this scan point is inside the polygon
        cv::Point2f point( point_it->x, point_it->y );

        if (cv::pointPolygonTest(pwf.polygon, point, false) > 0.5) {
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
