#include <path_follower/obstacle_avoidance/obstacledetectorpolygon.h>

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <path_follower/utils/visualizer.h>

using namespace std;

void ObstacleDetectorPolygon::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    ObstacleDetector::setMap(map);
    map_trans_.setMap(map);
}

bool ObstacleDetectorPolygon::checkOnMap(float width, float length, float course_angle, float curve_enlarge_factor)
{
    bool collision = false;

    #if DEBUG_PATHLOOKOUT
        cv::Mat debug(map_->info.height, map_->info.width, CV_8UC1, cv::Scalar::all(255));
        cv::namedWindow("ObstacleBox", CV_WINDOW_KEEPRATIO);
    #endif

    PolygonWithTfFrame pwf = getPolygon(width, length, course_angle, curve_enlarge_factor);
    PolygonWithTfFrame poly_for_viz = pwf;

    try {
        transformPolygonToMap(&pwf);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Error with transform obstacle polygon: %s", ex.what());
        // can't check for obstacles, so better assume there is one.
        return true;
    } catch (const std::runtime_error &ex) { // is thrown, if no obstacle map is available.
        ROS_ERROR("An error occured: %s", ex.what());
        return false;
    }

    vector<cv::Point2f> polygon = pwf.polygon;

    if (polygon.size() == 0) {
        ROS_WARN("Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }


    const unsigned data_size = map_->info.height * map_->info.width;
    for (unsigned i = 0; i < data_size; ++i) {

        #if DEBUG_PATHLOOKOUT
            debug.data[i] = map_->data[i] == OCCUPIED ? 127 : 255;
        #endif

        if (map_->data[i] == OCCUPIED) {
            // check if this map point is inside the polygon
            cv::Point2f point( i % map_->info.width, i / map_->info.width );

            if (cv::pointPolygonTest(polygon, point, false) == 1) {
                collision = true;
                break; // no need to check the remaining cells
            }
        }
    }


    #if DEBUG_PATHLOOKOUT
        for (size_t i = 1; i < polygon.size(); ++i) {
            cv::line(debug, polygon[i-1], polygon[i], cv::Scalar(0));
        }
        cv::line(debug, polygon.front(), polygon.back(), cv::Scalar(0));
        cv::imshow("ObstacleBox", debug);
        cv::waitKey(10);
    #endif


    // visualization
    visualize(poly_for_viz, collision);

    return collision;
}

bool ObstacleDetectorPolygon::checkOnScan(const sensor_msgs::LaserScanConstPtr &scan, float width, float length, float course_angle, float curve_enlarge_factor)
{
    bool collision = false;
    PolygonWithTfFrame pwf = getPolygon(width, length, course_angle, curve_enlarge_factor);


    if (pwf.polygon.size() == 0) {
        ROS_WARN("Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }

    /// transform scan to point cloud in polygon frame
    if(!tf_listener_.waitForTransform(
                scan->header.frame_id,
                pwf.frame,
                scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
                ros::Duration(0.5))) {
        ROS_WARN("Got no transform from scan to polygon. No obstacle check is done!");
        return false;
    }

    sensor_msgs::PointCloud cloud;
    try {
        laser_projector_.transformLaserScanToPointCloud(pwf.frame, *scan, cloud, tf_listener_);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Error with transform scan to obstacle polygon: %s", ex.what());
        // can't check for obstacles, so better assume there is one.
        return true;
    }


    /// now check each point of the scan
    vector<geometry_msgs::Point32>::const_iterator iter;
    for (iter = cloud.points.begin(); iter != cloud.points.end(); ++iter) {
        // check if this scan point is inside the polygon
        cv::Point2f point( iter->x, iter->y );

        if (cv::pointPolygonTest(pwf.polygon, point, false) == 1) {
            collision = true;
            break; // no need to check the remaining points
        }
    }


    // visualization
    visualize(pwf, collision);

    return collision;
}

bool ObstacleDetectorPolygon::checkOnCloud(ObstacleCloud::ConstPtr obstacles, float width, float length, float course_angle, float curve_enlarge_factor)
{
    bool collision = false;
    PolygonWithTfFrame pwf = getPolygon(width, length, course_angle, curve_enlarge_factor);


    if (pwf.polygon.size() == 0) {
        ROS_WARN("Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }



    ObstacleCloud cloud;
    try {
        tf_listener_.transformPointCloud(pwf.frame, *obstacles, cloud);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Failed to transform obstacle cloud to polygon frame: %s", ex.what());
        // can't check for obstacles, so better assume there is one.
        return true;
    }


    /// now check each point of the scan
    vector<geometry_msgs::Point32>::const_iterator iter;
    for (iter = cloud.points.begin(); iter != cloud.points.end(); ++iter) {
        // check if this scan point is inside the polygon
        cv::Point2f point( iter->x, iter->y );

        if (cv::pointPolygonTest(pwf.polygon, point, false) == 1) {
            collision = true;
            break; // no need to check the remaining points
        }
    }


    // visualization
    visualize(pwf, collision);

    return collision;
}

void ObstacleDetectorPolygon::transformPolygonToMap(PolygonWithTfFrame *polygon) const
{
    // transform each point
    vector<cv::Point2f>::iterator iter;
    for (iter = polygon->polygon.begin(); iter != polygon->polygon.end(); ++iter) {
        *iter = map_trans_.transformPointToMap(*iter, polygon->frame);
    }
    // set new frame
    polygon->frame = map_->header.frame_id;
}

void ObstacleDetectorPolygon::visualize(ObstacleDetectorPolygon::PolygonWithTfFrame polygon, bool hasObstacle) const
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
