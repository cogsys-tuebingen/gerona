#include "pathlookout.h"

#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;

PathLookout::PathLookout()
{
    cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);

    visualizer_ = Visualizer::getInstance();
}

void PathLookout::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
    map_trans_.setMap(map);


    // convert to image
    map_image_ = cv::Mat(map->info.height, map->info.width, CV_8UC1);
    // Directly copy. Now 0 -> free, >0 -> obstacle. For visualization, it would usually be inverted, here, however,
    // this is perfectly fine, as the obstacles have to be "true" for the intersection with the path image.
    // Note, that it is important, that the path image is then really binary with only 0 and 255, to make sure, that the
    // bitwise_and can not miss any obstacle pixels.
    std::copy(map->data.begin(), map->data.end(), map_image_.data);
    //map_image_.data = &map->data[0];  <- no copy, only references. Would be nice, but yields problems with const...

    // make sure that the path image is of the same size than the map
    path_image_ = cv::Mat(map->info.height, map->info.width, CV_8UC1, cv::Scalar(0));
}

void PathLookout::setPath(const Path &path)
{
    //path_ = path;
    drawPathToImage(path);
}

bool PathLookout::lookForObstacles()
{
    //FIXME: This does currently not take the postion of the robot into account (thus also obstacles behind the robot
    // can stop the robot).

    if (map_ == NULL) {
        ROS_WARN("PathLookout has not received any map yet. No obstacle lookout is done.");
        return false;
    }

    // Calculate intersection of the obstacle map and the path ==> provides the obstacles on the path
    cv::Mat intersect;
    cv::bitwise_and(map_image_, path_image_, intersect);

    // debug
    cv::imshow("Map", map_image_);
    cv::imshow("Path", path_image_);
    cv::imshow("Intersection", intersect);
    cv::waitKey(5);

    // find obstacle contours on the path
    vector<vector<cv::Point> > contours;
    cv::findContours(intersect, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Get center of each obstacle (for tracking)
    vector<cv::Point2f> obstacle_centers;
    obstacle_centers.reserve(contours.size());
    for(size_t i = 0; i < contours.size(); ++i) {
        try {
            // caclulate center of mass, using moments.
            cv::Moments mom = cv::moments(contours[i], true);
            cv::Point2f center(mom.m10/mom.m00, mom.m01/mom.m00);
            center = map_trans_.transformPointFromMap(center, "/base_link");

            obstacle_centers.push_back(center);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("TF-Error. Could not transform obstacle position. %s", ex.what());
        }
    }

    //TODO: Track the obstacles and weight them, depening on how long they are on the path, and how far away they are.

    tracker_.update(obstacle_centers);

    vector<ObstacleTracker::TrackedObstacle> tracked_obs = tracker_.getTrackedObstacles();

    if (tracked_obs.empty()) {
        return false;
    }

    vector<float> weights;
    weights.resize(tracked_obs.size());
    transform(tracked_obs.begin(), tracked_obs.end(), weights.begin(), boost::bind(&PathLookout::weightObstacle, this, _1));
    float max_weight = *max_element(weights.begin(), weights.end());
    ROS_DEBUG("Max Obstacle Weight: %g", max_weight);

    // visualize obstacles in rviz
    if (visualizer_->hasSubscriber()) {
        for(size_t i = 0; i < tracked_obs.size(); ++i) {
            // this should be a unique identifier for a tracked obstacle
            int id = tracked_obs[i].time_of_first_sight.toNSec();

            geometry_msgs::Point gp;
            gp.x = tracked_obs[i].last_position.x;
            gp.y = tracked_obs[i].last_position.y;
            visualizer_->drawMark(id, gp, "obstacleonpath", 1,0,0, "/base_link");

            // show the weight.
            stringstream s;
            s << weights[i];
            gp.z = 0.5;
            visualizer_->drawText(id, gp, s.str(), "obstacleonpath_weight", 1,0,0, "/base_link");
        }
    }

    if (contours.empty()) {
        // no obstacles on the path.
        return false;
    } else {
        return false;// don't stop, for testing
    }
}

void PathLookout::configure()
{
    //TODO: add params to documentation
    ros::param::param<float>("~obstacle_scale_distance", scale_obstacle_distance_, 2.0f);
    ros::param::param<float>("~obstacle_scale_time", scale_obstacle_distance_, 0.1f);
}


void PathLookout::drawPathToImage(const Path &path)
{
    // first reset image
    path_image_ = cv::Scalar(0);

    if (path.size() < 2) {
        ROS_WARN("Path has less than 2 waypoints. No obstacle lookout is done.");
        return;
    }

    Path::const_iterator iter = path.begin();

    try {
        // get first point
        cv::Point2f p1(iter->x, iter->y);
        p1 = map_trans_.transformPointToMap(p1, "/map");
        // iterate over second to last point
        for (++iter; iter != path.end(); ++iter) {
            cv::Point2f p2(iter->x, iter->y);
            p2 = map_trans_.transformPointToMap(p2, "/map");

            cv::line(path_image_, p1, p2, cv::Scalar(255), 2); //TODO: more reasonable value for thickness
            p1 = p2;
        }
    } catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to transform path to map in PathLookout. TF-Exception: %s\n PathLookout will not be able to check for obstacles on the path!", ex.what());
        // Nothing to do here. Maybe parts of the path have been drawed to the map before the exception occured.
        // In this case the lookout will work as intended but only check the first part of the path.
        // In the worst case, nothing was drawn, then the lookout will simply not see obstacles, but it will not crash
        // in any way.
    }
}

float PathLookout::weightObstacle(ObstacleTracker::TrackedObstacle o) const
{
    // This assumes, that the position is given in the robot frame (and thus pos_robot = 0).
    float dist_to_robot = cv::norm(o.last_position);
    ros::Duration lifetime = ros::Time::now() - o.time_of_first_sight;

    return scale_obstacle_distance_ * 1/dist_to_robot + scale_obstacle_duration_ * lifetime.toSec();
}
