#include "pathlookout.h"

#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;

PathLookout::PathLookout():
    obstacle_frame_("/map")
{
    cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);

    visualizer_ = Visualizer::getInstance();
    configure();
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
}

void PathLookout::setPath(const Path &path)
{
    // if there is no map yet, we do not know the size of the path image -> do nothing
    if (map_ == NULL) {
        return;
    }

    // initialize path image to fit map size
    if (path_image_.empty()) {
        path_image_ = cv::Mat(map_->info.height, map_->info.width, CV_8UC1, cv::Scalar(0));
    }

    //path_ = path;
    drawPathToImage(path);
}

bool PathLookout::lookForObstacles()
{
    //FIXME: do not store obstacles in robot frame...

    //FIXME: This does currently not take the postion of the robot into account (thus also obstacles behind the robot
    // can stop the robot).

    if (map_ == NULL) {
        ROS_WARN("PathLookout has not received any map yet. No obstacle lookout is done.");
        return false;
    }
    if (path_image_.empty()) {
        ROS_WARN("PathLookout has not received any path yet. No obstacle lookout is done.");
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
            center = map_trans_.transformPointFromMap(center, obstacle_frame_);

            obstacle_centers.push_back(center);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("TF-Error. Could not transform obstacle position. %s", ex.what());
        }
    }

    // Update tracker and calculate weights for the tracked obstacles.
    tracker_.update(obstacle_centers);

    vector<ObstacleTracker::TrackedObstacle> tracked_obs = tracker_.getTrackedObstacles();
    if (tracked_obs.empty()) {
        return false;
    }

    // get current robot position
//    try {
//        cv::Point2f robot_pos(0,0);
//        center = map_trans_.transformPointFromMap(center, obstacle_frame_);

//        obstacle_centers.push_back(center);
//    } catch (const tf::TransformException& ex) {
//        ROS_ERROR("TF-Error. Could not transform obstacle position. %s", ex.what());
//    }

    vector<float> weights;
    weights.resize(tracked_obs.size());
    cv::Point2f robot_pos(0,0);
    transform(tracked_obs.begin(), tracked_obs.end(), weights.begin(), boost::bind(&PathLookout::weightObstacle, this, robot_pos, _1));
    float max_weight = *max_element(weights.begin(), weights.end());

    // visualize obstacles in rviz
    if (visualizer_->hasSubscriber()) {
        for(size_t i = 0; i < tracked_obs.size(); ++i) {
            // this should be a unique identifier for a tracked obstacle
            int id = tracked_obs[i].time_of_first_sight().toNSec();

            geometry_msgs::Point gp;
            gp.x = tracked_obs[i].last_position().x;
            gp.y = tracked_obs[i].last_position().y;
            visualizer_->drawMark(id, gp, "obstacleonpath", 1,0,0, obstacle_frame_);

            // show the weight.
            stringstream s;
            s << weights[i];
            gp.z = 0.5;
            visualizer_->drawText(id, gp, s.str(), "obstacleonpath_weight", 1,0,0, obstacle_frame_);
        }
    }

    // report obstacle, if the highest weight is higher than the defined limit.

    ROS_DEBUG("Max Obstacle Weight: %g, limit: %g", max_weight, obstacle_weight_limit_);
    return max_weight > obstacle_weight_limit_;
}

void PathLookout::configure()
{
    //TODO: add params to documentation
    ros::param::param<float>("~obstacle_scale_distance", scale_obstacle_distance_, 2.0f);
    ros::param::param<float>("~obstacle_scale_lifetime", scale_obstacle_lifetime_, 10.0f);
    // there should be no need to make max. weight configurable, as it can be scaled using the parameters above.
    obstacle_weight_limit_ = 1.0f;
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

float PathLookout::weightObstacle(cv::Point2f robot_pos, ObstacleTracker::TrackedObstacle o) const
{
    // This assumes, that the position is given in the robot frame (and thus pos_robot = 0).
//    float dist_to_robot = cv::norm(o.last_position());
//    ros::Duration lifetime = ros::Time::now() - o.time_of_first_sight();
//    return scale_obstacle_distance_ * 1/dist_to_robot + scale_obstacle_duration_ * lifetime.toSec();

    float dist_to_robot = cv::norm(robot_pos - o.last_position());
    ros::Duration lifetime = ros::Time::now() - o.time_of_first_sight();
    float w_dist = 1/exp(dist_to_robot - scale_obstacle_distance_); //TODO: something linear or quadratic would be better
    float w_time = pow(lifetime.toSec()/scale_obstacle_lifetime_, 2);

    ROS_WARN("WEIGHT: d = %g, t = %g, wd = %g, wt = %g", dist_to_robot, lifetime.toSec(), w_dist, w_time);

    return w_dist + w_time;
}
