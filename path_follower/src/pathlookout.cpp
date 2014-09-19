#include "pathlookout.h"

#define DEBUG_PATHLOOKOUT 0

#if DEBUG_PATHLOOKOUT
    #include <opencv2/highgui/highgui.hpp>
#endif
#include <vector>
#include "pathfollower.h"
#include "boost/foreach.hpp"

using namespace std;

PathLookout::PathLookout(PathFollower *node):
    node_(node),
    obstacle_frame_("/map")
{
    #if DEBUG_PATHLOOKOUT
        cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);
    #endif

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

void PathLookout::setPath(const PathWithPosition &path)
{
    // if there is no map yet, we do not know the size of the path image -> do nothing
    if (map_ == NULL) {
        return;
    }

    // only use path from the last waypoint on ("do not look behind")
    Path path_ahead;
    if (path.wp_idx == 0) {
        path_ahead = *path.current_path;
    } else {
        Path::const_iterator start = path.current_path->begin();
        start += (path.wp_idx-1);
        path_ahead.assign(start, (Path::const_iterator) path.current_path->end());
    }

    drawPathToImage(path_ahead);
}

bool PathLookout::lookForObstacles(path_msgs::FollowPathFeedback *feedback)
{
    if (map_ == NULL) {
        ROS_WARN_THROTTLE(1, "PathLookout has not received any map yet. No obstacle lookout is done.");
        return false;
    }
    if (path_image_.empty()) {
        ROS_WARN_THROTTLE(1, "PathLookout has not received any path yet. No obstacle lookout is done.");
        return false;
    }

    // Calculate intersection of the obstacle map and the path ==> provides the obstacles on the path
    cv::Mat intersect;
    cv::bitwise_and(map_image_, path_image_, intersect);

    // find obstacle contours on the path
    vector<vector<cv::Point> > contours;
    cv::findContours(intersect, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Get center of each obstacle (for tracking)
    vector<Obstacle> observed_obstacles;
    observed_obstacles.reserve(contours.size());
    for(size_t i = 0; i < contours.size(); ++i) {
        try {
            // calculate center of mass, using moments.
            //cv::Moments mom = cv::moments(contours[i], true);
            //cv::Point2f center(mom.m10/mom.m00, mom.m01/mom.m00);

            Obstacle obstacle;
            cv::minEnclosingCircle(contours[i], obstacle.center, obstacle.radius);

            #if DEBUG_PATHLOOKOUT
                // dont need intersect anymore, so it is ok to draw debug stuff to it.
                cv::circle(intersect, obstacle.center, obstacle.radius, cv::Scalar(255));
            #endif

            // transform center and scale radius from pixel to meters.
            obstacle.center = map_trans_.transformPointFromMap(obstacle.center, obstacle_frame_);
            obstacle.radius *= map_->info.resolution;

            observed_obstacles.push_back(obstacle);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("TF-Error. Could not transform obstacle position. %s", ex.what());
        }
    }

    // debug
    #if DEBUG_PATHLOOKOUT
        cv::imshow("Map", map_image_);
        cv::imshow("Path", path_image_);
        cv::imshow("Intersection", intersect);
        cv::waitKey(5);
    #endif

    // Update tracker
    tracker_.update(observed_obstacles);

    vector<ObstacleTracker::TrackedObstacle> tracked_obs = tracker_.getTrackedObstacles();
    if (tracked_obs.empty()) {
        return false;
    }


    /* The following loop does three things at once:
     *  - calculate weight for each obstacle
     *  - determine max. weight
     *  - convert to obstacle-messages for feedback and visualization
     */

    // Calculaten is done with cv::Points, so robot pose has to be converted.
    Eigen::Vector3d robot_pose_eigen = node_->getRobotPose();
    cv::Point2f robot_pos(robot_pose_eigen[0], robot_pose_eigen[1]);

    vector<path_msgs::Obstacle> obstacle_msgs;
    obstacle_msgs.reserve(tracked_obs.size());

    float max_weight = -1000.0f;
    vector<ObstacleTracker::TrackedObstacle>::const_iterator it;
    for (it = tracked_obs.begin(); it != tracked_obs.end(); ++it) {
        // weight
        float w = weightObstacle(robot_pos, *it);
        max_weight = max(max_weight, w);

        // convert to message
        path_msgs::Obstacle o_msg = it->obstacle().toMsg();
        o_msg.weight = w;
        obstacle_msgs.push_back(o_msg);
    }


    // report obstacles via feedback
    if (feedback != NULL) {
        feedback->obstacles_on_path = obstacle_msgs;
    }


    // visualize obstacles in rviz
    if (visualizer_->hasSubscriber()) {
        for(size_t i = 0; i < tracked_obs.size(); ++i) {
            // this should be a unique identifier for a tracked obstacle
            int id = tracked_obs[i].time_of_first_sight().toNSec();

            geometry_msgs::Point gp = obstacle_msgs[i].position;
            //visualizer_->drawMark(id, gp, "obstacleonpath", 1,0,0, obstacle_frame_);
            visualizer_->drawCircle(id, gp, tracked_obs[i].obstacle().radius, obstacle_frame_, "obstacleonpath", 1,0,0,0.5, 0.1);

            // show the weight.
            stringstream s;
            s << obstacle_msgs[i].weight;
            gp.z = 0.5;
            visualizer_->drawText(id, gp, s.str(), "obstacleonpath_weight", 1,0,0, obstacle_frame_, 0.1);
        }
    }

    // report obstacle, if the highest weight is higher than the defined limit.
    ROS_DEBUG("Max Obstacle Weight: %g, limit: %g", max_weight, opt_.obstacle_weight_limit_);
    return max_weight > opt_.obstacle_weight_limit_;
}

void PathLookout::reset()
{
    // important! path mask has to be reseted, otherwise obstacles will be readded immediately.
    path_image_ = cv::Scalar(0);
    tracker_.reset();
}

void PathLookout::configure()
{
    ros::param::param<float>("~obstacle_scale_distance", opt_.scale_obstacle_distance_, 1.0f);
    ros::param::param<float>("~obstacle_scale_lifetime", opt_.scale_obstacle_lifetime_, 10.0f);
    ros::param::param<float>("~path_width", opt_.path_width_, 0.5f);

    // there should be no need to make max. weight configurable, as it can be scaled using the parameters above.
    opt_.obstacle_weight_limit_ = 1.0f;
}


void PathLookout::drawPathToImage(const Path &path)
{
    /// This method assumes, that the map is already set!

    // initialize path image to fit map size
    if (path_image_.empty()) {
        path_image_ = cv::Mat(map_->info.height, map_->info.width, CV_8UC1, cv::Scalar(0));
    }

    // first reset image
    path_image_ = cv::Scalar(0); //<- yup, this is possible in OpenCV :) Sets every pixel to 0.

    if (path.size() < 2) {
        ROS_WARN("Path has less than 2 waypoints. No obstacle lookout is done.");
        return;
    }

    int path_width_pixel = (int) ceil(opt_.path_width_ / map_->info.resolution);

    Path::const_iterator iter = path.begin();
    try {
        // get first point
        cv::Point2f p1(iter->x, iter->y);
        p1 = map_trans_.transformPointToMap(p1, "/map");
        // iterate over second to last point
        for (++iter; iter != path.end(); ++iter) {
            cv::Point2f p2(iter->x, iter->y);
            p2 = map_trans_.transformPointToMap(p2, "/map");

            cv::line(path_image_, p1, p2, cv::Scalar(255), path_width_pixel);
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
    float dist_to_robot = cv::norm(robot_pos - o.obstacle().center) - o.obstacle().radius;
    ros::Duration lifetime = ros::Time::now() - o.time_of_first_sight();

    // w_time is increasing quadratically with the time. t = scale => w_t = 1
    float w_time = pow(lifetime.toSec()/opt_.scale_obstacle_lifetime_, 2);
    // w_dist is increasing quadratically with decreasing distance. d = scale => w_d = 1.
    // For d > 2*scale, w_d is zero (-> cuts off the other half of the parable).
    float w_dist = dist_to_robot < 2*opt_.scale_obstacle_distance_
                   ? pow((dist_to_robot - 2*opt_.scale_obstacle_distance_ )/opt_.scale_obstacle_distance_, 2)
                   : 0;

    //ROS_WARN("WEIGHT: d = %g, t = %g, wd = %g, wt = %g", dist_to_robot, lifetime.toSec(), w_dist, w_time);

    return w_dist + w_time;
}
