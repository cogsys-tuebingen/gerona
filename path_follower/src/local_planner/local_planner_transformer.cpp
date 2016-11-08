/// HEADER
#include <path_follower/local_planner/local_planner_transformer.h>

/// PROJECT

#include <path_follower/utils/pose_tracker.h>

LocalPlannerTransformer::LocalPlannerTransformer()
{

}

void LocalPlannerTransformer::printNodeUsage(std::size_t& nnodes) const{
    (void)nnodes;
}

bool LocalPlannerTransformer::algo(Eigen::Vector3d& pose, SubPath& local_wps,
                                  std::size_t& nnodes)
{
    (void) nnodes;
    // this planner does not "plan" locally, but transforms the global path to the odometry frame
    // to eliminate odometry drift

    ros::Time now = ros::Time::now();

    // only calculate a new local path, if enough time has passed.
    // TODO: also replan for other reasons, e.g. the global path has changed, ...
    if(last_update_ + update_interval_ < now) {
        //ROS_INFO("updating local path");
        // only look at the first sub path for now

        // calculate the corrective transformation to map from world coordinates to odom
        if(!transformer_->waitForTransform("map", "odom", ros::Time(0), ros::Duration(0.1))) {
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return nullptr;
        }

        tf::StampedTransform now_map_to_odom;
        transformer_->lookupTransform("map", "odom", ros::Time(0), now_map_to_odom);

        tf::Transform transform_correction = now_map_to_odom.inverse();

        // transform the waypoints from world to odom
        for(Waypoint& wp : waypoints) {
            tf::Point pt(wp.x, wp.y, 0);
            pt = transform_correction * pt;
            wp.x = pt.x();
            wp.y = pt.y();

            tf::Quaternion rot = tf::createQuaternionFromYaw(wp.orientation);
            rot = transform_correction * rot;
            wp.orientation = tf::getYaw(rot);
        }

        // find the subpath that starts closest to the robot
        Eigen::Vector3d pose = pose_tracker_->getRobotPose();

        double closest_dist = std::numeric_limits<double>::infinity();
        std::size_t start = 0;
        for(std::size_t i = 0; i < waypoints.size(); ++i) {
            const Waypoint& wp = waypoints[i];
            double dist = std::hypot(wp.x - pose(0), wp.y - pose(1));
            if(dist < closest_dist) {
                closest_dist = dist;
                start = i;
            }
        }
        std::vector<Waypoint> local_wps;
        for(std::size_t i = start, n = std::min(start + 100, waypoints.size()); i < n; ++i) {
            local_wps.push_back(waypoints[i]);
        }
    }

    // here we just use the subpath without planning and checking constraints / scorerers
    return true;
}

void LocalPlannerTransformer::setParams(int nnodes, int ic, double dis2p, double adis, double fdis, double s_angle,
                                        int ia, double lmf, int max_level, double mu, double ef){
    (void) nnodes;
    (void) ic;
    (void) dis2p;
    (void) adis;
    (void) fdis;
    (void) s_angle;
    (void) ia;
    (void) lmf;
    (void) max_level;
    (void) mu;
    (void) ef;
}

void LocalPlannerTransformer::setVelocity(geometry_msgs::Twist::_linear_type vector){
    (void) vector;
}

void LocalPlannerTransformer::setVelocity(double velocity){
    (void) velocity;
}

void LocalPlannerTransformer::printVelocity(){}

void LocalPlannerTransformer::printLevelReached() const{}
