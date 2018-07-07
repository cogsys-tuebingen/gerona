/// HEADER
#include <path_follower/local_planner/local_planner_transformer.h>

/// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/factory/local_planner_factory.h>

REGISTER_LOCAL_PLANNER(LocalPlannerTransformer, Transformer);


LocalPlannerTransformer::LocalPlannerTransformer()
{

}

Path::Ptr LocalPlannerTransformer::updateLocalPath()
{
    // this planner does not "plan" locally, but transforms the global path to the odometry frame
    // to eliminate odometry drift

    ros::Time now = ros::Time::now();

    std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();

    // only calculate a new local path, if enough time has passed.
    // TODO: also replan for other reasons, e.g. the global path has changed, ...
    if(last_update_ + update_interval_ < now) {
        // only look at the first sub path for now
        // calculate the corrective transformation to map from world coordinates to odom
        if(!transformer_->waitForTransform(world_frame, odom_frame, ros::Time(0), ros::Duration(0.1))) {
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return {};
        }

        tf::StampedTransform now_map_to_odom;
        transformer_->lookupTransform(world_frame, odom_frame, ros::Time(0), now_map_to_odom);

        tf::Transform transform_correction = now_map_to_odom.inverse();

        // transform the waypoints from world to odom
        SubPath transformed_waypoints;
        for(std::size_t i = 0, n = global_path_.n(); i < n; ++i) {
            tf::Point pt(global_path_.p(i), global_path_.q(i), 0);
            pt = transform_correction * pt;

            tf::Quaternion rot = tf::createQuaternionFromYaw(global_path_.theta_p(i));
            rot = transform_correction * rot;
            transformed_waypoints.emplace_back(pt.x(), pt.y(), tf::getYaw(rot));
        }

        // find the subpath that starts closest to the robot
        Eigen::Vector3d pose = pose_tracker_->getRobotPose();

        double closest_dist = std::numeric_limits<double>::infinity();
        std::size_t start = 0;
        for(std::size_t i = 0; i < transformed_waypoints.size(); ++i) {
            const Waypoint& wp = transformed_waypoints[i];
            double dist = std::hypot(wp.x - pose(0), wp.y - pose(1));
            if(dist < closest_dist) {
                closest_dist = dist;
                start = i;
            }
        }
        SubPath local_wps;
        for(std::size_t i = start, n = std::min(start + 100, transformed_waypoints.size()); i < n; ++i) {
            local_wps.push_back(transformed_waypoints[i]);
        }

        // here we just use the subpath without planning and checking constraints / scorerers
        return setPath(odom_frame, local_wps, now);

    } else {
        return nullptr;
    }
}

void LocalPlannerTransformer::setParams(const LocalPlannerParameters& opt){
}

void LocalPlannerTransformer::setVelocity(geometry_msgs::Twist::_linear_type vector){
    (void) vector;
}

void LocalPlannerTransformer::setVelocity(double velocity){
    (void) velocity;
}
