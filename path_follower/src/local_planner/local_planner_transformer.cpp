/// HEADER
#include <path_follower/local_planner/local_planner_transformer.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerTransformer::LocalPlannerTransformer(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlanner(follower, transformer), last_update_(0), update_interval_(update_interval)
{

}

void LocalPlannerTransformer::setGlobalPath(Path::Ptr path)
{
    LocalPlanner::setGlobalPath(path);
}

Path::Ptr LocalPlannerTransformer::updateLocalPath()
{
    ros::Time now = ros::Time::now();
    if(last_update_ + update_interval_ < now) {

        auto wps = global_path_->getSubPath(0);

        Eigen::Vector3d pose = follower_.getRobotPose();

        if(!transformer_.waitForTransform("map", "odom", now, ros::Duration(0.1))) {
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return nullptr;
        }

        tf::StampedTransform now_map_to_odom;
        transformer_.lookupTransform("map", "odom", now, now_map_to_odom);


        tf::Transform transform_correction = now_map_to_odom.inverse();


        for(Waypoint& wp : wps) {
            tf::Point pt(wp.x, wp.y, 0);
            pt = transform_correction * pt;
            wp.x = pt.x();
            wp.y = pt.y();

            tf::Quaternion rot = tf::createQuaternionFromYaw(wp.orientation);
            rot = transform_correction * rot;
            wp.orientation = tf::getYaw(rot);
        }

        double closest_dist = std::numeric_limits<double>::infinity();
        std::size_t start = 0;
        for(std::size_t i = 0; i < wps.size(); ++i) {
            const Waypoint& wp = wps[i];
            double dist = std::hypot(wp.x - pose(0), wp.y - pose(1));
            if(dist < closest_dist) {
                closest_dist = dist;
                start = i;
            }
        }

        std::vector<Waypoint> local_wps;

        for(std::size_t i = start, n = std::min(start + 20, wps.size()); i < n; ++i) {
            local_wps.push_back(wps[i]);
        }

        Path::Ptr local_path(new Path("/odom"));
        local_path->setPath({local_wps});

        follower_.getController()->reset();
        follower_.getController()->setPath(local_path);
        last_update_ = now;

        return local_path;

    } else {
        return nullptr;
    }
}
