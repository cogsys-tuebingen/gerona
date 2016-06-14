/// HEADER
#include <path_follower/local_planner/local_planner_transformer.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerTransformer::LocalPlannerTransformer(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval)
{

}

Path::Ptr LocalPlannerTransformer::updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                                   const std::vector<Scorer::Ptr>& scorer,
                                                   const std::vector<bool>& fconstraints,
                                                   const std::vector<double>& wscorer)
{
    // this planner does not "plan" locally, but transforms the global path to the odometry frame
    // to eliminate odometry drift

    ros::Time now = ros::Time::now();

    // only calculate a new local path, if enough time has passed.
    // TODO: also replan for other reasons, e.g. the global path has changed, ...
    if(last_update_ + update_interval_ < now) {

        // only look at the first sub path for now
        auto waypoints = (SubPath) global_path_;

        if(transform2Odo(waypoints,now) == 0){
            return nullptr;
        }

        // find the subpath that starts closest to the robot
        Eigen::Vector3d pose = follower_.getRobotPose();

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
        for(std::size_t i = start, n = std::min(start + 20, waypoints.size()); i < n; ++i) {
            local_wps.push_back(waypoints[i]);
        }

        //        // example use of scorers
        //        // (return the path with the lowest score.)
        //        double score = 0;
        //        for(tf::Point& pt : local_path) {
        //            for(Scorer& scorer : scorers) {
        //                score += scorer.score(pt);
        //            }
        //        }

        //        // example use of constraints to check for goal conditions
        //        bool is_goal = true;
        //        tf::Point point = ....;
        //        for(Constraint& constraint : constraints) {
        //            if(!constraint.isSatisfied(point)) {
        //               is_goal = false;
        //               break;
        //            }
        //        }

        // here we just use the subpath without planning and checking constraints / scorerers
        Path::Ptr local_path(new Path("/odom"));
        setPath(local_path, local_wps, now);

        return local_path;

    } else {
        return nullptr;
    }
}
