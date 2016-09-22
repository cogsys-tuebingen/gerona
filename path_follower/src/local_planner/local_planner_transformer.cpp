/// HEADER
#include <path_follower/local_planner/local_planner_transformer.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerTransformer::LocalPlannerTransformer(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerImplemented(follower, transformer, update_interval)
{

}

void LocalPlannerTransformer::printNodeUsage(std::size_t& nnodes) const{
    (void)nnodes;
}

bool LocalPlannerTransformer::algo(Eigen::Vector3d& pose, SubPath& local_wps,
                                  const std::vector<Constraint::Ptr>& constraints,
                                  const std::vector<Scorer::Ptr>& scorer,
                                  const std::vector<bool>& fconstraints,
                                  const std::vector<double>& wscorer,
                                  std::size_t& nnodes){
    (void) constraints;
    (void) scorer;
    (void) fconstraints;
    (void) wscorer;
    (void) nnodes;
    // this planner does not "plan" locally, but transforms the global path to the odometry frame
    // to eliminate odometry drift

    // find the subpath that starts closest to the robot
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
    return true;
}

void LocalPlannerTransformer::setParams(int nnodes, int ic, double dis2p, double dis2o, double s_angle, int ia, double lmf, int max_level){
    (void) nnodes;
    (void) ic;
    (void) dis2p;
    (void) dis2o;
    (void) s_angle;
    (void) ia;
    (void) lmf;
    (void) max_level;
}

void LocalPlannerTransformer::setVelocity(geometry_msgs::Twist::_linear_type vector){
    (void) vector;
}

void LocalPlannerTransformer::setVelocity(double velocity){
    (void) velocity;
}

void LocalPlannerTransformer::printVelocity(){}

void LocalPlannerTransformer::printLevelReached() const{}
