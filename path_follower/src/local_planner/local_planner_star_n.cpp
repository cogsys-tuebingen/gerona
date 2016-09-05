/// HEADER
#include <path_follower/local_planner/local_planner_star_n.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerStarN::LocalPlannerStarN(PathFollower &controller, tf::Transformer &transformer,
                                     const ros::Duration &update_interval)
    :LocalPlannerStar(controller,transformer,update_interval)
{

}

double LocalPlannerStarN::f(double& g, double& score, double& heuristic){
    (void) score;
    return g + heuristic;
}
