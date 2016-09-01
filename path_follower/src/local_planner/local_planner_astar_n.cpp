/// HEADER
#include <path_follower/local_planner/local_planner_astar_n.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerAStarN::LocalPlannerAStarN(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerAStar(follower, transformer, update_interval)
{

}

double LocalPlannerAStarN::f(double& g, double& score, double& heuristic){
    (void) score;
    return g + heuristic;
}
