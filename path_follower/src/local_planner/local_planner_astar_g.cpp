/// HEADER
#include <path_follower/local_planner/local_planner_astar_g.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerAStarG::LocalPlannerAStarG(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerAStar(follower, transformer, update_interval)
{

}

double LocalPlannerAStarG::f(double& g, double& score, double& heuristic){
    (void) g;
    return score + heuristic;
}
