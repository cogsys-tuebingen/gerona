/// HEADER
#include <path_follower/local_planner/local_planner_star_g.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerStarG::LocalPlannerStarG(PathFollower &controller, tf::Transformer &transformer,
                                     const ros::Duration &update_interval)
    : LocalPlannerClassic(controller,transformer,update_interval),
      LocalPlannerStar(controller,transformer,update_interval)
{

}

double LocalPlannerStarG::f(double& g, double& score, double& heuristic){
    (void) g;
    return score + heuristic;
}
