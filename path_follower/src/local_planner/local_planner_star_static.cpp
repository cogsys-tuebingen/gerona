/// HEADER
#include <path_follower/local_planner/local_planner_star_static.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerStarStatic::LocalPlannerStarStatic(PathFollower &follower,
                                       tf::Transformer& transformer,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerStatic(follower, transformer, update_interval)
{

}

void LocalPlannerStarStatic::evaluate(double& current_p, double& heuristic, double& score){
    current_p = heuristic + score;
}
