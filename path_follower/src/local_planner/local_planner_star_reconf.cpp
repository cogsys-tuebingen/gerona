/// HEADER
#include <path_follower/local_planner/local_planner_star_reconf.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerStarReconf::LocalPlannerStarReconf(PathFollower &follower,
                                       tf::Transformer& transformer,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerReconf(follower, transformer, update_interval)
{

}

void LocalPlannerStarReconf::evaluate(double& current_p, double& heuristic, double& score){
    (void) current_p;
    (void) heuristic;
    (void) score;
}
