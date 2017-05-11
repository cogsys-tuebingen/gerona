/// HEADER
#include <path_follower/local_planner/high_speed/local_planner_star_reconf.h>

/// PROJECT


LocalPlannerStarReconf::LocalPlannerStarReconf()
{

}

void LocalPlannerStarReconf::evaluate(double& current_p, double& heuristic, double& score){
    current_p = heuristic + score;
}
