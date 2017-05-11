/// HEADER
#include <path_follower/local_planner/high_speed/local_planner_star_static.h>

/// PROJECT


LocalPlannerStarStatic::LocalPlannerStarStatic()
{

}

void LocalPlannerStarStatic::evaluate(double& current_p, double& heuristic, double& score){
    current_p = heuristic + score;
}
