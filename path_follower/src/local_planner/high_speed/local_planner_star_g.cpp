/// HEADER
#include <path_follower/local_planner/high_speed/local_planner_star_g.h>

/// PROJECT


LocalPlannerStarG::LocalPlannerStarG()
{

}

double LocalPlannerStarG::f(double& g, double& score, double& heuristic){
    (void) g;
    return score + heuristic;
}
