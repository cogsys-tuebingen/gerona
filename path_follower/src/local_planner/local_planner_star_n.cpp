/// HEADER
#include <path_follower/local_planner/local_planner_star_n.h>

/// PROJECT


LocalPlannerStarN::LocalPlannerStarN()
{

}

double LocalPlannerStarN::f(double& g, double& score, double& heuristic){
    (void) score;
    return g + heuristic;
}
