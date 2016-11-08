/// HEADER
#include <path_follower/local_planner/local_planner_bfs_static.h>

/// PROJECT


LocalPlannerBFSStatic::LocalPlannerBFSStatic()
{

}

void LocalPlannerBFSStatic::evaluate(double& current_p, LNode*& succ, double& dis2last)
{
    current_p = Heuristic(*succ, dis2last) + Score(*succ);
}
