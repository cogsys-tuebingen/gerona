/// HEADER
#include <path_follower/local_planner/local_planner_bfs_static.h>

/// PROJECT


LocalPlannerBFSStatic::LocalPlannerBFSStatic()
{

}

void LocalPlannerBFSStatic::evaluate(double& current_p, LNode*& succ, double& dis2last,
              const std::vector<Scorer::Ptr>& scorer,
              const std::vector<double>& wscorer){
    current_p = Heuristic(*succ, dis2last) + Score(*succ, scorer, wscorer);
}
