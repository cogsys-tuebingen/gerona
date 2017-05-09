/// HEADER
#include <path_follower/local_planner/local_planner_bfs_reconf.h>

/// PROJECT
#include <path_follower/factory/local_planner_factory.h>

REGISTER_LOCAL_PLANNER(LocalPlannerBFSReconf, BFSR);


LocalPlannerBFSReconf::LocalPlannerBFSReconf()
{

}

void LocalPlannerBFSReconf::evaluate(double& current_p, LNode*& succ, double& dis2last){
    (void) current_p;
    (void) succ;
    (void) dis2last;
}
