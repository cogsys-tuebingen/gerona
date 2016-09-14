/// HEADER
#include <path_follower/local_planner/local_planner_bfs_static.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerBFSStatic::LocalPlannerBFSStatic(PathFollower &follower,
                                       tf::Transformer& transformer,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerBFS(follower, transformer, update_interval),
      LocalPlannerStatic(follower, transformer, update_interval)
{

}

void LocalPlannerBFSStatic::evaluate(double& current_p, LNode*& succ, double& dis2last,
              const std::vector<Scorer::Ptr>& scorer,
              const std::vector<double>& wscorer){
    current_p = Heuristic(*succ, dis2last) + Score(*succ, scorer, wscorer);
}
