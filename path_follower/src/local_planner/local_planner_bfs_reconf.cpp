/// HEADER
#include <path_follower/local_planner/local_planner_bfs_reconf.h>

/// PROJECT


LocalPlannerBFSReconf::LocalPlannerBFSReconf(RobotController &follower,
                                       PoseTracker &pose_tracker,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerBFS(follower, pose_tracker, update_interval),
      LocalPlannerReconf(follower, pose_tracker, update_interval)
{

}

void LocalPlannerBFSReconf::evaluate(double& current_p, LNode*& succ, double& dis2last,
              const std::vector<Scorer::Ptr>& scorer,
              const std::vector<double>& wscorer){
    (void) current_p;
    (void) succ;
    (void) dis2last;
    (void) scorer;
    (void) wscorer;
}
