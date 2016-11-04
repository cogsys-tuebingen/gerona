/// HEADER
#include <path_follower/local_planner/local_planner_star_reconf.h>

/// PROJECT


LocalPlannerStarReconf::LocalPlannerStarReconf(RobotController &follower,
                                       PoseTracker &pose_tracker,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerStar(follower, pose_tracker, update_interval),
      LocalPlannerReconf(follower, pose_tracker, update_interval)
{

}

void LocalPlannerStarReconf::evaluate(double& current_p, double& heuristic, double& score){
    (void) current_p;
    (void) heuristic;
    (void) score;
}
