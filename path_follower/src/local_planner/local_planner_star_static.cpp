/// HEADER
#include <path_follower/local_planner/local_planner_star_static.h>

/// PROJECT


LocalPlannerStarStatic::LocalPlannerStarStatic(RobotController &follower,
                                       PoseTracker &pose_tracker,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerStar(follower, pose_tracker, update_interval),
      LocalPlannerStatic(follower, pose_tracker, update_interval)
{

}

void LocalPlannerStarStatic::evaluate(double& current_p, double& heuristic, double& score){
    current_p = heuristic + score;
}
