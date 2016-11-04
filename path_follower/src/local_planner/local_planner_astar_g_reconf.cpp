/// HEADER
#include <path_follower/local_planner/local_planner_astar_g_reconf.h>

/// PROJECT


LocalPlannerAStarGReconf::LocalPlannerAStarGReconf(RobotController &follower,
                                       PoseTracker &pose_tracker,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerStar(follower, pose_tracker, update_interval),
      LocalPlannerReconf(follower, pose_tracker, update_interval),
      LocalPlannerAStar(follower, pose_tracker, update_interval),
      LocalPlannerStarG(follower, pose_tracker, update_interval),
      LocalPlannerStarReconf(follower, pose_tracker, update_interval)
{

}
