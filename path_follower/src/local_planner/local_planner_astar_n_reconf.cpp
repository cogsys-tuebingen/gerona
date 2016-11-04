/// HEADER
#include <path_follower/local_planner/local_planner_astar_n_reconf.h>

/// PROJECT


LocalPlannerAStarNReconf::LocalPlannerAStarNReconf(RobotController &follower,
                                       PoseTracker &pose_tracker,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerStar(follower, pose_tracker, update_interval),
      LocalPlannerReconf(follower, pose_tracker, update_interval),
      LocalPlannerAStar(follower, pose_tracker, update_interval),
      LocalPlannerStarN(follower, pose_tracker, update_interval),
      LocalPlannerStarReconf(follower, pose_tracker, update_interval)
{

}
