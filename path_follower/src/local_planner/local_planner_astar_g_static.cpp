/// HEADER
#include <path_follower/local_planner/local_planner_astar_g_static.h>

/// PROJECT


LocalPlannerAStarGStatic::LocalPlannerAStarGStatic(RobotController& controller, PoseTracker& pose_tracker,
                                                   const ros::Duration& update_interval)
    : LocalPlannerClassic(controller, pose_tracker, update_interval),
      LocalPlannerStar(controller, pose_tracker, update_interval),
      LocalPlannerStatic(controller, pose_tracker, update_interval),
      LocalPlannerAStar(controller, pose_tracker, update_interval),
      LocalPlannerStarG(controller, pose_tracker, update_interval),
      LocalPlannerStarStatic(controller, pose_tracker, update_interval)
{

}
