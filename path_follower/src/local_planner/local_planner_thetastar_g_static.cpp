/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_g_static.h>

/// PROJECT


LocalPlannerThetaStarGStatic::LocalPlannerThetaStarGStatic(RobotController &follower,
                                               PoseTracker &pose_tracker,
                                               const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerStar(follower, pose_tracker, update_interval),
      LocalPlannerStatic(follower, pose_tracker, update_interval),
      LocalPlannerThetaStar(follower, pose_tracker, update_interval),
      LocalPlannerStarG(follower, pose_tracker, update_interval),
      LocalPlannerStarStatic(follower, pose_tracker, update_interval)
{

}
