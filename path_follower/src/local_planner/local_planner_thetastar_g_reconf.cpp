/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_g_reconf.h>

/// PROJECT


LocalPlannerThetaStarGReconf::LocalPlannerThetaStarGReconf(RobotController& controller, PoseTracker& pose_tracker,
                                                           const ros::Duration& update_interval)
    : LocalPlannerClassic(controller, pose_tracker, update_interval),
      LocalPlannerStar(controller, pose_tracker, update_interval),
      LocalPlannerReconf(controller, pose_tracker, update_interval),
      LocalPlannerThetaStar(controller, pose_tracker, update_interval),
      LocalPlannerStarG(controller, pose_tracker, update_interval),
      LocalPlannerStarReconf(controller, pose_tracker, update_interval)
{

}
