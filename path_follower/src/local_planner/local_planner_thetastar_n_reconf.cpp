/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_n_reconf.h>

/// PROJECT


LocalPlannerThetaStarNReconf::LocalPlannerThetaStarNReconf(RobotController &follower,
                                               PoseTracker &pose_tracker,
                                               const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, pose_tracker, update_interval),
      LocalPlannerStar(follower, pose_tracker, update_interval),
      LocalPlannerReconf(follower, pose_tracker, update_interval),
      LocalPlannerThetaStar(follower, pose_tracker, update_interval),
      LocalPlannerStarN(follower, pose_tracker, update_interval),
      LocalPlannerStarReconf(follower, pose_tracker, update_interval)
{

}
